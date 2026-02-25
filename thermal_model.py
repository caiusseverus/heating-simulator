"""
Thermal simulation models for the Heating Simulator.

Three models are available:

1. SimpleThermalModel  (R1C1 + first-order lag)
   -----------------------------------------------
   Original single-node model.  Fast, easy to parameterise.
   C·dT/dt = Q_heater − K·(T − T_ext)

2. R2C2ThermalModel  (two-node room + solar gain)
   -------------------------------------------------
   Splits the room into air node (C_air) and fabric node (C_fabric).
   Based on ISO 13790 simplified hourly method.

   C_air·dT_air/dt    = Q_heater + Q_solar + (T_fab − T_air)/R_fab
                        − (T_air − T_ext)/R_inf
   C_fab·dT_fab/dt    = (T_air − T_fab)/R_fab − (T_fab − T_ext)/R_ext

   Where:
     R_fab  = thermal resistance between air and fabric (°C/W)
     R_ext  = thermal resistance between fabric and outside (°C/W)
     R_inf  = infiltration/ventilation path air→outside (°C/W)
     Q_solar = irradiance × window_area × transmittance

3. WetRadiatorModel
   ------------------
   Explicit radiator water temperature T_rad. Valve position (0–1)
   controls mass flow rate, which determines heat input to radiator.
   Heat output to room follows BS EN 442 power law.

   C_rad·dT_rad/dt = Q_in − Q_out
     Q_in  = valve × m_dot_max × C_water × (T_flow − T_rad)   [clamp T_rad ≤ T_flow]
     Q_out = K_rad × |T_rad − T_room|^n × sign(T_rad − T_room)

   C_room·dT_room/dt = Q_out − K_loss·(T_room − T_ext)

   Pipe delay is modelled as a first-in-first-out queue of flow fractions
   so the radiator only sees flow that was commanded N seconds ago.

All models expose a common interface:
  .set_power_fraction(0–1)
  .set_external_temperature(°C)
  .step(dt_seconds)
  .temperature          → primary (air) temperature
  .heating_rate         → °C/s from heat source
  .heat_loss_rate       → °C/s to outside
  .net_heat_rate        → °C/s net
  .effective_heater_power → W delivered to air node
  .extra_state          → dict of model-specific diagnostics
"""

from __future__ import annotations
import math
from collections import deque


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


C_WATER = 4182.0  # J/(kg·°C)


# ---------------------------------------------------------------------------
# Model 1 — Simple R1C1
# ---------------------------------------------------------------------------

class SimpleThermalModel:
    """
    Single-node lumped capacitance model with first-order heater lag.

    Parameters
    ----------
    heater_power_watts : float
        Nominal heater power at 100% input (W).
    heat_loss_coeff : float
        K-value: heat loss per °C difference to outside (W/°C).
    thermal_mass : float
        Room thermal mass C (J/°C).
    thermal_inertia_tau : float
        First-order lag τ for heater output (s). 0 = instant.
    initial_temp : float
        Starting room temperature (°C).
    initial_external_temp : float
        Starting external temperature (°C).
    """

    def __init__(
        self,
        heater_power_watts: float = 2000.0,
        heat_loss_coeff: float = 50.0,
        thermal_mass: float = 10_000.0,
        thermal_inertia_tau: float = 0.0,
        initial_temp: float = 18.0,
        initial_external_temp: float = 5.0,
    ):
        self.heater_power_watts = heater_power_watts
        self.heat_loss_coeff = heat_loss_coeff
        self.thermal_mass = thermal_mass
        self.thermal_inertia_tau = thermal_inertia_tau

        self.temperature: float = initial_temp
        self.external_temperature: float = initial_external_temp
        self.effective_heater_power: float = 0.0
        self.power_setpoint: float = 0.0

        self.heating_rate: float = 0.0
        self.heat_loss_rate: float = 0.0
        self.net_heat_rate: float = 0.0

    def set_power_fraction(self, fraction: float) -> None:
        self.power_setpoint = _clamp(fraction, 0.0, 1.0)

    def set_external_temperature(self, temp: float) -> None:
        self.external_temperature = temp

    def step(self, dt: float) -> None:
        if dt <= 0:
            return
        # The heater lag now uses the exact exponential solution so no sub-stepping
        # is needed for stability. The room temperature ODE is also stable at typical
        # update intervals (τ_room = C/K is on the order of hours).
        self._euler(dt)
        self.heating_rate = self.effective_heater_power / self.thermal_mass
        self.heat_loss_rate = (
            self.heat_loss_coeff * (self.temperature - self.external_temperature) / self.thermal_mass
        )
        self.net_heat_rate = self.heating_rate - self.heat_loss_rate

    def _euler(self, dt: float) -> None:
        target = self.power_setpoint * self.heater_power_watts
        if self.thermal_inertia_tau > 0:
            # Exact discrete-time solution of first-order lag — unconditionally stable
            # for any dt, including dt >> τ (no clamping needed):
            #   P_eff(t) = P_target + (P_eff(t−dt) − P_target) · exp(−dt/τ)
            decay = math.exp(-dt / self.thermal_inertia_tau)
            self.effective_heater_power = (
                target + (self.effective_heater_power - target) * decay
            )
        else:
            self.effective_heater_power = target
        q_loss = self.heat_loss_coeff * (self.temperature - self.external_temperature)
        self.temperature += (self.effective_heater_power - q_loss) / self.thermal_mass * dt

    @property
    def steady_state_temperature(self) -> float:
        q = self.power_setpoint * self.heater_power_watts
        return self.external_temperature + q / self.heat_loss_coeff

    @property
    def time_to_equilibrium_tau(self) -> float:
        return self.thermal_mass / self.heat_loss_coeff

    @property
    def extra_state(self) -> dict:
        return {
            "steady_state_temp": round(self.steady_state_temperature, 2),
            "room_tau_s": round(self.time_to_equilibrium_tau, 1),
            "heater_inertia_tau_s": self.thermal_inertia_tau,
        }


# ---------------------------------------------------------------------------
# Model 2 — R2C2 with solar gain
# ---------------------------------------------------------------------------

class R2C2ThermalModel:
    """
    Two-node room model (air + fabric) with solar gain.

    Nodes
    -----
    T_air  : room air temperature (the "measured" temperature)
    T_fab  : building fabric temperature (walls, floor, ceiling)

    Heat flows
    ----------
    Q_heater  → T_air node directly
    Q_solar   → T_air node  (irradiance × window_area × g-value)
    T_air ↔ T_fab  via R_fab  (internal convection/radiation)
    T_fab → T_ext  via R_ext  (conduction through fabric)
    T_air → T_ext  via R_inf  (infiltration / ventilation)

    ODEs
    ----
    C_air · dT_air/dt = Q_heater + Q_solar
                        + (T_fab − T_air) / R_fab
                        − (T_air − T_ext) / R_inf

    C_fab · dT_fab/dt = (T_air − T_fab) / R_fab
                        − (T_fab − T_ext) / R_ext

    Parameters — resistances in °C/W
    ---------------------------------
    R_fab ≈ 0.005   good internal coupling (small room, lots of surfaces)
    R_ext ≈ 0.02    moderate insulation (~50 W/°C loss via fabric)
    R_inf ≈ 0.067   ~0.5 ACH infiltration in 100 m³ room
    """

    def __init__(
        self,
        heater_power_watts: float = 2000.0,
        c_air: float = 5_000.0,
        c_fabric: float = 80_000.0,
        r_fabric: float = 0.005,
        r_ext: float = 0.020,
        r_inf: float = 0.067,
        window_area: float = 2.0,
        window_transmittance: float = 0.6,
        initial_temp: float = 18.0,
        initial_external_temp: float = 5.0,
        initial_solar: float = 0.0,
    ):
        self.heater_power_watts = heater_power_watts
        self.c_air = c_air
        self.c_fabric = c_fabric
        self.r_fabric = r_fabric
        self.r_ext = r_ext
        self.r_inf = r_inf
        self.window_area = window_area
        self.window_transmittance = window_transmittance

        # State
        self.t_air: float = initial_temp
        self.t_fabric: float = initial_temp      # fabric starts at same temp as air
        self.external_temperature: float = initial_external_temp
        self.solar_irradiance: float = initial_solar   # W/m²
        self.power_setpoint: float = 0.0
        self.effective_heater_power: float = 0.0

        # Diagnostics
        self.heating_rate: float = 0.0
        self.heat_loss_rate: float = 0.0
        self.net_heat_rate: float = 0.0
        self.solar_gain_watts: float = 0.0
        self.fabric_heat_flux: float = 0.0   # W: positive = fabric heating air

    def set_power_fraction(self, fraction: float) -> None:
        self.power_setpoint = _clamp(fraction, 0.0, 1.0)

    def set_external_temperature(self, temp: float) -> None:
        self.external_temperature = temp

    def set_solar_irradiance(self, irradiance: float) -> None:
        """Set solar irradiance in W/m²."""
        self.solar_irradiance = max(0.0, irradiance)

    @property
    def temperature(self) -> float:
        """Primary output — air temperature."""
        return self.t_air

    def step(self, dt: float) -> None:
        if dt <= 0:
            return
        # Sub-step to avoid numerical instability with small R values
        # Stability criterion: sub_dt < min(C_air*R_fab, C_fab*R_ext) * 2
        min_tau = min(self.c_air * self.r_fabric, self.c_fabric * self.r_ext)
        max_sub = max(1.0, min_tau / 5.0)
        remaining = dt
        while remaining > 0:
            sub = min(remaining, max_sub)
            self._euler(sub)
            remaining -= sub
        self._update_diagnostics()

    # Fraction of solar gain absorbed directly by the fabric (floors, walls).
    # Shortwave radiation passes through the air and is absorbed primarily by
    # opaque surfaces; a small fraction heats the air directly via dust/humidity.
    SOLAR_FABRIC_FRACTION = 0.9

    def _euler(self, dt: float) -> None:
        # Heater — instant (no lag in R2C2; use radiator model for lag)
        self.effective_heater_power = self.power_setpoint * self.heater_power_watts

        # Solar gain — split between fabric (90%) and air (10%)
        q_solar_total = self.solar_irradiance * self.window_area * self.window_transmittance
        q_solar_fabric = q_solar_total * self.SOLAR_FABRIC_FRACTION
        q_solar_air = q_solar_total * (1.0 - self.SOLAR_FABRIC_FRACTION)

        # Fabric ↔ air exchange
        q_fab_to_air = (self.t_fabric - self.t_air) / self.r_fabric

        # Air → outside via infiltration
        q_inf = (self.t_air - self.external_temperature) / self.r_inf

        # Fabric → outside via conduction
        q_fab_to_ext = (self.t_fabric - self.external_temperature) / self.r_ext

        dT_air = (
            self.effective_heater_power + q_solar_air + q_fab_to_air - q_inf
        ) / self.c_air

        dT_fab = (
            q_solar_fabric - q_fab_to_air - q_fab_to_ext
        ) / self.c_fabric

        self.t_air += dT_air * dt
        self.t_fabric += dT_fab * dt
        self.solar_gain_watts = q_solar_total
        self.fabric_heat_flux = q_fab_to_air

    def _update_diagnostics(self) -> None:
        self.heating_rate = self.effective_heater_power / self.c_air
        q_inf_loss = (self.t_air - self.external_temperature) / self.r_inf
        q_fab_loss = (self.t_fabric - self.external_temperature) / self.r_ext
        total_loss_to_air = (q_inf_loss * self.c_air + q_fab_loss * self.c_air) / (self.c_air + self.c_fabric)
        self.heat_loss_rate = (q_inf_loss + max(0, -self.fabric_heat_flux)) / self.c_air
        self.net_heat_rate = (
            (self.effective_heater_power + self.solar_gain_watts)
            - (q_inf_loss + max(0, (self.t_fabric - self.external_temperature) / self.r_ext))
        ) / self.c_air

    @property
    def total_heat_loss_watts(self) -> float:
        """Total instantaneous heat loss through all paths (W)."""
        q_inf = (self.t_air - self.external_temperature) / self.r_inf
        q_fab = (self.t_fabric - self.external_temperature) / self.r_ext
        return q_inf + q_fab

    @property
    def effective_u_value(self) -> float:
        """Effective overall heat loss coefficient (W/°C) at current state."""
        delta = self.t_air - self.external_temperature
        if abs(delta) < 0.01:
            return 0.0
        return self.total_heat_loss_watts / delta

    @property
    def extra_state(self) -> dict:
        return {
            "fabric_temperature": round(self.t_fabric, 2),
            "solar_gain_w": round(self.solar_gain_watts, 1),
            "fabric_to_air_flux_w": round(self.fabric_heat_flux, 1),
            "total_heat_loss_w": round(self.total_heat_loss_watts, 1),
            "effective_u_value_W_per_C": round(self.effective_u_value, 2),
            "infiltration_loss_w": round((self.t_air - self.external_temperature) / self.r_inf, 1),
            "fabric_loss_w": round((self.t_fabric - self.external_temperature) / self.r_ext, 1),
        }


# ---------------------------------------------------------------------------
# Model 3 — Wet Radiator
# ---------------------------------------------------------------------------

class WetRadiatorModel:
    """
    Explicit wet radiator model with valve position control.

    State variables
    ---------------
    T_rad  : radiator mean water/metal temperature (°C)
    T_room : room air temperature (°C)

    Heat flows
    ----------
    Q_in   = valve_fraction × m_dot_max × C_water × (T_flow − T_rad)
             — heat delivered by hot water to radiator body
             — naturally zero when T_rad ≥ T_flow (cannot overheat)

    Q_out  = K_rad × |ΔT_rad_room|^n × sign(ΔT)
             — heat emitted by radiator to room (BS EN 442 power law)
             — n ≈ 1.3 for panel radiators

    Q_loss = K_loss × (T_room − T_ext)
             — room heat loss to outside

    ODEs
    ----
    C_rad  · dT_rad/dt  = Q_in − Q_out
    C_room · dT_room/dt = Q_out − Q_loss

    Asymmetry
    ---------
    Heating: Q_in is large (driven by boiler ΔT ~50°C) → fast warm-up
    Cooling: Q_in = 0, Q_out decays as T_rad → T_room → slow cool-down
    This naturally produces the asymmetric response real radiators show.

    Pipe delay
    ----------
    Optional FIFO queue buffers valve commands so the radiator only
    sees flow that was commanded `pipe_delay` seconds ago.

    Parameters
    ----------
    flow_temperature : float
        Boiler/heat pump flow temperature (°C). Default 70°C (traditional boiler).
    c_radiator : float
        Thermal mass of radiator (water + metal) in J/°C.
        Small single panel: ~3000, large double panel: ~15000.
    k_radiator : float
        BS EN 442 emission coefficient (W/°C^n).
        Sized so that at ΔT=50°C (70°C flow, 20°C room) output ≈ nominal power:
        k_rad = P_nominal / 50^n
    radiator_exponent : float
        BS EN 442 exponent n. Typically 1.3 for panel rads.
    flow_rate_max : float
        Maximum water mass flow rate (kg/s). Typical: 0.03–0.1 kg/s.
    heat_loss_coeff : float
        Room K-value (W/°C).
    c_room : float
        Room thermal mass (J/°C).
    pipe_delay : float
        Dead time from valve change to hot water arriving (s).
    """

    def __init__(
        self,
        flow_temperature: float = 70.0,
        c_radiator: float = 8_000.0,
        k_radiator: float = 10.0,
        radiator_exponent: float = 1.3,
        flow_rate_max: float = 0.05,
        heat_loss_coeff: float = 50.0,
        c_room: float = 15_000.0,
        pipe_delay: float = 0.0,
        initial_temp: float = 18.0,
        initial_external_temp: float = 5.0,
    ):
        self.flow_temperature = flow_temperature
        self.c_radiator = c_radiator
        self.k_radiator = k_radiator
        self.radiator_exponent = radiator_exponent
        self.flow_rate_max = flow_rate_max
        self.heat_loss_coeff = heat_loss_coeff
        self.c_room = c_room
        self.pipe_delay = pipe_delay

        # State
        self.t_rad: float = initial_temp    # radiator starts cold (= room temp)
        self.t_room: float = initial_temp
        self.external_temperature: float = initial_external_temp
        self.valve_fraction: float = 0.0   # 0–1 (= power_setpoint)

        # Pipe delay queue.
        # Each slot represents exactly 1 second of simulated time.
        # _pipe_accum tracks fractional seconds elapsed since the last push,
        # so the queue advances in real simulated-time seconds regardless of
        # how large dt is on each step() call.
        self._pipe_queue: deque[float] = deque()
        self._pipe_queue_size: int = max(1, int(round(pipe_delay)))  # slots = seconds
        self._pipe_accum: float = 0.0   # seconds accumulated since last push
        # Pre-fill with zeros (pipe starts cold / valve closed)
        for _ in range(self._pipe_queue_size):
            self._pipe_queue.append(0.0)

        # Diagnostics
        self.effective_heater_power: float = 0.0   # Q_out (W to room)
        self.q_in_watts: float = 0.0               # Q_in (W to radiator)
        self.q_out_watts: float = 0.0              # Q_out (W radiator→room)
        self.heating_rate: float = 0.0             # °C/s room
        self.heat_loss_rate: float = 0.0           # °C/s room
        self.net_heat_rate: float = 0.0            # °C/s room

    @property
    def temperature(self) -> float:
        """Primary output — room temperature."""
        return self.t_room

    @property
    def power_setpoint(self) -> float:
        return self.valve_fraction

    def set_power_fraction(self, fraction: float) -> None:
        """Set valve position 0–1."""
        self.valve_fraction = _clamp(fraction, 0.0, 1.0)

    def set_external_temperature(self, temp: float) -> None:
        self.external_temperature = temp

    def set_flow_temperature(self, temp: float) -> None:
        self.flow_temperature = temp

    def step(self, dt: float) -> None:
        if dt <= 0:
            return
        # Sub-step: stability requires sub_dt << C_rad / (m_dot * C_water)
        # Minimum charging tau = C_rad / (m_dot_max * C_water)
        min_tau = self.c_radiator / max(0.001, self.flow_rate_max * C_WATER)
        max_sub = max(0.5, min_tau / 10.0)
        remaining = dt
        while remaining > 0:
            sub = min(remaining, max_sub)
            self._euler(sub)
            remaining -= sub
        self._update_diagnostics(dt)

    def _delayed_valve(self, dt: float) -> float:
        """Return the valve fraction that the radiator actually sees after pipe delay."""
        if self.pipe_delay <= 0:
            return self.valve_fraction
        # Push current command into back of queue
        # (approximate: push once per step, not per sub-step for simplicity)
        # The queue is filled during _euler at sub-step level would be too expensive;
        # instead we update at the step level here and use a simple approximation.
        return self._pipe_queue[0]

    def _push_pipe_queue(self, dt: float) -> None:
        """Advance the pipe delay queue by dt seconds of simulated time.

        Each slot in the queue represents 1 second.  We accumulate fractional
        seconds in _pipe_accum and push a new slot for every whole second that
        elapses, so the actual delay is pipe_delay seconds of simulation time
        regardless of the update interval.
        """
        if self.pipe_delay <= 0:
            return
        self._pipe_accum += dt
        while self._pipe_accum >= 1.0:
            self._pipe_accum -= 1.0
            self._pipe_queue.append(self.valve_fraction)
            if len(self._pipe_queue) > self._pipe_queue_size:
                self._pipe_queue.popleft()

    def _euler(self, dt: float) -> None:
        # Effective valve seen at radiator (with pipe delay)
        if self.pipe_delay > 0:
            effective_valve = self._pipe_queue[0] if self._pipe_queue else 0.0
        else:
            effective_valve = self.valve_fraction

        # --- Heat input to radiator from boiler ---
        # Water carries (T_flow - T_rad) * m_dot * C_water watts into radiator
        # If T_rad >= T_flow no heat flows in (physically cannot overheat)
        delta_flow = max(0.0, self.flow_temperature - self.t_rad)
        q_in = effective_valve * self.flow_rate_max * C_WATER * delta_flow

        # --- Heat output from radiator to room ---
        # BS EN 442 power law: Q = K * |ΔT|^n
        delta_rad_room = self.t_rad - self.t_room
        if delta_rad_room > 0:
            q_out = self.k_radiator * (delta_rad_room ** self.radiator_exponent)
        else:
            # Radiator is cooler than room — it acts as a heat sink (rare but possible)
            q_out = -self.k_radiator * ((-delta_rad_room) ** self.radiator_exponent)

        # --- Room heat loss ---
        q_loss = self.heat_loss_coeff * (self.t_room - self.external_temperature)

        # --- Integrate ---
        dT_rad = (q_in - q_out) / self.c_radiator
        dT_room = (q_out - q_loss) / self.c_room

        self.t_rad += dT_rad * dt
        self.t_room += dT_room * dt

        self.q_in_watts = q_in
        self.q_out_watts = q_out
        self.effective_heater_power = q_out  # what matters for the room

    def _update_diagnostics(self, dt: float) -> None:
        self._push_pipe_queue(dt)
        self.heating_rate = self.q_out_watts / self.c_room
        self.heat_loss_rate = (
            self.heat_loss_coeff * (self.t_room - self.external_temperature) / self.c_room
        )
        self.net_heat_rate = self.heating_rate - self.heat_loss_rate

    @property
    def nominal_output_at_dt50(self) -> float:
        """Nominal radiator output at standard ΔT=50°C (W). Useful for sizing check."""
        return self.k_radiator * (50.0 ** self.radiator_exponent)

    @property
    def current_output_fraction(self) -> float:
        """Current heat output as fraction of nominal (at ΔT50)."""
        nom = self.nominal_output_at_dt50
        return self.q_out_watts / nom if nom > 0 else 0.0

    @property
    def return_temperature(self) -> float:
        """
        Estimated radiator return temperature using the Arithmetic Mean Temperature
        Difference (AMTD) approximation:
            T_return = 2·T_rad − T_flow

        This avoids the singularity in the energy-balance method (Q / (ṁ·Cp)) when
        mass flow approaches zero at low valve positions.  The result is clamped to
        at least T_room — the return can never be cooler than the room.
        """
        t_return = 2.0 * self.t_rad - self.flow_temperature
        return max(t_return, self.t_room)

    @property
    def extra_state(self) -> dict:
        return {
            "radiator_temperature": round(self.t_rad, 2),
            "q_in_watts": round(self.q_in_watts, 1),
            "q_out_watts": round(self.q_out_watts, 1),
            "return_temperature": round(self.return_temperature, 2),
            "nominal_output_w_at_dt50": round(self.nominal_output_at_dt50, 1),
            "output_fraction": round(self.current_output_fraction, 3),
            "valve_position_pct": round(self.valve_fraction * 100, 1),
            "pipe_delay_s": self.pipe_delay,
        }


# ---------------------------------------------------------------------------
# Model 4 — R2C2 Room + Wet Radiator (full combined model)
# ---------------------------------------------------------------------------

class R2C2RadiatorModel:
    """
    Full combined model: wet radiator heating an R2C2 two-node room with solar gain.

    This is the most physically complete model. It has three state variables:

      T_rad  : radiator mean water/metal temperature (°C)
      T_air  : room air temperature (°C)           ← the "measured" temperature
      T_fab  : building fabric temperature (°C)

    Heat flows
    ----------
    Boiler → radiator:
      Q_in  = valve × ṁ_max × C_water × max(0, T_flow − T_rad)

    Radiator output split (BS EN 442 / ISO 11855):
      A panel radiator's convective fraction varies by type:
        Type 10 (single panel, no fins):  ~50% convective, ~50% radiative
        Type 11 (single panel + fins):    ~65% convective
        Type 21 (double panel + fins):    ~75% convective
        Type 22 (double panel, 2× fins):  ~80% convective  ← most common UK domestic
        Fan coil / convector:             ~90–95% convective
        Underfloor heating (water):       ~50% convective
      The convective portion heats the air node directly; the radiative portion
      is absorbed by fabric surfaces (walls, floor, ceiling).
      Controlled by radiator_convective_fraction (default 0.75 for type 21/22).

      Q_conv = Q_out × conv_frac      → air node
      Q_rad  = Q_out × (1−conv_frac)  → fabric node

      Total Q_out = K_rad × |T_rad − T_air|^n  (BS EN 442 power law)

    IMPORTANT — c_air definition:
      c_air must represent the air PLUS all room contents that heat up on the
      same timescale as the air: furniture, carpet, curtains, books, clothing.
      This is typically 300,000–600,000 J/°C for a furnished room, NOT the
      ~60,000 J/°C of the air alone. Using only the air mass will produce
      unrealistically fast temperature responses.

    Solar gain (shortwave) — split 90/10 fabric/air:
      Q_solar_total = irradiance × window_area × g-value
      Q_solar_fab   = 0.9 × Q_solar_total   (absorbed by floors/walls)
      Q_solar_air   = 0.1 × Q_solar_total   (minor direct air heating)

    Room heat loss:
      Air → outside via infiltration:   (T_air − T_ext) / R_inf
      Fabric → outside via conduction:  (T_fab − T_ext) / R_ext
      Air ↔ fabric exchange:            (T_air − T_fab) / R_fab

    ODEs
    ----
    C_rad · dT_rad/dt = Q_in − Q_out

    C_air · dT_air/dt = Q_conv             (convective portion of radiator output)
                        + Q_solar_air
                        + (T_fab − T_air) / R_fab
                        − (T_air − T_ext) / R_inf

    C_fab · dT_fab/dt = Q_rad              (radiative portion of radiator output)
                        + Q_solar_fab
                        + (T_air − T_fab) / R_fab   ← note: sign reversal vs dT_air
                        − (T_fab − T_ext) / R_ext

    Parameters
    ----------
    All radiator parameters are identical to WetRadiatorModel.
    All room parameters are identical to R2C2ThermalModel, except
    heater_power_watts is absent (the radiator IS the heater).
    """

    SOLAR_FABRIC_FRACTION = 0.9

    def __init__(
        self,
        # Radiator
        flow_temperature: float = 70.0,
        c_radiator: float = 8_000.0,
        k_radiator: float = 10.0,
        radiator_exponent: float = 1.3,
        radiator_convective_fraction: float = 0.75,
        flow_rate_max: float = 0.05,
        pipe_delay: float = 0.0,
        # Room — NOTE: c_air must include ALL room contents (furniture etc), not just air
        c_air: float = 350_000.0,
        c_fabric: float = 5_000_000.0,
        r_fabric: float = 0.005,
        r_ext: float = 0.020,
        r_inf: float = 0.067,
        # Solar
        window_area: float = 2.0,
        window_transmittance: float = 0.6,
        # Initial conditions
        initial_temp: float = 18.0,
        initial_external_temp: float = 5.0,
        initial_solar: float = 0.0,
    ):
        # Radiator parameters
        self.flow_temperature = flow_temperature
        self.c_radiator = c_radiator
        self.k_radiator = k_radiator
        self.radiator_exponent = radiator_exponent
        self.radiator_convective_fraction = radiator_convective_fraction
        self.flow_rate_max = flow_rate_max
        self.pipe_delay = pipe_delay

        # Room parameters
        self.c_air = c_air
        self.c_fabric = c_fabric
        self.r_fabric = r_fabric
        self.r_ext = r_ext
        self.r_inf = r_inf

        # Solar parameters
        self.window_area = window_area
        self.window_transmittance = window_transmittance

        # State variables
        self.t_rad: float = initial_temp      # radiator starts cold (= room)
        self.t_air: float = initial_temp
        self.t_fabric: float = initial_temp
        self.external_temperature: float = initial_external_temp
        self.solar_irradiance: float = initial_solar
        self.valve_fraction: float = 0.0      # 0–1

        # Pipe delay FIFO queue — each slot = 1 second of simulated time.
        # _pipe_accum accumulates fractional seconds so the queue advances
        # in real simulation-time seconds regardless of step size.
        self._pipe_queue: deque[float] = deque()
        self._pipe_queue_size: int = max(1, int(round(pipe_delay)))
        self._pipe_accum: float = 0.0
        for _ in range(self._pipe_queue_size):
            self._pipe_queue.append(0.0)

        # Diagnostics (updated each step)
        self.effective_heater_power: float = 0.0  # total Q_out W from radiator
        self.q_in_watts: float = 0.0              # W into radiator from boiler
        self.q_out_watts: float = 0.0             # W total from radiator
        self.q_conv_watts: float = 0.0            # W convective portion → air
        self.q_rad_watts: float = 0.0             # W radiative portion → fabric
        self.solar_gain_watts: float = 0.0        # total solar gain W
        self.fabric_heat_flux: float = 0.0        # W: fabric→air (positive = heating air)
        self.heating_rate: float = 0.0            # °C/s air node
        self.heat_loss_rate: float = 0.0          # °C/s air node
        self.net_heat_rate: float = 0.0           # °C/s air node

    # ------------------------------------------------------------------
    # Common interface
    # ------------------------------------------------------------------

    @property
    def temperature(self) -> float:
        """Primary output — room air temperature."""
        return self.t_air

    @property
    def power_setpoint(self) -> float:
        return self.valve_fraction

    def set_power_fraction(self, fraction: float) -> None:
        """Set valve position 0–1."""
        self.valve_fraction = _clamp(fraction, 0.0, 1.0)

    def set_external_temperature(self, temp: float) -> None:
        self.external_temperature = temp

    def set_solar_irradiance(self, irradiance: float) -> None:
        self.solar_irradiance = max(0.0, irradiance)

    def set_flow_temperature(self, temp: float) -> None:
        self.flow_temperature = temp

    # ------------------------------------------------------------------
    # Integration
    # ------------------------------------------------------------------

    def step(self, dt: float) -> None:
        if dt <= 0:
            return
        # Sub-step size: limited by the fastest time constant in the system.
        # For the radiator: τ_rad_charge = C_rad / (ṁ_max × C_water)
        # For the room nodes: τ_air_fab = C_air × R_fab
        tau_rad = self.c_radiator / max(0.001, self.flow_rate_max * C_WATER)
        tau_air = self.c_air * self.r_fabric
        max_sub = max(0.5, min(tau_rad, tau_air) / 10.0)
        remaining = dt
        while remaining > 0:
            sub = min(remaining, max_sub)
            self._euler(sub)
            remaining -= sub
        self._update_diagnostics()
        self._push_pipe_queue(dt)

    def _euler(self, dt: float) -> None:
        # --- Delayed valve position ---
        if self.pipe_delay > 0:
            effective_valve = self._pipe_queue[0] if self._pipe_queue else 0.0
        else:
            effective_valve = self.valve_fraction

        # --- Radiator heat input from boiler ---
        delta_flow = max(0.0, self.flow_temperature - self.t_rad)
        q_in = effective_valve * self.flow_rate_max * C_WATER * delta_flow

        # --- Radiator heat output split (BS EN 442 / ISO 11855) ---
        # Q_out total follows the power law referenced to air temperature.
        # The convective fraction heats the air directly; the radiative fraction
        # is absorbed by fabric surfaces (walls, floor, ceiling).
        delta_rad_air = self.t_rad - self.t_air
        if delta_rad_air > 0:
            q_out = self.k_radiator * (delta_rad_air ** self.radiator_exponent)
        else:
            q_out = -self.k_radiator * ((-delta_rad_air) ** self.radiator_exponent)
        q_conv = q_out * self.radiator_convective_fraction        # → air node
        q_rad  = q_out * (1.0 - self.radiator_convective_fraction)  # → fabric node

        # --- Solar gain split ---
        q_solar_total = self.solar_irradiance * self.window_area * self.window_transmittance
        q_solar_fab = q_solar_total * self.SOLAR_FABRIC_FRACTION
        q_solar_air = q_solar_total * (1.0 - self.SOLAR_FABRIC_FRACTION)

        # --- Room heat exchange ---
        q_fab_to_air = (self.t_fabric - self.t_air) / self.r_fabric
        q_inf        = (self.t_air    - self.external_temperature) / self.r_inf
        q_fab_to_ext = (self.t_fabric - self.external_temperature) / self.r_ext

        # --- ODEs ---
        dT_rad = (q_in - q_out) / self.c_radiator

        dT_air = (
            q_conv         # convective portion of radiator output → air
            + q_solar_air
            + q_fab_to_air # fabric convects into air (or absorbs from air if cold)
            - q_inf        # infiltration loss
        ) / self.c_air

        dT_fab = (
            q_rad          # radiative portion of radiator output absorbed by fabric
            + q_solar_fab  # solar absorbed by fabric
            - q_fab_to_air # fabric loses heat to air (equal and opposite to dT_air term)
            - q_fab_to_ext # fabric loses heat to outside
        ) / self.c_fabric

        # --- Integrate ---
        self.t_rad     += dT_rad * dt
        self.t_air     += dT_air * dt
        self.t_fabric  += dT_fab * dt

        # Store instantaneous flows for diagnostics
        self.q_in_watts             = q_in
        self.q_out_watts            = q_out
        self.q_conv_watts           = q_conv
        self.q_rad_watts            = q_rad
        self.effective_heater_power = q_out   # total output from radiator
        self.solar_gain_watts       = q_solar_total
        self.fabric_heat_flux       = q_fab_to_air

    def _push_pipe_queue(self, dt: float) -> None:
        """Advance the pipe delay queue by dt seconds of simulated time."""
        if self.pipe_delay <= 0:
            return
        self._pipe_accum += dt
        while self._pipe_accum >= 1.0:
            self._pipe_accum -= 1.0
            self._pipe_queue.append(self.valve_fraction)
            if len(self._pipe_queue) > self._pipe_queue_size:
                self._pipe_queue.popleft()

    def _update_diagnostics(self) -> None:
        q_inf = (self.t_air - self.external_temperature) / self.r_inf
        self.heating_rate    = self.q_out_watts / self.c_air
        self.heat_loss_rate  = (q_inf + max(0.0, -self.fabric_heat_flux)) / self.c_air
        self.net_heat_rate   = self.heating_rate - self.heat_loss_rate

    # ------------------------------------------------------------------
    # Computed properties
    # ------------------------------------------------------------------

    @property
    def return_temperature(self) -> float:
        """AMTD return temperature estimate, clamped to T_air."""
        t_return = 2.0 * self.t_rad - self.flow_temperature
        return max(t_return, self.t_air)

    @property
    def nominal_output_at_dt50(self) -> float:
        return self.k_radiator * (50.0 ** self.radiator_exponent)

    @property
    def current_output_fraction(self) -> float:
        nom = self.nominal_output_at_dt50
        return self.q_out_watts / nom if nom > 0 else 0.0

    @property
    def total_heat_loss_watts(self) -> float:
        q_inf = (self.t_air    - self.external_temperature) / self.r_inf
        q_fab = (self.t_fabric - self.external_temperature) / self.r_ext
        return q_inf + q_fab

    @property
    def effective_u_value(self) -> float:
        delta = self.t_air - self.external_temperature
        if abs(delta) < 0.01:
            return 0.0
        return self.total_heat_loss_watts / delta

    @property
    def extra_state(self) -> dict:
        return {
            "radiator_temperature":      round(self.t_rad, 2),
            "fabric_temperature":        round(self.t_fabric, 2),
            "q_in_watts":                round(self.q_in_watts, 1),
            "q_out_watts":               round(self.q_out_watts, 1),
            "q_conv_watts":              round(self.q_conv_watts, 1),
            "q_rad_to_fabric_watts":     round(self.q_rad_watts, 1),
            "convective_fraction":       self.radiator_convective_fraction,
            "return_temperature":        round(self.return_temperature, 2),
            "solar_gain_w":              round(self.solar_gain_watts, 1),
            "fabric_to_air_flux_w":      round(self.fabric_heat_flux, 1),
            "total_heat_loss_w":         round(self.total_heat_loss_watts, 1),
            "effective_u_value_W_per_C": round(self.effective_u_value, 2),
            "nominal_output_w_at_dt50":  round(self.nominal_output_at_dt50, 1),
            "output_fraction":           round(self.current_output_fraction, 3),
            "valve_position_pct":        round(self.valve_fraction * 100, 1),
        }
