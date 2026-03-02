# Thermal Models

Heating Simulator provides four physics-based thermal models of increasing complexity. All four share a common interface and can be configured independently.

---

## Model 1 — Simple (R1C1)

### Overview

A single lumped-capacitance node representing the room air. The simplest model: one state variable, four parameters. Good for initial tuning of thermostats, testing PID logic, or any situation where you need a fast, easy-to-parameterise simulation.

An optional first-order lag on the heater output models the thermal inertia of an electric element or convector heater (the delay before full heat reaches the air).

### Mathematical model

```
C · dT/dt = Q_eff(t) + Q_internal − K_eff · (T − T_ext)
```

where:

| Symbol | Description |
|---|---|
| `C` | Room thermal mass (J/°C) |
| `T` | Room air temperature (°C) |
| `T_ext` | External temperature (°C) |
| `Q_eff` | Effective heater power after lag (W) |
| `Q_internal` | Internal gains from occupancy/cooking (W) |
| `K_eff` | Effective heat loss coefficient, including weather multiplier (W/°C) |

**Heater lag** (first-order low-pass filter on heater output):

```
τ · dQ_eff/dt = Q_input − Q_eff
```

This is solved with the exact discrete-time exponential:

```
Q_eff(t) = Q_target + (Q_eff(t−dt) − Q_target) · exp(−dt/τ)
```

This is unconditionally stable at any update interval, including when `dt >> τ`.

When `τ = 0`, `Q_eff = Q_input` immediately (no lag).

**Steady-state temperature** (useful for sizing):

```
T_ss = T_ext + (Q_input + Q_internal) / K_eff
```

**Room time constant:**

```
τ_room = C / K  (seconds)
```

The room reaches ~63% of its steady-state temperature change after one `τ_room`, and is essentially stable after `3 × τ_room`.

### Parameters

| Key | Description | Range | Default |
|---|---|---|---|
| `heater_power_watts` | Nominal heater output at 100% input (W) | 100–50 000 | 2 000 |
| `heat_loss_coefficient` | K-value: heat loss per °C above external (W/°C) | 0.001–2 000 | 50 |
| `thermal_mass` | Room thermal capacitance C (J/°C) | 100–20 000 000 | 350 000 |
| `thermal_inertia` | First-order heater lag τ (s); 0 = instant | 0–7 200 | 0 |

### Sizing guidance

**`heat_loss_coefficient`** — Measure or estimate your room's heat loss. A typical UK semi-detached living room loses 50–100 W/°C. You can estimate it from the heating system: if a 2 kW heater holds the room 40 °C above outside, `K ≈ 2000/40 = 50 W/°C`.

**`thermal_mass`** — The air alone in a 50 m³ room has about 60 000 J/°C (air density ~1.2 kg/m³, specific heat ~1 000 J/kg·°C). A furnished room including furniture, carpet, and soft furnishings is typically 300 000–600 000 J/°C.

**`heater_power_watts`** — Match your actual heater wattage. The simulation scales linearly from 0 to 100%.

---

## Model 2 — R2C2 with Solar Gain

### Overview

A two-node model splitting the room into an **air node** (T_air) and a **building fabric node** (T_fab, representing walls, floor, and ceiling). Based on the ISO 13790 simplified hourly method.

This model captures three effects the simple model cannot:
- **Fabric lag** — cold walls slow down the first warm-up of a room even after the air is warm.
- **Solar gain** — sunlight through windows heats the room without the boiler.
- **Independent heat-loss paths** — infiltration (draughts) through the air node and conduction through the fabric node behave differently.

### Mathematical model

```
C_air · dT_air/dt = Q_heater + Q_solar_air + Q_internal
                    + (T_fab − T_air) / R_fab
                    − (T_air − T_ext) / R_inf

C_fab · dT_fab/dt = Q_solar_fab
                    + (T_air − T_fab) / R_fab
                    − (T_fab − T_ext) / R_ext
```

**Solar gain split:**

Shortwave solar radiation passes through the air and is absorbed primarily by opaque surfaces. The split used is:

```
Q_solar_total = irradiance × window_area × g-value
Q_solar_fab   = 0.9 × Q_solar_total    (absorbed by fabric)
Q_solar_air   = 0.1 × Q_solar_total    (minor direct air heating)
```

**Heat flows summary:**

| Flow | Equation | Direction |
|---|---|---|
| Heater to air | `Q_heater = setpoint × P_nominal` | → air |
| Solar to air | `0.1 × irradiance × A_window × g` | → air |
| Solar to fabric | `0.9 × irradiance × A_window × g` | → fabric |
| Internal gains | `Q_internal` (occupancy/cooking) | → air |
| Air ↔ Fabric exchange | `(T_fab − T_air) / R_fab` | bidirectional |
| Infiltration loss | `(T_air − T_ext) / R_inf` | air → outside |
| Fabric conduction loss | `(T_fab − T_ext) / R_ext` | fabric → outside |

**Numerical stability:** Sub-stepping is applied automatically. The sub-step size is limited to `min(C_air·R_fab, C_fab·R_ext) / 5`, ensuring stability without user configuration.

### Parameters

| Key | Description | Range | Default |
|---|---|---|---|
| `heater_power_watts_r2c2` | Nominal heater output (W) | 100–50 000 | 2 000 |
| `c_air` | Air node thermal mass — **air plus all room contents** (J/°C) | 1 000–20 000 000 | 350 000 |
| `c_fabric` | Building fabric thermal mass — walls, floor, ceiling only (J/°C) | 10 000–50 000 000 | 5 000 000 |
| `r_fabric` | Internal surface resistance: air ↔ fabric (°C/W) | 0.0001–1.0 | 0.005 |
| `r_ext` | External fabric resistance: fabric ↔ outside (°C/W) | 0.0001–50.0 | 0.020 |
| `r_infiltration` | Infiltration path: air ↔ outside (°C/W) | 0.001–10.0 | 0.067 |
| `window_area_m2` | Glazed area receiving solar radiation (m²) | 0–100 | 2.0 |
| `window_transmittance` | Solar heat gain coefficient (g-value, 0–1) | 0–1 | 0.6 |
| `solar_irradiance_entity` | HA entity providing irradiance (W/m²); overrides fixed value | — | — |
| `solar_irradiance_fixed` | Fixed fallback irradiance (W/m²) | 0–1 500 | 0.0 |

### Sizing guidance

**`c_air`** must include air *plus all room contents* that heat up on the same timescale as the air: furniture, carpet, curtains, books. Air alone in 50 m³ ≈ 60 000 J/°C. A furnished 20 m² room is typically 300 000–600 000 J/°C.

**`c_fabric`** is the building fabric only: walls, floor, ceiling. Approximate:
- Brick cavity construction, 60 m² surfaces: ~6 000 000 J/°C
- Timber frame / plasterboard, same area: ~500 000 J/°C

**`r_ext`** relates to your fabric heat loss: `R_ext = 1 / K_fabric`. If the fabric (not infiltration) loses 40 W/°C, then `R_ext = 1/40 = 0.025 °C/W`.

**`r_infiltration`** relates to ventilation rate. At 0.5 ACH in a 100 m³ room: volume flow = 100×0.5/3600 = 0.014 m³/s; heat capacity flow = 0.014 × 1200 J/m³·°C ≈ 16.7 W/°C; `R_inf = 1/16.7 ≈ 0.060 °C/W`.

**`r_fabric`** represents the internal surface conductance (convection and radiation between air and fabric surfaces). A value of 0.005 °C/W corresponds to 200 W/°C of internal coupling — appropriate for a well-mixed room. Higher values (worse coupling) slow the air-fabric heat exchange.

**`window_transmittance`** (g-value) is typically 0.6–0.7 for standard double glazing, 0.3–0.5 for low-emissivity coatings.

---

## Model 3 — Wet Radiator

### Overview

An explicit wet radiator model with two state variables: radiator water temperature (T_rad) and room temperature (T_room). The valve position controls mass flow rate, which determines how fast the boiler heats the radiator. Heat output to the room follows the BS EN 442 power law.

This model captures the key asymmetry of a wet heating system:
- **Heating is fast** — driven by a large ΔT between flow water and radiator (~50 °C).
- **Cooling is slow** — the radiator passively decays toward room temperature, and BS EN 442 output drops as ΔT shrinks.

Output depends on `(T_rad − T_room)^n`, not valve position. A half-open valve does not give half the heat.

### Mathematical model

```
C_rad · dT_rad/dt  = Q_in − Q_out
C_room · dT_room/dt = Q_out + Q_internal − K_eff · (T_room − T_ext)
```

**Heat input from boiler:**

```
Q_in = valve × ṁ_max × C_water × max(0, T_flow − T_rad)
```

The `max(0, ...)` term means Q_in falls to zero when the radiator reaches flow temperature — the radiator cannot be overheated.

**Heat output to room (BS EN 442 power law):**

```
Q_out = K_rad × |T_rad − T_room|^n × sign(T_rad − T_room)
```

where `n ≈ 1.3` for panel radiators (BS EN 442 default). The `sign` term allows the radiator to act as a heat sink if it drops below room temperature (rare, but physically correct).

**Heat loss:**

```
Q_loss = K_eff · (T_room − T_ext)
K_eff = K_loss × weather_multiplier
```

**Pipe delay:** An optional FIFO queue buffers the valve command so the radiator only sees flow commanded `N` seconds ago. The queue is 1-second resolution, accumulated correctly across arbitrary update intervals.

**Return temperature estimate (AMTD approximation):**

```
T_return = 2·T_rad − T_flow   (clamped to ≥ T_room)
```

This avoids the singularity in the energy-balance method at low flow rates.

### Parameters

| Key | Description | Range | Default |
|---|---|---|---|
| `flow_temperature` | Boiler/heat pump flow temperature (°C) | 20–90 | 70 |
| `flow_temperature_entity` | HA entity providing live flow temperature | — | — |
| `c_radiator` | Radiator thermal mass — water + metal (J/°C) | 500–100 000 | 8 000 |
| `k_radiator` | BS EN 442 emission coefficient (W/°C^n) | 0.1–500 | 10 |
| `radiator_exponent` | BS EN 442 exponent n | 1.0–2.0 | 1.3 |
| `flow_rate_max_kg_s` | Maximum water mass flow rate (kg/s) | 0.001–1.0 | 0.05 |
| `heat_loss_coefficient_rad` | Room K-value — heat loss per °C above external (W/°C) | 0.001–2 000 | 50 |
| `c_room_rad` | Room thermal mass — air plus all contents (J/°C) | 1 000–20 000 000 | 500 000 |
| `pipe_delay_seconds` | Dead time for hot water to reach radiator (s) | 0–600 | 0 |

### Sizing guidance

**`k_radiator`** — Size from your radiator's nameplate rating. Radiators are rated at standard ΔT50 (flow 70 °C, room 20 °C):

```
K_rad = P_nominal / (ΔT50)^n = P_nominal / 50^1.3
```

Example: a 1 500 W radiator → `K_rad = 1500 / 142.5 ≈ 10.5 W/°C^1.3`.

**`radiator_exponent`** — Use 1.3 for panel radiators (BS EN 442 default), 1.5 for fan convectors.

**`c_radiator`** — Approximate from radiator type:
- Small single panel (Type 10): ~3 000 J/°C
- Large double panel + fins (Type 22): ~15 000 J/°C
- Default of 8 000 J/°C suits a medium domestic radiator.

**`flow_rate_max_kg_s`** — Typical domestic values are 0.03–0.10 kg/s (1.8–6 L/min). A 15 mm pipe at low pressure typically flows 0.05 kg/s. The maximum rate is achieved at full valve opening.

**`pipe_delay_seconds`** — Estimate from pipe volume and flow rate: `delay = volume(L) / flow_rate(L/s)`. A 5-metre run of 15 mm pipe holds about 0.7 L; at 0.05 kg/s that is ~14 seconds of delay.

**`flow_temperature`** — Traditional gas boiler: 70–80 °C. Heat pump (low temperature): 35–45 °C. Note that at low flow temperatures, radiator output at ΔT20 (40 °C flow, 20 °C room) drops to `K_rad × 20^1.3 / K_rad × 50^1.3 ≈ 28%` of its nameplate rating. You may need to oversize the radiator or reduce `flow_temperature` to reflect this.

---

## Model 4 — R2C2 + Radiator (Full Combined Model)

### Overview

The most physically complete model. Combines the wet radiator model with the two-node R2C2 room and solar gain. Three state variables: T_rad, T_air, T_fab.

The key addition over the standalone radiator model is the **convective split**: a real panel radiator does not heat the air alone. A significant fraction of its output is longwave radiation absorbed directly by floors and walls, which then slowly re-emit heat to the air. This produces:
- A more realistic fabric warm-up curve.
- Slower apparent air response than a purely convective heater.
- Better agreement with measured room data.

### Mathematical model

```
C_rad · dT_rad/dt = Q_in − Q_out

C_air · dT_air/dt = Q_conv + Q_solar_air + Q_internal
                    + (T_fab − T_air) / R_fab
                    − (T_air − T_ext) / R_inf

C_fab · dT_fab/dt = Q_rad + Q_solar_fab
                    − (T_fab − T_air) / R_fab
                    − (T_fab − T_ext) / R_ext
```

**Radiator heat input** (same as the wet radiator model):

```
Q_in = valve × ṁ_max × C_water × max(0, T_flow − T_rad)
```

**Radiator heat output split (BS EN 442 / ISO 11855):**

```
Q_out  = K_rad × |T_rad − T_air|^n
Q_conv = Q_out × conv_frac         → air node
Q_rad  = Q_out × (1 − conv_frac)  → fabric node
```

Note that `Q_out` is referenced to `T_air` (not `T_room` as in the standalone model), because the radiative fraction warms fabric surfaces directly.

**Solar gain split** (same 90/10 rule as the R2C2 model):

```
Q_solar_total = irradiance × A_window × g
Q_solar_fab   = 0.9 × Q_solar_total
Q_solar_air   = 0.1 × Q_solar_total
```

**Numerical stability:** Sub-step size is `min(τ_rad_charge, τ_air_fab) / 10`, where `τ_rad_charge = C_rad / (ṁ_max × C_water)` and `τ_air_fab = C_air × R_fab`.

### Convective fraction by radiator type

| Type | Description | Convective fraction |
|---|---|---|
| Type 10 | Single panel, no fins | ~0.50 |
| Type 11 | Single panel + fins | ~0.65 |
| Type 21 | Double panel + fins | ~0.75 |
| Type 22 | Double panel, 2× fins (most common UK domestic) | ~0.80 |
| Fan coil / convector | — | ~0.90–0.95 |
| Underfloor heating (water) | — | ~0.50 |

Source: BS EN 442 / ISO 11855.

### Parameters

All parameters from the Wet Radiator model (except `heat_loss_coefficient_rad` and `c_room_rad`, which are replaced by the R2C2 fabric node), plus all parameters from the R2C2 model (except `heater_power_watts_r2c2`, since the radiator is the heater), plus one additional parameter:

| Key | Description | Range | Default |
|---|---|---|---|
| `radiator_convective_fraction` | Fraction of Q_out directed to the air node (remainder goes to fabric) | 0.1–1.0 | 0.75 |

See the Wet Radiator and R2C2 parameter tables for the remaining parameters. Sizing guidance from those sections applies unchanged.

---

## Behavioural Calibration

All four models support back-calculation of key thermal parameters from **observed room behaviour**, instead of from physical measurements. This is useful when you can log how your room responds to heating and cooling but cannot measure construction details directly.

### First-order thermostat model

Calibration uses the simplified first-order model:

```
dT/dt = a · u  −  b · (T − T_ext)
```

where `u = 1` when heating at full power, `u = 0` when off.

| Symbol | Meaning | Unit |
|---|---|---|
| `a` | Heating rate coefficient at full power and steady radiator temperature | °C/min |
| `b` | Loss rate coefficient | 1/min |
| `τ` | Cooling time constant = 1/b | min |

### Calibration fields

| Field | Description |
|---|---|
| `calib_a` | Observed initial heating rate (°C/min) when heater is fully on and radiator is at operating temperature |
| `calib_b` | Observed loss rate coefficient 1/τ (1/min) |
| `calib_tau` | Observed cooling time constant (min); takes priority over `calib_b` if both set |

**Calibration fields are consumed on save.** The integration back-calculates the physical parameters, writes them into the normal parameter slots, and clears the calibration fields. They do not persist across restarts.

### What gets back-calculated

| Model | Inputs used | Parameters derived |
|---|---|---|
| Simple | `a` + (`tau` or `b`) + `heater_power` | `thermal_mass`, `heat_loss_coefficient` |
| R2C2 | `tau` or `b` only (`calib_a` is ignored) | `r_ext` (via bisection on the slow system eigenvalue) |
| Radiator | `a` + (`tau` or `b`) | `c_room_rad`, `heat_loss_coefficient_rad` |
| R2C2 + Radiator | `a` + (`tau` or `b`) | `c_air`, `r_ext` (preserving the existing c_fabric/c_air ratio) |

### Calibration procedure

**For Simple or Radiator models:**

1. Let the room reach a stable temperature with heating off.
2. Open the valve / turn the heater fully on and log the initial rate of temperature rise. This gives `a` (°C/min).
3. Turn the heater off from a stable heated temperature and log the time for the room to cool. Fit a time constant; this gives `tau` (min).
4. Enter `a` and `tau` (and your known `heater_power_watts`) in the options flow and save.

For the **Radiator model**, measure `a` after the radiator has already warmed up (flow water hot, valve fully open), so that Q_out is at its operating-point value. The back-calculation uses `k_radiator`, `radiator_exponent`, `flow_temperature`, and `flow_rate_max` to determine that output.

**For R2C2 models:**

Only `tau` (the long-term cooling time constant, measured over several hours) is used. `calib_a` is ignored. The integration finds `r_ext` by bisection on the slow eigenvalue of the two-node system matrix, leaving `c_air`, `c_fabric`, `r_fabric`, and `r_infiltration` unchanged.

### State restoration

Room temperature, fabric temperature (R2C2 models), and radiator temperature (radiator models) are restored from the Home Assistant state machine on restart using `RestoreSensor`. A simulation that has been running for hours resumes with realistic initial conditions rather than starting from `initial_temperature`.
