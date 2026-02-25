"""Config flow for Heating Simulator — multi-step, model-type aware."""
from __future__ import annotations

from typing import Any
import voluptuous as vol

from homeassistant import config_entries
from homeassistant.core import callback
from homeassistant.data_entry_flow import FlowResult

from .const import (
    DOMAIN,
    CONF_MODEL_TYPE,
    CONF_CONTROL_MODE,
    CONF_INITIAL_TEMP,
    CONF_EXTERNAL_TEMP,
    CONF_EXTERNAL_TEMP_FIXED,
    CONF_UPDATE_INTERVAL,
    # simple
    CONF_HEATER_POWER,
    CONF_HEAT_LOSS_COEFF,
    CONF_THERMAL_MASS,
    CONF_THERMAL_INERTIA,
    # R2C2
    CONF_C_AIR,
    CONF_C_FABRIC,
    CONF_R_FABRIC,
    CONF_R_EXT,
    CONF_R_INF,
    CONF_HEATER_POWER_R2C2,
    CONF_SOLAR_ENTITY,
    CONF_SOLAR_FIXED,
    CONF_WINDOW_AREA,
    CONF_WINDOW_TRANSMITTANCE,
    # radiator
    CONF_FLOW_TEMP,
    CONF_FLOW_TEMP_ENTITY,
    CONF_C_RAD,
    CONF_K_RAD,
    CONF_RAD_EXPONENT,
    CONF_RAD_CONVECTIVE_FRACTION,
    CONF_FLOW_RATE_MAX,
    CONF_HEAT_LOSS_COEFF_RAD,
    CONF_C_ROOM_RAD,
    CONF_PIPE_DELAY,
    # calibration
    CONF_CALIB_A,
    CONF_CALIB_B,
    CONF_CALIB_TAU,
    # model types / control modes
    MODEL_SIMPLE,
    MODEL_R2C2,
    MODEL_RADIATOR,
    MODEL_R2C2_RADIATOR,
    CONTROL_MODE_LINEAR,
    CONTROL_MODE_PWM,
    # defaults
    DEFAULT_INITIAL_TEMP,
    DEFAULT_EXTERNAL_TEMP_FIXED,
    DEFAULT_UPDATE_INTERVAL,
    DEFAULT_HEATER_POWER,
    DEFAULT_HEAT_LOSS_COEFF,
    DEFAULT_THERMAL_MASS,
    DEFAULT_THERMAL_INERTIA,
    DEFAULT_C_AIR,
    DEFAULT_C_FABRIC,
    DEFAULT_R_FABRIC,
    DEFAULT_R_EXT,
    DEFAULT_R_INF,
    DEFAULT_HEATER_POWER_R2C2,
    DEFAULT_SOLAR_FIXED,
    DEFAULT_WINDOW_AREA,
    DEFAULT_WINDOW_TRANSMITTANCE,
    DEFAULT_FLOW_TEMP,
    DEFAULT_C_RAD,
    DEFAULT_K_RAD,
    DEFAULT_RAD_EXPONENT,
    DEFAULT_RAD_CONVECTIVE_FRACTION,
    DEFAULT_FLOW_RATE_MAX,
    DEFAULT_HEAT_LOSS_COEFF_RAD,
    DEFAULT_C_ROOM_RAD,
    DEFAULT_PIPE_DELAY,
)

# ---------------------------------------------------------------------------
# Step 1 — common settings (name, model type, control mode, temps, interval)
# ---------------------------------------------------------------------------

STEP_COMMON_SCHEMA = vol.Schema({
    vol.Required("name", default="Heating Simulator"): str,
    vol.Required(CONF_MODEL_TYPE, default=MODEL_SIMPLE): vol.In(
        [MODEL_SIMPLE, MODEL_R2C2, MODEL_RADIATOR, MODEL_R2C2_RADIATOR]
    ),
    vol.Required(CONF_CONTROL_MODE, default=CONTROL_MODE_LINEAR): vol.In(
        [CONTROL_MODE_LINEAR, CONTROL_MODE_PWM]
    ),
    vol.Required(CONF_INITIAL_TEMP, default=DEFAULT_INITIAL_TEMP): vol.Coerce(float),
    vol.Optional(CONF_EXTERNAL_TEMP, default=""): str,
    vol.Required(CONF_EXTERNAL_TEMP_FIXED, default=DEFAULT_EXTERNAL_TEMP_FIXED): vol.Coerce(float),
    vol.Required(CONF_UPDATE_INTERVAL, default=DEFAULT_UPDATE_INTERVAL): vol.All(
        vol.Coerce(int), vol.Range(min=1, max=300)
    ),
})

# ---------------------------------------------------------------------------
# Step 2a — Simple model params
# ---------------------------------------------------------------------------

STEP_SIMPLE_SCHEMA = vol.Schema({
    vol.Required(CONF_HEATER_POWER, default=DEFAULT_HEATER_POWER): vol.All(
        vol.Coerce(float), vol.Range(min=100, max=50000)
    ),
    vol.Required(CONF_HEAT_LOSS_COEFF, default=DEFAULT_HEAT_LOSS_COEFF): vol.All(
        vol.Coerce(float), vol.Range(min=0.001, max=2000)
    ),
    vol.Required(CONF_THERMAL_MASS, default=DEFAULT_THERMAL_MASS): vol.All(
        vol.Coerce(float), vol.Range(min=100, max=20_000_000)
    ),
    vol.Required(CONF_THERMAL_INERTIA, default=DEFAULT_THERMAL_INERTIA): vol.All(
        vol.Coerce(float), vol.Range(min=0, max=7200)
    ),
    # --- Behavioural calibration (optional) ----------------------------------
    # Provide a and tau (or b) to back-calculate thermal_mass and heat_loss_coeff
    # from observed room behaviour. heater_power must be set correctly above.
    # Leave at 0 to use the physical values above.
    vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
    vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
    vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
})

# ---------------------------------------------------------------------------
# Step 2b — R2C2 model params
# ---------------------------------------------------------------------------

STEP_R2C2_SCHEMA = vol.Schema({
    vol.Required(CONF_HEATER_POWER_R2C2, default=DEFAULT_HEATER_POWER_R2C2): vol.All(
        vol.Coerce(float), vol.Range(min=100, max=50000)
    ),
    vol.Required(CONF_C_AIR, default=DEFAULT_C_AIR): vol.All(
        vol.Coerce(float), vol.Range(min=1000, max=20_000_000)
    ),
    vol.Required(CONF_C_FABRIC, default=DEFAULT_C_FABRIC): vol.All(
        vol.Coerce(float), vol.Range(min=10000, max=50_000_000)
    ),
    vol.Required(CONF_R_FABRIC, default=DEFAULT_R_FABRIC): vol.All(
        vol.Coerce(float), vol.Range(min=0.0001, max=1.0)
    ),
    vol.Required(CONF_R_EXT, default=DEFAULT_R_EXT): vol.All(
        vol.Coerce(float), vol.Range(min=0.0001, max=50.0)
    ),
    vol.Required(CONF_R_INF, default=DEFAULT_R_INF): vol.All(
        vol.Coerce(float), vol.Range(min=0.001, max=10.0)
    ),
    vol.Required(CONF_WINDOW_AREA, default=DEFAULT_WINDOW_AREA): vol.All(
        vol.Coerce(float), vol.Range(min=0, max=100)
    ),
    vol.Required(CONF_WINDOW_TRANSMITTANCE, default=DEFAULT_WINDOW_TRANSMITTANCE): vol.All(
        vol.Coerce(float), vol.Range(min=0, max=1)
    ),
    vol.Optional(CONF_SOLAR_ENTITY, default=""): str,
    vol.Required(CONF_SOLAR_FIXED, default=DEFAULT_SOLAR_FIXED): vol.All(
        vol.Coerce(float), vol.Range(min=0, max=1500)
    ),
    # --- Behavioural calibration (optional) ----------------------------------
    # Provide tau to back-calculate r_ext from the observed long-term cooling
    # time constant (typically hours for a well-insulated room).
    # c_air, c_fabric, r_fabric, r_inf and heater_power are kept unchanged.
    # Leave at 0 to use the physical values above.
    vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
    vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
})

# ---------------------------------------------------------------------------
# Step 2c — Wet radiator params
# ---------------------------------------------------------------------------

STEP_RADIATOR_SCHEMA = vol.Schema({
    vol.Required(CONF_FLOW_TEMP, default=DEFAULT_FLOW_TEMP): vol.All(
        vol.Coerce(float), vol.Range(min=20, max=90)
    ),
    vol.Optional(CONF_FLOW_TEMP_ENTITY, default=""): str,
    vol.Required(CONF_C_RAD, default=DEFAULT_C_RAD): vol.All(
        vol.Coerce(float), vol.Range(min=500, max=100_000)
    ),
    vol.Required(CONF_K_RAD, default=DEFAULT_K_RAD): vol.All(
        vol.Coerce(float), vol.Range(min=0.1, max=500)
    ),
    vol.Required(CONF_RAD_EXPONENT, default=DEFAULT_RAD_EXPONENT): vol.All(
        vol.Coerce(float), vol.Range(min=1.0, max=2.0)
    ),
    vol.Required(CONF_FLOW_RATE_MAX, default=DEFAULT_FLOW_RATE_MAX): vol.All(
        vol.Coerce(float), vol.Range(min=0.001, max=1.0)
    ),
    vol.Required(CONF_HEAT_LOSS_COEFF_RAD, default=DEFAULT_HEAT_LOSS_COEFF_RAD): vol.All(
        vol.Coerce(float), vol.Range(min=0.001, max=2000)
    ),
    vol.Required(CONF_C_ROOM_RAD, default=DEFAULT_C_ROOM_RAD): vol.All(
        vol.Coerce(float), vol.Range(min=1000, max=20_000_000)
    ),
    vol.Required(CONF_PIPE_DELAY, default=DEFAULT_PIPE_DELAY): vol.All(
        vol.Coerce(float), vol.Range(min=0, max=600)
    ),
    # --- Behavioural calibration (optional) ----------------------------------
    # Supply a and tau (or b) to back-calculate c_room and K_loss from observed
    # room behaviour instead of entering physical values directly.
    # Leave at 0 to use the physical values above.
    # On save these are consumed; the result is written into c_room / K_loss.
    vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
    vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
    vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
})

# Combined R2C2 + radiator schema — all parameters in one page.
# Note: no CONF_HEATER_POWER_R2C2 (the radiator IS the heater) and
# no CONF_HEAT_LOSS_COEFF_RAD / CONF_C_ROOM_RAD (replaced by R2C2 fabric params).
STEP_R2C2_RADIATOR_SCHEMA = vol.Schema({
    # Radiator
    vol.Required(CONF_FLOW_TEMP, default=DEFAULT_FLOW_TEMP): vol.All(
        vol.Coerce(float), vol.Range(min=20, max=90)
    ),
    vol.Optional(CONF_FLOW_TEMP_ENTITY, default=""): str,
    vol.Required(CONF_C_RAD, default=DEFAULT_C_RAD): vol.All(
        vol.Coerce(float), vol.Range(min=500, max=100_000)
    ),
    vol.Required(CONF_K_RAD, default=DEFAULT_K_RAD): vol.All(
        vol.Coerce(float), vol.Range(min=0.1, max=500)
    ),
    vol.Required(CONF_RAD_EXPONENT, default=DEFAULT_RAD_EXPONENT): vol.All(
        vol.Coerce(float), vol.Range(min=1.0, max=2.0)
    ),
    vol.Required(CONF_RAD_CONVECTIVE_FRACTION, default=DEFAULT_RAD_CONVECTIVE_FRACTION): vol.All(
        vol.Coerce(float), vol.Range(min=0.1, max=1.0)
    ),
    vol.Required(CONF_FLOW_RATE_MAX, default=DEFAULT_FLOW_RATE_MAX): vol.All(
        vol.Coerce(float), vol.Range(min=0.001, max=1.0)
    ),
    vol.Required(CONF_PIPE_DELAY, default=DEFAULT_PIPE_DELAY): vol.All(
        vol.Coerce(float), vol.Range(min=0, max=600)
    ),
    # Room (R2C2)
    # NOTE: c_air must include ALL room contents (air + furniture + carpet + curtains)
    # not just the air mass alone. Typical furnished room: 300,000–600,000 J/°C.
    vol.Required(CONF_C_AIR, default=DEFAULT_C_AIR): vol.All(
        vol.Coerce(float), vol.Range(min=1000, max=2_000_000)
    ),
    vol.Required(CONF_C_FABRIC, default=DEFAULT_C_FABRIC): vol.All(
        vol.Coerce(float), vol.Range(min=10000, max=50_000_000)
    ),
    vol.Required(CONF_R_FABRIC, default=DEFAULT_R_FABRIC): vol.All(
        vol.Coerce(float), vol.Range(min=0.0001, max=1.0)
    ),
    vol.Required(CONF_R_EXT, default=DEFAULT_R_EXT): vol.All(
        vol.Coerce(float), vol.Range(min=0.0001, max=50.0)
    ),
    vol.Required(CONF_R_INF, default=DEFAULT_R_INF): vol.All(
        vol.Coerce(float), vol.Range(min=0.001, max=10.0)
    ),
    # Solar
    vol.Required(CONF_WINDOW_AREA, default=DEFAULT_WINDOW_AREA): vol.All(
        vol.Coerce(float), vol.Range(min=0, max=100)
    ),
    vol.Required(CONF_WINDOW_TRANSMITTANCE, default=DEFAULT_WINDOW_TRANSMITTANCE): vol.All(
        vol.Coerce(float), vol.Range(min=0, max=1)
    ),
    vol.Optional(CONF_SOLAR_ENTITY, default=""): str,
    vol.Required(CONF_SOLAR_FIXED, default=DEFAULT_SOLAR_FIXED): vol.All(
        vol.Coerce(float), vol.Range(min=0, max=1500)
    ),
    # --- Behavioural calibration (optional) ----------------------------------
    # Supply a and tau (or b) to back-calculate c_air and r_ext from observed
    # room behaviour instead of entering physical values directly.
    # Leave at 0 to use the physical values above.
    vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
    vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
    vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
})


# ---------------------------------------------------------------------------
# Config flow
# ---------------------------------------------------------------------------

class HeatingSimulatorConfigFlow(config_entries.ConfigFlow, domain=DOMAIN):
    """Multi-step config flow. Step 1 = common, Step 2 = model-specific."""

    VERSION = 2

    def __init__(self) -> None:
        super().__init__()
        self._common_data: dict[str, Any] = {}

    async def async_step_user(self, user_input=None) -> FlowResult:
        """Step 1: common settings."""
        if user_input is not None:
            self._common_data = dict(user_input)   # copy — don't mutate framework dict
            model_type = user_input.get(CONF_MODEL_TYPE, MODEL_SIMPLE)
            if model_type == MODEL_R2C2:
                return await self.async_step_r2c2()
            elif model_type == MODEL_RADIATOR:
                return await self.async_step_radiator()
            elif model_type == MODEL_R2C2_RADIATOR:
                return await self.async_step_r2c2_radiator()
            else:
                return await self.async_step_simple()

        return self.async_show_form(step_id="user", data_schema=STEP_COMMON_SCHEMA)

    async def async_step_simple(self, user_input=None) -> FlowResult:
        if user_input is not None:
            name = self._common_data.get("name", "Heating Simulator")
            merged = {**self._common_data, **user_input}
            merged = apply_calibration_simple(merged, merged)
            return self.async_create_entry(title=name, data=merged)
        return self.async_show_form(step_id="simple", data_schema=STEP_SIMPLE_SCHEMA)

    async def async_step_r2c2(self, user_input=None) -> FlowResult:
        if user_input is not None:
            name = self._common_data.get("name", "Heating Simulator")
            merged = {**self._common_data, **user_input}
            merged = apply_calibration_r2c2(merged, merged)
            return self.async_create_entry(title=name, data=merged)
        return self.async_show_form(step_id="r2c2", data_schema=STEP_R2C2_SCHEMA)

    async def async_step_radiator(self, user_input=None) -> FlowResult:
        if user_input is not None:
            name = self._common_data.get("name", "Heating Simulator")
            merged = {**self._common_data, **user_input}
            merged = apply_calibration_radiator(merged, merged)
            return self.async_create_entry(title=name, data=merged)
        return self.async_show_form(step_id="radiator", data_schema=STEP_RADIATOR_SCHEMA)

    async def async_step_r2c2_radiator(self, user_input=None) -> FlowResult:
        if user_input is not None:
            name = self._common_data.get("name", "Heating Simulator")
            merged = {**self._common_data, **user_input}
            merged = apply_calibration_r2c2_radiator(merged, merged)
            return self.async_create_entry(title=name, data=merged)
        return self.async_show_form(step_id="r2c2_radiator", data_schema=STEP_R2C2_RADIATOR_SCHEMA)

    @staticmethod
    @callback
    def async_get_options_flow(config_entry):
        return HeatingSimulatorOptionsFlow()


# ---------------------------------------------------------------------------
# Options flow (reconfigure live params without re-adding)
# ---------------------------------------------------------------------------

class HeatingSimulatorOptionsFlow(config_entries.OptionsFlow):
    """
    Allow tweaking of runtime parameters without removing the entry.

    Options are stored in entry.options and always take precedence over
    entry.data (which holds the original setup values). The coordinator
    merges them as {**entry.data, **entry.options} so options win.

    To avoid configuration drift we populate schema defaults exclusively
    from the merged view — the same priority order the coordinator uses.
    """

    async def async_step_init(self, user_input=None) -> FlowResult:
        # Merge with options taking priority — same logic as coordinator
        cfg = {**self.config_entry.data, **self.config_entry.options}
        model_type = cfg.get(CONF_MODEL_TYPE, MODEL_SIMPLE)

        if user_input is not None:
            # Run calibration back-calculation if the user supplied a/tau/b
            if model_type == MODEL_SIMPLE:
                user_input = apply_calibration_simple(user_input, cfg)
            elif model_type == MODEL_R2C2:
                user_input = apply_calibration_r2c2(user_input, cfg)
            elif model_type == MODEL_RADIATOR:
                user_input = apply_calibration_radiator(user_input, cfg)
            elif model_type == MODEL_R2C2_RADIATOR:
                user_input = apply_calibration_r2c2_radiator(user_input, cfg)
            return self.async_create_entry(title="", data=user_input)

        if model_type == MODEL_R2C2:
            schema = _r2c2_options_schema(cfg)
        elif model_type == MODEL_RADIATOR:
            schema = _radiator_options_schema(cfg)
        elif model_type == MODEL_R2C2_RADIATOR:
            schema = _r2c2_radiator_options_schema(cfg)
        else:
            schema = _simple_options_schema(cfg)

        return self.async_show_form(step_id="init", data_schema=schema)


def _g(cfg, key, default):
    """
    Read a value from the merged config dict (options already take precedence
    because the caller always passes {**entry.data, **entry.options}).
    Falls back to the compiled default only if the key is absent from both.
    """
    return cfg.get(key, default)


# ---------------------------------------------------------------------------
# Calibration back-calculation helpers
# ---------------------------------------------------------------------------

_C_WATER = 4182.0  # J/(kg·K)


def _q_out_at_operating_point(
    k_rad: float,
    n: float,
    T_flow: float,
    T_room: float,
    flow_rate: float,
) -> float:
    """
    Return the radiator heat output Q_out (W) once the system reaches its
    operating steady-state radiator temperature.

    At steady state the heat carried in by the water equals Q_out:
        flow_rate × C_water × (T_flow − T_rad_ss) = k_rad × (T_rad_ss − T_room)^n

    Solved by bisection.  Falls back to the full-ΔT value if no root is found
    (e.g. T_flow too close to T_room).
    """
    def residual(T_r: float) -> float:
        return flow_rate * _C_WATER * (T_flow - T_r) - k_rad * (T_r - T_room) ** n

    lo, hi = T_room + 0.01, T_flow - 0.01
    if lo >= hi or residual(lo) * residual(hi) > 0:
        # Fallback: use full flow-to-room delta (overestimates slightly)
        return k_rad * (T_flow - T_room) ** n

    # Simple bisection (avoids importing scipy in HA)
    for _ in range(60):
        mid = (lo + hi) / 2
        if residual(mid) > 0:
            lo = mid
        else:
            hi = mid
    return k_rad * ((lo + hi) / 2 - T_room) ** n


def apply_calibration_radiator(user_input: dict, cfg: dict) -> dict:
    """
    If calib_a and (calib_tau or calib_b) are present and non-zero, back-calculate
    c_room and heat_loss_coeff for the simple wet-radiator model and write them
    into user_input, then clear the calibration fields so they don't persist.

    Mapping (thermostat model  dT/dt = a·u − b·(T−T_ext)):
        C_room  = Q_out_ss / (a / 60)
        K_loss  = C_room / (tau × 60)        tau = calib_tau if given, else 1/calib_b

    Q_out_ss is computed at the operating point defined by k_rad, T_flow, T_room
    (T_room taken as external_temperature_fixed + 15 as a representative setpoint).
    """
    a   = float(user_input.get(CONF_CALIB_A,   0) or 0)
    tau = float(user_input.get(CONF_CALIB_TAU, 0) or 0)
    b   = float(user_input.get(CONF_CALIB_B,   0) or 0)

    if a <= 0:
        return user_input  # no calibration requested

    # Resolve tau: explicit tau takes priority over 1/b
    if tau <= 0 and b > 0:
        tau = 1.0 / b
    if tau <= 0:
        return user_input  # need at least one of tau or b

    k_rad     = float(_g(cfg, CONF_K_RAD,       DEFAULT_K_RAD))
    n         = float(_g(cfg, CONF_RAD_EXPONENT, DEFAULT_RAD_EXPONENT))
    T_flow    = float(user_input.get(CONF_FLOW_TEMP, _g(cfg, CONF_FLOW_TEMP, DEFAULT_FLOW_TEMP)))
    flow_rate = float(_g(cfg, CONF_FLOW_RATE_MAX, DEFAULT_FLOW_RATE_MAX))
    T_ext     = float(user_input.get(CONF_EXTERNAL_TEMP_FIXED,
                                     _g(cfg, CONF_EXTERNAL_TEMP_FIXED, DEFAULT_EXTERNAL_TEMP_FIXED)))
    T_room    = T_ext + 15.0  # representative setpoint (15°C above external)

    Q_out_ss = _q_out_at_operating_point(k_rad, n, T_flow, T_room, flow_rate)

    c_room = Q_out_ss / (a / 60.0)           # J/°C
    k_loss = c_room / (tau * 60.0)           # W/°C

    out = dict(user_input)
    out[CONF_C_ROOM_RAD]          = round(c_room, 1)
    out[CONF_HEAT_LOSS_COEFF_RAD] = round(k_loss, 4)
    # Clear calibration fields — values are now stored in the physical params
    out.pop(CONF_CALIB_A,   None)
    out.pop(CONF_CALIB_B,   None)
    out.pop(CONF_CALIB_TAU, None)
    return out


def apply_calibration_r2c2_radiator(user_input: dict, cfg: dict) -> dict:
    """
    If calib_a and (calib_tau or calib_b) are present, back-calculate c_air and
    r_ext for the R2C2+Radiator model.

    Mapping:
        c_air  = Q_conv_ss / (a / 60)
                 where Q_conv_ss = conv_frac × Q_out_ss
        r_ext  derived from tau via:
                 tau × 60 = (c_air + c_fabric) / (K_inf + K_ext)
                 K_ext = (c_air + c_fabric)/(tau×60) − K_inf
                 r_ext = 1 / K_ext        (clamped to [0.0001, 50])
        c_fabric = (c_fabric/c_air ratio from current config) × c_air_new
        r_inf  unchanged (infiltration kept as configured)
        r_fabric unchanged
    """
    a   = float(user_input.get(CONF_CALIB_A,   0) or 0)
    tau = float(user_input.get(CONF_CALIB_TAU, 0) or 0)
    b   = float(user_input.get(CONF_CALIB_B,   0) or 0)

    if a <= 0:
        return user_input

    if tau <= 0 and b > 0:
        tau = 1.0 / b
    if tau <= 0:
        return user_input

    k_rad      = float(_g(cfg, CONF_K_RAD,                    DEFAULT_K_RAD))
    n          = float(_g(cfg, CONF_RAD_EXPONENT,              DEFAULT_RAD_EXPONENT))
    conv_frac  = float(_g(cfg, CONF_RAD_CONVECTIVE_FRACTION,   DEFAULT_RAD_CONVECTIVE_FRACTION))
    T_flow     = float(user_input.get(CONF_FLOW_TEMP, _g(cfg, CONF_FLOW_TEMP, DEFAULT_FLOW_TEMP)))
    flow_rate  = float(_g(cfg, CONF_FLOW_RATE_MAX,             DEFAULT_FLOW_RATE_MAX))
    r_inf      = float(user_input.get(CONF_R_INF, _g(cfg, CONF_R_INF, DEFAULT_R_INF)))
    T_ext      = float(user_input.get(CONF_EXTERNAL_TEMP_FIXED,
                                      _g(cfg, CONF_EXTERNAL_TEMP_FIXED, DEFAULT_EXTERNAL_TEMP_FIXED)))
    T_room     = T_ext + 15.0

    # Preserve existing c_fabric/c_air ratio
    c_air_cur  = float(_g(cfg, CONF_C_AIR,    DEFAULT_C_AIR))
    c_fab_cur  = float(_g(cfg, CONF_C_FABRIC, DEFAULT_C_FABRIC))
    fab_ratio  = c_fab_cur / c_air_cur if c_air_cur > 0 else (DEFAULT_C_FABRIC / DEFAULT_C_AIR)

    Q_out_ss  = _q_out_at_operating_point(k_rad, n, T_flow, T_room, flow_rate)
    Q_conv_ss = Q_out_ss * conv_frac

    c_air_new = Q_conv_ss / (a / 60.0)
    c_fab_new = fab_ratio * c_air_new

    # Derive r_ext from tau
    K_inf  = 1.0 / r_inf
    K_total_needed = (c_air_new + c_fab_new) / (tau * 60.0)
    K_ext  = K_total_needed - K_inf
    if K_ext > 0:
        r_ext_new = 1.0 / K_ext
    else:
        # tau is so long that infiltration alone dominates — very well insulated
        r_ext_new = 50.0
    r_ext_new = max(0.0001, min(50.0, r_ext_new))

    out = dict(user_input)
    out[CONF_C_AIR]    = round(c_air_new, 1)
    out[CONF_C_FABRIC] = round(c_fab_new, 1)
    out[CONF_R_EXT]    = round(r_ext_new, 6)
    out.pop(CONF_CALIB_A,   None)
    out.pop(CONF_CALIB_B,   None)
    out.pop(CONF_CALIB_TAU, None)
    return out


def apply_calibration_simple(user_input: dict, cfg: dict) -> dict:
    """
    Back-calculate thermal_mass and heat_loss_coeff for the Simple model from
    observed thermostat parameters a and tau (or b).

    The Simple model ODE is:
        C · dT/dt = P − K · (T − T_ext)
    Dividing by C gives the normalised thermostat form:
        dT/dt = a·u − b·(T − T_ext)
    where a = P/C and b = K/C = 1/tau (in consistent per-second units).

    Exact mapping (P taken from heater_power in user_input / cfg):
        C = P / (a / 60)          [a in °C/min → /60 for °C/s]
        K = C / (tau × 60)        [tau in min   → ×60 for seconds]

    tau takes priority over 1/b if both are given.
    """
    a   = float(user_input.get(CONF_CALIB_A,   0) or 0)
    tau = float(user_input.get(CONF_CALIB_TAU, 0) or 0)
    b   = float(user_input.get(CONF_CALIB_B,   0) or 0)

    if a <= 0:
        return user_input

    if tau <= 0 and b > 0:
        tau = 1.0 / b
    if tau <= 0:
        return user_input

    P = float(user_input.get(CONF_HEATER_POWER, _g(cfg, CONF_HEATER_POWER, DEFAULT_HEATER_POWER)))

    thermal_mass   = P / (a / 60.0)        # J/°C
    heat_loss_coeff = thermal_mass / (tau * 60.0)  # W/°C

    out = dict(user_input)
    out[CONF_THERMAL_MASS]    = round(thermal_mass, 1)
    out[CONF_HEAT_LOSS_COEFF] = round(heat_loss_coeff, 6)
    out.pop(CONF_CALIB_A,   None)
    out.pop(CONF_CALIB_B,   None)
    out.pop(CONF_CALIB_TAU, None)
    return out


def apply_calibration_r2c2(user_input: dict, cfg: dict) -> dict:
    """
    Back-calculate r_ext for the R2C2 model from the observed long-term cooling
    time constant tau (or b = 1/tau).

    The R2C2 system has two cooling eigenvalues. The slow one (dominant on
    long timescales, as fitted by a thermostat over many hours) equals:

        lambda_slow = largest eigenvalue of the 2×2 matrix:
            A = [[ -(K_inf+K_fab)/c_air,   K_fab/c_air  ],
                 [  K_fab/c_fab,          -(K_fab+K_ext)/c_fab ]]

    We find r_ext via bisection so that:
        -1 / lambda_slow = tau × 60   (seconds)

    c_air, c_fabric, r_fabric, r_inf and heater_power are kept unchanged.
    Only r_ext is written back.

    tau takes priority over 1/b. CONF_CALIB_A is ignored (not applicable —
    there is no radiator operating-point to anchor P/C_air independently).
    """
    tau = float(user_input.get(CONF_CALIB_TAU, 0) or 0)
    b   = float(user_input.get(CONF_CALIB_B,   0) or 0)

    if tau <= 0 and b > 0:
        tau = 1.0 / b
    if tau <= 0:
        return user_input

    c_air  = float(user_input.get(CONF_C_AIR,    _g(cfg, CONF_C_AIR,    DEFAULT_C_AIR)))
    c_fab  = float(user_input.get(CONF_C_FABRIC,  _g(cfg, CONF_C_FABRIC,  DEFAULT_C_FABRIC)))
    r_fab  = float(user_input.get(CONF_R_FABRIC,  _g(cfg, CONF_R_FABRIC,  DEFAULT_R_FABRIC)))
    r_inf  = float(user_input.get(CONF_R_INF,     _g(cfg, CONF_R_INF,     DEFAULT_R_INF)))

    target_lambda = -1.0 / (tau * 60.0)   # target slow eigenvalue (negative)

    def slow_eigenvalue(r_ext: float) -> float:
        """Slow (less-negative) eigenvalue of the 2-node cooling system matrix."""
        import math as _math
        K_inf = 1.0 / r_inf
        K_fab = 1.0 / r_fab
        K_ext = 1.0 / r_ext
        a11 = -(K_inf + K_fab) / c_air
        a12 =  K_fab / c_air
        a21 =  K_fab / c_fab
        a22 = -(K_fab + K_ext) / c_fab
        tr   = a11 + a22
        det  = a11 * a22 - a12 * a21
        disc = tr * tr - 4.0 * det
        if disc < 0:
            return tr / 2.0
        sqrt_disc = _math.sqrt(disc)
        return max((tr + sqrt_disc) / 2.0, (tr - sqrt_disc) / 2.0)

    def f(r_ext: float) -> float:
        return slow_eigenvalue(r_ext) - target_lambda

    # Bisect over r_ext in [0.0001, 100].
    # Increasing r_ext → less fabric loss → slower cooling → less-negative lambda → f increases.
    lo, hi = 0.0001, 100.0
    f_lo, f_hi = f(lo), f(hi)

    if f_lo > 0:
        # Even maximum fabric conduction gives a slower tau than requested — clamp small
        r_ext_new = lo
    elif f_hi < 0:
        # Even perfect fabric insulation can't achieve the requested tau — clamp large
        r_ext_new = hi
    else:
        for _ in range(64):
            mid = (lo + hi) / 2.0
            if f(mid) < 0:
                lo = mid
            else:
                hi = mid
        r_ext_new = (lo + hi) / 2.0

    out = dict(user_input)
    out[CONF_R_EXT] = round(r_ext_new, 6)
    out.pop(CONF_CALIB_A,   None)
    out.pop(CONF_CALIB_B,   None)
    out.pop(CONF_CALIB_TAU, None)
    return out


# ---------------------------------------------------------------------------
# Options schemas
# ---------------------------------------------------------------------------

def _simple_options_schema(cfg) -> vol.Schema:
    return vol.Schema({
        vol.Required(CONF_HEATER_POWER, default=_g(cfg, CONF_HEATER_POWER, DEFAULT_HEATER_POWER)): vol.All(vol.Coerce(float), vol.Range(min=100, max=50000)),
        vol.Required(CONF_HEAT_LOSS_COEFF, default=_g(cfg, CONF_HEAT_LOSS_COEFF, DEFAULT_HEAT_LOSS_COEFF)): vol.All(vol.Coerce(float), vol.Range(min=0.001, max=2000)),
        vol.Required(CONF_THERMAL_MASS, default=_g(cfg, CONF_THERMAL_MASS, DEFAULT_THERMAL_MASS)): vol.All(vol.Coerce(float), vol.Range(min=100, max=20_000_000)),
        vol.Required(CONF_THERMAL_INERTIA, default=_g(cfg, CONF_THERMAL_INERTIA, DEFAULT_THERMAL_INERTIA)): vol.All(vol.Coerce(float), vol.Range(min=0, max=7200)),
        vol.Required(CONF_EXTERNAL_TEMP_FIXED, default=_g(cfg, CONF_EXTERNAL_TEMP_FIXED, DEFAULT_EXTERNAL_TEMP_FIXED)): vol.Coerce(float),
        vol.Required(CONF_UPDATE_INTERVAL, default=_g(cfg, CONF_UPDATE_INTERVAL, DEFAULT_UPDATE_INTERVAL)): vol.All(vol.Coerce(int), vol.Range(min=1, max=300)),
        vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
        vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
        vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
    })


def _r2c2_options_schema(cfg) -> vol.Schema:
    return vol.Schema({
        vol.Required(CONF_HEATER_POWER_R2C2, default=_g(cfg, CONF_HEATER_POWER_R2C2, DEFAULT_HEATER_POWER_R2C2)): vol.All(vol.Coerce(float), vol.Range(min=100, max=50000)),
        vol.Required(CONF_C_AIR, default=_g(cfg, CONF_C_AIR, DEFAULT_C_AIR)): vol.All(vol.Coerce(float), vol.Range(min=1000, max=20_000_000)),
        vol.Required(CONF_C_FABRIC, default=_g(cfg, CONF_C_FABRIC, DEFAULT_C_FABRIC)): vol.All(vol.Coerce(float), vol.Range(min=10000, max=50_000_000)),
        vol.Required(CONF_R_EXT, default=_g(cfg, CONF_R_EXT, DEFAULT_R_EXT)): vol.All(vol.Coerce(float), vol.Range(min=0.0001, max=50.0)),
        vol.Required(CONF_R_INF, default=_g(cfg, CONF_R_INF, DEFAULT_R_INF)): vol.All(vol.Coerce(float), vol.Range(min=0.001, max=10.0)),
        vol.Required(CONF_WINDOW_AREA, default=_g(cfg, CONF_WINDOW_AREA, DEFAULT_WINDOW_AREA)): vol.All(vol.Coerce(float), vol.Range(min=0, max=100)),
        vol.Required(CONF_SOLAR_FIXED, default=_g(cfg, CONF_SOLAR_FIXED, DEFAULT_SOLAR_FIXED)): vol.All(vol.Coerce(float), vol.Range(min=0, max=1500)),
        vol.Required(CONF_EXTERNAL_TEMP_FIXED, default=_g(cfg, CONF_EXTERNAL_TEMP_FIXED, DEFAULT_EXTERNAL_TEMP_FIXED)): vol.Coerce(float),
        vol.Required(CONF_UPDATE_INTERVAL, default=_g(cfg, CONF_UPDATE_INTERVAL, DEFAULT_UPDATE_INTERVAL)): vol.All(vol.Coerce(int), vol.Range(min=1, max=300)),
        # tau-only calibration: finds r_ext from the long-term cooling constant
        vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
        vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
    })


def _radiator_options_schema(cfg) -> vol.Schema:
    return vol.Schema({
        vol.Required(CONF_FLOW_TEMP, default=_g(cfg, CONF_FLOW_TEMP, DEFAULT_FLOW_TEMP)): vol.All(vol.Coerce(float), vol.Range(min=20, max=90)),
        vol.Required(CONF_C_RAD, default=_g(cfg, CONF_C_RAD, DEFAULT_C_RAD)): vol.All(vol.Coerce(float), vol.Range(min=500, max=100_000)),
        vol.Required(CONF_C_ROOM_RAD, default=_g(cfg, CONF_C_ROOM_RAD, DEFAULT_C_ROOM_RAD)): vol.All(vol.Coerce(float), vol.Range(min=1000, max=20_000_000)),
        vol.Required(CONF_K_RAD, default=_g(cfg, CONF_K_RAD, DEFAULT_K_RAD)): vol.All(vol.Coerce(float), vol.Range(min=0.1, max=500)),
        vol.Required(CONF_FLOW_RATE_MAX, default=_g(cfg, CONF_FLOW_RATE_MAX, DEFAULT_FLOW_RATE_MAX)): vol.All(vol.Coerce(float), vol.Range(min=0.001, max=1.0)),
        vol.Required(CONF_HEAT_LOSS_COEFF_RAD, default=_g(cfg, CONF_HEAT_LOSS_COEFF_RAD, DEFAULT_HEAT_LOSS_COEFF_RAD)): vol.All(vol.Coerce(float), vol.Range(min=0.001, max=2000)),
        vol.Required(CONF_PIPE_DELAY, default=_g(cfg, CONF_PIPE_DELAY, DEFAULT_PIPE_DELAY)): vol.All(vol.Coerce(float), vol.Range(min=0, max=600)),
        vol.Required(CONF_EXTERNAL_TEMP_FIXED, default=_g(cfg, CONF_EXTERNAL_TEMP_FIXED, DEFAULT_EXTERNAL_TEMP_FIXED)): vol.Coerce(float),
        vol.Required(CONF_UPDATE_INTERVAL, default=_g(cfg, CONF_UPDATE_INTERVAL, DEFAULT_UPDATE_INTERVAL)): vol.All(vol.Coerce(int), vol.Range(min=1, max=300)),
        # --- Behavioural calibration (optional) ----------------------------
        # Fill in a and tau (or b) to back-calculate c_room and K_loss from
        # observed room behaviour.  Leave at 0 to keep manual physical values.
        # On save, these are consumed and the result written into c_room / K_loss.
        vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
        vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
        vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
    })


def _r2c2_radiator_options_schema(cfg) -> vol.Schema:
    return vol.Schema({
        # Radiator
        vol.Required(CONF_FLOW_TEMP, default=_g(cfg, CONF_FLOW_TEMP, DEFAULT_FLOW_TEMP)): vol.All(vol.Coerce(float), vol.Range(min=20, max=90)),
        vol.Required(CONF_C_RAD, default=_g(cfg, CONF_C_RAD, DEFAULT_C_RAD)): vol.All(vol.Coerce(float), vol.Range(min=500, max=100_000)),
        vol.Required(CONF_K_RAD, default=_g(cfg, CONF_K_RAD, DEFAULT_K_RAD)): vol.All(vol.Coerce(float), vol.Range(min=0.1, max=500)),
        vol.Required(CONF_RAD_CONVECTIVE_FRACTION, default=_g(cfg, CONF_RAD_CONVECTIVE_FRACTION, DEFAULT_RAD_CONVECTIVE_FRACTION)): vol.All(vol.Coerce(float), vol.Range(min=0.1, max=1.0)),
        vol.Required(CONF_FLOW_RATE_MAX, default=_g(cfg, CONF_FLOW_RATE_MAX, DEFAULT_FLOW_RATE_MAX)): vol.All(vol.Coerce(float), vol.Range(min=0.001, max=1.0)),
        vol.Required(CONF_PIPE_DELAY, default=_g(cfg, CONF_PIPE_DELAY, DEFAULT_PIPE_DELAY)): vol.All(vol.Coerce(float), vol.Range(min=0, max=600)),
        # Room
        vol.Required(CONF_C_AIR, default=_g(cfg, CONF_C_AIR, DEFAULT_C_AIR)): vol.All(vol.Coerce(float), vol.Range(min=1000, max=20_000_000)),
        vol.Required(CONF_C_FABRIC, default=_g(cfg, CONF_C_FABRIC, DEFAULT_C_FABRIC)): vol.All(vol.Coerce(float), vol.Range(min=10000, max=50_000_000)),
        vol.Required(CONF_R_EXT, default=_g(cfg, CONF_R_EXT, DEFAULT_R_EXT)): vol.All(vol.Coerce(float), vol.Range(min=0.0001, max=50.0)),
        vol.Required(CONF_R_INF, default=_g(cfg, CONF_R_INF, DEFAULT_R_INF)): vol.All(vol.Coerce(float), vol.Range(min=0.001, max=10.0)),
        # Solar
        vol.Required(CONF_WINDOW_AREA, default=_g(cfg, CONF_WINDOW_AREA, DEFAULT_WINDOW_AREA)): vol.All(vol.Coerce(float), vol.Range(min=0, max=100)),
        vol.Required(CONF_SOLAR_FIXED, default=_g(cfg, CONF_SOLAR_FIXED, DEFAULT_SOLAR_FIXED)): vol.All(vol.Coerce(float), vol.Range(min=0, max=1500)),
        # Shared
        vol.Required(CONF_EXTERNAL_TEMP_FIXED, default=_g(cfg, CONF_EXTERNAL_TEMP_FIXED, DEFAULT_EXTERNAL_TEMP_FIXED)): vol.Coerce(float),
        vol.Required(CONF_UPDATE_INTERVAL, default=_g(cfg, CONF_UPDATE_INTERVAL, DEFAULT_UPDATE_INTERVAL)): vol.All(vol.Coerce(int), vol.Range(min=1, max=300)),
        # --- Behavioural calibration (optional) ----------------------------
        vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
        vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
        vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
    })
