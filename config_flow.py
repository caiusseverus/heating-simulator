"""Config flow for Heating Simulator.

Setup flow  (2 steps):
  1. user          — name, model type, control mode, initial/external temperature, update interval
  2. model params  — model-specific physics parameters (step name depends on model type)

Options flow (menu-driven, all sections re-editable):
  Menu → general        — control mode, temperatures, update interval
       → thermal_model  — model-specific physics (same schemas as setup step 2)
       → disturbances   — occupancy, external temperature profile, wind, rain
       → sensor         — sensor noise/bias/lag/quantisation
"""

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
    # sensor imperfection suite
    CONF_SENSOR_NOISE_STD_DEV,
    CONF_SENSOR_BIAS,
    CONF_SENSOR_QUANTISATION,
    CONF_SENSOR_LAG_TAU,
    CONF_SENSOR_UPDATE_RATE,
    DEFAULT_SENSOR_NOISE_STD_DEV,
    DEFAULT_SENSOR_BIAS,
    DEFAULT_SENSOR_QUANTISATION,
    DEFAULT_SENSOR_LAG_TAU,
    DEFAULT_SENSOR_UPDATE_RATE,
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
    # external temp profile
    CONF_EXT_TEMP_PROFILE_ENABLED, CONF_EXT_TEMP_BASE, CONF_EXT_TEMP_AMPLITUDE,
    CONF_EXT_TEMP_MIN_HOUR, CONF_EXT_TEMP_MAX_HOUR,
    DEFAULT_EXT_TEMP_PROFILE_ENABLED, DEFAULT_EXT_TEMP_BASE, DEFAULT_EXT_TEMP_AMPLITUDE,
    DEFAULT_EXT_TEMP_MIN_HOUR, DEFAULT_EXT_TEMP_MAX_HOUR,
    # occupancy
    CONF_OCCUPANCY_ENABLED, CONF_OCCUPANCY_MAX_OCCUPANTS, CONF_OCCUPANCY_COOKING_POWER,
    CONF_OCCUPANCY_COOKING_DURATION, CONF_OCCUPANCY_COOKING_EVENTS_PER_DAY, CONF_OCCUPANCY_SEED,
    DEFAULT_OCCUPANCY_ENABLED, DEFAULT_OCCUPANCY_MAX_OCCUPANTS, DEFAULT_OCCUPANCY_COOKING_POWER,
    DEFAULT_OCCUPANCY_COOKING_DURATION, DEFAULT_OCCUPANCY_COOKING_EVENTS_PER_DAY, DEFAULT_OCCUPANCY_SEED,
    # weather
    CONF_WIND_SPEED, CONF_WIND_COEFFICIENT, CONF_RAIN_INTENSITY, CONF_RAIN_MOISTURE_FACTOR,
    DEFAULT_WIND_SPEED, DEFAULT_WIND_COEFFICIENT, DEFAULT_RAIN_INTENSITY, DEFAULT_RAIN_MOISTURE_FACTOR,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _g(cfg: dict, key: str, default):
    """Read from merged config; fall back to default if absent."""
    return cfg.get(key, default)


# ---------------------------------------------------------------------------
# Setup step 1 — identity and model selection
# ---------------------------------------------------------------------------

STEP_SETUP_SCHEMA = vol.Schema({
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
# Model parameter schemas — initial setup (no current-value defaults needed)
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
    vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
    vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
    vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
})

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
    vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
    vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
})

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
    vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
    vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
    vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
})

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
    vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
    vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
    vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
})


# ---------------------------------------------------------------------------
# Options schemas — pre-populated from current config
# ---------------------------------------------------------------------------

def _general_options_schema(cfg: dict) -> vol.Schema:
    """General settings: control mode, temperatures, update interval."""
    return vol.Schema({
        vol.Required(CONF_CONTROL_MODE, default=_g(cfg, CONF_CONTROL_MODE, CONTROL_MODE_LINEAR)): vol.In(
            [CONTROL_MODE_LINEAR, CONTROL_MODE_PWM]
        ),
        vol.Optional(CONF_EXTERNAL_TEMP, default=_g(cfg, CONF_EXTERNAL_TEMP, "")): str,
        vol.Required(CONF_EXTERNAL_TEMP_FIXED, default=_g(cfg, CONF_EXTERNAL_TEMP_FIXED, DEFAULT_EXTERNAL_TEMP_FIXED)): vol.Coerce(float),
        vol.Required(CONF_UPDATE_INTERVAL, default=_g(cfg, CONF_UPDATE_INTERVAL, DEFAULT_UPDATE_INTERVAL)): vol.All(
            vol.Coerce(int), vol.Range(min=1, max=300)
        ),
    })


def _simple_model_schema(cfg: dict) -> vol.Schema:
    return vol.Schema({
        vol.Required(CONF_HEATER_POWER, default=_g(cfg, CONF_HEATER_POWER, DEFAULT_HEATER_POWER)): vol.All(
            vol.Coerce(float), vol.Range(min=100, max=50000)
        ),
        vol.Required(CONF_HEAT_LOSS_COEFF, default=_g(cfg, CONF_HEAT_LOSS_COEFF, DEFAULT_HEAT_LOSS_COEFF)): vol.All(
            vol.Coerce(float), vol.Range(min=0.001, max=2000)
        ),
        vol.Required(CONF_THERMAL_MASS, default=_g(cfg, CONF_THERMAL_MASS, DEFAULT_THERMAL_MASS)): vol.All(
            vol.Coerce(float), vol.Range(min=100, max=20_000_000)
        ),
        vol.Required(CONF_THERMAL_INERTIA, default=_g(cfg, CONF_THERMAL_INERTIA, DEFAULT_THERMAL_INERTIA)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=7200)
        ),
        vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
        vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
        vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
    })


def _r2c2_model_schema(cfg: dict) -> vol.Schema:
    return vol.Schema({
        vol.Required(CONF_HEATER_POWER_R2C2, default=_g(cfg, CONF_HEATER_POWER_R2C2, DEFAULT_HEATER_POWER_R2C2)): vol.All(
            vol.Coerce(float), vol.Range(min=100, max=50000)
        ),
        vol.Required(CONF_C_AIR, default=_g(cfg, CONF_C_AIR, DEFAULT_C_AIR)): vol.All(
            vol.Coerce(float), vol.Range(min=1000, max=20_000_000)
        ),
        vol.Required(CONF_C_FABRIC, default=_g(cfg, CONF_C_FABRIC, DEFAULT_C_FABRIC)): vol.All(
            vol.Coerce(float), vol.Range(min=10000, max=50_000_000)
        ),
        vol.Required(CONF_R_EXT, default=_g(cfg, CONF_R_EXT, DEFAULT_R_EXT)): vol.All(
            vol.Coerce(float), vol.Range(min=0.0001, max=50.0)
        ),
        vol.Required(CONF_R_INF, default=_g(cfg, CONF_R_INF, DEFAULT_R_INF)): vol.All(
            vol.Coerce(float), vol.Range(min=0.001, max=10.0)
        ),
        vol.Required(CONF_WINDOW_AREA, default=_g(cfg, CONF_WINDOW_AREA, DEFAULT_WINDOW_AREA)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=100)
        ),
        vol.Required(CONF_SOLAR_FIXED, default=_g(cfg, CONF_SOLAR_FIXED, DEFAULT_SOLAR_FIXED)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=1500)
        ),
        vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
        vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
    })


def _radiator_model_schema(cfg: dict) -> vol.Schema:
    return vol.Schema({
        vol.Required(CONF_FLOW_TEMP, default=_g(cfg, CONF_FLOW_TEMP, DEFAULT_FLOW_TEMP)): vol.All(
            vol.Coerce(float), vol.Range(min=20, max=90)
        ),
        vol.Optional(CONF_FLOW_TEMP_ENTITY, default=_g(cfg, CONF_FLOW_TEMP_ENTITY, "")): str,
        vol.Required(CONF_C_RAD, default=_g(cfg, CONF_C_RAD, DEFAULT_C_RAD)): vol.All(
            vol.Coerce(float), vol.Range(min=500, max=100_000)
        ),
        vol.Required(CONF_C_ROOM_RAD, default=_g(cfg, CONF_C_ROOM_RAD, DEFAULT_C_ROOM_RAD)): vol.All(
            vol.Coerce(float), vol.Range(min=1000, max=20_000_000)
        ),
        vol.Required(CONF_K_RAD, default=_g(cfg, CONF_K_RAD, DEFAULT_K_RAD)): vol.All(
            vol.Coerce(float), vol.Range(min=0.1, max=500)
        ),
        vol.Required(CONF_FLOW_RATE_MAX, default=_g(cfg, CONF_FLOW_RATE_MAX, DEFAULT_FLOW_RATE_MAX)): vol.All(
            vol.Coerce(float), vol.Range(min=0.001, max=1.0)
        ),
        vol.Required(CONF_HEAT_LOSS_COEFF_RAD, default=_g(cfg, CONF_HEAT_LOSS_COEFF_RAD, DEFAULT_HEAT_LOSS_COEFF_RAD)): vol.All(
            vol.Coerce(float), vol.Range(min=0.001, max=2000)
        ),
        vol.Required(CONF_PIPE_DELAY, default=_g(cfg, CONF_PIPE_DELAY, DEFAULT_PIPE_DELAY)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=600)
        ),
        vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
        vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
        vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
    })


def _r2c2_radiator_model_schema(cfg: dict) -> vol.Schema:
    return vol.Schema({
        # Radiator
        vol.Required(CONF_FLOW_TEMP, default=_g(cfg, CONF_FLOW_TEMP, DEFAULT_FLOW_TEMP)): vol.All(
            vol.Coerce(float), vol.Range(min=20, max=90)
        ),
        vol.Optional(CONF_FLOW_TEMP_ENTITY, default=_g(cfg, CONF_FLOW_TEMP_ENTITY, "")): str,
        vol.Required(CONF_C_RAD, default=_g(cfg, CONF_C_RAD, DEFAULT_C_RAD)): vol.All(
            vol.Coerce(float), vol.Range(min=500, max=100_000)
        ),
        vol.Required(CONF_K_RAD, default=_g(cfg, CONF_K_RAD, DEFAULT_K_RAD)): vol.All(
            vol.Coerce(float), vol.Range(min=0.1, max=500)
        ),
        vol.Required(CONF_RAD_EXPONENT, default=_g(cfg, CONF_RAD_EXPONENT, DEFAULT_RAD_EXPONENT)): vol.All(
            vol.Coerce(float), vol.Range(min=1.0, max=2.0)
        ),
        vol.Required(CONF_RAD_CONVECTIVE_FRACTION, default=_g(cfg, CONF_RAD_CONVECTIVE_FRACTION, DEFAULT_RAD_CONVECTIVE_FRACTION)): vol.All(
            vol.Coerce(float), vol.Range(min=0.1, max=1.0)
        ),
        vol.Required(CONF_FLOW_RATE_MAX, default=_g(cfg, CONF_FLOW_RATE_MAX, DEFAULT_FLOW_RATE_MAX)): vol.All(
            vol.Coerce(float), vol.Range(min=0.001, max=1.0)
        ),
        vol.Required(CONF_PIPE_DELAY, default=_g(cfg, CONF_PIPE_DELAY, DEFAULT_PIPE_DELAY)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=600)
        ),
        # Room (R2C2)
        vol.Required(CONF_C_AIR, default=_g(cfg, CONF_C_AIR, DEFAULT_C_AIR)): vol.All(
            vol.Coerce(float), vol.Range(min=1000, max=2_000_000)
        ),
        vol.Required(CONF_C_FABRIC, default=_g(cfg, CONF_C_FABRIC, DEFAULT_C_FABRIC)): vol.All(
            vol.Coerce(float), vol.Range(min=10000, max=50_000_000)
        ),
        vol.Required(CONF_R_FABRIC, default=_g(cfg, CONF_R_FABRIC, DEFAULT_R_FABRIC)): vol.All(
            vol.Coerce(float), vol.Range(min=0.0001, max=1.0)
        ),
        vol.Required(CONF_R_EXT, default=_g(cfg, CONF_R_EXT, DEFAULT_R_EXT)): vol.All(
            vol.Coerce(float), vol.Range(min=0.0001, max=50.0)
        ),
        vol.Required(CONF_R_INF, default=_g(cfg, CONF_R_INF, DEFAULT_R_INF)): vol.All(
            vol.Coerce(float), vol.Range(min=0.001, max=10.0)
        ),
        # Solar
        vol.Required(CONF_WINDOW_AREA, default=_g(cfg, CONF_WINDOW_AREA, DEFAULT_WINDOW_AREA)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=100)
        ),
        vol.Required(CONF_WINDOW_TRANSMITTANCE, default=_g(cfg, CONF_WINDOW_TRANSMITTANCE, DEFAULT_WINDOW_TRANSMITTANCE)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=1)
        ),
        vol.Optional(CONF_SOLAR_ENTITY, default=_g(cfg, CONF_SOLAR_ENTITY, "")): str,
        vol.Required(CONF_SOLAR_FIXED, default=_g(cfg, CONF_SOLAR_FIXED, DEFAULT_SOLAR_FIXED)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=1500)
        ),
        vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
        vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
        vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
    })


def _sensor_options_schema(cfg: dict) -> vol.Schema:
    """Sensor imperfection parameters (noise, bias, quantisation, lag)."""
    return vol.Schema({
        vol.Optional(CONF_SENSOR_NOISE_STD_DEV, default=_g(cfg, CONF_SENSOR_NOISE_STD_DEV, DEFAULT_SENSOR_NOISE_STD_DEV)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=10)
        ),
        vol.Optional(CONF_SENSOR_BIAS, default=_g(cfg, CONF_SENSOR_BIAS, DEFAULT_SENSOR_BIAS)): vol.All(
            vol.Coerce(float), vol.Range(min=-20, max=20)
        ),
        vol.Optional(CONF_SENSOR_QUANTISATION, default=_g(cfg, CONF_SENSOR_QUANTISATION, DEFAULT_SENSOR_QUANTISATION)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=5)
        ),
        vol.Optional(CONF_SENSOR_LAG_TAU, default=_g(cfg, CONF_SENSOR_LAG_TAU, DEFAULT_SENSOR_LAG_TAU)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=3600)
        ),
        vol.Optional(CONF_SENSOR_UPDATE_RATE, default=_g(cfg, CONF_SENSOR_UPDATE_RATE, DEFAULT_SENSOR_UPDATE_RATE)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=3600)
        ),
    })


def _disturbances_options_schema(cfg: dict) -> vol.Schema:
    """Disturbance profiles: occupancy, external temperature, wind, rain."""
    return vol.Schema({
        # External temperature daily profile
        vol.Optional(CONF_EXT_TEMP_PROFILE_ENABLED, default=bool(_g(cfg, CONF_EXT_TEMP_PROFILE_ENABLED, DEFAULT_EXT_TEMP_PROFILE_ENABLED))): bool,
        vol.Optional(CONF_EXT_TEMP_BASE, default=float(_g(cfg, CONF_EXT_TEMP_BASE, DEFAULT_EXT_TEMP_BASE))): vol.All(
            vol.Coerce(float), vol.Range(min=-30, max=40)
        ),
        vol.Optional(CONF_EXT_TEMP_AMPLITUDE, default=float(_g(cfg, CONF_EXT_TEMP_AMPLITUDE, DEFAULT_EXT_TEMP_AMPLITUDE))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=20)
        ),
        vol.Optional(CONF_EXT_TEMP_MIN_HOUR, default=float(_g(cfg, CONF_EXT_TEMP_MIN_HOUR, DEFAULT_EXT_TEMP_MIN_HOUR))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=24)
        ),
        vol.Optional(CONF_EXT_TEMP_MAX_HOUR, default=float(_g(cfg, CONF_EXT_TEMP_MAX_HOUR, DEFAULT_EXT_TEMP_MAX_HOUR))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=24)
        ),
        # Occupancy and internal heat gain
        vol.Optional(CONF_OCCUPANCY_ENABLED, default=bool(_g(cfg, CONF_OCCUPANCY_ENABLED, DEFAULT_OCCUPANCY_ENABLED))): bool,
        vol.Optional(CONF_OCCUPANCY_MAX_OCCUPANTS, default=int(_g(cfg, CONF_OCCUPANCY_MAX_OCCUPANTS, DEFAULT_OCCUPANCY_MAX_OCCUPANTS))): vol.All(
            vol.Coerce(int), vol.Range(min=0, max=20)
        ),
        vol.Optional(CONF_OCCUPANCY_COOKING_POWER, default=float(_g(cfg, CONF_OCCUPANCY_COOKING_POWER, DEFAULT_OCCUPANCY_COOKING_POWER))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=10000)
        ),
        vol.Optional(CONF_OCCUPANCY_COOKING_DURATION, default=float(_g(cfg, CONF_OCCUPANCY_COOKING_DURATION, DEFAULT_OCCUPANCY_COOKING_DURATION))): vol.All(
            vol.Coerce(float), vol.Range(min=60, max=7200)
        ),
        vol.Optional(CONF_OCCUPANCY_COOKING_EVENTS_PER_DAY, default=float(_g(cfg, CONF_OCCUPANCY_COOKING_EVENTS_PER_DAY, DEFAULT_OCCUPANCY_COOKING_EVENTS_PER_DAY))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=10)
        ),
        vol.Optional(CONF_OCCUPANCY_SEED, default=int(_g(cfg, CONF_OCCUPANCY_SEED, DEFAULT_OCCUPANCY_SEED))): vol.All(
            vol.Coerce(int), vol.Range(min=0, max=999999)
        ),
        # Wind effect
        vol.Optional(CONF_WIND_SPEED, default=float(_g(cfg, CONF_WIND_SPEED, DEFAULT_WIND_SPEED))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=50)
        ),
        vol.Optional(CONF_WIND_COEFFICIENT, default=float(_g(cfg, CONF_WIND_COEFFICIENT, DEFAULT_WIND_COEFFICIENT))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=0.5)
        ),
        # Rain / moisture effect
        vol.Optional(CONF_RAIN_INTENSITY, default=float(_g(cfg, CONF_RAIN_INTENSITY, DEFAULT_RAIN_INTENSITY))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=1)
        ),
        vol.Optional(CONF_RAIN_MOISTURE_FACTOR, default=float(_g(cfg, CONF_RAIN_MOISTURE_FACTOR, DEFAULT_RAIN_MOISTURE_FACTOR))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=0.5)
        ),
    })


def _thermal_model_schema_for_cfg(cfg: dict) -> vol.Schema:
    """Return the correct model schema pre-populated from cfg."""
    model_type = cfg.get(CONF_MODEL_TYPE, MODEL_SIMPLE)
    if model_type == MODEL_R2C2:
        return _r2c2_model_schema(cfg)
    elif model_type == MODEL_RADIATOR:
        return _radiator_model_schema(cfg)
    elif model_type == MODEL_R2C2_RADIATOR:
        return _r2c2_radiator_model_schema(cfg)
    return _simple_model_schema(cfg)


# ---------------------------------------------------------------------------
# Config flow (initial setup)
# ---------------------------------------------------------------------------

class HeatingSimulatorConfigFlow(config_entries.ConfigFlow, domain=DOMAIN):
    """Two-step setup: name/model selection → model physics parameters."""

    VERSION = 2

    def __init__(self) -> None:
        super().__init__()
        self._setup_data: dict[str, Any] = {}

    async def async_step_user(self, user_input=None) -> FlowResult:
        """Step 1: name, model type, control mode, temperatures, update interval."""
        if user_input is not None:
            self._setup_data = dict(user_input)
            model_type = user_input.get(CONF_MODEL_TYPE, MODEL_SIMPLE)
            # Route to the appropriate model-params step
            if model_type == MODEL_R2C2:
                return await self.async_step_model_r2c2()
            elif model_type == MODEL_RADIATOR:
                return await self.async_step_model_radiator()
            elif model_type == MODEL_R2C2_RADIATOR:
                return await self.async_step_model_r2c2_radiator()
            return await self.async_step_model_simple()

        return self.async_show_form(step_id="user", data_schema=STEP_SETUP_SCHEMA)

    async def async_step_model_simple(self, user_input=None) -> FlowResult:
        """Step 2a: Simple (R1C1) model parameters."""
        if user_input is not None:
            name = self._setup_data.get("name", "Heating Simulator")
            merged = {**self._setup_data, **user_input}
            merged = apply_calibration_simple(merged, merged)
            return self.async_create_entry(title=name, data=merged)
        return self.async_show_form(step_id="model_simple", data_schema=STEP_SIMPLE_SCHEMA)

    async def async_step_model_r2c2(self, user_input=None) -> FlowResult:
        """Step 2b: R2C2 model parameters."""
        if user_input is not None:
            name = self._setup_data.get("name", "Heating Simulator")
            merged = {**self._setup_data, **user_input}
            merged = apply_calibration_r2c2(merged, merged)
            return self.async_create_entry(title=name, data=merged)
        return self.async_show_form(step_id="model_r2c2", data_schema=STEP_R2C2_SCHEMA)

    async def async_step_model_radiator(self, user_input=None) -> FlowResult:
        """Step 2c: Wet radiator model parameters."""
        if user_input is not None:
            name = self._setup_data.get("name", "Heating Simulator")
            merged = {**self._setup_data, **user_input}
            merged = apply_calibration_radiator(merged, merged)
            return self.async_create_entry(title=name, data=merged)
        return self.async_show_form(step_id="model_radiator", data_schema=STEP_RADIATOR_SCHEMA)

    async def async_step_model_r2c2_radiator(self, user_input=None) -> FlowResult:
        """Step 2d: R2C2 + radiator model parameters."""
        if user_input is not None:
            name = self._setup_data.get("name", "Heating Simulator")
            merged = {**self._setup_data, **user_input}
            merged = apply_calibration_r2c2_radiator(merged, merged)
            return self.async_create_entry(title=name, data=merged)
        return self.async_show_form(step_id="model_r2c2_radiator", data_schema=STEP_R2C2_RADIATOR_SCHEMA)

    @staticmethod
    @callback
    def async_get_options_flow(config_entry):
        return HeatingSimulatorOptionsFlow(config_entry)


# ---------------------------------------------------------------------------
# Options flow (menu-driven, all sections re-configurable)
# ---------------------------------------------------------------------------

class HeatingSimulatorOptionsFlow(config_entries.OptionsFlow):
    """
    Menu-based options flow with four sub-pages:

      General        — control mode, external temperature source, update interval
      Thermal Model  — model-specific physics parameters (calibration included)
      Disturbances   — occupancy, external temperature profile, wind, rain
      Sensor         — sensor noise, bias, quantisation, lag

    All accumulated changes are written atomically when the user saves any
    sub-page and returns to the menu (or explicitly finishes).  Each sub-page
    save merges its changes into _pending and commits immediately so the user
    can edit one section at a time without losing earlier changes.
    """

    def __init__(self, config_entry: config_entries.ConfigEntry) -> None:
        super().__init__()
        self._config_entry = config_entry

    @property
    def _cfg(self) -> dict:
        """Merged view of stored data + options (options take precedence)."""
        return {**self._config_entry.data, **self._config_entry.options}

    async def async_step_init(self, user_input=None) -> FlowResult:
        """Entry point: show the options menu."""
        return self.async_show_menu(
            step_id="init",
            menu_options=["general", "thermal_model", "disturbances", "sensor"],
        )

    # -- General settings --------------------------------------------------

    async def async_step_general(self, user_input=None) -> FlowResult:
        """Control mode, external temperature, update interval."""
        cfg = self._cfg
        if user_input is not None:
            return self._save_and_return(user_input)
        return self.async_show_form(
            step_id="general",
            data_schema=_general_options_schema(cfg),
        )

    # -- Thermal model parameters ------------------------------------------

    async def async_step_thermal_model(self, user_input=None) -> FlowResult:
        """Model physics parameters, pre-populated from current config."""
        cfg = self._cfg
        if user_input is not None:
            model_type = cfg.get(CONF_MODEL_TYPE, MODEL_SIMPLE)
            if model_type == MODEL_SIMPLE:
                user_input = apply_calibration_simple(user_input, cfg)
            elif model_type == MODEL_R2C2:
                user_input = apply_calibration_r2c2(user_input, cfg)
            elif model_type == MODEL_RADIATOR:
                user_input = apply_calibration_radiator(user_input, cfg)
            elif model_type == MODEL_R2C2_RADIATOR:
                user_input = apply_calibration_r2c2_radiator(user_input, cfg)
            return self._save_and_return(user_input)
        return self.async_show_form(
            step_id="thermal_model",
            data_schema=_thermal_model_schema_for_cfg(cfg),
        )

    # -- Disturbances ------------------------------------------------------

    async def async_step_disturbances(self, user_input=None) -> FlowResult:
        """Occupancy, external temperature profile, wind, rain."""
        cfg = self._cfg
        if user_input is not None:
            return self._save_and_return(user_input)
        return self.async_show_form(
            step_id="disturbances",
            data_schema=_disturbances_options_schema(cfg),
        )

    # -- Sensor degradation ------------------------------------------------

    async def async_step_sensor(self, user_input=None) -> FlowResult:
        """Sensor noise, bias, quantisation, and lag parameters."""
        cfg = self._cfg
        if user_input is not None:
            return self._save_and_return(user_input)
        return self.async_show_form(
            step_id="sensor",
            data_schema=_sensor_options_schema(cfg),
        )

    # -- Helpers -----------------------------------------------------------

    def _save_and_return(self, new_values: dict) -> FlowResult:
        """
        Merge new_values into the existing options and commit.

        We merge rather than replace so that editing one sub-page does not
        erase changes made to other sub-pages in previous saves.
        """
        updated = {**self._config_entry.options, **new_values}
        return self.async_create_entry(title="", data=updated)


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
    Return the radiator steady-state heat output Q_out (W).

    At steady state:  flow_rate × C_water × (T_flow − T_rad) = k_rad × (T_rad − T_room)^n
    Solved by bisection.  Falls back to full-ΔT estimate if no root is found.
    """
    def residual(T_r: float) -> float:
        return flow_rate * _C_WATER * (T_flow - T_r) - k_rad * (T_r - T_room) ** n

    lo, hi = T_room + 0.01, T_flow - 0.01
    if lo >= hi or residual(lo) * residual(hi) > 0:
        return k_rad * (T_flow - T_room) ** n

    for _ in range(60):
        mid = (lo + hi) / 2
        if residual(mid) > 0:
            lo = mid
        else:
            hi = mid
    return k_rad * ((lo + hi) / 2 - T_room) ** n


def apply_calibration_simple(user_input: dict, cfg: dict) -> dict:
    """
    Back-calculate thermal_mass and heat_loss_coeff for the Simple model.

    Requires calib_a > 0 and either calib_tau > 0 or calib_b > 0.
    Mapping (P = heater_power):
        C = P / (a / 60)           [a in °C/min]
        K = C / (tau × 60)         [tau in min]
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
    thermal_mass    = P / (a / 60.0)
    heat_loss_coeff = thermal_mass / (tau * 60.0)

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
    time constant tau (or b = 1/tau) via bisection on the slow eigenvalue.

    c_air, c_fabric, r_fabric, r_inf and heater_power are unchanged.
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

    target_lambda = -1.0 / (tau * 60.0)

    def slow_eigenvalue(r_ext: float) -> float:
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

    lo, hi = 0.0001, 100.0
    f_lo, f_hi = f(lo), f(hi)

    if f_lo > 0:
        r_ext_new = lo
    elif f_hi < 0:
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


def apply_calibration_radiator(user_input: dict, cfg: dict) -> dict:
    """
    Back-calculate c_room and heat_loss_coeff for the simple wet-radiator model.

    Requires calib_a > 0 and calib_tau (or calib_b > 0).
    Mapping:
        C_room = Q_out_ss / (a / 60)
        K_loss = C_room / (tau × 60)
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

    k_rad  = float(_g(cfg, CONF_K_RAD,       DEFAULT_K_RAD))
    n      = float(_g(cfg, CONF_RAD_EXPONENT, DEFAULT_RAD_EXPONENT))
    T_flow = float(user_input.get(CONF_FLOW_TEMP, _g(cfg, CONF_FLOW_TEMP, DEFAULT_FLOW_TEMP)))
    flow_rate = float(_g(cfg, CONF_FLOW_RATE_MAX, DEFAULT_FLOW_RATE_MAX))
    T_ext  = float(user_input.get(CONF_EXTERNAL_TEMP_FIXED, _g(cfg, CONF_EXTERNAL_TEMP_FIXED, DEFAULT_EXTERNAL_TEMP_FIXED)))
    T_room = T_ext + 15.0

    Q_out_ss = _q_out_at_operating_point(k_rad, n, T_flow, T_room, flow_rate)
    c_room   = Q_out_ss / (a / 60.0)
    k_loss   = c_room / (tau * 60.0)

    out = dict(user_input)
    out[CONF_C_ROOM_RAD]          = round(c_room, 1)
    out[CONF_HEAT_LOSS_COEFF_RAD] = round(k_loss, 4)
    out.pop(CONF_CALIB_A,   None)
    out.pop(CONF_CALIB_B,   None)
    out.pop(CONF_CALIB_TAU, None)
    return out


def apply_calibration_r2c2_radiator(user_input: dict, cfg: dict) -> dict:
    """
    Back-calculate c_air and r_ext for the R2C2+Radiator model.

    Requires calib_a > 0 and calib_tau (or calib_b > 0).
    Mapping:
        c_air  = Q_conv_ss / (a / 60)    where Q_conv_ss = conv_frac × Q_out_ss
        c_fabric scaled to preserve existing c_fabric/c_air ratio
        r_ext  derived from tau: K_ext = (c_air+c_fabric)/(tau×60) − K_inf
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

    k_rad     = float(_g(cfg, CONF_K_RAD,                  DEFAULT_K_RAD))
    n         = float(_g(cfg, CONF_RAD_EXPONENT,            DEFAULT_RAD_EXPONENT))
    conv_frac = float(_g(cfg, CONF_RAD_CONVECTIVE_FRACTION, DEFAULT_RAD_CONVECTIVE_FRACTION))
    T_flow    = float(user_input.get(CONF_FLOW_TEMP, _g(cfg, CONF_FLOW_TEMP, DEFAULT_FLOW_TEMP)))
    flow_rate = float(_g(cfg, CONF_FLOW_RATE_MAX,           DEFAULT_FLOW_RATE_MAX))
    r_inf     = float(user_input.get(CONF_R_INF, _g(cfg, CONF_R_INF, DEFAULT_R_INF)))
    T_ext     = float(user_input.get(CONF_EXTERNAL_TEMP_FIXED, _g(cfg, CONF_EXTERNAL_TEMP_FIXED, DEFAULT_EXTERNAL_TEMP_FIXED)))
    T_room    = T_ext + 15.0

    c_air_cur = float(_g(cfg, CONF_C_AIR,    DEFAULT_C_AIR))
    c_fab_cur = float(_g(cfg, CONF_C_FABRIC, DEFAULT_C_FABRIC))
    fab_ratio = c_fab_cur / c_air_cur if c_air_cur > 0 else (DEFAULT_C_FABRIC / DEFAULT_C_AIR)

    Q_out_ss  = _q_out_at_operating_point(k_rad, n, T_flow, T_room, flow_rate)
    Q_conv_ss = Q_out_ss * conv_frac
    c_air_new = Q_conv_ss / (a / 60.0)
    c_fab_new = fab_ratio * c_air_new

    K_inf = 1.0 / r_inf
    K_total_needed = (c_air_new + c_fab_new) / (tau * 60.0)
    K_ext = K_total_needed - K_inf
    r_ext_new = max(0.0001, min(50.0, 1.0 / K_ext)) if K_ext > 0 else 50.0

    out = dict(user_input)
    out[CONF_C_AIR]    = round(c_air_new, 1)
    out[CONF_C_FABRIC] = round(c_fab_new, 1)
    out[CONF_R_EXT]    = round(r_ext_new, 6)
    out.pop(CONF_CALIB_A,   None)
    out.pop(CONF_CALIB_B,   None)
    out.pop(CONF_CALIB_TAU, None)
    return out
