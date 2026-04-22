"""Config flow for Heating Simulator.

Setup flow  (2 steps):
  1. user          — name, model type, control mode, initial/external temperature, update interval
  2. model params  — model-specific physics parameters (step name depends on model type)

Options flow (menu-driven, all sections re-editable):
  Menu → general        — control mode, temperatures, update interval
       → thermal_model  — model-specific physics (same schemas as setup step 2)
       → disturbances   — occupancy, external temperature profile, wind, rain
       → sensor         — sensor noise/bias/lag/quantisation/Zigbee reporting
"""

from __future__ import annotations

from typing import Any
import voluptuous as vol

from homeassistant import config_entries
from homeassistant.core import callback
from homeassistant.data_entry_flow import FlowResult
from homeassistant.helpers.schema_config_entry_flow import add_suggested_values_to_schema

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
    CONF_VALVE_CHARACTERISTIC,
    VALVE_CHAR_LINEAR,
    VALVE_CHAR_QUICK_OPENING,
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
    # Zigbee-style reporting
    CONF_SENSOR_MIN_INTERVAL,
    CONF_SENSOR_MAX_INTERVAL,
    CONF_SENSOR_DELTA,
    DEFAULT_SENSOR_MIN_INTERVAL,
    DEFAULT_SENSOR_MAX_INTERVAL,
    DEFAULT_SENSOR_DELTA,
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
    DEFAULT_VALVE_CHARACTERISTIC,
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
# Static setup schemas (used only during initial config flow, no pre-population)
# ---------------------------------------------------------------------------

STEP_USER_SCHEMA = vol.Schema({
    vol.Required("name", default="Heating Simulator"): str,
    vol.Required(CONF_MODEL_TYPE, default=MODEL_SIMPLE): vol.In(
        [MODEL_SIMPLE, MODEL_R2C2, MODEL_RADIATOR, MODEL_R2C2_RADIATOR]
    ),
    vol.Required(CONF_CONTROL_MODE, default=CONTROL_MODE_LINEAR): vol.In(
        [CONTROL_MODE_LINEAR, CONTROL_MODE_PWM]
    ),
    vol.Required(CONF_INITIAL_TEMP, default=DEFAULT_INITIAL_TEMP): vol.All(
        vol.Coerce(float), vol.Range(min=-20, max=40)
    ),
    vol.Optional(CONF_EXTERNAL_TEMP, default=""): str,
    vol.Required(CONF_EXTERNAL_TEMP_FIXED, default=DEFAULT_EXTERNAL_TEMP_FIXED): vol.All(
        vol.Coerce(float), vol.Range(min=-30, max=50)
    ),
    vol.Required(CONF_UPDATE_INTERVAL, default=DEFAULT_UPDATE_INTERVAL): vol.All(
        vol.Coerce(int), vol.Range(min=1, max=300)
    ),
})

STEP_SIMPLE_SCHEMA = vol.Schema({
    vol.Required(CONF_HEATER_POWER, default=DEFAULT_HEATER_POWER): vol.All(
        vol.Coerce(float), vol.Range(min=100, max=50000)
    ),
    vol.Required(CONF_HEAT_LOSS_COEFF, default=DEFAULT_HEAT_LOSS_COEFF): vol.All(
        vol.Coerce(float), vol.Range(min=1, max=2000)
    ),
    vol.Required(CONF_THERMAL_MASS, default=DEFAULT_THERMAL_MASS): vol.All(
        vol.Coerce(float), vol.Range(min=10000, max=50_000_000)
    ),
    vol.Required(CONF_THERMAL_INERTIA, default=DEFAULT_THERMAL_INERTIA): vol.All(
        vol.Coerce(float), vol.Range(min=0, max=3600)
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
    vol.Required(CONF_VALVE_CHARACTERISTIC, default=DEFAULT_VALVE_CHARACTERISTIC): vol.In(
        [VALVE_CHAR_LINEAR, VALVE_CHAR_QUICK_OPENING]
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
    vol.Required(CONF_VALVE_CHARACTERISTIC, default=DEFAULT_VALVE_CHARACTERISTIC): vol.In(
        [VALVE_CHAR_LINEAR, VALVE_CHAR_QUICK_OPENING]
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
        # Intentionally no default from stored config here.
        # Existing values are injected via add_suggested_values_to_schema so
        # the field remains truly clearable in HA options flows.
        vol.Optional(CONF_EXTERNAL_TEMP): str,
        vol.Required(CONF_EXTERNAL_TEMP_FIXED, default=float(_g(cfg, CONF_EXTERNAL_TEMP_FIXED, DEFAULT_EXTERNAL_TEMP_FIXED))): vol.All(
            vol.Coerce(float), vol.Range(min=-30, max=50)
        ),
        vol.Required(CONF_UPDATE_INTERVAL, default=int(_g(cfg, CONF_UPDATE_INTERVAL, DEFAULT_UPDATE_INTERVAL))): vol.All(
            vol.Coerce(int), vol.Range(min=1, max=300)
        ),
    })


def _simple_model_schema(cfg: dict) -> vol.Schema:
    return vol.Schema({
        vol.Required(CONF_HEATER_POWER, default=float(_g(cfg, CONF_HEATER_POWER, DEFAULT_HEATER_POWER))): vol.All(
            vol.Coerce(float), vol.Range(min=100, max=50000)
        ),
        vol.Required(CONF_HEAT_LOSS_COEFF, default=float(_g(cfg, CONF_HEAT_LOSS_COEFF, DEFAULT_HEAT_LOSS_COEFF))): vol.All(
            vol.Coerce(float), vol.Range(min=1, max=2000)
        ),
        vol.Required(CONF_THERMAL_MASS, default=float(_g(cfg, CONF_THERMAL_MASS, DEFAULT_THERMAL_MASS))): vol.All(
            vol.Coerce(float), vol.Range(min=10000, max=50_000_000)
        ),
        vol.Required(CONF_THERMAL_INERTIA, default=float(_g(cfg, CONF_THERMAL_INERTIA, DEFAULT_THERMAL_INERTIA))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=3600)
        ),
        vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
        vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
        vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
    })


def _r2c2_model_schema(cfg: dict) -> vol.Schema:
    return vol.Schema({
        vol.Required(CONF_HEATER_POWER_R2C2, default=float(_g(cfg, CONF_HEATER_POWER_R2C2, DEFAULT_HEATER_POWER_R2C2))): vol.All(
            vol.Coerce(float), vol.Range(min=100, max=50000)
        ),
        vol.Required(CONF_C_AIR, default=float(_g(cfg, CONF_C_AIR, DEFAULT_C_AIR))): vol.All(
            vol.Coerce(float), vol.Range(min=1000, max=2_000_000)
        ),
        vol.Required(CONF_C_FABRIC, default=float(_g(cfg, CONF_C_FABRIC, DEFAULT_C_FABRIC))): vol.All(
            vol.Coerce(float), vol.Range(min=10000, max=50_000_000)
        ),
        vol.Required(CONF_R_FABRIC, default=float(_g(cfg, CONF_R_FABRIC, DEFAULT_R_FABRIC))): vol.All(
            vol.Coerce(float), vol.Range(min=0.0001, max=1.0)
        ),
        vol.Required(CONF_R_EXT, default=float(_g(cfg, CONF_R_EXT, DEFAULT_R_EXT))): vol.All(
            vol.Coerce(float), vol.Range(min=0.0001, max=50.0)
        ),
        vol.Required(CONF_R_INF, default=float(_g(cfg, CONF_R_INF, DEFAULT_R_INF))): vol.All(
            vol.Coerce(float), vol.Range(min=0.001, max=10.0)
        ),
        vol.Required(CONF_WINDOW_AREA, default=float(_g(cfg, CONF_WINDOW_AREA, DEFAULT_WINDOW_AREA))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=100)
        ),
        vol.Required(CONF_WINDOW_TRANSMITTANCE, default=float(_g(cfg, CONF_WINDOW_TRANSMITTANCE, DEFAULT_WINDOW_TRANSMITTANCE))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=1)
        ),
        # Keep clearable entity fields default-free; suggested values are
        # provided in async_step_thermal_model.
        vol.Optional(CONF_SOLAR_ENTITY): str,
        vol.Required(CONF_SOLAR_FIXED, default=float(_g(cfg, CONF_SOLAR_FIXED, DEFAULT_SOLAR_FIXED))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=1500)
        ),
        vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
        vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
    })


def _radiator_model_schema(cfg: dict) -> vol.Schema:
    return vol.Schema({
        vol.Required(CONF_FLOW_TEMP, default=float(_g(cfg, CONF_FLOW_TEMP, DEFAULT_FLOW_TEMP))): vol.All(
            vol.Coerce(float), vol.Range(min=20, max=90)
        ),
        # Keep clearable entity fields default-free; suggested values are
        # provided in async_step_thermal_model.
        vol.Optional(CONF_FLOW_TEMP_ENTITY): str,
        vol.Required(CONF_C_RAD, default=float(_g(cfg, CONF_C_RAD, DEFAULT_C_RAD))): vol.All(
            vol.Coerce(float), vol.Range(min=500, max=100_000)
        ),
        vol.Required(CONF_K_RAD, default=float(_g(cfg, CONF_K_RAD, DEFAULT_K_RAD))): vol.All(
            vol.Coerce(float), vol.Range(min=0.1, max=500)
        ),
        vol.Required(CONF_RAD_EXPONENT, default=float(_g(cfg, CONF_RAD_EXPONENT, DEFAULT_RAD_EXPONENT))): vol.All(
            vol.Coerce(float), vol.Range(min=1.0, max=2.0)
        ),
        vol.Required(CONF_FLOW_RATE_MAX, default=float(_g(cfg, CONF_FLOW_RATE_MAX, DEFAULT_FLOW_RATE_MAX))): vol.All(
            vol.Coerce(float), vol.Range(min=0.001, max=1.0)
        ),
        vol.Required(CONF_HEAT_LOSS_COEFF_RAD, default=float(_g(cfg, CONF_HEAT_LOSS_COEFF_RAD, DEFAULT_HEAT_LOSS_COEFF_RAD))): vol.All(
            vol.Coerce(float), vol.Range(min=0.001, max=2000)
        ),
        vol.Required(CONF_C_ROOM_RAD, default=float(_g(cfg, CONF_C_ROOM_RAD, DEFAULT_C_ROOM_RAD))): vol.All(
            vol.Coerce(float), vol.Range(min=1000, max=20_000_000)
        ),
        vol.Required(CONF_PIPE_DELAY, default=float(_g(cfg, CONF_PIPE_DELAY, DEFAULT_PIPE_DELAY))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=600)
        ),
        vol.Required(CONF_VALVE_CHARACTERISTIC, default=_g(cfg, CONF_VALVE_CHARACTERISTIC, DEFAULT_VALVE_CHARACTERISTIC)): vol.In(
            [VALVE_CHAR_LINEAR, VALVE_CHAR_QUICK_OPENING]
        ),
        vol.Optional(CONF_CALIB_A,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=10)),
        vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
        vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
    })


def _r2c2_radiator_model_schema(cfg: dict) -> vol.Schema:
    return vol.Schema({
        # Radiator
        vol.Required(CONF_FLOW_TEMP, default=float(_g(cfg, CONF_FLOW_TEMP, DEFAULT_FLOW_TEMP))): vol.All(
            vol.Coerce(float), vol.Range(min=20, max=90)
        ),
        # Keep clearable entity fields default-free; suggested values are
        # provided in async_step_thermal_model.
        vol.Optional(CONF_FLOW_TEMP_ENTITY): str,
        vol.Required(CONF_C_RAD, default=float(_g(cfg, CONF_C_RAD, DEFAULT_C_RAD))): vol.All(
            vol.Coerce(float), vol.Range(min=500, max=100_000)
        ),
        vol.Required(CONF_K_RAD, default=float(_g(cfg, CONF_K_RAD, DEFAULT_K_RAD))): vol.All(
            vol.Coerce(float), vol.Range(min=0.1, max=500)
        ),
        vol.Required(CONF_RAD_EXPONENT, default=float(_g(cfg, CONF_RAD_EXPONENT, DEFAULT_RAD_EXPONENT))): vol.All(
            vol.Coerce(float), vol.Range(min=1.0, max=2.0)
        ),
        vol.Required(CONF_RAD_CONVECTIVE_FRACTION, default=float(_g(cfg, CONF_RAD_CONVECTIVE_FRACTION, DEFAULT_RAD_CONVECTIVE_FRACTION))): vol.All(
            vol.Coerce(float), vol.Range(min=0.1, max=1.0)
        ),
        vol.Required(CONF_FLOW_RATE_MAX, default=float(_g(cfg, CONF_FLOW_RATE_MAX, DEFAULT_FLOW_RATE_MAX))): vol.All(
            vol.Coerce(float), vol.Range(min=0.001, max=1.0)
        ),
        vol.Required(CONF_PIPE_DELAY, default=float(_g(cfg, CONF_PIPE_DELAY, DEFAULT_PIPE_DELAY))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=600)
        ),
        vol.Required(CONF_VALVE_CHARACTERISTIC, default=_g(cfg, CONF_VALVE_CHARACTERISTIC, DEFAULT_VALVE_CHARACTERISTIC)): vol.In(
            [VALVE_CHAR_LINEAR, VALVE_CHAR_QUICK_OPENING]
        ),
        # Room (R2C2)
        vol.Required(CONF_C_AIR, default=float(_g(cfg, CONF_C_AIR, DEFAULT_C_AIR))): vol.All(
            vol.Coerce(float), vol.Range(min=1000, max=2_000_000)
        ),
        vol.Required(CONF_C_FABRIC, default=float(_g(cfg, CONF_C_FABRIC, DEFAULT_C_FABRIC))): vol.All(
            vol.Coerce(float), vol.Range(min=10000, max=50_000_000)
        ),
        vol.Required(CONF_R_FABRIC, default=float(_g(cfg, CONF_R_FABRIC, DEFAULT_R_FABRIC))): vol.All(
            vol.Coerce(float), vol.Range(min=0.0001, max=1.0)
        ),
        vol.Required(CONF_R_EXT, default=float(_g(cfg, CONF_R_EXT, DEFAULT_R_EXT))): vol.All(
            vol.Coerce(float), vol.Range(min=0.0001, max=50.0)
        ),
        vol.Required(CONF_R_INF, default=float(_g(cfg, CONF_R_INF, DEFAULT_R_INF))): vol.All(
            vol.Coerce(float), vol.Range(min=0.001, max=10.0)
        ),
        # Solar
        vol.Required(CONF_WINDOW_AREA, default=float(_g(cfg, CONF_WINDOW_AREA, DEFAULT_WINDOW_AREA))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=100)
        ),
        vol.Required(CONF_WINDOW_TRANSMITTANCE, default=float(_g(cfg, CONF_WINDOW_TRANSMITTANCE, DEFAULT_WINDOW_TRANSMITTANCE))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=1)
        ),
        # Keep clearable entity fields default-free; suggested values are
        # provided in async_step_thermal_model.
        vol.Optional(CONF_SOLAR_ENTITY): str,
        vol.Required(CONF_SOLAR_FIXED, default=float(_g(cfg, CONF_SOLAR_FIXED, DEFAULT_SOLAR_FIXED))): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=1500)
        ),
        vol.Optional(CONF_CALIB_TAU, default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=100000)),
        vol.Optional(CONF_CALIB_B,   default=0.0): vol.All(vol.Coerce(float), vol.Range(min=0, max=1)),
    })


def _validate_sensor_options(data: dict) -> dict:
    """Cross-field validator for sensor options.

    Enforces two mutual-exclusion / ordering rules:
      1. sensor_update_rate and Zigbee min/max/delta cannot both be active.
      2. sensor_min_interval must be strictly less than sensor_max_interval
         when both are non-zero.
    """
    has_simple_rate = float(data.get(CONF_SENSOR_UPDATE_RATE, 0)) > 0
    has_zigbee = any(
        float(data.get(k, 0)) > 0
        for k in (CONF_SENSOR_MIN_INTERVAL, CONF_SENSOR_MAX_INTERVAL, CONF_SENSOR_DELTA)
    )
    if has_simple_rate and has_zigbee:
        raise vol.Invalid(
            "sensor_update_rate cannot be used together with Zigbee "
            "min/max interval or delta. Set sensor_update_rate to 0 "
            "when using Zigbee-style reporting."
        )
    min_i = float(data.get(CONF_SENSOR_MIN_INTERVAL, 0))
    max_i = float(data.get(CONF_SENSOR_MAX_INTERVAL, 0))
    if min_i > 0 and max_i > 0 and min_i >= max_i:
        raise vol.Invalid(
            "sensor_min_interval must be strictly less than sensor_max_interval."
        )
    return data


def _validate_disturbances_options(data: dict) -> dict:
    """Cross-field validator for disturbance options."""
    if not data.get(CONF_EXT_TEMP_PROFILE_ENABLED, False):
        return data

    min_hour = float(data.get(CONF_EXT_TEMP_MIN_HOUR, DEFAULT_EXT_TEMP_MIN_HOUR)) % 24.0
    max_hour = float(data.get(CONF_EXT_TEMP_MAX_HOUR, DEFAULT_EXT_TEMP_MAX_HOUR)) % 24.0
    if min_hour == max_hour:
        raise vol.Invalid(
            "ext_temp_profile_min_hour and ext_temp_profile_max_hour must be different."
        )
    return data


def _strip_unused_r2c2_radiator_fields(data: dict) -> dict:
    """Drop combined-model fields that are not consumed by the runtime model."""
    cleaned = dict(data)
    for key in (CONF_HEAT_LOSS_COEFF_RAD, CONF_C_ROOM_RAD, CONF_CALIB_A):
        cleaned.pop(key, None)
    return cleaned


def _sensor_options_schema(cfg: dict) -> vol.Schema:
    """Sensor imperfection parameters (noise, bias, quantisation, lag, Zigbee reporting)."""
    raw_schema = vol.Schema({
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
        # Zigbee-style conditional reporting
        vol.Optional(CONF_SENSOR_MIN_INTERVAL, default=_g(cfg, CONF_SENSOR_MIN_INTERVAL, DEFAULT_SENSOR_MIN_INTERVAL)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=3600)
        ),
        vol.Optional(CONF_SENSOR_MAX_INTERVAL, default=_g(cfg, CONF_SENSOR_MAX_INTERVAL, DEFAULT_SENSOR_MAX_INTERVAL)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=86400)
        ),
        vol.Optional(CONF_SENSOR_DELTA, default=_g(cfg, CONF_SENSOR_DELTA, DEFAULT_SENSOR_DELTA)): vol.All(
            vol.Coerce(float), vol.Range(min=0, max=10)
        ),
    })
    # Cross-field validation (update_rate vs Zigbee, min < max) is performed
    # in async_step_sensor rather than here, because voluptuous_serialize
    # cannot serialise a vol.All(schema, function) wrapper for the HA frontend.
    return raw_schema


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

    def __init__(self):
        self._setup_data: dict[str, Any] = {}

    async def async_step_user(self, user_input=None) -> FlowResult:
        """Step 1: name, model type, control mode, temperatures, update interval."""
        if user_input is not None:
            self._setup_data = user_input
            model_type = user_input.get(CONF_MODEL_TYPE, MODEL_SIMPLE)
            if model_type == MODEL_R2C2:
                return await self.async_step_model_r2c2()
            elif model_type == MODEL_RADIATOR:
                return await self.async_step_model_radiator()
            elif model_type == MODEL_R2C2_RADIATOR:
                return await self.async_step_model_r2c2_radiator()
            return await self.async_step_model_simple()
        return self.async_show_form(step_id="user", data_schema=STEP_USER_SCHEMA)

    async def async_step_model_simple(self, user_input=None) -> FlowResult:
        """Step 2a: Simple model parameters."""
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
            merged = _strip_unused_r2c2_radiator_fields(merged)
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
      Sensor         — sensor noise, bias, quantisation, lag, Zigbee reporting

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
            # Keep clearable entity-id fields removable: HA may omit optional
            # empty fields from user_input, so preserve explicit clearing.
            user_input = dict(user_input)
            user_input = self._normalise_clearable_text_fields(user_input, [CONF_EXTERNAL_TEMP])
            user_input.setdefault(CONF_EXTERNAL_TEMP, "")
            return self._save_and_return(user_input)
        schema = _general_options_schema(cfg)
        schema = self.add_suggested_values_to_schema(
            schema,
            {CONF_EXTERNAL_TEMP: _g(cfg, CONF_EXTERNAL_TEMP, "")},
        )
        return self.async_show_form(
            step_id="general",
            data_schema=schema,
        )

    # -- Thermal model parameters ------------------------------------------

    async def async_step_thermal_model(self, user_input=None) -> FlowResult:
        """Model physics parameters, pre-populated from current config."""
        cfg = self._cfg
        if user_input is not None:
            user_input = dict(user_input)
            model_type = cfg.get(CONF_MODEL_TYPE, MODEL_SIMPLE)
            if model_type == MODEL_SIMPLE:
                user_input = apply_calibration_simple(user_input, cfg)
            elif model_type == MODEL_R2C2:
                user_input = self._normalise_clearable_text_fields(user_input, [CONF_SOLAR_ENTITY])
                user_input.setdefault(CONF_SOLAR_ENTITY, "")
                user_input = apply_calibration_r2c2(user_input, cfg)
            elif model_type == MODEL_RADIATOR:
                user_input = self._normalise_clearable_text_fields(user_input, [CONF_FLOW_TEMP_ENTITY])
                user_input.setdefault(CONF_FLOW_TEMP_ENTITY, "")
                user_input = apply_calibration_radiator(user_input, cfg)
            elif model_type == MODEL_R2C2_RADIATOR:
                user_input = self._normalise_clearable_text_fields(
                    user_input,
                    [CONF_SOLAR_ENTITY, CONF_FLOW_TEMP_ENTITY],
                )
                user_input.setdefault(CONF_SOLAR_ENTITY, "")
                user_input.setdefault(CONF_FLOW_TEMP_ENTITY, "")
                user_input = apply_calibration_r2c2_radiator(user_input, cfg)
                user_input = _strip_unused_r2c2_radiator_fields(user_input)
            return self._save_and_return(user_input)
        schema = _thermal_model_schema_for_cfg(cfg)
        model_type = cfg.get(CONF_MODEL_TYPE, MODEL_SIMPLE)
        suggested_values: dict[str, Any] = {}
        if model_type == MODEL_R2C2:
            suggested_values[CONF_SOLAR_ENTITY] = _g(cfg, CONF_SOLAR_ENTITY, "")
        elif model_type == MODEL_RADIATOR:
            suggested_values[CONF_FLOW_TEMP_ENTITY] = _g(cfg, CONF_FLOW_TEMP_ENTITY, "")
        elif model_type == MODEL_R2C2_RADIATOR:
            suggested_values[CONF_SOLAR_ENTITY] = _g(cfg, CONF_SOLAR_ENTITY, "")
            suggested_values[CONF_FLOW_TEMP_ENTITY] = _g(cfg, CONF_FLOW_TEMP_ENTITY, "")
        if suggested_values:
            schema = self.add_suggested_values_to_schema(schema, suggested_values)
        return self.async_show_form(
            step_id="thermal_model",
            data_schema=schema,
        )

    # -- Disturbances ------------------------------------------------------

    async def async_step_disturbances(self, user_input=None) -> FlowResult:
        """Occupancy, external temperature profile, wind, rain."""
        cfg = self._cfg
        errors: dict[str, str] = {}
        if user_input is not None:
            try:
                _validate_disturbances_options(user_input)
            except vol.Invalid as exc:
                errors["base"] = str(exc)
            else:
                return self._save_and_return(user_input)
        return self.async_show_form(
            step_id="disturbances",
            data_schema=_disturbances_options_schema(cfg),
            errors=errors,
        )

    # -- Sensor degradation ------------------------------------------------

    async def async_step_sensor(self, user_input=None) -> FlowResult:
        """Sensor noise, bias, quantisation, lag, and Zigbee reporting parameters."""
        cfg = self._cfg
        errors: dict[str, str] = {}
        if user_input is not None:
            try:
                # Run the cross-field validator explicitly so we can surface errors
                _validate_sensor_options(user_input)
            except vol.Invalid as exc:
                errors["base"] = str(exc)
            else:
                return self._save_and_return(user_input)
        return self.async_show_form(
            step_id="sensor",
            data_schema=_sensor_options_schema(cfg),
            errors=errors,
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

    @staticmethod
    def _normalise_clearable_text_fields(data: dict, keys: list[str]) -> dict:
        """Normalise clearable text/entity fields so blanks become empty strings."""
        normalised = dict(data)
        for key in keys:
            if key in normalised:
                value = normalised.get(key)
                if value is None:
                    normalised[key] = ""
                else:
                    value_s = str(value).strip()
                    normalised[key] = value_s if value_s else ""
        return normalised


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
    If calibration fields are absent or zero, returns user_input unchanged.
    """
    a   = float(user_input.get(CONF_CALIB_A,   0))
    tau = float(user_input.get(CONF_CALIB_TAU, 0))
    b   = float(user_input.get(CONF_CALIB_B,   0))
    P   = float(user_input.get(CONF_HEATER_POWER, cfg.get(CONF_HEATER_POWER, DEFAULT_HEATER_POWER)))

    if a <= 0:
        return user_input
    if tau > 0:
        b = 1.0 / tau
    if b <= 0:
        return user_input

    # dT/dt = a * u  −  b * (T − T_ext)
    # At heating: C * dT/dt = P * u − K * (T − T_ext)
    # → a [°C/min] → a/60 [°C/s]
    # C = P / (a/60) = P * 60 / a
    # K = b/60 * C = b/60 * P * 60 / a = b * P / a
    a_per_s = a / 60.0
    b_per_s = b / 60.0
    C = P / a_per_s
    K = b_per_s * C
    user_input = dict(user_input)
    user_input[CONF_THERMAL_MASS]    = round(C, 1)
    user_input[CONF_HEAT_LOSS_COEFF] = round(K, 4)
    return user_input


def apply_calibration_r2c2(user_input: dict, cfg: dict) -> dict:
    """
    Back-calculate c_air and r_ext for the R2C2 model from observed cooling behaviour.

    Only tau (or b) is used — the R2C2 infiltration path is lumped into r_ext.
    calib_a is not meaningful for R2C2 (no single heater power node).
    """
    tau = float(user_input.get(CONF_CALIB_TAU, 0))
    b   = float(user_input.get(CONF_CALIB_B,   0))

    if tau > 0:
        b = 1.0 / tau
    if b <= 0:
        return user_input

    # For the air node dominant approximation:
    # τ_air ≈ C_air * R_ext  (minutes)
    # → R_ext = τ / (C_air * 60)
    b_per_s = b / 60.0
    c_air = float(user_input.get(CONF_C_AIR, cfg.get(CONF_C_AIR, DEFAULT_C_AIR)))
    r_ext = 1.0 / (b_per_s * c_air)
    user_input = dict(user_input)
    user_input[CONF_R_EXT] = round(r_ext, 6)
    return user_input


def apply_calibration_radiator(user_input: dict, cfg: dict) -> dict:
    """
    Back-calculate heat_loss_coeff_rad and c_room_rad for the Wet Radiator model.
    """
    a   = float(user_input.get(CONF_CALIB_A,   0))
    tau = float(user_input.get(CONF_CALIB_TAU, 0))
    b   = float(user_input.get(CONF_CALIB_B,   0))

    if tau > 0:
        b = 1.0 / tau
    if a <= 0 or b <= 0:
        return user_input

    k_rad      = float(user_input.get(CONF_K_RAD,      cfg.get(CONF_K_RAD,      DEFAULT_K_RAD)))
    n          = float(user_input.get(CONF_RAD_EXPONENT,cfg.get(CONF_RAD_EXPONENT,DEFAULT_RAD_EXPONENT)))
    T_flow     = float(user_input.get(CONF_FLOW_TEMP,   cfg.get(CONF_FLOW_TEMP,   DEFAULT_FLOW_TEMP)))
    flow_rate  = float(user_input.get(CONF_FLOW_RATE_MAX,cfg.get(CONF_FLOW_RATE_MAX,DEFAULT_FLOW_RATE_MAX)))
    T_room_op  = 20.0  # operating point assumption

    Q_rad = _q_out_at_operating_point(k_rad, n, T_flow, T_room_op, flow_rate)

    a_per_s = a / 60.0
    b_per_s = b / 60.0
    C = Q_rad / a_per_s
    K = b_per_s * C
    user_input = dict(user_input)
    user_input[CONF_C_ROOM_RAD]        = round(C, 1)
    user_input[CONF_HEAT_LOSS_COEFF_RAD] = round(K, 4)
    return user_input


def apply_calibration_r2c2_radiator(user_input: dict, cfg: dict) -> dict:
    """
    Back-calculate r_ext (R2C2 cooling) for the R2C2 + Radiator model.
    Uses the same tau→r_ext path as the pure R2C2 model.
    """
    return apply_calibration_r2c2(user_input, cfg)
