"""Constants for the Heating Simulator integration."""

DOMAIN = "heating_simulator"

# ---------------------------------------------------------------------------
# Config entry keys — shared
# ---------------------------------------------------------------------------
CONF_MODEL_TYPE = "model_type"
CONF_CONTROL_MODE = "control_mode"
CONF_INITIAL_TEMP = "initial_temperature"
CONF_EXTERNAL_TEMP = "external_temperature_entity"
CONF_EXTERNAL_TEMP_FIXED = "external_temperature_fixed"
CONF_UPDATE_INTERVAL = "update_interval_seconds"

# ---------------------------------------------------------------------------
# Config keys — Simple (R1C1) model
# ---------------------------------------------------------------------------
CONF_HEATER_POWER = "heater_power_watts"
CONF_HEAT_LOSS_COEFF = "heat_loss_coefficient"
CONF_THERMAL_MASS = "thermal_mass"
CONF_THERMAL_INERTIA = "thermal_inertia"

# ---------------------------------------------------------------------------
# Config keys — R2C2 + solar gain model
# ---------------------------------------------------------------------------
CONF_C_AIR = "c_air"                         # J/°C  — air thermal mass
CONF_C_FABRIC = "c_fabric"                   # J/°C  — building fabric thermal mass
CONF_R_FABRIC = "r_fabric"                   # °C/W  — resistance air↔fabric
CONF_R_EXT = "r_ext"                         # °C/W  — resistance fabric↔outside
CONF_R_INF = "r_infiltration"                # °C/W  — infiltration/ventilation path air↔outside
CONF_HEATER_POWER_R2C2 = "heater_power_watts_r2c2"
CONF_SOLAR_ENTITY = "solar_irradiance_entity"
CONF_SOLAR_FIXED = "solar_irradiance_fixed"  # W/m²
CONF_WINDOW_AREA = "window_area_m2"
CONF_WINDOW_TRANSMITTANCE = "window_transmittance"

# ---------------------------------------------------------------------------
# Config keys — Wet radiator model
# ---------------------------------------------------------------------------
CONF_FLOW_TEMP = "flow_temperature"          # °C  boiler flow temperature
CONF_FLOW_TEMP_ENTITY = "flow_temperature_entity"
CONF_C_RAD = "c_radiator"                    # J/°C  — radiator water+metal thermal mass
CONF_K_RAD = "k_radiator"                    # W/°C^n  — radiator emission coefficient
CONF_RAD_EXPONENT = "radiator_exponent"      # n ≈ 1.3 for panel rads (BS EN 442)
CONF_RAD_CONVECTIVE_FRACTION = "radiator_convective_fraction"  # fraction of Q_out to air node
CONF_FLOW_RATE_MAX = "flow_rate_max_kg_s"    # kg/s max water flow rate
CONF_HEAT_LOSS_COEFF_RAD = "heat_loss_coefficient_rad"
CONF_C_ROOM_RAD = "c_room_rad"              # J/°C  — room thermal mass (rad model)
CONF_PIPE_DELAY = "pipe_delay_seconds"       # dead-time for hot water to reach radiator

# ---------------------------------------------------------------------------
# Config keys — Behavioural calibration (optional, overrides physical params)
# ---------------------------------------------------------------------------
# These allow the room thermal parameters to be derived from observed behaviour
# rather than physical measurements.  When populated, the options flow back-
# calculates c_room + K_loss (simple model) or c_air + r_ext (R2C2) and writes
# the result into the normal physical parameter slots.
#
# Parameters match the first-order thermostat model:
#   dT/dt = a·u  −  b·(T − T_ext)
# where u=1 when heating, u=0 when off.
#
CONF_CALIB_A   = "calib_a"    # °C/min  — heating rate coefficient (u=1, radiator at op. temp)
CONF_CALIB_B   = "calib_b"    # 1/min   — loss rate coefficient (= 1/tau)
CONF_CALIB_TAU = "calib_tau"  # min     — cooling time constant (= 1/b); takes priority over b

# ---------------------------------------------------------------------------
# Model types
# ---------------------------------------------------------------------------
MODEL_SIMPLE = "simple"              # Original R1C1 with first-order lag
MODEL_R2C2 = "r2c2"                 # Two-node room with solar gain
MODEL_RADIATOR = "radiator"          # Wet radiator with explicit water temperature
MODEL_R2C2_RADIATOR = "r2c2_radiator"  # Full model: wet radiator + R2C2 room + solar

# ---------------------------------------------------------------------------
# Control modes
# ---------------------------------------------------------------------------
CONTROL_MODE_LINEAR = "linear"   # 0–100% power/valve
CONTROL_MODE_PWM = "pwm"         # on/off switch

# ---------------------------------------------------------------------------
# Defaults — shared
# ---------------------------------------------------------------------------
DEFAULT_INITIAL_TEMP = 18.0
DEFAULT_EXTERNAL_TEMP_FIXED = 5.0
DEFAULT_UPDATE_INTERVAL = 10

# Defaults — simple
DEFAULT_HEATER_POWER = 2000.0
DEFAULT_HEAT_LOSS_COEFF = 50.0
DEFAULT_THERMAL_MASS = 350000.0
DEFAULT_THERMAL_INERTIA = 0.0

# Defaults — R2C2
# Represents a typical furnished room (~20 m²) with moderate insulation.
# IMPORTANT: c_air represents air + ALL room contents (furniture, carpet, curtains)
# that heat up on the same timescale as the air (minutes). It is NOT just the air mass.
# Air alone in 50m³ ≈ 60,000 J/°C; furnished room ≈ 300,000–500,000 J/°C.
DEFAULT_C_AIR = 350_000.0        # J/°C  (air + furniture + soft furnishings + surface layers)
# c_fabric is ONLY the building fabric (walls, floor, ceiling).
# A 20m² brick cavity room has ~60m² of surfaces × 120 kg/m² × 840 J/(kg·°C) ≈ 6,000,000 J/°C.
# Timber-frame/plasterboard is lighter: ~500,000 J/°C.
DEFAULT_C_FABRIC = 5_000_000.0   # J/°C  (brick cavity construction, 60m² surfaces)
DEFAULT_R_FABRIC = 0.005         # °C/W  (good internal surface conductance)
DEFAULT_R_EXT = 0.020            # °C/W  (~50 W/°C at ΔT=1; moderate insulation)
DEFAULT_R_INF = 0.067            # °C/W  (~0.5 ACH infiltration)
DEFAULT_HEATER_POWER_R2C2 = 2000.0
DEFAULT_SOLAR_FIXED = 0.0        # W/m²
DEFAULT_WINDOW_AREA = 2.0        # m²
DEFAULT_WINDOW_TRANSMITTANCE = 0.6

# Defaults — Wet radiator
DEFAULT_FLOW_TEMP = 70.0         # °C  traditional boiler
DEFAULT_C_RAD = 8_000.0          # J/°C  (medium double panel radiator + water)
DEFAULT_K_RAD = 10.0             # W/°C^n (size coefficient)
DEFAULT_RAD_EXPONENT = 1.3       # BS EN 442 panel radiator
# Convective fraction: fraction of Q_out that heats the air node directly.
# The remainder is longwave radiation absorbed directly by fabric surfaces.
# BS EN 442 values by radiator type:
#   Type 10 (single panel, no fins):   ~50% convective
#   Type 11 (single panel + fins):     ~65% convective
#   Type 21 (double panel + fins):     ~75% convective
#   Type 22 (double panel, 2× fins):   ~80% convective  ← most common UK domestic
#   Fan coil / convector:              ~90–95% convective
#   Underfloor heating (water):        ~50% convective
# Default 0.75 suits a typical type 21/22 panel radiator.
DEFAULT_RAD_CONVECTIVE_FRACTION = 0.75
DEFAULT_FLOW_RATE_MAX = 0.05     # kg/s  (~3 L/min)
DEFAULT_HEAT_LOSS_COEFF_RAD = 50.0
DEFAULT_C_ROOM_RAD = 500_000.0   # J/°C  (air + all room contents, same definition as c_air)
DEFAULT_PIPE_DELAY = 0.0         # seconds

# ---------------------------------------------------------------------------
# Config keys — Sensor imperfection suite (F-02, F-04, F-07)
# ---------------------------------------------------------------------------
# All parameters are optional; omitting them (or setting to 0) disables
# the corresponding effect so the sensor behaves identically to before.
CONF_SENSOR_NOISE_STD_DEV   = "sensor_noise_std_dev"   # °C  — Gaussian noise σ
CONF_SENSOR_BIAS            = "sensor_bias"             # °C  — fixed additive offset
CONF_SENSOR_QUANTISATION    = "sensor_quantisation"     # °C  — minimum reportable step
CONF_SENSOR_LAG_TAU         = "sensor_lag_tau"          # s   — first-order low-pass time constant
CONF_SENSOR_UPDATE_RATE     = "sensor_update_rate_s"    # s   — minimum interval between reports (0 = every tick)

# Defaults — sensor imperfection (all zero = perfect sensor, back-compatible)
DEFAULT_SENSOR_NOISE_STD_DEV = 0.0
DEFAULT_SENSOR_BIAS          = 0.0
DEFAULT_SENSOR_QUANTISATION  = 0.0
DEFAULT_SENSOR_LAG_TAU       = 0.0
DEFAULT_SENSOR_UPDATE_RATE   = 0.0

# ---------------------------------------------------------------------------
# Config keys — F-11  External temperature profile
# ---------------------------------------------------------------------------
CONF_EXT_TEMP_PROFILE_ENABLED = "ext_temp_profile_enabled"
CONF_EXT_TEMP_BASE            = "ext_temp_profile_base"        # °C  mean temp
CONF_EXT_TEMP_AMPLITUDE       = "ext_temp_profile_amplitude"   # °C  half-swing
CONF_EXT_TEMP_MIN_HOUR        = "ext_temp_profile_min_hour"    # h   trough time
CONF_EXT_TEMP_MAX_HOUR        = "ext_temp_profile_max_hour"    # h   peak time

# Defaults — F-11 (disabled = back-compatible)
DEFAULT_EXT_TEMP_PROFILE_ENABLED = False
DEFAULT_EXT_TEMP_BASE            = 5.0
DEFAULT_EXT_TEMP_AMPLITUDE       = 3.0
DEFAULT_EXT_TEMP_MIN_HOUR        = 5.5
DEFAULT_EXT_TEMP_MAX_HOUR        = 14.5

# ---------------------------------------------------------------------------
# Config keys — F-05  Occupancy and internal heat gain
# ---------------------------------------------------------------------------
CONF_OCCUPANCY_ENABLED             = "occupancy_enabled"
CONF_OCCUPANCY_MAX_OCCUPANTS       = "occupancy_max_occupants"      # persons
CONF_OCCUPANCY_COOKING_POWER       = "occupancy_cooking_power_w"    # W
CONF_OCCUPANCY_COOKING_DURATION    = "occupancy_cooking_duration_s" # s
CONF_OCCUPANCY_COOKING_EVENTS_PER_DAY = "occupancy_cooking_events_per_day"
CONF_OCCUPANCY_SEED                = "occupancy_seed"               # int

# Defaults — F-05 (disabled = back-compatible)
DEFAULT_OCCUPANCY_ENABLED             = False
DEFAULT_OCCUPANCY_MAX_OCCUPANTS       = 2
DEFAULT_OCCUPANCY_COOKING_POWER       = 0.0
DEFAULT_OCCUPANCY_COOKING_DURATION    = 1200.0
DEFAULT_OCCUPANCY_COOKING_EVENTS_PER_DAY = 2.0
DEFAULT_OCCUPANCY_SEED                = 42

# ---------------------------------------------------------------------------
# Config keys — F-06, F-14  Wind and rain effects
# ---------------------------------------------------------------------------
CONF_WIND_SPEED           = "weather_wind_speed_m_s"        # m/s
CONF_WIND_COEFFICIENT     = "weather_wind_coefficient"      # per (m/s) — 0 = disabled
CONF_RAIN_INTENSITY       = "weather_rain_intensity"        # 0–1
CONF_RAIN_MOISTURE_FACTOR = "weather_rain_moisture_factor"  # 0–0.5 — 0 = disabled

# Defaults — F-06, F-14 (all zero = neutral, back-compatible)
DEFAULT_WIND_SPEED           = 0.0
DEFAULT_WIND_COEFFICIENT     = 0.0
DEFAULT_RAIN_INTENSITY       = 0.0
DEFAULT_RAIN_MOISTURE_FACTOR = 0.0

# ---------------------------------------------------------------------------
# Reset action
# ---------------------------------------------------------------------------
ACTION_RESET = "reset_model"

PRESET_COLD_START       = "cold_start"       # all nodes at external temp — heating off for days
PRESET_OVERNIGHT        = "overnight"        # air 16°C, fabric 14°C — heating off overnight
PRESET_ROOM_TEMPERATURE = "room_temperature" # air 18°C, fabric 17°C — typical occupied room

# Fabric lags air due to high thermal mass:
#   cold_start:       fabric = external temp (fully equilibrated over days)
#   overnight:        air 16°C, fabric 14°C — fabric only partially cooled in ~8 h
#   room_temperature: air 18°C, fabric 17°C — fabric nearly at air temp after sustained heating
#
# t_room=None in cold_start means "use current external_temperature at reset time".
RESET_PRESETS: dict[str, dict] = {
    PRESET_COLD_START:       {"t_room": None, "t_fabric": None},
    PRESET_OVERNIGHT:        {"t_room": 16.0, "t_fabric": 14.0},
    PRESET_ROOM_TEMPERATURE: {"t_room": 18.0, "t_fabric": 17.0},
}
