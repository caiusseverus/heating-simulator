# Heating Simulator Feature Reference

This document explains the major simulator features, how they are implemented, and how to configure them.

## 1) Control input modes

### Linear mode (`control_mode = linear`)

- A number entity provides a continuous command (`0..100%`).
- Internally mapped to `u ∈ [0,1]` and applied directly each simulation tick.

### PWM mode (`control_mode = pwm`)

- A switch entity acts as on/off heat call.
- The simulator applies full-off/full-on command behaviour suited to thermostat relay logic.

Configuration:

- `control_mode`: choose `linear` or `pwm`.

---

## 2) External temperature sourcing

Implementation priority:

1. `external_temperature_entity` (if valid sensor value exists)
2. `external_temperature_fixed` fallback

This allows quick standalone operation with smooth migration to live weather data.

Configuration:

- `external_temperature_entity`
- `external_temperature_fixed`

---

## 3) Disturbance features (weather and occupancy)

These features modify model forcing terms to create realistic non-ideal conditions.

### 3.1 External temperature profile (F-11)

Implements a piecewise raised-cosine diurnal cycle with configurable trough/peak times.

- `ext_temp_profile_enabled`: enable/disable profile
- `ext_temp_profile_base`: daily mean temperature (°C)
- `ext_temp_profile_amplitude`: half daily swing (°C)
- `ext_temp_profile_min_hour`: trough time
- `ext_temp_profile_max_hour`: peak time

When disabled, external temperature remains fixed/entity-driven.

### 3.2 Occupancy/internal gains (F-05)

Adds stochastic internal heat gains from occupants and optional cooking/appliance bursts.

- Occupancy schedule is regenerated per simulated day from seed.
- Heat gain is injected as `Q_int` into the air node / room node.

Configuration:

- `occupancy_enabled`
- `occupancy_max_occupants`
- `occupancy_cooking_power_w`
- `occupancy_cooking_duration_s`
- `occupancy_cooking_events_per_day`
- `occupancy_seed`

### 3.3 Wind/rain envelope effects (F-06/F-14)

Applies a weather multiplier to external heat-loss conductance paths.

- In simple/radiator models: scales effective loss coefficient.
- In R2C2 models: scales `1/r_ext` and `1/r_infiltration` paths.

Configuration:

- `weather_wind_speed_m_s`
- `weather_wind_coefficient`
- `weather_rain_intensity`
- `weather_rain_moisture_factor`

---

## 4) Sensor realism pipeline

The reported room-temperature sensor can be degraded while preserving the true model state.

Implementation order per tick:

1. **Lag**: first-order low-pass filter with `alpha = dt/(tau+dt)`
2. **Bias**: additive offset
3. **Noise**: Gaussian random sample
4. **Quantisation**: round to step
5. **Rate limit**: hold previous report until interval elapses

A companion `*_true_room_temperature` sensor exposes the undegraded value.

Configuration:

- `sensor_lag_tau`
- `sensor_bias`
- `sensor_noise_std_dev`
- `sensor_quantisation`
- `sensor_update_rate_s`

Use cases:

- Mimic TRV quantised sensors (e.g. 0.5 °C steps).
- Emulate delayed battery sensor response.
- Test controller robustness against noise/bias.

---

## 5) Behavioural calibration

Optional calibration fields derive physical parameters from observed thermal response.

Reference first-order form:

\[
\frac{dT}{dt}=a\,u-b(T-T_{ext})
\]

Inputs:

- `calib_a` (heating rate term)
- `calib_b` (loss coefficient term)
- `calib_tau` (time constant; used preferentially over `calib_b`)

Model-specific behaviour:

- **simple**: derives thermal mass and loss coefficient
- **r2c2**: derives envelope term (`r_ext`) from long time constant
- **radiator**: derives room capacitance/loss terms with radiator operating point context
- **r2c2_radiator**: adjusts equivalent room-envelope parameters while preserving model structure

Calibration fields are consumed during save and converted into normal physical parameters.

---

## 6) Solar gain and flow-temperature overrides

Model-specific number entities allow runtime perturbation:

- `*_solar_irradiance_override` (R2C2 models)
- `*_flow_temperature_override` (radiator models)
- `*_external_temperature_override` (all models)

These are useful for scenario testing (e.g., sunny intervals, weather compensation, cold snaps).

---

## 7) Entity outputs and diagnostics

Depending on model, the integration exposes:

- Core temperatures and rates (`room_temperature`, `heating_rate`, `heat_loss_rate`, `net_heating_rate`).
- Effective heat-delivery metrics.
- R2C2 diagnostics (`fabric_temperature`, `solar_gain`, `total_heat_loss`).
- Radiator diagnostics (`radiator_temperature`, `radiator_heat_input`, `radiator_heat_output`, return temperature estimate).

Diagnostic attributes include model-specific quantities such as heat-flow breakdowns, weather multipliers, and radiator output fraction.

---

## 8) Configuration quick guide

### Common options

- `model_type`
- `control_mode`
- `initial_temperature`
- `external_temperature_entity`
- `external_temperature_fixed`
- `update_interval_seconds`

### Choose by objective

- Fast smoke tests: `simple` + low complexity.
- Envelope realism: `r2c2`.
- Hydronic/TRV realism: `radiator`.
- Full interaction fidelity: `r2c2_radiator`.

### Practical workflow

1. Start with physically plausible defaults.
2. Match steady-state losses first.
3. Match dynamic timing with capacitances and inertia/delay.
4. Add disturbances and sensor realism last.
