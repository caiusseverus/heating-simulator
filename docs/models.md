# Heating Simulator Model Reference

This document describes each thermal model in detail: governing equations, numerical behaviour, practical interpretation, and configuration guidance.

## Shared simulation concepts

All models use these shared inputs:

- `T_ext`: external temperature (fixed value or entity).
- `u`: control input (0–1), from linear power/valve input or PWM switch.
- `dt`: simulation timestep (seconds), set by `update_interval_seconds`.

Most models expose:

- Primary room temperature (air node for multi-node models).
- Heating/loss/net rates.
- Effective heater power and model-specific diagnostics.

---

## 1) Simple model (`simple`, R1C1)

### Physical interpretation

A single lumped room thermal mass with outside heat loss and optional first-order heater inertia.

### Equations

Room energy balance:

\[
C\,\frac{dT}{dt}=Q_{eff}-K(T-T_{ext})+Q_{int}
\]

Where:

- `C = thermal_mass` (J/°C)
- `K = heat_loss_coefficient` (W/°C)
- `Q_eff` is effective heater output (W)
- `Q_int` is internal gains (occupancy/cooking disturbances when enabled)

Heater inertia (`thermal_inertia = τ`) is implemented as exact discrete-time first-order lag:

\[
Q_{eff}(t)=Q_{target} + (Q_{eff}(t-\Delta t)-Q_{target})e^{-\Delta t/\tau}
\]

with `Q_target = u * heater_power_watts`. If `τ = 0`, output is instantaneous.

### Numerical behaviour

- Heater lag uses exact exponential update, so it is stable for any `dt`.
- Temperature state is integrated directly each tick; suitable at normal Home Assistant update rates.

### Configuration guide

| Option | What it controls | Typical effect |
|---|---|---|
| `heater_power_watts` | Max heating power at `u=1` | Larger value raises warm-up slope |
| `heat_loss_coefficient` | Envelope+ventilation loss to outside | Larger value lowers steady-state temp and speeds cooldown |
| `thermal_mass` | Effective room capacitance | Larger value slows both warm-up and cooldown |
| `thermal_inertia` | First-order lag in heater output | Adds actuator delay / smoother transitions |
| `calib_a`, `calib_b`, `calib_tau` | Behavioural calibration inputs | Back-calculates physical parameters from observed behaviour |

---

## 2) R2C2 + Solar model (`r2c2`)

### Physical interpretation

Two thermal nodes:

- Air node `T_air` (fast dynamics; what controls usually measure)
- Fabric node `T_fab` (walls/floor/ceiling thermal storage)

Solar input is modelled through effective glazed area and transmittance.

### Equations

\[
C_{air}\frac{dT_{air}}{dt}=Q_{heater}+Q_{solar,air}+Q_{int}+\frac{T_{fab}-T_{air}}{R_{fab}}-\frac{T_{air}-T_{ext}}{R_{inf}}
\]

\[
C_{fab}\frac{dT_{fab}}{dt}=Q_{solar,fab}-\frac{T_{fab}-T_{air}}{R_{fab}}-\frac{T_{fab}-T_{ext}}{R_{ext}}
\]

Solar gain:

\[
Q_{solar,total}=I_{solar}\cdot A_{window}\cdot g
\]

Implementation split:

- `Q_solar,fab = 0.9 * Q_solar,total`
- `Q_solar,air = 0.1 * Q_solar,total`

### Numerical behaviour

The model automatically sub-steps integration when needed for stability with small thermal resistances. This avoids requiring users to manually tune solver settings.

### Configuration guide

| Option | Meaning |
|---|---|
| `heater_power_watts_r2c2` | Heater power injected into air node |
| `c_air` | Air-side effective capacitance (air + fast-responding contents) |
| `c_fabric` | Building fabric capacitance (slow storage) |
| `r_fabric` | Air↔fabric coupling resistance |
| `r_ext` | Fabric↔outside resistance |
| `r_infiltration` | Air↔outside ventilation/infiltration resistance |
| `window_area_m2` | Effective solar aperture |
| `window_transmittance` | Solar heat gain coefficient |
| `solar_irradiance_entity` / `solar_irradiance_fixed` | Dynamic or fallback irradiance source |
| `calib_tau`, `calib_b` | Long-time-constant calibration for envelope response |

Tuning notes:

- Increase `c_fabric` to lengthen post-heating “coast-down”.
- Decrease `r_ext` to represent a leakier/poorer-insulated envelope.
- Decrease `r_infiltration` for draughtier spaces.

---

## 3) Wet Radiator model (`radiator`)

### Physical interpretation

Explicitly models radiator thermal state and non-linear radiator output. Captures asymmetry between rapid heating and slower passive cooling.

### Equations

Radiator state:

\[
C_{rad}\frac{dT_{rad}}{dt}=Q_{in}-Q_{out}
\]

Room state:

\[
C_{room}\frac{dT_{room}}{dt}=Q_{out}-K_{loss}(T_{room}-T_{ext})+Q_{int}
\]

Input from hydronic loop:

\[
Q_{in}=u\cdot \dot m_{max}\cdot c_{water}\cdot \max(0,T_{flow}-T_{rad})
\]

Radiator emission (BS EN 442 style):

\[
Q_{out}=K_{rad}\cdot |T_{rad}-T_{room}|^n\cdot sign(T_{rad}-T_{room})
\]

Where `n` is typically ~1.3 for panel radiators.

### Pipe delay

`pipe_delay_seconds` is implemented as a FIFO delay on valve command, representing transport dead-time between actuation and effective hot-water arrival.

### Configuration guide

| Option | Meaning |
|---|---|
| `flow_temperature` / `flow_temperature_entity` | Supply water temperature source |
| `c_radiator` | Radiator water+metal thermal mass |
| `k_radiator` | Emission coefficient (set from rated output) |
| `radiator_exponent` | Non-linear emission exponent |
| `flow_rate_max_kg_s` | Maximum hydronic mass flow |
| `heat_loss_coefficient_rad` | Room envelope loss coefficient |
| `c_room_rad` | Room thermal mass for this model |
| `pipe_delay_seconds` | Dead-time in hydronic response |
| `calib_a`, `calib_b`, `calib_tau` | Behavioural calibration inputs |

Sizing tip for `k_radiator` at ΔT50 rating:

\[
K_{rad}=\frac{P_{rated}}{50^n}
\]

---

## 4) Combined R2C2 + Radiator (`r2c2_radiator`)

### Physical interpretation

Three-state model combining hydronic emitter dynamics with two-node room envelope dynamics.

States:

- `T_rad`: radiator mean temperature
- `T_air`: room air
- `T_fab`: room fabric

### Equations

Radiator:

\[
C_{rad}\frac{dT_{rad}}{dt}=Q_{in}-Q_{out}
\]

Radiator output split by convective fraction `f_conv`:

\[
Q_{conv}=f_{conv}Q_{out},\quad Q_{rad}=(1-f_{conv})Q_{out}
\]

Air node:

\[
C_{air}\frac{dT_{air}}{dt}=Q_{conv}+Q_{solar,air}+Q_{int}+\frac{T_{fab}-T_{air}}{R_{fab}}-\frac{T_{air}-T_{ext}}{R_{inf}}
\]

Fabric node:

\[
C_{fab}\frac{dT_{fab}}{dt}=Q_{rad}+Q_{solar,fab}+\frac{T_{air}-T_{fab}}{R_{fab}}-\frac{T_{fab}-T_{ext}}{R_{ext}}
\]

### Why this model is useful

- Captures emitter-limited warm-up under low flow temperatures.
- Captures air/fabric lag and rebound after heating cycles.
- Supports solar gain and envelope storage simultaneously.

### Configuration guide

This model combines radiator and R2C2 parameters plus:

| Option | Meaning |
|---|---|
| `radiator_convective_fraction` | Split of radiator output to air (rest to fabric) |

Practical tuning order:

1. Set radiator parameters (`flow_temperature`, `k_radiator`, `flow_rate_max_kg_s`).
2. Tune room losses (`r_ext`, `r_infiltration`) to match steady-state requirement.
3. Tune capacitances (`c_air`, `c_fabric`, `c_radiator`) to match timing.
4. Add `pipe_delay_seconds` if valve response appears too immediate.

---

## Shared setup options (all models)

| Option | Description |
|---|---|
| `model_type` | Selects simulation model |
| `control_mode` | `linear` (0–100%) or `pwm` (switch-driven duty control) |
| `initial_temperature` | Initial state at setup/reset |
| `external_temperature_entity` | Optional live outside temperature source |
| `external_temperature_fixed` | Fallback outside temperature |
| `update_interval_seconds` | Simulation tick interval |
