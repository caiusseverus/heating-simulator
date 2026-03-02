# Heating Simulator — Home Assistant Integration

A custom integration providing physics-based room thermal models for testing and tuning thermostat controllers, TRV automations, PID loops, and smart heating systems — without needing real hardware.

---

## Features

- **Four thermal models** of increasing physical fidelity — from a simple single-node model to a full wet radiator with two-node room and solar gain
- **Sensor imperfection pipeline** — simulate real-world sensor lag, bias, noise, quantisation, and rate-limiting
- **Disturbance inputs** — diurnal external temperature profile, stochastic occupancy and cooking heat gain, and wind/rain effects on heat loss
- **Behavioural calibration** — derive model parameters from observed room behaviour instead of physical measurements
- **Live overrides** — solar irradiance, flow temperature, and external temperature can all be overridden at runtime via number entities
- **Service calls** — `reset_model` to set known initial conditions, `set_weather` to update wind and rain live
- **State restoration** — room, fabric, and radiator temperatures are restored from the HA state machine on restart
- **Two control modes** — proportional (0–100% number entity) or PWM (on/off switch)

---

## Models

| Model | Key | State variables | Best for |
|---|---|---|---|
| Simple (R1C1) | `simple` | T_room | Quick tests, thermostat validation |
| R2C2 + Solar | `r2c2` | T_air, T_fab | Fabric lag, solar gain testing |
| Wet Radiator | `radiator` | T_rad, T_room | TRV/PID tuning, flow temperature effects |
| R2C2 + Radiator | `r2c2_radiator` | T_rad, T_air, T_fab | Full physical simulation |

---

## Installation

1. Copy `custom_components/heating_simulator/` into your HA `config/custom_components/`.
2. Restart Home Assistant.
3. **Settings → Devices & Services → Add Integration → Heating Simulator**
4. Step 1: choose name, model type, control mode, temperatures, and update interval.
5. Step 2: model-specific parameters, sensor imperfection, and disturbance options.

To reconfigure without removing the entry: **Settings → Devices & Services → Heating Simulator → Configure**.

---

## Quick Configuration

| Setting | What it does |
|---|---|
| `model_type` | Choose which thermal model to use |
| `control_mode` | `linear` (0–100% slider) or `pwm` (on/off switch) |
| `initial_temperature` | Starting room temperature (°C) |
| `external_temperature_entity` | Live outdoor sensor (takes priority over fixed value) |
| `external_temperature_fixed` | Fixed fallback outdoor temperature (°C) |
| `update_interval_seconds` | Simulation tick rate — lower = smoother, more CPU |

For model-specific parameters, calibration, sensor imperfection, and disturbance settings, see the full configuration reference.

---

## Documentation

| Document | Contents |
|---|---|
| [docs/models.md](docs/models.md) | Mathematical models, equations, sizing guidance, behavioural calibration |
| [docs/configuration.md](docs/configuration.md) | All configuration parameters with descriptions and recommended values |
| [docs/entities.md](docs/entities.md) | Full entity reference (sensors, numbers, switches) |
| [docs/usage-examples.md](docs/usage-examples.md) | Automation examples, dashboard YAML, testing workflows |
