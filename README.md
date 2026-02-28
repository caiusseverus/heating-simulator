# Heating Simulator — Home Assistant Integration

A custom integration providing four physics-based room thermal models for testing and tuning thermostat controllers, TRV automations, PID loops, and smart heating systems — without needing real hardware.

---

## Models

### 1. Simple (R1C1)

Single thermal mass with an optional first-order heater lag. Good for quick tests and simple thermostat validation.

```
C · dT/dt = Q_heater(t) − K · (T − T_ext)
```

Heater lag: `τ · dQ_eff/dt = Q_input − Q_eff` (symmetric, instant if τ = 0).
The lag uses the exact discrete-time exponential solution, so it is unconditionally stable at any update interval.

**Parameters**

| Key | Description | Range | Default |
|---|---|---|---|
| `heater_power_watts` | Nominal heater power at 100% (W) | 100–50 000 | 2 000 |
| `heat_loss_coefficient` | Room K-value — heat loss per °C above outside (W/°C) | 0.001–2 000 | 50 |
| `thermal_mass` | Room thermal mass C (J/°C) | 100–20 000 000 | 10 000 |
| `thermal_inertia` | First-order heater lag τ (s); 0 = instant | 0–7 200 | 0 |

---

### 2. R2C2 + Solar Gain

Two-node model — air and building fabric — with solar gain through windows. Based on the ISO 13790 simplified hourly method.

```
C_air · dT_air/dt = Q_heater + Q_solar_air + (T_fab − T_air)/R_fab − (T_air − T_ext)/R_inf
C_fab · dT_fab/dt = Q_solar_fab + (T_air − T_fab)/R_fab − (T_fab − T_ext)/R_ext
```

Solar gain is split 10 % to the air node and 90 % to the fabric node (shortwave radiation is absorbed primarily by opaque surfaces). Sub-stepping is applied automatically to maintain numerical stability when R values are small.

**Why this matters:**
- Cold walls slow down the first warm-up of a room even after the air is warm.
- Solar gain through windows can eliminate the need for heating on sunny days.
- Infiltration (draughts) and fabric loss are independent heat-loss paths.
- The fabric node acts as thermal storage — the room cools slowly after heating stops.

**Parameters**

| Key | Description | Range | Default |
|---|---|---|---|
| `heater_power_watts_r2c2` | Nominal heater power (W) | 100–50 000 | 2 000 |
| `c_air` | Air + lightweight furniture thermal mass (J/°C). **Include all room contents** that heat up on the same timescale as the air — furniture, carpet, curtains. A furnished 20 m² room is typically 300 000–600 000 J/°C. | 1 000–20 000 000 | 350 000 |
| `c_fabric` | Building fabric only — walls, floor, ceiling (J/°C). Brick cavity 20 m² room ≈ 5 000 000; timber frame ≈ 500 000. | 10 000–50 000 000 | 5 000 000 |
| `r_fabric` | Internal surface resistance air↔fabric (°C/W) | 0.0001–1.0 | 0.005 |
| `r_ext` | Fabric↔outside resistance (°C/W). `1/K_fabric`. At 50 W/°C fabric loss: 0.020. | 0.0001–50.0 | 0.020 |
| `r_infiltration` | Infiltration/ventilation path air↔outside (°C/W). ~0.5 ACH in 100 m³: 0.067. | 0.001–10.0 | 0.067 |
| `window_area_m2` | Glazed area facing the sun (m²) | 0–100 | 2.0 |
| `window_transmittance` | Solar heat gain coefficient (g-value) | 0–1 | 0.6 |
| `solar_irradiance_entity` | HA entity providing irradiance (W/m²); overrides fixed value when available | — | — |
| `solar_irradiance_fixed` | Fixed fallback irradiance (W/m²) | 0–1 500 | 0.0 |

**Sizing R_ext:** `R_ext = 1 / K_fabric` where K_fabric is your fabric heat loss in W/°C.
For a room losing 40 W/°C through the fabric: `R_ext = 1/40 = 0.025 °C/W`.

---

### 3. Wet Radiator

Explicit radiator water temperature with BS EN 442 power-law output. Models the physical asymmetry between heating (driven by boiler ΔT) and cooling (passive decay to room temperature). An optional pipe delay (FIFO queue) models the dead time between valve actuation and hot water arriving at the radiator.

```
C_rad  · dT_rad/dt  = Q_in − Q_out
C_room · dT_room/dt = Q_out − K_loss · (T_room − T_ext)

Q_in  = valve × ṁ_max × C_water × max(0, T_flow − T_rad)
Q_out = K_rad × |T_rad − T_room|^n    (n ≈ 1.3, BS EN 442)
```

**Why this matters:**
- Heating is fast (limited by boiler ΔT ~50 °C × flow rate).
- Cooling is slow (governed by the shrinking ΔT between radiator and room).
- Output drops as the room warms up — `Q_out ∝ ΔT^1.3`, not valve position.
- At low flow temperatures (heat pump, 35–45 °C) output is dramatically reduced.
- Valve position → mass flow → Q_in is a non-linear chain, not a simple multiplier.

**Parameters**

| Key | Description | Range | Default |
|---|---|---|---|
| `flow_temperature` | Boiler/heat pump flow temperature (°C) | 20–90 | 70 |
| `flow_temperature_entity` | HA entity providing live flow temperature; overrides fixed value | — | — |
| `c_radiator` | Radiator water + metal thermal mass (J/°C). Small single panel: ~3 000; large double panel: ~15 000. | 500–100 000 | 8 000 |
| `k_radiator` | BS EN 442 emission coefficient (W/°C^n). `P_nominal / 50^n`. | 0.1–500 | 10 |
| `radiator_exponent` | BS EN 442 exponent n. Panel: 1.3, convector: 1.5. | 1.0–2.0 | 1.3 |
| `flow_rate_max_kg_s` | Maximum water mass flow rate (kg/s). Typical: 0.03–0.1. | 0.001–1.0 | 0.05 |
| `heat_loss_coefficient_rad` | Room K-value (W/°C) | 0.001–2 000 | 50 |
| `c_room_rad` | Room thermal mass (J/°C) — air + all room contents | 1 000–20 000 000 | 500 000 |
| `pipe_delay_seconds` | Dead time for hot water to reach radiator (s). Modelled as a 1-s-resolution FIFO queue. | 0–600 | 0 |

**Sizing K_radiator:**
If your radiator is rated at 1 500 W at ΔT50 (70 °C flow, 20 °C room):
```
K_rad = 1500 / (50^1.3) = 1500 / 142.5 ≈ 10.5 W/°C^1.3
```

---

### 4. R2C2 + Radiator (full combined model)

The most physically complete model. Combines the wet radiator with the two-node R2C2 room and solar gain. Three state variables: T_rad, T_air, T_fab.

```
C_rad · dT_rad/dt = Q_in − Q_out

C_air · dT_air/dt = Q_conv + Q_solar_air + (T_fab − T_air)/R_fab − (T_air − T_ext)/R_inf

C_fab · dT_fab/dt = Q_rad + Q_solar_fab − (T_fab − T_air)/R_fab − (T_fab − T_ext)/R_ext
```

`Q_out` (BS EN 442 power law referenced to T_air) is split between:
- `Q_conv = Q_out × conv_frac` → heats air node directly
- `Q_rad  = Q_out × (1 − conv_frac)` → absorbed by fabric surfaces

Solar gain is also split 10 % / 90 % to air / fabric as in the R2C2 model.

**Why the convective split matters:**
A radiator does not heat the air alone. The radiative fraction warms floors and walls directly, which then slowly re-emit heat to the air. This produces a more realistic fabric warm-up, a slower apparent response in the air node, and better agreement with measured room data.

**Convective fraction by radiator type (BS EN 442 / ISO 11855):**

| Type | Description | Conv. fraction |
|---|---|---|
| Type 10 | Single panel, no fins | ~0.50 |
| Type 11 | Single panel + fins | ~0.65 |
| Type 21 | Double panel + fins | ~0.75 |
| Type 22 | Double panel, 2× fins (most common UK domestic) | ~0.80 |
| Fan coil / convector | — | ~0.90–0.95 |
| Underfloor heating (water) | — | ~0.50 |

**Parameters** — all Wet Radiator parameters except `heat_loss_coefficient_rad` and `c_room_rad` (replaced by the R2C2 fabric node), plus all R2C2 parameters except `heater_power_watts_r2c2` (the radiator is the heater).

| Key | Description | Range | Default |
|---|---|---|---|
| `flow_temperature` | Boiler/heat pump flow temperature (°C) | 20–90 | 70 |
| `flow_temperature_entity` | HA entity providing live flow temperature | — | — |
| `c_radiator` | Radiator water + metal thermal mass (J/°C) | 500–100 000 | 8 000 |
| `k_radiator` | BS EN 442 emission coefficient (W/°C^n) | 0.1–500 | 10 |
| `radiator_exponent` | BS EN 442 exponent n | 1.0–2.0 | 1.3 |
| `radiator_convective_fraction` | Fraction of Q_out to air node (see table above) | 0.1–1.0 | 0.75 |
| `flow_rate_max_kg_s` | Maximum water mass flow rate (kg/s) | 0.001–1.0 | 0.05 |
| `pipe_delay_seconds` | Dead time to radiator (s) | 0–600 | 0 |
| `c_air` | Air + furniture thermal mass (J/°C) — **not just air** | 1 000–2 000 000 | 350 000 |
| `c_fabric` | Building fabric thermal mass (J/°C) | 10 000–50 000 000 | 5 000 000 |
| `r_fabric` | Internal surface resistance air↔fabric (°C/W) | 0.0001–1.0 | 0.005 |
| `r_ext` | Fabric↔outside resistance (°C/W) | 0.0001–50.0 | 0.020 |
| `r_infiltration` | Infiltration path air↔outside (°C/W) | 0.001–10.0 | 0.067 |
| `window_area_m2` | Glazed area facing the sun (m²) | 0–100 | 2.0 |
| `window_transmittance` | Solar heat gain coefficient (g-value) | 0–1 | 0.6 |
| `solar_irradiance_entity` | HA entity providing irradiance (W/m²) | — | — |
| `solar_irradiance_fixed` | Fixed fallback irradiance (W/m²) | 0–1 500 | 0.0 |

---

## Shared Configuration

These options appear on the first setup screen and apply to all models.

| Key | Description | Range | Default |
|---|---|---|---|
| `model_type` | `simple` / `r2c2` / `radiator` / `r2c2_radiator` | — | `simple` |
| `control_mode` | `linear` (0–100 % slider) or `pwm` (on/off switch) | — | `linear` |
| `initial_temperature` | Starting room temperature (°C) | — | 18.0 |
| `external_temperature_entity` | HA entity providing live outdoor temperature; takes priority over fixed value | — | — |
| `external_temperature_fixed` | Fixed fallback outdoor temperature (°C) | — | 5.0 |
| `update_interval_seconds` | Simulation tick rate (s). Lower = smoother, more CPU. | 1–300 | 10 |

---

## Behavioural Calibration

All four models support optional back-calculation of key thermal parameters from **observed room behaviour** instead of physical measurements. This is particularly useful when you can log how your room responds to heating and cooling but don't know the exact construction details.

The calibration fields use the first-order thermostat model:

```
dT/dt = a · u  −  b · (T − T_ext)
```

where `u = 1` when heating at full power, `u = 0` when off.

| Calibration field | Description | Units |
|---|---|---|
| `calib_a` | Heating rate at full power and steady radiator temperature | °C/min |
| `calib_b` | Loss rate coefficient (= 1/tau) | 1/min |
| `calib_tau` | Cooling time constant; takes priority over `calib_b` if both are set | min |

**Calibration fields are consumed on save.** Once processed, the back-calculated physical parameters are written into the normal parameter slots and the calibration fields are cleared. They do not persist.

### What gets back-calculated per model

| Model | From calibration | Derived parameters |
|---|---|---|
| Simple | `a` + `tau` or `b` + `heater_power` | `thermal_mass`, `heat_loss_coefficient` |
| R2C2 | `tau` or `b` (only; `a` is not used) | `r_ext` (via bisection on the slow system eigenvalue) |
| Radiator | `a` + `tau` or `b` | `c_room_rad`, `heat_loss_coefficient_rad` |
| R2C2+Radiator | `a` + `tau` or `b` | `c_air`, `r_ext` (preserving the existing c_fabric/c_air ratio) |

### Calibration procedure (Simple or Radiator models)

1. With the room at a stable temperature, open the valve / turn the heater fully on and log how fast the room warms: this gives `a` (°C/min).
2. Let the room cool from that stable point with heating off and log the time constant: this gives `tau` (min).
3. Enter `a` and `tau` in the options flow and save. The integration recalculates `thermal_mass` and `heat_loss_coefficient` (or `c_room_rad` and `heat_loss_coefficient_rad`) automatically.

For the **Radiator model**, `a` is measured with the radiator already at its operating temperature (warm flow water, valve fully open) so that `Q_out` is at its steady-state value. The back-calculation uses the configured `k_radiator`, `radiator_exponent`, `flow_temperature`, and `flow_rate_max` to determine that operating-point output before deriving the room parameters.

For the **R2C2 model**, only `tau` (the long-term cooling constant, typically measured over several hours) is used. `calib_a` is ignored. The integration finds `r_ext` by bisection on the slow eigenvalue of the two-node system matrix, leaving `c_air`, `c_fabric`, `r_fabric`, and `r_inf` unchanged.

---

## Installation

1. Copy `custom_components/heating_simulator/` into your HA `config/custom_components/`.
2. Restart Home Assistant.
3. **Settings → Devices & Services → Add Integration → Heating Simulator**
4. Step 1: choose name, model type, control mode, temperatures, and update interval.
5. Step 2: model-specific parameters (and optional calibration fields).

To reconfigure parameters without removing the entry: **Settings → Devices & Services → Heating Simulator → Configure**. Options always take precedence over the original setup values.

---

## State Restoration

Room temperature, fabric temperature (R2C2 models), and radiator temperature (radiator models) are restored from the HA state machine on restart using `RestoreSensor`. This means a simulation that has been running for hours will resume with realistic initial conditions rather than starting from `initial_temperature`.

---

## Sensor Imperfection

The `sensor.*_room_temperature` entity supports an optional post-processing pipeline that degrades the reported value to simulate real-world sensor behaviour. The thermal model always evolves at full precision; only the value reported to Home Assistant is affected. All stages are disabled when set to 0 (the default), preserving backwards-compatible behaviour.

The pipeline is configured in the second step of the options flow (**Settings → Devices & Services → Heating Simulator → Configure**).

### Pipeline stages (applied in order)

| Stage | Parameter | Unit | Default | Description |
|---|---|---|---|---|
| 1. Sensor lag | `sensor_lag_tau` | s | 0 | First-order low-pass filter time constant. Simulates the thermal mass of the sensor housing. α = dt / (τ + dt) per tick. |
| 2. Bias | `sensor_bias` | °C | 0 | Fixed additive offset. Simulates a miscalibrated sensor. A positive value causes the reported temperature to read high, which typically causes a controller to under-heat. |
| 3. Noise | `sensor_noise_std_dev` | °C | 0 | Gaussian noise standard deviation. Sampled independently each tick. |
| 4. Quantisation | `sensor_quantisation` | °C | 0 | Minimum reportable step. The value is rounded to the nearest multiple of this parameter (e.g. 0.5 → reported in 0.5 °C increments). |
| 5. Rate limit | `sensor_update_rate` | s | 0 | Minimum interval between state changes. If this interval has not elapsed since the last reported value, the previous value is re-emitted. Set to 0 to report on every simulation tick. |

### Companion entity

`sensor.*_true_room_temperature` exposes the raw model temperature before any pipeline stage. Use it alongside `sensor.*_room_temperature` to observe the effect of the active imperfections. This entity is always available and does not participate in state restoration.

### Known behaviours

**Sensor chattering.** When `sensor_noise_std_dev` is close to half the `sensor_quantisation` step (e.g. noise σ = 0.25 °C with 0.5 °C quantisation), the reported value may alternate rapidly between two adjacent quanta. This is physically realistic — it reflects real TRV behaviour — but may be surprising. Increase `sensor_update_rate` to suppress it.

**Apparent dead sensor with rate limiting.** When `sensor_update_rate` is set and the room temperature changes slowly, the sensor may hold the same value for extended periods. Home Assistant's state machine suppresses state-change events when the value is unchanged. Automations that trigger on `state_changed` will not fire during these periods. This is an intentional simulation of the Zigbee/Z-Wave TRV failure mode where slow polling causes controllers to perceive a frozen sensor.

### Example presets (for reference)

| Scenario | `sensor_lag_tau` | `sensor_bias` | `sensor_noise_std_dev` | `sensor_quantisation` | `sensor_update_rate` |
|---|---|---|---|---|---|
| Ideal (default) | 0 | 0 | 0 | 0 | 0 |
| Budget TRV | 0 | 0 | 0.5 | 0.5 | 60 |
| Smart TRV (Zigbee) | 10 | 0 | 0.1 | 0.1 | 30 |
| Miscalibrated sensor | 0 | +2.0 | 0.1 | 0 | 0 |

---

## Entities

### All models

| Entity | Description |
|---|---|
| `sensor.*_room_temperature` | Primary room air temperature (°C) |
| `sensor.*_heating_rate` | Heat input rate to air node (°C/s) |
| `sensor.*_heat_loss_rate` | Heat loss rate from air node (°C/s) |
| `sensor.*_net_heating_rate` | Net rate — positive = warming, negative = cooling (°C/s) |
| `sensor.*_effective_heater_power` | Heat actually delivered to air node (W) |
| `sensor.*_external_temperature` | External temperature used by model (°C) |
| `number.*_heater_power_input` | Power input 0–100 %; labelled "Valve Position" for radiator models |
| `number.*_external_temperature_override` | Live external temperature override (°C) |
| `switch.*_heater_switch_pwm` | On/off switch (PWM mode or hard override) |

### Simple model (additional)

| Entity | Description |
|---|---|
| `sensor.*_steady_state_temperature` | Steady-state temperature at current power and T_ext (°C); includes room time constant τ in attributes |

### R2C2 model (additional)

| Entity | Description |
|---|---|
| `sensor.*_fabric_temperature` | Building fabric / wall temperature (°C) |
| `sensor.*_solar_gain` | Solar heat gain through windows (W) |
| `sensor.*_total_heat_loss` | Combined infiltration + fabric loss (W); includes per-path breakdown in attributes |
| `number.*_solar_irradiance_override` | Solar irradiance override (W/m²) |

### Wet Radiator model (additional)

| Entity | Description |
|---|---|
| `sensor.*_radiator_temperature` | Radiator water temperature (°C) |
| `sensor.*_radiator_heat_input` | Q_in: heat from boiler into radiator (W) |
| `sensor.*_radiator_heat_output` | Q_out: heat from radiator to room (W); includes nominal output and output fraction in attributes |
| `sensor.*_radiator_return_temperature` | Estimated return water temperature using AMTD approximation, clamped to T_room (°C) |
| `number.*_flow_temperature_override` | Boiler flow temperature override (°C) |

### R2C2 + Radiator model (additional)

All R2C2 and all Wet Radiator additional entities above, combined.

---

## Usage Examples

### PWM thermostat (all models)

Point `climate.generic_thermostat` at:
- **heater**: `switch.*_heater_switch_pwm`
- **sensor**: `sensor.*_room_temperature`

### Modulating TRV / PID (radiator model)

Output your PID (0–100) to `number.*_valve_position`.
Read `sensor.*_room_temperature` as the process variable.
Observe `sensor.*_radiator_temperature` to see the thermal lag.

### Weather-compensated flow temperature

```yaml
automation:
  trigger:
    - platform: state
      entity_id: sensor.outdoor_temperature
  action:
    - service: number.set_value
      target:
        entity_id: number.heating_sim_flow_temperature_override
      data:
        # Weather compensation curve: 70°C at -5°C outside, 40°C at 15°C outside
        value: >
          {{ [[70, [40, (70 + (states('sensor.outdoor_temperature')|float - (-5))
              * (40-70)/(15-(-5)))]|max]|min] }}
```

### Simulating a sunny day

```yaml
service: number.set_value
target:
  entity_id: number.heating_sim_solar_irradiance_override
data:
  value: 600   # W/m² — bright winter sunshine
```

---

## Lovelace Dashboard (R2C2 + Radiator)

```yaml
type: entities
title: Heating Simulator
entities:
  - entity: sensor.heating_sim_room_temperature
  - entity: sensor.heating_sim_fabric_temperature
  - entity: sensor.heating_sim_radiator_temperature
  - entity: sensor.heating_sim_net_heating_rate
  - entity: sensor.heating_sim_radiator_heat_output
  - entity: sensor.heating_sim_solar_gain
  - entity: sensor.heating_sim_total_heat_loss
  - entity: number.heating_sim_valve_position
  - entity: number.heating_sim_flow_temperature_override
  - entity: number.heating_sim_solar_irradiance_override
  - entity: number.heating_sim_external_temperature_override
  - entity: switch.heating_sim_heater_switch_pwm
```

---

## Notes

- Config flow is multi-step: step 1 selects model type and shared settings; step 2 shows only that model's parameters.
- Calibration fields are available in both the initial setup flow and the options flow. They are consumed on save and do not persist.
- The wet radiator model's asymmetric heating/cooling is a core feature — cooling time constants are typically 3–5× longer than heating.
- R2C2 solar gain uses a fixed-area model with a 90/10 fabric/air split. For higher accuracy, feed a real irradiance sensor (many weather integrations provide `sensor.solar_irradiance`).
- Flow temperature override is particularly useful for testing heat pump weather compensation curves at different setpoints.
- The `c_air` parameter in all R2C2 models must represent **air plus all room contents** (furniture, carpet, soft furnishings), not the air mass alone. Air in 50 m³ ≈ 60 000 J/°C; a furnished room is typically 300 000–600 000 J/°C.
- Sub-stepping is applied automatically in all models to maintain numerical stability; no user configuration is required.
