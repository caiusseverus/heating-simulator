# Heating Simulator v2 — Home Assistant Integration

A custom integration providing three physics-based room thermal models for testing and tuning thermostat controllers, TRV automations, PID loops, and smart heating systems — without needing real hardware.

---

## Models

### 1. Simple (R1C1)
Single thermal mass with optional first-order heater lag. Good for quick tests and simple thermostat validation.

```
C · dT/dt = Q_heater(t) − K · (T − T_ext)
```
Heater lag: `τ · dQ_eff/dt = Q_input − Q_eff` (symmetric, instant if τ=0)

---

### 2. R2C2 + Solar Gain
Two-node model — air and building fabric — with solar gain through windows. Based on the ISO 13790 simplified hourly method.

```
C_air · dT_air/dt  = Q_heater + Q_solar + (T_fab−T_air)/R_fab − (T_air−T_ext)/R_inf
C_fab · dT_fab/dt  = (T_air−T_fab)/R_fab − (T_fab−T_ext)/R_ext
```

**Why this matters:**
- Cold walls slow down the first-warm-up of a room even after the air is warm
- Solar gain through windows can eliminate the need for heating on sunny days
- Infiltration (draughts) and fabric loss are independent paths — you can model a draughty well-insulated house
- The fabric node acts as thermal storage — the room cools slowly after heating stops

**Parameter guide:**

| Parameter | Meaning | Typical values |
|---|---|---|
| `C_air` | Air + lightweight furniture thermal mass (J/°C) | 3000–10000 |
| `C_fabric` | Walls, floor, ceiling thermal mass (J/°C) | 30000–300000 |
| `R_fabric` | Internal surface resistance, air↔fabric (°C/W) | 0.002–0.01 |
| `R_ext` | Fabric↔outside resistance (°C/W) | 0.01 = very lossy, 0.05 = well insulated |
| `R_inf` | Infiltration/ventilation path air↔outside (°C/W) | 0.03–0.2 |
| `Window area` | Total glazed area facing sun (m²) | 1–10 |
| `Transmittance` | Solar heat gain coefficient (g-value) | 0.4–0.7 |

**Sizing R_ext:** `R_ext = 1 / K_fabric` where K_fabric is your fabric heat loss in W/°C.  
For a room losing 40 W/°C through the fabric: `R_ext = 1/40 = 0.025 °C/W`

---

### 3. Wet Radiator
Explicit radiator water temperature with BS EN 442 power law output. Models the physical asymmetry between heating (driven by boiler ΔT) and cooling (passive decay to room temperature).

```
C_rad  · dT_rad/dt  = Q_in − Q_out
C_room · dT_room/dt = Q_out − K_loss · (T_room − T_ext)

Q_in  = valve × ṁ_max × C_water × max(0, T_flow − T_rad)
Q_out = K_rad × |T_rad − T_room|^n    (n ≈ 1.3, BS EN 442)
```

**Why this matters:**
- Heating is fast (limited by boiler ΔT ~50°C × flow rate)
- Cooling is slow (governed by shrinking ΔT between radiator and room)
- Output drops as the room warms up, even at full valve — the output is proportional to `ΔT^1.3`, not valve position
- At low flow temperatures (heat pump operation, 35–45°C) the output is dramatically reduced
- Valve position → mass flow → Q_in is a non-linear chain, not a simple multiplier

**Key parameters:**

| Parameter | Meaning | Typical values |
|---|---|---|
| `Flow temperature` | Boiler/heat pump flow temp (°C) | 35 (ASHP), 55 (ASHP high), 70 (condensing boiler) |
| `C_radiator` | Radiator water + metal thermal mass (J/°C) | 3000 (small single panel), 15000 (large double panel) |
| `K_radiator` | BS EN 442 emission coefficient | `P_nominal / 50^1.3` — see below |
| `Radiator exponent n` | Panel: 1.3, convector: 1.5 | 1.3 |
| `Max flow rate` | kg/s at fully open valve | 0.03–0.1 |
| `Heat loss coefficient` | Room K-value (W/°C) | 20–200 |
| `Pipe delay` | Dead time for hot water to arrive (s) | 0–120 |

**Sizing K_radiator:**  
If your radiator is rated at 1500 W at ΔT50 (70°C flow, 20°C room):
```
K_rad = 1500 / (50^1.3) = 1500 / 142.5 ≈ 10.5 W/°C^1.3
```

---

## Installation

1. Copy `custom_components/heating_simulator/` into your HA `config/custom_components/`.
2. Restart Home Assistant.
3. **Settings → Devices & Services → Add Integration → Heating Simulator**
4. Step 1: choose name, model type, control mode, temperatures.
5. Step 2: model-specific parameters.

---

## Entities

### All models

| Entity | Description |
|---|---|
| `sensor.*_room_temperature` | Primary room air temperature (°C) |
| `sensor.*_heating_rate` | Heat input rate (°C/s) |
| `sensor.*_heat_loss_rate` | Heat loss rate (°C/s) |
| `sensor.*_net_heating_rate` | Net rate — positive = warming, negative = cooling |
| `sensor.*_effective_heater_power` | Heat actually reaching the air node (W) |
| `sensor.*_external_temperature` | External temperature used by model (°C) |
| `number.*_heater_power_input` | Power input 0–100% (or valve position for radiator) |
| `number.*_external_temperature_override` | Live external temperature override |
| `switch.*_heater_switch_pwm` | On/off switch (PWM mode or hard override) |

### R2C2 model (additional)

| Entity | Description |
|---|---|
| `sensor.*_fabric_temperature` | Building fabric / wall temperature (°C) |
| `sensor.*_solar_gain` | Solar heat gain through windows (W) |
| `sensor.*_total_heat_loss` | Combined infiltration + fabric loss (W) |
| `number.*_solar_irradiance_override` | Solar irradiance override (W/m²) |

### Wet Radiator model (additional)

| Entity | Description |
|---|---|
| `sensor.*_radiator_temperature` | Radiator water temperature (°C) |
| `sensor.*_radiator_heat_input` | Q_in: heat from boiler into radiator (W) |
| `sensor.*_radiator_heat_output` | Q_out: heat from radiator to room (W) |
| `sensor.*_radiator_return_temperature` | Estimated return water temperature (°C) |
| `number.*_flow_temperature_override` | Boiler flow temperature override (°C) |

---

## Usage examples

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
          {{ [70, [40, (70 + (states('sensor.outdoor_temperature')|float - (-5)) * (40-70)/(15-(-5))) ]|max]|min }}
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

## Lovelace dashboard (R2C2 + Radiator)

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
  - entity: number.heating_sim_valve_position
  - entity: number.heating_sim_flow_temperature_override
  - entity: number.heating_sim_solar_irradiance_override
  - entity: number.heating_sim_external_temperature_override
  - entity: switch.heating_sim_heater_switch_pwm
```

---

## Notes

- Config flow is multi-step: step 1 selects model type, step 2 shows only that model's parameters.
- All models reset to initial temperature on HA restart (state is in-memory only).
- The wet radiator model's asymmetric heating/cooling is a key feature — cooling time constants are typically 3–5× longer than heating.
- R2C2 solar gain uses a simple fixed-area model. For more accuracy, feed a real irradiance sensor (many weather integrations provide this).
- Flow temperature override is particularly useful for testing heat pump weather compensation curves at different setpoints.
