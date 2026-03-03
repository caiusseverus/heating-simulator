# Entity Reference

All entities use the integration name as a prefix (e.g. `sensor.heating_sim_room_temperature` if the entry is named "Heating Sim").

---

## Sensor Entities

### All models

| Entity | Description | Unit |
|---|---|---|
| `sensor.*_room_temperature` | Primary room air temperature, after sensor imperfection pipeline | °C |
| `sensor.*_true_room_temperature` | Raw model temperature before any sensor imperfection pipeline stage | °C |
| `sensor.*_heating_rate` | Rate of heat input to the air node | °C/s |
| `sensor.*_heat_loss_rate` | Rate of heat loss from the air node | °C/s |
| `sensor.*_net_heating_rate` | Net rate of temperature change: positive = warming, negative = cooling | °C/s |
| `sensor.*_effective_heater_power` | Heat actually delivered to the air node (W) | W |
| `sensor.*_external_temperature` | External temperature currently used by the model | °C |

`sensor.*_true_room_temperature` is always available and does not participate in state restoration. Use it alongside `sensor.*_room_temperature` to observe the effect of any active sensor imperfections.

---

### Simple model (additional entities)

| Entity | Description | Unit |
|---|---|---|
| `sensor.*_steady_state_temperature` | Equilibrium temperature the room would reach at current power and T_ext | °C |

The `steady_state_temperature` entity includes the following attributes:

| Attribute | Description |
|---|---|
| `room_tau_s` | Room time constant C/K in seconds |
| `heater_inertia_tau_s` | Heater lag time constant τ in seconds |
| `internal_gain_w` | Current internal gain from occupancy/cooking (W) |
| `weather_k_multiplier` | Current weather multiplier on K |

---

### R2C2 model (additional entities)

| Entity | Description | Unit |
|---|---|---|
| `sensor.*_fabric_temperature` | Building fabric (walls/floor/ceiling) temperature | °C |
| `sensor.*_solar_gain` | Total solar heat gain through windows | W |
| `sensor.*_total_heat_loss` | Combined heat loss through all paths | W |

`sensor.*_total_heat_loss` includes per-path breakdown in attributes:

| Attribute | Description |
|---|---|
| `infiltration_loss_w` | Heat loss through infiltration path (W) |
| `fabric_loss_w` | Heat loss through fabric path (W) |
| `effective_u_value_W_per_C` | Overall heat loss coefficient at current state |
| `fabric_to_air_flux_w` | Heat flux from fabric to air (positive = warming air) |
| `weather_k_multiplier` | Current weather multiplier |

---

### Wet Radiator model (additional entities)

| Entity | Description | Unit |
|---|---|---|
| `sensor.*_radiator_temperature` | Radiator mean water/metal temperature | °C |
| `sensor.*_radiator_heat_input` | Q_in: heat transferred from boiler into the radiator | W |
| `sensor.*_radiator_heat_output` | Q_out: heat emitted by the radiator to the room | W |
| `sensor.*_radiator_return_temperature` | Estimated radiator return water temperature (AMTD approximation, clamped to T_room) | °C |

`sensor.*_radiator_heat_output` includes additional attributes:

| Attribute | Description |
|---|---|
| `nominal_output_w_dt50` | Radiator nameplate output at standard ΔT50 (W) |
| `output_fraction` | Current output as a fraction of nominal ΔT50 output |

---

### R2C2 + Radiator model (additional entities)

All R2C2 additional entities and all Wet Radiator additional entities, combined. The `sensor.*_radiator_heat_output` entity exposes the same two attributes as in the plain Wet Radiator model.

---

## Number (Input) Entities

These entities allow you to drive the simulation and override parameters.

### All models

| Entity | Description | Range | Unit |
|---|---|---|---|
| `number.*_heater_power_input` | Heater power or valve position input (linear mode). Labelled "Valve Position" for radiator models. | 0–100 | % |
| `number.*_external_temperature_override` | Live external temperature override. Takes priority over both the configured entity and fixed value while set to a non-default value. | −40 to 60 | °C |

---

### R2C2 model (additional)

| Entity | Description | Range | Unit |
|---|---|---|---|
| `number.*_solar_irradiance_override` | Solar irradiance override. Writes directly to the model; the configured entity or fixed value will resume control on its next state-change event. | 0–1 500 | W/m² |

---

### Wet Radiator and R2C2 + Radiator models (additional)

| Entity | Description | Range | Unit |
|---|---|---|---|
| `number.*_flow_temperature_override` | Boiler flow temperature override (°C). Overrides both entity and fixed value. | 20–90 | °C |

---

## Switch Entities

### All models

| Entity | Description |
|---|---|
| `switch.*_heater_switch_pwm` | On/off heater control. In PWM mode, this is the primary control input. In linear mode, switching this off forces heater power to 0 regardless of the number entity (hard override). |

---

## State Restoration

The following entities participate in state restoration via `RestoreSensor`:

| Model | Restored entities |
|---|---|
| Simple | `sensor.*_room_temperature` |
| R2C2 | `sensor.*_room_temperature`, `sensor.*_fabric_temperature` |
| Wet Radiator | `sensor.*_room_temperature`, `sensor.*_radiator_temperature` |
| R2C2 + Radiator | `sensor.*_room_temperature`, `sensor.*_fabric_temperature`, `sensor.*_radiator_temperature` |

On restart, the simulation resumes from the last reported state rather than from `initial_temperature`. The `sensor.*_true_room_temperature` entity does **not** participate in state restoration.
