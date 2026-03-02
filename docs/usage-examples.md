# Usage Examples

---

## Control Modes

### PWM thermostat (all models)

Connect `climate.generic_thermostat` to the simulator:

```yaml
climate:
  - platform: generic_thermostat
    name: Simulated Room
    heater: switch.heating_sim_heater_switch_pwm
    target_sensor: sensor.heating_sim_room_temperature
    min_temp: 15
    max_temp: 25
    target_temp: 21
    cold_tolerance: 0.3
    hot_tolerance: 0.3
```

---

### Modulating TRV / PID controller (Radiator or R2C2+Radiator models)

Write your PID output (0–100) to the valve position entity and read the room temperature as the process variable:

```yaml
# Example using the PID controller integration
pid:
  - platform: ...
    output: number.heating_sim_valve_position
    input: sensor.heating_sim_room_temperature
    setpoint: 21.0
    kp: 5.0
    ki: 0.01
    kd: 0.5
```

Observe `sensor.heating_sim_radiator_temperature` to see the thermal lag between valve position and room response.

---

## Automation Examples

### Weather-compensated flow temperature

Adjust boiler flow temperature based on outdoor conditions. This automation updates the override whenever the outdoor sensor changes:

```yaml
automation:
  - alias: Weather compensation curve
    trigger:
      - platform: state
        entity_id: sensor.outdoor_temperature
    action:
      - service: number.set_value
        target:
          entity_id: number.heating_sim_flow_temperature_override
        data:
          # Linear compensation: 70 °C at −5 °C outside, 40 °C at +15 °C outside
          value: >
            {% set t_out = states('sensor.outdoor_temperature') | float(5) %}
            {% set t_flow = 70 + (t_out - (-5)) * (40 - 70) / (15 - (-5)) %}
            {{ [40, [70, t_flow] | min] | max | round(1) }}
```

---

### Simulating a sunny day

Set solar irradiance to simulate different weather conditions:

```yaml
# Bright winter sunshine (south-facing, clear sky)
service: number.set_value
target:
  entity_id: number.heating_sim_solar_irradiance_override
data:
  value: 600   # W/m²

# Overcast day (no useful solar gain)
service: number.set_value
target:
  entity_id: number.heating_sim_solar_irradiance_override
data:
  value: 30    # W/m²
```

---

### Simulating a windy, wet day

Use the `set_weather` service to apply wind and rain effects (requires `weather_wind_coefficient` and `weather_rain_moisture_factor` to be set to non-zero values in the options):

```yaml
service: heating_simulator.set_weather
target:
  device_id: <your_device_id>
data:
  wind_speed_m_s: 12.0         # Beaufort 6 — strong breeze
  rain_intensity_fraction: 0.8  # Heavy rain
```

Restore to calm conditions:

```yaml
service: heating_simulator.set_weather
target:
  device_id: <your_device_id>
data:
  wind_speed_m_s: 0.0
  rain_intensity_fraction: 0.0
```

---

### Testing cold-start behaviour

Reset the simulation to a fully cold state to observe first warm-up characteristics:

```yaml
service: heating_simulator.reset_model
target:
  device_id: <your_device_id>
data:
  preset: cold_start
```

Or set specific initial conditions for reproducible test runs:

```yaml
service: heating_simulator.reset_model
target:
  device_id: <your_device_id>
data:
  t_room: 10.0    # Very cold room
  t_fabric: 6.0   # Cold walls
  t_rad: 10.0     # Cold radiator
```

---

### Testing a setback / boost scenario

Script to test overnight setback and morning boost:

```yaml
script:
  test_setback_boost:
    sequence:
      # Simulate overnight: setback to 16 °C, room has cooled
      - service: heating_simulator.reset_model
        target:
          device_id: <device_id>
        data:
          preset: overnight
      # Set thermostat to setback temperature
      - service: climate.set_temperature
        target:
          entity_id: climate.simulated_room
        data:
          temperature: 16
      - delay: "00:30:00"   # Wait 30 simulated minutes
      # Morning boost — raise to comfort temperature
      - service: climate.set_temperature
        target:
          entity_id: climate.simulated_room
        data:
          temperature: 21
```

---

## Lovelace Dashboard

A suggested dashboard for the R2C2 + Radiator model showing all key state variables:

```yaml
type: entities
title: Heating Simulator
entities:
  - entity: sensor.heating_sim_room_temperature
    name: Room temperature (sensor)
  - entity: sensor.heating_sim_true_room_temperature
    name: True room temperature
  - entity: sensor.heating_sim_fabric_temperature
    name: Fabric (wall) temperature
  - entity: sensor.heating_sim_radiator_temperature
    name: Radiator temperature
  - entity: sensor.heating_sim_net_heating_rate
    name: Net heating rate
  - entity: sensor.heating_sim_radiator_heat_output
    name: Radiator output
  - entity: sensor.heating_sim_solar_gain
    name: Solar gain
  - entity: sensor.heating_sim_total_heat_loss
    name: Total heat loss
  - entity: number.heating_sim_valve_position
    name: Valve position
  - entity: number.heating_sim_flow_temperature_override
    name: Flow temperature override
  - entity: number.heating_sim_solar_irradiance_override
    name: Solar irradiance override
  - entity: number.heating_sim_external_temperature_override
    name: External temperature override
  - entity: switch.heating_sim_heater_switch_pwm
    name: Heater switch (PWM)
```

For a graphical view of temperature trends, add a history-graph card:

```yaml
type: history-graph
title: Temperature history
entities:
  - entity: sensor.heating_sim_room_temperature
    name: Room (sensor)
  - entity: sensor.heating_sim_true_room_temperature
    name: Room (true)
  - entity: sensor.heating_sim_fabric_temperature
    name: Fabric
  - entity: sensor.heating_sim_radiator_temperature
    name: Radiator
  - entity: sensor.heating_sim_external_temperature
    name: External
hours_to_show: 6
```

---

## Typical Testing Workflows

### 1. Validate a hysteresis thermostat

1. Use the **Simple** model, `pwm` control mode.
2. Configure `climate.generic_thermostat` pointing at `switch.*_heater_switch_pwm` and `sensor.*_room_temperature`.
3. Set `update_interval_seconds = 1` for fast feedback.
4. Observe the switching pattern and temperature band.

### 2. Tune a PID controller

1. Use the **Wet Radiator** model, `linear` control mode.
2. Drive `number.*_valve_position` from your PID output.
3. Start with `preset: cold_start`, set a comfort setpoint, and observe the step response.
4. Tune PID gains to minimise overshoot and settling time.
5. Test disturbance rejection by applying a solar gain spike or changing external temperature.

### 3. Evaluate heat pump performance

1. Use the **R2C2 + Radiator** model.
2. Set `flow_temperature = 40` (low temperature heat pump).
3. Size `k_radiator` for your actual radiators (remember: output at ΔT20 is ~28% of nameplate).
4. Compare room warm-up time and peak output against a 70 °C boiler scenario.
5. Test weather-compensated flow temperature using the automation example above.

### 4. Test sensor quality impact

1. Use any model.
2. Configure your thermostat to use `sensor.*_room_temperature` (degraded).
3. Monitor both `sensor.*_room_temperature` and `sensor.*_true_room_temperature`.
4. Apply sensor presets (Budget TRV, Zigbee TRV) and observe the impact on control quality.
5. Increase sensor noise and observe chattering; apply `sensor_update_rate` to suppress it.

### 5. Quantify solar gain benefit

1. Use the **R2C2** or **R2C2 + Radiator** model.
2. Disable heating (valve = 0).
3. Set `solar_irradiance_override` to 600 W/m² and observe how quickly the room warms.
4. For a 3 m² south-facing window with g = 0.6: peak solar gain = 3 × 0.6 × 600 = 1 080 W.
5. Compare this to your heater power to assess the solar contribution on sunny winter days.
