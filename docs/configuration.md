# Configuration Reference

All configuration is done through the Home Assistant UI:

- **Initial setup:** Settings → Devices & Services → Add Integration → Heating Simulator
- **Reconfigure:** Settings → Devices & Services → Heating Simulator → Configure

The config flow is two-step: step 1 collects shared settings (including model selection), step 2 shows the parameters specific to the chosen model, plus sensor imperfection and disturbance options.

---

## Shared Parameters

These appear on the first setup screen and apply to all models.

### `model_type`

Which thermal model to use.

| Value | Model |
|---|---|
| `simple` | R1C1 single-node with optional heater lag |
| `r2c2` | Two-node room with solar gain (ISO 13790) |
| `radiator` | Wet radiator with explicit water temperature |
| `r2c2_radiator` | Full combined: wet radiator + R2C2 room + solar |

**Recommendation:** Start with `simple` to validate your control logic, then move to `radiator` or `r2c2_radiator` for realistic system behaviour.

---

### `control_mode`

How the heater/valve is controlled.

| Value | Description |
|---|---|
| `linear` | A 0–100% number entity (`number.*_heater_power_input`). Use this for PID controllers, modulating TRVs, or any proportional output. |
| `pwm` | A binary switch entity (`switch.*_heater_switch_pwm`). Use this with `climate.generic_thermostat` or any on/off thermostat. |

Both modes also expose a switch entity that acts as a hard override (forcing the heater off regardless of the number entity).

**Recommendation:** Use `linear` for PID tuning. Use `pwm` for thermostat testing.

---

### `initial_temperature`

Starting room temperature (°C) when the integration first loads, or after a `reset_model` service call without a temperature argument.

**Default:** 18.0 °C. Set to your expected room temperature to avoid a large initial transient.

---

### `external_temperature_entity`

A Home Assistant entity (e.g. `sensor.outdoor_temperature`) that provides the current external temperature in °C. When this entity has a valid numeric state, it takes priority over `external_temperature_fixed`. Updated on every simulation tick.

Leave blank to use the fixed value only.

---

### `external_temperature_fixed`

Fixed external temperature (°C) used when no live entity is configured, or when the entity is unavailable.

**Default:** 5.0 °C (typical UK winter morning).

---

### `update_interval_seconds`

How often the simulation advances (seconds). This is the simulation tick rate.

| Value | Effect |
|---|---|
| 1–5 s | High resolution; smooth sensor readings; higher CPU usage |
| 10 s | Default; good balance for most automation testing |
| 30–60 s | Low CPU; adequate for slow-response testing |
| 300 s | Minimum load; suitable for very slow models (large C/K) |

**Default:** 10 s. Sub-stepping within each tick is handled automatically by the models to maintain numerical stability regardless of this setting.

---

## Simple Model Parameters

See [models.md — Simple (R1C1)](models.md#model-1--simple-r1c1) for the underlying mathematics.

### `heater_power_watts`

Nominal heater output at 100% input (W). The simulation scales linearly: at 50% input, 50% of this wattage is delivered (after any lag).

**Default:** 2 000 W. Match your actual heater rating.

**Typical values:**
- Small electric panel heater: 500–1 000 W
- Standard electric convector / oil-filled radiator: 1 500–2 500 W
- Underfloor heating zone: 1 000–3 000 W

---

### `heat_loss_coefficient`

The room's K-value: total heat loss rate per degree above external temperature (W/°C). Combines fabric conduction and infiltration in a single figure.

**Default:** 50 W/°C (moderate UK domestic room).

**Estimation method:**
- If you know the room reaches steady state at temperature T_ss with heater at power Q and external temperature T_ext:
  `K = Q / (T_ss − T_ext)`
- Alternatively, CIBSE Guide A gives U-values per element; sum the area-weighted losses.

**Typical values:**
- Well-insulated modern room (20 m²): 20–35 W/°C
- Average UK semi-detached room (20 m²): 40–70 W/°C
- Poorly insulated or large room: 80–150 W/°C

---

### `thermal_mass`

Room thermal capacitance C (J/°C). Determines how quickly the room temperature responds to heating and cooling.

**Default:** 350 000 J/°C (furnished 20 m² room).

**Estimation:**
- Air in 50 m³ room: ~60 000 J/°C
- Furniture, carpet, soft furnishings: add 200 000–400 000 J/°C
- Total furnished room: typically 300 000–600 000 J/°C

**Effect:** Higher C means slower response (larger room time constant `τ = C/K`). Increasing C to 500 000 J/°C with K = 50 W/°C gives `τ = 10 000 s ≈ 2.8 hours`.

---

### `thermal_inertia`

First-order lag time constant τ on heater output (seconds). Models the thermal inertia of an electric element heating up, or the delay of air flowing through a convector.

**Default:** 0 (instant response).

**When to use:** Set to 60–300 s for an electric panel heater or convector that takes a minute or two to reach full output after switching on. Leave at 0 for a direct electric element (e.g. underfloor) or for simplicity.

---

## R2C2 Model Parameters

See [models.md — R2C2](models.md#model-2--r2c2-with-solar-gain) for the underlying mathematics.

### `heater_power_watts_r2c2`

Same meaning as `heater_power_watts` in the simple model. Heat is injected directly into the air node.

**Default:** 2 000 W.

---

### `c_air`

Thermal mass of the air node (J/°C). **Must include air plus all room contents** that warm up on the same timescale as the air: furniture, carpet, curtains, books.

**Default:** 350 000 J/°C.

Do not use only the air mass (≈60 000 J/°C for 50 m³). This produces unrealistically fast air temperature responses. A furnished room is typically 5–10× the air mass alone.

---

### `c_fabric`

Thermal mass of the building fabric: walls, floor, and ceiling only (J/°C). This is separate from `c_air`.

**Default:** 5 000 000 J/°C (brick cavity construction, ~60 m² of surfaces).

**Estimation:**
- Mass of surface layers: area (m²) × surface density (kg/m²) × specific heat (J/kg·°C)
- Brick cavity (heavy): ~150 kg/m² × 840 J/kg·°C × 60 m² ≈ 7 500 000 J/°C
- Timber frame / plasterboard (light): ~20 kg/m² × 1 000 J/kg·°C × 60 m² ≈ 1 200 000 J/°C

**Effect:** High `c_fabric` means the walls are slow to warm up after a cold start, and slow to cool overnight. This is realistic for brick construction.

---

### `r_fabric`

Internal surface thermal resistance between the air node and the fabric node (°C/W). Represents convection and radiation across the internal air-to-surface boundary.

**Default:** 0.005 °C/W (200 W/°C coupling; appropriate for a typical room).

**Interpretation:** `1/R_fab` is the total internal surface conductance. Standard internal surface resistance (ISO 6946) is 0.13 m²·°C/W per unit area; for 60 m² of surfaces, this gives `R_fab = 0.13/60 ≈ 0.002 °C/W`. Values of 0.002–0.010 are physically reasonable.

**Effect:** Higher `r_fabric` slows the exchange between air and fabric. This makes the air temperature respond faster to the heater (fabric does not buffer it), but the fabric warm-up is delayed.

---

### `r_ext`

Thermal resistance between the fabric node and the external environment (°C/W). Represents conduction through walls, roof, and floor.

**Default:** 0.020 °C/W (50 W/°C fabric loss; moderate insulation).

**Sizing:** `R_ext = 1 / K_fabric` where K_fabric is the fabric-only heat loss rate (W/°C).

| Insulation level | K_fabric (W/°C) | R_ext (°C/W) |
|---|---|---|
| Poor (solid brick, uninsulated) | 80–120 | 0.008–0.012 |
| Moderate (cavity wall, no insulation) | 40–70 | 0.014–0.025 |
| Good (cavity + insulation, double glazing) | 20–40 | 0.025–0.050 |
| Excellent (Passivhaus standard) | 5–15 | 0.067–0.200 |

---

### `r_infiltration`

Thermal resistance of the infiltration and ventilation path from the air node to outside (°C/W).

**Default:** 0.067 °C/W (≈0.5 ACH in a 100 m³ room; ~15 W/°C infiltration loss).

**Sizing:** At air change rate ACH (h⁻¹) in volume V (m³):

```
heat capacity flow = (V × ACH / 3600) × 1200  [W/°C]
R_inf = 1 / heat capacity flow
```

| ACH | R_inf (°C/W) for 100 m³ |
|---|---|
| 0.3 | 0.111 |
| 0.5 | 0.067 |
| 1.0 | 0.033 |
| 2.0 | 0.017 |

Draughty older homes are often 1–2 ACH; modern well-sealed buildings may be 0.1–0.3 ACH.

---

### `window_area_m2`

The glazed area that receives direct solar irradiance (m²). For a south-facing room in the Northern Hemisphere, use the south-facing window area. For other orientations, apply a correction factor (roughly: south = 1.0, east/west = 0.6, north = 0.1).

**Default:** 2.0 m².

---

### `window_transmittance`

Solar heat gain coefficient (g-value or SHGC): fraction of incident solar irradiance that enters the room as heat.

**Default:** 0.6.

**Typical values:**
- Standard double glazing: 0.6–0.7
- Low-emissivity (low-e) double glazing: 0.3–0.5
- Triple glazing: 0.4–0.6
- Laminated / tinted glass: 0.2–0.4

---

### `solar_irradiance_entity`

A Home Assistant entity providing current solar irradiance (W/m²). Many weather integrations supply this (e.g. `sensor.solar_irradiance`, or calculated from position + DNI data). Takes priority over `solar_irradiance_fixed` when available.

---

### `solar_irradiance_fixed`

Fixed solar irradiance (W/m²) used when no live entity is configured.

**Default:** 0.0 (solar gain disabled).

**Representative values:**
- Overcast UK winter day: 0–50 W/m²
- Bright UK winter day: 200–400 W/m²
- Clear UK summer midday (south-facing): 600–900 W/m²
- Peak global horizontal irradiance: ~1 000 W/m²

---

## Wet Radiator Model Parameters

See [models.md — Wet Radiator](models.md#model-3--wet-radiator) for the underlying mathematics.

### `flow_temperature`

Boiler or heat pump flow (supply) temperature (°C). The temperature of water entering the radiator circuit.

**Default:** 70 °C (traditional gas boiler).

**Typical values:**
- Traditional gas boiler: 65–80 °C
- Condensing boiler (weather-compensated): 40–70 °C
- Air source heat pump: 35–50 °C
- Ground source heat pump: 40–55 °C

**Note:** At low flow temperatures, radiator output is significantly reduced. At 40 °C flow / 20 °C room (ΔT = 20 °C), a panel radiator rated at ΔT50 delivers only about 28% of its nameplate output.

---

### `flow_temperature_entity`

A Home Assistant entity providing live boiler flow temperature (°C). Overrides `flow_temperature` when available. Useful for testing weather-compensated systems.

---

### `c_radiator`

Thermal mass of the radiator: water content plus metal (J/°C).

**Default:** 8 000 J/°C.

**Estimation:**
- Water content (L) × 4 182 J/L·°C
- Steel/iron mass (kg) × 500 J/kg·°C
- Small single-panel radiator (600×600 mm): ~3 000 J/°C
- Large double-panel + fins (1 200×600 mm): ~12 000–18 000 J/°C

**Effect:** Lower `c_radiator` means the radiator heats up and cools down faster. This affects the initial rise time after the valve opens and the tail-off after it closes.

---

### `k_radiator`

BS EN 442 emission coefficient (W/°C^n). Sized so that at standard ΔT50 the output equals the radiator's nameplate power:

```
K_rad = P_nominal / (50)^n
```

For `n = 1.3`: `50^1.3 ≈ 142.5`.

**Default:** 10 W/°C^1.3 (roughly a 1 400 W radiator at ΔT50).

**Examples:**

| Nameplate power | K_rad (n=1.3) |
|---|---|
| 500 W | 3.5 |
| 1 000 W | 7.0 |
| 1 500 W | 10.5 |
| 2 500 W | 17.5 |

---

### `radiator_exponent`

BS EN 442 heat emission exponent n.

**Default:** 1.3 (panel radiators).

| Radiator type | Exponent n |
|---|---|
| Panel radiator (Type 10/11/21/22) | 1.3 |
| Fan convector | 1.5 |
| Underfloor heating | 1.1 |

---

### `flow_rate_max_kg_s`

Maximum water mass flow rate through the radiator at full valve opening (kg/s).

**Default:** 0.05 kg/s (≈3 L/min).

**Typical values:**
- Small TRV radiator, 15 mm pipe: 0.03–0.06 kg/s
- Medium domestic radiator, 22 mm pipe: 0.05–0.10 kg/s

**Effect:** A higher flow rate charges the radiator faster (faster initial heating) and supports higher maximum heat output. Reducing this simulates a partially restricted or small-bore pipe.

---

### `heat_loss_coefficient_rad`

Room K-value (W/°C) for the Wet Radiator model. Same meaning as `heat_loss_coefficient` in the Simple model.

**Default:** 50 W/°C.

---

### `c_room_rad`

Room thermal mass (J/°C) for the Wet Radiator model. Same meaning as `thermal_mass` in the Simple model — must include air plus all room contents.

**Default:** 500 000 J/°C.

---

### `pipe_delay_seconds`

Dead time (seconds) from a valve position change to when hot water actually arrives at the radiator. Modelled as a 1-second-resolution FIFO queue.

**Default:** 0 (no delay).

**When to set:** Use if you want to simulate the real lag in long pipe runs — for example, a radiator at the far end of a long circuit. Estimate from pipe volume and flow rate (see [models.md — Wet Radiator](models.md#model-3--wet-radiator)).

---

### `valve_characteristic`

The relationship between valve position and water flow rate.

| Value | Description |
|---|---|
| `linear` | Flow proportional to position — default |
| `quick_opening` | Piecewise-linear TRV crack-open curve: nearly sealed below 2%, rapid opening between 3–5%, saturating above ~14% at ~89% flow |

**Default:** `linear`.

**When to use `quick_opening`:** Select this when you want to simulate a real thermostatic radiator valve (TRV). Actual TRVs are quick-opening types: the valve seat is nearly sealed at small openings and then cracks open suddenly. This means a controller using small valve modulation steps near the closed end achieves much larger flow changes than a linear model would predict, and partial valve positions do not give proportional heat output. Use `linear` for simpler proportional control testing or when modelling a motorised modulating valve.

---

## R2C2 + Radiator Parameters

This model uses all parameters from the Wet Radiator and R2C2 sections above, with two exceptions:
- `heat_loss_coefficient_rad` and `c_room_rad` are **not used** (replaced by the R2C2 fabric node).
- `heater_power_watts_r2c2` is **not used** (the radiator is the heater).

This includes `valve_characteristic` and `pipe_delay_seconds` from the Wet Radiator section.

One additional parameter is added:

### `radiator_convective_fraction`

Fraction of total radiator output directed to the air node. The remainder goes to the fabric node as longwave radiation.

**Default:** 0.75 (Type 21/22 double-panel radiator).

| Radiator type | Recommended value |
|---|---|
| Type 10 (single panel, no fins) | 0.50 |
| Type 11 (single panel + fins) | 0.65 |
| Type 21 (double panel + fins) | 0.75 |
| Type 22 (double panel, 2× fins) | 0.80 |
| Fan coil / convector | 0.90–0.95 |
| Underfloor heating (water) | 0.50 |

---

## Sensor Imperfection

The primary room temperature sensor (`sensor.*_room_temperature`) includes an optional pipeline that degrades the reported value to simulate real-world sensor behaviour. The thermal model runs at full precision; only the reported value is affected.

All stages are disabled at their default values of 0, preserving backwards-compatible behaviour.

A companion entity `sensor.*_true_room_temperature` always reports the raw model value before any pipeline stage.

### Pipeline stages (applied in order)

**Stage 1 — Sensor lag (`sensor_lag_tau`, seconds)**

A first-order low-pass filter on the reported value. Simulates the thermal mass of the sensor housing — a sensor enclosed in plastic takes time to reach the true air temperature.

```
α = dt / (τ + dt)
T_reported(t) = T_reported(t−1) + α × (T_true − T_reported(t−1))
```

**Default:** 0 (disabled). **Recommended values:**

- Zigbee TRV sensor in plastic housing: 10–30 s
- Sensor mounted on cold wall: 30–120 s
- Ideal sensor: 0

---

**Stage 2 — Bias (`sensor_bias`, °C)**

A fixed additive offset to the reported temperature. Simulates a miscalibrated sensor. A positive bias causes the sensor to read high, which makes a controller under-heat (the controller thinks the room is warmer than it is).

**Default:** 0 °C. **Recommended values:**

- Sensor near cold external wall: −1.0 to −2.0 °C (reads low)
- Miscalibrated budget TRV: ±1–3 °C
- Ideal: 0

---

**Stage 3 — Noise (`sensor_noise_std_dev`, °C)**

Gaussian white noise added to the reported value on every tick. Simulates electrical noise and quantisation variation.

**Default:** 0 °C. **Recommended values:**

- Ideal sensor: 0
- Zigbee TRV: 0.1–0.2 °C σ
- Budget TRV: 0.3–0.5 °C σ

**Note:** If noise σ is close to half the quantisation step, the reported value may chatter between two adjacent quanta. This is physically realistic but may be surprising. Use `sensor_update_rate` or the Zigbee delta threshold (Stage 6) to suppress it.

---

**Stage 4 — Quantisation (`sensor_quantisation`, °C)**

The minimum reportable temperature increment. The reported value is rounded to the nearest multiple of this parameter.

**Default:** 0 (disabled — continuous). **Recommended values:**

- High-resolution sensor: 0.01–0.05 °C
- Zigbee TRV: 0.1 °C
- Budget TRV: 0.5 °C

---

**Stage 5 — Fixed rate limit (`sensor_update_rate`, seconds)**

Minimum interval between state changes. If the interval since the last reported value has not elapsed, the previous value is re-emitted. Simulates sensors that only transmit at a fixed interval regardless of whether the value has changed.

**Default:** 0 (report every tick). **Recommended values:**

- Continuous sensor: 0
- Battery-powered sensor (reports every 5 min): 300

**Caution:** When rate-limiting is active and the room changes slowly, the sensor may hold the same value for long periods. Home Assistant state-changed automations will not fire during these silent periods. This intentionally simulates the Zigbee/Z-Wave TRV failure mode.

**Mutual exclusion:** `sensor_update_rate` cannot be used at the same time as the Zigbee parameters (Stage 6). Set it to 0 when using Stage 6.

---

**Stage 6 — Zigbee-style conditional reporting (`sensor_min_interval`, `sensor_max_interval`, `sensor_delta`)**

Simulates the conditional reporting behaviour of Zigbee and Z-Wave sensors. Three cooperating rules replace the simple fixed rate-limit:

| Parameter | Key | Description | Default |
| --- | --- | --- | --- |
| Minimum interval | `sensor_min_interval_s` | Shortest time between any two reports (s) | 0 (disabled) |
| Maximum interval | `sensor_max_interval_s` | Heartbeat — always report at least this often, even if value unchanged (s) | 0 (disabled) |
| Delta threshold | `sensor_delta` | Minimum post-quantisation change in °C required to trigger a report | 0 (disabled) |

**Logic (evaluated each tick):**

| Heartbeat due? | Min interval elapsed? | Delta crossed? | Action |
| --- | --- | --- | --- |
| Yes | any | any | **Emit** (keepalive) |
| No  | Yes | Yes | **Emit** (change report) |
| No  | Yes | No  | Suppress |
| No  | No  | any | Suppress |

Setting all three to 0 disables Stage 6 entirely.

**Interaction with quantisation:** the delta comparison is performed against the post-quantisation value, so Stage 4 runs first. With a quantisation step of 0.1 °C and a delta of 0.05 °C, a report fires whenever the rounded value moves to a new quantum (i.e. every full 0.1 °C step). This matches real Zigbee sensor firmware behaviour and prevents chatter at quantum boundaries.

**Rounding convention:** standard half-up rounding (Python `round()`). A true temperature rise of exactly 0.05 °C will reach the next 0.1 °C quantum and cross a 0.05 °C delta threshold; a rise of 0.04 °C will not.

**Mutual exclusion with Stage 5:** `sensor_update_rate` and the Zigbee parameters cannot both be non-zero simultaneously. The options flow enforces this and displays an error if both are set. Set `sensor_update_rate` to 0 when using Zigbee-style reporting.

**State restoration:** Zigbee state (last reported value and last report time) is seeded from the restored room temperature on HA restart. This prevents a spurious immediate heartbeat report on boot.

**Recommended values:**

- Typical Zigbee TRV (e.g. Aqara, Sonoff SNZB-02): min 30 s, max 600 s, delta 0.1 °C
- Aggressive reporting: min 10 s, max 300 s, delta 0.05 °C
- Conservative battery saving: min 60 s, max 1800 s, delta 0.5 °C

---

### Example presets

| Scenario | `lag_tau` | `bias` | `noise_std` | `quantisation` | `update_rate` | `min_interval` | `max_interval` | `delta` |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Ideal (default) | 0   | 0   | 0   | 0   | 0   | 0   | 0   | 0   |
| Budget TRV | 0   | 0   | 0.5 | 0.5 | 60  | 0   | 0   | 0   |
| Smart TRV — Zigbee | 10  | 0   | 0.1 | 0.1 | 0   | 30  | 600 | 0.1 |
| Miscalibrated sensor | 0   | +2.0 | 0.1 | 0   | 0   | 0   | 0   | 0   |
| Wall-mounted (cold wall) | 60  | −1.5 | 0.1 | 0.1 | 0   | 0   | 0   | 0   |
| Zigbee — aggressive | 10  | 0   | 0.1 | 0.1 | 0   | 10  | 300 | 0.05 |

The **Smart TRV — Zigbee** preset replaces the earlier "Smart TRV (Zigbee)" preset which used `update_rate = 30`. The new preset is more physically accurate: the sensor now suppresses trivial noise-driven changes (delta gate) while still guaranteeing a report every 10 minutes as a keepalive heartbeat.

---

## Disturbances

Disturbances inject realistic time-varying inputs into the simulation. All disturbances are disabled by default (back-compatible). They are configured in step 2 of the options flow.

### External Temperature Profile (F-11)

A synthetic diurnal (24-hour) external temperature curve. When enabled, replaces the fixed external temperature with a profile that has a realistic minimum before dawn and maximum in mid-afternoon.

**Shape:** Piece-wise raised-cosine with asymmetric rise and fall periods. The warming phase (trough to peak) is shorter than the cooling phase.

#### Parameters

| Key | Description | Default |
|---|---|---|
| `ext_temp_profile_enabled` | Master switch | false |
| `ext_temp_profile_base` | Mean temperature for the day (°C) | 5.0 |
| `ext_temp_profile_amplitude` | Half-swing from mean to peak/trough (°C) | 3.0 |
| `ext_temp_profile_min_hour` | Hour of minimum temperature (0–24) | 5.5 (05:30) |
| `ext_temp_profile_max_hour` | Hour of maximum temperature (0–24) | 14.5 (14:30) |

**Recommended values:**
- UK winter day: base = 3 °C, amplitude = 3 °C (range −0 to +6 °C)
- UK summer day: base = 18 °C, amplitude = 6 °C (range 12–24 °C)
- UK shoulder season: base = 10 °C, amplitude = 4 °C

**Note:** When a live `external_temperature_entity` is configured, it takes priority over this profile. The profile only applies when no live entity is available.

---

### Occupancy and Internal Heat Gain (F-05)

Simulates the heat generated by people and cooking/appliances. This is a disturbance that a controller must reject to maintain setpoint.

**Model:**
- Night (22:00–07:00): room always empty.
- Daytime: stochastic occupant count varying between 0 and `max_occupants`. Visits are either short (5–25 min) or long (1–3 h).
- Cooking events: Poisson-distributed events placed between 08:00 and 21:00.
- Each adult produces 90 W of sensible heat (CIBSE Guide A, light activity).
- The schedule is regenerated each day, deterministically from `seed + day_number`.

#### Parameters

| Key | Description | Default |
|---|---|---|
| `occupancy_enabled` | Master switch | false |
| `occupancy_max_occupants` | Maximum simultaneous occupants | 2 |
| `occupancy_cooking_power_w` | Peak heat gain during a cooking event (W). 0 = disabled | 0.0 |
| `occupancy_cooking_duration_s` | Typical cooking event duration (s) | 1 200 (20 min) |
| `occupancy_cooking_events_per_day` | Mean cooking events per day (Poisson mean) | 2.0 |
| `occupancy_seed` | Random seed for reproducibility | 42 |

**Recommended values:**
- Single occupant, no cooking: `max_occupants = 1`, `cooking_power_w = 0`
- Couple at home, kitchen: `max_occupants = 2`, `cooking_power_w = 1000`, `events_per_day = 2`
- Use `seed = 42` for reproducible test runs; change seed to get a different schedule.

---

### Wind and Rain Effects (F-06, F-14)

Wind and rain increase the effective heat loss coefficient. Both effects are multiplicative and combine into a single weather multiplier:

```
K_eff = K_base × (1 + wind_coefficient × wind_speed) × (1 + rain_moisture_factor × rain_intensity)
```

Both effects are disabled at their defaults (coefficients = 0).

#### Parameters

| Key | Description | Default |
|---|---|---|
| `weather_wind_speed_m_s` | Current wind speed (m/s) | 0.0 |
| `weather_wind_coefficient` | Sensitivity of K to wind speed (per m/s). 0 = disabled | 0.0 |
| `weather_rain_intensity` | Rain intensity fraction (0 = dry, 1 = heavy) | 0.0 |
| `weather_rain_moisture_factor` | Fractional increase in K at full rain intensity | 0.0 |

**Recommended values for `wind_coefficient`:** 0.02–0.10 per m/s. At Beaufort 5 (≈10 m/s) and coefficient 0.05, the wind factor is 1.50 (50% more heat loss).

**Recommended values for `rain_moisture_factor`:** 0.0–0.30. A value of 0.20 means heavy rain increases fabric heat loss by 20%.

**Wind speed reference (Beaufort scale):**

| Beaufort | Description | m/s |
|---|---|---|
| 1–2 | Light air / light breeze | 1–4 |
| 3 | Gentle breeze | 4–6 |
| 5 | Fresh breeze | 8–11 |
| 7 | Near gale | 13–16 |

**Live updates:** Wind speed and rain intensity can be updated without reloading using the `heating_simulator.set_weather` service call. See [entities.md](entities.md) for the override number entities.

---

## Services

### `heating_simulator.reset_model`

Reset the simulation to a known state. All fields are optional and can be combined freely.

| Field | Description |
|---|---|
| `preset` | Named starting condition: `cold_start`, `overnight`, `room_temperature` |
| `t_room` | Room air temperature (°C). Overrides preset. |
| `t_fabric` | Fabric temperature (°C). R2C2 models only. Defaults to `t_room`. |
| `t_rad` | Radiator temperature (°C). Radiator models only. Defaults to `t_room`. |

**Presets:**

| Preset | Room | Fabric | Description |
|---|---|---|---|
| `cold_start` | External temp | External temp | Heating off for days; all nodes equilibrated to outside |
| `overnight` | 16 °C | 14 °C | Heating off overnight; fabric partially cooled |
| `room_temperature` | 18 °C | 17 °C | Typical occupied room after sustained heating |

**Example — reset to warm start for testing cooling:**

```yaml
service: heating_simulator.reset_model
target:
  device_id: <device_id>
data:
  t_room: 21.0
  t_fabric: 19.0
  t_rad: 35.0
```

---

### `heating_simulator.set_weather`

Update wind speed and/or rain intensity live, without reloading the integration. Only supplied fields are changed; omitting a field leaves the current value unchanged.

| Field | Description | Range |
|---|---|---|
| `wind_speed_m_s` | Wind speed (m/s) | 0–50 |
| `rain_intensity_fraction` | Rain intensity (0 = dry, 1 = heavy) | 0–1 |

**Note:** Wind and rain have no effect unless their sensitivity coefficients (`weather_wind_coefficient`, `weather_rain_moisture_factor`) are set to non-zero values in the simulator options.

**Example:**

```yaml
service: heating_simulator.set_weather
target:
  device_id: <device_id>
data:
  wind_speed_m_s: 10.0
  rain_intensity_fraction: 0.7
```
