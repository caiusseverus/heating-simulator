# Heating Simulator Review Report

Date: 2026-04-02

## Summary

The integration is generally coherent and follows a consistent push-update architecture, but several substantive issues need follow-up: one runtime failure path in config validation, one Home Assistant unload lifecycle bug, and two physical-model/config mismatches that make exposed settings or defaults materially incorrect. Priority should be: fix the config and unload bugs first, then correct the combined-model parameter wiring and the R2C2 defaults. There is also no automated test suite in this repo, so the risky paths below are currently unguarded.

## Layer 1 — Code Quality Findings

**[SEVERITY: Medium]** `sensor.py → TemperatureSensor.native_value()`
Issue: `native_value` is stateful and non-deterministic. It mutates `_lagged_temp`, reporting timestamps, and Zigbee report state, and it also samples fresh Gaussian noise on read (`sensor.py:215-314`). In Home Assistant, `native_value` should be a cheap read of already-computed state; recomputing on property access can produce inconsistent values outside simulator ticks.
Fix: Compute the degraded sensor reading once per simulator update in `_on_update()` or a dedicated `update_from_model()` method, store it on the entity, and have `native_value` return the stored value without side effects.

**[SEVERITY: Medium]** `__init__.py → HeatingSimulator._async_ext_temp_changed()`
Issue: `_async_ext_temp_changed()`, `_async_solar_changed()`, and `_async_flow_temp_changed()` silently swallow parse and attribute errors with `pass` (`__init__.py:621-642`). That makes bad upstream entity states invisible during diagnosis.
Fix: Log at `debug` or `warning` with the source entity ID and rejected state, and only suppress the exception after recording why the update was ignored.

**[SEVERITY: Medium]** `config_flow.py → _r2c2_radiator_model_schema()`
Issue: The combined-model options schema asks for `c_room_rad`, `heat_loss_coefficient_rad`, and calibration inputs that the actual `R2C2RadiatorModel` never consumes. The constructor used in setup does not accept those parameters (`__init__.py:353-373`), and `apply_calibration_r2c2_radiator()` only recalculates `r_ext` (`config_flow.py:929-934`).
Fix: Remove unused combined-model fields from the schema, or wire them into the model if they are meant to matter. As written, they are dead configuration and will mislead users.

## Layer 2 — Home Assistant Standards Findings

**[SEVERITY: High]** `config_flow.py → HeatingSimulatorOptionsFlow.async_step_disturbances()`
Issue: The config flow does not validate `ext_temp_profile_min_hour != ext_temp_profile_max_hour`. If a user sets them equal, `ExternalTempProfile.temperature_at()` divides by zero when calculating the raised-cosine phase (`disturbances.py:96-103`).
Fix: Add cross-field validation in the disturbances step and reject equal hours with a form error before saving options.

**[SEVERITY: High]** `__init__.py → async_unload_entry()`
Issue: The entry is removed from `hass.data` and services may be removed before `async_unload_platforms()` has succeeded. If platform unload fails, the integration can be left half-unloaded with missing runtime state.
Fix: First call `await hass.config_entries.async_unload_platforms(...)`, check the boolean result, and only then stop the simulator, remove services, and delete `hass.data[DOMAIN][entry.entry_id]`.

**[SEVERITY: Medium]** `__init__.py → async_setup_entry()`
Issue: The registered services advertise `entity_id` and `area_id` in the schema (`__init__.py:122-143`), but `_handle_reset()` and `_handle_set_weather()` only resolve `device_id` (`__init__.py:160-183`, `__init__.py:195-215`). Entity- and area-targeted calls will be accepted but ignored.
Fix: Either implement full target resolution via Home Assistant’s service target helpers, or remove unsupported target keys from the schema.

**[SEVERITY: Low]** `manifest.json → metadata`
Issue: The manifest still contains placeholder metadata: `documentation` points to `https://github.com/your-repo/heating-simulator` and `codeowners` is empty (`manifest.json:5-9`). That is weak for HACS/current custom-integration standards.
Fix: Replace the placeholder URL with the real documentation repository path and populate `codeowners` with at least one maintainer handle.

## Layer 3 — Physical Model Findings

**[SEVERITY: High]** `__init__.py → HeatingSimulator._build_model()`
Issue: The combined `R2C2RadiatorModel` ignores the configured `radiator_convective_fraction`. The config flow exposes it (`config_flow.py:444-446`), the model supports it (`thermal_model.py:909`), but `_build_model()` never passes it, so the model always uses the default `0.75`.
Fix: Pass `radiator_convective_fraction=float(cfg.get(CONF_RAD_CONVECTIVE_FRACTION, DEFAULT_RAD_CONVECTIVE_FRACTION))` when instantiating `R2C2RadiatorModel`.

**[SEVERITY: High]** `const.py → R2C2 default parameters`
Issue: The shipped R2C2 defaults are physically inconsistent with the model’s own documented operating ranges. `DEFAULT_R_EXT = 2.0` and `DEFAULT_R_INF = 0.5` (`const.py:101-105`) are roughly 100x and 7x larger than the values documented in the model comments (`thermal_model.py:305-307`). Because setup and model construction use those constants (`config_flow.py:197-202`, `__init__.py:393-396`), the default simulated room is unrealistically well insulated.
Fix: Reconcile the constants and documentation, then retune the defaults to one physically consistent set before exposing them as recommended values.

**[SEVERITY: Medium]** `thermal_model.py → R2C2ThermalModel._update_diagnostics()`
Issue: The reported heating/loss/net rates are not physically consistent with the model equations. `heat_loss_rate` ignores the fabric-to-exterior loss path and instead uses `max(0, -fabric_heat_flux)` (`thermal_model.py:459-466`); `net_heat_rate` also mixes whole-building heat flows and divides them only by `c_air`. These diagnostics back the exposed sensors, so the integration is publishing misleading thermodynamic rates.
Fix: Recompute diagnostics directly from the same node-balance equations used in `_euler()`, with separate air-node and fabric-node terms and consistent denominators.

**[SEVERITY: Medium]** `thermal_model.py → R2C2RadiatorModel._update_diagnostics()`
Issue: The combined model has the same problem. `heating_rate` uses total radiator output `q_out_watts` even though only `q_conv_watts` heats the air node directly, and `heat_loss_rate` again omits fabric-to-exterior loss (`thermal_model.py:1185-1188`). The diagnostic sensors therefore overstate air heating and understate total loss.
Fix: Base air-node diagnostics on `q_conv_watts + q_solar_air + q_fab_to_air + internal_gain - q_inf`, and expose fabric-node terms separately if needed.

## Recommended Action Order

1. Layer 2: Add disturbances-step validation for equal external profile hours, because that is the clearest runtime failure path.
2. Layer 2: Fix `async_unload_entry()` ordering so failed platform unloads do not leave the integration in a broken partially-removed state.
3. Layer 3: Wire `radiator_convective_fraction` into `R2C2RadiatorModel` construction, because the UI currently exposes a physically meaningful setting that has no effect.
4. Layer 3: Reconcile and retune the R2C2 default resistances in `const.py`, then verify the resulting transient and steady-state behaviour against the documented assumptions.
5. Layer 1 and Layer 3: Refactor rate and diagnostic calculations so entity sensor values are derived from one consistent set of node-balance equations.
6. Layer 1 and Layer 2: Make sensor value generation deterministic per tick, and replace silent exception swallowing in entity-state callbacks with minimal structured logging.
7. Layer 1: Remove or properly implement dead combined-model config fields so users are not asked to tune parameters the simulator ignores.
