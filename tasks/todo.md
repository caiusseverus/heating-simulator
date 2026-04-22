# Fix Plan

- [x] Review the reported findings against the current source and confirm the required scope
- [x] Fix config-flow validation for equal external temperature profile hours
- [x] Fix unload ordering so runtime state is only removed after platform unload succeeds
- [x] Fix service schemas or handlers so supported targets match actual behavior
- [x] Wire combined-model configuration fields to the model or remove dead fields
- [x] Rework per-tick sensor value generation to avoid side effects in `native_value`
- [x] Replace silent upstream state parse failures with structured logging
- [x] Fix combined-model convective-fraction wiring
- [x] Reconcile R2C2 default resistance values with the model assumptions
- [x] Recompute R2C2 and combined-model diagnostics from consistent node-balance equations
- [x] Verify the final diff for correctness and document review results

# Notes

- `tasks/lessons.md` is not present in this repository, so there was nothing to review at session start.
- Verification will be source-based unless a lightweight local check is available; this repo does not ship a test suite or runnable Home Assistant harness.

# Review

- Fixed the options-flow runtime failure by validating equal external-temperature profile hours before save, and added a runtime guard in `ExternalTempProfile` so old invalid configs do not divide by zero.
- Fixed unload sequencing so `hass.data` and registered services are only torn down after `async_unload_platforms()` succeeds.
- Removed unsupported `entity_id` and `area_id` service targets from the schemas to match the current device-only handlers.
- Removed dead combined-model config fields from setup and options schemas, while preserving the radiator-only fields where they are actually consumed.
- Cached the degraded room temperature on simulator updates so `TemperatureSensor.native_value` is now a side-effect-free read.
- Replaced silent parse failures for bound HA entities with structured debug logging that includes the source entity and rejected state.
- Wired `radiator_convective_fraction` into combined-model construction and reconciled the shipped R2C2 resistance defaults with the model documentation.
- Recomputed the R2C2 and combined-model air-node diagnostics from consistent signed heat-flow terms instead of mixing whole-building and air-node losses.
- Updated manifest metadata to use the real repository URL and maintainer handle.
- Verification: `python3 -m py_compile __init__.py config_flow.py const.py disturbances.py sensor.py thermal_model.py`
