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

# PWM timing investigation (2026-09-09)

Scope: suggest resolutions only; no integration implementation changes.

- [x] Confirm investigation plan: trace switch, physics stepping, and sensor publication separately.
- [x] Inspect coordinator and entity source (GitNexus index unavailable for this repository).
- [ ] Reproduce pulse-position energy error with the isolated thermal model.
- [ ] Assess resolution options and verification requirements, including radiator behavior.

Recommended design: advance physics to each input event using elapsed monotonic time and the previous input, then apply the new input. Advance remaining time at publication ticks. Separate control notifications from sensor publication. Use bounded numerical integration steps independent of publication cadence where needed. Validate with a fake clock and fixed-power energy accounting, then compare all models against a fine-step reference.

## PWM investigation review

- [x] Reproduced current stepping behavior using SimpleThermalModel with 2 kW, no heat loss or inertia, and 30-second publications: 10-second pulses at 5–15 / 25–35 / 35–45 seconds produce 0 / 60,000 / 0 J instead of 20,000 J each.
- [x] Assessed event-driven stepping, timestamped replay, and faster polling. Recommend event-driven stepping plus separate sensor publication notifications.
- Numerical follow-up: simple-model heater lag currently uses endpoint power times elapsed time, rather than integrated power; radiator delay queues advance at outer-step boundaries. Both matter when checking publication-cadence independence.
- Verification proposal: fake-clock coordinator checks, energy tests under ideal fixed-power conditions, pulse-boundary and delayed-callback coverage, and all-model comparison with a converged numerical reference. Equal final temperatures are not generally expected for shifted pulses with heat loss or radiator dynamics.
- No runtime files changed; no live Home Assistant verification performed.

# PWM timing implementation

Plan confirmed against requested minimal scope: retain thermal equations and publication timer; account for fractional elapsed time at input transitions.

- [x] Add monotonic elapsed-time advancement and lifecycle handling.
- [x] Separate control notifications from scheduled sensor updates; advance before physical input mutations.
- [x] Add deterministic pulse-duration and publication regression tests.
- [x] Run tests, inspect diff, and document results/limitations.

## Implementation review

- Physics now advances using fractional monotonic elapsed time before PWM, linear, and external input changes. Scheduled ticks integrate only the remainder. Startup/shutdown and resets avoid replaying elapsed or offline time.
- Control subscriptions update immediately; sensor subscriptions run on publication ticks and explicit reset. README documents the behavior.
- Regression verification uses the actual coordinator class and pure thermal model without requiring Home Assistant. Ideal 2 kW / 11.4-second pulses yield 22,800 J at all tested positions and 5/10/30-second publication intervals. Tests also cover multiple short pulses, duplicate commands, delayed ticks, linear power, inactive clocks, and reset.
- Python compilation and git diff whitespace checks pass. Independent source review found no blocker.
- Limits: no live Home Assistant validation; thermal equations, heater lag integration, radiator delay approximations, and sensor lag's nominal interval remain unchanged. These retain their existing numerical limitations.
