# Heating Simulator

Heating Simulator is a Home Assistant custom integration for testing heating controls without physical hardware.
It simulates realistic room temperature dynamics across multiple thermal models (including radiator and multi-node building models), so you can validate automations, thermostat logic, and controller tuning safely.

## Installation

> HACS is **not currently supported** for this repository.

1. Copy this repository to:
   ```
   <config>/custom_components/heating_simulator
   ```
2. Restart Home Assistant.
3. Go to **Settings → Devices & Services → Add Integration**.
4. Search for **Heating Simulator** and complete the setup flow.

## Configuration

During setup you select:
- Model type (`simple`, `r2c2`, `radiator`, `r2c2_radiator`)
- Control mode (`linear` or `pwm`)
- Initial and external temperature source
- Model-specific physical parameters
- Optional realism/calibration features

You can later adjust options via **Settings → Devices & Services → Heating Simulator → Configure**.

## Detailed documentation

- [Model reference](docs/models.md) — detailed equations, model operation, radiator mathematics, and full model configuration guidance.
- [Feature reference](docs/features.md) — implementation details for controls, disturbances, sensor realism, calibration, entities, and configuration options.
