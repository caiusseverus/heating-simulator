"""Number entities for Heating Simulator — power/valve input + overrides."""
from __future__ import annotations

from homeassistant.components.number import NumberEntity, NumberMode
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.entity import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .const import DOMAIN, MODEL_RADIATOR, MODEL_R2C2, MODEL_R2C2_RADIATOR
from . import HeatingSimulator


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    simulator: HeatingSimulator = hass.data[DOMAIN][entry.entry_id]

    entities = [
        PowerInputNumber(simulator, entry),
        ExternalTempOverride(simulator, entry),
    ]

    if simulator.model_type in (MODEL_R2C2, MODEL_R2C2_RADIATOR):
        entities.append(SolarIrradianceOverride(simulator, entry))

    if simulator.model_type in (MODEL_RADIATOR, MODEL_R2C2_RADIATOR):
        entities.append(FlowTempOverride(simulator, entry))

    async_add_entities(entities)


def _device(entry):
    return DeviceInfo(identifiers={(DOMAIN, entry.entry_id)}, name=entry.title)


class _BaseNumber(NumberEntity):
    _attr_has_entity_name = True
    _attr_should_poll = False

    def __init__(self, sim: HeatingSimulator, entry: ConfigEntry) -> None:
        self._sim = sim
        self._attr_device_info = _device(entry)
        self._unsub = None

    async def async_added_to_hass(self) -> None:
        self._unsub = self._sim.register_listener(self._on_update)

    async def async_will_remove_from_hass(self) -> None:
        if self._unsub:
            self._unsub()

    @callback
    def _on_update(self) -> None:
        self.async_write_ha_state()


class PowerInputNumber(_BaseNumber):
    """
    Heater power 0–100% for linear mode, or valve position 0–100% for radiator model.
    In PWM mode this still works as a direct override.
    """
    _attr_native_min_value = 0.0
    _attr_native_max_value = 100.0
    _attr_native_step = 0.5
    _attr_native_unit_of_measurement = "%"
    _attr_mode = NumberMode.SLIDER
    _attr_icon = "mdi:fire"

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_power_input"
        if sim.model_type in (MODEL_RADIATOR, MODEL_R2C2_RADIATOR):
            self._attr_name = "Valve Position"
            self._attr_icon = "mdi:valve"
        else:
            self._attr_name = "Heater Power Input"

    @property
    def native_value(self):
        return self._sim.power_percent

    async def async_set_native_value(self, value: float) -> None:
        self._sim.set_linear_power(value)


class ExternalTempOverride(_BaseNumber):
    _attr_name = "External Temperature Override"
    _attr_native_min_value = -30.0
    _attr_native_max_value = 50.0
    _attr_native_step = 0.5
    _attr_native_unit_of_measurement = "°C"
    _attr_mode = NumberMode.BOX
    _attr_icon = "mdi:thermometer-low"

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_ext_temp_override"

    @property
    def native_value(self):
        return self._sim.model.external_temperature

    async def async_set_native_value(self, value: float) -> None:
        self._sim.model.set_external_temperature(value)
        self._sim._notify_listeners()


class SolarIrradianceOverride(_BaseNumber):
    """Override solar irradiance for the R2C2 model (W/m²)."""
    _attr_name = "Solar Irradiance Override"
    _attr_native_min_value = 0.0
    _attr_native_max_value = 1200.0
    _attr_native_step = 10.0
    _attr_native_unit_of_measurement = "W/m²"
    _attr_mode = NumberMode.SLIDER
    _attr_icon = "mdi:weather-sunny"

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_solar_irradiance"

    @property
    def native_value(self):
        return self._sim.model.solar_irradiance

    async def async_set_native_value(self, value: float) -> None:
        self._sim.model.set_solar_irradiance(value)
        self._sim._notify_listeners()


class FlowTempOverride(_BaseNumber):
    """Override boiler flow temperature for the wet radiator model."""
    _attr_name = "Flow Temperature Override"
    _attr_native_min_value = 20.0
    _attr_native_max_value = 85.0
    _attr_native_step = 1.0
    _attr_native_unit_of_measurement = "°C"
    _attr_mode = NumberMode.SLIDER
    _attr_icon = "mdi:water-thermometer"

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_flow_temp_override"

    @property
    def native_value(self):
        return self._sim.model.flow_temperature

    async def async_set_native_value(self, value: float) -> None:
        self._sim.model.set_flow_temperature(value)
        self._sim._notify_listeners()
