"""Number entities for Heating Simulator — power/valve input + overrides."""
from __future__ import annotations

from homeassistant.components.number import NumberEntity, NumberMode
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.entity import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.restore_state import RestoreEntity

from .const import (
    DOMAIN, MODEL_RADIATOR, MODEL_R2C2, MODEL_R2C2_RADIATOR,
    DEFAULT_WIND_COEFFICIENT, DEFAULT_RAIN_MOISTURE_FACTOR,
)
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

    # Weather disturbance number entities — only shown when the
    # corresponding sensitivity coefficient is non-zero (i.e. the feature
    # is intentionally enabled in config). If the coefficient is 0 the
    # slider would have no effect, which would be confusing.
    cfg = {**entry.data, **entry.options}
    if float(cfg.get("weather_wind_coefficient", DEFAULT_WIND_COEFFICIENT)) > 0:
        entities.append(WindSpeedNumber(simulator, entry))
    if float(cfg.get("weather_rain_moisture_factor", DEFAULT_RAIN_MOISTURE_FACTOR)) > 0:
        entities.append(RainIntensityNumber(simulator, entry))

    async_add_entities(entities)


def _device(entry):
    return DeviceInfo(identifiers={(DOMAIN, entry.entry_id)}, name=entry.title)


class _BaseNumber(NumberEntity, RestoreEntity):
    _attr_has_entity_name = True
    _attr_should_poll = False

    def __init__(self, sim: HeatingSimulator, entry: ConfigEntry) -> None:
        self._sim = sim
        self._attr_device_info = _device(entry)
        self._unsub = None

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        self._unsub = self._sim.register_listener(self._on_update)

    async def async_will_remove_from_hass(self) -> None:
        if self._unsub:
            self._unsub()

    @callback
    def _on_update(self) -> None:
        self.async_write_ha_state()

    async def _async_restore_number(self) -> float | None:
        if self._sim.snapshot_restored:
            return None
        last_state = await self.async_get_last_state()
        if last_state is None or last_state.state in ("unknown", "unavailable", None):
            return None
        try:
            return float(last_state.state)
        except (TypeError, ValueError):
            return None


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

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        restored = await self._async_restore_number()
        if restored is not None:
            self._sim.set_linear_power(restored)

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

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        restored = await self._async_restore_number()
        if restored is not None:
            self._sim.model.set_external_temperature(restored)
            self._sim._notify_listeners()
            self._sim._schedule_state_save()

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

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        restored = await self._async_restore_number()
        if restored is not None:
            self._sim.model.set_solar_irradiance(restored)
            self._sim._notify_listeners()
            self._sim._schedule_state_save()

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

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        restored = await self._async_restore_number()
        if restored is not None:
            self._sim.model.set_flow_temperature(restored)
            self._sim._notify_listeners()
            self._sim._schedule_state_save()

    async def async_set_native_value(self, value: float) -> None:
        self._sim.model.set_flow_temperature(value)
        self._sim._notify_listeners()


class WindSpeedNumber(_BaseNumber):
    """
    Live wind speed input (m/s) for the F-06 wind effect on heat loss.

    Only created when wind_coefficient > 0 in config, so the slider is
    never shown if the feature is disabled. Changing this value takes
    effect on the very next simulator tick without any reload.
    """
    _attr_name = "Wind Speed"
    _attr_native_min_value = 0.0
    _attr_native_max_value = 50.0
    _attr_native_step = 0.5
    _attr_native_unit_of_measurement = "m/s"
    _attr_mode = NumberMode.SLIDER
    _attr_icon = "mdi:weather-windy"

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_wind_speed"

    @property
    def native_value(self):
        return self._sim.wind_speed

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        restored = await self._async_restore_number()
        if restored is not None:
            self._sim.set_weather(wind_speed_m_s=restored)

    async def async_set_native_value(self, value: float) -> None:
        self._sim.set_weather(wind_speed_m_s=value)


class RainIntensityNumber(_BaseNumber):
    """
    Live rain intensity input (0-1) for the F-14 rain/moisture effect.

    Only created when rain_moisture_factor > 0 in config. 0 = dry,
    1 = heavy rain. Changing this value takes effect on the next tick.
    """
    _attr_name = "Rain Intensity"
    _attr_native_min_value = 0.0
    _attr_native_max_value = 1.0
    _attr_native_step = 0.05
    _attr_native_unit_of_measurement = ""
    _attr_mode = NumberMode.SLIDER
    _attr_icon = "mdi:weather-rainy"

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_rain_intensity"

    @property
    def native_value(self):
        return self._sim.rain_intensity

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        restored = await self._async_restore_number()
        if restored is not None:
            self._sim.set_weather(rain_intensity_fraction=restored)

    async def async_set_native_value(self, value: float) -> None:
        self._sim.set_weather(rain_intensity_fraction=value)
