"""Switch entity for PWM heater control."""
from __future__ import annotations

from homeassistant.components.switch import SwitchEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.entity import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .const import DOMAIN, CONTROL_MODE_PWM
from . import HeatingSimulator


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up switch entities."""
    simulator: HeatingSimulator = hass.data[DOMAIN][entry.entry_id]
    async_add_entities([HeaterSwitch(simulator, entry)])


class HeaterSwitch(SwitchEntity):
    """
    PWM heater switch.

    In PWM mode a thermostat controller turns this on/off;
    the simulator responds as a full-power or off heater with thermal inertia.
    The switch is also available in linear mode for hard on/off overrides.
    """

    _attr_has_entity_name = True
    _attr_name = "Heater Switch (PWM)"
    _attr_icon = "mdi:radiator"
    _attr_should_poll = False

    def __init__(self, simulator: HeatingSimulator, entry: ConfigEntry) -> None:
        self._simulator = simulator
        self._attr_unique_id = f"{entry.entry_id}_heater_switch"
        self._attr_device_info = DeviceInfo(
            identifiers={(DOMAIN, entry.entry_id)},
            name=entry.title,
        )
        self._unsub = None

    async def async_added_to_hass(self) -> None:
        self._unsub = self._simulator.register_listener(self._handle_update)

    async def async_will_remove_from_hass(self) -> None:
        if self._unsub:
            self._unsub()

    @callback
    def _handle_update(self) -> None:
        self.async_write_ha_state()

    @property
    def is_on(self) -> bool:
        return self._simulator.pwm_on

    async def async_turn_on(self, **kwargs) -> None:
        self._simulator.set_pwm_switch(True)

    async def async_turn_off(self, **kwargs) -> None:
        self._simulator.set_pwm_switch(False)

    @property
    def extra_state_attributes(self):
        sim = self._simulator
        attrs = {
            "control_mode": sim.control_mode,
            "effective_power_w": round(sim.model.effective_heater_power, 1),
        }
        # heater_power_watts only exists on Simple and R2C2 models;
        # radiator models are sized by flow_temperature + k_radiator instead
        if hasattr(sim.model, "heater_power_watts"):
            attrs["nominal_power_w"] = sim.model.heater_power_watts
        elif hasattr(sim.model, "flow_temperature"):
            attrs["flow_temperature"] = sim.model.flow_temperature
            attrs["nominal_output_w_at_dt50"] = round(sim.model.nominal_output_at_dt50, 1)
        return attrs
