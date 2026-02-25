"""Heating Simulator integration for Home Assistant."""
from __future__ import annotations

import logging
from datetime import timedelta
from typing import Any

import voluptuous as vol

from homeassistant.config_entries import ConfigEntry
from homeassistant.const import Platform
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.event import (
    async_track_state_change_event,
    async_track_time_interval,
)
from homeassistant.helpers import device_registry as dr

from .const import (
    DOMAIN,
    # shared
    CONF_MODEL_TYPE,
    CONF_CONTROL_MODE,
    CONF_INITIAL_TEMP,
    CONF_EXTERNAL_TEMP,
    CONF_EXTERNAL_TEMP_FIXED,
    CONF_UPDATE_INTERVAL,
    # simple
    CONF_HEATER_POWER,
    CONF_HEAT_LOSS_COEFF,
    CONF_THERMAL_MASS,
    CONF_THERMAL_INERTIA,
    # R2C2
    CONF_C_AIR,
    CONF_C_FABRIC,
    CONF_R_FABRIC,
    CONF_R_EXT,
    CONF_R_INF,
    CONF_HEATER_POWER_R2C2,
    CONF_SOLAR_ENTITY,
    CONF_SOLAR_FIXED,
    CONF_WINDOW_AREA,
    CONF_WINDOW_TRANSMITTANCE,
    # radiator
    CONF_FLOW_TEMP,
    CONF_FLOW_TEMP_ENTITY,
    CONF_C_RAD,
    CONF_K_RAD,
    CONF_RAD_EXPONENT,
    CONF_FLOW_RATE_MAX,
    CONF_HEAT_LOSS_COEFF_RAD,
    CONF_C_ROOM_RAD,
    CONF_PIPE_DELAY,
    # model types
    MODEL_SIMPLE,
    MODEL_R2C2,
    MODEL_RADIATOR,
    MODEL_R2C2_RADIATOR,
    CONTROL_MODE_LINEAR,
    # defaults — simple
    DEFAULT_HEATER_POWER,
    DEFAULT_HEAT_LOSS_COEFF,
    DEFAULT_THERMAL_MASS,
    DEFAULT_THERMAL_INERTIA,
    # defaults — R2C2
    DEFAULT_C_AIR,
    DEFAULT_C_FABRIC,
    DEFAULT_R_FABRIC,
    DEFAULT_R_EXT,
    DEFAULT_R_INF,
    DEFAULT_HEATER_POWER_R2C2,
    DEFAULT_SOLAR_FIXED,
    DEFAULT_WINDOW_AREA,
    DEFAULT_WINDOW_TRANSMITTANCE,
    # defaults — radiator
    DEFAULT_FLOW_TEMP,
    DEFAULT_C_RAD,
    DEFAULT_K_RAD,
    DEFAULT_RAD_EXPONENT,
    DEFAULT_FLOW_RATE_MAX,
    DEFAULT_HEAT_LOSS_COEFF_RAD,
    DEFAULT_C_ROOM_RAD,
    DEFAULT_PIPE_DELAY,
    # defaults — shared
    DEFAULT_INITIAL_TEMP,
    DEFAULT_EXTERNAL_TEMP_FIXED,
    DEFAULT_UPDATE_INTERVAL,
    # reset action
    ACTION_RESET,
    PRESET_COLD_START,
    PRESET_OVERNIGHT,
    PRESET_ROOM_TEMPERATURE,
    RESET_PRESETS,
)
from .thermal_model import SimpleThermalModel, R2C2ThermalModel, WetRadiatorModel, R2C2RadiatorModel

_LOGGER = logging.getLogger(__name__)

PLATFORMS = [Platform.SENSOR, Platform.NUMBER, Platform.SWITCH]

_RESET_SERVICE_SCHEMA = vol.Schema(
    {
        # Add target keys injected by the frontend
        vol.Optional("device_id"): vol.Any(str, [str]),
        vol.Optional("entity_id"): vol.Any(str, [str]),
        vol.Optional("area_id"): vol.Any(str, [str]),
        
        # Existing parameters
        vol.Optional("preset"): vol.In(
            [PRESET_COLD_START, PRESET_OVERNIGHT, PRESET_ROOM_TEMPERATURE]
        ),
        vol.Optional("t_room"):   vol.Coerce(float),
        vol.Optional("t_fabric"): vol.Coerce(float),
        vol.Optional("t_rad"):    vol.Coerce(float),
    }
)


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Set up Heating Simulator from a config entry."""
    hass.data.setdefault(DOMAIN, {})
    config = {**entry.data, **entry.options}
    simulator = HeatingSimulator(hass, entry, config)
    hass.data[DOMAIN][entry.entry_id] = simulator
    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)
    await simulator.async_start()
    entry.async_on_unload(entry.add_update_listener(_async_update_options))

    async def _handle_reset(call) -> None:
        """Handle the reset_model service call."""
        target_entry_ids: set[str] | None = None
        
        # Extract target devices from the service call data
        target_devices = call.data.get("device_id")

        if target_devices:
            # Normalize to a list to handle both single and multiple selections
            if isinstance(target_devices, str):
                target_devices = [target_devices]
                
            dev_reg = dr.async_get(hass)
            target_entry_ids = set()
            for device_id in target_devices:
                device = dev_reg.async_get(device_id)
                if device is None:
                    continue
                # Each simulator device has exactly one config entry
                for entry_id in device.config_entries:
                    if entry_id in hass.data[DOMAIN]:
                        target_entry_ids.add(entry_id)

        for entry_id, sim in hass.data[DOMAIN].items():
            if target_entry_ids is None or entry_id in target_entry_ids:
                sim.reset_model(
                    t_room=call.data.get("t_room"),
                    t_fabric=call.data.get("t_fabric"),
                    t_rad=call.data.get("t_rad"),
                    preset=call.data.get("preset"),
                )

    # Register once per domain — safe to call on each entry load because of the guard
    if not hass.services.has_service(DOMAIN, ACTION_RESET):
        hass.services.async_register(
            DOMAIN,
            ACTION_RESET,
            _handle_reset,
            schema=_RESET_SERVICE_SCHEMA,
        )

    return True


async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Unload a config entry."""
    simulator: HeatingSimulator = hass.data[DOMAIN].pop(entry.entry_id)
    simulator.async_stop()
    # Remove the service only when the last instance is unloaded
    if not hass.data[DOMAIN]:
        hass.services.async_remove(DOMAIN, ACTION_RESET)
    return await hass.config_entries.async_unload_platforms(entry, PLATFORMS)


async def _async_update_options(hass: HomeAssistant, entry: ConfigEntry) -> None:
    await hass.config_entries.async_reload(entry.entry_id)


# ---------------------------------------------------------------------------
# Coordinator
# ---------------------------------------------------------------------------

class HeatingSimulator:
    """
    Coordinates the active thermal model and exposes its state to HA entities.

    Responsibilities:
    - Instantiate the correct ThermalModel subclass from config.
    - Tick the model on a fixed interval.
    - Track external temperature, solar irradiance, and flow temperature entities.
    - Provide a unified set_power_fraction / set_pwm_switch API for entities.
    - Notify listeners (push, not poll) on every tick and on input changes.
    """

    def __init__(
        self,
        hass: HomeAssistant,
        entry: ConfigEntry,
        config: dict[str, Any],
    ) -> None:
        self.hass = hass
        self.entry = entry
        self.config = config

        self.model_type: str = config.get(CONF_MODEL_TYPE, MODEL_SIMPLE)
        self.control_mode: str = config.get(CONF_CONTROL_MODE, CONTROL_MODE_LINEAR)
        self.update_interval: int = int(config.get(CONF_UPDATE_INTERVAL, DEFAULT_UPDATE_INTERVAL))

        initial_temp = float(config.get(CONF_INITIAL_TEMP, DEFAULT_INITIAL_TEMP))
        initial_ext = float(config.get(CONF_EXTERNAL_TEMP_FIXED, DEFAULT_EXTERNAL_TEMP_FIXED))

        self.model = self._build_model(config, initial_temp, initial_ext)

        self._pwm_on: bool = False
        self._listeners: list = []
        self._unsub_interval = None
        self._unsub_ext_temp = None
        self._unsub_solar = None
        self._unsub_flow_temp = None

    # ------------------------------------------------------------------
    # Model factory
    # ------------------------------------------------------------------

    def _build_model(self, cfg: dict, initial_temp: float, initial_ext: float):
        if self.model_type == MODEL_R2C2:
            return R2C2ThermalModel(
                heater_power_watts=float(cfg.get(CONF_HEATER_POWER_R2C2, DEFAULT_HEATER_POWER_R2C2)),
                c_air=float(cfg.get(CONF_C_AIR, DEFAULT_C_AIR)),
                c_fabric=float(cfg.get(CONF_C_FABRIC, DEFAULT_C_FABRIC)),
                r_fabric=float(cfg.get(CONF_R_FABRIC, DEFAULT_R_FABRIC)),
                r_ext=float(cfg.get(CONF_R_EXT, DEFAULT_R_EXT)),
                r_inf=float(cfg.get(CONF_R_INF, DEFAULT_R_INF)),
                window_area=float(cfg.get(CONF_WINDOW_AREA, DEFAULT_WINDOW_AREA)),
                window_transmittance=float(cfg.get(CONF_WINDOW_TRANSMITTANCE, DEFAULT_WINDOW_TRANSMITTANCE)),
                initial_temp=initial_temp,
                initial_external_temp=initial_ext,
                initial_solar=float(cfg.get(CONF_SOLAR_FIXED, DEFAULT_SOLAR_FIXED)),
            )
        elif self.model_type == MODEL_RADIATOR:
            return WetRadiatorModel(
                flow_temperature=float(cfg.get(CONF_FLOW_TEMP, DEFAULT_FLOW_TEMP)),
                c_radiator=float(cfg.get(CONF_C_RAD, DEFAULT_C_RAD)),
                k_radiator=float(cfg.get(CONF_K_RAD, DEFAULT_K_RAD)),
                radiator_exponent=float(cfg.get(CONF_RAD_EXPONENT, DEFAULT_RAD_EXPONENT)),
                flow_rate_max=float(cfg.get(CONF_FLOW_RATE_MAX, DEFAULT_FLOW_RATE_MAX)),
                heat_loss_coeff=float(cfg.get(CONF_HEAT_LOSS_COEFF_RAD, DEFAULT_HEAT_LOSS_COEFF_RAD)),
                c_room=float(cfg.get(CONF_C_ROOM_RAD, DEFAULT_C_ROOM_RAD)),
                pipe_delay=float(cfg.get(CONF_PIPE_DELAY, DEFAULT_PIPE_DELAY)),
                initial_temp=initial_temp,
                initial_external_temp=initial_ext,
            )
        elif self.model_type == MODEL_R2C2_RADIATOR:
            return R2C2RadiatorModel(
                # Radiator
                flow_temperature=float(cfg.get(CONF_FLOW_TEMP, DEFAULT_FLOW_TEMP)),
                c_radiator=float(cfg.get(CONF_C_RAD, DEFAULT_C_RAD)),
                k_radiator=float(cfg.get(CONF_K_RAD, DEFAULT_K_RAD)),
                radiator_exponent=float(cfg.get(CONF_RAD_EXPONENT, DEFAULT_RAD_EXPONENT)),
                flow_rate_max=float(cfg.get(CONF_FLOW_RATE_MAX, DEFAULT_FLOW_RATE_MAX)),
                pipe_delay=float(cfg.get(CONF_PIPE_DELAY, DEFAULT_PIPE_DELAY)),
                # Room
                c_air=float(cfg.get(CONF_C_AIR, DEFAULT_C_AIR)),
                c_fabric=float(cfg.get(CONF_C_FABRIC, DEFAULT_C_FABRIC)),
                r_fabric=float(cfg.get(CONF_R_FABRIC, DEFAULT_R_FABRIC)),
                r_ext=float(cfg.get(CONF_R_EXT, DEFAULT_R_EXT)),
                r_inf=float(cfg.get(CONF_R_INF, DEFAULT_R_INF)),
                # Solar
                window_area=float(cfg.get(CONF_WINDOW_AREA, DEFAULT_WINDOW_AREA)),
                window_transmittance=float(cfg.get(CONF_WINDOW_TRANSMITTANCE, DEFAULT_WINDOW_TRANSMITTANCE)),
                # Initial conditions
                initial_temp=initial_temp,
                initial_external_temp=initial_ext,
                initial_solar=float(cfg.get(CONF_SOLAR_FIXED, DEFAULT_SOLAR_FIXED)),
            )
        else:  # MODEL_SIMPLE
            return SimpleThermalModel(
                heater_power_watts=float(cfg.get(CONF_HEATER_POWER, DEFAULT_HEATER_POWER)),
                heat_loss_coeff=float(cfg.get(CONF_HEAT_LOSS_COEFF, DEFAULT_HEAT_LOSS_COEFF)),
                thermal_mass=float(cfg.get(CONF_THERMAL_MASS, DEFAULT_THERMAL_MASS)),
                thermal_inertia_tau=float(cfg.get(CONF_THERMAL_INERTIA, DEFAULT_THERMAL_INERTIA)),
                initial_temp=initial_temp,
                initial_external_temp=initial_ext,
            )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def async_start(self) -> None:
        """Start the simulation loop and subscribe to input entities."""
        # NOTE: State restoration is handled by the temperature sensor entities
        # via RestoreSensor. They call restore_state() on their own and then
        # push the recovered values into the model before the first tick fires.
        # We defer starting the tick loop by one HA event loop cycle so that
        # async_added_to_hass() on all entities has already run by the time
        # the first tick fires.
        self._unsub_interval = async_track_time_interval(
            self.hass,
            self._async_tick,
            timedelta(seconds=self.update_interval),
        )
        self._subscribe_ext_temp()
        self._subscribe_solar()
        self._subscribe_flow_temp()

    def restore_temperatures(
        self,
        t_room: float | None,
        t_fabric: float | None = None,
        t_rad: float | None = None,
    ) -> None:
        """
        Inject previously-persisted temperatures into the model after HA restarts.
        Called by the primary temperature sensor entities in async_added_to_hass().
        Only applied if a valid (non-None) value is recovered.

        Each model exposes its state differently:
          Simple:         .temperature  (direct float attribute)
          R2C2:           .t_air, .t_fabric
          Radiator:       .t_room, .t_rad
          R2C2+Radiator:  .t_air, .t_fabric, .t_rad
        """
        if t_room is not None:
            if hasattr(self.model, "t_air"):
                self.model.t_air = t_room
            elif hasattr(self.model, "t_room"):
                self.model.t_room = t_room
            else:
                self.model.temperature = t_room
            _LOGGER.debug("Restored room/air temperature: %.2f°C", t_room)

        if t_fabric is not None and hasattr(self.model, "t_fabric"):
            self.model.t_fabric = t_fabric
            _LOGGER.debug("Restored fabric temperature: %.2f°C", t_fabric)

        if t_rad is not None and hasattr(self.model, "t_rad"):
            self.model.t_rad = t_rad
            _LOGGER.debug("Restored radiator temperature: %.2f°C", t_rad)

    def reset_model(
        self,
        t_room: float | None = None,
        t_fabric: float | None = None,
        t_rad: float | None = None,
        preset: str | None = None,
    ) -> None:
        """
        Reset all model state variables to known values.

        Priority: explicit t_room/t_fabric/t_rad args > preset > configured initial_temp.
        For cold_start preset, t_room defaults to the current external temperature.
        Unspecified fabric and radiator temperatures default to the resolved t_room,
        giving a fully equilibrated starting condition.
        """
        # Resolve preset defaults first, then let explicit args override
        p_room: float | None = None
        p_fabric: float | None = None

        if preset and preset in RESET_PRESETS:
            p = RESET_PRESETS[preset]
            p_room = p["t_room"]     # None means "use T_ext" (cold_start)
            p_fabric = p["t_fabric"]

        # Resolve final room temperature
        if t_room is not None:
            final_room = t_room
        elif p_room is not None:
            final_room = p_room
        elif preset == PRESET_COLD_START:
            # cold_start with no explicit t_room: equilibrate to current external temp
            final_room = self.model.external_temperature
        else:
            final_room = float(self.config.get(CONF_INITIAL_TEMP, DEFAULT_INITIAL_TEMP))

        # Resolve final fabric temperature
        if t_fabric is not None:
            final_fabric = t_fabric
        elif p_fabric is not None:
            final_fabric = p_fabric
        elif preset == PRESET_COLD_START:
            final_fabric = self.model.external_temperature
        else:
            # No preset, no explicit value — default to room temp (fully equilibrated)
            final_fabric = final_room

        # Resolve final radiator temperature — default to room temp (cold radiator)
        final_rad = t_rad if t_rad is not None else final_room

        # --- Apply to model ---
        m = self.model

        if hasattr(m, "t_air"):
            m.t_air = final_room
        elif hasattr(m, "t_room"):
            m.t_room = final_room
        else:
            m.temperature = final_room

        if hasattr(m, "t_fabric"):
            m.t_fabric = final_fabric

        if hasattr(m, "t_rad"):
            m.t_rad = final_rad

        # Reset heater/valve state — model starts from rest
        m.power_setpoint = 0.0
        m.effective_heater_power = 0.0
        if hasattr(m, "valve_fraction"):
            m.valve_fraction = 0.0

        # Flush pipe delay queue — stale commands must not carry over
        if hasattr(m, "_pipe_queue") and hasattr(m, "_pipe_queue_size"):
            m._pipe_queue.clear()
            m._pipe_accum = 0.0
            for _ in range(m._pipe_queue_size):
                m._pipe_queue.append(0.0)

        # Reset diagnostic rates so sensors don't show stale values
        m.heating_rate = 0.0
        m.heat_loss_rate = 0.0
        m.net_heat_rate = 0.0

        _LOGGER.info(
            "Model reset: preset=%s t_room=%.1f t_fabric=%.1f t_rad=%.1f",
            preset or "manual",
            final_room,
            final_fabric,
            final_rad,
        )
        self._notify_listeners()

    def _subscribe_ext_temp(self) -> None:
        entity = self.config.get(CONF_EXTERNAL_TEMP, "")
        if not entity:
            return
        self._unsub_ext_temp = async_track_state_change_event(
            self.hass, [entity], self._async_ext_temp_changed
        )
        state = self.hass.states.get(entity)
        if state and state.state not in ("unknown", "unavailable"):
            try:
                self.model.set_external_temperature(float(state.state))
            except ValueError:
                pass

    def _subscribe_solar(self) -> None:
        if self.model_type not in (MODEL_R2C2, MODEL_R2C2_RADIATOR):
            return
        entity = self.config.get(CONF_SOLAR_ENTITY, "")
        if not entity:
            return
        self._unsub_solar = async_track_state_change_event(
            self.hass, [entity], self._async_solar_changed
        )
        state = self.hass.states.get(entity)
        if state and state.state not in ("unknown", "unavailable"):
            try:
                self.model.set_solar_irradiance(float(state.state))
            except (ValueError, AttributeError):
                pass

    def _subscribe_flow_temp(self) -> None:
        if self.model_type not in (MODEL_RADIATOR, MODEL_R2C2_RADIATOR):
            return
        entity = self.config.get(CONF_FLOW_TEMP_ENTITY, "")
        if not entity:
            return
        self._unsub_flow_temp = async_track_state_change_event(
            self.hass, [entity], self._async_flow_temp_changed
        )
        state = self.hass.states.get(entity)
        if state and state.state not in ("unknown", "unavailable"):
            try:
                self.model.set_flow_temperature(float(state.state))
            except (ValueError, AttributeError):
                pass

    @callback
    def async_stop(self) -> None:
        for unsub in (
            self._unsub_interval,
            self._unsub_ext_temp,
            self._unsub_solar,
            self._unsub_flow_temp,
        ):
            if unsub:
                unsub()

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    @callback
    def _async_tick(self, _now) -> None:
        self.model.step(self.update_interval)
        self._notify_listeners()

    # ------------------------------------------------------------------
    # Entity state callbacks
    # ------------------------------------------------------------------

    @callback
    def _async_ext_temp_changed(self, event) -> None:
        new_state = event.data.get("new_state")
        if new_state and new_state.state not in ("unknown", "unavailable"):
            try:
                self.model.set_external_temperature(float(new_state.state))
            except ValueError:
                pass

    @callback
    def _async_solar_changed(self, event) -> None:
        new_state = event.data.get("new_state")
        if new_state and new_state.state not in ("unknown", "unavailable"):
            try:
                self.model.set_solar_irradiance(float(new_state.state))
            except (ValueError, AttributeError):
                pass

    @callback
    def _async_flow_temp_changed(self, event) -> None:
        new_state = event.data.get("new_state")
        if new_state and new_state.state not in ("unknown", "unavailable"):
            try:
                self.model.set_flow_temperature(float(new_state.state))
            except (ValueError, AttributeError):
                pass

    # ------------------------------------------------------------------
    # Power control (unified API for entities)
    # ------------------------------------------------------------------

    def set_linear_power(self, percent: float) -> None:
        self.model.set_power_fraction(percent / 100.0)
        self._notify_listeners()

    def set_pwm_switch(self, on: bool) -> None:
        self._pwm_on = on
        self.model.set_power_fraction(1.0 if on else 0.0)
        self._notify_listeners()

    @property
    def pwm_on(self) -> bool:
        return self._pwm_on

    @property
    def power_percent(self) -> float:
        return self.model.power_setpoint * 100.0

    # ------------------------------------------------------------------
    # Listeners
    # ------------------------------------------------------------------

    def register_listener(self, cb) -> callback:
        self._listeners.append(cb)

        @callback
        def unsubscribe():
            if cb in self._listeners:
                self._listeners.remove(cb)

        return unsubscribe

    @callback
    def _notify_listeners(self) -> None:
        for cb in self._listeners:
            cb()
