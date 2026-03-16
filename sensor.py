"""Sensor entities for Heating Simulator — all three model types."""
from __future__ import annotations
import logging
import random
import time

from homeassistant.components.sensor import (
    SensorDeviceClass,
    SensorEntity,
    SensorStateClass,
    RestoreSensor,
)
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import UnitOfTemperature, UnitOfPower
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.entity import DeviceInfo
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .const import (
    DOMAIN, MODEL_R2C2, MODEL_RADIATOR, MODEL_R2C2_RADIATOR,
    CONF_SENSOR_NOISE_STD_DEV, CONF_SENSOR_BIAS, CONF_SENSOR_QUANTISATION,
    CONF_SENSOR_LAG_TAU, CONF_SENSOR_UPDATE_RATE,
    DEFAULT_SENSOR_NOISE_STD_DEV, DEFAULT_SENSOR_BIAS, DEFAULT_SENSOR_QUANTISATION,
    DEFAULT_SENSOR_LAG_TAU, DEFAULT_SENSOR_UPDATE_RATE,
    CONF_SENSOR_MIN_INTERVAL, CONF_SENSOR_MAX_INTERVAL, CONF_SENSOR_DELTA,
    DEFAULT_SENSOR_MIN_INTERVAL, DEFAULT_SENSOR_MAX_INTERVAL, DEFAULT_SENSOR_DELTA,
)
from . import HeatingSimulator

_LOGGER = logging.getLogger(__name__)


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    simulator: HeatingSimulator = hass.data[DOMAIN][entry.entry_id]

    # Common sensors for all model types
    entities = [
        TemperatureSensor(simulator, entry),
        TrueTemperatureSensor(simulator, entry),
        HeatingRateSensor(simulator, entry),
        HeatLossRateSensor(simulator, entry),
        NetHeatRateSensor(simulator, entry),
        EffectivePowerSensor(simulator, entry),
        ExternalTempSensor(simulator, entry),
    ]

    # Model-specific sensors
    if simulator.model_type == MODEL_R2C2:
        entities += [
            FabricTemperatureSensor(simulator, entry),
            SolarGainSensor(simulator, entry),
            TotalHeatLossSensor(simulator, entry),
        ]
    elif simulator.model_type == MODEL_RADIATOR:
        entities += [
            RadiatorTemperatureSensor(simulator, entry),
            RadiatorQInSensor(simulator, entry),
            RadiatorQOutSensor(simulator, entry),
            ReturnTemperatureSensor(simulator, entry),
        ]
    elif simulator.model_type == MODEL_R2C2_RADIATOR:
        # Full set: all R2C2 sensors + all radiator sensors
        entities += [
            FabricTemperatureSensor(simulator, entry),
            SolarGainSensor(simulator, entry),
            TotalHeatLossSensor(simulator, entry),
            RadiatorTemperatureSensor(simulator, entry),
            RadiatorQInSensor(simulator, entry),
            RadiatorQOutSensor(simulator, entry),
            ReturnTemperatureSensor(simulator, entry),
        ]
    else:
        # Simple model: steady-state temp is meaningful
        entities.append(SteadyStateTempSensor(simulator, entry))

    async_add_entities(entities)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _device(entry: ConfigEntry) -> DeviceInfo:
    from .const import MODEL_SIMPLE, MODEL_R2C2, MODEL_RADIATOR
    models = {MODEL_SIMPLE: "Simple R1C1", MODEL_R2C2: "R2C2 + Solar", MODEL_RADIATOR: "Wet Radiator"}
    sim: HeatingSimulator = entry.runtime_data if hasattr(entry, "runtime_data") else None
    model_name = "Thermal Room Model"
    return DeviceInfo(
        identifiers={(DOMAIN, entry.entry_id)},
        name=entry.title,
        manufacturer="Heating Simulator",
        model=model_name,
    )


class _Base(SensorEntity):
    _attr_has_entity_name = True
    _attr_should_poll = False

    def __init__(self, sim: HeatingSimulator, entry: ConfigEntry) -> None:
        self._sim = sim
        self._attr_device_info = DeviceInfo(identifiers={(DOMAIN, entry.entry_id)}, name=entry.title)
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


# ---------------------------------------------------------------------------
# Common sensors
# ---------------------------------------------------------------------------

class TemperatureSensor(_Base, RestoreSensor):
    """
    Primary room air temperature sensor.

    Extends RestoreSensor so the last known value survives HA restarts and
    can be injected back into the thermal model before the first tick fires.

    Applies the sensor imperfection pipeline in order:
      1. Sensor lag    — first-order low-pass  (F-04)
      2. Bias          — fixed additive offset  (F-07)
      3. Noise         — Gaussian sample        (F-02)
      4. Quantisation  — round to step size     (F-02)
      5. Rate-limit    — suppress if interval not elapsed (F-04)
      6. Zigbee-style  — min interval / max interval (heartbeat) / delta (F-NEW)

    All pipeline stages are disabled when their parameter is 0 (the default),
    making the sensor behave identically to the pre-feature version.

    Stages 5 and 6 are mutually exclusive: sensor_update_rate and the Zigbee
    parameters cannot both be non-zero simultaneously (enforced by config flow
    validation).
    """
    _attr_name = "Room Temperature"
    _attr_device_class = SensorDeviceClass.TEMPERATURE
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = UnitOfTemperature.CELSIUS
    _attr_suggested_display_precision = 2

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_temperature"

        # --- Sensor imperfection state ---
        # Lag filter: initialised to None; set on first tick from true model temp
        self._lagged_temp: float | None = None
        # Stage 5 rate-limiter: last time we emitted a new value (monotonic clock)
        self._last_report_time: float = 0.0
        # Stage 5 rate-limiter: the last value we actually reported (so we can repeat it)
        self._last_reported_value: float | None = None
        # Stage 6 Zigbee reporting state
        self._zigbee_last_report_time: float = 0.0
        self._zigbee_last_reported_value: float | None = None

    def _sensor_cfg(self) -> dict:
        """Return merged data+options config for the parent entry."""
        return {**self._sim.entry.data, **self._sim.entry.options}

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        # Attempt to restore the last known room temperature
        last = await self.async_get_last_sensor_data()
        if last is not None and last.native_value is not None:
            try:
                t_room = float(last.native_value)
                self._sim.restore_temperatures(t_room=t_room)
                # Seed the lag filter so it doesn't start from zero
                self._lagged_temp = t_room
                self._last_reported_value = t_room
                # Seed Zigbee state — prevents a spurious immediate heartbeat after restart
                self._zigbee_last_reported_value = t_room
                self._zigbee_last_report_time = time.monotonic()
                _LOGGER.debug(
                    "Restored room temperature %.2f°C from previous state", t_room
                )
            except (ValueError, TypeError) as exc:
                _LOGGER.warning("Could not restore room temperature: %s", exc)

    @property
    def native_value(self):
        cfg = self._sensor_cfg()
        noise_std   = float(cfg.get(CONF_SENSOR_NOISE_STD_DEV, DEFAULT_SENSOR_NOISE_STD_DEV))
        bias        = float(cfg.get(CONF_SENSOR_BIAS,           DEFAULT_SENSOR_BIAS))
        quantise    = float(cfg.get(CONF_SENSOR_QUANTISATION,   DEFAULT_SENSOR_QUANTISATION))
        lag_tau     = float(cfg.get(CONF_SENSOR_LAG_TAU,        DEFAULT_SENSOR_LAG_TAU))
        update_rate = float(cfg.get(CONF_SENSOR_UPDATE_RATE,    DEFAULT_SENSOR_UPDATE_RATE))

        true_temp = self._sim.model.temperature

        # ------------------------------------------------------------------
        # Stage 1 — Sensor lag (F-04): first-order low-pass filter
        #   lagged += dt/tau * (true - lagged)   →  α = dt / (tau + dt)
        # We use the simulator's update interval as dt.
        # ------------------------------------------------------------------
        if lag_tau > 0.0:
            dt = float(self._sim.update_interval)  # seconds per tick
            alpha = dt / (lag_tau + dt)
            if self._lagged_temp is None:
                self._lagged_temp = true_temp      # cold-start: no lag on first tick
            else:
                self._lagged_temp += alpha * (true_temp - self._lagged_temp)
            value = self._lagged_temp
        else:
            # No lag — reset state so switching lag on mid-run starts cleanly
            self._lagged_temp = true_temp
            value = true_temp

        # ------------------------------------------------------------------
        # Stage 2 — Bias (F-07): fixed additive offset
        # ------------------------------------------------------------------
        value += bias

        # ------------------------------------------------------------------
        # Stage 3 — Noise (F-02): Gaussian, σ = noise_std_dev
        # ------------------------------------------------------------------
        if noise_std > 0.0:
            value += random.gauss(0.0, noise_std)

        # ------------------------------------------------------------------
        # Stage 4 — Quantisation (F-02): round to nearest step
        # ------------------------------------------------------------------
        if quantise > 0.0:
            value = round(value / quantise) * quantise

        # ------------------------------------------------------------------
        # Stage 5 — Rate-limit (F-04): suppress report until interval elapses
        # ------------------------------------------------------------------
        if update_rate > 0.0:
            now = time.monotonic()
            if (now - self._last_report_time) < update_rate:
                # Interval not yet elapsed — return the previous reported value.
                # If we have no previous value yet (first tick), fall through.
                if self._last_reported_value is not None:
                    return round(self._last_reported_value, 3)
            # Interval elapsed (or first report): latch the new value and timestamp
            self._last_report_time = now
            self._last_reported_value = value
        else:
            self._last_reported_value = value

        # ------------------------------------------------------------------
        # Stage 6 — Zigbee-style conditional reporting (F-NEW)
        #
        # Three cooperating rules evaluated against the post-quantisation value:
        #   min_interval  — shortest time between any two reports
        #   max_interval  — heartbeat: always report once this elapses
        #   delta         — minimum change vs last reported value to trigger a report
        #
        # Logic table:
        #   heartbeat_due=True               → always emit (keepalive)
        #   heartbeat_due=False, min elapsed, delta crossed → emit (change report)
        #   anything else                    → suppress, return last reported value
        #
        # Mutually exclusive with Stage 5 (update_rate) — enforced by config
        # flow validation; both being non-zero simultaneously is not permitted.
        # ------------------------------------------------------------------
        min_interval = float(cfg.get(CONF_SENSOR_MIN_INTERVAL, DEFAULT_SENSOR_MIN_INTERVAL))
        max_interval = float(cfg.get(CONF_SENSOR_MAX_INTERVAL, DEFAULT_SENSOR_MAX_INTERVAL))
        delta        = float(cfg.get(CONF_SENSOR_DELTA,        DEFAULT_SENSOR_DELTA))

        if min_interval > 0.0 or max_interval > 0.0 or delta > 0.0:
            now = time.monotonic()

            # First tick after cold start or HA restart: always report and seed state
            if self._zigbee_last_reported_value is None:
                self._zigbee_last_report_time = now
                self._zigbee_last_reported_value = value
                # fall through — value reported normally

            else:
                elapsed = now - self._zigbee_last_report_time
                change  = abs(value - self._zigbee_last_reported_value)

                heartbeat_due = (max_interval > 0.0) and (elapsed >= max_interval)
                min_elapsed   = (min_interval <= 0.0) or (elapsed >= min_interval)
                delta_crossed = (delta <= 0.0) or (change >= delta)

                if heartbeat_due:
                    # Max interval elapsed — always report regardless of delta / min
                    self._zigbee_last_report_time = now
                    self._zigbee_last_reported_value = value
                    # fall through

                elif min_elapsed and delta_crossed:
                    # Both min interval and delta conditions satisfied — emit
                    self._zigbee_last_report_time = now
                    self._zigbee_last_reported_value = value
                    # fall through

                else:
                    # Suppress — return the last successfully reported value
                    return round(self._zigbee_last_reported_value, 3)

        return round(value, 3)

    @property
    def extra_state_attributes(self):
        cfg = self._sensor_cfg()
        attrs = {"model_type": self._sim.model_type}
        # Expose active imperfection parameters for transparency / debugging
        noise_std    = float(cfg.get(CONF_SENSOR_NOISE_STD_DEV,  DEFAULT_SENSOR_NOISE_STD_DEV))
        bias         = float(cfg.get(CONF_SENSOR_BIAS,            DEFAULT_SENSOR_BIAS))
        quantise     = float(cfg.get(CONF_SENSOR_QUANTISATION,    DEFAULT_SENSOR_QUANTISATION))
        lag_tau      = float(cfg.get(CONF_SENSOR_LAG_TAU,         DEFAULT_SENSOR_LAG_TAU))
        update_rate  = float(cfg.get(CONF_SENSOR_UPDATE_RATE,     DEFAULT_SENSOR_UPDATE_RATE))
        min_interval = float(cfg.get(CONF_SENSOR_MIN_INTERVAL,    DEFAULT_SENSOR_MIN_INTERVAL))
        max_interval = float(cfg.get(CONF_SENSOR_MAX_INTERVAL,    DEFAULT_SENSOR_MAX_INTERVAL))
        delta        = float(cfg.get(CONF_SENSOR_DELTA,           DEFAULT_SENSOR_DELTA))

        if any([noise_std, bias, quantise, lag_tau, update_rate,
                min_interval, max_interval, delta]):
            attrs["sensor_noise_std_dev"]  = noise_std
            attrs["sensor_bias"]           = bias
            attrs["sensor_quantisation"]   = quantise
            attrs["sensor_lag_tau_s"]      = lag_tau
            attrs["sensor_update_rate_s"]  = update_rate
            attrs["sensor_min_interval_s"] = min_interval
            attrs["sensor_max_interval_s"] = max_interval
            attrs["sensor_delta"]          = delta
            if self._lagged_temp is not None and lag_tau > 0.0:
                attrs["sensor_lagged_temp"] = round(self._lagged_temp, 3)
            if self._zigbee_last_reported_value is not None and any([min_interval, max_interval, delta]):
                attrs["sensor_zigbee_last_value"] = round(self._zigbee_last_reported_value, 3)
        return attrs


class TrueTemperatureSensor(_Base):
    """
    Exposes the unmodified model room temperature as a standalone sensor.

    This is the value before the F-02/F-04/F-07 imperfection pipeline is
    applied. Useful for comparing against the degraded TemperatureSensor
    to observe the effect of noise, lag, bias, and quantisation, and for
    confirming the model is evolving correctly independent of sensor artefacts.

    Intentionally not a RestoreSensor — the model true temperature is
    restored via TemperatureSensor; this entity just reads it.
    """
    _attr_name = "Room Temperature (True)"
    _attr_device_class = SensorDeviceClass.TEMPERATURE
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = UnitOfTemperature.CELSIUS
    _attr_suggested_display_precision = 3
    _attr_icon = "mdi:thermometer-check"

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_true_temperature"

    @property
    def native_value(self):
        return round(self._sim.model.temperature, 3)

    @property
    def extra_state_attributes(self):
        return {"model_type": self._sim.model_type}


class HeatingRateSensor(_Base):
    _attr_name = "Heating Rate"
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = "°C/s"
    _attr_icon = "mdi:thermometer-plus"
    _attr_suggested_display_precision = 5

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_heating_rate"

    @property
    def native_value(self):
        return round(self._sim.model.heating_rate, 6)

    @property
    def extra_state_attributes(self):
        return {"per_hour": round(self._sim.model.heating_rate * 3600, 3)}


class HeatLossRateSensor(_Base):
    _attr_name = "Heat Loss Rate"
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = "°C/s"
    _attr_icon = "mdi:thermometer-minus"
    _attr_suggested_display_precision = 5

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_heat_loss_rate"

    @property
    def native_value(self):
        return round(self._sim.model.heat_loss_rate, 6)

    @property
    def extra_state_attributes(self):
        return {"per_hour": round(self._sim.model.heat_loss_rate * 3600, 3)}


class NetHeatRateSensor(_Base):
    _attr_name = "Net Heating Rate"
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = "°C/s"
    _attr_icon = "mdi:thermometer-lines"
    _attr_suggested_display_precision = 5

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_net_heat_rate"

    @property
    def native_value(self):
        return round(self._sim.model.net_heat_rate, 6)

    @property
    def extra_state_attributes(self):
        return {
            "per_hour": round(self._sim.model.net_heat_rate * 3600, 3),
            "trend": "heating" if self._sim.model.net_heat_rate > 0 else "cooling",
        }


class EffectivePowerSensor(_Base):
    _attr_name = "Effective Heater Power"
    _attr_device_class = SensorDeviceClass.POWER
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = UnitOfPower.WATT
    _attr_suggested_display_precision = 1

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_effective_power"

    @property
    def native_value(self):
        return round(self._sim.model.effective_heater_power, 1)

    @property
    def extra_state_attributes(self):
        return {
            "power_setpoint_pct": round(self._sim.model.power_setpoint * 100, 1),
            **self._sim.model.extra_state,
        }


class ExternalTempSensor(_Base):
    _attr_name = "External Temperature"
    _attr_device_class = SensorDeviceClass.TEMPERATURE
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = UnitOfTemperature.CELSIUS
    _attr_suggested_display_precision = 1

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_external_temp"

    @property
    def native_value(self):
        return round(self._sim.model.external_temperature, 2)


# ---------------------------------------------------------------------------
# Simple model sensors
# ---------------------------------------------------------------------------

class SteadyStateTempSensor(_Base):
    _attr_name = "Steady State Temperature"
    _attr_device_class = SensorDeviceClass.TEMPERATURE
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = UnitOfTemperature.CELSIUS
    _attr_icon = "mdi:thermometer-check"
    _attr_suggested_display_precision = 1

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_steady_state_temp"

    @property
    def native_value(self):
        return round(self._sim.model.steady_state_temperature, 2)

    @property
    def extra_state_attributes(self):
        return {
            "room_tau_s": round(self._sim.model.time_to_equilibrium_tau, 1),
            "room_tau_min": round(self._sim.model.time_to_equilibrium_tau / 60, 2),
        }


# ---------------------------------------------------------------------------
# R2C2 model sensors
# ---------------------------------------------------------------------------

class FabricTemperatureSensor(_Base, RestoreSensor):
    """
    Building fabric temperature. Uses RestoreSensor so the fabric thermal
    state (which can take hours to change significantly) is preserved across restarts.
    """
    _attr_name = "Fabric Temperature"
    _attr_device_class = SensorDeviceClass.TEMPERATURE
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = UnitOfTemperature.CELSIUS
    _attr_icon = "mdi:wall"
    _attr_suggested_display_precision = 2

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_fabric_temp"

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        last = await self.async_get_last_sensor_data()
        if last is not None and last.native_value is not None:
            try:
                t_fabric = float(last.native_value)
                self._sim.restore_temperatures(t_room=None, t_fabric=t_fabric)
                _LOGGER.debug("Restored fabric temperature %.2f°C", t_fabric)
            except (ValueError, TypeError) as exc:
                _LOGGER.warning("Could not restore fabric temperature: %s", exc)

    @property
    def native_value(self):
        return round(self._sim.model.t_fabric, 3)

    @property
    def extra_state_attributes(self):
        m = self._sim.model
        return {
            "delta_air_fabric": round(m.t_air - m.t_fabric, 3),
        }


class SolarGainSensor(_Base):
    _attr_name = "Solar Gain"
    _attr_device_class = SensorDeviceClass.POWER
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = UnitOfPower.WATT
    _attr_icon = "mdi:solar-power"
    _attr_suggested_display_precision = 1

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_solar_gain"

    @property
    def native_value(self):
        return round(self._sim.model.solar_gain_watts, 1)


class TotalHeatLossSensor(_Base):
    _attr_name = "Total Heat Loss"
    _attr_device_class = SensorDeviceClass.POWER
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = UnitOfPower.WATT
    _attr_icon = "mdi:home-thermometer-outline"
    _attr_suggested_display_precision = 1

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_total_heat_loss"

    @property
    def native_value(self):
        return round(self._sim.model.total_heat_loss_watts, 1)


# ---------------------------------------------------------------------------
# Wet radiator model sensors
# ---------------------------------------------------------------------------

class RadiatorTemperatureSensor(_Base, RestoreSensor):
    """
    Radiator water temperature. Uses RestoreSensor so the radiator thermal
    state is preserved across HA restarts.  Restoring this on boot avoids the
    simulation starting from a cold radiator when the real system was already warm.
    """
    _attr_name = "Radiator Temperature"
    _attr_device_class = SensorDeviceClass.TEMPERATURE
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = UnitOfTemperature.CELSIUS
    _attr_icon = "mdi:radiator"
    _attr_suggested_display_precision = 2

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_radiator_temp"

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        last = await self.async_get_last_sensor_data()
        if last is not None and last.native_value is not None:
            try:
                t_rad = float(last.native_value)
                self._sim.restore_temperatures(t_room=None, t_rad=t_rad)
                _LOGGER.debug("Restored radiator temperature %.2f°C", t_rad)
            except (ValueError, TypeError) as exc:
                _LOGGER.warning("Could not restore radiator temperature: %s", exc)

    @property
    def native_value(self):
        return round(self._sim.model.t_rad, 3)

    @property
    def extra_state_attributes(self):
        m = self._sim.model
        # R2C2RadiatorModel uses t_air; WetRadiatorModel uses t_room
        t_room = m.t_air if hasattr(m, "t_air") else m.t_room
        return {
            "flow_temperature": m.flow_temperature,
            "delta_t_rad_room": round(m.t_rad - t_room, 3),
            "valve_position_pct": round(m.valve_fraction * 100, 1),
        }


class RadiatorQInSensor(_Base):
    _attr_name = "Radiator Heat Input"
    _attr_device_class = SensorDeviceClass.POWER
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = UnitOfPower.WATT
    _attr_icon = "mdi:water-boiler"
    _attr_suggested_display_precision = 1

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_rad_q_in"

    @property
    def native_value(self):
        return round(self._sim.model.q_in_watts, 1)


class RadiatorQOutSensor(_Base):
    _attr_name = "Radiator Heat Output"
    _attr_device_class = SensorDeviceClass.POWER
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = UnitOfPower.WATT
    _attr_icon = "mdi:radiator"
    _attr_suggested_display_precision = 1

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_rad_q_out"

    @property
    def native_value(self):
        return round(self._sim.model.q_out_watts, 1)


class ReturnTemperatureSensor(_Base):
    _attr_name = "Return Temperature"
    _attr_device_class = SensorDeviceClass.TEMPERATURE
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_native_unit_of_measurement = UnitOfTemperature.CELSIUS
    _attr_icon = "mdi:thermometer-water"
    _attr_suggested_display_precision = 2

    def __init__(self, sim, entry):
        super().__init__(sim, entry)
        self._attr_unique_id = f"{entry.entry_id}_return_temp"

    @property
    def native_value(self):
        return round(self._sim.model.t_return, 2)
