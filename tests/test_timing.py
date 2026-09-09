"""Coordinator timing regressions, runnable without a Home Assistant install.

Load the real coordinator class through AST extraction to omit HA-only imports
and service registration. Only clock, persistence, and profiles are stubbed;
control, integration, publication, and thermal-model code run unchanged.
"""

from __future__ import annotations

import ast
import asyncio
import logging
from pathlib import Path
import runpy
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
SimpleThermalModel = runpy.run_path(str(ROOT / "thermal_model.py"))["SimpleThermalModel"]


def load_coordinator(clock):
    source = ast.parse((ROOT / "__init__.py").read_text())
    coordinator = next(
        node for node in source.body
        if isinstance(node, ast.ClassDef) and node.name == "HeatingSimulator"
    )
    module = ast.Module(
        body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), coordinator],
        type_ignores=[],
    )
    namespace = runpy.run_path(str(ROOT / "const.py"))
    namespace.update(
        callback=lambda fn: fn,
        time=clock,
        _LOGGER=logging.getLogger(__name__),
    )
    exec(compile(ast.fix_missing_locations(module), str(ROOT / "__init__.py"), "exec"), namespace)
    return namespace["HeatingSimulator"]


class CoordinatorTimingTests(unittest.TestCase):
    def make_simulator(self, interval=30):
        clock = SimpleNamespace(now=0.0)
        clock.monotonic = lambda: clock.now
        coordinator = load_coordinator(clock)
        sim = coordinator.__new__(coordinator)
        sim.model = SimpleThermalModel(
            heater_power_watts=2000.0,
            heat_loss_coeff=0.0,
            thermal_mass=10000.0,
            thermal_inertia_tau=0.0,
            initial_temp=20.0,
        )
        sim.update_interval = interval
        sim._last_update_time = 0.0
        sim._sim_time_s = 0.0
        sim._pwm_on = False
        sim._listeners = []
        sim._control_listeners = []
        sim._ext_temp_profile = SimpleNamespace(enabled=False)
        sim._occupancy_profile = SimpleNamespace(gain_at=lambda _: 0.0)
        sim._weather_profile = SimpleNamespace(multiplier=1.0)
        sim._schedule_state_save = lambda: None
        return sim, clock

    def tick(self, sim, clock, when):
        clock.now = when
        # HA's wall-clock argument must not determine elapsed physics time.
        asyncio.run(sim._async_tick(None))

    def energy(self, sim):
        return (sim.model.temperature - 20.0) * sim.model.thermal_mass

    def test_fractional_pulses_have_equal_energy_at_every_offset_and_interval(self):
        for interval in (5, 10, 30):
            for start in (0.1, 7.2, 25.3, 35.7):
                with self.subTest(interval=interval, start=start):
                    sim, clock = self.make_simulator(interval)
                    publications = []
                    sim._listeners.append(lambda: publications.append(clock.now))
                    events = [(float(t), None) for t in range(interval, 61, interval)]
                    events.extend([(start, True), (start + 11.4, False)])
                    for when, state in sorted(events):
                        clock.now = when
                        if state is None:
                            self.tick(sim, clock, when)
                        else:
                            sim.set_pwm_switch(state)
                    self.assertAlmostEqual(self.energy(sim), 2000.0 * 11.4, places=7)
                    self.assertAlmostEqual(sim._sim_time_s, 60.0)
                    self.assertEqual(publications, list(range(interval, 61, interval)))

    def test_multiple_short_pulses_between_publications(self):
        sim, clock = self.make_simulator()
        for when, state in ((1.1, True), (1.2, False), (8.3, True), (8.5, False), (29.8, True), (29.9, False)):
            clock.now = when
            sim.set_pwm_switch(state)
        self.tick(sim, clock, 30.0)
        self.assertAlmostEqual(self.energy(sim), 800.0, places=7)

    def test_switch_edge_on_publication_boundary_is_independent_of_callback_order(self):
        for tick_first in (True, False):
            with self.subTest(tick_first=tick_first):
                sim, clock = self.make_simulator()
                clock.now = 30.0
                if tick_first:
                    self.tick(sim, clock, 30.0)
                    sim.set_pwm_switch(True)
                else:
                    sim.set_pwm_switch(True)
                    self.tick(sim, clock, 30.0)
                self.assertAlmostEqual(self.energy(sim), 0.0)
                clock.now = 40.0
                sim.set_pwm_switch(False)
                self.tick(sim, clock, 60.0)
                self.assertAlmostEqual(self.energy(sim), 20000.0, places=7)
                self.assertEqual(sim._sim_time_s, 60.0)

    def test_duplicate_commands_do_not_add_or_drop_time(self):
        sim, clock = self.make_simulator()
        for when, state in ((2.3, True), (2.3, True), (6.7, True), (13.7, False), (15.2, False)):
            clock.now = when
            sim.set_pwm_switch(state)
        self.tick(sim, clock, 30.0)
        self.tick(sim, clock, 30.0)
        self.assertAlmostEqual(self.energy(sim), 22800.0, places=7)
        self.assertAlmostEqual(sim._sim_time_s, 30.0)

    def test_delayed_tick_uses_actual_elapsed_time(self):
        sim, clock = self.make_simulator()
        sim.set_pwm_switch(True)
        self.tick(sim, clock, 37.25)
        self.assertAlmostEqual(self.energy(sim), 74500.0, places=7)
        self.tick(sim, clock, 60.0)
        self.assertAlmostEqual(self.energy(sim), 120000.0, places=7)

    def test_equal_or_older_timestamp_cannot_rewind_physics_clock(self):
        sim, clock = self.make_simulator()
        sim.set_pwm_switch(True)
        sim._advance_to(10.5)
        sim._advance_to(10.5)
        sim._advance_to(5.0)
        self.assertEqual(sim._last_update_time, 10.5)
        sim._advance_to(20.0)
        self.assertAlmostEqual(self.energy(sim), 40000.0, places=7)
        self.assertEqual(sim._sim_time_s, 20.0)

    def test_linear_changes_integrate_previous_power(self):
        sim, clock = self.make_simulator()
        clock.now = 2.5
        sim.set_linear_power(25.0)
        clock.now = 12.5
        sim.set_linear_power(75.0)
        clock.now = 20.0
        sim.set_linear_power(0.0)
        self.tick(sim, clock, 30.0)
        self.assertAlmostEqual(self.energy(sim), 500.0 * 10 + 1500.0 * 7.5, places=7)

    def test_controls_update_immediately_but_sensors_wait_for_publication(self):
        sim, clock = self.make_simulator()
        controls, sensors = [], []
        sim.register_listener(lambda: controls.append((clock.now, sim.pwm_on)), control=True)
        sim.register_listener(lambda: sensors.append(clock.now))
        clock.now = 7.2
        sim.set_pwm_switch(True)
        self.assertEqual(controls, [(7.2, True)])
        self.assertEqual(sensors, [])
        clock.now = 18.6
        sim.set_pwm_switch(False)
        self.assertEqual(controls[-1], (18.6, False))
        self.assertEqual(sensors, [])
        self.tick(sim, clock, 30.0)
        self.assertEqual(sensors, [30.0])

    def test_inactive_clock_does_not_integrate_startup_or_downtime(self):
        sim, clock = self.make_simulator()
        sim._last_update_time = None
        clock.now = 500.0
        sim.set_pwm_switch(True)
        sim._advance_to()
        self.assertEqual(sim.model.temperature, 20.0)
        self.assertEqual(sim._sim_time_s, 0.0)
        self.assertIsNone(sim._last_update_time)

    def test_reset_does_not_replay_elapsed_pre_reset_time(self):
        sim, clock = self.make_simulator()
        sim.set_pwm_switch(True)
        clock.now = 12.5
        sim.reset_model(t_room=20.0)
        self.tick(sim, clock, 30.0)
        self.assertAlmostEqual(self.energy(sim), 2000.0 * 17.5, places=7)


if __name__ == "__main__":
    unittest.main()
