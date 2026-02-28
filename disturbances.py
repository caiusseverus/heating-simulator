"""
Disturbance profile generators for the Heating Simulator.

Each class is a self-contained, stateless-or-seedable generator that
produces a value for a given simulated time (seconds since midnight).
All classes are pure Python with no Home Assistant dependencies so they
can be unit-tested independently.

Provided:
  ExternalTempProfile  (F-11) — realistic day/night external temperature curve
  OccupancyProfile     (F-05) — stochastic room occupancy + internal heat gain
  WeatherProfile       (F-06, F-14) — wind and rain effects on K-value

Design rules
------------
- All parameters default to neutral values (no effect on the simulation).
- Profiles are evaluated on demand; they hold minimal mutable state (only
  a cached daily schedule so we don't regenerate on every tick).
- Time is always absolute simulated seconds from midnight of day 0 so the
  profiles wrap correctly across day boundaries.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# F-11  External temperature profile
# ---------------------------------------------------------------------------

@dataclass
class ExternalTempProfile:
    """
    Realistic external temperature that varies over a 24-hour period.

    The curve is NOT sinusoidal. Real outdoor temperature has:
      - A slow rise from the pre-dawn minimum, accelerating after sunrise.
      - A peak in mid-afternoon (typically 14:00–15:00 local solar time).
      - A gradual fall through evening, steepening after midnight.

    This is approximated with a piece-wise raised-cosine that places the
    minimum at `min_hour` and the maximum at `max_hour`, with an
    asymmetric rise/fall so the warm period is shorter than the cold period
    (more pronounced for short winter days).

    Parameters
    ----------
    enabled : bool
        Master switch. When False, `temperature_at()` returns `base_temp`
        unchanged (i.e. a fixed external temperature, back-compatible).
    base_temp : float
        Mean temperature for the day (°C). The midpoint of min/max.
    temp_amplitude : float
        Half-swing from mean to peak/trough (°C). Daily range = 2 × amplitude.
        Typical UK winter: 2–4°C amplitude. Summer: 5–8°C.
    min_hour : float
        Hour of day (0–24) at which the minimum occurs. Typically 05:00–06:00.
    max_hour : float
        Hour of day (0–24) at which the maximum occurs. Typically 14:00–15:00.
    """

    enabled: bool = False
    base_temp: float = 5.0
    temp_amplitude: float = 3.0
    min_hour: float = 5.5    # 05:30 — pre-dawn minimum
    max_hour: float = 14.5   # 14:30 — mid-afternoon peak

    def temperature_at(self, sim_seconds: float) -> float:
        """
        Return external temperature (°C) at `sim_seconds` seconds since
        midnight. The profile repeats with a 24-hour period.
        """
        if not self.enabled:
            return self.base_temp

        hour = (sim_seconds / 3600.0) % 24.0  # fractional hour within day

        min_h = self.min_hour % 24.0
        max_h = self.max_hour % 24.0

        # Duration of the warming phase (trough → peak) and cooling phase (peak → trough).
        # Handle wrap-around midnight for either boundary.
        if max_h > min_h:
            warm_duration = max_h - min_h
        else:
            warm_duration = (24.0 - min_h) + max_h

        cool_duration = 24.0 - warm_duration

        # Hours elapsed since the trough
        hours_since_min = (hour - min_h) % 24.0

        if hours_since_min <= warm_duration:
            # Rising limb: raised cosine maps [0, warm_duration] → [-1, +1]
            t = hours_since_min / warm_duration
            phase_value = -math.cos(math.pi * t)
        else:
            # Falling limb: raised cosine maps [0, cool_duration] → [+1, -1]
            t = (hours_since_min - warm_duration) / cool_duration
            phase_value = math.cos(math.pi * t)

        return self.base_temp + self.temp_amplitude * phase_value


# ---------------------------------------------------------------------------
# F-05  Occupancy and internal heat gain
# ---------------------------------------------------------------------------

# Sensible heat emitted per seated adult (CIBSE Guide A, light activity).
_HEAT_PER_PERSON_W = 90.0  # W


@dataclass
class OccupancyProfile:
    """
    Stochastic room occupancy and internal heat gain (F-05).

    Generates a time-varying occupant count and cooking/appliance heat gain
    that a controller must reject.

    Occupancy model
    ---------------
    - Night (22:00–07:00): room always empty.
    - Daytime: occupant count varies stochastically between 0 and
      `max_occupants`. Visits are either long (1–3 h, seated) or short
      (5–25 min, passing through), with random gaps between them.
    - The schedule is regenerated each simulated day, deterministically
      from `seed + day_number` so runs are reproducible.

    Cooking / appliance events (F-05 extension)
    -------------------------------------------
    - `cooking_power_watts`: peak gain during an event (W). 0 = disabled.
    - `cooking_duration_s`: typical event length (s).
    - `cooking_events_per_day`: Poisson mean events per day.
    - Events are placed between 08:00 and 21:00.

    Parameters
    ----------
    enabled : bool
        Master switch. gain_at() returns 0.0 when False.
    max_occupants : int
        Maximum simultaneous occupants.
    cooking_power_watts : float
        Internal gain during a cooking event (W). 0 = disabled.
    cooking_duration_s : float
        Typical cooking event length (s).
    cooking_events_per_day : float
        Mean cooking events per day (Poisson).
    seed : int | None
        Reproducibility seed. None = non-deterministic.
    """

    enabled: bool = False
    max_occupants: int = 2
    cooking_power_watts: float = 0.0
    cooking_duration_s: float = 1200.0   # 20 min default
    cooking_events_per_day: float = 2.0
    seed: int | None = 42

    # Cached daily schedule — rebuilt only when the simulated day changes.
    # These are not dataclass fields so they don't appear in __init__.
    def __post_init__(self) -> None:
        self._schedule_day: int = -1
        self._occupancy_events: list[tuple[float, float, int]] = []
        self._cooking_events: list[tuple[float, float]] = []

    def gain_at(self, sim_seconds: float) -> float:
        """
        Return total internal heat gain (W) at `sim_seconds` seconds since
        midnight of day 0. Intended to be called once per simulator tick.
        """
        if not self.enabled:
            return 0.0

        day = int(sim_seconds // 86400)
        time_of_day = sim_seconds % 86400.0

        if day != self._schedule_day:
            self._build_day_schedule(day)

        # Occupant gain: find the event covering this time slot
        occupants = 0
        for start, end, count in self._occupancy_events:
            if start <= time_of_day < end:
                occupants = count
                break

        # Cooking gain
        q_cooking = 0.0
        if self.cooking_power_watts > 0:
            for start, end in self._cooking_events:
                if start <= time_of_day < end:
                    q_cooking = self.cooking_power_watts
                    break

        return occupants * _HEAT_PER_PERSON_W + q_cooking

    def _build_day_schedule(self, day: int) -> None:
        """
        Generate the full occupancy and cooking schedule for `day`.

        Re-seeded from (seed + day) so each day is independent but
        deterministic for a fixed seed.
        """
        rng = random.Random(
            (self.seed if self.seed is not None else 0) + day * 1_000_003
        )

        DAY_START   =  7 * 3600   # 07:00
        DAY_END     = 22 * 3600   # 22:00

        events: list[tuple[float, float, int]] = []
        t = float(DAY_START)

        while t < DAY_END and self.max_occupants > 0:
            count = rng.randint(0, self.max_occupants)

            # Bimodal duration: 30% chance of a short visit, 70% long
            if rng.random() < 0.30:
                duration = rng.uniform(5 * 60, 25 * 60)    # short: 5–25 min
            else:
                duration = rng.uniform(60 * 60, 3 * 3600)  # long: 1–3 h

            end = min(t + duration, float(DAY_END))
            events.append((t, end, count))
            t = end

            # Gap before next event (2–30 min)
            t += rng.uniform(2 * 60, 30 * 60)

        self._occupancy_events = events

        # Cooking events
        cooking: list[tuple[float, float]] = []
        if self.cooking_power_watts > 0 and self.cooking_events_per_day > 0:
            COOK_START = 8 * 3600
            COOK_END   = 21 * 3600
            n = _poisson_sample(rng, self.cooking_events_per_day)
            for _ in range(n):
                max_start = COOK_END - self.cooking_duration_s
                if max_start <= COOK_START:
                    break
                start = rng.uniform(COOK_START, max_start)
                dur = self.cooking_duration_s * rng.uniform(0.7, 1.3)
                end = min(start + dur, float(COOK_END))
                cooking.append((start, end))
            cooking.sort()

        self._cooking_events = cooking
        self._schedule_day = day


def _poisson_sample(rng: random.Random, mean: float) -> int:
    """Knuth's Poisson sampler. Accurate for mean < ~30."""
    if mean <= 0:
        return 0
    L = math.exp(-mean)
    k, p = 0, 1.0
    while p > L:
        k += 1
        p *= rng.random()
    return k - 1


# ---------------------------------------------------------------------------
# F-06, F-14  Wind and rain effects on heat loss
# ---------------------------------------------------------------------------

@dataclass
class WeatherProfile:
    """
    Wind and rain effects on the effective heat loss coefficient (F-06, F-14).

    The combined effect is expressed as a dimensionless multiplier on the
    base K-value (or equivalent 1/R for R2C2 models):

        K_eff = K_base × multiplier

    Wind effect (F-06)
    ------------------
    Wind increases infiltration and convective surface film conductance.
    At zero wind the contribution is 1.0 (no change).

        wind_factor = 1.0 + wind_coefficient × wind_speed_m_s

    `wind_coefficient` ≈ 0.02–0.10 per (m/s). At Beaufort 5 (≈10 m/s) and
    coefficient = 0.05, wind_factor = 1.50 (50% more heat loss).

    Rain / moisture effect (F-14)
    ------------------------------
    Wet fabric has higher thermal conductivity, increasing the effective
    U-value of the building envelope.

        rain_factor = 1.0 + rain_moisture_factor × rain_intensity_fraction

    `rain_moisture_factor` ≈ 0.0–0.5. `rain_intensity_fraction` is 0 (dry)
    to 1.0 (heavy rain).

    Combined multiplier
    -------------------
        multiplier = wind_factor × rain_factor

    Both effects default to 0.0 (disabled), preserving existing behaviour
    when neither parameter is set.

    Parameters
    ----------
    wind_speed_m_s : float
        Current wind speed in m/s.
    wind_coefficient : float
        Sensitivity of K to wind speed (per m/s). 0 = no wind effect.
    rain_intensity_fraction : float
        Rain intensity 0.0 (dry) to 1.0 (heavy).
    rain_moisture_factor : float
        Fractional increase in K at full rain intensity. 0 = no rain effect.
    """

    wind_speed_m_s: float = 0.0
    wind_coefficient: float = 0.0
    rain_intensity_fraction: float = 0.0
    rain_moisture_factor: float = 0.0

    @property
    def multiplier(self) -> float:
        """Combined K multiplier. Returns 1.0 when all effects are disabled."""
        return self.wind_factor * self.rain_factor

    @property
    def wind_factor(self) -> float:
        return 1.0 + max(0.0, self.wind_coefficient) * max(0.0, self.wind_speed_m_s)

    @property
    def rain_factor(self) -> float:
        return 1.0 + max(0.0, self.rain_moisture_factor) * max(0.0, self.rain_intensity_fraction)
