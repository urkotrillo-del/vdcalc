"""Straight-line results: acceleration with gear changes, braking, top speed and a performance summary.

Everything is integrated in the speed domain: dt = dv / a and ds = v dv / a. A gear change adds a time without
drive force during which the car coasts at constant speed (declared simplification).
"""
from dataclasses import dataclass

import numpy as np

from .limits import acceleration, braking_deceleration
from .loads import FLAT
from .units import MPH

# An upshift is made only when another gear accelerates clearly more than the current one [m/s2]. This also
# keeps the gear when all gears accelerate the same (grip-limited), instead of changing at random.
GEAR_HYSTERESIS = 0.02


class SpeedNotReachable(Exception):
    def __init__(self, max_speed: float):
        self.max_speed = max_speed
        super().__init__(f"The car cannot reach this speed. The highest speed it reaches is {max_speed * 3.6:.0f} km/h.")


@dataclass
class RunResult:
    time: np.ndarray        # s
    speed: np.ndarray       # m/s
    distance: np.ndarray    # m
    gear: np.ndarray        # gear index (0 = first); -1 in braking runs

    @property
    def total_time(self) -> float:
        return float(self.time[-1])

    @property
    def total_distance(self) -> float:
        return float(self.distance[-1])

    def _at(self, values: np.ndarray, speed: float) -> float:
        s = self.speed
        if s[0] > s[-1]:                                   # braking run: speed decreases
            s, values = s[::-1], values[::-1]
        i = int(np.searchsorted(s, speed, side="left"))
        if i <= 0:
            return float(values[0])
        if i >= len(s):
            raise ValueError("The run never reaches that speed.")
        f = (speed - s[i - 1]) / (s[i] - s[i - 1]) if s[i] > s[i - 1] else 0.0
        return float(values[i - 1] + f * (values[i] - values[i - 1]))

    def time_at(self, speed: float) -> float:
        return self._at(self.time, speed)

    def distance_at(self, speed: float) -> float:
        return self._at(self.distance, speed)


def accelerate(vehicle, env, v_start, v_end, launch=None, dv=0.2, cond=FLAT) -> RunResult:
    """Full-throttle acceleration from v_start to v_end [m/s], choosing the best gear at every speed.

    launch=None means a standing start (clutch slip below the first point of the torque curve) if v_start is 0.
    Raises SpeedNotReachable if the car stops accelerating before v_end.
    """
    if launch is None:
        launch = v_start <= 0.0
    n = max(int(np.ceil((v_end - v_start) / dv)), 1)
    v = np.linspace(v_start, v_end, n + 1)
    n_gears = len(vehicle.powertrain.gear_ratios)
    a_all = np.array([acceleration(vehicle, env, v, g, cond, launch)[0] for g in range(n_gears)])

    gears = np.empty(len(v), dtype=int)
    a = np.empty(len(v))
    current = None
    for i in range(len(v)):
        col = a_all[:, i]
        valid = np.where(np.isnan(col), -np.inf, col)
        best = int(np.argmax(valid))
        if valid[best] == -np.inf:
            raise SpeedNotReachable(float(v[max(i - 1, 0)]))
        if current is None or valid[current] < valid[best] - GEAR_HYSTERESIS:
            current = best
        gears[i], a[i] = current, valid[current]
        if a[i] <= 0.0:
            raise SpeedNotReachable(float(v[max(i - 1, 0)]))

    shift = vehicle.powertrain.shift_time_s
    t, d, sp, gr = [0.0], [0.0], [v[0]], [gears[0]]
    for i in range(len(v) - 1):
        dvi = v[i + 1] - v[i]
        t.append(t[-1] + dvi * 0.5 * (1.0 / a[i] + 1.0 / a[i + 1]))
        d.append(d[-1] + dvi * 0.5 * (v[i] / a[i] + v[i + 1] / a[i + 1]))
        sp.append(v[i + 1])
        gr.append(gears[i + 1])
        if gears[i + 1] > gears[i] and shift > 0.0:          # upshift: coast for the shift time
            t.append(t[-1] + shift)
            d.append(d[-1] + v[i + 1] * shift)
            sp.append(v[i + 1])
            gr.append(gears[i + 1])
    return RunResult(np.array(t), np.array(sp), np.array(d), np.array(gr))


def brake(vehicle, env, v_start, v_end=0.0, dv=0.2, cond=FLAT) -> RunResult:
    """Full braking from v_start down to v_end [m/s] (no gear changes, engine disconnected)."""
    n = max(int(np.ceil((v_start - v_end) / dv)), 1)
    v = np.linspace(v_start, v_end, n + 1)                    # descending
    a, _ = braking_deceleration(vehicle, env, v, cond)
    if np.any(a <= 0.0):
        raise SpeedNotReachable(0.0)
    step = v[:-1] - v[1:]
    dt = step * 0.5 * (1.0 / a[:-1] + 1.0 / a[1:])
    ds = step * 0.5 * (v[:-1] / a[:-1] + v[1:] / a[1:])
    return RunResult(np.concatenate([[0.0], np.cumsum(dt)]), v, np.concatenate([[0.0], np.cumsum(ds)]),
                     np.full(len(v), -1, dtype=int))


@dataclass
class TopSpeed:
    speed: float            # m/s
    limited_by: str         # "power" (drag equals the drive force), "rev_limit" or "none" (not reached below 120 m/s)


def top_speed(vehicle, env, cond=FLAT, a_min=0.05, v_max=120.0, dv=0.05) -> TopSpeed:
    """Highest speed at which the car still accelerates more than a_min [m/s2].

    At the true top speed the acceleration is zero and the time to reach it is infinite, hence the threshold.
    """
    v = np.arange(1.0, v_max, dv)
    n_gears = len(vehicle.powertrain.gear_ratios)
    a_all = np.array([acceleration(vehicle, env, v, g, cond)[0] for g in range(n_gears)])
    best = np.where(np.isnan(a_all), -np.inf, a_all).max(axis=0)
    bad = np.where(best <= a_min)[0]
    if len(bad) == 0:
        return TopSpeed(float(v[-1]), "none")
    i = int(bad[0])
    return TopSpeed(float(v[max(i - 1, 0)]), "rev_limit" if best[i] == -np.inf else "power")


def speed_distance_profile(vehicle, env, v_peak=None, cond=FLAT):
    """Accelerate from standstill to v_peak and brake straight away to a stop (for example 0-200-0).

    Returns (distance [m], speed [m/s], time [s]). If v_peak is None or above what the car can reach, the car
    accelerates to 98 % of its top speed.
    """
    reach = top_speed(vehicle, env, cond).speed
    peak = 0.98 * reach if v_peak is None else min(v_peak, 0.98 * reach)
    up = accelerate(vehicle, env, 0.0, peak, cond=cond)
    down = brake(vehicle, env, peak, 0.0, cond=cond)
    distance = np.concatenate([up.distance, up.distance[-1] + down.distance[1:]])
    speed = np.concatenate([up.speed, down.speed[1:]])
    time = np.concatenate([up.time, up.time[-1] + down.time[1:]])
    return distance, speed, time


def performance_summary(vehicle, env, cond=FLAT) -> dict:
    """The key figures of the vehicle page. Entries that cannot be computed are None."""
    out: dict = {}
    ts = top_speed(vehicle, env, cond)
    out["top_speed_ms"], out["top_speed_limited_by"] = ts.speed, ts.limited_by
    for label, target in (("0_100_kmh", 100 / 3.6), ("0_200_kmh", 200 / 3.6), ("0_60_mph", 60 * MPH)):
        try:
            r = accelerate(vehicle, env, 0.0, target, cond=cond)
            out[label] = {"time_s": r.total_time, "distance_m": r.total_distance}
        except SpeedNotReachable:
            out[label] = None
    for label, v0 in (("100_0_kmh", 100 / 3.6), ("200_0_kmh", 200 / 3.6), ("70_0_mph", 70 * MPH)):
        try:
            r = brake(vehicle, env, v0, 0.0, cond=cond)
            out[label] = {"time_s": r.total_time, "distance_m": r.total_distance}
        except SpeedNotReachable:
            out[label] = None
    # runs to the top speed (98 % of it, as in the 0-top-0 profile: the true top speed takes forever to reach)
    peak = 0.98 * ts.speed
    out["run_top_speed_ms"] = peak
    for label, fn in (("0_top", lambda: accelerate(vehicle, env, 0.0, peak, cond=cond)),
                      ("top_0", lambda: brake(vehicle, env, peak, 0.0, cond=cond))):
        try:
            r = fn()
            out[label] = {"time_s": r.total_time, "distance_m": r.total_distance}
        except SpeedNotReachable:
            out[label] = None
    for label, v_peak in (("0_100_0", 100 / 3.6), ("0_200_0", 200 / 3.6), ("0_top_0", peak)):
        if v_peak > peak + 1e-9:
            out[label] = None
            continue
        try:
            d, _, t = speed_distance_profile(vehicle, env, v_peak, cond)
            out[label] = {"time_s": float(t[-1]), "distance_m": float(d[-1])}
        except SpeedNotReachable:
            out[label] = None
    curve = vehicle.powertrain.torque_curve
    out["peak_torque_nm"], out["peak_torque_rpm"] = curve.peak_torque()
    p, rpm = curve.peak_power()
    out["peak_power_kw"], out["peak_power_rpm"] = p / 1000.0, rpm
    return out
