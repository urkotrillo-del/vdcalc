"""Fixing a reference and comparing against it.

The reference is a copy of the key figures (and of the lap, if any) at the moment the user fixes it. After changing
a datum, every figure is shown with its difference and the lap time is drawn as a delta along the circuit.
"""
import numpy as np

from .lap import LapResult
from .straightline import performance_summary


def key_figures(vehicle, env, lap: LapResult = None) -> dict:
    """The scalar results of the car (and of the lap) as a flat dictionary. Missing results are None."""
    s = performance_summary(vehicle, env)
    f = {"top_speed_kmh": s["top_speed_ms"] * 3.6, "peak_power_kw": s["peak_power_kw"], "peak_torque_nm": s["peak_torque_nm"]}
    f["run_top_speed_kmh"] = s["run_top_speed_ms"] * 3.6
    for key in ("0_60_mph", "0_100_kmh", "0_200_kmh", "0_top", "0_100_0", "0_200_0", "0_top_0"):
        f["time_" + key] = None if s[key] is None else s[key]["time_s"]
    for key in ("100_0_kmh", "200_0_kmh", "top_0", "0_100_0", "0_200_0", "0_top_0"):
        f["distance_" + key] = None if s[key] is None else s[key]["distance_m"]
    if lap is not None:
        f["lap_time_s"] = lap.lap_time
        f["lap_speed_min_kmh"] = float(lap.speed.min() * 3.6)
        f["lap_speed_max_kmh"] = float(lap.speed.max() * 3.6)
        f["lap_speed_avg_kmh"] = float(lap.track.length / lap.lap_time * 3.6)
        f["lap_max_lateral_g"] = float(np.abs(lap.lat_accel).max() / 9.81)
        f["lap_max_braking_g"] = float(-lap.long_accel.min() / 9.81)
    return f


def difference(reference: dict, current: dict) -> dict:
    """For each figure present in both: reference, current, delta and delta in per cent (None if not computable)."""
    out = {}
    for key, ref in reference.items():
        cur = current.get(key)
        if ref is None or cur is None:
            out[key] = {"reference": ref, "current": cur, "delta": None, "delta_pct": None}
        else:
            out[key] = {"reference": ref, "current": cur, "delta": cur - ref, "delta_pct": 100.0 * (cur - ref) / ref if ref != 0 else None}
    return out


def lap_time_delta(reference: LapResult, current: LapResult) -> np.ndarray:
    """Time difference [s] along the circuit: positive where the current lap has lost time against the reference.

    Both laps must be on the same circuit; if the points differ the reference time is interpolated.
    """
    if abs(reference.track.length - current.track.length) > 0.01 * reference.track.length:
        raise ValueError("The two laps are not on the same circuit.")
    ref_time = np.interp(current.position, reference.position, reference.time)
    return current.time - ref_time
