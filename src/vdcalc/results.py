"""Page-level results: everything each page of the program needs, as plain data.

The functions return nested dictionaries that `to_jsonable` can turn into JSON (NaN and infinity become null), so the
interface only has to draw them. Speeds are in m/s, as everywhere in the core; the display units (km/h, min:s, g)
are chosen by the interface, but the lap summary also carries ready-to-show values.
"""
import math

import numpy as np

from . import curves
from .circuit import Track
from .lap import LapResult, solve_lap
from .lateral import gg_envelope
from .limits import accelerations
from .loads import FLAT
from .powertrain import rpm_from_speed
from .straightline import performance_summary, speed_distance_profile, top_speed
from .units import format_laptime

G0 = 9.81   # the unit "g" used for display


def to_jsonable(obj):
    """Converts NumPy arrays and numbers to lists and floats; NaN and infinity become None."""
    if isinstance(obj, dict):
        return {str(k): to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return to_jsonable(obj.tolist())
    if isinstance(obj, (np.floating, float)):
        return None if not math.isfinite(float(obj)) else float(obj)
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.bool_,)):
        return bool(obj)
    return obj


def vehicle_page(vehicle, env) -> dict:
    """Everything on the vehicle page: drawing, engine, gears, forces, tyres and key figures."""
    return {
        "drawing": curves.drawing(vehicle),
        "engine": curves.engine_curves(vehicle),
        "gears": curves.gear_curves(vehicle, env),
        "forces": curves.force_curves(vehicle, env),
        "tyres": curves.tire_curves(vehicle, env),
        "ellipse_front": curves.friction_ellipse(vehicle, env, "front"),
        "ellipse_rear": curves.friction_ellipse(vehicle, env, "rear"),
        "key_figures": performance_summary(vehicle, env),
    }


def _best_gear(vehicle, env, speed_ms):
    """(best gear index, acceleration [m/s2]) at full throttle in a straight line, for an array of speeds."""
    a_all, _ = accelerations(vehicle, env, speed_ms, FLAT, False, 1.0)
    valid = np.where(np.isnan(a_all), -np.inf, a_all)
    gear = np.argmax(valid, axis=0)
    return gear, np.take_along_axis(valid, gear[None, ...], axis=0)[0]


def gg_speed_grid(vehicle, env, step_min_kmh=4.0, step_rel=0.10, step_max_kmh=25.0, click_kmh=0.3) -> list:
    """Speeds [km/h] of the g-g diagrams.

    Close together at low speed, where the diagram changes fast, and farther apart as the speed grows. Around every
    change of the best gear there is one speed just before and one just after it, so the step of the diagram at the
    gear change is visible when the slider crosses it.
    """
    reach = top_speed(vehicle, env).speed * 3.6
    grid, s = [], 10.0
    while s < reach:
        grid.append(s)
        s += min(max(step_min_kmh, step_rel * s), step_max_kmh)
    if reach - grid[-1] > 5.0:
        grid.append(float(int(reach) - 1))                       # the last diagram is close to the top speed
    fine = np.arange(1.0, reach, 0.5)
    gear, _ = _best_gear(vehicle, env, fine / 3.6)
    clicks = []
    for i in np.flatnonzero(np.diff(gear) != 0):
        lo, hi = fine[i], fine[i + 1]
        g_lo = gear[i]
        for _ in range(20):                                     # bisection on the speed where the best gear changes
            mid = 0.5 * (lo + hi)
            if _best_gear(vehicle, env, np.array([mid / 3.6]))[0][0] == g_lo:
                lo = mid
            else:
                hi = mid
        clicks += [lo - click_kmh, hi + click_kmh]
    grid = [g for g in grid if all(abs(g - c) > 1.5 for c in clicks)] + clicks
    return sorted(round(float(g), 2) for g in grid if 5.0 <= g <= reach)


def gg_set(vehicle, env, speeds_kmh=None, n=41) -> dict:
    """The g-g diagram at several speeds, so that the speed slider answers instantly. Accelerations in g.

    Each diagram also says which gear gives the best acceleration at that speed and the engine speed in it, and
    `straight` is the acceleration in a straight line against speed (the line the diagrams cross at ay = 0).
    """
    if speeds_kmh is None:
        speeds_kmh = gg_speed_grid(vehicle, env) or [20.0]
    out = []
    for s in speeds_kmh:
        ay, acc, brk = gg_envelope(vehicle, env, s / 3.6, n=n)
        gear, _ = _best_gear(vehicle, env, np.array([s / 3.6]))
        rpm = float(rpm_from_speed(vehicle, s / 3.6, int(gear[0])))
        out.append({"speed_kmh": s, "gear": int(gear[0]) + 1, "rpm": rpm, "ay_g": ay / G0, "ax_accel_g": acc / G0, "ax_brake_g": brk / G0})
    reach = top_speed(vehicle, env).speed * 3.6
    fine = np.arange(1.0, reach, 1.0)
    gear, acc = _best_gear(vehicle, env, fine / 3.6)
    return {"speeds_kmh": list(speeds_kmh), "diagrams": out,
            "straight": {"speed_kmh": fine, "ax_accel_g": acc / G0, "gear": gear + 1}}


def performance_page(vehicle, env, profile_speed_kmh=200.0) -> dict:
    """The test page: key figures, speed against distance for 0-V-0 runs, and the g-g diagrams."""
    reach = top_speed(vehicle, env).speed

    def profile(v_peak):
        d, v, t = speed_distance_profile(vehicle, env, v_peak)
        return {"distance_m": d, "speed_ms": v, "time_s": t, "peak_speed_ms": float(v.max())}

    return {"key_figures": performance_summary(vehicle, env),
            "profile_requested": profile(profile_speed_kmh / 3.6), "profile_max": profile(None),
            "top_speed_ms": reach, "gg": gg_set(vehicle, env)}


def lap_summary(lap: LapResult) -> dict:
    t = lap.track
    return {
        "lap_time_s": lap.lap_time, "lap_time_text": format_laptime(lap.lap_time), "length_m": t.length, "ds_m": t.ds,
        "speed_min_kmh": float(lap.speed.min() * 3.6), "speed_max_kmh": float(lap.speed.max() * 3.6),
        "speed_avg_kmh": float(t.length / lap.lap_time * 3.6),
        "max_lateral_g": float(np.abs(lap.lat_accel).max() / G0), "max_braking_g": float(-lap.long_accel.min() / G0),
        "max_acceleration_g": float(lap.long_accel.max() / G0),
        "height_min_m": float(t.z.min()), "height_max_m": float(t.z.max()),
    }


def circuit_page(vehicle, env, track: Track, lap: LapResult = None) -> dict:
    """The circuit page: the lap, ready to draw."""
    lap = solve_lap(vehicle, env, track) if lap is None else lap
    unlimited = lap.limit >= 0.999 * lap.speed_cap
    return {
        "summary": lap_summary(lap),
        "position_m": lap.position, "x_m": track.x, "y_m": track.y, "z_m": track.z,
        "bank_deg": np.degrees(track.bank), "curvature": track.curvature, "slope_pct": np.tan(track.slope) * 100.0,
        "speed_kmh": lap.speed * 3.6, "limit_kmh": np.where(unlimited, np.nan, lap.limit * 3.6),
        "long_accel_g": lap.long_accel / G0, "lat_accel_g": lap.lat_accel / G0,
        "gear": [None if g < 0 else int(g) + 1 for g in lap.gear], "n_gears": len(vehicle.powertrain.gear_ratios), "time_s": lap.time,
        "start_heading_rad": float(math.atan2(track.y[1] - track.y[0], track.x[1] - track.x[0])),
    }
