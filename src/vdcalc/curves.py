"""Data for the charts of the vehicle page. Every function returns plain NumPy arrays and numbers (SI units).

Nothing here is drawn: the interface decides how to show it. Arrays can hold NaN where something does not exist
(for example the drive force of a gear beyond the rev limit); `results.to_jsonable` turns NaN into null.
"""
import math

import numpy as np

from .limits import max_tractive_force
from .loads import FLAT, load_terms
from .powertrain import drive_force, rev_limit, rpm_from_speed, total_ratio
from .straightline import SpeedNotReachable, accelerate, top_speed


def drawing(vehicle) -> dict:
    """The numbers written on the side and front view of the car."""
    g = vehicle.general
    return {
        "wheelbase_m": g.wheelbase_m,
        "front_track_m": g.front_track_m,
        "rear_track_m": g.rear_track_m,
        "cg_height_m": g.cg_height_m,
        "wheel_radius_m": vehicle.tires.wheel_radius_m,
        "cg_from_front_axle_m": (1.0 - g.front_weight_fraction) * g.wheelbase_m,
        "cg_from_rear_axle_m": g.front_weight_fraction * g.wheelbase_m,
        "mass_kg": g.mass_kg,
        "front_weight_fraction": g.front_weight_fraction,
        "front_axle_mass_kg": g.mass_kg * g.front_weight_fraction,
        "rear_axle_mass_kg": g.mass_kg * (1.0 - g.front_weight_fraction),
        "drive": g.drive,
    }


def engine_curves(vehicle, n=200) -> dict:
    """Torque and power against engine speed, from the first point of the curve to the rev limit."""
    curve = vehicle.powertrain.torque_curve
    top = rev_limit(vehicle)
    rpm = np.linspace(curve.rpm_min, top, n)
    torque = curve.torque(rpm)
    power = curve.power_w(rpm) / 1000.0
    i_t, i_p = int(np.argmax(torque)), int(np.argmax(power))
    return {
        "rpm": rpm, "torque_nm": torque, "power_kw": power,
        "points_rpm": curve.rpm, "points_torque_nm": curve.torque_nm,
        "measured_at": curve.measured_at, "rev_limit_rpm": top,
        "peak_torque_nm": float(torque[i_t]), "peak_torque_rpm": float(rpm[i_t]),
        "peak_power_kw": float(power[i_p]), "peak_power_rpm": float(rpm[i_p]),
    }


def gear_curves(vehicle, env) -> dict:
    """Engine speed against vehicle speed in every gear, and the sawtooth that full-throttle acceleration follows."""
    pt = vehicle.powertrain
    r = vehicle.tires.wheel_radius_m
    rpm_lo, rpm_hi = pt.torque_curve.rpm_min, rev_limit(vehicle)
    gears = []
    for g in range(len(pt.gear_ratios)):
        ratio = total_ratio(vehicle, g)
        speeds = np.array([rpm_lo, rpm_hi]) * 2.0 * math.pi / 60.0 * r / ratio
        gears.append({"gear": g + 1, "total_ratio": ratio, "rpm": np.array([rpm_lo, rpm_hi]), "speed_ms": speeds})

    reach = top_speed(vehicle, env).speed
    saw_v, saw_rpm, shifts = [], [], []
    try:
        run = accelerate(vehicle, env, 0.0, 0.98 * reach)
        start = 0
        for i in range(1, len(run.gear) + 1):
            if i == len(run.gear) or run.gear[i] != run.gear[start]:
                g = int(run.gear[start])
                v0, v1 = float(run.speed[start]), float(run.speed[i - 1])
                rpm0 = max(float(rpm_from_speed(vehicle, v0, g)), rpm_lo)     # standing start: the clutch slips
                rpm1 = float(rpm_from_speed(vehicle, v1, g))
                if start == 0:                                           # from rest: clutch slip at the lowest rpm, then the gear line
                    v_lo = float(rpm_lo * 2.0 * math.pi / 60.0 * r / total_ratio(vehicle, g))
                    saw_v += [v0, v_lo, v1]
                    saw_rpm += [rpm0, rpm_lo, rpm1]
                else:
                    saw_v += [v0, v1]
                    saw_rpm += [rpm0, rpm1]
                if i < len(run.gear):
                    nxt = int(run.gear[i])
                    shifts.append({"from_gear": g + 1, "to_gear": nxt + 1, "speed_ms": v1, "rpm_before": rpm1,
                                   "rpm_after": float(rpm_from_speed(vehicle, v1, nxt))})
                start = i
    except SpeedNotReachable:
        pass
    return {"gears": gears, "rpm_min": rpm_lo, "rev_limit_rpm": rpm_hi,
            "sawtooth_speed_ms": np.array(saw_v), "sawtooth_rpm": np.array(saw_rpm), "shifts": shifts}


def force_curves(vehicle, env, n=300) -> dict:
    """Forces against speed on the flat: drive force of each gear, the tyre limit, and the resistances.

    The drive force is the one at the wheels before taking the inertia of the rotating parts into account. Each
    gear is shown only where the engine would be above the first point of the torque curve (first gear from rest).
    """
    reach = top_speed(vehicle, env).speed
    v = np.linspace(0.0, 1.05 * reach, n)
    rpm_lo = vehicle.powertrain.torque_curve.rpm_min
    per_gear = []
    for g in range(len(vehicle.powertrain.gear_ratios)):
        f = np.array(drive_force(vehicle, v, g), dtype=float)
        if g > 0:
            f[rpm_from_speed(vehicle, v, g) < rpm_lo] = np.nan
        per_gear.append(f)
    stack = np.array(per_gear)
    envelope = np.where(np.all(np.isnan(stack), axis=0), np.nan, np.nanmax(np.where(np.isnan(stack), -np.inf, stack), axis=0))
    t = load_terms(vehicle, env, v, FLAT)
    return {
        "speed_ms": v, "gear_force_n": per_gear, "engine_envelope_n": envelope,
        "tyre_limit_n": np.array(max_tractive_force(vehicle, env, v, terms=t), dtype=float),
        "drag_n": np.array(t.drag, dtype=float), "rolling_n": np.array(t.rolling, dtype=float),
        "resistance_n": np.array(t.drag + t.rolling, dtype=float), "downforce_n": np.array(t.downforce, dtype=float),
        "top_speed_ms": reach,
    }


def tire_curves(vehicle, env, n=60) -> dict:
    """Friction coefficients and maximum forces of one tyre against its vertical load, for each axle.

    The static load of one tyre of each axle is marked: it is where the car stands. The range reaches 2.5 times the
    heaviest static load, which covers load transfer and downforce.
    """
    g, t = vehicle.general, vehicle.tires
    weight = g.mass_kg * env.gravity
    static = {"front": weight * g.front_weight_fraction / 2.0, "rear": weight * (1.0 - g.front_weight_fraction) / 2.0}
    loads = np.linspace(0.0, 2.5 * max(static.values()), n)
    out = {"static_load_n": static}
    for axle, tire in (("front", t.front), ("rear", t.rear)):
        mu_x = np.maximum(tire.mu_x0 - tire.c_x * loads / 1000.0, 0.0)
        mu_y = np.maximum(tire.mu_y0 - tire.c_y * loads / 1000.0, 0.0)
        out[axle] = {"load_n": loads, "mu_x": mu_x, "mu_y": mu_y, "fx_max_n": mu_x * loads, "fy_max_n": mu_y * loads}
    out["mode"] = t.mode
    return out


def friction_ellipse(vehicle, env, axle="front", load_per_tire=None, n=73) -> dict:
    """Friction ellipse of one tyre of an axle for a vertical load [N] (default: its static load).

    Fx is longitudinal (traction and braking) and Fy lateral. The semi-axes are the peak forces at that load.
    """
    g, t = vehicle.general, vehicle.tires
    weight = g.mass_kg * env.gravity
    if load_per_tire is None:
        load_per_tire = weight * (g.front_weight_fraction if axle == "front" else 1.0 - g.front_weight_fraction) / 2.0
    tire = t.front if axle == "front" else t.rear
    a = max(tire.mu_x0 - tire.c_x * load_per_tire / 1000.0, 0.0) * load_per_tire
    b = max(tire.mu_y0 - tire.c_y * load_per_tire / 1000.0, 0.0) * load_per_tire
    theta = np.linspace(0.0, 2.0 * math.pi, n)
    return {"axle": axle, "load_n": float(load_per_tire), "fx_max_n": float(a), "fy_max_n": float(b),
            "fx": a * np.cos(theta), "fy": b * np.sin(theta)}
