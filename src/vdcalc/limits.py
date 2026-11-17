"""Longitudinal limits of the car at a given speed: traction, braking and acceleration.

The tyre force limit depends on the axle load, which depends on the force itself through the load transfer.
With mu = mu0 - c*Fz the equation is a quadratic in the force, so the limit has a closed form (no iteration).

Axle limit, for an axle that carries a share `lam` of the force:

    lam * F' + lam * R_term = G(N),   N = A + s * k * F',   G(N) = mu0 * N - C * N^2

with F' = Fx - R, k = h / L, C = c / 2000 (per N: two tyres, c per kN), s = +1 if the load grows with the force
and -1 if it decreases.
"""
import numpy as np

from .loads import FLAT, load_terms
from .powertrain import drive_force, effective_mass


def _positive_root(lam, s, A, k, rterm, mu0, C):
    """Positive root of  C k^2 F^2 + (lam - mu0 s k + 2 C A s k) F + (rterm - mu0 A + C A^2) = 0.

    Written as -2 c0 / (b + sqrt(b^2 - 4 a c0)), which is also valid when a = 0 (constant friction).
    Where the car can never reach the limit (the load transfer outgrows the friction) the root is +inf.
    """
    a = C * k * k
    b = lam - mu0 * s * k + 2.0 * C * A * s * k
    c0 = rterm - mu0 * A + C * A * A
    disc = np.maximum(b * b - 4.0 * a * c0, 0.0)
    denom = b + np.sqrt(disc)
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(denom > 1e-12, -2.0 * c0 / denom, np.inf)


def max_tractive_force(vehicle, env, speed, cond=FLAT, terms=None):
    """Largest force the tyres can push the car with [N], at a speed. Depends on the drive layout.

    Also limited by the front axle losing contact (wheelie) when the load transfer to the rear is too large.
    """
    t = terms if terms is not None else load_terms(vehicle, env, speed, cond)
    g, tires = vehicle.general, vehicle.tires
    k = g.cg_height_m / g.wheelbase_m
    R = t.rolling
    front, rear = tires.front, tires.rear
    cf, cr = front.c_x / 2000.0, rear.c_x / 2000.0
    if g.drive == "RWD":
        f_net = _positive_root(1.0, +1.0, t.rear_base, k, R, rear.mu_x0, cr)
    elif g.drive == "FWD":
        f_net = _positive_root(1.0, -1.0, t.front_base, k, R, front.mu_x0, cf)
    else:  # AWD with a fixed torque split
        share = g.awd_front_torque_fraction
        f_front = _positive_root(share, -1.0, t.front_base, k, share * R, front.mu_x0, cf)
        f_rear = _positive_root(1.0 - share, +1.0, t.rear_base, k, (1.0 - share) * R, rear.mu_x0, cr)
        f_net = np.minimum(f_front, f_rear)
    if k > 0:
        f_net = np.minimum(f_net, t.front_base / k)   # front axle unloaded: wheelie
    return f_net + R


def max_braking_force(vehicle, env, speed, cond=FLAT, terms=None):
    """(force [N], front_limits) : the largest total braking force of the tyres, with the fixed brake bias.

    The axle that reaches its limit first sets the maximum. The rear axle can also lift (stoppie).
    front_limits is True where the front axle saturates first.
    """
    t = terms if terms is not None else load_terms(vehicle, env, speed, cond)
    g, tires = vehicle.general, vehicle.tires
    k = g.cg_height_m / g.wheelbase_m
    bias = g.brake_front_fraction
    front, rear = tires.front, tires.rear
    b_front = t.front_base + k * t.rolling
    b_rear = t.rear_base - k * t.rolling
    f_front = _positive_root(bias, +1.0, b_front, k, 0.0, front.mu_x0, front.c_x / 2000.0)
    f_rear = _positive_root(1.0 - bias, -1.0, b_rear, k, 0.0, rear.mu_x0, rear.c_x / 2000.0)
    force = np.minimum(f_front, f_rear)
    if k > 0:
        force = np.minimum(force, b_rear / k)
    return force, f_front < f_rear


def braking_deceleration(vehicle, env, speed, cond=FLAT, grip_fraction=1.0):
    """(deceleration [m/s2, positive], front_limits). The inertia of the rotating parts does not matter:
    the limit is a force on the road, and the brakes provide the extra torque to slow the wheels.

    grip_fraction: share of the longitudinal tyre limit that is still free [0, 1] when the car is also cornering
    (friction ellipse)."""
    t = load_terms(vehicle, env, speed, cond)
    force, front_limits = max_braking_force(vehicle, env, speed, cond, terms=t)
    return (grip_fraction * force + t.drag + t.rolling + t.grade) / vehicle.general.mass_kg, front_limits


def acceleration(vehicle, env, speed, gear_index, cond=FLAT, launch=False, grip_fraction=1.0):
    """(acceleration [m/s2], grip_limited) in a gear: the lower of what the engine and the tyres allow.

    Engine limit: (F_drive - D - R - grade) / M_eff.   Tyre limit: (F_tyre_max - D - R - grade) / M.
    NaN where the gear is beyond the rev limit.
    grip_fraction: share of the longitudinal tyre limit still free when the car is also cornering (friction ellipse).
    """
    t = load_terms(vehicle, env, speed, cond)
    f_drive = drive_force(vehicle, speed, gear_index, launch)
    a_engine = (f_drive - t.drag - t.rolling - t.grade) / effective_mass(vehicle, gear_index)
    f_grip = grip_fraction * max_tractive_force(vehicle, env, speed, cond, terms=t)
    a_grip = (f_grip - t.drag - t.rolling - t.grade) / vehicle.general.mass_kg
    return np.minimum(a_engine, a_grip), a_grip < a_engine


def accelerations(vehicle, env, speed, cond=FLAT, launch=False, grip_fraction=1.0):
    """Acceleration in EVERY gear at once: array of shape (gears, ...). Shares the load and grip terms (faster).

    Returns (a, grip_limited) with the same meaning as `acceleration`. NaN where a gear is beyond the rev limit.
    """
    t = load_terms(vehicle, env, speed, cond)
    f_grip = grip_fraction * max_tractive_force(vehicle, env, speed, cond, terms=t)
    resist = t.drag + t.rolling + t.grade
    a_grip = (f_grip - resist) / vehicle.general.mass_kg
    a_engine = np.array([(drive_force(vehicle, speed, g, launch) - resist) / effective_mass(vehicle, g)
                         for g in range(len(vehicle.powertrain.gear_ratios))])
    return np.minimum(a_engine, a_grip), a_grip < a_engine
