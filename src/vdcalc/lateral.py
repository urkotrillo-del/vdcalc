"""Lateral limits: how hard the car can corner, and the friction ellipse that couples it with traction and braking.

Steady-state cornering in the road frame, on a road banked by beta (positive = the inside of the turn is lower):

    required lateral force of the tyres   Fy = m (a_c cos(beta) - g sin(beta)),  a_c = v^2 * curvature
    normal load                           N  = m (g cos(beta) + a_c sin(beta) + v^2 * kv) + downforce

The lateral load transfer is the moment Fy * h shared between the axles by the LLTD (front share): the load moved
from the inner to the outer tyre of axle i is  d_i = share_i * Fy * h / track_i.  With mu = mu0 - c * Fz (Fz of one
tyre in kN) the capacity of an axle is

    G(N/2 + d) + G(N/2 - d) = (mu0 N - c' N^2 / 2) - 2 c' d^2,   G(z) = mu0 z - c' z^2,   c' = c / 1000,

as long as the inner tyre stays loaded (d <= N/2). The limit force solves  axle_share * Fy = capacity(Fy), which is a
quadratic in Fy: closed form, no iteration. With constant friction (c = 0) the transfer has no effect, as expected.
"""
import numpy as np

from .aero import downforce
from .limits import accelerations, braking_deceleration
from .loads import FLAT


# Below this curvature [1/m] (radius over 10 km) a stretch of road is a straight: there is no lateral demand.
# Without it, the numerical noise of the curvature on a straight would impose a speed limit wherever the tyres are
# unloaded (a crest), even though a car that leaves the ground on a straight needs no lateral force.
STRAIGHT_CURVATURE = 1e-4


def _axle_lateral_force(mu0, c_per_kn, axle_load, force_share, transfer_share, track, cg_height):
    """Largest total lateral force Fy [N] for which one axle can supply force_share * Fy."""
    cp = c_per_kn / 1000.0
    n = np.maximum(axle_load, 0.0)
    cap0 = np.maximum(mu0 * n - cp * n * n / 2.0, 0.0)             # capacity with equal load on both tyres
    q = transfer_share * cg_height / track                          # load transfer per newton of lateral force
    root = 2.0 * cap0 / (force_share + np.sqrt(force_share**2 + 8.0 * cp * q * q * cap0))
    lifted = root * q > n / 2.0                                     # the inner tyre would be unloaded
    cap_lifted = np.maximum(mu0 * n - cp * n * n, 0.0)              # the outer tyre carries the whole axle load
    return np.where(lifted, cap_lifted / force_share, root)


def lateral_state(vehicle, env, speed, kappa=0.0, bank=0.0, cond=FLAT):
    """(front axle load [N], rear axle load [N], required lateral force [N], signed) at a speed in a corner.

    kappa: path curvature [1/m] (only its absolute value is used). bank: effective bank angle [rad], positive when
    it helps the turn (inside lower).
    """
    v = np.asarray(speed, dtype=float)
    k = np.abs(np.asarray(kappa, dtype=float))
    beta = np.asarray(bank, dtype=float)
    m = vehicle.general.mass_kg
    g = env.gravity * np.cos(cond.slope)
    a_c = v * v * k
    wn = m * (g * np.cos(beta) + a_c * np.sin(beta) + v * v * cond.vertical_curvature)
    down = downforce(env, vehicle.aero, v)
    wf, phi = vehicle.general.front_weight_fraction, vehicle.aero.front_fraction
    airborne = (wn + down) <= 0.0                         # the car has left the ground: no load on any axle
    n_front = np.where(airborne, 0.0, np.maximum(wn * wf + down * phi, 0.0))
    n_rear = np.where(airborne, 0.0, np.maximum(wn * (1.0 - wf) + down * (1.0 - phi), 0.0))
    required = m * (a_c * np.cos(beta) - g * np.sin(beta))
    return n_front, n_rear, required


def lateral_force_capacity(vehicle, env, speed, kappa=0.0, bank=0.0, cond=FLAT):
    """Largest total lateral force the tyres can give at this speed in this corner [N]."""
    g, tires = vehicle.general, vehicle.tires
    n_front, n_rear, _ = lateral_state(vehicle, env, speed, kappa, bank, cond)
    lltd = g.front_weight_fraction if g.lltd_front is None else g.lltd_front
    wf = g.front_weight_fraction
    f_front = _axle_lateral_force(tires.front.mu_y0, tires.front.c_y, n_front, wf, lltd, g.front_track_m, g.cg_height_m)
    f_rear = _axle_lateral_force(tires.rear.mu_y0, tires.rear.c_y, n_rear, 1.0 - wf, 1.0 - lltd, g.rear_track_m, g.cg_height_m)
    return np.minimum(f_front, f_rear)


def lateral_usage(vehicle, env, speed, kappa, bank=0.0, cond=FLAT):
    """Fraction of the lateral limit used at a speed in a corner (1 = at the limit, above 1 = beyond it)."""
    capacity = lateral_force_capacity(vehicle, env, speed, kappa, bank, cond)
    _, _, required = lateral_state(vehicle, env, speed, kappa, bank, cond)
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(capacity > 0.0, np.abs(required) / capacity, np.where(np.abs(required) > 0.0, np.inf, 0.0))


def remaining_fraction(usage):
    """Friction ellipse: share of the longitudinal limit still free when a share `usage` of the lateral one is used."""
    return np.sqrt(np.clip(1.0 - np.asarray(usage, dtype=float) ** 2, 0.0, 1.0))


def corner_speed(vehicle, env, kappa, bank=0.0, cond=FLAT, v_max=120.0, iterations=42):
    """Highest speed at which the car can take a corner [m/s]; v_max where there is no limit (straights, strong downforce).

    kappa: path curvature [1/m]; bank: effective bank angle [rad] (positive = helps the turn). Arrays are allowed.
    Found by bisection on the speed: the margin capacity - required force falls as the speed grows.
    """
    k, beta = np.broadcast_arrays(np.abs(np.asarray(kappa, dtype=float)), np.asarray(bank, dtype=float))
    k = np.where(k < STRAIGHT_CURVATURE, 0.0, k)

    def margin(v):
        _, _, required = lateral_state(vehicle, env, v, k, beta, cond)
        return lateral_force_capacity(vehicle, env, v, k, beta, cond) - np.abs(required)

    lo = np.zeros(k.shape)
    hi = np.full(k.shape, float(v_max))
    holds_at_top = margin(hi) >= 0.0
    cannot_stand = margin(lo) < 0.0                                 # the car would slide down a very steep bank at rest
    for _ in range(iterations):
        mid = 0.5 * (lo + hi)
        ok = margin(mid) >= 0.0
        lo, hi = np.where(ok, mid, lo), np.where(ok, hi, mid)
    v = np.where(holds_at_top | (k == 0.0), float(v_max), lo)
    return np.where(cannot_stand, 0.0, v)


def max_lateral_acceleration(vehicle, env, speed, cond=FLAT):
    """Highest lateral acceleration [m/s2] on a flat, unbanked corner at a speed (grows with the downforce)."""
    return lateral_force_capacity(vehicle, env, speed, 0.0, 0.0, cond) / vehicle.general.mass_kg


def gg_envelope(vehicle, env, speed, n=61, cond=FLAT):
    """The g-g diagram at one speed: the limit of the combined accelerations [m/s2].

    Returns (ay, ax_accel, ax_brake): for each lateral acceleration, the best acceleration and the strongest braking
    still possible. The longitudinal limit that comes from grip shrinks with the friction ellipse; the engine limit
    does not, so the acceleration side is flat where the engine is the limit.
    """
    ay_max = float(max_lateral_acceleration(vehicle, env, speed, cond))
    ay = np.linspace(-ay_max, ay_max, n)
    fraction = remaining_fraction(np.abs(ay) / ay_max)
    ax_accel = np.empty(n)
    ax_brake = np.empty(n)
    for i, f in enumerate(fraction):
        a_all, _ = accelerations(vehicle, env, speed, cond, grip_fraction=f)
        ax_accel[i] = np.nanmax(a_all) if not np.all(np.isnan(a_all)) else np.nan
        ax_brake[i] = float(braking_deceleration(vehicle, env, speed, cond, grip_fraction=f)[0])
    return ay, ax_accel, ax_brake
