"""Lap time: the speed profile along a closed circuit (quasi-steady-state, point mass, perfect driver).

Steps:
  1. Speed limit at every point from cornering (grip, banking, downforce, crests).
  2. Backward pass: the fastest the car can be at a point and still brake in time for the next one. The cap of
     the profile is the minimum of the two.
  3. Forward pass: the fastest way of driving to the cap, accelerating at the limit of the car (engine with gears,
     rev limit and shift time; tyres with traction limit and friction ellipse). It works one ACCELERATION SEGMENT at a
     time: where the cap rises, the car starts the segment in the gear that accelerates best or in one of the next
     ones and upshifts (constant speed during the shift), either as soon as the next gear accelerates more (where the
     acceleration curves of two gears cross, as in the straight-line run) or at the rev limit. The start gear and the
     shift rule that give the least time at the end of the segment are kept. Where the car is held by the cap (braking, or holding the cornering speed) it is
     off the throttle and no gear is shown.
The profile starts at the slowest point of the circuit, where the speed is the limit by definition, so the lap
closes on itself.

Every step between two points uses the average of the acceleration at its two ends (second order: the error of the
lap time falls with the square of the spacing, not with the spacing itself), and the curvature of a step is the average
of its two ends.

Where the lateral limit is nearly used, the friction ellipse leaves little longitudinal force, so the car sheds
a little speed to pay for drag and rolling resistance: the same trade-off a real car faces at the limit.
"""
import math
from dataclasses import dataclass

import numpy as np

from .circuit import Track
from .lateral import corner_speed, lateral_usage, remaining_fraction
from .limits import acceleration, accelerations, braking_deceleration
from .loads import Conditions, load_terms
from .powertrain import rev_limit, total_ratio
from .straightline import GEAR_HYSTERESIS, top_speed

SPEED_CAP_FACTOR = 1.25     # the speed limit on straights is this multiple of the top speed on the flat


@dataclass
class LapResult:
    position: np.ndarray        # distance from the finish line [m]
    speed: np.ndarray           # [m/s]
    limit: np.ndarray           # cornering speed limit [m/s]
    time: np.ndarray            # time elapsed since the finish line [s]
    lap_time: float             # [s]
    long_accel: np.ndarray      # acceleration along the road [m/s2]
    lat_accel: np.ndarray       # lateral acceleration, signed (positive = turning left) [m/s2]
    gear: np.ndarray            # gear index (0 = first) at every point; -1 where the car is off the throttle (no gear)
    track: Track
    speed_cap: float = float("inf")   # where the limit equals this, there is no cornering limit (straights)


def _intervals(track: Track, order: np.ndarray):
    """Conditions of each interval between consecutive nodes (in the given order): the average of the two ends.

    (Taking the tighter of the two ends instead is a first-order error: it makes every corner slightly slower, by an
    amount that grows with the spacing.)
    """
    a, b = order[:-1], order[1:]
    return (0.5 * (track.curvature[a] + track.curvature[b]), 0.5 * (track.effective_bank[a] + track.effective_bank[b]),
            0.5 * (track.slope[a] + track.slope[b]), 0.5 * (track.vertical_curvature[a] + track.vertical_curvature[b]))


START_GEAR_WINDOW = 2        # an acceleration segment is tried starting in the best gear and in up to this many gears above it
ROOT_TOLERANCE = 1e-9        # [m/s] accuracy of the braking speed
RISE_TOLERANCE = 0.02        # [m/s] the cap must fall or rise by more than this to count as falling or rising (ignores wiggles)


def _root_increasing(h, lo, hi, h_lo, h_hi, guess, method="secant", tol=ROOT_TOLERANCE):
    """Root of an increasing function h in [lo, hi] (h(lo) < 0 < h(hi)).

    "bisection": halves the bracket until it is narrower than tol. Slow, and it cannot fail: the reference.
    "secant": from two points it jumps to where the straight line through them crosses zero. Fast near the root.
    Every point found also shrinks the bracket, and a step that falls outside the bracket, or a bracket that did not
    halve in the last two steps, is replaced by a bisection step, so it is never worse than about twice the bisection.
    """
    if method == "bisection":
        while hi - lo > tol:
            mid = 0.5 * (lo + hi)
            if h(mid) > 0.0:
                hi = mid
            else:
                lo = mid
        return 0.5 * (lo + hi)
    a = min(max(guess, lo), hi)
    fa = h(a)
    b, fb = lo, h_lo
    widths = []
    for it in range(80):
        if fa == 0.0:
            return a
        if fa > 0.0:
            hi = a
        else:
            lo = a
        if hi - lo < tol:
            break
        widths.append(hi - lo)
        x = a - fa * (a - b) / (fa - fb) if fa != fb else 0.5 * (lo + hi)
        if not lo < x < hi or (it >= 2 and widths[-1] > 0.5 * widths[-3]):
            x = 0.5 * (lo + hi)
        b, fb = a, fa
        a, fa = x, h(x)
        if abs(a - b) < tol:
            return a
    return 0.5 * (lo + hi)


def _backward(vehicle, env, limit, k_int, bank_int, slope_int, kv_int, ds, method="secant"):
    """Fastest speed at each point from which the car can still brake down to the speed of the next point.

    Over one step the car brakes from v to v_next with the average of the braking at both ends:
        v^2 = v_next^2 + ds * (b(v) + b(v_next)),   b = braking deceleration (it depends on the speed through the
        aerodynamic load and the share of grip used by the corner).
    This is solved for v at every point, from the end of the lap backwards. The left side minus the right side
    increases with v, so there is one root and it lies between v_next and the cornering limit of the point.
    """
    n = len(limit)
    speed = np.empty(n)
    speed[-1] = limit[-1]
    for i in range(n - 2, -1, -1):
        v_next = float(speed[i + 1])
        cond = Conditions(slope_int[i], kv_int[i])
        kappa, bank = k_int[i], bank_int[i]

        def decel(v):
            usage = min(float(lateral_usage(vehicle, env, v, kappa, bank, cond)), 1.0)
            return float(braking_deceleration(vehicle, env, v, cond, float(remaining_fraction(usage)))[0])

        b_next = decel(v_next)

        def h(v):
            return v * v - v_next * v_next - ds * (decel(v) + b_next)

        top = float(limit[i])
        h_top = h(top)
        if h_top <= 0.0:                                             # it can brake from the limit itself
            speed[i] = top
        else:
            guess = math.sqrt(v_next * v_next + 2.0 * b_next * ds)   # first-order estimate
            speed[i] = _root_increasing(h, v_next, top, -2.0 * ds * b_next, h_top, guess, method)
    return speed


class _Run:
    """State of the car while driving one acceleration segment: speed, time, gear and the shift in progress."""
    __slots__ = ("v", "t", "gear", "coast", "next_gear")

    def __init__(self, v, t, gear, coast=0.0, next_gear=None):
        self.v, self.t, self.gear, self.coast = v, t, gear, coast
        self.next_gear = gear if next_gear is None else next_gear     # gear after the shift

    def copy(self):
        return _Run(self.v, self.t, self.gear, self.coast, self.next_gear)


def _forward(vehicle, env, cap, k_int, bank_int, slope_int, kv_int, ds, start_window=START_GEAR_WINDOW,
             early_shifts=True):
    """Fastest way to drive the circuit accelerating at the limit of the car, choosing the gear in each acceleration.

    cap: the speed the car cannot exceed at each point (the cornering limit and what it can brake from).
    Returns the speed, the gear (-1 where the car is held by the cap: braking or holding the cornering speed) and the
    time of each step.

    The car follows the cap where the cap does not rise. Where it rises, an acceleration segment starts, up to the next
    minimum of the cap. For every start gear tried, the car drives the segment at the limit of the tyres and the engine:
      - upshifts when the engine reaches the rev limit (in the middle of a step if that is where it happens), or, with the
        early rule, as soon as the next gear accelerates more than the current one (the curves of the two gears cross),
      - a shift takes the shift time at constant speed (no drive force),
      - its speed never exceeds the cap (it follows it when it reaches it).
    The start gear and shift rule with the least time at the end of the segment are kept. The early rule is only tried
    again as the plain rev-limit rule when it actually shifted before the rev limit (otherwise both are the same
    drive). Downshifts are not needed: a gear that cannot be used at the start speed is not tried, and the cap only ever
    makes the car slower than its limit.
    Each step uses the average of the acceleration at its start and at its end (predict, then correct).
    """
    n = len(cap)
    pt = vehicle.powertrain
    n_gears = len(pt.gear_ratios)
    shift_time = pt.shift_time_s
    mass = vehicle.general.mass_kg
    radius = vehicle.tires.wheel_radius_m
    v_cut = np.array([rev_limit(vehicle) / 60.0 * 2.0 * math.pi * radius / total_ratio(vehicle, g) for g in range(n_gears)])
    speed = np.empty(n)
    gear = np.full(n, -1)
    step_time = np.zeros(n - 1)

    def accel(v, g, k, cond):
        usage = min(float(lateral_usage(vehicle, env, v, k_int[k], bank_int[k], cond)), 1.0)
        a = float(acceleration(vehicle, env, v, g, cond, False, float(remaining_fraction(usage)))[0])
        if math.isnan(a) and g == n_gears - 1:                        # beyond the rev limit in the last gear: no drive
            t = load_terms(vehicle, env, v, cond)
            a = -float(t.drag + t.rolling + t.grade) / mass
        return a

    def accel_pair(v, g, k, cond):
        """Acceleration in gear g and in the next gear at the same speed (one call: they share the loads)."""
        usage = min(float(lateral_usage(vehicle, env, v, k_int[k], bank_int[k], cond)), 1.0)
        a_all = np.asarray(accelerations(vehicle, env, v, cond, False, float(remaining_fraction(usage)))[0], dtype=float).reshape(n_gears)
        return float(a_all[g]), float(a_all[g + 1])

    def advance(run, k, early, fired):
        """Drive the step k -> k+1 from the state `run`; returns the new state.

        early: upshift as soon as the next gear accelerates more (see GEAR_HYSTERESIS); fired[0] is set when it did.
        """
        v, t, g, coast, g_after = run.v, run.t, run.gear, run.coast, run.next_gear
        cap_next = cap[k + 1]
        cond = Conditions(slope_int[k], kv_int[k])
        left = ds                                                   # distance still to drive in this step
        if cap_next < v - 1e-9:                                     # the cap pulls the speed down: braking, gear is free
            return _Run(cap_next, t + 2.0 * ds / (v + cap_next), g_after if coast > 0.0 else g)
        for _ in range(8):
            if coast > 0.0:                                         # shifting: constant speed
                d = min(coast, left)
                t, left, coast = t + d / v, left - d, coast - d
                if coast <= 1e-12:
                    coast, g = 0.0, g_after
                if left <= 1e-12:
                    return _Run(v, t, g, coast, g_after)
            if g < n_gears - 1 and v >= v_cut[g] * (1.0 - 1e-12):   # at the rev limit: upshift
                g_after = g + 1
                if shift_time > 0.0:
                    coast = v * shift_time
                else:
                    g = g_after
                continue
            if early and g < n_gears - 1:
                a0, a_up = accel_pair(v, g, k, cond)
            else:
                a0, a_up = accel(v, g, k, cond), math.nan
            if g < n_gears - 1 and math.isnan(a0):
                g_after = g + 1
                if shift_time > 0.0:
                    coast = v * shift_time
                else:
                    g = g_after
                continue
            if early and g < n_gears - 1:                           # the curves of this gear and the next one cross
                if not math.isnan(a_up) and a_up > a0 + GEAR_HYSTERESIS:
                    fired[0] = True
                    g_after = g + 1
                    if shift_time > 0.0:
                        coast = v * shift_time
                    else:
                        g = g_after
                    continue
            v_pred = math.sqrt(max(v * v + 2.0 * a0 * left, 0.0))
            if g < n_gears - 1 and v_pred >= v_cut[g]:              # the rev limit falls inside this step
                v_c = v_cut[g] * (1.0 - 1e-9)
                x = min((v_c * v_c - v * v) / max(a0 + accel(v_c, g, k, cond), 1e-9), left)
                t, left, v = t + 2.0 * x / (v + v_c), left - x, v_c
                g_after = g + 1
                if shift_time > 0.0:
                    coast = v * shift_time
                else:
                    g = g_after
                if left <= 1e-12:
                    return _Run(v, t, g, coast, g_after)
                continue
            a1 = accel(v_pred, g, k, cond)
            if g < n_gears - 1 and math.isnan(a1):
                a1 = a0
            v_end = math.sqrt(max(v * v + (a0 + a1) * left, 0.0))
            if v_end >= cap_next:                                   # reaches the cap: follows it
                return _Run(cap_next, t + 2.0 * left / (v + cap_next), g, coast, g_after)
            return _Run(v_end, t + 2.0 * left / (v + v_end), g, coast, g_after)
        return _Run(v, t + left / v, g, coast, g_after)             # (not reached with real data) drive the rest at constant speed

    def segment_end(i):
        """Node of the next minimum of the cap after the rise that starts at i. Wiggles smaller than RISE_TOLERANCE do not count."""
        k, top = i, cap[i]
        while k + 1 < n and cap[k + 1] > top - RISE_TOLERANCE:      # still rising (or flat)
            top = max(top, cap[k + 1])
            k += 1
        if k + 1 >= n:
            return n - 1
        low, node = cap[k + 1], k + 1
        k += 1
        while k + 1 < n and cap[k + 1] < low + RISE_TOLERANCE:      # falling (or flat) until it rises again
            if cap[k + 1] < low:
                low, node = cap[k + 1], k + 1
            k += 1
        return node if k + 1 < n else n - 1

    speed[0] = cap[0]
    carried = None                    # state of a car that is still accelerating when the segment ends (cap not reached)
    i = 0
    while i < n - 1:
        if carried is None and cap[i + 1] <= speed[i] + 1e-9:       # the cap does not rise: follow it
            speed[i + 1] = cap[i + 1]
            step_time[i] = 2.0 * ds / (speed[i] + speed[i + 1])
            i += 1
            continue
        end = segment_end(i)
        if carried is not None:
            starts = [carried]
        else:
            v0 = float(speed[i])
            cond = Conditions(slope_int[i], kv_int[i])
            usage = min(float(lateral_usage(vehicle, env, v0, k_int[i], bank_int[i], cond)), 1.0)
            a_all = np.asarray(accelerations(vehicle, env, v0, cond, False, float(remaining_fraction(usage)))[0], dtype=float).reshape(n_gears)
            usable = np.flatnonzero(~np.isnan(a_all))
            best = int(usable[np.argmax(a_all[usable])]) if len(usable) else n_gears - 1
            tried = [g for g in range(best, min(best + start_window, n_gears - 1) + 1) if g in usable] or [n_gears - 1]
            starts = [_Run(v0, 0.0, g) for g in tried]
        results = []
        for run in starts:
            for early in ((True, False) if early_shifts else (False,)):
                fired = [False]
                car = run.copy()
                v_log, g_log, t_log = [car.v], [], [0.0]
                for k in range(i, end):
                    g_log.append(car.gear)
                    car = advance(car, k, early, fired)
                    v_log.append(car.v)
                    t_log.append(car.t)
                results.append((car, v_log, g_log, t_log))
                if early and not fired[0]:                         # it never shifted early: the rev-limit rule is the same drive
                    break
        if len(results) > 1:                                        # same end point: least time; one that is slower there pays the time to catch up
            v_best = max(r[0].v for r in results)
            a_end = max(accel(v_best, results[0][0].gear, end - 1, Conditions(slope_int[end - 1], kv_int[end - 1])), 0.5)
            cost = [r[0].t + (v_best - r[0].v) / a_end for r in results]
        else:
            cost = [0.0]
        run, v_log, g_log, t_log = results[int(np.argmin(cost))]
        for j, k in enumerate(range(i, end)):
            speed[k + 1] = v_log[j + 1]
            step_time[k] = t_log[j + 1] - t_log[j]
            gear[k] = g_log[j]
        carried = _Run(run.v, 0.0, run.gear, run.coast, run.next_gear) if run.v < cap[end] - 1e-6 else None
        i = end
    for k in range(n - 1):                                          # held by the cap and not accelerating: no gear
        if abs(speed[k] - cap[k]) < 1e-6 and cap[k + 1] <= cap[k] + 1e-9:
            gear[k] = -1
    gear[-1] = gear[-2]
    return speed, gear, step_time


def solve_lap(vehicle, env, track: Track, backward_method="secant", early_shifts=True) -> LapResult:
    """Speed profile and lap time of the vehicle on the circuit.

    backward_method: "secant" (default) or "bisection" (slower, the reference the secant is checked against).
    early_shifts: also try upshifting before the rev limit, where the next gear accelerates more (default). With False
    the car only shifts at the rev limit.
    """
    n = len(track.s)
    ds = track.ds
    cap = SPEED_CAP_FACTOR * top_speed(vehicle, env).speed
    cond_all = Conditions(track.slope, track.vertical_curvature)
    limit = np.minimum(corner_speed(vehicle, env, track.curvature, track.effective_bank, cond_all, v_max=cap), cap)

    start = int(np.argmin(limit))                                    # slowest point: the speed there is the limit
    order = (np.arange(n + 1) + start) % n                           # n + 1 nodes: the last one is the start again
    k_int, bank_int, slope_int, kv_int = _intervals(track, order)
    lim = limit[order]
    bwd = _backward(vehicle, env, lim, k_int, bank_int, slope_int, kv_int, ds, backward_method)
    cap_profile = np.minimum(lim, bwd)
    profile, gears, step_time = _forward(vehicle, env, cap_profile, k_int, bank_int, slope_int, kv_int, ds, early_shifts=early_shifts)
    profile[-1] = profile[0]

    speed = np.empty(n)
    gear = np.empty(n, dtype=int)
    speed[order[:-1]] = profile[:-1]
    gear[order[:-1]] = gears[:-1]
    dt = np.empty(n)
    dt[order[:-1]] = step_time
    time = np.concatenate([[0.0], np.cumsum(dt)[:-1]])
    long_accel = (np.roll(speed, -1) ** 2 - np.roll(speed, 1) ** 2) / (4.0 * ds)
    return LapResult(position=track.s, speed=speed, limit=limit, time=time, lap_time=float(dt.sum()),
                     long_accel=long_accel, lat_accel=speed**2 * track.curvature, gear=gear, track=track,
                     speed_cap=float(cap))
