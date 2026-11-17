"""Lap time: the speed profile along a closed circuit (quasi-steady-state, point mass, perfect driver).

Steps:
  1. Speed limit at every point from cornering (grip, banking, downforce, crests).
  2. Backward pass: the fastest the car can be at a point and still brake in time for the next one. The cap of
     the profile is the minimum of the two.
  3. Forward pass: the fastest way of driving to the cap, accelerating at the limit of the car (engine with gears,
     rev limit and shift time; tyres with traction limit and friction ellipse). The gears are chosen by dynamic
     programming over (point, gear) to give the least lap time, not the most acceleration at each instant.
     Downshifts are free, upshifts cost the shift time. Off the throttle (braking, holding the cornering speed)
     the driver downshifts on the way down and enters the corner in the gear that drives out of it.
The profile starts at the slowest point of the circuit, where the speed is the limit by definition, so the lap
closes on itself.

Where the lateral limit is nearly used, the friction ellipse leaves little longitudinal force, so the car sheds
a little speed to pay for drag and rolling resistance: the same trade-off a real car faces at the limit.
"""
from dataclasses import dataclass

import numpy as np

from .circuit import Track
from .lateral import corner_speed, lateral_usage, remaining_fraction
from .limits import accelerations, braking_deceleration
from .loads import Conditions, load_terms
from .straightline import top_speed

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
    gear: np.ndarray            # gear index (0 = first) at every point
    track: Track
    speed_cap: float = float("inf")   # where the limit equals this, there is no cornering limit (straights)


def _intervals(track: Track, order: np.ndarray):
    """Conditions of each interval between consecutive nodes (in the given order).

    Curvature and bank come from the node with the tighter curvature (the conservative one); slope and vertical
    curvature are averaged.
    """
    a, b = order[:-1], order[1:]
    kappa, bank = track.curvature, track.effective_bank
    use_b = np.abs(kappa[b]) > np.abs(kappa[a])
    return (np.where(use_b, kappa[b], kappa[a]), np.where(use_b, bank[b], bank[a]),
            0.5 * (track.slope[a] + track.slope[b]), 0.5 * (track.vertical_curvature[a] + track.vertical_curvature[b]))


class _Label:
    """One way of being at a point of the circuit: in a gear, at a speed, after a time, reached from `parent`."""
    __slots__ = ("node", "gear", "v", "t", "parent", "coast_nodes", "chosen", "index")
    pool = []                       # every label of the current search; `parent` is an index into it, not a reference
                                    # (a long chain of references overflows the stack when freed, in the browser)

    def __init__(self, node, gear, v, t, parent, coast_nodes=0, chosen=False):
        self.node, self.gear, self.v, self.t, self.coast_nodes = node, gear, v, t, coast_nodes
        self.parent = -1 if parent is None else parent.index
        self.index = len(_Label.pool)
        _Label.pool.append(self)
        self.chosen = chosen            # gear picked freely here (downshift or off the throttle): must be usable at this speed


_CHANGE_TIME = 0.02          # [s] charged to every gear change in the search: a change that gains less than this is not made
_MAX_LABELS_PER_GEAR = 3     # alternatives kept in each gear at each point (faster and slower-but-earlier ones)


def _prune(labels):
    """Keep, in each gear, the labels that are not worse in speed and in time than another one."""
    by_gear = {}
    for lab in labels:
        by_gear.setdefault(lab.gear, []).append(lab)
    out = []
    for group in by_gear.values():
        group.sort(key=lambda x: (-x.v, x.t))
        best_t = np.inf
        kept = 0
        for lab in group:
            if lab.t < best_t - 1e-12 and kept < _MAX_LABELS_PER_GEAR:
                out.append(lab)
                best_t = lab.t
                kept += 1
    return out


def _forward(vehicle, env, cap, k_int, bank_int, slope_int, kv_int, ds):
    """Fastest way to drive the circuit accelerating at the limit of the car, choosing the gear at every point.

    cap: the speed the car cannot exceed at each point (the cornering limit and what it can brake from). Returns the
    speed, the gear and whether the car is off the throttle (holding the cap) at each point.

    The gears are chosen to give the least time, not the most acceleration at each instant. It is dynamic programming
    over (point, gear): at every point the car can be in any gear, with the speed and time it would have if it had come
    there in the best way, and it may
      - keep accelerating in its gear,
      - upshift: no drive force for the shift time, so it covers that distance at constant speed,
      - downshift: free.
    Off the throttle (braking, or holding the cornering speed) the speed is the cap, whatever the gear, and the gear
    is free: the car arrives at the next acceleration in the gear that suits it best.
    """
    n = len(cap)
    pt = vehicle.powertrain
    n_gears = len(pt.gear_ratios)
    shift_time = pt.shift_time_s
    mass = vehicle.general.mass_kg
    speed = np.empty(n)
    gear = np.zeros(n, dtype=int)
    off_throttle = np.zeros(n, dtype=bool)

    _Label.pool = []
    current = [_Label(0, g, float(cap[0]), 0.0, None) for g in range(n_gears)]
    off_throttle[0] = True
    arrivals = {}                                                   # node -> labels that arrive there after a shift
    for i in range(n - 1):
        labels = current + arrivals.pop(i, [])
        # free changes at this point: downshifts (and upshifts too when they cost nothing)
        for g in range(n_gears):
            source = [lab for lab in labels if lab.gear > g or (shift_time == 0.0 and lab.gear != g)]
            labels = labels + [_Label(i, g, lab.v, lab.t + _CHANGE_TIME, lab, 0, True) for lab in source if lab.gear != g]
        labels = _prune(labels)
        v = np.array([lab.v for lab in labels])
        g_idx = np.array([lab.gear for lab in labels])
        cond = Conditions(slope_int[i], kv_int[i])
        usage = np.minimum(np.asarray(lateral_usage(vehicle, env, v, k_int[i], bank_int[i], cond), dtype=float), 1.0)
        fraction = remaining_fraction(usage)
        a_all, _ = accelerations(vehicle, env, v, cond, False, fraction)
        a = np.atleast_2d(a_all)[g_idx, np.arange(len(labels))]
        t_terms = load_terms(vehicle, env, v, cond)
        a_coast = -np.asarray(t_terms.drag + t_terms.rolling + t_terms.grade, dtype=float) / mass

        new, clamped = [], []
        for k, lab in enumerate(labels):
            g = lab.gear
            ak = a[k]
            if lab.chosen and np.isnan(ak) and g != n_gears - 1:      # a gear the engine cannot use at this speed
                continue
            if np.isnan(ak):                                         # beyond the rev limit in this gear
                if g != n_gears - 1:
                    ak = None
                else:
                    ak = float(np.atleast_1d(a_coast)[k] if np.ndim(a_coast) else a_coast)
            if ak is not None:
                v_next = float(np.sqrt(max(lab.v**2 + 2.0 * ak * ds, 0.0)))
                t_next = lab.t + 2.0 * ds / max(lab.v + v_next, 1e-3)
                if v_next > cap[i + 1]:
                    clamped.append(_Label(i + 1, g, float(cap[i + 1]), t_next, lab))
                else:
                    new.append(_Label(i + 1, g, v_next, t_next, lab))
            if g < n_gears - 1 and shift_time > 0.0 and lab.v > 1e-3:   # upshift: coast for the shift time
                m = max(int(np.ceil(lab.v * shift_time / ds - 1e-9)), 1)
                j = i + m
                if j < n - 1 and np.all(cap[i + 1:j + 1] >= lab.v):
                    arrivals.setdefault(j, []).append(_Label(j, g + 1, lab.v, lab.t + m * ds / lab.v + _CHANGE_TIME, lab, m))
        if clamped:                                                  # off the throttle: the speed is the cap in any gear
            c = min(clamped, key=lambda x: x.t)
            current = [_Label(i + 1, g, c.v, c.t + (_CHANGE_TIME if g != c.gear else 0.0), c, 0, g != c.gear) for g in range(n_gears)]
            off_throttle[i + 1] = True
        else:
            current = new
            if not current:                                          # nothing can drive: take the slowest way out
                current = [_Label(i + 1, n_gears - 1, float(cap[i + 1]), min(lab.t for lab in labels), labels[0])]

    final = min(current + arrivals.pop(n - 1, []), key=lambda x: x.t)
    chain = []
    lab = final
    while lab is not None:
        chain.append(lab)
        lab = _Label.pool[lab.parent] if lab.parent >= 0 else None
    chain.reverse()
    pool = _Label.pool
    for lab in chain:
        if lab.coast_nodes:                                          # shifting: constant speed in the old gear
            for node in range(lab.node - lab.coast_nodes, lab.node):
                speed[node], gear[node] = lab.v, pool[lab.parent].gear
        speed[lab.node], gear[lab.node] = lab.v, lab.gear
    _Label.pool = []
    return speed, gear, off_throttle


def _display_gears(vehicle, env, k_int, slope_int, kv_int, profile, gears, off_throttle):
    """Gear at every point as the driver would show it. With the throttle on it is the gear chosen. Off the throttle
    (braking, holding the cornering speed) the driver downshifts on the way down: the gear follows the best one for
    the actual speed, never below the gear in which the car leaves the corner (unless that would over-rev) and never above the previous point."""
    out = gears.copy()
    n = len(gears)
    i = 0
    while i < n - 1:
        if not off_throttle[i]:
            i += 1
            continue
        j = i
        while j + 1 < n - 1 and off_throttle[j + 1]:
            j += 1
        exit_gear = int(gears[j])                                    # the gear that drives out of this stretch
        previous = int(out[i - 1]) if i > 0 else exit_gear
        for k in range(i, j + 1):
            a_all, _ = accelerations(vehicle, env, float(profile[k]), Conditions(slope_int[k], kv_int[k]), False, 1.0)
            valid = np.where(np.isnan(a_all), -np.inf, a_all)
            best = int(np.argmax(valid)) if valid.max() > -np.inf else previous
            lowest = int(np.argmax(~np.isnan(a_all))) if not np.all(np.isnan(a_all)) else 0   # lower gears over-rev
            out[k] = max(lowest, exit_gear, min(previous, best))
            previous = int(out[k])
        out[j] = exit_gear
        i = j + 1
    out[-1] = out[-2]
    return out


def _backward(vehicle, env, limit, k_int, bank_int, slope_int, kv_int, ds):
    n = len(limit)
    speed = np.empty(n)
    speed[-1] = limit[-1]
    for i in range(n - 2, -1, -1):
        v_next = speed[i + 1]
        v = v_next
        cond = Conditions(slope_int[i], kv_int[i])
        for _ in range(2):                                           # the braking limit depends on the speed it starts at
            fraction = float(remaining_fraction(min(float(lateral_usage(vehicle, env, v, k_int[i], bank_int[i], cond)), 1.0)))
            a, _ = braking_deceleration(vehicle, env, v, cond, fraction)
            v = float(np.sqrt(max(v_next * v_next + 2.0 * float(a) * ds, 0.0)))
        speed[i] = min(v, limit[i])
    return speed


def solve_lap(vehicle, env, track: Track) -> LapResult:
    """Speed profile and lap time of the vehicle on the circuit."""
    n = len(track.s)
    ds = track.ds
    cap = SPEED_CAP_FACTOR * top_speed(vehicle, env).speed
    cond_all = Conditions(track.slope, track.vertical_curvature)
    limit = np.minimum(corner_speed(vehicle, env, track.curvature, track.effective_bank, cond_all, v_max=cap), cap)

    start = int(np.argmin(limit))                                    # slowest point: the speed there is the limit
    order = (np.arange(n + 1) + start) % n                           # n + 1 nodes: the last one is the start again
    k_int, bank_int, slope_int, kv_int = _intervals(track, order)
    lim = limit[order]
    bwd = _backward(vehicle, env, lim, k_int, bank_int, slope_int, kv_int, ds)
    cap_profile = np.minimum(lim, bwd)
    profile, gears, off_throttle = _forward(vehicle, env, cap_profile, k_int, bank_int, slope_int, kv_int, ds)
    profile = np.minimum(profile, cap_profile)
    profile[-1] = profile[0]
    gears = _display_gears(vehicle, env, k_int, slope_int, kv_int, profile, gears, off_throttle)

    speed = np.empty(n)
    gear = np.empty(n, dtype=int)
    speed[order[:-1]] = profile[:-1]
    gear[order[:-1]] = gears[:-1]
    v_next = np.roll(speed, -1)
    dt = 2.0 * ds / np.maximum(speed + v_next, 1e-3)
    time = np.concatenate([[0.0], np.cumsum(dt)[:-1]])
    long_accel = (np.roll(speed, -1) ** 2 - np.roll(speed, 1) ** 2) / (4.0 * ds)
    return LapResult(position=track.s, speed=speed, limit=limit, time=time, lap_time=float(dt.sum()),
                     long_accel=long_accel, lat_accel=speed**2 * track.curvature, gear=gear, track=track,
                     speed_cap=float(cap))
