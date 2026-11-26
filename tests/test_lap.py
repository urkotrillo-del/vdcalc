import math
from pathlib import Path

import numpy as np
import pytest
from scipy.optimize import brentq

from conftest import build
from vdcalc.circuit import build_track, load_track
from vdcalc.lap import solve_lap
from vdcalc.powertrain import rev_limit, rpm_from_speed
from vdcalc.straightline import accelerate, brake

EXAMPLE = Path(__file__).parent.parent / "data" / "circuits" / "example_circuit.csv"


def circle_points(radius, n=90, clockwise=False):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False) * (-1 if clockwise else 1)
    return radius * np.cos(a), radius * np.sin(a)


def stadium_points(length, radius, spacing=5.0):
    ns, na = int(length / spacing), int(np.pi * radius / spacing)
    pts = [(-length / 2 + i * length / ns, -radius) for i in range(ns)]
    pts += [(length / 2 + radius * math.cos(a), radius * math.sin(a)) for a in np.linspace(-np.pi / 2, np.pi / 2, na, endpoint=False)]
    pts += [(length / 2 - i * length / ns, radius) for i in range(ns)]
    pts += [(-length / 2 + radius * math.cos(a), radius * math.sin(a)) for a in np.linspace(np.pi / 2, 3 * np.pi / 2, na, endpoint=False)]
    x, y = zip(*pts)
    return np.array(x), np.array(y)


def test_flat_circle_matches_the_analytic_lap_time(car, env):
    radius = 50.0
    t = build_track(*circle_points(radius), ds=2.0)
    expected = 2 * math.pi * radius / math.sqrt(0.89 * env.gravity * radius)
    got = solve_lap(car, env, t).lap_time
    assert got == pytest.approx(expected, rel=0.004) and got >= expected          # drag costs a little speed at the limit


def test_banked_circle_matches_the_analytic_lap_time(car, env):
    radius, mu = 80.0, 0.89
    for deg in (4.0, 8.0):
        t = build_track(*circle_points(radius), bank_deg=np.full(90, deg), ds=2.0)
        tan = math.tan(math.radians(deg))
        v = math.sqrt(env.gravity * radius * (mu + tan) / (1 - mu * tan))
        assert solve_lap(car, env, t).lap_time == pytest.approx(2 * math.pi * radius / v, rel=0.006)


def test_the_bank_follows_the_direction_of_travel(car, env):
    x, y = circle_points(80.0)
    ccw = build_track(x, y, bank_deg=np.full(90, 6.0), ds=2.0)                  # tilts left, turning left: helps
    cw = build_track(x, y, bank_deg=np.full(90, 6.0), ds=2.0, reverse=True)     # same road driven the other way: still helps
    flat = build_track(x, y, ds=2.0)
    t_ccw, t_cw, t_flat = (solve_lap(car, env, t).lap_time for t in (ccw, cw, flat))
    assert t_ccw < t_flat and t_ccw == pytest.approx(t_cw, rel=0.003)
    adverse = build_track(x, y, bank_deg=np.full(90, -6.0), ds=2.0)             # tilts right while turning left: opposes
    assert solve_lap(car, env, adverse).lap_time > t_flat


def test_lap_time_does_not_depend_on_where_the_finish_line_is(car, env):
    x, y = stadium_points(400.0, 60.0)
    base = solve_lap(car, env, build_track(x, y, ds=2.0)).lap_time
    for start in (0, 40, 77):
        assert solve_lap(car, env, build_track(x, y, ds=2.0, start=start)).lap_time == pytest.approx(base, rel=0.002)


def test_lap_time_converges_with_the_step(car, env):
    x, y = stadium_points(400.0, 60.0)
    times = [solve_lap(car, env, build_track(x, y, ds=ds)).lap_time for ds in (4.0, 2.0, 1.0)]
    assert times[1] == pytest.approx(times[2], rel=0.005) and times[0] == pytest.approx(times[2], rel=0.012)


def test_stadium_agrees_with_the_straight_line_functions(car, env):
    """Two independent code paths: the lap solver, and acceleration + braking on the straight plus the corner speed."""
    length, radius = 1200.0, 60.0
    v_c = math.sqrt(0.89 * env.gravity * radius)

    def straight_distance(v_peak):
        return accelerate(car, env, v_c, v_peak, launch=False).total_distance + brake(car, env, v_peak, v_c).total_distance

    v_peak = brentq(lambda v: straight_distance(v) - length, v_c + 1, 60.0)
    t_straight = accelerate(car, env, v_c, v_peak, launch=False).total_time + brake(car, env, v_peak, v_c).total_time
    expected = 2 * t_straight + 2 * math.pi * radius / v_c
    got = solve_lap(car, env, build_track(*stadium_points(length, radius), ds=2.0)).lap_time
    assert got == pytest.approx(expected, rel=0.02)


def test_profile_respects_the_limits_and_the_slowest_point_is_the_limit(car, env):
    t = load_track(EXAMPLE, ds=2.0)
    r = solve_lap(car, env, t)
    assert np.all(r.speed <= r.limit + 1e-6) and np.all(r.speed > 0)
    i = int(np.argmin(r.limit))
    assert r.speed[i] == pytest.approx(r.limit[i], rel=1e-6)
    assert np.all(np.diff(r.time) > 0) and r.time[-1] < r.lap_time
    assert np.all(np.abs(r.lat_accel) < 1.25 * env.gravity)                    # about 1.1 g at most on the banked corner
    assert np.max(r.long_accel) < 0.65 * env.gravity and np.min(r.long_accel) > -1.3 * env.gravity


def test_example_circuit_lap_is_plausible_and_converged(car, env):
    t2 = solve_lap(car, env, load_track(EXAMPLE, ds=2.0))
    t1 = solve_lap(car, env, load_track(EXAMPLE, ds=1.0))
    assert 120.0 < t2.lap_time < 170.0 and t2.lap_time == pytest.approx(t1.lap_time, rel=0.003)
    assert 40 / 3.6 < t2.speed.min() < 70 / 3.6                                # the hairpin
    assert 180 / 3.6 < t2.speed.max() < 230 / 3.6


def test_more_power_grip_and_downforce_make_the_lap_faster(car, env):
    t = build_track(*stadium_points(500.0, 60.0), ds=2.0)
    base = solve_lap(car, env, t).lap_time
    strong = build({"powertrain.torque_curve": {"rpm": list(car.powertrain.torque_curve.rpm),
                                                "torque_nm": [1.3 * x for x in car.powertrain.torque_curve.torque_nm]}})
    grippy = build({"tires.front": {"mu_x": 1.2, "mu_y": 1.1}, "tires.rear": {"mu_x": 1.2, "mu_y": 1.1}})
    wing = build({"aero.cl": -2.0})
    for better in (strong, grippy, wing):
        assert solve_lap(better, env, t).lap_time < base


def test_more_mass_and_more_drag_make_the_lap_slower(car, env):
    t = build_track(*stadium_points(500.0, 60.0), ds=2.0)
    base = solve_lap(car, env, t).lap_time
    assert solve_lap(build({"general.mass_kg": 1250.0}), env, t).lap_time > base
    assert solve_lap(build({"aero.cd": 0.7}), env, t).lap_time > base


def test_hills_on_a_circle_slow_the_lap_down(car, env):
    x, y = circle_points(100.0, n=120)
    a = np.linspace(0, 2 * np.pi, 120, endpoint=False)
    flat = solve_lap(car, env, build_track(x, y, ds=2.0)).lap_time
    hilly = solve_lap(car, env, build_track(x, y, z=6.0 * np.cos(2 * a), ds=2.0)).lap_time
    assert hilly > flat


def test_driving_direction_changes_the_lap_of_the_example_circuit(car, env):
    fwd = solve_lap(car, env, load_track(EXAMPLE, ds=2.0)).lap_time
    back = solve_lap(car, env, load_track(EXAMPLE, ds=2.0, reverse=True)).lap_time
    assert back > 100.0 and abs(back - fwd) > 0.05


@pytest.mark.parametrize("changes", [
    {"general.drive": "FWD"},
    {"general.drive": "AWD", "general.awd_front_torque_fraction": 0.4},
    {"tires.mode": "load_dependent", "tires.front": {"mu_x0": 1.5, "c_x": 0.12, "mu_y0": 1.3, "c_y": 0.1},
     "tires.rear": {"mu_x0": 1.6, "c_x": 0.10, "mu_y0": 1.35, "c_y": 0.1}, "general.lltd_front": 0.6},
    {"aero.cl": -1.5, "aero.front_fraction": 0.4},
])
def test_other_layouts_and_tyre_models_give_a_sensible_lap(env, changes):
    r = solve_lap(build(changes), env, load_track(EXAMPLE, ds=2.0))
    assert 100.0 < r.lap_time < 220.0 and np.all(r.speed <= r.limit + 1e-6) and np.all(np.isfinite(r.speed))


# ---------------------------------------------------------------------------------------------
# gear selection
# ---------------------------------------------------------------------------------------------


def test_there_is_no_gear_where_the_car_is_braking_and_there_is_one_where_it_accelerates(car, env):
    lap = solve_lap(car, env, load_track(EXAMPLE, ds=2.0))
    g = lap.gear.astype(int)
    braking = lap.long_accel < -0.5 * 9.81                             # (the acceleration is a central difference: the first braking node still drives)
    accelerating = lap.long_accel > 0.05 * 9.81
    assert braking.sum() > 100 and accelerating.sum() > 100
    assert np.all(g[braking] == -1) and np.all(g[accelerating] >= 0)
    assert g.max() >= 3                                                        # it climbs through the gears on the straights


def test_under_power_the_car_only_upshifts(car, env):
    lap = solve_lap(car, env, load_track(EXAMPLE, ds=2.0))
    g, v = lap.gear.astype(int), lap.speed
    pairs = (g[:-1] >= 0) & (g[1:] >= 0) & (v[1:] > v[:-1] + 1e-6)
    assert pairs.sum() > 300 and np.all(np.diff(g)[pairs] >= 0)


def test_a_corner_that_opens_slowly_has_a_gear_all_the_way_up(car, env):
    """On an ellipse the speed limit rises slowly along the sides: the car follows it, so it has to be in a gear."""
    a = np.linspace(0, 2 * np.pi, 160, endpoint=False)
    lap = solve_lap(car, env, build_track(400.0 * np.cos(a), 90.0 * np.sin(a), ds=2.0))
    g, v = lap.gear.astype(int), lap.speed
    rising = np.zeros(len(v), dtype=bool)
    rising[:-1] = v[1:] > v[:-1] + 1e-6
    assert rising.sum() > 100 and np.all(g[rising] >= 0)
    assert np.all(v <= lap.limit + 1e-6)
    shifts = np.flatnonzero(rising[:-1] & rising[1:] & (np.diff(g) > 0))
    assert len(shifts) >= 1                                                     # it also upshifts on the way up


def test_the_secant_gives_the_same_lap_as_the_bisection(car, env):
    """The braking speed is found with the secant; the bisection (slow, cannot fail) is the control."""
    for track in (load_track(EXAMPLE, ds=2.0), build_track(*stadium_points(400.0, 60.0), ds=2.0)):
        fast = solve_lap(car, env, track)
        slow = solve_lap(car, env, track, backward_method="bisection")
        assert np.max(np.abs(fast.speed - slow.speed)) < 1e-6
        assert fast.lap_time == pytest.approx(slow.lap_time, abs=1e-6)


def _car_with_falling_torque(car, falloff):
    """The example car whose torque falls above 4500 rpm (to (1 - 0.55 * falloff) of its value at the last point)."""
    rpm = np.array(car.powertrain.torque_curve.rpm)
    torque = np.array(car.powertrain.torque_curve.torque_nm)
    factor = np.where(rpm > 4500.0, 1.0 - 0.55 * falloff * (rpm - 4500.0) / (rpm.max() - 4500.0), 1.0)
    return build({"powertrain.torque_curve": {"rpm": rpm.tolist(), "torque_nm": (torque * factor).tolist()}})


@pytest.mark.parametrize("falloff", [0.0, 0.6, 1.0])
def test_shifting_early_never_makes_the_lap_slower(car, env, falloff):
    t = build_track(*stadium_points(1500.0, 60.0), ds=2.0)
    c = _car_with_falling_torque(car, falloff)
    early = solve_lap(c, env, t).lap_time
    at_the_limit = solve_lap(c, env, t, early_shifts=False).lap_time
    assert early <= at_the_limit + 1e-6


def test_a_car_whose_torque_falls_shifts_before_the_rev_limit_and_laps_faster(car, env):
    t = build_track(*stadium_points(1500.0, 60.0), ds=2.0)
    c = _car_with_falling_torque(car, 1.0)
    early = solve_lap(c, env, t)
    at_the_limit = solve_lap(c, env, t, early_shifts=False)
    assert early.lap_time < at_the_limit.lap_time - 0.05
    ups = [k for k in range(len(early.gear) - 1) if early.gear[k] >= 0 and early.gear[k + 1] > early.gear[k]]
    assert ups
    rpm = [float(rpm_from_speed(c, early.speed[k], int(early.gear[k]))) for k in ups]
    assert min(rpm) < 0.93 * rev_limit(c)                                   # some upshift happens well before the rev limit


def test_when_the_best_gear_only_changes_at_the_rev_limit_both_rules_agree(car, env):
    t = build_track(*stadium_points(1500.0, 60.0), ds=2.0)
    a = solve_lap(car, env, t)
    b = solve_lap(car, env, t, early_shifts=False)
    assert a.lap_time == pytest.approx(b.lap_time, abs=1e-9)
    assert np.array_equal(a.gear, b.gear)


def test_the_lap_time_error_falls_fast_with_the_step(car, env):
    """Second order: with the same smoothing of the curvature the lap time barely moves between 2 m and 0.4 m."""
    t2 = solve_lap(car, env, load_track(EXAMPLE, ds=2.0, smooth_m=10.0)).lap_time
    t04 = solve_lap(car, env, load_track(EXAMPLE, ds=0.4, smooth_m=10.0)).lap_time
    assert abs(t2 - t04) < 0.04


def test_a_costlier_shift_never_makes_the_lap_faster_and_gears_are_usable(env):
    track = load_track(EXAMPLE, ds=2.0)
    laps = [solve_lap(build({"powertrain.shift_time_s": t}), env, track) for t in (0.0, 0.35, 1.0)]
    assert laps[0].lap_time <= laps[1].lap_time + 1e-6 <= laps[2].lap_time + 2e-6
    changes = [int(np.sum(np.diff(lap.gear.astype(int)) != 0)) for lap in laps]
    assert all(c < 60 for c in changes)                                  # no flip-flopping between gears


def test_the_chosen_gear_can_be_used_at_the_speed_of_the_car(car, env):
    from vdcalc.limits import accelerations
    from vdcalc.loads import Conditions
    lap = solve_lap(car, env, load_track(EXAMPLE, ds=2.0))
    g = lap.gear.astype(int)
    for i in range(0, len(g), 7):
        if abs(lap.long_accel[i]) < 0.05:                                # holding speed, or coasting through a shift
            continue
        a, _ = accelerations(car, env, float(lap.speed[i]), Conditions(0.0, 0.0), False, 1.0)
        assert not np.isnan(np.ravel(a)[g[i]]) or g[i] == len(np.ravel(a)) - 1


def test_gear_choice_has_no_effect_on_a_single_speed_car(env):
    one = build({"powertrain.gear_ratios": [3.0]})
    lap = solve_lap(one, env, load_track(EXAMPLE, ds=2.0))
    assert np.all(lap.gear[lap.gear >= 0] == 0) and np.any(lap.gear == 0)
