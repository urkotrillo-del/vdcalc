import math

import numpy as np
import pytest
from scipy.optimize import brentq

from conftest import build, without_drag
from vdcalc.limits import acceleration, braking_deceleration, max_braking_force, max_tractive_force
from vdcalc.loads import FLAT, Conditions, axle_loads, load_terms
from vdcalc.tires import axle_force_max

LOADED = {"tires.mode": "load_dependent",
          "tires.front": {"mu_x0": 1.5, "c_x": 0.12, "mu_y0": 1.3, "c_y": 0.1},
          "tires.rear": {"mu_x0": 1.6, "c_x": 0.10, "mu_y0": 1.35, "c_y": 0.1}}


def brute_traction(car, env, v):
    """Independent solution: root finding on the physical equations (loads from axle_loads, force from the tyre)."""
    g, tires = car.general, car.tires
    R = float(load_terms(car, env, v).rolling)

    def residual(fx, axle, share):
        front, rear = axle_loads(car, env, v, fx)
        tire, load = (tires.front, front) if axle == "front" else (tires.rear, rear)
        return share * fx - float(axle_force_max(tire.mu_x0, tire.c_x, load))

    if g.drive == "RWD":
        return brentq(residual, R, 1e5, args=("rear", 1.0))
    if g.drive == "FWD":
        return brentq(residual, R, 1e5, args=("front", 1.0))
    s = g.awd_front_torque_fraction
    return min(brentq(residual, R, 1e5, args=("front", s)), brentq(residual, R, 1e5, args=("rear", 1.0 - s)))


def test_rwd_constant_friction_matches_the_closed_form(env):
    car = without_drag(build({}))
    mu, g_, h, L = 1.02, 9.81, 0.46, 2.309
    k, W, R = h / L, 1138.0 * 9.81, 0.012 * 1138.0 * 9.81
    expected = mu * (W * (1 - 0.53) - k * R) / (1 - mu * k)
    assert float(max_tractive_force(car, env, 0.0)) == pytest.approx(expected, rel=1e-9)


def test_session_1_example_rwd_limit():
    # M = 1100 kg, L = 2.45 m, 50/50, h = 0.5 m, mu = 1, no resistances: F = 6778 N (a = 0.628 g)
    from vdcalc.environment import Environment
    car = without_drag(build({"general.mass_kg": 1100.0, "general.wheelbase_m": 2.45, "general.front_weight_fraction": 0.5,
                              "general.cg_height_m": 0.5, "tires.rolling_coefficient": 0.0,
                              "tires.front": {"mu_x": 1.0, "mu_y": 1.0}, "tires.rear": {"mu_x": 1.0, "mu_y": 1.0}}))
    assert float(max_tractive_force(car, Environment(), 0.0)) == pytest.approx(6778.0, abs=5.0)


@pytest.mark.parametrize("drive", ["RWD", "FWD", "AWD"])
@pytest.mark.parametrize("speed", [0.0, 15.0, 40.0, 70.0])
def test_load_dependent_closed_form_matches_brute_force(env, drive, speed):
    car = build({**LOADED, "general.drive": drive, "general.awd_front_torque_fraction": 0.45, "aero.cl": -1.2})
    exact = brute_traction(car, env, speed)
    assert float(max_tractive_force(car, env, speed)) == pytest.approx(exact, rel=1e-8)


def test_wheelie_limit_caps_the_traction_force(env):
    car = build({"general.cg_height_m": 0.9, "general.wheelbase_m": 1.8, "tires.rolling_coefficient": 0.0,
                 "tires.front": {"mu_x": 2.5, "mu_y": 2.5}, "tires.rear": {"mu_x": 2.5, "mu_y": 2.5}})
    fx = float(max_tractive_force(car, env, 0.0))
    front, rear = axle_loads(car, env, 0.0, fx)
    assert float(front) == pytest.approx(0.0, abs=1e-6)               # front axle just unloaded
    assert float(rear) == pytest.approx(car.general.mass_kg * env.gravity, rel=1e-9)


def test_load_transfer_direction_and_conservation(car, env):
    for fx in (-4000.0, 0.0, 4000.0):
        front, rear = axle_loads(car, env, 20.0, fx)
        t = load_terms(car, env, 20.0)
        assert float(front + rear) == pytest.approx(float(t.total_normal))
    f_acc, r_acc = axle_loads(car, env, 20.0, 4000.0)
    f_brk, r_brk = axle_loads(car, env, 20.0, -4000.0)
    assert r_acc > r_brk and f_brk > f_acc


def test_ideal_brake_bias_gives_mu_times_g(env):
    # with both axles at the limit, a = mu * g (no drag, no rolling resistance, no aero, flat)
    mu, wf, h, L = 1.0, 0.5, 0.5, 2.5
    k = h / L
    car = without_drag(build({"general.front_weight_fraction": wf, "general.cg_height_m": h, "general.wheelbase_m": L,
                              "general.brake_front_fraction": wf + mu * k, "tires.rolling_coefficient": 0.0,
                              "tires.front": {"mu_x": mu, "mu_y": mu}, "tires.rear": {"mu_x": mu, "mu_y": mu}}))
    a, _ = braking_deceleration(car, env, 20.0)
    assert float(a) == pytest.approx(mu * env.gravity, rel=1e-9)
    worse = without_drag(build({"general.front_weight_fraction": wf, "general.cg_height_m": h, "general.wheelbase_m": L,
                                "general.brake_front_fraction": 0.6, "tires.rolling_coefficient": 0.0,
                                "tires.front": {"mu_x": mu, "mu_y": mu}, "tires.rear": {"mu_x": mu, "mu_y": mu}}))
    assert float(braking_deceleration(worse, env, 20.0)[0]) < mu * env.gravity


def test_which_axle_locks_first(car, env):
    _, front_first = max_braking_force(car, env, 20.0)
    assert bool(front_first)                                           # 75 % front bias on this car
    rear_biased = build({"general.brake_front_fraction": 0.5})
    assert not bool(max_braking_force(rear_biased, env, 20.0)[1])


def test_braking_with_load_dependent_tyres_matches_brute_force(env):
    car = build({**LOADED})
    v = 30.0
    bias = car.general.brake_front_fraction
    R = float(load_terms(car, env, v).rolling)

    def resid_front(fb):
        front, _ = axle_loads(car, env, v, -fb - 0.0)       # tyre force Fx = -Fb
        return bias * fb - float(axle_force_max(car.tires.front.mu_x0, car.tires.front.c_x, front))

    def resid_rear(fb):
        _, rear = axle_loads(car, env, v, -fb)
        return (1 - bias) * fb - float(axle_force_max(car.tires.rear.mu_x0, car.tires.rear.c_x, rear))

    exact = min(brentq(resid_front, 100, 1e5), brentq(resid_rear, 100, 1e5))
    assert float(max_braking_force(car, env, v)[0]) == pytest.approx(exact, rel=1e-8)


def test_downforce_increases_braking_and_traction_at_speed(env):
    flat = build({"aero.cl": 0.0})
    wing = build({"aero.cl": -2.0})
    assert float(braking_deceleration(wing, env, 60.0)[0]) > float(braking_deceleration(flat, env, 60.0)[0])
    assert float(max_tractive_force(wing, env, 60.0)) > float(max_tractive_force(flat, env, 60.0))
    assert float(braking_deceleration(wing, env, 0.0)[0]) == pytest.approx(float(braking_deceleration(flat, env, 0.0)[0]))


def test_slope_changes_braking_and_acceleration(car, env):
    up, down = Conditions(slope=math.radians(5)), Conditions(slope=math.radians(-5))
    flat_b = float(braking_deceleration(car, env, 20.0)[0])
    assert float(braking_deceleration(car, env, 20.0, up)[0]) > flat_b > float(braking_deceleration(car, env, 20.0, down)[0])
    assert float(braking_deceleration(car, env, 20.0, up)[0]) - flat_b == pytest.approx(env.gravity * math.sin(math.radians(5)), rel=0.1)
    a_flat = float(acceleration(car, env, 30.0, 2)[0])
    assert float(acceleration(car, env, 30.0, 2, up)[0]) < a_flat < float(acceleration(car, env, 30.0, 2, down)[0])


def test_crest_unloads_the_tyres_and_a_compression_loads_them(car, env):
    crest, dip = Conditions(vertical_curvature=-0.004), Conditions(vertical_curvature=+0.004)
    flat_n = float(load_terms(car, env, 40.0).total_normal)
    assert float(load_terms(car, env, 40.0, crest).total_normal) < flat_n < float(load_terms(car, env, 40.0, dip).total_normal)
    assert float(braking_deceleration(car, env, 40.0, crest)[0]) < float(braking_deceleration(car, env, 40.0)[0])


def test_the_car_leaves_the_ground_over_a_sharp_crest(car, env):
    crest = Conditions(vertical_curvature=-0.01)                       # v^2 * 0.01 > g above ~113 km/h
    assert float(load_terms(car, env, 40.0, crest).total_normal) == 0.0


def test_acceleration_takes_the_lower_of_engine_and_grip(car, env):
    # standing start (launch): the engine can give its peak torque, so first gear is limited by the tyres
    a, grip = acceleration(car, env, np.array([5.0]), 0, launch=True)
    assert bool(grip[0])
    # without the launch hypothesis the engine is the limit at 18 km/h (torque held below the curve start)
    assert not bool(acceleration(car, env, np.array([5.0]), 0)[1][0])
    # a much stronger engine cannot beat the grip
    strong = build({"powertrain.torque_curve": {"rpm": list(car.powertrain.torque_curve.rpm),
                                                "torque_nm": [3 * t for t in car.powertrain.torque_curve.torque_nm]}})
    assert float(acceleration(strong, env, 5.0, 0, launch=True)[0]) == pytest.approx(float(a[0]), rel=1e-9)


def test_gear_beyond_the_rev_limit_gives_nan(car, env):
    assert np.isnan(acceleration(car, env, 80.0, 0)[0])


def test_awd_shares_the_force_between_the_axles(env):
    rwd = build({"general.drive": "RWD"})
    awd = build({"general.drive": "AWD", "general.awd_front_torque_fraction": 0.4})
    assert float(max_tractive_force(awd, env, 10.0)) > float(max_tractive_force(rwd, env, 10.0))


def test_a_car_in_the_air_has_no_grip_and_no_fictitious_forces(car, env):
    crest = Conditions(vertical_curvature=-0.0048)                      # lifts off above ~165 km/h
    v = 190 / 3.6
    terms = load_terms(car, env, v, crest)
    assert float(terms.total_normal) == 0.0 and float(terms.front_base) == 0.0 and float(terms.rear_base) == 0.0
    assert float(max_tractive_force(car, env, v, cond=crest)) == pytest.approx(0.0, abs=1e-9)
    assert float(max_braking_force(car, env, v, crest)[0]) == pytest.approx(0.0, abs=1e-9)
    a = float(acceleration(car, env, v, 4, crest)[0])
    assert a == pytest.approx(-float(terms.drag) / car.general.mass_kg, rel=1e-9)   # only the drag acts: no phantom braking
    assert a > -0.2 * env.gravity
