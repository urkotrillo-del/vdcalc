import numpy as np
import pytest

from conftest import build
from vdcalc.straightline import (SpeedNotReachable, accelerate, brake, performance_summary, speed_distance_profile,
                                 top_speed)
from vdcalc.units import MPH


def test_acceleration_run_is_monotonic_and_plausible(car, env):
    run = accelerate(car, env, 0.0, 100 / 3.6)
    assert np.all(np.diff(run.time) >= 0) and np.all(np.diff(run.speed) >= -1e-12) and np.all(np.diff(run.distance) >= 0)
    assert 4.0 < run.total_time < 12.0 and 60.0 < run.total_distance < 250.0
    assert run.time_at(100 / 3.6) == pytest.approx(run.total_time, abs=0.05)


def test_gears_never_go_down_while_accelerating(car, env):
    run = accelerate(car, env, 0.0, 180 / 3.6)
    assert np.all(np.diff(run.gear) >= 0) and run.gear[0] == 0 and run.gear[-1] >= 3


def test_shift_time_costs_time(env):
    free = build({"powertrain.shift_time_s": 0.0})
    slow = build({"powertrain.shift_time_s": 0.35})
    t_free, t_slow = accelerate(free, env, 0.0, 100 / 3.6).total_time, accelerate(slow, env, 0.0, 100 / 3.6).total_time
    assert 0.1 < t_slow - t_free < 0.8


def test_more_grip_and_more_power_are_faster(car, env):
    t0 = accelerate(car, env, 0.0, 100 / 3.6).total_time
    grippy = build({"tires.front": {"mu_x": 1.5, "mu_y": 1.4}, "tires.rear": {"mu_x": 1.5, "mu_y": 1.4}})
    strong = build({"powertrain.torque_curve": {"rpm": list(car.powertrain.torque_curve.rpm),
                                                "torque_nm": [1.3 * t for t in car.powertrain.torque_curve.torque_nm]}})
    assert accelerate(grippy, env, 0.0, 100 / 3.6).total_time < t0
    assert accelerate(strong, env, 0.0, 100 / 3.6).total_time < t0


def test_awd_launches_better_than_rwd(env):
    rwd, awd = build({"general.drive": "RWD"}), build({"general.drive": "AWD"})
    assert accelerate(awd, env, 0.0, 100 / 3.6).total_time < accelerate(rwd, env, 0.0, 100 / 3.6).total_time


def test_unreachable_speed_raises(car, env):
    with pytest.raises(SpeedNotReachable) as e:
        accelerate(car, env, 0.0, 400 / 3.6)
    assert 150 / 3.6 < e.value.max_speed < 300 / 3.6


def test_time_and_distance_at_a_speed(car, env):
    run = accelerate(car, env, 0.0, 120 / 3.6)
    assert run.time_at(60 / 3.6) < run.time_at(100 / 3.6) and run.distance_at(60 / 3.6) < run.distance_at(100 / 3.6)
    assert run.time_at(0.0) == 0.0


def test_braking_run(car, env):
    run = brake(car, env, 100 / 3.6, 0.0)
    assert np.all(np.diff(run.speed) <= 0) and np.all(np.diff(run.distance) >= 0)
    assert 25.0 < run.total_distance < 60.0
    assert brake(car, env, 200 / 3.6).total_distance > 3 * run.total_distance        # distance grows with speed squared
    assert run.distance_at(50 / 3.6) == pytest.approx(run.total_distance - brake(car, env, 50 / 3.6).total_distance, rel=0.02)


def test_braking_is_shorter_with_more_grip_and_with_downforce(car, env):
    grippy = build({"tires.front": {"mu_x": 1.6, "mu_y": 1.4}, "tires.rear": {"mu_x": 1.6, "mu_y": 1.4}})
    assert brake(grippy, env, 100 / 3.6).total_distance < brake(car, env, 100 / 3.6).total_distance
    wing = build({"aero.cl": -2.0})
    assert brake(wing, env, 200 / 3.6).total_distance < brake(car, env, 200 / 3.6).total_distance


def test_top_speed_is_limited_and_responds_to_drag(car, env):
    ts = top_speed(car, env)
    assert 150 / 3.6 < ts.speed < 300 / 3.6 and ts.limited_by in ("power", "rev_limit")
    slippery = build({"aero.cd": 0.25})
    assert top_speed(slippery, env).speed >= ts.speed
    draggy = build({"aero.cd": 0.9})
    assert top_speed(draggy, env).limited_by == "power" and top_speed(draggy, env).speed < ts.speed


def test_speed_distance_profile_ends_at_rest(car, env):
    distance, speed, time = speed_distance_profile(car, env, 150 / 3.6)
    assert speed[0] == 0.0 and speed[-1] == pytest.approx(0.0, abs=1e-9)
    assert np.max(speed) == pytest.approx(150 / 3.6) and np.all(np.diff(distance) >= 0) and np.all(np.diff(time) >= 0)


def test_profile_peak_is_capped_by_what_the_car_can_reach(car, env):
    _, speed, _ = speed_distance_profile(car, env, 500 / 3.6)
    assert np.max(speed) < top_speed(car, env).speed


def test_performance_summary_of_a_car_that_cannot_reach_200(car, env):
    weak = build({"powertrain.torque_curve": {"rpm": list(car.powertrain.torque_curve.rpm),
                                              "torque_nm": [0.3 * t for t in car.powertrain.torque_curve.torque_nm]}})
    summary = performance_summary(weak, env)
    assert summary["0_200_kmh"] is None and summary["0_100_kmh"] is not None
    full = performance_summary(car, env)
    assert full["0_60_mph"]["time_s"] < full["0_100_kmh"]["time_s"]
    assert full["70_0_mph"]["distance_m"] > full["100_0_kmh"]["distance_m"]            # 70 mph is faster than 100 km/h
    assert full["peak_power_kw"] > 100 and 4000 < full["peak_torque_rpm"] < 6000


def test_runs_to_the_top_speed_and_back_to_a_stop(car, env):
    s = performance_summary(car, env)
    assert s["0_top"]["time_s"] > s["0_200_kmh"]["time_s"] and s["top_0"]["distance_m"] > s["200_0_kmh"]["distance_m"]
    assert s["0_100_0"]["time_s"] < s["0_200_0"]["time_s"] < s["0_top_0"]["time_s"]
    assert s["0_top_0"]["distance_m"] == pytest.approx(s["0_top"]["distance_m"] + s["top_0"]["distance_m"], rel=0.02)
    assert s["run_top_speed_ms"] == pytest.approx(0.98 * s["top_speed_ms"])
