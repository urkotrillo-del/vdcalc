import numpy as np
import pytest

from conftest import build
from test_lap import stadium_points
from vdcalc.circuit import build_track
from vdcalc.compare import difference, key_figures, lap_time_delta
from vdcalc.lap import solve_lap


@pytest.fixture(scope="module")
def track():
    return build_track(*stadium_points(500.0, 60.0), ds=3.0)


def test_key_figures_without_and_with_a_lap(car, env, track):
    f = key_figures(car, env)
    assert f["time_0_100_kmh"] > 0 and f["distance_100_0_kmh"] > 0 and f["top_speed_kmh"] > 150 and "lap_time_s" not in f
    lap = solve_lap(car, env, track)
    g = key_figures(car, env, lap)
    assert g["lap_time_s"] == lap.lap_time and g["lap_speed_min_kmh"] < g["lap_speed_avg_kmh"] < g["lap_speed_max_kmh"]
    assert 0.5 < g["lap_max_lateral_g"] < 1.2 and 0.5 < g["lap_max_braking_g"] < 1.3


def test_key_figures_mark_what_cannot_be_computed(car, env):
    weak = build({"powertrain.torque_curve": {"rpm": list(car.powertrain.torque_curve.rpm),
                                              "torque_nm": [0.3 * t for t in car.powertrain.torque_curve.torque_nm]}})
    f = key_figures(weak, env)
    assert f["time_0_200_kmh"] is None and f["time_0_100_kmh"] is not None


def test_difference():
    d = difference({"a": 10.0, "b": 0.0, "c": None, "d": 5.0}, {"a": 11.0, "b": 1.0, "c": 3.0, "d": None})
    assert d["a"] == {"reference": 10.0, "current": 11.0, "delta": 1.0, "delta_pct": pytest.approx(10.0)}
    assert d["b"]["delta"] == 1.0 and d["b"]["delta_pct"] is None
    assert d["c"]["delta"] is None and d["d"]["delta_pct"] is None


def test_lap_time_delta_is_zero_against_itself_and_positive_for_a_slower_car(car, env, track):
    ref = solve_lap(car, env, track)
    assert np.allclose(lap_time_delta(ref, ref), 0.0)
    slow = solve_lap(build({"general.mass_kg": 1300.0}), env, track)
    delta = lap_time_delta(ref, slow)
    assert delta[0] == 0.0 and delta[-1] > 0.0
    assert delta[-1] == pytest.approx(slow.lap_time - ref.lap_time, abs=0.1)
    assert np.all(delta > -0.05)                                                         # a heavier car never gains time on this circuit


def test_laps_on_different_circuits_cannot_be_compared(car, env, track):
    other = build_track(*stadium_points(900.0, 60.0), ds=3.0)
    with pytest.raises(ValueError):
        lap_time_delta(solve_lap(car, env, track), solve_lap(car, env, other))
