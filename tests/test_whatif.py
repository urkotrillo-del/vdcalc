import json

import pytest

from conftest import build
from test_lap import stadium_points
from vdcalc.circuit import build_track
from vdcalc.lap import solve_lap
from vdcalc.results import to_jsonable
from vdcalc.whatif import DEFAULT_VARIATIONS, Variation, run_study


@pytest.fixture(scope="module")
def track():
    return build_track(*stadium_points(500.0, 60.0), ds=4.0)


@pytest.fixture(scope="module")
def study(track):
    from vdcalc.environment import Environment
    from conftest import example_dict
    from vdcalc.vehicle import vehicle_from_dict
    return run_study(vehicle_from_dict(example_dict()), Environment(), track)


def test_every_effect_has_the_right_sign(study):
    rows = {r["key"]: r for r in study["rows"]}
    assert rows["mass"]["delta_plus_s"] > 0 > rows["mass"]["delta_minus_s"]
    assert rows["power"]["delta_plus_s"] < 0 < rows["power"]["delta_minus_s"]
    assert rows["drag"]["delta_plus_s"] > 0 > rows["drag"]["delta_minus_s"]
    assert rows["grip"]["delta_plus_s"] < 0 < rows["grip"]["delta_minus_s"]
    assert rows["rolling"]["delta_plus_s"] > 0
    assert rows["mass"]["effect_per_percent_s"] > 0 > rows["grip"]["effect_per_percent_s"]


def test_rows_are_sorted_by_importance_and_consistent(study):
    sizes = [max(abs(r["delta_minus_s"]), abs(r["delta_plus_s"])) for r in study["rows"]]
    assert sizes == sorted(sizes, reverse=True)
    for r in study["rows"]:
        assert r["effect_per_percent_s"] == pytest.approx((r["lap_time_plus_s"] - r["lap_time_minus_s"]) / (2 * r["relative"] * 100))
        assert r["delta_plus_s"] == pytest.approx(r["lap_time_plus_s"] - study["baseline_lap_time_s"])


def test_downforce_is_studied_only_when_the_car_has_some(study, track, env):
    assert "downforce" not in {r["key"] for r in study["rows"]}
    winged = run_study(build({"aero.cl": -1.5}), env, track)
    assert "downforce" in {r["key"] for r in winged["rows"]}


def test_baseline_matches_the_lap_and_can_be_supplied(car, env, track):
    lap = solve_lap(car, env, track).lap_time
    laps = []
    result = run_study(car, env, track, baseline=lap, progress=lambda d, t: laps.append((d, t)))
    assert result["baseline_lap_time_s"] == lap
    assert laps[-1] == (laps[-1][1], laps[-1][1]) and laps[-1][1] == 2 * len(result["rows"])      # no extra lap for the baseline


def test_progress_reports_every_lap(car, env, track):
    calls = []
    result = run_study(car, env, track, progress=lambda d, t: calls.append((d, t)))
    total = 2 * len(result["rows"]) + 1
    assert [d for d, _ in calls] == list(range(1, total + 1)) and all(t == total for _, t in calls)


def test_a_variation_that_makes_the_data_invalid_is_skipped(car, env, track):
    broken = Variation("broken", "Broken", 0.05, lambda data, factor: data["general"].update(mass_kg=-1.0))
    calls = []
    result = run_study(car, env, track, variations=[broken, DEFAULT_VARIATIONS[0]], progress=lambda d, t: calls.append((d, t)))
    assert [r["key"] for r in result["rows"]] == ["mass"] and result["skipped"][0]["key"] == "broken"
    assert calls[-1][0] == calls[-1][1] == 5                                                      # baseline + 2 + 2


def test_the_size_of_the_change_can_be_set(car, env, track):
    small = run_study(car, env, track, variations=[DEFAULT_VARIATIONS[0]], relative=0.02)
    large = run_study(car, env, track, variations=[DEFAULT_VARIATIONS[0]], relative=0.10)
    assert small["rows"][0]["relative"] == 0.02
    assert abs(large["rows"][0]["delta_plus_s"]) > 3 * abs(small["rows"][0]["delta_plus_s"])


def test_the_study_does_not_modify_the_vehicle(car, env, track):
    before = car.general.mass_kg
    run_study(car, env, track, variations=[DEFAULT_VARIATIONS[0]])
    assert car.general.mass_kg == before


def test_result_is_serialisable(study):
    json.dumps(to_jsonable(study), allow_nan=False)


def test_with_constant_friction_the_track_width_does_not_matter_and_lltd_is_not_studied(car, env, track):
    result = run_study(car, env, track, variations=[v for v in DEFAULT_VARIATIONS if v.key in ("track", "lltd")])
    assert [r["key"] for r in result["rows"]] == ["track"]                       # lltd is empty in this car: not relevant
    assert abs(result["rows"][0]["delta_plus_s"]) < 0.005                        # constant friction ignores load transfer
