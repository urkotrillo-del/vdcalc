import json
import math
from pathlib import Path

import numpy as np
import pytest

from test_lap import stadium_points
from vdcalc.circuit import build_track, load_track
from vdcalc.results import circuit_page, gg_set, lap_summary, performance_page, to_jsonable, vehicle_page
from vdcalc.lap import solve_lap

EXAMPLE = Path(__file__).parent.parent / "data" / "circuits" / "example_circuit.csv"


def dumps(obj):
    return json.dumps(to_jsonable(obj), allow_nan=False)


def test_to_jsonable():
    out = to_jsonable({"a": np.array([1.0, np.nan, np.inf]), "b": (np.float64(2.5), np.int64(3), np.bool_(True)), "c": {"d": [np.array([1, 2])]}})
    assert out == {"a": [1.0, None, None], "b": [2.5, 3, True], "c": {"d": [[1, 2]]}}
    assert dumps({"x": float("nan")}) == '{"x": null}'


def test_vehicle_page(car, env):
    p = vehicle_page(car, env)
    assert set(p) == {"drawing", "engine", "gears", "forces", "tyres", "ellipse_front", "ellipse_rear", "key_figures"}
    dumps(p)
    assert p["key_figures"]["0_100_kmh"]["time_s"] > 0


def test_gg_set_has_one_diagram_per_speed_in_g(car, env):
    g = gg_set(car, env)
    assert g["speeds_kmh"][0] == 10 and len(g["diagrams"]) == len(g["speeds_kmh"]) >= 20
    d = next(x for x in g["diagrams"] if x["speed_kmh"] >= 20)
    assert max(d["ay_g"]) == pytest.approx(0.89, abs=0.02) and max(d["ax_brake_g"]) > 0.95
    assert gg_set(car, env, speeds_kmh=[50, 100])["speeds_kmh"] == [50, 100]


def test_gg_speed_grid_is_denser_at_low_speed_and_brackets_every_gear_change(car, env):
    from vdcalc.results import gg_speed_grid
    grid = gg_speed_grid(car, env)
    assert grid == sorted(grid)
    gaps = [b - a for a, b in zip(grid, grid[1:]) if b - a > 1.0]            # ignore the pairs around a gear change
    assert gaps[0] < gaps[-1]
    g = gg_set(car, env)
    gears = [d["gear"] for d in g["diagrams"]]
    changes = [i for i in range(len(gears) - 1) if gears[i] != gears[i + 1]]
    assert len(changes) == len(set(g["straight"]["gear"])) - 1                 # one change per gear passed
    for i in changes:                                                           # the pair around a change is close together
        assert g["diagrams"][i + 1]["speed_kmh"] - g["diagrams"][i]["speed_kmh"] < 1.0
        assert g["diagrams"][i + 1]["ax_accel_g"][len(g["diagrams"][i]["ay_g"]) // 2] < g["diagrams"][i]["ax_accel_g"][len(g["diagrams"][i]["ay_g"]) // 2]


def test_performance_page(car, env):
    p = performance_page(car, env)
    dumps(p)
    for key in ("profile_requested", "profile_max"):
        assert p[key]["speed_ms"][0] == 0.0 and p[key]["speed_ms"][-1] == pytest.approx(0.0, abs=1e-9)
    assert p["profile_requested"]["peak_speed_ms"] == pytest.approx(200 / 3.6)
    assert p["profile_max"]["peak_speed_ms"] > p["profile_requested"]["peak_speed_ms"]
    assert p["profile_max"]["distance_m"][-1] > p["profile_requested"]["distance_m"][-1]


def test_circuit_page_on_the_example_circuit(car, env):
    track = load_track(EXAMPLE, ds=4.0)
    p = circuit_page(car, env, track)
    dumps(p)
    n = len(p["position_m"])
    for key in ("x_m", "y_m", "z_m", "bank_deg", "speed_kmh", "limit_kmh", "long_accel_g", "lat_accel_g", "gear", "time_s"):
        assert len(p[key]) == n
    assert p["summary"]["lap_time_text"].count(":") == 1 and 100 < p["summary"]["lap_time_s"] < 170
    driven = [g for g in p["gear"] if g is not None]                           # None: off the throttle, no gear
    assert len(driven) > 0.3 * n and min(driven) >= 1 and max(driven) <= 6
    assert abs(p["start_heading_rad"]) < 0.05                                           # the example starts heading east
    limit = np.array([np.nan if v is None else v for v in p["limit_kmh"]])
    assert np.isnan(limit).sum() > 0.4 * n and np.nanmin(limit) < 100                    # straights have no limit, corners do
    assert np.all(np.array(p["speed_kmh"])[~np.isnan(limit)] <= limit[~np.isnan(limit)] + 1e-6)


def test_lap_summary(car, env):
    lap = solve_lap(car, env, build_track(*stadium_points(500.0, 60.0), ds=3.0))
    s = lap_summary(lap)
    assert s["speed_min_kmh"] < s["speed_avg_kmh"] < s["speed_max_kmh"] and s["length_m"] == pytest.approx(lap.track.length)
    assert math.isfinite(s["max_lateral_g"]) and s["max_braking_g"] > 0.5
