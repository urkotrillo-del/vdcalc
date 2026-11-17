"""The example car that ships with the program: load-dependent friction and a lateral load transfer share.

It is calibrated to keep the reference results of the first project (70-0 mph in 48.77 m, 0.89 g on the skidpad) while
showing the effects of load transfer that a constant-friction tyre cannot show.
"""
import copy
import json
from pathlib import Path

import pytest

from vdcalc.environment import Environment
from vdcalc.lateral import corner_speed
from vdcalc.registry import validate_vehicle_dict
from vdcalc.straightline import accelerate, brake
from vdcalc.units import MPH
from vdcalc.vehicle import vehicle_from_dict
from vdcalc.whatif import DEFAULT_VARIATIONS, run_study
from vdcalc.circuit import load_track

ROOT = Path(__file__).parent.parent
SHIPPED = ROOT / "data" / "examples" / "mx5_nd2.json"
SKIDPAD_RADIUS = 45.72      # 300 ft diameter
env = Environment()


def shipped_dict():
    return json.loads(SHIPPED.read_text(encoding="utf-8"))


def car(**changes):
    data = copy.deepcopy(shipped_dict())
    for key, value in changes.items():
        group, name = key.split("__")
        data[group][name] = value
    return vehicle_from_dict(data)


def skidpad_g(c):
    v = float(corner_speed(c, env, 1.0 / SKIDPAD_RADIUS))
    return v * v / SKIDPAD_RADIUS / env.gravity


def test_the_example_uses_load_dependent_friction_and_a_lateral_transfer_share():
    data = shipped_dict()
    assert data["tires"]["mode"] == "load_dependent" and data["general"]["lltd_front"] == 0.58
    issues = validate_vehicle_dict(data)
    assert not [i for i in issues if i.level == "error"]
    assert not [i for i in issues if "drops below" in i.message]
    c = car()
    assert c.tires.front.c_x > 0 and c.tires.front.c_y > c.tires.front.c_x          # lateral grip is more load sensitive


def test_it_keeps_the_reference_results_of_the_first_project():
    c = car()
    assert brake(c, env, 70 * MPH, 0.0).total_distance == pytest.approx(48.77, abs=0.15)
    assert skidpad_g(c) == pytest.approx(0.89, abs=0.005)
    assert 5.7 < accelerate(c, env, 0.0, 60 * MPH).total_time < 6.3


def test_the_friction_at_the_static_load_is_a_sensible_road_tyre():
    c = car()
    front, rear = 1138 * env.gravity * 0.53 / 2000, 1138 * env.gravity * 0.47 / 2000      # kN per tyre
    for tire, load in ((c.tires.front, front), (c.tires.rear, rear)):
        assert 1.0 < tire.mu_x0 - tire.c_x * load < 1.25 and 0.9 < tire.mu_y0 - tire.c_y * load < 1.1


def test_load_transfer_now_costs_lateral_grip():
    base = skidpad_g(car())
    assert skidpad_g(car(general__cg_height_m=0.56)) < base - 0.02                    # a higher centre of gravity: more transfer
    assert skidpad_g(car(general__front_track_m=1.596, general__rear_track_m=1.606)) > base   # a wider track: less transfer
    assert skidpad_g(car(general__mass_kg=1138 * 1.2)) < base - 0.03                  # a heavier car loses grip per kg
    assert skidpad_g(car(general__lltd_front=0.70)) < base                            # too much transfer on the limiting axle


def test_the_what_if_study_shows_the_transfer_effects():
    track = load_track(ROOT / "data" / "circuits" / "example_circuit.csv", ds=4.0)
    keys = {"mass", "cg_height", "track", "lltd"}
    study = run_study(car(), env, track, variations=[v for v in DEFAULT_VARIATIONS if v.key in keys])
    effect = {r["key"]: r["effect_per_percent_s"] for r in study["rows"]}
    assert set(effect) == keys
    assert effect["cg_height"] > 0.02 and effect["track"] < -0.02 and effect["lltd"] > 0.02     # all with the physical sign
    assert effect["mass"] > 0.15                                                               # a heavier car is much slower


def test_the_other_levers_are_in_the_default_study():
    assert {"lltd", "track"} <= {v.key for v in DEFAULT_VARIATIONS}
