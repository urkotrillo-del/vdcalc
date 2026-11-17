"""Anchor test against the numbers of the first project (the Mazda MX-5 simulator).

It needs the real torque curve of that project: copy `mx5_nd2_torque_curve.csv` to `tests/data/`. Without the
file the test is skipped. With it, the new core must reproduce the old results (0-60 mph 6.06 s, 70-0 mph 48.77 m).
"""
from pathlib import Path

import pytest

from conftest import example_dict
from vdcalc.straightline import accelerate, brake
from vdcalc.units import MPH
from vdcalc.vehicle import load_torque_curve_csv, vehicle_from_dict

CURVE = Path(__file__).parent / "data" / "mx5_nd2_torque_curve.csv"


@pytest.mark.skipif(not CURVE.exists(), reason="copy mx5_nd2_torque_curve.csv to tests/data/ to run the anchor test")
def test_mx5_reproduces_the_first_project(env):
    data = example_dict()
    data["powertrain"]["torque_curve"] = load_torque_curve_csv(CURVE)
    car = vehicle_from_dict(data)
    t60 = accelerate(car, env, 0.0, 60 * MPH).total_time
    d70 = brake(car, env, 70 * MPH, 0.0).total_distance
    assert t60 == pytest.approx(6.06, abs=0.04)
    assert d70 == pytest.approx(48.77, abs=0.3)
