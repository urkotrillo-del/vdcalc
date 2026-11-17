import copy
import json
from pathlib import Path

import pytest

from vdcalc.environment import Environment
from vdcalc.vehicle import vehicle_from_dict

ROOT = Path(__file__).parent.parent
# The reference car of the tests has CONSTANT friction (exact formulas are checked against it). The example car that
# ships with the program (data/examples/mx5_nd2.json) has load-dependent friction and is tested in test_example_car.py.
EXAMPLE_CAR = ROOT / "tests" / "data" / "mx5_constant_mu.json"


def example_dict() -> dict:
    return json.loads(EXAMPLE_CAR.read_text(encoding="utf-8"))


def modified(**changes) -> dict:
    """The example car dictionary with dotted-key changes, e.g. modified(**{"general.drive": "FWD"})."""
    from vdcalc.registry import set_path
    data = copy.deepcopy(example_dict())
    for key, value in changes.items():
        set_path(data, key.replace("__", "."), value)
    return data


@pytest.fixture
def car():
    return vehicle_from_dict(example_dict())


@pytest.fixture
def env():
    return Environment()


def build(changes: dict):
    """Vehicle built from the example with dotted-key changes given as a normal dict."""
    from vdcalc.registry import set_path
    data = copy.deepcopy(example_dict())
    for key, value in changes.items():
        set_path(data, key, value)
    return vehicle_from_dict(data)


def without_drag(vehicle):
    """The same vehicle with zero drag, for analytic tests (the validation rejects Cd = 0 on purpose)."""
    from dataclasses import replace
    return replace(vehicle, aero=replace(vehicle.aero, cd=0.0))
