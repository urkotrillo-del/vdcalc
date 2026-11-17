import json
from pathlib import Path

import pytest

from vdcalc import webapi

ROOT = Path(__file__).resolve().parent.parent
CAR = (ROOT / "data" / "examples" / "mx5_nd2.json").read_text(encoding="utf-8")
CIRCUIT_TEXT = (ROOT / "data" / "circuits" / "example_circuit.csv").read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def circuit():
    c = json.loads(webapi.parse_circuit(CIRCUIT_TEXT))["circuit"]
    c.update(reverse=False, start=0)
    return c


def test_registry_has_all_groups():
    r = json.loads(webapi.registry())
    assert [g["group"] for g in r["groups"]] == ["general", "powertrain", "tires", "aero", "environment"]
    keys = [p["key"] for g in r["groups"] for p in g["params"]]
    assert "general.mass_kg" in keys and "environment.air_density" in keys


def test_parse_vehicle_ok_and_errors():
    assert json.loads(webapi.parse_vehicle(CAR))["ok"]
    assert not json.loads(webapi.parse_vehicle("not json"))["ok"]
    assert not json.loads(webapi.parse_vehicle('{"format_version": 9}'))["ok"]


def test_parse_torque_curve():
    r = json.loads(webapi.parse_torque_curve("rpm,torque\n2000,150\n3000,180\n4000,200\n5000,190\n"))
    assert r["ok"] and r["curve"]["rpm"][0] == 2000
    assert not json.loads(webapi.parse_torque_curve("nothing here"))["ok"]


def test_parse_circuit_without_z_and_bank():
    r = json.loads(webapi.parse_circuit("# c\n" + "\n".join(f"{x},{y}" for x, y in
                                                         [(0, 0), (50, 0), (100, 0), (100, 50), (100, 100), (50, 100), (0, 100), (0, 50),
                                                          (25, 25), (75, 25), (75, 75), (25, 75)])))
    assert r["circuit"]["z"] is None and r["circuit"]["bank_deg"] is None


def test_validate_flags_negative_mass():
    data = json.loads(CAR)
    data["general"]["mass_kg"] = -5
    r = json.loads(webapi.validate(json.dumps(data), "{}"))
    assert not r["ok"] and any(i["key"] == "general.mass_kg" and i["level"] == "error" for i in r["issues"])


def test_calculate_all(circuit):
    r = json.loads(webapi.calculate_all(CAR, "{}", json.dumps(circuit)))
    assert r["ok"] and r["circuit"]["summary"]["lap_time_s"] > 60
    assert r["performance"]["gg"]["diagrams"] and r["vehicle"]["engine"]["rpm"]
    assert r["key_figures"]["lap_time_s"] == pytest.approx(r["circuit"]["summary"]["lap_time_s"])


def test_calculate_all_invalid_vehicle_gives_no_results(circuit):
    data = json.loads(CAR)
    data["general"]["mass_kg"] = -5
    r = json.loads(webapi.calculate_all(json.dumps(data), "{}", json.dumps(circuit)))
    assert not r["ok"] and "vehicle" not in r


def test_bad_circuit_does_not_break_the_car_results():
    bad = {"x": [0, 1, 2], "y": [0, 0, 0], "z": None, "bank_deg": None, "reverse": False, "start": 0}
    r = json.loads(webapi.calculate_all(CAR, "{}", json.dumps(bad)))
    assert r["ok"] and r["circuit"] is None and r["circuit_error"] and r["vehicle"]


def test_reverse_and_start_change_the_preview(circuit):
    a = json.loads(webapi.circuit_preview(json.dumps(circuit)))
    b = json.loads(webapi.circuit_preview(json.dumps({**circuit, "reverse": True})))
    c = json.loads(webapi.circuit_preview(json.dumps({**circuit, "start": 300})))
    assert a["ok"] and b["ok"] and c["ok"]
    assert abs(a["length_m"] - b["length_m"]) < 1.0
    assert (a["x_m"][0], a["y_m"][0]) != (c["x_m"][0], c["y_m"][0])


def test_environment_changes_the_lap(circuit):
    a = json.loads(webapi.calculate_all(CAR, "{}", json.dumps(circuit)))
    b = json.loads(webapi.calculate_all(CAR, '{"air_density": 1.0}', json.dumps(circuit)))
    assert a["circuit"]["summary"]["lap_time_s"] != b["circuit"]["summary"]["lap_time_s"]


def test_difference():
    r = json.loads(webapi.difference('{"a": 10.0, "b": null}', '{"a": 11.0, "b": 2.0}'))
    assert r["a"]["delta"] == pytest.approx(1.0) and r["a"]["delta_pct"] == pytest.approx(10.0) and r["b"]["delta"] is None


def test_whatif_reports_progress(circuit):
    calls = []
    r = json.loads(webapi.whatif(CAR, "{}", json.dumps(circuit), relative=0.05, progress=lambda d, t: calls.append((d, t)),
                                 study_ds=6.0))
    assert r["ok"] and r["rows"] and calls[-1][0] == calls[-1][1]
