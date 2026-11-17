import math

import numpy as np
import pytest

from conftest import build
from vdcalc import curves
from vdcalc.limits import max_tractive_force
from vdcalc.powertrain import rev_limit


def test_drawing_numbers_are_consistent(car):
    d = curves.drawing(car)
    assert d["cg_from_front_axle_m"] + d["cg_from_rear_axle_m"] == pytest.approx(d["wheelbase_m"])
    assert d["front_axle_mass_kg"] + d["rear_axle_mass_kg"] == pytest.approx(d["mass_kg"])
    assert d["cg_from_rear_axle_m"] == pytest.approx(0.53 * 2.309)             # more weight on the front: the CG is nearer the front
    assert d["cg_from_rear_axle_m"] > d["cg_from_front_axle_m"] and d["drive"] == "RWD"


def test_engine_curves(car):
    e = curves.engine_curves(car, n=300)
    assert e["rpm"][0] == car.powertrain.torque_curve.rpm_min and e["rpm"][-1] == rev_limit(car)
    assert np.allclose(e["power_kw"], e["torque_nm"] * e["rpm"] * 2 * math.pi / 60 / 1000)
    assert e["peak_torque_nm"] == pytest.approx(e["torque_nm"].max()) and e["peak_power_kw"] == pytest.approx(e["power_kw"].max())
    assert e["peak_power_rpm"] >= e["peak_torque_rpm"] and e["measured_at"] == "wheel_inertial"


def test_peaks_are_limited_by_a_lower_rev_limit():
    car = build({"powertrain.rev_limit_rpm": 5000.0})
    e = curves.engine_curves(car)
    assert e["rpm"][-1] == 5000.0 and e["peak_power_rpm"] <= 5000.0


def test_gear_lines_and_the_sawtooth(car, env):
    g = curves.gear_curves(car, env)
    assert len(g["gears"]) == 6
    tops = [x["speed_ms"][1] for x in g["gears"]]
    assert all(b > a for a, b in zip(tops, tops[1:]))                          # taller gears reach higher speeds
    assert all(x["speed_ms"][1] > x["speed_ms"][0] > 0 for x in g["gears"])
    shifts = g["shifts"]
    assert len(shifts) >= 4 and [s["from_gear"] for s in shifts] == list(range(1, len(shifts) + 1))
    assert all(s["rpm_after"] < s["rpm_before"] for s in shifts)                # engine speed drops on every upshift
    assert all(g["rpm_min"] - 1 <= s["rpm_after"] and s["rpm_before"] <= g["rev_limit_rpm"] + 1 for s in shifts)
    assert all(b > a for a, b in zip([s["speed_ms"] for s in shifts], [s["speed_ms"] for s in shifts][1:]))
    v, rpm = g["sawtooth_speed_ms"], g["sawtooth_rpm"]
    assert len(v) == len(rpm) and np.all(np.diff(v) >= 0) and rpm.max() <= g["rev_limit_rpm"] + 1


def test_force_curves(car, env):
    f = curves.force_curves(car, env)
    v = f["speed_ms"]
    assert v[0] == 0.0 and np.all(np.diff(v) > 0) and len(f["gear_force_n"]) == 6
    # the envelope is the largest force of any gear
    stack = np.array(f["gear_force_n"])
    ok = ~np.isnan(f["engine_envelope_n"])
    assert np.allclose(f["engine_envelope_n"][ok], np.nanmax(np.where(np.isnan(stack), -np.inf, stack), axis=0)[ok])
    assert np.allclose(f["resistance_n"], f["drag_n"] + f["rolling_n"])
    assert np.all(np.diff(f["drag_n"]) > 0) and np.allclose(f["downforce_n"], 0.0)       # the MX-5 has no downforce
    assert f["tyre_limit_n"][0] == pytest.approx(float(max_tractive_force(car, env, 0.0)))
    # at the top speed the engine force equals the resistances
    i = int(np.argmin(np.abs(v - f["top_speed_ms"])))
    assert f["engine_envelope_n"][i] == pytest.approx(f["resistance_n"][i], rel=0.08)
    assert f["engine_envelope_n"][i + 3] < f["resistance_n"][i + 3]                    # just above it, the drag wins


def test_a_gear_is_shown_only_where_the_engine_is_in_its_curve(car, env):
    f = curves.force_curves(car, env)
    assert not np.isnan(f["gear_force_n"][0][0])                                          # first gear starts from rest
    assert np.isnan(f["gear_force_n"][5][0])                                              # sixth gear does not


def test_downforce_appears_in_the_force_curves(env):
    f = curves.force_curves(build({"aero.cl": -2.0}), env)
    assert f["downforce_n"][-1] > 0 and np.all(np.diff(f["downforce_n"]) >= 0)


def test_tyre_curves(car, env):
    t = curves.tire_curves(car, env)
    assert t["static_load_n"]["front"] == pytest.approx(1138 * 9.81 * 0.53 / 2)
    assert np.allclose(t["front"]["mu_x"], 1.02) and np.allclose(t["front"]["mu_y"], 0.89)    # constant friction: flat lines
    assert np.allclose(t["front"]["fx_max_n"], 1.02 * t["front"]["load_n"])
    loaded = build({"tires.mode": "load_dependent", "tires.front": {"mu_x0": 1.5, "c_x": 0.12, "mu_y0": 1.3, "c_y": 0.1},
                    "tires.rear": {"mu_x0": 1.6, "c_x": 0.10, "mu_y0": 1.35, "c_y": 0.1}})
    tl = curves.tire_curves(loaded, env)
    assert np.all(np.diff(tl["front"]["mu_x"]) < 0) and tl["front"]["mu_x"][0] == pytest.approx(1.5)
    assert np.all(np.diff(tl["front"]["fx_max_n"][:25]) > 0)                                  # the force still grows with the load
    assert tl["front"]["mu_y"][10] == pytest.approx(1.3 - 0.1 * tl["front"]["load_n"][10] / 1000)


def test_friction_ellipse(car, env):
    e = curves.friction_ellipse(car, env, "rear")
    fz = 1138 * 9.81 * 0.47 / 2
    assert e["fx_max_n"] == pytest.approx(1.02 * fz) and e["fy_max_n"] == pytest.approx(0.89 * fz)
    assert np.allclose((e["fx"] / e["fx_max_n"]) ** 2 + (e["fy"] / e["fy_max_n"]) ** 2, 1.0)
    assert e["fx"][0] == pytest.approx(e["fx"][-1]) and e["fy"][0] == pytest.approx(e["fy"][-1], abs=1e-9)
    assert curves.friction_ellipse(car, env, "front", load_per_tire=3000.0)["fy_max_n"] == pytest.approx(0.89 * 3000.0)
