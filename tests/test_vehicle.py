import json

import pytest

from conftest import example_dict
from vdcalc.vehicle import (VehicleDataError, load_torque_curve_csv, load_vehicle, save_vehicle, vehicle_from_dict,
                            vehicle_to_dict)


def test_example_car_loads(car):
    assert car.general.mass_kg == 1138.0 and car.general.drive == "RWD"
    assert car.powertrain.gear_ratios[0] == 5.087 and len(car.powertrain.gear_ratios) == 6
    assert car.tires.front.mu_x0 == 1.02 and car.tires.front.c_x == 0.0   # constant friction: c = 0


def test_aero_front_fraction_defaults_to_the_static_front_weight_fraction(car):
    assert car.aero.front_fraction == car.general.front_weight_fraction


def test_explicit_aero_balance_is_kept():
    data = example_dict()
    data["aero"]["front_fraction"] = 0.4
    assert vehicle_from_dict(data).aero.front_fraction == 0.4


def test_defaults_fill_missing_optional_values():
    data = example_dict()
    del data["powertrain"]["engine_inertia_kgm2"]
    assert vehicle_from_dict(data).powertrain.engine_inertia_kgm2 == 0.15


def test_dictionary_roundtrip(car):
    again = vehicle_from_dict(vehicle_to_dict(car))
    assert again == car or (again.general == car.general and again.tires == car.tires and again.aero == car.aero
                            and again.powertrain.gear_ratios == car.powertrain.gear_ratios)
    assert list(again.powertrain.torque_curve.rpm) == list(car.powertrain.torque_curve.rpm)


def test_load_dependent_tyres_roundtrip():
    data = example_dict()
    data["tires"] = {"wheel_radius_m": 0.3, "rolling_coefficient": 0.012, "mode": "load_dependent",
                     "front": {"mu_x0": 1.5, "c_x": 0.1, "mu_y0": 1.3, "c_y": 0.12},
                     "rear": {"mu_x0": 1.6, "c_x": 0.11, "mu_y0": 1.35, "c_y": 0.1}}
    car = vehicle_from_dict(data)
    assert car.tires.rear.c_y == 0.1
    assert vehicle_to_dict(car)["tires"]["front"]["mu_y0"] == 1.3


def test_file_roundtrip(tmp_path, car):
    path = tmp_path / "car.json"
    save_vehicle(car, path)
    assert json.loads(path.read_text())["format_version"] == 1
    again = load_vehicle(path)
    assert again.general == car.general and again.aero == car.aero


def test_unsupported_file_version_is_rejected(tmp_path):
    data = example_dict()
    data["format_version"] = 99
    path = tmp_path / "car.json"
    path.write_text(json.dumps(data))
    with pytest.raises(VehicleDataError, match="format version"):
        load_vehicle(path)


def test_invalid_data_raises_with_the_list_of_issues():
    data = example_dict()
    data["general"]["mass_kg"] = -1
    data["aero"]["cd"] = "x"
    with pytest.raises(VehicleDataError) as e:
        vehicle_from_dict(data)
    assert {i.key for i in e.value.issues} == {"general.mass_kg", "aero.cd"}


def test_torque_curve_csv_with_comments_and_header(tmp_path):
    path = tmp_path / "curve.csv"
    path.write_text("# measured on a dyno\nrpm,torque_nm\n3000,150\n4000,180.5\n5000,190\n6000,170\n")
    assert load_torque_curve_csv(path) == {"rpm": [3000.0, 4000.0, 5000.0, 6000.0], "torque_nm": [150.0, 180.5, 190.0, 170.0]}


def test_empty_torque_curve_csv_is_rejected(tmp_path):
    path = tmp_path / "empty.csv"
    path.write_text("# nothing here\n")
    with pytest.raises(VehicleDataError):
        load_torque_curve_csv(path)
