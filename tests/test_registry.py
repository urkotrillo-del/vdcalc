import copy

import pytest

from conftest import example_dict
from vdcalc.environment import EnvironmentDataError, environment_from_dict, validate_environment_dict
from vdcalc.registry import (GROUP_LABELS, PARAMS, REGISTRY, default_environment, get_path, is_visible, params_in, set_path,
                             validate_params, validate_torque_curve, validate_vehicle_dict)


def errors(issues):
    return [i for i in issues if i.level == "error"]


def warnings(issues):
    return [i for i in issues if i.level == "warning"]


def test_keys_are_unique_and_every_entry_is_documented():
    keys = [p.key for p in REGISTRY]
    assert len(keys) == len(set(keys))
    for p in REGISTRY:
        assert p.label and p.help and p.kind in ("float", "choice", "float_list")
        assert p.key.split(".")[0] in GROUP_LABELS


def test_visibility_rules_point_to_existing_parameters():
    for p in REGISTRY:
        if p.visible_if:
            assert p.visible_if[0] in PARAMS


def test_every_group_has_parameters():
    for group in GROUP_LABELS:
        assert params_in(group)


def test_example_car_has_no_errors_and_only_the_inertial_dyno_warning():
    issues = validate_vehicle_dict(example_dict())
    assert not errors(issues)
    assert [i.key for i in warnings(issues)] == ["powertrain.curve_measured_at"]


def test_missing_required_value_is_an_error():
    data = example_dict()
    del data["general"]["mass_kg"]
    assert any(i.key == "general.mass_kg" for i in errors(validate_vehicle_dict(data)))


def test_value_out_of_hard_limits_is_an_error_and_out_of_typical_is_a_warning():
    data = example_dict()
    data["general"]["mass_kg"] = -5
    assert any(i.key == "general.mass_kg" for i in errors(validate_vehicle_dict(data)))
    data["general"]["mass_kg"] = 3500
    issues = validate_vehicle_dict(data)
    assert not errors(issues) and any(i.key == "general.mass_kg" for i in warnings(issues))


def test_wrong_type_and_invalid_choice_are_errors():
    data = example_dict()
    data["general"]["wheelbase_m"] = "long"
    data["general"]["drive"] = "6x6"
    keys = {i.key for i in errors(validate_vehicle_dict(data))}
    assert {"general.wheelbase_m", "general.drive"} <= keys


def test_hidden_parameters_are_not_required():
    data = example_dict()
    del data["general"]["awd_front_torque_fraction"]       # only used by AWD cars
    assert not errors(validate_vehicle_dict(data))
    data["general"]["drive"] = "AWD"                       # now visible, but it has a default
    assert not errors(validate_vehicle_dict(data))


def test_load_dependent_tyres_require_their_own_parameters():
    data = example_dict()
    data["tires"]["mode"] = "load_dependent"
    keys = {i.key for i in errors(validate_vehicle_dict(data))}
    assert {"tires.front.mu_x0", "tires.rear.c_y"} <= keys
    for axle in ("front", "rear"):
        data["tires"][axle] = {"mu_x0": 1.4, "c_x": 0.1, "mu_y0": 1.2, "c_y": 0.1}
    assert not errors(validate_vehicle_dict(data))


def test_too_steep_load_sensitivity_is_flagged():
    data = example_dict()
    data["tires"]["mode"] = "load_dependent"
    for axle in ("front", "rear"):
        data["tires"][axle] = {"mu_x0": 1.0, "c_x": 0.3, "mu_y0": 1.0, "c_y": 0.3}
    assert any("drops below 0.3" in i.message for i in warnings(validate_vehicle_dict(data)))


@pytest.mark.parametrize("curve", [
    {"rpm": [1000, 2000, 3000], "torque_nm": [100, 120, 130]},                       # too few points
    {"rpm": [1000, 3000, 2000, 4000], "torque_nm": [100, 120, 130, 120]},            # not increasing
    {"rpm": [1000, 2000, 3000, 4000], "torque_nm": [100, 120, 130]},                 # different lengths
    {"rpm": [1000, 2000, 3000, 4000], "torque_nm": [100, -5, 130, 120]},             # negative torque
    {}, None,
])
def test_invalid_torque_curves_are_errors(curve):
    assert errors(validate_torque_curve(curve))


def test_unusual_torque_curves_give_warnings():
    assert warnings(validate_torque_curve({"rpm": [5000, 6000, 7000, 8000], "torque_nm": [100, 120, 130, 120]}))
    assert warnings(validate_torque_curve({"rpm": [1000, 2000, 3000, 4000], "torque_nm": [1000, 4000, 5000, 4000]}))


def test_gear_ratios_not_decreasing_is_a_warning():
    data = example_dict()
    data["powertrain"]["gear_ratios"] = [1.0, 2.0, 3.0]
    issues = validate_vehicle_dict(data)
    assert not errors(issues) and any(i.key == "powertrain.gear_ratios" for i in warnings(issues))


def test_rev_limit_below_the_curve_is_an_error():
    data = example_dict()
    data["powertrain"]["rev_limit_rpm"] = 1000
    assert any(i.key == "powertrain.rev_limit_rpm" for i in errors(validate_vehicle_dict(data)))


def test_environment_defaults_and_validation():
    assert environment_from_dict().gravity == 9.81 and environment_from_dict().air_density == 1.225
    assert environment_from_dict({"gravity": 9.78}).gravity == 9.78
    with pytest.raises(EnvironmentDataError):
        environment_from_dict({"air_density": -1})
    assert default_environment() == {"gravity": 9.81, "air_density": 1.225}
    assert any(i.level == "warning" for i in validate_environment_dict({"gravity": 9.9}))


def test_path_helpers():
    d = {}
    set_path(d, "a.b.c", 3)
    assert d == {"a": {"b": {"c": 3}}} and get_path(d, "a.b.c") == 3 and get_path(d, "a.x", "none") == "none"
    assert is_visible(PARAMS["general.awd_front_torque_fraction"], {"general": {"drive": "AWD"}})
    assert not is_visible(PARAMS["general.awd_front_torque_fraction"], {"general": {"drive": "RWD"}})
