"""Vehicle data model.

A vehicle is stored as ONE JSON file that contains everything, including the torque curve. The same
dictionary structure is what the input form edits, so loading a file simply fills the form.
"""
import csv
import io
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .powertrain import TorqueCurve
from .registry import PARAMS, get_path, set_path, validate_vehicle_dict

FORMAT_VERSION = 1


class VehicleDataError(ValueError):
    """Raised when the data of a vehicle has errors. `issues` lists every problem found."""

    def __init__(self, issues):
        self.issues = issues
        super().__init__("; ".join(i.message for i in issues if i.level == "error"))


@dataclass(frozen=True)
class General:
    mass_kg: float
    front_weight_fraction: float
    cg_height_m: float
    wheelbase_m: float
    front_track_m: float
    rear_track_m: float
    drive: str
    awd_front_torque_fraction: float
    brake_front_fraction: float
    lltd_front: Optional[float]


@dataclass(frozen=True)
class Powertrain:
    torque_curve: TorqueCurve
    gear_ratios: tuple
    final_drive: float
    drivetrain_efficiency: float
    engine_inertia_kgm2: float
    wheels_inertia_kgm2: float
    shift_time_s: float
    rev_limit_rpm: Optional[float]


@dataclass(frozen=True)
class AxleTire:
    """Friction of the tyres of one axle: mu = mu0 - c * Fz, with Fz the load on ONE tyre in kN.

    A constant-friction tyre is the particular case c = 0.
    """
    mu_x0: float
    c_x: float
    mu_y0: float
    c_y: float


@dataclass(frozen=True)
class Tires:
    wheel_radius_m: float
    rolling_coefficient: float
    mode: str
    front: AxleTire
    rear: AxleTire


@dataclass(frozen=True)
class Aero:
    cd: float
    cl: float
    frontal_area_m2: float
    front_fraction: float        # resolved: never None


@dataclass(frozen=True)
class Vehicle:
    name: str
    general: General
    powertrain: Powertrain
    tires: Tires
    aero: Aero


# ---------------------------------------------------------------------------------------------
# dictionary <-> Vehicle
# ---------------------------------------------------------------------------------------------


def _value(data: dict, key: str):
    value = get_path(data, key)
    return PARAMS[key].default if value is None else value


def _axle(data: dict, axle: str, mode: str) -> AxleTire:
    if mode == "constant":
        return AxleTire(float(_value(data, f"tires.{axle}.mu_x")), 0.0, float(_value(data, f"tires.{axle}.mu_y")), 0.0)
    return AxleTire(float(_value(data, f"tires.{axle}.mu_x0")), float(_value(data, f"tires.{axle}.c_x")),
                    float(_value(data, f"tires.{axle}.mu_y0")), float(_value(data, f"tires.{axle}.c_y")))


def vehicle_from_dict(data: dict) -> Vehicle:
    """Validates the dictionary and builds a Vehicle. Raises VehicleDataError if there are errors."""
    issues = validate_vehicle_dict(data)
    errors = [i for i in issues if i.level == "error"]
    if errors:
        raise VehicleDataError(errors)
    v = lambda key: _value(data, key)
    opt = lambda key: None if v(key) is None else float(v(key))
    curve = get_path(data, "powertrain.torque_curve")
    front_weight = float(v("general.front_weight_fraction"))
    mode = v("tires.mode")
    return Vehicle(
        name=str(data.get("name", "Unnamed vehicle")),
        general=General(float(v("general.mass_kg")), front_weight, float(v("general.cg_height_m")),
                        float(v("general.wheelbase_m")), float(v("general.front_track_m")), float(v("general.rear_track_m")),
                        str(v("general.drive")), float(v("general.awd_front_torque_fraction")),
                        float(v("general.brake_front_fraction")), opt("general.lltd_front")),
        powertrain=Powertrain(TorqueCurve(curve["rpm"], curve["torque_nm"], v("powertrain.curve_measured_at")),
                              tuple(float(x) for x in v("powertrain.gear_ratios")), float(v("powertrain.final_drive")),
                              float(v("powertrain.drivetrain_efficiency")), float(v("powertrain.engine_inertia_kgm2")),
                              float(v("powertrain.wheels_inertia_kgm2")), float(v("powertrain.shift_time_s")),
                              opt("powertrain.rev_limit_rpm")),
        tires=Tires(float(v("tires.wheel_radius_m")), float(v("tires.rolling_coefficient")), mode,
                    _axle(data, "front", mode), _axle(data, "rear", mode)),
        aero=Aero(float(v("aero.cd")), float(v("aero.cl")), float(v("aero.frontal_area_m2")),
                  front_weight if v("aero.front_fraction") is None else float(v("aero.front_fraction"))),
    )


def vehicle_to_dict(vehicle: Vehicle) -> dict:
    """The dictionary that the input form shows and the file stores."""
    g, pt, t, a = vehicle.general, vehicle.powertrain, vehicle.tires, vehicle.aero
    data: dict = {"name": vehicle.name}
    for key, value in (
        ("general.mass_kg", g.mass_kg), ("general.front_weight_fraction", g.front_weight_fraction),
        ("general.cg_height_m", g.cg_height_m), ("general.wheelbase_m", g.wheelbase_m),
        ("general.front_track_m", g.front_track_m), ("general.rear_track_m", g.rear_track_m),
        ("general.drive", g.drive), ("general.awd_front_torque_fraction", g.awd_front_torque_fraction),
        ("general.brake_front_fraction", g.brake_front_fraction), ("general.lltd_front", g.lltd_front),
        ("powertrain.curve_measured_at", pt.torque_curve.measured_at), ("powertrain.gear_ratios", list(pt.gear_ratios)),
        ("powertrain.final_drive", pt.final_drive), ("powertrain.drivetrain_efficiency", pt.drivetrain_efficiency),
        ("powertrain.engine_inertia_kgm2", pt.engine_inertia_kgm2), ("powertrain.wheels_inertia_kgm2", pt.wheels_inertia_kgm2),
        ("powertrain.shift_time_s", pt.shift_time_s), ("powertrain.rev_limit_rpm", pt.rev_limit_rpm),
        ("tires.wheel_radius_m", t.wheel_radius_m), ("tires.rolling_coefficient", t.rolling_coefficient), ("tires.mode", t.mode),
        ("aero.cd", a.cd), ("aero.cl", a.cl), ("aero.frontal_area_m2", a.frontal_area_m2), ("aero.front_fraction", a.front_fraction),
    ):
        set_path(data, key, value)
    set_path(data, "powertrain.torque_curve", {"rpm": [float(x) for x in pt.torque_curve.rpm],
                                              "torque_nm": [float(x) for x in pt.torque_curve.torque_nm]})
    for axle, tire in (("front", t.front), ("rear", t.rear)):
        if t.mode == "constant":
            set_path(data, f"tires.{axle}.mu_x", tire.mu_x0)
            set_path(data, f"tires.{axle}.mu_y", tire.mu_y0)
        else:
            for k, val in (("mu_x0", tire.mu_x0), ("c_x", tire.c_x), ("mu_y0", tire.mu_y0), ("c_y", tire.c_y)):
                set_path(data, f"tires.{axle}.{k}", val)
    return data


# ---------------------------------------------------------------------------------------------
# files
# ---------------------------------------------------------------------------------------------


def save_vehicle(vehicle: Vehicle, path) -> None:
    data = {"format_version": FORMAT_VERSION, **vehicle_to_dict(vehicle)}
    Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")


def load_vehicle_dict(path) -> dict:
    return parse_vehicle_text(Path(path).read_text(encoding="utf-8-sig"))


def load_vehicle(path) -> Vehicle:
    return vehicle_from_dict(load_vehicle_dict(path))


def _issue(message):
    from .registry import Issue
    return Issue("error", "file", message)


def parse_torque_curve_text(text: str) -> dict:
    """Reads the text of a two-column CSV (rpm, torque in N*m). Comment lines start with # and a header line is allowed.

    Returns {"rpm": [...], "torque_nm": [...]}, the structure stored in the vehicle file.
    """
    rpm, torque = [], []
    for row in csv.reader(io.StringIO(text)):
        if not row or row[0].strip().startswith("#"):
            continue
        try:
            a, b = float(row[0]), float(row[1])
        except (ValueError, IndexError):
            continue  # header or malformed line
        rpm.append(a)
        torque.append(b)
    if not rpm:
        raise VehicleDataError([_issue("The torque curve file has no numeric rows (expected: rpm, torque in N*m).")])
    return {"rpm": rpm, "torque_nm": torque}


def load_torque_curve_csv(path) -> dict:
    """Reads a torque curve CSV file (see parse_torque_curve_text)."""
    with open(path, newline="", encoding="utf-8-sig") as f:
        return parse_torque_curve_text(f.read())


def parse_vehicle_text(text: str) -> dict:
    """Reads the text of a vehicle file and checks its format version. Returns the dictionary that fills the form."""
    try:
        data = json.loads(text)
    except json.JSONDecodeError as e:
        raise VehicleDataError([_issue(f"The file is not a valid vehicle file (JSON error: {e}).")])
    if not isinstance(data, dict):
        raise VehicleDataError([_issue("The file is not a valid vehicle file.")])
    version = data.get("format_version")
    if version != FORMAT_VERSION:
        raise VehicleDataError([_issue(f"Unsupported file format version: {version!r} (expected {FORMAT_VERSION}).")])
    return data
