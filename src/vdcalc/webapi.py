"""Bridge between the web page and the core.

Every function takes JSON text (or plain text for files) and returns JSON text, so that the page only has to move
strings between JavaScript and Python. Nothing here does physics: it validates, calls the core and packs the results.

Shape of the data exchanged
  vehicle   the dictionary of the vehicle file (the form edits it)
  env       {"gravity": ..., "air_density": ...}
  circuit   {"x": [...], "y": [...], "z": [...] or null, "bank_deg": [...] or null, "reverse": bool, "start": int}
Every answer has "ok" and "issues" ([{"level": "error" | "warning", "key": ..., "message": ...}]).
"""
import json

from . import compare, results
from .circuit import CircuitDataError, build_track, check_circuit, parse_circuit_text
from .environment import EnvironmentDataError, environment_from_dict, validate_environment_dict
from .lap import solve_lap
from .registry import GROUP_LABELS, REGISTRY, validate_torque_curve, validate_vehicle_dict
from .vehicle import VehicleDataError, parse_torque_curve_text, parse_vehicle_text, vehicle_from_dict
from . import whatif as whatif_module

CIRCUIT_PREVIEW_DS = 4.0


def _dump(obj) -> str:
    return json.dumps(results.to_jsonable(obj), allow_nan=False)


def _issue_list(issues) -> list:
    return [{"level": i.level, "key": i.key, "message": i.message} for i in issues]


def _failure(message, key="") -> dict:
    return {"ok": False, "issues": [{"level": "error", "key": key, "message": str(message)}]}


def _issues_of(exc) -> list:
    issues = getattr(exc, "issues", None)
    return _issue_list(issues) if issues else [{"level": "error", "key": "", "message": str(exc)}]


# ---------------------------------------------------------------------------------------------
# description of the data, files
# ---------------------------------------------------------------------------------------------


def registry() -> str:
    """Everything the page needs to draw the input forms: groups, parameters, units, limits and help texts."""
    groups = []
    for group, label in GROUP_LABELS.items():
        params = []
        for p in REGISTRY:
            if p.key.split(".")[0] != group:
                continue
            params.append({"key": p.key, "label": p.label, "unit": p.unit, "kind": p.kind, "help": p.help,
                           "default": p.default, "min": p.min, "max": p.max, "typical": p.typical,
                           "choices": list(p.choices), "optional": p.optional,
                           "visible_if": None if p.visible_if is None else [p.visible_if[0], list(p.visible_if[1])]})
        groups.append({"group": group, "label": label, "params": params})
    return _dump({"groups": groups})


def parse_vehicle(text: str) -> str:
    try:
        data = parse_vehicle_text(text)
    except VehicleDataError as e:
        return _dump({"ok": False, "issues": _issues_of(e)})
    return _dump({"ok": True, "data": data, "issues": []})


def parse_torque_curve(text: str) -> str:
    try:
        curve = parse_torque_curve_text(text)
    except VehicleDataError as e:
        return _dump({"ok": False, "issues": _issues_of(e)})
    issues = validate_torque_curve(curve)
    return _dump({"ok": not any(i.level == "error" for i in issues), "curve": curve, "issues": _issue_list(issues)})


def parse_circuit(text: str) -> str:
    try:
        circuit = parse_circuit_text(text)
    except CircuitDataError as e:
        return _dump({"ok": False, "issues": _issues_of(e)})
    issues = check_circuit(circuit["x"], circuit["y"], circuit["z"], circuit["bank_deg"])
    return _dump({"ok": not any(i.level == "error" for i in issues), "circuit": circuit, "issues": _issue_list(issues)})


# ---------------------------------------------------------------------------------------------
# validation and preview
# ---------------------------------------------------------------------------------------------


def validate(vehicle_json: str, env_json: str) -> str:
    """Errors and warnings of the vehicle and the environment, for the form."""
    issues = validate_vehicle_dict(json.loads(vehicle_json)) + validate_environment_dict(json.loads(env_json))
    return _dump({"ok": not any(i.level == "error" for i in issues), "issues": _issue_list(issues)})


def _track(circuit: dict, ds=2.0):
    return build_track(circuit["x"], circuit["y"], circuit.get("z"), circuit.get("bank_deg"), ds=ds,
                       reverse=bool(circuit.get("reverse", False)), start=int(circuit.get("start", 0)))


def circuit_preview(circuit_json: str) -> str:
    """The contour of the circuit as it will be driven (direction and finish line applied)."""
    circuit = json.loads(circuit_json)
    try:
        track = _track(circuit, CIRCUIT_PREVIEW_DS)
    except (CircuitDataError, ValueError) as e:
        return _dump({"ok": False, "issues": _issues_of(e)})
    issues = check_circuit(circuit["x"], circuit["y"], circuit.get("z"), circuit.get("bank_deg"))
    return _dump({"ok": True, "issues": _issue_list(issues), "x_m": track.x, "y_m": track.y, "length_m": track.length,
                  "z_m": track.z, "n_points": len(circuit["x"])})


# ---------------------------------------------------------------------------------------------
# calculation
# ---------------------------------------------------------------------------------------------


def calculate_all(vehicle_json: str, env_json: str, circuit_json: str) -> str:
    """All the results of the three pages. A section that cannot be computed carries its own error."""
    vehicle_data, env_data, circuit = json.loads(vehicle_json), json.loads(env_json), json.loads(circuit_json)
    issues = validate_vehicle_dict(vehicle_data) + validate_environment_dict(env_data)
    out = {"ok": False, "issues": _issue_list(issues)}
    if any(i.level == "error" for i in issues):
        return _dump(out)
    try:
        vehicle, env = vehicle_from_dict(vehicle_data), environment_from_dict(env_data)
    except (VehicleDataError, EnvironmentDataError) as e:
        out["issues"] = _issues_of(e)
        return _dump(out)

    out["ok"] = True
    out["vehicle"], out["performance"], out["circuit"], out["circuit_error"], out["key_figures"] = None, None, None, None, None
    try:
        out["vehicle"] = results.vehicle_page(vehicle, env)
        out["performance"] = results.performance_page(vehicle, env)
    except Exception as e:                                           # the car cannot be analysed: report it, do not crash
        out["ok"] = False
        out["issues"] += _failure(f"The vehicle results could not be computed: {e}")["issues"]
        return _dump(out)
    lap = None
    try:
        track = _track(circuit)
        lap = solve_lap(vehicle, env, track)
        out["circuit"] = results.circuit_page(vehicle, env, track, lap)
    except (CircuitDataError, ValueError) as e:
        out["circuit_error"] = _issues_of(e)
    except Exception as e:
        out["circuit_error"] = _failure(f"The lap could not be computed: {e}")["issues"]
    out["key_figures"] = compare.key_figures(vehicle, env, lap)
    return _dump(out)


def difference(reference_json: str, current_json: str) -> str:
    """Differences between two flat dictionaries of key figures."""
    return _dump(compare.difference(json.loads(reference_json), json.loads(current_json)))


def whatif(vehicle_json: str, env_json: str, circuit_json: str, relative=None, progress=None, study_ds=3.0) -> str:
    """The 'what if' study on the circuit. `progress(done, total)` is called after each lap. study_ds: point spacing."""
    vehicle_data, env_data, circuit = json.loads(vehicle_json), json.loads(env_json), json.loads(circuit_json)
    try:
        vehicle, env = vehicle_from_dict(vehicle_data), environment_from_dict(env_data)
        track = _track(circuit, study_ds)
    except (VehicleDataError, EnvironmentDataError, CircuitDataError, ValueError) as e:
        return _dump({"ok": False, "issues": _issues_of(e)})
    study = whatif_module.run_study(vehicle, env, track, relative=relative, progress=progress)
    return _dump({"ok": True, "issues": [], **study})
