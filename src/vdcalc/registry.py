"""Parameter registry: one entry per input datum.

The registry is the single source of truth for everything that describes an input: its unit, valid
range, typical range, help text and the conditions under which it is shown. The input form, the
hover hints, the validation rules and the file schema are all generated from it.

Units are SI throughout. The only prefixed unit is the tyre load sensitivity (1/kN), because values in
1/N would be unreadable (0.0001 instead of 0.1).
"""
from dataclasses import dataclass
from typing import Any, Optional

# ---------------------------------------------------------------------------------------------
# Parameter definition
# ---------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Param:
    key: str                              # dotted path inside the vehicle dictionary
    label: str
    unit: str
    kind: str                             # "float", "choice", "float_list"
    help: str
    default: Any = None                   # None = no default, the user must provide it
    min: Optional[float] = None           # hard limits: outside is an error
    max: Optional[float] = None
    typical: Optional[tuple] = None       # typical range: outside is a warning
    choices: tuple = ()
    optional: bool = False                # may be absent (None)
    visible_if: Optional[tuple] = None    # (other_key, (values,...)): shown only when the other key has one of them


@dataclass(frozen=True)
class Issue:
    level: str                            # "error" blocks the calculation, "warning" does not
    key: str
    message: str


GROUP_LABELS = {
    "general": "General",
    "powertrain": "Powertrain",
    "tires": "Tyres",
    "aero": "Aerodynamics",
    "environment": "Environment",
}


def _axle_params(axle: str) -> list:
    name = "Front" if axle == "front" else "Rear"
    p = f"tires.{axle}."
    constant = ("tires.mode", ("constant",))
    loaded = ("tires.mode", ("load_dependent",))
    return [
        Param(p + "mu_x", f"{name} longitudinal grip", "-", "float", visible_if=constant, min=0.1, max=4.0,
              typical=(0.7, 2.2),
              help="Peak longitudinal friction coefficient of the tyres on this axle (traction and braking). "
                   "Constant: it does not change with the load on the tyre."),
        Param(p + "mu_y", f"{name} lateral grip", "-", "float", visible_if=constant, min=0.1, max=4.0,
              typical=(0.7, 2.2),
              help="Peak lateral friction coefficient of the tyres on this axle (cornering). It is usually a bit "
                   "lower than the longitudinal one."),
        Param(p + "mu_x0", f"{name} longitudinal grip at zero load", "-", "float", visible_if=loaded, min=0.1, max=6.0,
              typical=(0.8, 3.0),
              help="Longitudinal friction coefficient extrapolated to zero tyre load, in mu = mu0 - c * Fz."),
        Param(p + "c_x", f"{name} longitudinal load sensitivity", "1/kN", "float", visible_if=loaded, min=0.0, max=1.0,
              typical=(0.0, 0.3),
              help="Slope c of the longitudinal friction coefficient against the load on one tyre, in kN: "
                   "mu = mu0 - c * Fz. Typical values are a few hundredths to a few tenths per kN."),
        Param(p + "mu_y0", f"{name} lateral grip at zero load", "-", "float", visible_if=loaded, min=0.1, max=6.0,
              typical=(0.8, 3.0),
              help="Lateral friction coefficient extrapolated to zero tyre load, in mu = mu0 - c * Fz."),
        Param(p + "c_y", f"{name} lateral load sensitivity", "1/kN", "float", visible_if=loaded, min=0.0, max=1.0,
              typical=(0.0, 0.3),
              help="Slope c of the lateral friction coefficient against the load on one tyre, in kN."),
    ]


REGISTRY = [
    # --- general ---------------------------------------------------------------------------------
    Param("general.mass_kg", "Mass", "kg", "float", min=50, max=6000, typical=(250, 2600),
          help="Total mass of the car as it runs: including driver, fluids and fuel."),
    Param("general.front_weight_fraction", "Front weight fraction", "-", "float", min=0.2, max=0.8, typical=(0.38, 0.62),
          help="Fraction of the static weight carried by the front axle (0.5 = 50/50). Weigh the car on four scales."),
    Param("general.cg_height_m", "Centre of gravity height", "m", "float", min=0.1, max=1.6, typical=(0.2, 0.7),
          help="Height of the centre of gravity above the ground. It can be measured by lifting one axle and "
               "weighing the change in axle load, or estimated from the geometry."),
    Param("general.wheelbase_m", "Wheelbase", "m", "float", min=0.8, max=5.0, typical=(1.5, 3.4),
          help="Distance between the front and rear axle centres."),
    Param("general.front_track_m", "Front track", "m", "float", min=0.6, max=2.4, typical=(1.0, 1.8),
          help="Distance between the centres of the two front tyre contact patches."),
    Param("general.rear_track_m", "Rear track", "m", "float", min=0.6, max=2.4, typical=(1.0, 1.8),
          help="Distance between the centres of the two rear tyre contact patches."),
    Param("general.drive", "Drive layout", "-", "choice", choices=("RWD", "FWD", "AWD"),
          help="Which wheels are driven: rear (RWD), front (FWD) or all four (AWD)."),
    Param("general.awd_front_torque_fraction", "Front torque share (AWD)", "-", "float", default=0.5, min=0.0, max=1.0,
          typical=(0.2, 0.8), visible_if=("general.drive", ("AWD",)),
          help="Fraction of the drive force sent to the front axle in an all-wheel-drive car."),
    Param("general.brake_front_fraction", "Front brake force share", "-", "float", min=0.2, max=0.95, typical=(0.5, 0.8),
          help="Fraction of the braking force applied at the front axle. It can be calculated from the brake "
               "hardware (pistons, discs, master cylinders) or measured."),
    Param("general.lltd_front", "Front lateral load transfer share (optional)", "-", "float", optional=True,
          min=0.0, max=1.0, typical=(0.4, 0.75),
          help="Fraction of the lateral load transfer carried by the front axle. Optional: if left empty, the "
               "front static weight fraction is used and results that depend on it are marked as estimated."),
    # --- powertrain ------------------------------------------------------------------------------
    Param("powertrain.curve_measured_at", "Torque curve measured at", "-", "choice",
          choices=("crank", "wheel_brake", "wheel_inertial"),
          help="Where the torque curve was measured. crank: engine dyno (losses are applied by the model). "
               "wheel_brake: chassis dyno at steady speeds (losses already included). wheel_inertial: chassis dyno "
               "that accelerates a roller (losses included, and part of the inertia too)."),
    Param("powertrain.gear_ratios", "Gear ratios", "-", "float_list", min=0.05, max=30.0,
          help="Gear ratios from first to last gear. One value for a direct-drive or single-speed car."),
    Param("powertrain.final_drive", "Final drive ratio", "-", "float", min=0.5, max=15.0, typical=(2.0, 6.5),
          help="Final drive (differential) ratio, including any primary reduction or chain ratio."),
    Param("powertrain.drivetrain_efficiency", "Drivetrain efficiency", "-", "float", default=0.93, min=0.5, max=1.0,
          typical=(0.8, 0.98),
          help="Fraction of the power that reaches the wheels. Applied to a crank curve, and to the engine "
               "inertia term in all cases. Typical default shown; change it if you know it."),
    Param("powertrain.engine_inertia_kgm2", "Engine and flywheel inertia", "kg*m2", "float", default=0.15, min=0.0,
          max=3.0, typical=(0.03, 0.6),
          help="Rotating inertia of engine, flywheel and clutch. It is rarely known; the default is a typical "
               "value for a passenger car engine."),
    Param("powertrain.wheels_inertia_kgm2", "Wheels inertia (all four)", "kg*m2", "float", default=3.0, min=0.0,
          max=40.0, typical=(0.5, 8.0),
          help="Sum of the rotating inertias of the four wheels, tyres and brake discs."),
    Param("powertrain.shift_time_s", "Shift time", "s", "float", default=0.35, min=0.0, max=3.0, typical=(0.02, 1.0),
          help="Time without drive force during an upshift. Use 0 for a single-speed car."),
    Param("powertrain.rev_limit_rpm", "Rev limit (optional)", "rpm", "float", optional=True, min=500, max=30000,
          typical=(2000, 20000),
          help="Maximum engine speed. If empty, the last point of the torque curve is the limit."),
    # --- tyres -----------------------------------------------------------------------------------
    Param("tires.wheel_radius_m", "Wheel radius", "m", "float", min=0.1, max=0.8, typical=(0.2, 0.42),
          help="Rolling radius of the tyre under load (slightly smaller than the unloaded radius)."),
    Param("tires.rolling_coefficient", "Rolling resistance coefficient", "-", "float", default=0.012, min=0.0, max=0.1,
          typical=(0.006, 0.03),
          help="Rolling resistance force divided by the vertical load. Typical default shown; "
               "on asphalt, road tyres are around 0.010-0.015."),
    Param("tires.mode", "Tyre model", "-", "choice", choices=("constant", "load_dependent"),
          help="constant: one friction coefficient per direction. load_dependent: the coefficient decreases "
               "linearly with the load on the tyre, mu = mu0 - c * Fz."),
    *_axle_params("front"),
    *_axle_params("rear"),
    # --- aerodynamics ----------------------------------------------------------------------------
    Param("aero.cd", "Drag coefficient", "-", "float", min=0.05, max=3.0, typical=(0.25, 1.3),
          help="Drag coefficient of the whole car. It is the same convention used for road cars (Cd)."),
    Param("aero.cl", "Lift coefficient (negative = downforce)", "-", "float", min=-8.0, max=2.0, typical=(-4.0, 0.4),
          help="Lift coefficient of the whole car. A negative value means downforce. Use 0 for a car with no "
               "relevant aerodynamic load."),
    Param("aero.frontal_area_m2", "Frontal area", "m2", "float", min=0.3, max=6.0, typical=(0.6, 2.8),
          help="Reference area for both coefficients (projected frontal area)."),
    Param("aero.front_fraction", "Front aerodynamic balance (optional)", "-", "float", optional=True, min=-0.5, max=1.5,
          typical=(0.2, 0.6),
          help="Fraction of the downforce carried by the front axle. If empty, the front static weight fraction "
               "is used (neutral balance)."),
    # --- environment -----------------------------------------------------------------------------
    Param("environment.gravity", "Gravity", "m/s2", "float", default=9.81, min=9.5, max=10.1, typical=(9.76, 9.84),
          help="Gravitational acceleration at the circuit. The default is the standard value."),
    Param("environment.air_density", "Air density", "kg/m3", "float", default=1.225, min=0.6, max=1.5, typical=(1.0, 1.3),
          help="Air density. The default is the standard value at sea level and 15 C; it decreases with altitude "
               "and temperature."),
]

PARAMS = {p.key: p for p in REGISTRY}


def params_in(group: str) -> list:
    return [p for p in REGISTRY if p.key.split(".")[0] == group]


# ---------------------------------------------------------------------------------------------
# Helpers over nested dictionaries
# ---------------------------------------------------------------------------------------------


def get_path(data: dict, key: str, default=None):
    node = data
    for part in key.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def set_path(data: dict, key: str, value) -> None:
    parts = key.split(".")
    node = data
    for part in parts[:-1]:
        node = node.setdefault(part, {})
    node[parts[-1]] = value


def is_visible(param: Param, data: dict) -> bool:
    if param.visible_if is None:
        return True
    other, values = param.visible_if
    return get_path(data, other, PARAMS[other].default) in values


def default_environment() -> dict:
    return {"gravity": PARAMS["environment.gravity"].default, "air_density": PARAMS["environment.air_density"].default}


# ---------------------------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------------------------


def _check_number(param: Param, value, issues: list) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        issues.append(Issue("error", param.key, f"{param.label}: a number is required."))
        return
    if value != value or value in (float("inf"), float("-inf")):
        issues.append(Issue("error", param.key, f"{param.label}: the value is not a finite number."))
        return
    if param.min is not None and value < param.min:
        issues.append(Issue("error", param.key, f"{param.label}: {value} {param.unit} is below the minimum allowed ({param.min})."))
    elif param.max is not None and value > param.max:
        issues.append(Issue("error", param.key, f"{param.label}: {value} {param.unit} is above the maximum allowed ({param.max})."))
    elif param.typical is not None and not (param.typical[0] <= value <= param.typical[1]):
        issues.append(Issue("warning", param.key,
                            f"{param.label}: {value} {param.unit} is outside the usual range "
                            f"({param.typical[0]} to {param.typical[1]}). Check it."))


def validate_params(data: dict) -> list:
    """Checks every visible registry entry of a nested dictionary and returns a list of Issue."""
    issues: list = []
    for param in REGISTRY:
        if not is_visible(param, data):
            continue
        value = get_path(data, param.key)
        if value is None:
            value = param.default
        if value is None:
            if not param.optional:
                issues.append(Issue("error", param.key, f"{param.label}: a value is required."))
            continue
        if param.kind == "float":
            _check_number(param, value, issues)
        elif param.kind == "choice":
            if value not in param.choices:
                issues.append(Issue("error", param.key, f"{param.label}: must be one of {', '.join(param.choices)}."))
        elif param.kind == "float_list":
            if not isinstance(value, (list, tuple)) or len(value) == 0:
                issues.append(Issue("error", param.key, f"{param.label}: at least one value is required."))
            else:
                for v in value:
                    _check_number(param, v, issues)
    return issues


def validate_torque_curve(curve: dict) -> list:
    """curve: {"rpm": [...], "torque_nm": [...]}"""
    issues: list = []
    key = "powertrain.torque_curve"
    rpm, torque = (curve or {}).get("rpm"), (curve or {}).get("torque_nm")
    if not rpm or not torque:
        return [Issue("error", key, "Torque curve: rpm and torque values are required.")]
    if len(rpm) != len(torque):
        return [Issue("error", key, "Torque curve: rpm and torque must have the same number of points.")]
    if len(rpm) < 4:
        issues.append(Issue("error", key, "Torque curve: at least 4 points are needed."))
    if any(b <= a for a, b in zip(rpm, rpm[1:])):
        issues.append(Issue("error", key, "Torque curve: rpm values must be strictly increasing."))
    if any(t < 0 for t in torque) or max(torque) <= 0:
        issues.append(Issue("error", key, "Torque curve: torque must be non-negative and not all zero."))
    if not issues:
        if rpm[0] > 4000:
            issues.append(Issue("warning", key, f"Torque curve starts at {rpm[0]} rpm: below it the model keeps the "
                                                f"torque of the first point."))
        if max(torque) > 3000:
            issues.append(Issue("warning", key, "Torque curve: values above 3000 N*m are unusual for one engine. "
                                                "Check the unit (it must be N*m, not lb*ft)."))
    return issues


def validate_vehicle_dict(data: dict) -> list:
    """Full validation of a vehicle dictionary: registry rules plus the checks that involve several data."""
    issues = validate_params(data)
    issues += validate_torque_curve(get_path(data, "powertrain.torque_curve"))
    if any(i.level == "error" for i in issues):
        return issues  # cross-checks need valid numbers

    g = lambda k: get_path(data, k, PARAMS[k].default if k in PARAMS else None)
    ratios = g("powertrain.gear_ratios")
    if any(b >= a for a, b in zip(ratios, ratios[1:])):
        issues.append(Issue("warning", "powertrain.gear_ratios", "Gear ratios are not strictly decreasing from first to last gear."))
    curve = get_path(data, "powertrain.torque_curve")
    if g("powertrain.curve_measured_at") == "wheel_inertial":
        issues.append(Issue("warning", "powertrain.curve_measured_at",
                            "An inertial dyno already subtracts part of the rotating inertia, measured at the sweep "
                            "acceleration. The model also applies the inertia of each gear, so the drive force may be "
                            "slightly underestimated (a few per cent in the highest gears)."))
    mass, h, L = g("general.mass_kg"), g("general.cg_height_m"), g("general.wheelbase_m")
    if h > 0.6 * L:
        issues.append(Issue("warning", "general.cg_height_m", "Centre of gravity height is very large compared with the wheelbase."))
    # tyre load sensitivity: the friction coefficient must stay positive at high load
    if g("tires.mode") == "load_dependent":
        static_front = mass * 9.81 * g("general.front_weight_fraction") / 2 / 1000
        static_rear = mass * 9.81 * (1 - g("general.front_weight_fraction")) / 2 / 1000
        for axle, load in (("front", static_front), ("rear", static_rear)):
            for d in ("x", "y"):
                mu0, c = g(f"tires.{axle}.mu_{d}0"), g(f"tires.{axle}.c_{d}")
                if mu0 - c * 3 * load < 0.3:
                    issues.append(Issue("warning", f"tires.{axle}.c_{d}",
                                        f"With this load sensitivity the {axle} {d} friction coefficient drops below 0.3 "
                                        f"at three times the static load of one tyre. Check mu0 and c."))
    rev = g("powertrain.rev_limit_rpm")
    if rev is not None and rev < curve["rpm"][0]:
        issues.append(Issue("error", "powertrain.rev_limit_rpm", "Rev limit is below the first point of the torque curve."))
    return issues
