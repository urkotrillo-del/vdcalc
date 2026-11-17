"""'What if' study: how much the lap time changes when each datum changes by a fixed percentage.

For every variation the lap is solved with the datum reduced and increased, and the effect is reported in seconds.
The result is sorted by how much each datum matters on this circuit, which tells where an accurate measurement
(or a design change) pays off most.
"""
from dataclasses import dataclass
from typing import Callable

from .lap import solve_lap
from .registry import get_path, set_path
from .vehicle import VehicleDataError, vehicle_from_dict, vehicle_to_dict


@dataclass(frozen=True)
class Variation:
    key: str
    label: str
    relative: float                       # default size of the change: 0.05 = plus and minus 5 %
    apply: Callable                       # apply(vehicle_dict, factor) modifies the dictionary
    relevant: Callable = lambda d: True   # does it make sense for this vehicle?


def _scale(path):
    def apply(data, factor):
        value = get_path(data, path)
        if value is not None:
            set_path(data, path, value * factor)
    return apply


def _scale_torque(data, factor):
    curve = get_path(data, "powertrain.torque_curve")
    set_path(data, "powertrain.torque_curve", {"rpm": list(curve["rpm"]), "torque_nm": [t * factor for t in curve["torque_nm"]]})


def _scale_grip(data, factor):
    """Scales the friction of all tyres. With load-dependent tyres mu0 and c scale together, so mu scales exactly."""
    for axle in ("front", "rear"):
        for key in ("mu_x", "mu_y", "mu_x0", "mu_y0", "c_x", "c_y"):
            path = f"tires.{axle}.{key}"
            if get_path(data, path) is not None:
                set_path(data, path, get_path(data, path) * factor)


def _scale_track(data, factor):
    for path in ("general.front_track_m", "general.rear_track_m"):
        value = get_path(data, path)
        if value is not None:
            set_path(data, path, value * factor)


DEFAULT_VARIATIONS = (
    Variation("mass", "Mass", 0.05, _scale("general.mass_kg")),
    Variation("power", "Engine power", 0.05, _scale_torque),
    Variation("drag", "Drag coefficient (Cd)", 0.05, _scale("aero.cd")),
    Variation("downforce", "Downforce (Cl)", 0.05, _scale("aero.cl"), relevant=lambda d: abs(get_path(d, "aero.cl") or 0.0) > 1e-9),
    Variation("grip", "Tyre grip (all tyres)", 0.05, _scale_grip),
    Variation("cg_height", "Centre of gravity height", 0.05, _scale("general.cg_height_m")),
    Variation("front_weight", "Front weight fraction", 0.05, _scale("general.front_weight_fraction")),
    Variation("lltd", "Front lateral load transfer share", 0.05, _scale("general.lltd_front"),
              relevant=lambda d: get_path(d, "general.lltd_front") is not None),
    Variation("track", "Track width (both axles)", 0.05, _scale_track),
    Variation("brake_bias", "Front brake force share", 0.05, _scale("general.brake_front_fraction")),
    Variation("final_drive", "Final drive ratio", 0.05, _scale("powertrain.final_drive")),
    Variation("rolling", "Rolling resistance", 0.05, _scale("tires.rolling_coefficient")),
)


def _lap_time(data, env, track):
    return solve_lap(vehicle_from_dict(data), env, track).lap_time


def run_study(vehicle, env, track, variations=None, relative=None, progress=None, baseline=None) -> dict:
    """Runs the study.

    variations: list of Variation (default: DEFAULT_VARIATIONS). relative: size of the change for all of them, if given.
    progress: optional callback progress(done, total), called after each lap, for a progress bar.
    baseline: lap time of the unchanged vehicle, if already known (saves one lap).

    Returns {"baseline_lap_time_s", "rows": [...], "skipped": [...]}. Each row has the lap time with the datum reduced
    and increased, the differences against the baseline [s], and the effect per 1 % of change [s/%]. Rows are sorted by
    the size of the effect. A variation that makes the data invalid is skipped and reported.
    """
    base_data = vehicle_to_dict(vehicle)
    chosen = [v for v in (variations or DEFAULT_VARIATIONS) if v.relevant(base_data)]
    total = 2 * len(chosen) + (0 if baseline is not None else 1)
    done = 0

    def tick():
        nonlocal done
        done += 1
        if progress:
            progress(done, total)

    if baseline is None:
        baseline = _lap_time(base_data, env, track)
        tick()
    rows, skipped = [], []
    for var in chosen:
        size = var.relative if relative is None else relative
        times = {}
        failure = None
        for sign, name in ((-1, "minus"), (+1, "plus")):
            if failure is None:
                data = vehicle_to_dict(vehicle)
                var.apply(data, 1.0 + sign * size)
                try:
                    times[name] = _lap_time(data, env, track)
                except VehicleDataError as e:
                    failure = str(e)
            tick()                                           # a skipped lap still counts for the progress bar
        if failure is not None:
            skipped.append({"key": var.key, "label": var.label, "reason": failure})
            continue
        d_minus, d_plus = times["minus"] - baseline, times["plus"] - baseline
        rows.append({"key": var.key, "label": var.label, "relative": size, "lap_time_minus_s": times["minus"],
                     "lap_time_plus_s": times["plus"], "delta_minus_s": d_minus, "delta_plus_s": d_plus,
                     "effect_per_percent_s": (times["plus"] - times["minus"]) / (2.0 * size * 100.0)})
    rows.sort(key=lambda r: -max(abs(r["delta_minus_s"]), abs(r["delta_plus_s"])))
    return {"baseline_lap_time_s": baseline, "rows": rows, "skipped": skipped}
