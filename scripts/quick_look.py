"""Quick look at a vehicle on a circuit: key figures, lap time and charts.

Usage:
    python scripts/quick_look.py [vehicle.json] [circuit.csv] [--save picture.png]

Without arguments it uses the example vehicle and the example circuit. The charts need matplotlib
(pip install matplotlib); without it only the text is printed.
"""
import sys
from pathlib import Path

import numpy as np

from vdcalc.circuit import load_track
from vdcalc.environment import Environment
from vdcalc.lap import solve_lap
from vdcalc.straightline import performance_summary
from vdcalc.units import format_laptime, ms_to_kmh
from vdcalc.vehicle import load_vehicle

ROOT = Path(__file__).parent.parent
args = [a for a in sys.argv[1:] if not a.startswith("--")]
save = sys.argv[sys.argv.index("--save") + 1] if "--save" in sys.argv else None
if save and save in args:
    args.remove(save)
car = load_vehicle(args[0] if args else ROOT / "data" / "examples" / "mx5_nd2.json")
circuit = args[1] if len(args) > 1 else ROOT / "data" / "circuits" / "example_circuit.csv"
env = Environment()

s = performance_summary(car, env)
print(f"\n{car.name}")
for label, key in (("0-60 mph", "0_60_mph"), ("0-100 km/h", "0_100_kmh"), ("0-200 km/h", "0_200_kmh")):
    v = s[key]
    print(f"  {label:11s}", "not reachable" if v is None else f"{v['time_s']:.2f} s  ({v['distance_m']:.0f} m)")
for label, key in (("70-0 mph", "70_0_mph"), ("100-0 km/h", "100_0_kmh"), ("200-0 km/h", "200_0_kmh")):
    v = s[key]
    print(f"  {label:11s}", "-" if v is None else f"{v['distance_m']:.1f} m")
print(f"  top speed   {ms_to_kmh(s['top_speed_ms']):.0f} km/h (limited by {s['top_speed_limited_by']})")

track = load_track(circuit, ds=2.0)
lap = solve_lap(car, env, track)
print(f"\nCircuit: {Path(circuit).name}, {track.length:.0f} m")
print(f"  lap time    {format_laptime(lap.lap_time)}")
print(f"  speed       min {ms_to_kmh(lap.speed.min()):.0f}, max {ms_to_kmh(lap.speed.max()):.0f}, average {ms_to_kmh(track.length / lap.lap_time):.0f} km/h")

try:
    import matplotlib
    if save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
except ImportError:
    sys.exit("\n(install matplotlib to see the charts)")

fig, axes = plt.subplots(2, 2, figsize=(14, 9))
ax = axes[0, 0]
pts = np.column_stack([track.x, track.y]).reshape(-1, 1, 2)
lc = LineCollection(np.concatenate([pts[:-1], pts[1:]], axis=1), array=ms_to_kmh(lap.speed[:-1]), cmap="turbo", linewidths=4)
ax.add_collection(lc); ax.autoscale(); ax.set_aspect("equal"); ax.set_title("Speed [km/h]"); fig.colorbar(lc, ax=ax, shrink=0.7)
ax.plot(track.x[0], track.y[0], "k^", ms=10)
ax = axes[0, 1]
ax.plot(lap.position, ms_to_kmh(lap.speed), lw=2, label="speed"); ax.plot(lap.position, ms_to_kmh(np.minimum(lap.limit, 1.25 * lap.speed.max())), "--", lw=1, label="cornering limit")
ax.set_ylim(0, ms_to_kmh(lap.speed.max()) * 1.15); ax.set_xlabel("distance [m]"); ax.set_ylabel("km/h"); ax.grid(True); ax.legend(); ax.set_title("Speed profile")
ax = axes[1, 0]
ax.plot(lap.position, lap.long_accel / 9.81, label="longitudinal"); ax.plot(lap.position, np.abs(lap.lat_accel) / 9.81, label="lateral (abs)")
ax.set_xlabel("distance [m]"); ax.set_ylabel("g"); ax.grid(True); ax.legend(); ax.set_title("Accelerations")
ax = axes[1, 1]
ax.plot(track.s, track.z, label="height [m]"); ax2 = ax.twinx(); ax2.plot(track.s, np.degrees(track.bank), "tab:orange", label="bank [deg]")
ax.set_xlabel("distance [m]"); ax.set_ylabel("m"); ax2.set_ylabel("deg"); ax.grid(True); ax.set_title("Height and bank")
fig.tight_layout()
if save:
    fig.savefig(save, dpi=80)
    print(f"\nChart saved to {save}")
else:
    plt.show()
