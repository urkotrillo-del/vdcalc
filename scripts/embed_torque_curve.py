"""Replaces the torque curve inside a vehicle JSON file with the one of a CSV file.

Usage:
    python scripts/embed_torque_curve.py data/examples/mx5_nd2.json path/to/curve.csv [measured_at]

The CSV has two columns: rpm and torque in N*m (comment lines start with #, a header line is allowed).
"""
import json
import sys
from pathlib import Path

from vdcalc.vehicle import load_torque_curve_csv

if len(sys.argv) < 3:
    sys.exit(__doc__)
car_path, csv_path = Path(sys.argv[1]), sys.argv[2]
car = json.loads(car_path.read_text(encoding="utf-8"))
car["powertrain"]["torque_curve"] = load_torque_curve_csv(csv_path)
if len(sys.argv) > 3:
    car["powertrain"]["curve_measured_at"] = sys.argv[3]
car_path.write_text(json.dumps(car, indent=2), encoding="utf-8")
n = len(car["powertrain"]["torque_curve"]["rpm"])
print(f"{car_path}: torque curve replaced ({n} points).")
