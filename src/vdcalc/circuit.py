"""Circuit: a closed line described by points, turned into a table of curvature, slope and bank.

File format (CSV, comment lines start with #):  x_m, y_m [, z_m [, bank_deg]]
  - x_m, y_m: position of the centre line in metres, in the order of travel. The first point is the finish line.
  - z_m: height in metres (optional, 0 if absent).
  - bank_deg: lateral inclination of the road in degrees (optional, 0 if absent). Positive: the road tilts to the
    LEFT of the direction of travel (the left side is lower), so it helps left turns and opposes right turns.
The line is closed: the last point connects with the first one. Do not repeat the first point at the end.

The distance s is measured along the horizontal projection of the line. With road slopes below 10 % the difference
with the real road length is under 0.5 %, which is neglected (declared simplification).
"""
import csv
import io
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline

from .registry import Issue


class CircuitDataError(ValueError):
    def __init__(self, issues):
        self.issues = issues
        super().__init__("; ".join(i.message for i in issues if i.level == "error"))


@dataclass(frozen=True)
class Track:
    s: np.ndarray                   # position along the line [m], from the finish line
    x: np.ndarray                   # [m]
    y: np.ndarray                   # [m]
    z: np.ndarray                   # height [m]
    curvature: np.ndarray           # signed horizontal curvature [1/m]; positive = turning left
    slope: np.ndarray               # road angle [rad]; positive uphill
    vertical_curvature: np.ndarray  # d(slope)/ds [1/m]; negative over a crest, positive in a compression
    bank: np.ndarray                # bank angle [rad]; positive = road tilts left
    length: float                   # total length [m]

    @property
    def ds(self) -> float:
        return float(self.length / len(self.s))

    @property
    def effective_bank(self) -> np.ndarray:
        """Bank as seen by the turn: positive when it helps the turn (inside lower), whichever the direction.

        Zero on straights (curvature below 1e-4 1/m), where the direction of the turn is not defined."""
        return np.where(np.abs(self.curvature) < 1e-4, 0.0, self.bank * np.sign(self.curvature))


def _moving_average(values: np.ndarray, width: int) -> np.ndarray:
    """Periodic moving average over `width` samples (odd)."""
    if width <= 1:
        return values
    pad = width // 2
    wrapped = np.concatenate([values[-pad:], values, values[:pad]])
    return np.convolve(wrapped, np.ones(width) / width, mode="valid")


def parse_circuit_text(text: str) -> dict:
    """Reads the text of a circuit file. Returns {"x": [...], "y": [...], "z": [...] or None, "bank_deg": [...] or None}."""
    rows = []
    for row in csv.reader(io.StringIO(text)):
        if not row or row[0].strip().startswith("#"):
            continue
        try:
            rows.append([float(c) for c in row if c.strip() != ""])
        except ValueError:
            continue                                           # header line
    if not rows:
        raise CircuitDataError([Issue("error", "file", "The circuit file has no numeric rows (expected x_m, y_m[, z_m[, bank_deg]]).")])
    width = min(len(r) for r in rows)
    if width < 2:
        raise CircuitDataError([Issue("error", "file", "Every row needs at least two numbers: x_m and y_m.")])
    a = np.array([r[:width] for r in rows])
    return {"x": a[:, 0].tolist(), "y": a[:, 1].tolist(), "z": a[:, 2].tolist() if width > 2 else None,
            "bank_deg": a[:, 3].tolist() if width > 3 else None}


def load_circuit_csv(path) -> dict:
    """Reads the circuit file (see parse_circuit_text)."""
    with open(path, newline="", encoding="utf-8-sig") as f:
        return parse_circuit_text(f.read())


def check_circuit(x, y, z=None, bank_deg=None) -> list:
    """Problems found in the raw points: errors block the use of the circuit, warnings do not."""
    issues = []
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    if len(x) < 12:
        return [Issue("error", "points", "A circuit needs at least 12 points.")]
    if not (np.all(np.isfinite(x)) and np.all(np.isfinite(y))):
        return [Issue("error", "points", "The coordinates contain values that are not numbers.")]
    gaps = np.hypot(np.diff(np.append(x, x[0])), np.diff(np.append(y, y[0])))
    median = float(np.median(gaps))
    if median <= 0 or np.any(gaps[:-1] < 1e-6):
        issues.append(Issue("error", "points", "Two consecutive points are at the same position."))
    if gaps[-1] > 10 * median:
        issues.append(Issue("error", "points", "The circuit is not closed: the last point is far from the first one."))
    if np.max(gaps) > 30 * median:
        issues.append(Issue("warning", "points", "The spacing of the points is very irregular."))
    if z is not None and abs(z[-1] - z[0]) > 2.0 + 0.1 * np.sum(gaps) / 1000:
        issues.append(Issue("warning", "z_m", "The height at the end of the lap differs from the height at the start."))
    if bank_deg is not None and np.max(np.abs(bank_deg)) > 45:
        issues.append(Issue("warning", "bank_deg", "Bank angles above 45 degrees are unusual. Check the unit (degrees)."))
    return issues


def build_track(x, y, z=None, bank_deg=None, ds=2.0, reverse=False, start=0, smooth_m=8.0) -> Track:
    """Builds the table of the circuit with points every `ds` metres.

    reverse: drive the circuit the other way round (the bank is relative to the direction of travel, so its sign flips).
    start: index of the point that becomes the finish line (applied before reversing).
    smooth_m: the curvature (horizontal and vertical) is averaged over this distance [m]. A car does not respond to
    curvature peaks narrower than a few metres, and the spline through the points overshoots where a straight meets
    an arc (and amplifies the noise of measured data). 0 disables the averaging.
    """
    issues = check_circuit(x, y, z, bank_deg)
    if any(i.level == "error" for i in issues):
        raise CircuitDataError(issues)
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    n = len(x)
    z = np.zeros(n) if z is None else np.asarray(z, dtype=float)
    bank = np.zeros(n) if bank_deg is None else np.asarray(bank_deg, dtype=float)
    idx = (np.arange(n) + start) % n
    x, y, z, bank = x[idx], y[idx], z[idx], bank[idx]
    if reverse:
        x, y, z, bank = x[::-1], y[::-1], z[::-1], -bank[::-1]
        x, y, z, bank = np.roll(x, 1), np.roll(y, 1), np.roll(z, 1), np.roll(bank, 1)   # keep the same finish point

    closed = np.column_stack([np.append(x, x[0]), np.append(y, y[0]), np.append(z, z[0])])
    chord = np.hypot(*np.diff(closed[:, :2], axis=0).T)
    t = np.concatenate([[0.0], np.cumsum(chord)])
    spline = CubicSpline(t, closed, bc_type="periodic")

    # arc length of the horizontal projection, from a dense sampling of the spline
    t_dense = np.linspace(0.0, t[-1], 40001)
    d1 = spline(t_dense, 1)
    h_speed = np.hypot(d1[:, 0], d1[:, 1])
    s_dense = np.concatenate([[0.0], np.cumsum(0.5 * (h_speed[1:] + h_speed[:-1]) * np.diff(t_dense))])
    length = float(s_dense[-1])
    m = max(int(round(length / ds)), 8)
    s = np.arange(m) * length / m
    tt = np.interp(s, s_dense, t_dense)
    d1, d2 = spline(tt, 1), spline(tt, 2)
    h1 = np.hypot(d1[:, 0], d1[:, 1])
    curvature = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / h1**3
    dz_ds = d1[:, 2] / h1
    # vertical curvature: derivative of the slope along s (central difference, the loop is periodic)
    step = length / m
    kv = (np.roll(dz_ds, -1) - np.roll(dz_ds, 1)) / (2.0 * step) / (1.0 + dz_ds**2) ** 1.5
    width = max(int(round(smooth_m / step)), 1)
    width += 1 - width % 2                                   # odd
    curvature, kv = _moving_average(curvature, width), _moving_average(kv, width)
    # bank: linear interpolation along s (periodic)
    s_pts = np.interp(t[:-1], t_dense, s_dense)
    bank_rad = np.radians(np.interp(s, np.append(s_pts, length), np.append(bank, bank[0])))
    pos = spline(tt)
    return Track(s=s, x=pos[:, 0], y=pos[:, 1], z=pos[:, 2], curvature=curvature, slope=np.arctan(dz_ds),
                 vertical_curvature=kv, bank=bank_rad, length=length)


def load_track(path, ds=2.0, reverse=False, start=0, smooth_m=8.0) -> Track:
    data = load_circuit_csv(path)
    return build_track(data["x"], data["y"], data["z"], data["bank_deg"], ds=ds, reverse=reverse, start=start,
                       smooth_m=smooth_m)
