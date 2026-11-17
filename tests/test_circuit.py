from pathlib import Path

import numpy as np
import pytest

from vdcalc.circuit import CircuitDataError, build_track, check_circuit, load_circuit_csv, load_track

EXAMPLE = Path(__file__).parent.parent / "data" / "circuits" / "example_circuit.csv"


def circle(radius, n=72, z=None, bank=None, clockwise=False):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False) * (-1 if clockwise else 1)
    return radius * np.cos(a), radius * np.sin(a)


def test_circle_has_constant_curvature_and_the_right_length():
    x, y = circle(80.0)
    t = build_track(x, y, ds=2.0)
    assert t.length == pytest.approx(2 * np.pi * 80.0, rel=1e-3)
    assert np.allclose(t.curvature, 1 / 80.0, rtol=5e-3)
    assert np.allclose(t.slope, 0.0) and np.allclose(t.vertical_curvature, 0.0) and np.allclose(t.bank, 0.0)


def test_clockwise_circle_turns_right():
    x, y = circle(80.0, clockwise=True)
    assert np.all(build_track(x, y).curvature < 0)


def test_spacing_is_uniform_and_the_table_closes():
    x, y = circle(60.0)
    t = build_track(x, y, ds=3.0)
    assert np.allclose(np.diff(t.s), t.ds) and t.ds == pytest.approx(3.0, rel=0.02)
    assert t.s[0] == 0.0 and t.s[-1] + t.ds == pytest.approx(t.length)


def test_constant_slope_and_bank_are_recovered():
    x = np.linspace(0, 1000, 60)
    # an out-and-back line is not a circuit; use a long oval and give it a tilted plane: z = 0.05 * x
    a = np.linspace(0, 2 * np.pi, 120, endpoint=False)
    xs, ys = 300 * np.cos(a), 150 * np.sin(a)
    z = 0.05 * xs                                             # a tilted plane: the slope varies with the heading
    t = build_track(xs, ys, z=z, bank_deg=np.full(120, 4.0))
    assert np.max(np.abs(np.tan(t.slope))) == pytest.approx(0.05, rel=0.02)
    assert np.allclose(np.degrees(t.bank), 4.0, atol=1e-6)


def test_vertical_curvature_of_a_crest():
    a = np.linspace(0, 2 * np.pi, 240, endpoint=False)
    xs, ys = 400 * np.cos(a), 200 * np.sin(a)
    z = 3.0 * np.cos(2 * a)                                   # smooth hills around the oval
    t = build_track(xs, ys, z=z)
    assert t.vertical_curvature.min() < -1e-4 and t.vertical_curvature.max() > 3e-5
    assert t.z.max() == pytest.approx(3.0, abs=0.05)


def test_effective_bank_follows_the_turn_direction():
    a = np.linspace(0, 2 * np.pi, 80, endpoint=False)
    left = build_track(60 * np.cos(a), 60 * np.sin(a), bank_deg=np.full(80, 5.0))
    right = build_track(60 * np.cos(-a), 60 * np.sin(-a), bank_deg=np.full(80, 5.0))
    assert np.all(left.effective_bank > 0)        # tilts left, turning left: helps
    assert np.all(right.effective_bank < 0)       # tilts left, turning right: opposes


def test_reversing_flips_curvature_and_bank_and_keeps_the_start():
    a = np.linspace(0, 2 * np.pi, 80, endpoint=False)
    x, y, bank = 60 * np.cos(a), 60 * np.sin(a), np.full(80, 5.0)
    fwd, back = build_track(x, y, bank_deg=bank), build_track(x, y, bank_deg=bank, reverse=True)
    assert np.all(fwd.curvature > 0) and np.all(back.curvature < 0)
    assert np.allclose(np.degrees(back.bank), -5.0, atol=1e-6)
    assert back.x[0] == pytest.approx(fwd.x[0], abs=0.1) and back.y[0] == pytest.approx(fwd.y[0], abs=0.1)
    assert np.all(back.effective_bank > 0)        # the same banked turn still helps when driven the other way


def test_start_index_moves_the_finish_line():
    a = np.linspace(0, 2 * np.pi, 80, endpoint=False)
    t = build_track(60 * np.cos(a), 60 * np.sin(a), start=20)
    assert (t.x[0], t.y[0]) == pytest.approx((60 * np.cos(a[20]), 60 * np.sin(a[20])), abs=0.1)


def test_invalid_circuits_are_rejected():
    x, y = circle(60.0, n=8)
    with pytest.raises(CircuitDataError):
        build_track(x, y)                                     # too few points
    x, y = circle(60.0)
    x, y = x[:30], y[:30]                                     # half a circle: the last point is far from the first
    assert any("not closed" in i.message for i in check_circuit(x, y))
    xx, yy = circle(60.0)
    xx[5] = xx[4]; yy[5] = yy[4]
    assert any("same position" in i.message for i in check_circuit(xx, yy))
    assert any(i.level == "warning" for i in check_circuit(*circle(60.0), bank_deg=np.full(72, 80.0)))


def test_csv_reader_with_comments_header_and_optional_columns(tmp_path):
    p = tmp_path / "c.csv"
    p.write_text("# my circuit\nx_m,y_m,z_m,bank_deg\n0,0,1,0\n10,0,2,1\n10,10,3,2\n")
    d = load_circuit_csv(p)
    assert d["x"] == [0, 10, 10] and d["z"] == [1, 2, 3] and d["bank_deg"] == [0, 1, 2]
    p2 = tmp_path / "d.csv"
    p2.write_text("0,0\n10,0\n10,10\n")
    d2 = load_circuit_csv(p2)
    assert d2["z"] is None and d2["bank_deg"] is None
    (tmp_path / "e.csv").write_text("# nothing\n")
    with pytest.raises(CircuitDataError):
        load_circuit_csv(tmp_path / "e.csv")


def test_example_circuit_loads():
    t = load_track(EXAMPLE, ds=2.0)
    assert t.length == pytest.approx(4298, abs=15)
    assert -9.5 < t.z.min() < -8 and 17 < t.z.max() < 19
    assert np.degrees(t.bank).min() < -6 and np.degrees(t.bank).max() > 2
    assert 25 < 1 / np.abs(t.curvature).max() < 32                                  # the hairpin
    assert (t.curvature < 0).sum() > (t.curvature > 0).sum() > 0                    # clockwise, with left-hand corners too
    assert t.vertical_curvature.min() == pytest.approx(-0.0048, rel=0.25)         # the jump crest
    assert np.max(np.abs(np.tan(t.slope))) < 0.09


def test_example_circuit_is_smooth():
    """The example is smooth by construction: curvature, bank and height change gradually (no noise, no steps)."""
    t = load_track(EXAMPLE, ds=2.0)
    assert np.max(np.abs(np.diff(t.curvature) / np.diff(t.s))) < 0.0012            # radius changes slowly (clothoid-like)
    assert np.max(np.abs(np.diff(np.degrees(t.bank)) / np.diff(t.s))) < 0.15       # bank ramps over tens of metres
    away_from_crest = np.abs(t.s - 1246) > 100
    assert np.max(np.abs(t.vertical_curvature[away_from_crest])) < 0.0012          # height changes gently
    data = load_circuit_csv(EXAMPLE)
    gaps = np.hypot(np.diff(np.append(data["x"], data["x"][0])), np.diff(np.append(data["y"], data["y"][0])))
    assert gaps.min() > 4.5 and gaps.max() < 5.5                                    # the loop closes with the same spacing
    heading_start = np.arctan2(*np.diff([data["y"][0], data["y"][1]]), *np.diff([data["x"][0], data["x"][1]]))
    heading_end = np.arctan2(data["y"][0] - data["y"][-1], data["x"][0] - data["x"][-1])
    assert abs(heading_end - heading_start) < 0.02                                  # no kink at the finish line
