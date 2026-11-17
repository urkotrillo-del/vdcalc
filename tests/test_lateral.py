import math

import numpy as np
import pytest
from scipy.optimize import brentq

from conftest import build
from vdcalc.lateral import (corner_speed, gg_envelope, lateral_force_capacity, lateral_state, lateral_usage,
                            max_lateral_acceleration, remaining_fraction)
from vdcalc.loads import Conditions

def loaded_car(**extra):
    return build({"tires.mode": "load_dependent",
                  "tires.front": {"mu_x0": 1.5, "c_x": 0.12, "mu_y0": 1.4, "c_y": 0.12},
                  "tires.rear": {"mu_x0": 1.6, "c_x": 0.10, "mu_y0": 1.5, "c_y": 0.10}, **extra})


def test_flat_corner_speed_is_sqrt_mu_g_r(car, env):
    for radius in (30.0, 60.0, 200.0):
        assert float(corner_speed(car, env, 1 / radius)) == pytest.approx(math.sqrt(0.89 * env.gravity * radius), rel=1e-9)


def test_banked_corner_matches_the_textbook_formula(car, env):
    mu, radius = 0.89, 80.0
    for deg in (3.0, 6.0, 10.0):
        t = math.tan(math.radians(deg))
        favourable = math.sqrt(env.gravity * radius * (mu + t) / (1 - mu * t))
        adverse = math.sqrt(env.gravity * radius * (mu - t) / (1 + mu * t))
        assert float(corner_speed(car, env, 1 / radius, math.radians(deg))) == pytest.approx(favourable, rel=1e-6)
        assert float(corner_speed(car, env, 1 / radius, -math.radians(deg))) == pytest.approx(adverse, rel=1e-6)


def test_the_car_cannot_stand_on_a_bank_steeper_than_its_grip(car, env):
    assert float(corner_speed(car, env, 1 / 50.0, -math.radians(60))) == 0.0


def test_no_curvature_means_no_lateral_limit(car, env):
    assert float(corner_speed(car, env, 0.0, v_max=100.0)) == 100.0
    assert float(corner_speed(car, env, -1 / 60.0)) == pytest.approx(float(corner_speed(car, env, 1 / 60.0)))


def test_corner_speed_accepts_arrays(car, env):
    k = np.array([1 / 40, 1 / 80, 0.0, -1 / 120])
    v = corner_speed(car, env, k, np.array([0.0, 0.05, 0.0, 0.0]))
    assert v.shape == (4,) and v[0] < v[1] and v[2] == 120.0 and v[3] > v[0]


def brute_capacity(car, env, v, kappa, bank):
    """Independent solution of each axle equation by root finding, tyre by tyre."""
    n_f, n_r, _ = lateral_state(car, env, v, kappa, bank)
    g, t = car.general, car.tires
    lltd = g.front_weight_fraction if g.lltd_front is None else g.lltd_front

    def tyre(z, mu0, c):                                   # force of one tyre with load z [N]
        return max(mu0 - c * z / 1000.0, 0.0) * z

    def axle(n_axle, mu0, c, share, transfer, track):
        n_half = n_axle / 2.0

        def residual(fy):
            d = min(transfer * fy * g.cg_height_m / track, n_half)
            return tyre(n_half + d, mu0, c) + tyre(n_half - d, mu0, c) - share * fy
        return brentq(residual, 0.0, 1e6)

    wf = g.front_weight_fraction
    f = axle(float(n_f), t.front.mu_y0, t.front.c_y, wf, lltd, g.front_track_m)
    r = axle(float(n_r), t.rear.mu_y0, t.rear.c_y, 1 - wf, 1 - lltd, g.rear_track_m)
    return min(f, r)


@pytest.mark.parametrize("lltd", [None, 0.4, 0.7, 0.95])
@pytest.mark.parametrize("v, kappa, bank", [(10.0, 0.01, 0.0), (35.0, 0.02, 0.05), (60.0, 0.005, -0.04)])
def test_load_dependent_capacity_matches_brute_force(env, lltd, v, kappa, bank):
    car = loaded_car(**{"general.lltd_front": lltd, "aero.cl": -1.5})
    assert float(lateral_force_capacity(car, env, v, kappa, bank)) == pytest.approx(brute_capacity(car, env, v, kappa, bank), rel=1e-8)


def test_capacity_also_matches_when_the_inner_tyre_lifts(env):
    car = loaded_car(**{"general.cg_height_m": 0.85, "general.front_track_m": 1.2, "general.rear_track_m": 1.2,
                        "general.lltd_front": 0.9, "tires.rolling_coefficient": 0.012})
    assert float(lateral_force_capacity(car, env, 20.0, 0.01)) == pytest.approx(brute_capacity(car, env, 20.0, 0.01, 0.0), rel=1e-8)


def test_load_transfer_only_matters_with_load_sensitive_tyres(car, env):
    base = float(lateral_force_capacity(car, env, 30.0, 0.01))
    for lltd in (0.3, 0.6, 0.9):
        c = build({"general.lltd_front": lltd})
        assert float(lateral_force_capacity(c, env, 30.0, 0.01)) == pytest.approx(base, rel=1e-12)   # constant mu: no effect
    values = [float(lateral_force_capacity(loaded_car(**{"general.lltd_front": x}), env, 30.0, 0.01)) for x in (0.3, 0.5, 0.7, 0.9)]
    assert max(values) - min(values) > 50.0                                                              # load sensitivity: it matters


def test_load_sensitivity_lowers_the_grip_on_the_loaded_outer_tyre(env):
    flat_tyres = build({})
    loaded = loaded_car(**{"tires.front": {"mu_x0": 1.02, "c_x": 0.0, "mu_y0": 0.89, "c_y": 0.0},
                           "tires.rear": {"mu_x0": 1.02, "c_x": 0.0, "mu_y0": 0.89, "c_y": 0.0}})
    assert float(lateral_force_capacity(loaded, env, 30.0, 0.01)) == pytest.approx(float(lateral_force_capacity(flat_tyres, env, 30.0, 0.01)))
    steeper = loaded_car(**{"tires.front": {"mu_x0": 1.3, "c_x": 0.0, "mu_y0": 1.2, "c_y": 0.15},
                            "tires.rear": {"mu_x0": 1.3, "c_x": 0.0, "mu_y0": 1.2, "c_y": 0.15}})
    # same mu at the static load, but with transfer the load-sensitive car gives less than mu * N
    static_mu_n = 0.89 * 1138 * env.gravity
    assert float(lateral_force_capacity(steeper, env, 0.0, 0.0)) < 1.2 * 1138 * env.gravity


def test_downforce_raises_the_corner_speed_and_a_crest_lowers_it(car, env):
    wing = build({"aero.cl": -2.5})
    assert float(corner_speed(wing, env, 1 / 150.0)) > float(corner_speed(car, env, 1 / 150.0))
    crest, dip = Conditions(vertical_curvature=-0.002), Conditions(vertical_curvature=0.002)
    base = float(corner_speed(car, env, 1 / 150.0))
    assert float(corner_speed(car, env, 1 / 150.0, cond=crest)) < base < float(corner_speed(car, env, 1 / 150.0, cond=dip))


def test_usage_is_one_at_the_corner_speed(car, env):
    k = 1 / 70.0
    v = float(corner_speed(car, env, k))
    assert float(lateral_usage(car, env, v, k)) == pytest.approx(1.0, abs=1e-6)
    assert float(lateral_usage(car, env, 0.8 * v, k)) < 1.0 < float(lateral_usage(car, env, 1.1 * v, k))
    assert float(lateral_usage(car, env, 30.0, 0.0)) == 0.0


def test_friction_ellipse_fraction():
    assert float(remaining_fraction(0.0)) == 1.0 and float(remaining_fraction(0.6)) == pytest.approx(0.8)
    assert float(remaining_fraction(1.0)) == 0.0 and float(remaining_fraction(1.4)) == 0.0


def test_max_lateral_acceleration_matches_mu_g_and_grows_with_downforce(car, env):
    assert float(max_lateral_acceleration(car, env, 20.0)) == pytest.approx(0.89 * env.gravity)
    wing = build({"aero.cl": -3.0})
    assert float(max_lateral_acceleration(wing, env, 60.0)) > float(max_lateral_acceleration(wing, env, 20.0)) > 0.89 * env.gravity


def test_gg_diagram_shape(car, env):
    ay, ax_acc, ax_brk = gg_envelope(car, env, 25.0, n=41)
    assert ay[0] == pytest.approx(-ay[-1]) and np.allclose(ax_acc, ax_acc[::-1]) and np.allclose(ax_brk, ax_brk[::-1])
    mid = len(ay) // 2
    assert ax_acc[mid] == pytest.approx(ax_acc.max()) and ax_brk[mid] == pytest.approx(ax_brk.max())
    assert ax_brk[mid] > 8.0                                          # about 1 g of braking in a straight line
    assert ax_brk[0] < 0.5 * ax_brk[mid]                              # at the lateral limit almost nothing is left to brake
    assert np.all(np.diff(ax_brk[mid:]) <= 1e-9) and np.all(np.diff(ax_acc[mid:]) <= 1e-9)


def test_a_straight_has_no_lateral_limit_even_over_a_crest(car, env):
    crest = Conditions(vertical_curvature=-0.005)
    assert float(corner_speed(car, env, 5e-5, 0.0, crest, v_max=100.0)) == 100.0      # numerical noise, not a corner
    assert float(corner_speed(car, env, 0.0, 0.0, crest, v_max=100.0)) == 100.0
    assert float(corner_speed(car, env, 1 / 300.0, 0.0, crest, v_max=100.0)) < 100.0  # a real corner over a crest is limited
