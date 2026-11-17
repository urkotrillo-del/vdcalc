import math

import numpy as np
import pytest

from conftest import build
from vdcalc.powertrain import TorqueCurve, drive_force, effective_mass, rev_limit, rpm_from_speed, total_ratio


def test_effective_mass_matches_the_hand_calculation():
    # M = 1100, Iw = 4, Ie = 0.15, r = 0.31, efficiency 1: first gear (13) adds ~306 kg, a tall gear (3.5) adds ~61 kg
    car = build({"general.mass_kg": 1100.0, "powertrain.gear_ratios": [13.0, 3.5], "powertrain.final_drive": 1.0,
                 "powertrain.wheels_inertia_kgm2": 4.0, "powertrain.engine_inertia_kgm2": 0.15,
                 "powertrain.drivetrain_efficiency": 1.0, "tires.wheel_radius_m": 0.31})
    assert effective_mass(car, 0) - 1100.0 == pytest.approx(306.0, abs=1.0)
    assert effective_mass(car, 1) - 1100.0 == pytest.approx(61.0, abs=1.0)


def test_effective_mass_is_larger_in_lower_gears(car):
    masses = [effective_mass(car, g) for g in range(len(car.powertrain.gear_ratios))]
    assert all(a > b for a, b in zip(masses, masses[1:])) and masses[-1] > car.general.mass_kg


def test_rpm_from_speed(car):
    v = 100 / 3.6
    expected = v * total_ratio(car, 2) / car.tires.wheel_radius_m * 60 / (2 * math.pi)
    assert float(rpm_from_speed(car, v, 2)) == pytest.approx(expected)


def test_drive_force_applies_the_efficiency_only_to_a_crank_curve():
    crank = build({"powertrain.curve_measured_at": "crank"})
    wheel = build({"powertrain.curve_measured_at": "wheel_brake"})
    v = 20.0
    assert float(drive_force(crank, v, 2)) == pytest.approx(float(drive_force(wheel, v, 2)) * crank.powertrain.drivetrain_efficiency)


def test_drive_force_is_nan_above_the_rev_limit(car):
    top = rev_limit(car)
    v_over = (top * 1.05) * 2 * math.pi / 60 * car.tires.wheel_radius_m / total_ratio(car, 0)
    assert np.isnan(drive_force(car, v_over, 0))


def test_user_rev_limit_below_the_curve_end_is_used():
    car = build({"powertrain.rev_limit_rpm": 6500.0})
    assert rev_limit(car) == 6500.0


def test_launch_gives_peak_torque_below_the_curve_start(car):
    free = float(drive_force(car, 0.5, 0))
    launch = float(drive_force(car, 0.5, 0, launch=True))
    peak = car.powertrain.torque_curve.peak_torque()[0]
    assert launch > free
    assert launch == pytest.approx(peak * total_ratio(car, 0) / car.tires.wheel_radius_m)
    assert float(drive_force(car, 0.5, 1, launch=True)) == pytest.approx(float(drive_force(car, 0.5, 1)))  # first gear only


def test_torque_curve_holds_below_and_stops_above():
    curve = TorqueCurve([3000, 4000, 5000, 6000], [150, 180, 190, 170])
    assert float(curve.torque(1000)) == 150.0
    assert np.isnan(curve.torque(7000))
    assert curve.torque(4000) == pytest.approx(180.0)


def test_peaks_of_the_curve():
    curve = TorqueCurve([3000, 4000, 5000, 6000], [150, 180, 190, 170])
    torque, rpm = curve.peak_torque()
    power, rpm_p = curve.peak_power()
    assert torque == pytest.approx(190.0, abs=1.0) and 4500 < rpm < 5500
    point_powers = [150 * 3000, 180 * 4000, 190 * 5000, 170 * 6000]
    assert power >= max(point_powers) * 2 * math.pi / 60 * 0.999 and rpm_p >= rpm   # peak power is above the peak torque


def test_invalid_measured_at_is_rejected():
    with pytest.raises(ValueError):
        TorqueCurve([1000, 2000], [100, 110], measured_at="other")
