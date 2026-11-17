"""Powertrain: torque curve, gearing, drive force and effective mass."""
import math

import numpy as np
from scipy.interpolate import PchipInterpolator

MEASURED_AT = ("crank", "wheel_brake", "wheel_inertial")


class TorqueCurve:
    """Torque against engine speed, interpolated monotonically (PCHIP).

    Below the first point the torque is held at the value of that point (a declared hypothesis: there is
    usually no data there). Above the last point the engine cannot run: the torque is NaN.
    """

    def __init__(self, rpm, torque_nm, measured_at="crank"):
        self.rpm = np.asarray(rpm, dtype=float)
        self.torque_nm = np.asarray(torque_nm, dtype=float)
        if measured_at not in MEASURED_AT:
            raise ValueError(f"measured_at must be one of {MEASURED_AT}")
        self.measured_at = measured_at
        self._f = PchipInterpolator(self.rpm, self.torque_nm, extrapolate=False)

    @property
    def rpm_min(self) -> float:
        return float(self.rpm[0])

    @property
    def rpm_max(self) -> float:
        return float(self.rpm[-1])

    @property
    def applies_efficiency(self) -> bool:
        """True if the curve is at the crank: the drivetrain losses still have to be applied."""
        return self.measured_at == "crank"

    def torque(self, rpm):
        rpm = np.asarray(rpm, dtype=float)
        return self._f(np.maximum(rpm, self.rpm_min))

    def power_w(self, rpm):
        return self.torque(rpm) * np.asarray(rpm, dtype=float) * 2.0 * math.pi / 60.0

    def peak_torque(self):
        """(torque [N*m], rpm) of the highest point of the interpolated curve."""
        grid = np.linspace(self.rpm_min, self.rpm_max, 2000)
        t = self._f(grid)
        i = int(np.argmax(t))
        return float(t[i]), float(grid[i])

    def peak_power(self):
        """(power [W], rpm) of the highest point of the interpolated curve."""
        grid = np.linspace(self.rpm_min, self.rpm_max, 2000)
        p = self.power_w(grid)
        i = int(np.argmax(p))
        return float(p[i]), float(grid[i])


def rev_limit(vehicle) -> float:
    """Highest engine speed the model allows [rpm]."""
    pt = vehicle.powertrain
    if pt.rev_limit_rpm is None:
        return pt.torque_curve.rpm_max
    return min(pt.rev_limit_rpm, pt.torque_curve.rpm_max)


def total_ratio(vehicle, gear_index: int) -> float:
    return vehicle.powertrain.gear_ratios[gear_index] * vehicle.powertrain.final_drive


def rpm_from_speed(vehicle, speed, gear_index: int):
    """Engine speed [rpm] for a vehicle speed [m/s] in a gear."""
    return np.asarray(speed, dtype=float) * total_ratio(vehicle, gear_index) / vehicle.tires.wheel_radius_m * 60.0 / (2.0 * math.pi)


def drive_force(vehicle, speed, gear_index: int, launch: bool = False):
    """Drive force at the wheels [N] before subtracting the inertia of the rotating parts.

    NaN where the engine would exceed its rev limit. With `launch`, in first gear and below the first point of
    the torque curve, the clutch slips and the engine can give its peak torque (declared hypothesis).
    """
    pt = vehicle.powertrain
    ratio = total_ratio(vehicle, gear_index)
    eta = pt.drivetrain_efficiency if pt.torque_curve.applies_efficiency else 1.0
    rpm = rpm_from_speed(vehicle, speed, gear_index)
    torque = pt.torque_curve.torque(rpm)
    if launch and gear_index == 0:
        torque = np.where(rpm < pt.torque_curve.rpm_min, pt.torque_curve.peak_torque()[0], torque)
    torque = np.where(rpm > rev_limit(vehicle), np.nan, torque)
    return torque * ratio * eta / vehicle.tires.wheel_radius_m


def effective_mass(vehicle, gear_index: int) -> float:
    """Mass that the drive force accelerates in a gear [kg]: the car plus the inertia of wheels and engine.

    M_eff = M + Iw / r^2 + Ie * eta * N^2 / r^2
    """
    pt = vehicle.powertrain
    r = vehicle.tires.wheel_radius_m
    n = total_ratio(vehicle, gear_index)
    return vehicle.general.mass_kg + pt.wheels_inertia_kgm2 / r**2 + pt.engine_inertia_kgm2 * pt.drivetrain_efficiency * n**2 / r**2
