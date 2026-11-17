"""Vertical loads and resistances at a given speed and road condition.

Moment equilibrium of the car (side view), with the aerodynamic drag at the height of the centre of gravity:

    N_front = Wn * wf + Fdown * phi  - (Fx - R) * h / L
    N_rear  = Wn * (1 - wf) + Fdown * (1 - phi) + (Fx - R) * h / L

where Fx is the longitudinal force of the tyres on the road (positive = driving), R the rolling resistance,
Wn the normal weight, Fdown the downforce, wf the front weight fraction, phi the front aerodynamic share, h the
height of the centre of gravity and L the wheelbase.
"""
from collections import namedtuple
from dataclasses import dataclass

import numpy as np

from .aero import downforce, drag_force


@dataclass(frozen=True)
class Conditions:
    """Local road condition. The default is a flat, straight road.

    slope: road angle [rad], positive uphill.
    vertical_curvature: d(slope)/ds [1/m]. Positive in a compression (dip), negative over a crest.
    Both may be NumPy arrays (one value per point) as long as the speed has a compatible shape.
    """
    slope: float = 0.0
    vertical_curvature: float = 0.0


FLAT = Conditions()

LoadTerms = namedtuple("LoadTerms", "weight_normal downforce drag rolling grade front_base rear_base total_normal")


def load_terms(vehicle, env, speed, cond: Conditions = FLAT) -> LoadTerms:
    """Everything the limits need at this speed.

    front_base and rear_base are the axle loads for Fx = R (no longitudinal load transfer).
    """
    v = np.asarray(speed, dtype=float)
    m = vehicle.general.mass_kg
    wn = m * (env.gravity * np.cos(cond.slope) + v**2 * cond.vertical_curvature)
    down = downforce(env, vehicle.aero, v)
    total = np.maximum(wn + down, 0.0)
    rolling = vehicle.tires.rolling_coefficient * total
    drag = drag_force(env, vehicle.aero, v)
    grade = m * env.gravity * np.sin(cond.slope)
    wf, phi = vehicle.general.front_weight_fraction, vehicle.aero.front_fraction
    airborne = (wn + down) <= 0.0                         # the car has left the ground: no load on any axle
    front = np.where(airborne, 0.0, np.maximum(wn * wf + down * phi, 0.0))
    rear = np.where(airborne, 0.0, np.maximum(wn * (1.0 - wf) + down * (1.0 - phi), 0.0))
    return LoadTerms(wn, down, drag, rolling, grade, front, rear, total)


def axle_loads(vehicle, env, speed, tire_force, cond: Conditions = FLAT):
    """(front, rear) axle loads [N] when the tyres push the car with a longitudinal force (negative = braking)."""
    t = load_terms(vehicle, env, speed, cond)
    transfer = (np.asarray(tire_force, dtype=float) - t.rolling) * vehicle.general.cg_height_m / vehicle.general.wheelbase_m
    return t.front_base - transfer, t.rear_base + transfer
