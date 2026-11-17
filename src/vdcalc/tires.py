"""Tyre friction. One axle = two identical tyres carrying the same load."""
import numpy as np


def axle_force_max(mu0, c_per_kn, axle_load):
    """Maximum force of the two tyres of an axle [N] for a given axle load [N].

    Each tyre carries Fz = load / 2 and gives mu * Fz, with mu = mu0 - c * Fz[kN] (never below 0).
    A constant-friction tyre is the case c = 0.
    """
    load = np.maximum(np.asarray(axle_load, dtype=float), 0.0)
    fz = load / 2.0
    mu = np.maximum(mu0 - c_per_kn * fz / 1000.0, 0.0)
    return 2.0 * mu * fz
