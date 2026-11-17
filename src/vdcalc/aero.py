"""Aerodynamic forces. The whole car is described by Cd, Cl and a reference area."""


def drag_force(env, aero, speed):
    """Drag [N] at the given speed [m/s]. Acts against the motion."""
    return 0.5 * env.air_density * aero.cd * aero.frontal_area_m2 * speed**2


def downforce(env, aero, speed):
    """Downforce [N]: positive when it presses the car onto the road (negative lift coefficient)."""
    return -0.5 * env.air_density * aero.cl * aero.frontal_area_m2 * speed**2
