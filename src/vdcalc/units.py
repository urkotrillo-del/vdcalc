"""Unit conversions used at the edges of the program (input and display). The core always works in SI."""

MPH = 0.44704  # m/s per mph


def kmh_to_ms(v):
    return v / 3.6


def ms_to_kmh(v):
    return v * 3.6


def mph_to_ms(v):
    return v * MPH


def format_laptime(seconds: float) -> str:
    """Lap time as min:seconds, for example 2:29.05."""
    minutes = int(seconds // 60)
    return f"{minutes}:{seconds - 60 * minutes:05.2f}"
