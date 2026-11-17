"""Environment: the two constants that every result depends on."""
from dataclasses import dataclass

from .registry import Issue, default_environment, validate_params


@dataclass(frozen=True)
class Environment:
    gravity: float = 9.81        # m/s2
    air_density: float = 1.225   # kg/m3


class EnvironmentDataError(ValueError):
    def __init__(self, issues):
        self.issues = issues
        super().__init__("; ".join(i.message for i in issues if i.level == "error"))


def environment_from_dict(data: dict = None) -> Environment:
    """Builds an Environment, filling the missing values with the defaults and validating the result."""
    merged = default_environment()
    merged.update(data or {})
    issues = validate_params({"environment": merged})
    errors = [i for i in issues if i.level == "error" and i.key.startswith("environment.")]
    if errors:
        raise EnvironmentDataError(errors)
    return Environment(gravity=float(merged["gravity"]), air_density=float(merged["air_density"]))


def validate_environment_dict(data: dict = None) -> list:
    merged = default_environment()
    merged.update(data or {})
    return [i for i in validate_params({"environment": merged}) if i.key.startswith("environment.")]
