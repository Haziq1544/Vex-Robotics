"""Load tunable simulator measurements; never modify the robot's source code."""

from copy import deepcopy
import json
import math
from pathlib import Path


DEFAULT_CONFIG_PATH = Path(__file__).with_name("default_config.json")


def default_config():
    """Return an independent dictionary of the documented approximate defaults."""
    with DEFAULT_CONFIG_PATH.open(encoding="utf-8") as stream:
        return json.load(stream)


def _merge(base, overrides, prefix=""):
    for key, value in overrides.items():
        name = prefix + key
        if key not in base:
            raise ValueError("Unknown simulator setting: " + name)
        if isinstance(base[key], dict):
            if not isinstance(value, dict):
                raise ValueError(name + " must be an object")
            _merge(base[key], value, name + ".")
        else:
            base[key] = deepcopy(value)


def load_config(path=None):
    """Load defaults, optionally merge a partial JSON override, and validate."""
    result = default_config()
    if path is not None:
        with Path(path).open(encoding="utf-8") as stream:
            overrides = json.load(stream)
        if not isinstance(overrides, dict):
            raise ValueError("Simulator configuration must be a JSON object")
        _merge(result, overrides)
    validate_config(result)
    return result


def validate_config(config):
    """Reject invalid dimensions/physics early, with a useful setting name."""
    positive = {
        "field": ("width_mm", "height_mm", "tile_mm"),
        "robot": ("body_length_mm", "body_width_mm", "wheel_diameter_mm",
                  "track_width_mm", "external_ratio", "mass_kg",
                  "motor_free_rpm", "motor_stall_torque_nm"),
        "physics": ("motor_response_s", "brake_response_s", "coast_response_s",
                    "wheel_effective_mass_kg", "tire_stiffness_n_per_m_s",
                    "inertia_scale"),
        "gps": ("sample_hz",),
        "simulation": ("step_ms",),
    }
    nonnegative = {
        "physics": ("traction_mu", "lateral_mu", "rolling_drag_n_per_m_s",
                    "lateral_damping_per_s", "yaw_drag_nm_per_rad_s", "restitution"),
        "gps": ("latency_ms", "position_noise_mm", "heading_noise_deg",
                "turn_dropout_deg_s", "wall_min_distance_mm"),
    }
    defaults = default_config()
    for section, values in defaults.items():
        if section == "notes":
            continue
        if section not in config or not isinstance(config[section], dict):
            raise ValueError("Missing configuration section: " + section)
        for key in values:
            value = config[section].get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(section + "." + key + " must be a finite number")
    for section, keys in positive.items():
        for key in keys:
            if config[section][key] <= 0:
                raise ValueError(section + "." + key + " must be positive")
    for section, keys in nonnegative.items():
        for key in keys:
            if config[section][key] < 0:
                raise ValueError(section + "." + key + " cannot be negative")
    robot, field, physics = config["robot"], config["field"], config["physics"]
    for key in ("left_mount_sign", "right_mount_sign"):
        if robot[key] not in (-1, 1):
            raise ValueError("robot." + key + " must be -1 or 1")
    ports = [robot[key] for key in ("left_port", "right_port", "gps_port")]
    if any(int(p) != p or p < 1 or p > 21 for p in ports) or len(set(ports)) != 3:
        raise ValueError("Motor and GPS ports must be distinct integers from 1 to 21")
    if robot["track_width_mm"] > robot["body_width_mm"]:
        raise ValueError("robot.track_width_mm cannot exceed the collision body width")
    if math.hypot(robot["body_length_mm"], robot["body_width_mm"]) >= min(field["width_mm"], field["height_mm"]):
        raise ValueError("Robot must fit inside the field at every heading")
    if not 0 <= physics["restitution"] <= 1:
        raise ValueError("physics.restitution must be between 0 and 1")
    if physics["traction_mu"] > 3 or physics["lateral_mu"] > 3:
        raise ValueError("Friction coefficients must be between 0 and 3")
    if config["simulation"]["step_ms"] > 10:
        raise ValueError("simulation.step_ms must be at most 10 for stable integration")
    if int(config["simulation"]["seed"]) != config["simulation"]["seed"]:
        raise ValueError("simulation.seed must be an integer")
    return config
