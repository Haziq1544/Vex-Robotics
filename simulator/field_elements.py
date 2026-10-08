"""Pin-free Override layouts and measured planar footprints, in millimetres.

See FIELD_SOURCES.md for drawing provenance and approximations. Elements use
the same clockwise-from-+Y convention as the robot. Goals and simplified loader
footprints are fixed. Cups/stackers and wall-top toggles are visual placeholders.
"""
from copy import deepcopy
import math
from typing import Any
from uuid import uuid4


PRESETS = {"empty": "Empty field", "override": "Override (no pins)", "custom": "Custom"}

_GOAL = dict(width_mm=142.5, depth_mm=142.5, movable=False, collidable=True,
             hole_diameter_mm=60.1, radius_mm=81.9)
ELEMENT_SPECS: dict[str, dict[str, Any]] = {
    "goal_center": dict(_GOAL, label="Centre goal", height_mm=222.7, color="#30353b", tag_id=0),
    "goal_neutral": dict(_GOAL, label="Neutral goal", height_mm=146.5, color="#424950", tag_id=1),
    "goal_red": dict(_GOAL, label="Red goal", height_mm=82.5, color="#d95059", tag_id=2),
    "goal_blue": dict(_GOAL, label="Blue goal", height_mm=82.5, color="#318bc3", tag_id=3),
    "cup": dict(label="Cup / stacker (later)", width_mm=80.2, depth_mm=80.2, height_mm=164.5,
                color="#aeb8bf", movable=False, collidable=False),
    "loader": dict(label="Loader", width_mm=95.0, depth_mm=102.1, height_mm=365.0,
                   color="#80919d", movable=False, collidable=True),
}


def element_spec(kind):
    if not isinstance(kind, str) or kind not in ELEMENT_SPECS:
        raise ValueError("Unknown field element kind: " + str(kind))
    return deepcopy(ELEMENT_SPECS[kind])


def make_element(kind, x_mm: float = 0, y_mm: float = 0, heading_deg: float = 0, element_id=None):
    element_spec(kind)
    return {"id": element_id or kind + "_" + uuid4().hex[:8], "kind": kind,
            "x_mm": float(x_mm), "y_mm": float(y_mm), "heading_deg": float(heading_deg)}


def _local_footprint(kind):
    spec = ELEMENT_SPECS[kind]
    w, d = spec["width_mm"] / 2, spec["depth_mm"] / 2
    if kind == "loader":
        return [(-w, -d), (w, -d), (w, d), (-w, d)]
    if kind == "cup":
        return [(w * math.cos(i * math.tau / 16), w * math.sin(i * math.tau / 16)) for i in range(16)]
    # Circle truncated by four flat sides. Include exact arc/flat intersections,
    # then sample arcs every 5 degrees (maximum chord error <0.08 mm).
    r = spec["radius_mm"]
    angle = math.acos(w / r)
    angles = [i * math.tau / 72 for i in range(72)]
    angles += [(quarter * math.pi / 2 + sign * angle) % math.tau
               for quarter in range(4) for sign in (-1, 1)]
    return [(r * math.cos(a), r * math.sin(a)) for a in sorted(angles)
            if abs(r * math.cos(a)) <= w + 1e-7 and abs(r * math.sin(a)) <= d + 1e-7]


_FOOTPRINTS = {kind: _local_footprint(kind) for kind in ELEMENT_SPECS}


def element_polygon(element):
    a = math.radians(element.get("heading_deg", 0))
    c, s = math.cos(a), math.sin(a)
    return [(element["x_mm"] + x * c + y * s,
             element["y_mm"] - x * s + y * c) for x, y in _FOOTPRINTS[element["kind"]]]


def _override_elements(config):
    elements = [make_element("goal_center", element_id="centre_goal")]
    positions = {
        "goal_neutral": [(-1196.1, 598.1), (-598.1, 1196.1), (1196.1, -598.1), (598.1, -1196.1)],
        "goal_red": [(-1196.1, -598.1), (-598.1, -1196.1)],
        "goal_blue": [(598.1, 1196.1), (1196.1, 598.1)],
    }
    for kind, points in positions.items():
        for i, (x, y) in enumerate(points):
            element = make_element(kind, x, y, element_id=kind + "_" + str(i + 1))
            if kind == "goal_neutral":
                element["tag_id"] = 4 if abs(y) > abs(x) else 1
            elements.append(element)
    hw, hh = config["field"]["width_mm"] / 2, config["field"]["height_mm"] / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            elements.append(make_element("loader", sx * (hw - 47.5), sy * (hh - 290.6),
                                         element_id="loader_%s_%s" % (sx, sy)))
    cup_points = [(sx * dist, sy * dist) for dist in (598.1, 1196.1)
                  for sx in (-1, 1) for sy in (-1, 1)]
    cup_points += [(-598.1, 0), (598.1, 0), (0, -598.1), (0, 598.1)]
    for i, (x, y) in enumerate(cup_points):
        elements.append(dict(make_element("cup", x, y, element_id="floor_cup_%02d" % (i + 1)), face="clear"))
    for wall in ("north", "south", "east", "west"):
        for group in (-1, 1):
            for offset in (-1, 0, 1):
                along = group * 598.1 + offset * 80.2
                x, y = ((along, (hh - 40.1) * (1 if wall == "north" else -1))
                        if wall in ("north", "south") else
                        ((hw - 40.1) * (1 if wall == "east" else -1), along))
                elements.append(dict(make_element("cup", x, y,
                    element_id="%s_cup_%s_%s" % (wall, group, offset)), face="opaque"))
    return elements


def resolve_elements(config):
    layout = config.get("layout", {"preset": "empty", "elements": []})
    if not isinstance(layout, dict) or not isinstance(layout.get("preset"), str) or layout["preset"] not in PRESETS:
        raise ValueError("layout.preset must be empty, override or custom")
    preset = layout["preset"]
    if preset == "override":
        return _override_elements(config)
    return deepcopy(layout.get("elements", [])) if preset == "custom" else []


def set_preset(config, preset):
    if preset not in PRESETS:
        raise ValueError("Unknown field preset: " + str(preset))
    if preset == "custom":
        materialize_layout(config)
    else:
        config["layout"] = {"preset": preset, "elements": []}
    return config["layout"]


def materialize_layout(config):
    config["layout"] = {"preset": "custom", "elements": resolve_elements(config)}
    return config["layout"]["elements"]


def validate_layout(config):
    """Validate saved layouts without mutating them; touching footprints are OK."""
    from .collisions import polygons_overlap
    layout = config.get("layout", {"preset": "empty", "elements": []})
    elements = resolve_elements(config)
    if not isinstance(layout.get("elements", []), list):
        raise ValueError("layout.elements must be a list")
    if layout["preset"] != "custom" and layout.get("elements"):
        raise ValueError("Use the custom preset to store edited elements")
    if len(elements) > 256:
        raise ValueError("At most 256 field elements are supported")
    ids, polygons = set(), []
    for element in elements:
        if not isinstance(element, dict):
            raise ValueError("Each field element must be an object")
        identity = element.get("id")
        if not isinstance(identity, str) or not identity.strip() or identity in ids:
            raise ValueError("Field elements need unique, nonempty string IDs")
        ids.add(identity)
        element_spec(element.get("kind"))
        for key in ("x_mm", "y_mm", "heading_deg"):
            value = element.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(identity + "." + key + " must be a finite number")
        if element.get("face", "clear") not in ("clear", "opaque"):
            raise ValueError(identity + ".face must be clear or opaque")
        polygon = element_polygon(element)
        if any(abs(x) > config["field"]["width_mm"] / 2 + 1e-6 or
               abs(y) > config["field"]["height_mm"] / 2 + 1e-6 for x, y in polygon):
            raise ValueError(identity + " must fit completely inside the field")
        for other_id, other in polygons:
            if polygons_overlap(polygon, other):
                raise ValueError(identity + " overlaps " + other_id)
        polygons.append((identity, polygon))
    return config
