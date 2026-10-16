"""Geometric AprilTag observations, not a camera image or AprilTag decoder.

VEX AI Vision reports 320x240 images and a 74x63 degree field of view. Four
tagged faces on each goal are projected through an ideal pinhole lens. Finite
range, small tags, back faces, clipping and intervening goal/loader bodies can
prevent detection. Lens distortion, lighting and actual decoder errors need
hardware calibration. Camera mount and tag geometry are editable settings.
"""
from collections import deque
from copy import deepcopy
import math
from types import SimpleNamespace

from .field_elements import element_polygon, element_spec


WIDTH, HEIGHT = 320, 240


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def subtract(a, b):
    return tuple(x - y for x, y in zip(a, b))


def camera_pose(world, config):
    heading = math.radians(world.heading_deg)
    x = world.x_mm + config["x_mm"] * math.cos(heading) + config["y_mm"] * math.sin(heading)
    y = world.y_mm - config["x_mm"] * math.sin(heading) + config["y_mm"] * math.cos(heading)
    yaw = math.radians(world.heading_deg + config["yaw_deg"])
    pitch = math.radians(config["pitch_deg"])
    forward = (math.sin(yaw) * math.cos(pitch), math.cos(yaw) * math.cos(pitch), math.sin(pitch))
    right = (math.cos(yaw), -math.sin(yaw), 0)
    up = (-math.sin(yaw) * math.sin(pitch), -math.cos(yaw) * math.sin(pitch), math.cos(pitch))
    return (x, y, config["height_mm"]), forward, right, up


def _segment_blocked(start, end, polygon, height):
    """Clip a sightline against an upright convex prism (CCW footprints)."""
    delta = subtract(end, start)
    near, far = 0.0, 1.0
    # Every constraint has form value + slope*t >= 0.
    constraints = [(start[2], delta[2]), (height - start[2], -delta[2])]
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        constraints.append((dx * (start[1] - a[1]) - dy * (start[0] - a[0]),
                            dx * delta[1] - dy * delta[0]))
    for value, slope in constraints:
        if abs(slope) < 1e-9:
            if value < 0:
                return False
        elif slope > 0:
            near = max(near, -value / slope)
        else:
            far = min(far, -value / slope)
        if near > far:
            return False
    return far > 1e-6 and near < 1 - 1e-6


class VisionModel:
    def __init__(self, world, config):
        self.world, self.config = world, config
        self.next_sample = 0.0
        self.pending = deque()
        self.objects = []
        self.timestamp_ms = 0
        self.loss_until = 0.0
        self.enabled = True
        self.connected = True
        self.family = 0  # AiVision.TAG_CIRCLE21H7
        self.fx = WIDTH / (2 * math.tan(math.radians(config["horizontal_fov_deg"]) / 2))
        self.fy = HEIGHT / (2 * math.tan(math.radians(config["vertical_fov_deg"]) / 2))

    def _project(self, point, pose):
        position, forward, right, up = pose
        relative = subtract(point, position)
        z = dot(relative, forward)
        if z <= 1:
            return None
        return WIDTH / 2 + self.fx * dot(relative, right) / z, HEIGHT / 2 - self.fy * dot(relative, up) / z

    def _observe(self):
        pose = camera_pose(self.world, self.config)
        position = pose[0]
        objects = []
        size = self.config["tag_size_mm"]
        tilt = math.radians(self.config["tag_tilt_deg"])
        # Alliance goal upper part is shared with pedestal-mounted neutral goals.
        # Retain the estimated tag centre on the goal independently of its tilt.
        face_offset = 71.25 - (37.8 - 10) * (71.25 - 44.4) / (82.5 - 10)
        occluders = [(e, element_polygon(e), element_spec(e["kind"])["height_mm"])
                     for e in self.world.elements]
        for element in self.world.elements:
            if not element["kind"].startswith("goal_"):
                continue
            spec = element_spec(element["kind"])
            tag_id = element.get("tag_id", spec["tag_id"])
            for side in range(4):
                angle = math.radians(element["heading_deg"] + 90 * side)
                nx, ny = math.sin(angle), math.cos(angle)
                centre = (element["x_mm"] + nx * face_offset,
                          element["y_mm"] + ny * face_offset, spec["height_mm"] - 82.5 + 37.8)
                toward_camera = subtract(position, centre)
                distance = math.sqrt(dot(toward_camera, toward_camera))
                normal = (nx * math.cos(tilt), ny * math.cos(tilt), math.sin(tilt))
                if not self.config["near_mm"] <= distance <= self.config["far_mm"]:
                    continue
                if dot(normal, toward_camera) / distance < math.cos(math.radians(self.config["max_tag_angle_deg"])):
                    continue
                across = (ny, -nx, 0)
                up = (-nx * math.sin(tilt), -ny * math.sin(tilt), math.cos(tilt))
                corners = [tuple(centre[i] + size / 2 * (sx * across[i] + sy * up[i]) for i in range(3))
                           for sx, sy in ((-1, 1), (1, 1), (1, -1), (-1, -1))]
                projected = [self._project(p, pose) for p in corners]
                if any(p is None or not 0 <= p[0] <= WIDTH or not 0 <= p[1] <= HEIGHT for p in projected):
                    continue
                # Conservative rule: a covered corner or centre loses the tag.
                if any(_segment_blocked(position, point, poly, height)
                       for other, poly, height in occluders if other["id"] != element["id"]
                       for point in [centre] + corners):
                    continue
                px = [round(p[0]) for p in projected if p is not None]
                py = [round(p[1]) for p in projected if p is not None]
                width, height = max(px) - min(px), max(py) - min(py)
                if min(width, height) < self.config["minimum_tag_pixels"]:
                    continue
                objects.append({"id": tag_id, "originX": min(px), "originY": min(py),
                                "width": width, "height": height,
                                "centerX": round((max(px) + min(px)) / 2),
                                "centerY": round((max(py) + min(py)) / 2),
                                "angle": round(math.degrees(math.atan2(py[1] - py[0], px[1] - px[0]))) % 360,
                                "corners_x": px, "corners_y": py,
                                # UI-only provenance; never exposed by the VEX adapter.
                                "element_id": element["id"], "face": side})
        return sorted(objects, key=lambda item: -item["width"])

    def update(self):
        now = self.world.time_s
        if now + 1e-9 >= self.next_sample:
            detected = self._observe() if self.connected and self.enabled and self.family == 0 and now >= self.loss_until else []
            self.pending.append((now + self.config["latency_ms"] / 1000, round(now * 1000), detected))
            self.next_sample = now + 1 / self.config["sample_hz"]
        while self.pending and self.pending[0][0] <= now + 1e-9:
            _, self.timestamp_ms, self.objects = self.pending.popleft()

    def read(self):
        if not self.connected or not self.enabled or self.world.time_s < self.loss_until or self.family != 0:
            return []
        return deepcopy(self.objects)

    def snapshot(self):
        position, _, _, _ = camera_pose(self.world, self.config)
        return {"objects": self.read(), "timestamp_ms": self.timestamp_ms,
                "camera_x_mm": position[0], "camera_y_mm": position[1],
                "camera_heading_deg": (self.world.heading_deg + self.config["yaw_deg"]) % 360,
                "horizontal_fov_deg": self.config["horizontal_fov_deg"], "far_mm": self.config["far_mm"]}


def vex_object(item):
    """Expose only camera observations, never world position or element identity."""
    public = {key: item[key] for key in ("id", "originX", "originY", "width", "height", "centerX", "centerY", "angle")}
    return SimpleNamespace(**public, exists=True, type=8, score=0, area=item["width"] * item["height"],
                           tag=SimpleNamespace(x=tuple(item["corners_x"]), y=tuple(item["corners_y"])))
