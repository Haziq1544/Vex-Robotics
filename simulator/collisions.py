"""Convex, planar contact helpers used by the simulator and layout validation.

All geometry is in millimetres; velocities, masses and impulses use SI units.
Contacts approximate rigid, upright footprints: they do not model tipping,
stacking, deformation, lifting, or contact with elevated mechanisms.
"""

from dataclasses import dataclass, field
import math


def bounds(polygon):
    return (min(p[0] for p in polygon), min(p[1] for p in polygon),
            max(p[0] for p in polygon), max(p[1] for p in polygon))


def _bounds_overlap(a, b, tolerance=1e-7):
    return (a[2] - b[0] > tolerance and b[2] - a[0] > tolerance and
            a[3] - b[1] > tolerance and b[3] - a[1] > tolerance)


def _separation(polygon_a, polygon_b, tolerance):
    if not _bounds_overlap(bounds(polygon_a), bounds(polygon_b), tolerance):
        return None
    best = (float("inf"), 0.0, 0.0)
    for polygon in (polygon_a, polygon_b):
        for first, second in zip(polygon, polygon[1:] + polygon[:1]):
            dx, dy = second[0] - first[0], second[1] - first[1]
            length = math.hypot(dx, dy)
            if length < 1e-10:
                continue
            nx, ny = -dy / length, dx / length
            projected_a = [x * nx + y * ny for x, y in polygon_a]
            projected_b = [x * nx + y * ny for x, y in polygon_b]
            positive = max(projected_b) - min(projected_a)
            negative = max(projected_a) - min(projected_b)
            if min(positive, negative) <= tolerance:
                return None
            # Using exit distances also handles a polygon contained in another.
            if positive < best[0]:
                best = positive, nx, ny
            if negative < best[0]:
                best = negative, -nx, -ny
    return best


def polygons_overlap(polygon_a, polygon_b, tolerance_mm=1e-6):
    """Whether convex footprints penetrate; exact tangency is permitted."""
    return _separation(polygon_a, polygon_b, tolerance_mm) is not None


def _intersection(subject, clip):
    """Sutherland-Hodgman convex clipping, accepting either winding."""
    area = sum(a[0] * b[1] - b[0] * a[1]
               for a, b in zip(clip, clip[1:] + clip[:1]))
    direction = 1 if area >= 0 else -1
    output = list(subject)
    for a, b in zip(clip, clip[1:] + clip[:1]):
        if not output:
            break
        def side(p):
            return direction * ((b[0] - a[0]) * (p[1] - a[1]) -
                                (b[1] - a[1]) * (p[0] - a[0]))
        previous = output[-1]
        prev_side = side(previous)
        source, output = output, []
        for current in source:
            cur_side = side(current)
            if (cur_side >= 0) != (prev_side >= 0):
                fraction = prev_side / (prev_side - cur_side)
                output.append((previous[0] + fraction * (current[0] - previous[0]),
                               previous[1] + fraction * (current[1] - previous[1])))
            if cur_side >= 0:
                output.append(current)
            previous, prev_side = current, cur_side
    return output


def polygon_contact(polygon_a, polygon_b):
    """Return (depth_mm, nx, ny, points_mm) or None; normal pushes A from B.

    The manifold uses the tangent extent of the intersecting footprints. A
    shallow flat impact therefore has two contacts; a corner impact has one
    or two nearby contacts. Tangency alone is not penetration.
    """
    separation = _separation(polygon_a, polygon_b, 1e-7)
    if separation is None:
        return None
    depth, nx, ny = separation
    intersection = _intersection(polygon_a, polygon_b)
    if not intersection:
        return None
    tangent = [-ny * x + nx * y for x, y in intersection]
    low, high = min(tangent), max(tangent)
    normal = (min(x * nx + y * ny for x, y in polygon_a) +
              max(x * nx + y * ny for x, y in polygon_b)) / 2
    locations = [(low + high) / 2] if high - low < 0.05 else [low, high]
    points = [(nx * normal - ny * t, ny * normal + nx * t) for t in locations]
    return depth, nx, ny, points


@dataclass
class Body:
    """A convex body with clockwise angular velocity and a cached footprint."""
    local: list
    x_mm: float
    y_mm: float
    heading_deg: float
    inverse_mass: float
    inverse_inertia: float
    element: dict | None = None
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    _pose: tuple | None = None
    _polygon: list = field(default_factory=list)
    _bounds: tuple = (0, 0, 0, 0)

    @property
    def polygon(self):
        pose = self.x_mm, self.y_mm, self.heading_deg
        if pose != self._pose:
            angle = math.radians(self.heading_deg)
            s, c = math.sin(angle), math.cos(angle)
            self._polygon = [(self.x_mm + x * c + y * s,
                              self.y_mm - x * s + y * c) for x, y in self.local]
            self._bounds = bounds(self._polygon)
            self._pose = pose
        return self._polygon

    @property
    def aabb(self):
        self.polygon
        return self._bounds


def broadphase_pairs(bodies):
    """Deterministic spatial buckets limit nearby obstacle checks."""
    buckets = {}
    candidates = set()
    for index, body in enumerate(bodies):
        left, bottom, right, top = body.aabb
        for x in range(math.floor(left / 250), math.floor(right / 250) + 1):
            for y in range(math.floor(bottom / 250), math.floor(top / 250) + 1):
                bucket = buckets.setdefault((x, y), [])
                for other in bucket:
                    if body.inverse_mass or bodies[other].inverse_mass:
                        candidates.add((other, index))
                bucket.append(index)
    return [(bodies[a], bodies[b]) for a, b in sorted(candidates)
            if _bounds_overlap(bodies[a].aabb, bodies[b].aabb)]


def _contact_impulse(a, b, point, nx, ny, restitution):
    ax, ay = (point[0] - a.x_mm) / 1000, (point[1] - a.y_mm) / 1000
    bx, by = (point[0] - b.x_mm) / 1000, (point[1] - b.y_mm) / 1000
    relative_x = a.vx + a.omega * ay - b.vx - b.omega * by
    relative_y = a.vy - a.omega * ax - b.vy + b.omega * bx
    normal_speed = relative_x * nx + relative_y * ny
    if normal_speed >= 0:
        return
    arm_a, arm_b = ay * nx - ax * ny, by * nx - bx * ny
    effective_inverse_mass = (a.inverse_mass + b.inverse_mass +
                              arm_a ** 2 * a.inverse_inertia + arm_b ** 2 * b.inverse_inertia)
    bounce = restitution if normal_speed < -0.1 else 0.0
    impulse = -(1 + bounce) * normal_speed / effective_inverse_mass
    a.vx += impulse * nx * a.inverse_mass
    a.vy += impulse * ny * a.inverse_mass
    a.omega += impulse * arm_a * a.inverse_inertia
    b.vx -= impulse * nx * b.inverse_mass
    b.vy -= impulse * ny * b.inverse_mass
    b.omega -= impulse * arm_b * b.inverse_inertia


def resolve_contact(a, b, contact, restitution):
    """Project penetration and apply normal impulses with angular response.

    Contacts themselves are smooth (no tangential contact friction). Floor
    friction is handled separately. This permits sliding around goal corners.
    """
    depth, nx, ny, points = contact
    # Iterate a two-point manifold to avoid invented torque on a flat impact.
    for _ in range(10 if len(points) == 2 else 1):
        for point in points:
            _contact_impulse(a, b, point, nx, ny, restitution)
    # Field elements have infinite mass, so only the robot is projected.
    weight_a, weight_b = a.inverse_mass, b.inverse_mass
    total = weight_a + weight_b
    correction = depth + 1e-5
    a.x_mm += nx * correction * weight_a / total
    a.y_mm += ny * correction * weight_a / total
    b.x_mm -= nx * correction * weight_b / total
    b.y_mm -= ny * correction * weight_b / total


def contain_body(body, half_width_mm, half_height_mm, restitution):
    """Project a body's full footprint into the field and stop wall penetration."""
    collided = False
    for _ in range(4):
        points = body.polygon
        contacts = []
        for x, y in points:
            if x < -half_width_mm:
                contacts.append((-half_width_mm - x, 1, 0, (x, y)))
            if x > half_width_mm:
                contacts.append((x - half_width_mm, -1, 0, (x, y)))
            if y < -half_height_mm:
                contacts.append((-half_height_mm - y, 0, 1, (x, y)))
            if y > half_height_mm:
                contacts.append((y - half_height_mm, 0, -1, (x, y)))
        if not contacts:
            break
        collided = True
        depth, nx, ny, _ = max(contacts)
        face = [point for penetration, xnormal, ynormal, point in contacts
                if xnormal == nx and ynormal == ny and abs(penetration - depth) < 1e-5]
        # A virtual infinite-mass body represents the selected wall.
        wall = Body([], 0, 0, 0, 0, 0)
        resolve_contact(body, wall, (depth, nx, ny, face[:1] + face[-1:] if len(face) > 1 else face),
                        restitution)
    return collided
