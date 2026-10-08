"""Small planar rigid-body model of a two-motor VEX differential drivetrain.

Public positions are mm, heading is clockwise from +Y. Integration uses SI
units, independent wheel inertia, a bounded tire force proportional to slip,
motor torque/speed saturation, lateral friction and an oriented wall collider.
These are tunable approximations, not a model of VEX firmware or foam material.
"""

from copy import deepcopy
import math

from .config import validate_config
from .collisions import Body, broadphase_pairs, contain_body, polygon_contact, resolve_contact
from .field_elements import element_polygon, element_spec, resolve_elements


def clamp(value, low, high):
    return max(low, min(high, value))


class World:
    def __init__(self, config, start_pose=None):
        self.config = deepcopy(validate_config(config))
        self.robot = self.config["robot"]
        self.physics = self.config["physics"]
        pose = {"x_mm": -1000.0, "y_mm": -1000.0, "heading_deg": 0.0}
        if start_pose:
            pose.update(start_pose)
        if any(not math.isfinite(float(v)) for v in pose.values()):
            raise ValueError("Start pose must contain finite numbers")
        self.x_mm = float(pose["x_mm"])
        self.y_mm = float(pose["y_mm"])
        self.heading_deg = float(pose["heading_deg"]) % 360.0
        self.time_s = 0.0
        self.vx = self.vy = self.omega = 0.0  # m/s, m/s, clockwise rad/s
        self.wheel_speed = [0.0, 0.0]  # forward-positive tread m/s
        self.wheel_distance = [0.0, 0.0]  # tread meters, independent of slip
        self.commands = [{"percent": 0.0, "brake": "coast"} for _ in range(2)]
        self.hold_distance = [0.0, 0.0]
        self.collision = False
        self.collision_count = 0
        self.collision_elements = set()
        self.radius = self.robot["wheel_diameter_mm"] / 2000.0
        self.track = self.robot["track_width_mm"] / 1000.0
        self.mass = self.robot["mass_kg"]
        self.inertia = self.mass * ((self.robot["body_length_mm"] / 1000) ** 2 +
                                    (self.robot["body_width_mm"] / 1000) ** 2) / 12 * self.physics["inertia_scale"]
        self.free_speed = self.robot["motor_free_rpm"] * 2 * math.pi / 60 * self.radius / self.robot["external_ratio"]
        self.stall_force = self.robot["motor_stall_torque_nm"] * self.robot["external_ratio"] / self.radius
        self.elements = resolve_elements(self.config)
        self._element_bodies = []
        for element in self.elements:
            spec = element_spec(element["kind"])
            if not spec["collidable"]:
                continue  # Future non-colliding kinds must not create bodies.
            centered = dict(element, x_mm=0.0, y_mm=0.0, heading_deg=0.0)
            local = element_polygon(centered)
            self._element_bodies.append(Body(local, element["x_mm"], element["y_mm"],
                                            element["heading_deg"], 0, 0, element=element))
        half_w, half_l = self.robot["body_width_mm"] / 2, self.robot["body_length_mm"] / 2
        self._robot_body = Body([(-half_w, half_l), (half_w, half_l),
                                 (half_w, -half_l), (-half_w, -half_l)],
                                self.x_mm, self.y_mm, self.heading_deg, 1 / self.mass, 1 / self.inertia)
        self._contact_bodies = [self._robot_body] + self._element_bodies
        self._fit_start()

    def _fit_start(self):
        half_w = self.config["field"]["width_mm"] / 2
        half_h = self.config["field"]["height_mm"] / 2
        if any(abs(x) > half_w + 1e-6 or abs(y) > half_h + 1e-6 for x, y in self.robot_corners()):
            raise ValueError("The complete robot must start inside the field walls")
        for body in self._element_bodies:
            if polygon_contact(self.robot_corners(), body.polygon) is not None:
                raise ValueError("Robot start overlaps game element " + str(body.element["id"]))

    def set_motor(self, port, command):
        """Accept physical shaft percent; mounting sign converts it to wheel motion."""
        ports = (self.robot["left_port"], self.robot["right_port"])
        if port not in ports:
            raise NotImplementedError("No simulated drive motor on port %s" % port)
        index = ports.index(port)
        percent = float(command.get("percent", 0.0))
        mode = command.get("brake", "coast")
        if not math.isfinite(percent):
            raise ValueError("Motor percent must be finite")
        if mode not in ("coast", "brake", "hold"):
            raise ValueError("Unknown motor brake mode: " + str(mode))
        if mode == "hold" and (self.commands[index]["brake"] != "hold" or self.commands[index]["percent"] != 0):
            self.hold_distance[index] = self.wheel_distance[index]
        self.commands[index] = {"percent": clamp(percent, -100, 100), "brake": mode}

    def robot_corners(self):
        angle = math.radians(self.heading_deg)
        s, c = math.sin(angle), math.cos(angle)
        half_w = self.robot["body_width_mm"] / 2
        half_l = self.robot["body_length_mm"] / 2
        return [(self.x_mm + x * c + y * s, self.y_mm - x * s + y * c)
                for x, y in ((-half_w, half_l), (half_w, half_l),
                             (half_w, -half_l), (-half_w, -half_l))]

    def step(self, dt):
        """Advance simulated time, subdividing larger calls for stable contact forces."""
        dt = float(dt)
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Physics step must be finite and nonnegative")
        if dt == 0:
            return
        p = self.physics
        # Explicit tire coupling has a stiffness-dependent stability limit.
        stable_dt = 0.2 / (p["tire_stiffness_n_per_m_s"] *
                           (1 / p["wheel_effective_mass_kg"] + 2 / self.mass + self.track ** 2 / (2 * self.inertia)))
        step = min(self.config["simulation"]["step_ms"] / 1000, stable_dt,
                   p["motor_response_s"] / 8, p["brake_response_s"] / 8)
        if self._element_bodies:
            # Limit travel of every vertex, including during fast rotation. The
            # normal drivetrain is already safely below this 8 mm sweep bound.
            robot_radius = math.hypot(self.robot["body_width_mm"], self.robot["body_length_mm"]) / 2000
            sweep_speed = max(2 * self.free_speed + 2 * self.free_speed / self.track * robot_radius,
                              math.hypot(self.vx, self.vy) + abs(self.omega) * robot_radius)
            step = min(step, 0.008 / max(sweep_speed, 1e-6))
        count = max(1, math.ceil(dt / step))
        interval = dt / count
        self.collision = False
        self.collision_elements.clear()
        for _ in range(count):
            self._integrate(interval)
        self.time_s += dt

    def _motor_force(self, index):
        p = self.physics
        command = self.commands[index]
        sign = self.robot["left_mount_sign" if index == 0 else "right_mount_sign"]
        target = command["percent"] * sign / 100 * self.free_speed
        speed = self.wheel_speed[index]
        effective_mass = p["wheel_effective_mass_kg"] + self.mass / 2
        if command["percent"] == 0:
            if command["brake"] == "coast":
                return -p["wheel_effective_mass_kg"] * speed / p["coast_response_s"]
            if command["brake"] == "hold":
                target = clamp((self.hold_distance[index] - self.wheel_distance[index]) * 8,
                               -self.free_speed / 4, self.free_speed / 4)
            requested = effective_mass * (target - speed) / p["brake_response_s"]
        else:
            requested = effective_mass * (target - speed) / p["motor_response_s"]
        # Approximate torque-speed envelope. Deceleration can use full torque.
        limit = self.stall_force
        if requested * speed > 0:
            limit *= max(0.0, 1.0 - abs(speed) / self.free_speed)
        return clamp(requested, -limit, limit)

    def _integrate(self, dt):
        angle = math.radians(self.heading_deg)
        s, c = math.sin(angle), math.cos(angle)
        forward_v = self.vx * s + self.vy * c
        lateral_v = self.vx * c - self.vy * s
        p = self.physics
        contact_speed = (forward_v + self.omega * self.track / 2,
                         forward_v - self.omega * self.track / 2)
        traction_limit = p["traction_mu"] * self.mass * 9.81 / 2
        forces = []
        for index in range(2):
            slip = self.wheel_speed[index] - contact_speed[index]
            tire_force = clamp(p["tire_stiffness_n_per_m_s"] * slip, -traction_limit, traction_limit)
            motor_force = self._motor_force(index)
            self.wheel_speed[index] += (motor_force - tire_force) / p["wheel_effective_mass_kg"] * dt
            self.wheel_distance[index] += self.wheel_speed[index] * dt
            forces.append(tire_force)
        lateral_force = clamp(-self.mass * lateral_v * p["lateral_damping_per_s"],
                              -p["lateral_mu"] * self.mass * 9.81,
                              p["lateral_mu"] * self.mass * 9.81)
        drive_force = sum(forces) - p["rolling_drag_n_per_m_s"] * forward_v
        self.vx += (drive_force * s + lateral_force * c) / self.mass * dt
        self.vy += (drive_force * c - lateral_force * s) / self.mass * dt
        torque = (forces[0] - forces[1]) * self.track / 2 - p["yaw_drag_nm_per_rad_s"] * self.omega
        self.omega += torque / self.inertia * dt
        self.x_mm += self.vx * dt * 1000
        self.y_mm += self.vy * dt * 1000
        self.heading_deg = (self.heading_deg + math.degrees(self.omega * dt)) % 360
        self._wall_contacts()
        if self._element_bodies:
            self._element_contacts()

    def _record_element_collision(self, element_id=None):
        if not self.collision:
            self.collision_count += 1
        self.collision = True
        if element_id is not None:
            self.collision_elements.add(element_id)

    def _element_contacts(self):
        """Resolve the robot against fixed goal/loader footprints and walls."""
        robot = self._robot_body
        robot.x_mm, robot.y_mm, robot.heading_deg = self.x_mm, self.y_mm, self.heading_deg
        robot.vx, robot.vy, robot.omega = self.vx, self.vy, self.omega
        bodies = self._contact_bodies
        half_w, half_h = self.config["field"]["width_mm"] / 2, self.config["field"]["height_mm"] / 2
        restitution = self.physics["restitution"]
        # Iterate so resolving one obstacle cannot leave the robot in a wall
        # or another obstacle. Every field body has zero inverse mass.
        for _ in range(24):
            resolved = False
            for a, b in broadphase_pairs(bodies):
                contact = polygon_contact(a.polygon, b.polygon)
                if contact is None:
                    continue
                if a is robot:
                    self._record_element_collision(b.element["id"])
                elif b is robot:
                    self._record_element_collision(a.element["id"])
                resolve_contact(a, b, contact, restitution)
                # Sub-micron contacts still receive impulses, but need no
                # further expensive projection pass at this substep.
                resolved |= contact[0] > 0.001
            if contain_body(robot, half_w, half_h, restitution):
                resolved = True
                self._record_element_collision()
            if not resolved:
                break
        self.x_mm, self.y_mm, self.heading_deg = robot.x_mm, robot.y_mm, robot.heading_deg
        self.vx, self.vy, self.omega = robot.vx, robot.vy, robot.omega

    def _wall_contacts(self):
        half_w = self.config["field"]["width_mm"] / 2
        half_h = self.config["field"]["height_mm"] / 2
        # Position projection keeps the full rotated rectangle in bounds. Contact
        # impulses remove only incoming normal velocity, including angular motion.
        for _ in range(4):
            corners = self.robot_corners()
            contacts = []
            for x, y in corners:
                if x < -half_w:
                    contacts.append((-half_w - x, 1.0, 0.0, x, y))
                if x > half_w:
                    contacts.append((x - half_w, -1.0, 0.0, x, y))
                if y < -half_h:
                    contacts.append((-half_h - y, 0.0, 1.0, x, y))
                if y > half_h:
                    contacts.append((y - half_h, 0.0, -1.0, x, y))
            if not contacts:
                break
            if not self.collision:
                self.collision_count += 1
            self.collision = True
            penetration, nx, ny, cx, cy = max(contacts)
            # Average coplanar contact points avoids false torque on flat impact.
            same_face = [(x, y) for depth, xnormal, ynormal, x, y in contacts
                         if xnormal == nx and ynormal == ny and abs(depth - penetration) < 1e-5]
            cx = sum(point[0] for point in same_face) / len(same_face)
            cy = sum(point[1] for point in same_face) / len(same_face)
            rx, ry = (cx - self.x_mm) / 1000, (cy - self.y_mm) / 1000
            normal_v = (self.vx + self.omega * ry) * nx + (self.vy - self.omega * rx) * ny
            moment = ry * nx - rx * ny
            if normal_v < 0:
                impulse = -(1 + self.physics["restitution"]) * normal_v / (1 / self.mass + moment ** 2 / self.inertia)
                self.vx += impulse * nx / self.mass
                self.vy += impulse * ny / self.mass
                self.omega += impulse * moment / self.inertia
            self.x_mm += nx * penetration
            self.y_mm += ny * penetration

    def snapshot(self):
        angle = math.radians(self.heading_deg)
        shaft_per_meter = self.robot["external_ratio"] / (2 * math.pi * self.radius)
        signs = (self.robot["left_mount_sign"], self.robot["right_mount_sign"])
        result = {
            "time_s": self.time_s, "x_mm": self.x_mm, "y_mm": self.y_mm,
            "heading_deg": self.heading_deg,
            "v_mm_s": (self.vx * math.sin(angle) + self.vy * math.cos(angle)) * 1000,
            "omega_deg_s": math.degrees(self.omega), "collision": self.collision,
            "collision_count": self.collision_count,
            "collision_elements": sorted(self.collision_elements),
            "elements": deepcopy(self.elements),
        }
        for index, side in enumerate(("left", "right")):
            result[side + "_rpm"] = self.wheel_speed[index] * shaft_per_meter * 60 * signs[index]
            result[side + "_position_deg"] = self.wheel_distance[index] * shaft_per_meter * 360 * signs[index]
        return result
