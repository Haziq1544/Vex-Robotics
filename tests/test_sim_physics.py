"""Behavior checks for the desktop simulator (no VEX hardware required)."""

import json
import math
from pathlib import Path
import tempfile
import unittest

from simulator.config import default_config, load_config, validate_config
from simulator.physics import World


class PhysicsTests(unittest.TestCase):
    def world(self, **pose):
        return World(default_config(), {"x_mm": 0, "y_mm": 0, "heading_deg": 0, **pose})

    def drive(self, world, left, right):
        world.set_motor(9, {"percent": left, "brake": "brake"})
        world.set_motor(10, {"percent": -right, "brake": "brake"})

    def run_for(self, world, duration):
        for _ in range(round(duration / 0.01)):
            world.step(0.01)

    def test_equal_wheels_drive_straight_with_finite_acceleration(self):
        world = self.world(y_mm=-1000)
        self.drive(world, 50, 50)
        world.step(0.005)
        self.assertLess(world.snapshot()["v_mm_s"], 50)
        self.run_for(world, 2)
        self.assertAlmostEqual(world.x_mm, 0, places=6)
        self.assertAlmostEqual(world.heading_deg, 0, places=6)
        self.assertGreater(world.y_mm, -300)
        self.assertLess(world.snapshot()["v_mm_s"], 550)
        self.assertGreater(world.snapshot()["left_position_deg"], 0)
        self.assertLess(world.snapshot()["right_position_deg"], 0)

    def test_equal_opposite_wheels_rotate_clockwise_without_translation(self):
        world = self.world()
        self.drive(world, 20, -20)
        self.run_for(world, 1)
        self.assertGreater(world.heading_deg, 20)
        self.assertLess(world.heading_deg, 90)
        self.assertAlmostEqual(world.x_mm, 0, places=5)
        self.assertAlmostEqual(world.y_mm, 0, places=5)

    def test_heading_90_drives_positive_x(self):
        world = self.world(heading_deg=90)
        self.drive(world, 30, 30)
        self.run_for(world, 1)
        self.assertGreater(world.x_mm, 150)
        self.assertAlmostEqual(world.y_mm, 0, places=5)

    def test_motor_reversal_mistake_is_visible(self):
        world = self.world()
        world.set_motor(9, {"percent": 30})
        world.set_motor(10, {"percent": 30})
        self.run_for(world, 1)
        self.assertLess(abs(world.x_mm) + abs(world.y_mm), 0.1)
        self.assertGreater(world.heading_deg, 30)

    def test_brake_stops_in_less_distance_than_coast(self):
        distances = []
        for mode in ("brake", "coast"):
            world = self.world(y_mm=-1000)
            self.drive(world, 50, 50)
            self.run_for(world, 1)
            start = world.y_mm
            for port in (9, 10):
                world.set_motor(port, {"percent": 0, "brake": mode})
            self.run_for(world, 1)
            distances.append(world.y_mm - start)
        self.assertGreater(distances[0], 0)
        self.assertLess(distances[0], distances[1] / 3)

    def test_lower_traction_reduces_acceleration_and_increases_slip(self):
        results = []
        for mu in (0.8, 0.02):
            config = default_config()
            config["physics"]["traction_mu"] = mu
            world = World(config, {"x_mm": 0, "y_mm": 0, "heading_deg": 0})
            self.drive(world, 100, 100)
            self.run_for(world, 0.3)
            state = world.snapshot()
            wheel_travel = state["left_position_deg"] / 360 * math.pi * config["robot"]["wheel_diameter_mm"]
            results.append((world.y_mm, wheel_travel - world.y_mm))
        self.assertLess(results[1][0], results[0][0] / 3)
        self.assertGreater(results[1][1], results[0][1])

    def test_robot_rectangle_cannot_pass_through_wall_at_an_angle(self):
        world = self.world(x_mm=1400, y_mm=1400, heading_deg=45)
        self.drive(world, 100, 100)
        collided = False
        for _ in range(500):
            world.step(0.01)
            collided |= world.collision
            for x, y in world.robot_corners():
                self.assertLessEqual(abs(x), world.config["field"]["width_mm"] / 2 + 1e-5)
                self.assertLessEqual(abs(y), world.config["field"]["height_mm"] / 2 + 1e-5)
        self.assertTrue(collided)
        self.assertGreater(world.snapshot()["left_position_deg"], 300)

    def test_larger_step_subdivision_matches_small_steps(self):
        worlds = [self.world() for _ in range(2)]
        for world in worlds:
            self.drive(world, 30, 20)
        worlds[0].step(0.5)
        self.run_for(worlds[1], 0.5)
        self.assertLess(abs(worlds[0].x_mm - worlds[1].x_mm), 1)
        self.assertLess(abs(worlds[0].y_mm - worlds[1].y_mm), 1)
        self.assertLess(abs(worlds[0].heading_deg - worlds[1].heading_deg), 0.1)

    def test_external_reduction_reduces_wheel_speed(self):
        speeds = []
        for ratio in (1, 2):
            config = default_config()
            config["robot"]["external_ratio"] = ratio
            world = World(config, {"x_mm": 0, "y_mm": -1200, "heading_deg": 0})
            self.drive(world, 50, 50)
            self.run_for(world, 2)
            speeds.append(world.snapshot()["v_mm_s"])
        self.assertAlmostEqual(speeds[0] / speeds[1], 2, delta=0.1)

    def test_invalid_configuration_and_start_rejected(self):
        for section, name, value in (("robot", "mass_kg", 0), ("robot", "wheel_diameter_mm", -2),
                                     ("physics", "restitution", 2), ("gps", "sample_hz", float("nan")),
                                     ("simulation", "step_ms", 100), ("robot", "left_port", 10)):
            config = default_config()
            config[section][name] = value
            with self.assertRaises(ValueError):
                validate_config(config)
        with self.assertRaises(ValueError):
            self.world(x_mm=1780)
        with self.assertRaises(NotImplementedError):
            self.world().set_motor(11, {"percent": 20})

    def test_partial_override_load_and_independent_defaults(self):
        first = default_config()
        first["robot"]["mass_kg"] = 10
        self.assertNotEqual(first, default_config())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot.json"
            path.write_text(json.dumps({"robot": {"mass_kg": 8.3}}), encoding="utf-8")
            loaded = load_config(path)
            self.assertEqual(loaded["robot"]["mass_kg"], 8.3)
            self.assertEqual(loaded["robot"]["left_port"], 9)
            path.write_text('{"robot": {"mass_kgg": 8.3}}', encoding="utf-8")
            with self.assertRaises(ValueError):
                load_config(path)


if __name__ == "__main__":
    unittest.main()
