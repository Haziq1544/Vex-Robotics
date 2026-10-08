"""Behavior checks for anchored obstacles and inactive future stackers."""

from copy import deepcopy
import unittest

from simulator.collisions import polygon_contact, polygons_overlap
from simulator.config import default_config
from simulator.field_elements import element_polygon, element_spec, make_element
from simulator.physics import World


class ElementPhysicsTests(unittest.TestCase):
    def world(self, elements, **pose):
        config = default_config()
        config["layout"] = {"preset": "custom", "elements": elements}
        return World(config, {"x_mm": 0, "y_mm": -700, "heading_deg": 0, **pose})

    def run_for(self, world, duration, left=100, right=100):
        world.set_motor(9, {"percent": left, "brake": "brake"})
        world.set_motor(10, {"percent": -right, "brake": "brake"})
        touched = set()
        for _ in range(round(duration / 0.01)):
            world.step(0.01)
            touched.update(world.collision_elements)
        return touched

    def assert_clear(self, world, tolerance=0.02):
        polygons = [("robot", world.robot_corners())]
        polygons += [(e["id"], element_polygon(e)) for e in world.elements
                     if element_spec(e["kind"])["collidable"]]
        for index, (first_id, first) in enumerate(polygons):
            for second_id, second in polygons[index + 1:]:
                self.assertFalse(polygons_overlap(first, second, tolerance),
                                 first_id + " overlaps " + second_id)
        hw, hh = world.config["field"]["width_mm"] / 2, world.config["field"]["height_mm"] / 2
        for identity, polygon in polygons:
            self.assertTrue(all(abs(x) <= hw + tolerance and abs(y) <= hh + tolerance for x, y in polygon),
                            identity + " escaped field walls")

    def test_tangency_and_containment_are_distinguished(self):
        square = [(-10, -10), (10, -10), (10, 10), (-10, 10)]
        adjacent = [(x + 20, y) for x, y in square]
        contained = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        self.assertIsNone(polygon_contact(square, adjacent))
        self.assertTrue(polygons_overlap(square, contained))
        contact = polygon_contact(contained, square)
        self.assertAlmostEqual(contact[0], 11)
        moved = [(x + contact[1] * contact[0], y + contact[2] * contact[0]) for x, y in contained]
        self.assertFalse(polygons_overlap(square, moved))

    def test_start_overlap_rejected_with_element_id(self):
        with self.assertRaisesRegex(ValueError, "test_goal"):
            self.world([make_element("goal_center", element_id="test_goal")], y_mm=0)

    def test_full_power_cannot_cross_or_move_any_goal_or_loader(self):
        for kind in ("goal_center", "goal_neutral", "goal_red", "goal_blue", "loader"):
            with self.subTest(kind=kind):
                obstacle = make_element(kind, element_id="fixed_obstacle")
                world = self.world([obstacle])
                world.set_motor(9, {"percent": 100})
                world.set_motor(10, {"percent": -100})
                touched = set()
                stop_y = -element_spec(kind)["depth_mm"] / 2 - world.robot["body_length_mm"] / 2
                for _ in range(150):
                    world.step(0.02)
                    touched.update(world.collision_elements)
                    self.assertLessEqual(world.y_mm, stop_y + 0.02)
                    self.assertEqual(world.elements, [obstacle])
                    self.assert_clear(world)
                self.assertIn("fixed_obstacle", touched)
                self.assertAlmostEqual(world.y_mm, stop_y, delta=1)
                self.assertAlmostEqual(world.x_mm, 0, delta=1)
                self.assertAlmostEqual((world.heading_deg + 180) % 360 - 180, 0, delta=0.3)
                self.assertGreater(abs(world.snapshot()["left_position_deg"]), 500)

    def test_wall_mounted_loader_remains_fixed_during_off_centre_impact(self):
        config = default_config()
        half_width = config["field"]["width_mm"] / 2
        loader = make_element("loader", x_mm=-half_width + element_spec("loader")["width_mm"] / 2,
                              element_id="wall_loader")
        world = self.world([loader], x_mm=-half_width + config["robot"]["body_width_mm"] / 2)
        world.set_motor(9, {"percent": 100})
        world.set_motor(10, {"percent": -100})
        touched = set()
        for _ in range(150):
            world.step(0.02)
            touched.update(world.collision_elements)
            self.assertEqual(world.elements, [loader])
            self.assert_clear(world)
        self.assertIn("wall_loader", touched)

    def test_angled_goal_impact_changes_heading_without_penetrating(self):
        world = self.world([make_element("goal_neutral", element_id="goal")],
                           x_mm=-430, y_mm=-700, heading_deg=30)
        touched = self.run_for(world, 2)
        self.assertIn("goal", touched)
        self.assertGreater(abs((world.heading_deg - 30 + 180) % 360 - 180), 2)
        self.assert_clear(world)

    def test_close_parallel_pass_has_no_phantom_circle_collision(self):
        goal_half = element_spec("goal_neutral")["width_mm"] / 2
        world = self.world([make_element("goal_neutral", element_id="goal")],
                           x_mm=200 + goal_half + 5)
        touched = self.run_for(world, 2)
        self.assertNotIn("goal", touched)
        self.assertGreater(world.y_mm, 100)
        self.assertAlmostEqual(world.x_mm, 200 + goal_half + 5, places=5)

    def test_large_time_step_at_top_speed_cannot_tunnel_goal(self):
        world = self.world([make_element("goal_center", element_id="goal")])
        world.vy = world.free_speed
        world.wheel_speed[:] = [world.free_speed, world.free_speed]
        world.set_motor(9, {"percent": 100})
        world.set_motor(10, {"percent": -100})
        world.step(1.0)
        self.assertIn("goal", world.collision_elements)
        self.assertLess(world.y_mm, -250)
        self.assert_clear(world)

    def test_even_high_initial_velocity_is_substepped(self):
        world = self.world([make_element("loader", element_id="loader")])
        world.vy = 20.0  # Above motor capability, to stress the sweep limit.
        world.step(0.1)
        self.assertIn("loader", world.collision_elements)
        self.assertLess(world.y_mm, -200)
        self.assert_clear(world)

    def test_inactive_cup_is_absent_from_world_and_snapshot_but_retained_in_config(self):
        cup = make_element("cup", element_id="cup")
        original = deepcopy(cup)
        world = self.world([cup])
        touched = self.run_for(world, 1.7)
        self.assertNotIn("cup", touched)
        self.assertGreater(world.y_mm, 100)
        self.assertEqual(world.elements, [])
        self.assertEqual(world.snapshot()["elements"], [])
        self.assertEqual(world.config["layout"]["elements"], [original])
        self.assertEqual(cup, original)
        self.assert_clear(world)

    def test_active_element_snapshot_is_independent(self):
        goal = make_element("goal_center", element_id="goal")
        world = self.world([goal])
        state = world.snapshot()
        state["elements"][0]["x_mm"] = 999
        self.assertEqual(world.elements, [goal])

    def test_inactive_cup_does_not_interfere_with_goal_collision(self):
        goal = make_element("goal_center", y_mm=250, element_id="goal")
        world = self.world([make_element("cup", element_id="cup"), goal])
        touched = self.run_for(world, 3)
        self.assertNotIn("cup", touched)
        self.assertIn("goal", touched)
        self.assertEqual(world.elements, [goal])
        self.assertEqual(world.snapshot()["elements"], [goal])
        self.assertLess(world.y_mm, 250)
        self.assert_clear(world, tolerance=0.15)

    def test_saved_cups_remain_in_config_without_creating_physics_bodies(self):
        diameter = element_spec("cup")["width_mm"]
        cups = [make_element("cup", y_mm=i * diameter, element_id="cup" + str(i)) for i in range(3)]
        world = self.world(cups)
        self.run_for(world, 2)
        self.assertEqual(world.elements, [])
        self.assertEqual(world._element_bodies, [])
        self.assertEqual(world.config["layout"]["elements"], cups)
        self.assertGreater(world.y_mm, 240)
        self.assert_clear(world, tolerance=0.15)

    def test_robot_can_start_at_inactive_cup_position(self):
        world = self.world([make_element("cup", element_id="cup")], y_mm=0)
        self.assertEqual(world.elements, [])
        self.assertEqual(world.snapshot()["elements"], [])
        self.assertEqual(world._element_bodies, [])

    def test_rotating_rectangle_hits_nearby_obstacle(self):
        goal_half = element_spec("goal_neutral")["width_mm"] / 2
        world = self.world([make_element("goal_neutral", x_mm=200 + goal_half + 20, element_id="goal")], y_mm=0)
        touched = self.run_for(world, 1, left=100, right=-100)
        self.assertIn("goal", touched)
        self.assert_clear(world)

    def test_full_override_preset_is_stable_at_rest(self):
        config = default_config()
        config["layout"] = {"preset": "override", "elements": []}
        world = World(config, {"x_mm": 0, "y_mm": -1000, "heading_deg": 0})
        original = deepcopy(world.elements)
        self.run_for(world, 0.2, left=0, right=0)
        self.assertEqual(len(world.elements), 13)
        self.assertEqual(sum(element["kind"].startswith("goal_") for element in world.elements), 9)
        self.assertEqual(sum(element["kind"] == "loader" for element in world.elements), 4)
        self.assertNotIn("cup", {element["kind"] for element in world.snapshot()["elements"]})
        for before, after in zip(original, world.elements):
            self.assertAlmostEqual(before["x_mm"], after["x_mm"], places=4)
            self.assertAlmostEqual(before["y_mm"], after["y_mm"], places=4)
        self.assertFalse(world.collision)
        self.assert_clear(world)


if __name__ == "__main__":
    unittest.main()
