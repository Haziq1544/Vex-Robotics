"""Override drawing measurements, editable layout persistence, and real-code contact."""
from collections import Counter
from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import unittest

from simulate import run_headless
from simulator.config import default_config, load_config, validate_config
from simulator.field_elements import (
    element_polygon, element_spec, make_element, materialize_layout,
    resolve_elements, set_preset,
)


PROGRAM = Path(__file__).resolve().parents[1] / "src" / "main.py"


class OverrideLayoutTests(unittest.TestCase):
    def setUp(self):
        self.config = default_config()
        set_preset(self.config, "override")

    def test_preset_has_all_retained_objects_and_no_pins(self):
        elements = resolve_elements(self.config)
        counts = Counter(element["kind"] for element in elements)
        self.assertEqual(counts, {"goal_center": 1, "goal_neutral": 4,
                                  "goal_red": 2, "goal_blue": 2,
                                  "loader": 4, "cup": 36})
        self.assertEqual(len({element["id"] for element in elements}), 49)
        # Also exercises all-pair overlap and field-boundary validation. Cups
        # deliberately touch their neighbours and walls in this valid preset.
        validate_config(self.config)

    def test_goals_match_the_audience_view_and_centre_origin(self):
        elements = resolve_elements(self.config)
        expected = {
            "goal_center": {(0.0, 0.0)},
            "goal_neutral": {(-1196.1, 598.1), (-598.1, 1196.1),
                             (1196.1, -598.1), (598.1, -1196.1)},
            "goal_red": {(-1196.1, -598.1), (-598.1, -1196.1)},
            "goal_blue": {(598.1, 1196.1), (1196.1, 598.1)},
        }
        for kind, points in expected.items():
            with self.subTest(kind=kind):
                self.assertEqual({(e["x_mm"], e["y_mm"]) for e in elements
                                  if e["kind"] == kind}, points)

    def test_goals_preserve_flat_width_and_rounded_corner_extent(self):
        for kind, height in (("goal_center", 222.7), ("goal_neutral", 146.5),
                             ("goal_red", 82.5), ("goal_blue", 82.5)):
            with self.subTest(kind=kind):
                outline = element_polygon(make_element(kind))
                self.assertAlmostEqual(max(x for x, _ in outline), 71.25)
                self.assertAlmostEqual(min(x for x, _ in outline), -71.25)
                self.assertAlmostEqual(max(y for _, y in outline), 71.25)
                self.assertAlmostEqual(min(y for _, y in outline), -71.25)
                self.assertAlmostEqual(max(math.hypot(x, y) for x, y in outline), 81.9)
                self.assertAlmostEqual(element_spec(kind)["height_mm"], height)
                self.assertFalse(element_spec(kind)["movable"])

    def test_cup_footprints_faces_and_perimeter_groups(self):
        cups = [e for e in resolve_elements(self.config) if e["kind"] == "cup"]
        self.assertEqual(Counter(cup["face"] for cup in cups), {"clear": 12, "opaque": 24})
        clear_points = {(cup["x_mm"], cup["y_mm"]) for cup in cups if cup["face"] == "clear"}
        self.assertEqual(clear_points, {
            (-1196.1, -1196.1), (-1196.1, 1196.1), (1196.1, -1196.1), (1196.1, 1196.1),
            (-598.1, -598.1), (-598.1, 598.1), (598.1, -598.1), (598.1, 598.1),
            (-598.1, 0), (598.1, 0), (0, -598.1), (0, 598.1),
        })
        half_w = self.config["field"]["width_mm"] / 2
        half_h = self.config["field"]["height_mm"] / 2
        for cup in cups:
            for x, y in element_polygon(cup):
                self.assertAlmostEqual(math.hypot(x - cup["x_mm"], y - cup["y_mm"]), 40.1)
            if cup["face"] == "opaque":
                distance_to_wall = min(half_w - abs(cup["x_mm"]), half_h - abs(cup["y_mm"]))
                self.assertAlmostEqual(distance_to_wall, 40.1)
                along = cup["x_mm"] if abs(cup["y_mm"]) > 1700 else cup["y_mm"]
                self.assertTrue(any(abs(along - value) < 1e-6 for value in
                                    (-678.3, -598.1, -517.9, 517.9, 598.1, 678.3)))
        cup_spec = element_spec("cup")
        self.assertEqual(cup_spec["height_mm"], 164.5)
        self.assertFalse(cup_spec["movable"])
        self.assertFalse(cup_spec["collidable"])
        self.assertNotIn(cup_spec["color"], (element_spec("goal_red")["color"],
                                          element_spec("goal_blue")["color"]))

    def test_loaders_are_mounted_at_the_walls_with_measured_inward_depth(self):
        half_w = self.config["field"]["width_mm"] / 2
        half_h = self.config["field"]["height_mm"] / 2
        for loader in (e for e in resolve_elements(self.config) if e["kind"] == "loader"):
            outline = element_polygon(loader)
            self.assertAlmostEqual(abs(loader["x_mm"]), half_w - 47.5)
            self.assertAlmostEqual(abs(loader["y_mm"]), half_h - 290.6)
            self.assertAlmostEqual(max(abs(x) for x, _ in outline), half_w)
            self.assertAlmostEqual(max(x for x, _ in outline) - min(x for x, _ in outline), 95.0)
            self.assertAlmostEqual(max(y for _, y in outline) - min(y for _, y in outline), 102.1)

    def test_wall_mounted_objects_follow_wall_without_scaling_interior_goals(self):
        before = resolve_elements(self.config)
        self.config["field"].update(width_mm=4000.0, height_mm=4200.0)
        after = resolve_elements(self.config)
        for first, second in zip(before, after):
            if first["kind"].startswith("goal_"):
                self.assertEqual(first, second)
            elif first["kind"] == "loader":
                self.assertAlmostEqual(abs(second["x_mm"]), 1952.5)
                self.assertAlmostEqual(abs(second["y_mm"]), 1809.4)
        validate_config(self.config)

    def test_generated_layouts_and_specs_are_independent(self):
        original = resolve_elements(self.config)
        altered = resolve_elements(self.config)
        altered[0]["x_mm"] = 123.0
        altered.pop()
        self.assertEqual(resolve_elements(self.config), original)
        spec = element_spec("cup")
        spec["width_mm"] = 999.0
        self.assertEqual(element_spec("cup")["width_mm"], 80.2)
        untouched_defaults = default_config()
        self.config["field"]["width_mm"] = 9999.0
        self.assertEqual(default_config(), untouched_defaults)

    def test_custom_edit_reset_and_empty_preset(self):
        original = resolve_elements(self.config)
        materialize_layout(self.config)
        self.assertEqual(self.config["layout"]["preset"], "custom")
        self.config["layout"]["elements"][0].update(x_mm=150.0, heading_deg=45.0)
        detached = resolve_elements(self.config)
        detached[0]["x_mm"] = 999.0
        self.assertEqual(self.config["layout"]["elements"][0]["x_mm"], 150.0)
        set_preset(self.config, "override")
        self.assertEqual(resolve_elements(self.config), original)
        set_preset(self.config, "empty")
        self.assertEqual(resolve_elements(self.config), [])
        set_preset(self.config, "custom")
        self.assertEqual(self.config["layout"], {"preset": "custom", "elements": []})


class LayoutPersistenceAndValidationTests(unittest.TestCase):
    def setUp(self):
        self.config = default_config()
        self.config["layout"] = {"preset": "custom", "elements": [
            make_element("goal_red", -400.5, 300.25, 27.5, "fixed_goal"),
            dict(make_element("cup", 625.0, -456.0, 13.0, "loose_cup"), face="opaque"),
        ]}

    def test_saved_custom_layout_preserves_element_poses_and_cup_face(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "custom_field.json"
            path.write_text(json.dumps(self.config), encoding="utf-8")
            loaded = load_config(path)
        self.assertEqual(loaded, self.config)
        self.assertEqual(resolve_elements(loaded), self.config["layout"]["elements"])
        loaded["layout"]["elements"][0]["x_mm"] = 0
        self.assertEqual(self.config["layout"]["elements"][0]["x_mm"], -400.5)

    def test_legacy_partial_configuration_keeps_the_empty_field(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "old_settings.json"
            path.write_text('{"robot": {"mass_kg": 7.25}, "gps": {"latency_ms": 60}}', encoding="utf-8")
            loaded = load_config(path)
        self.assertEqual(loaded["robot"]["mass_kg"], 7.25)
        self.assertEqual(loaded["gps"]["latency_ms"], 60)
        self.assertEqual(resolve_elements(loaded), [])

    def test_nonfinite_or_boolean_element_pose_is_rejected(self):
        for key in ("x_mm", "y_mm", "heading_deg"):
            for invalid in (float("nan"), float("inf"), -float("inf"), True):
                with self.subTest(key=key, invalid=invalid):
                    config = deepcopy(self.config)
                    config["layout"]["elements"][0][key] = invalid
                    with self.assertRaisesRegex(ValueError, "finite number"):
                        validate_config(config)

    def test_unknown_kind_duplicate_ids_and_invalid_faces_are_rejected(self):
        for key, value, message in (("kind", "pin", "Unknown field element"),
                                     ("id", "loose_cup", "unique"),
                                     ("face", "red", "face")):
            with self.subTest(key=key):
                config = deepcopy(self.config)
                config["layout"]["elements"][0][key] = value
                with self.assertRaisesRegex(ValueError, message):
                    validate_config(config)

    def test_overlapping_elements_and_rotated_out_of_bounds_are_rejected(self):
        config = deepcopy(self.config)
        config["layout"]["elements"][1].update(x_mm=-400.5, y_mm=300.25)
        with self.assertRaisesRegex(ValueError, "overlaps"):
            validate_config(config)
        config = deepcopy(self.config)
        # At 45 degrees the rounded corner extends beyond the wall although
        # an unrotated flat face would fit. Validate the actual footprint.
        config["layout"]["elements"][0].update(
            x_mm=config["field"]["width_mm"] / 2 - 77.0,
            y_mm=0.0, heading_deg=45.0)
        with self.assertRaisesRegex(ValueError, "inside the field"):
            validate_config(config)

    def test_invalid_layout_structure_and_unknown_preset_are_rejected(self):
        for layout in ({"preset": "missing", "elements": []},
                       {"preset": "custom", "elements": {}},
                       {"preset": "custom", "elements": [None]},
                       {"preset": "empty", "elements": [make_element("cup")]}):
            with self.subTest(layout=layout):
                config = deepcopy(self.config)
                config["layout"] = layout
                with self.assertRaises(ValueError):
                    validate_config(config)


class OverrideProgramExecutionTests(unittest.TestCase):
    def test_actual_centre_program_stops_at_an_obstacle_without_moving_goals(self):
        config = default_config()
        set_preset(config, "override")
        initial = {e["id"]: e for e in resolve_elements(config)
                   if not element_spec(e["kind"])["movable"]}
        result = run_headless(config, PROGRAM,
                              {"x_mm": -900, "y_mm": -900, "heading_deg": 0},
                              duration=20, gps_mode="ideal")
        self.assertIsNone(result["result"]["error"])
        snapshot = result["snapshot"]
        self.assertEqual(snapshot["status"], "Stopped: no progress")
        self.assertGreater(snapshot["world"]["collision_count"], 0)
        self.assertGreater(math.hypot(snapshot["world"]["x_mm"], snapshot["world"]["y_mm"]), 100)
        for element in snapshot["world"]["elements"]:
            if element["id"] in initial:
                for key in ("x_mm", "y_mm", "heading_deg"):
                    self.assertEqual(element[key], initial[element["id"]][key])
        self.assertLess(snapshot["time_s"], 20)


if __name__ == "__main__":
    unittest.main()
