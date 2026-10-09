"""AprilTag visibility, VEX compatibility and collision-free approach scenarios.

The steering mode and 50 mm gap below belong to temporary validation programs.
They do not choose defaults or modify the user's robot source.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
import re
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from simulate import main as launch_simulator, run_headless
from simulator.config import default_config
from simulator.field_elements import make_element, set_preset
from simulator.runtime import _Runtime, build_vex_module, SimulatorSession, UnsupportedVexAPI
from simulator.vision import VisionModel, vex_object
from simulator.program import read_program, VISION_KIND


PROGRAM = Path(__file__).resolve().parents[1] / "src" / "main.py"


def camera_world(x=0, y=-800, heading=0, elements=None):
    return SimpleNamespace(x_mm=x, y_mm=y, heading_deg=heading, time_s=0.0,
                           elements=elements if elements is not None else
                           [make_element("goal_center", element_id="target")])


def validation_source(mode="steer", gap=50):
    source = PROGRAM.read_text(encoding="utf-8")
    source = re.sub(r"^APPROACH_MODE = .*", "APPROACH_MODE = " + repr(mode), source, flags=re.MULTILINE)
    source = re.sub(r"^STOP_CLEARANCE_MM = .*", "STOP_CLEARANCE_MM = " + str(gap), source, flags=re.MULTILINE)
    return source


class VisionGeometryTests(unittest.TestCase):
    def test_tags_have_projected_corners_and_only_facing_faces_are_seen(self):
        model = VisionModel(camera_world(), default_config()["vision"])
        tags = model._observe()
        self.assertEqual(len(tags), 1)
        self.assertEqual(tags[0]["id"], 0)
        self.assertEqual(tags[0]["centerX"], 160)
        self.assertEqual(tags[0]["face"], 2)
        self.assertEqual(len(tags[0]["corners_x"]), 4)
        model.world.heading_deg = 45
        self.assertEqual(model._observe(), [])
        model.world.heading_deg = 180
        self.assertEqual(model._observe(), [])

    def test_perspective_size_range_and_image_clipping_limit_visibility(self):
        config = default_config()["vision"]
        model = VisionModel(camera_world(), config)
        distant_width = model._observe()[0]["width"]
        model.world.y_mm = -400
        self.assertGreater(model._observe()[0]["width"], distant_width)
        config["far_mm"] = 100
        self.assertEqual(model._observe(), [])
        config["far_mm"] = 2500
        config["minimum_tag_pixels"] = 100
        self.assertEqual(model._observe(), [])
        config["minimum_tag_pixels"] = 4
        model.world.elements = [make_element("goal_red", element_id="low_tag")]
        model.world.y_mm = -800
        self.assertTrue(model._observe())
        model.world.y_mm = -300
        self.assertEqual(model._observe(), [], "Low tags must leave the vertical image when too close")

    def test_loader_occludes_goal_without_becoming_a_tag_target(self):
        world = camera_world()
        model = VisionModel(world, default_config()["vision"])
        self.assertTrue(model._observe())
        world.elements.append(make_element("loader", 0, -400, element_id="blocker"))
        self.assertEqual(model._observe(), [])
        world.elements[-1]["x_mm"] = 300
        self.assertEqual([tag["id"] for tag in model._observe()], [0])
        world.elements = [world.elements[-1]]
        self.assertEqual(model._observe(), [])

    def test_oblique_view_can_see_two_faces_with_same_tag_id(self):
        model = VisionModel(camera_world(-500, -500, 45), default_config()["vision"])
        tags = model._observe()
        self.assertEqual([tag["id"] for tag in tags], [0, 0])
        self.assertEqual(len({tag["face"] for tag in tags}), 2)
        self.assertEqual([tag["width"] for tag in tags], sorted([tag["width"] for tag in tags], reverse=True))

    def test_sample_latency_and_injected_loss_do_not_reveal_current_truth(self):
        config = default_config()["vision"]
        config["latency_ms"] = 100
        model = VisionModel(camera_world(), config)
        model.update()
        self.assertEqual(model.read(), [])
        model.world.time_s = .1
        model.update()
        self.assertTrue(model.read())
        model.world.heading_deg = 180
        model.world.time_s = .15
        model.update()
        self.assertTrue(model.read(), "The new empty image must wait for camera latency")
        model.world.time_s = .26
        model.update()
        self.assertEqual(model.read(), [])
        self.assertEqual(model.timestamp_ms, 150)
        model.world.heading_deg = 0
        model.world.time_s = .4
        model.update()
        model.world.time_s = .51
        model.update()
        self.assertTrue(model.read())
        model.loss_until = 1
        self.assertEqual(model.read(), [])


class VisionAdapterTests(unittest.TestCase):
    def setUp(self):
        config = default_config()
        config["layout"] = {"preset": "custom", "elements": [make_element("goal_center", element_id="target")]}
        self.runtime = _Runtime(config, {"x_mm": 0, "y_mm": -800, "heading_deg": 0}, "ideal")
        self.vex = build_vex_module(self.runtime)

    def test_only_standard_image_measurements_reach_robot_code(self):
        item = self.runtime.vision._observe()[0]
        self.assertIn("element_id", item)
        obj = vex_object(item)
        self.assertEqual(set(vars(obj)), {"id", "originX", "originY", "width", "height",
                                         "centerX", "centerY", "angle", "exists", "type", "score", "area", "tag"})
        self.assertEqual(set(vars(obj.tag)), {"x", "y"})
        self.assertIsInstance(obj.tag.x, tuple)
        self.assertEqual(obj.type, 8)

    def test_snapshot_tuple_filter_count_empty_and_timestamp(self):
        vex = self.vex.__dict__
        camera = vex["AiVision"](None, vex["AiVision"].ALL_TAGS)
        self.assertEqual(camera.take_snapshot(camera.ALL_TAGS), ())
        self.assertIsNone(camera.largest_object())
        item = self.runtime.vision._observe()[0]
        second = deepcopy(item)
        second.update(id=2, width=item["width"] - 1)
        self.runtime.vision.objects = [item, second]
        self.runtime.vision.timestamp_ms = 123
        self.assertEqual(camera.timestamp(), 123)
        tags = camera.take_snapshot(camera.ALL_TAGS, 1)
        self.assertIsInstance(tags, tuple)
        self.assertEqual(camera.object_count(), 1)
        self.assertIs(camera.largest_object(), tags[0])
        self.assertEqual([obj.id for obj in camera.take_snapshot(vex["Tagdesc"](2))], [2])
        self.assertEqual(camera.take_snapshot(vex["Tagdesc"](3)), ())
        camera.tag_detection(False)
        self.assertEqual(camera.take_snapshot(camera.ALL_TAGS), ())
        self.runtime.vision.connected = False
        self.assertFalse(camera.installed())

    def test_camera_does_not_claim_existing_ports_or_unsupported_detection(self):
        camera_type = self.vex.__dict__["AiVision"]
        for port in (9, 10, 18, 0, 22):
            with self.subTest(port=port), self.assertRaises(ValueError):
                camera_type(port)
        camera = camera_type(None)
        with self.assertRaises(UnsupportedVexAPI):
            camera.set_tag_family(camera.TAG_36H11)
        with self.assertRaises(UnsupportedVexAPI):
            camera.take_snapshot(1)
        with self.assertRaises(ValueError):
            camera.take_snapshot(camera.ALL_TAGS, 25)

    def test_real_device_with_unset_port_brakes_without_creating_camera(self):
        self.vex.__dict__["AiVision"].SIMULATED = False
        namespace = {"__name__": "vision_validation"}
        with patch.dict(sys.modules, {"vex": self.vex}):
            exec(compile(PROGRAM.read_text(encoding="utf-8"), str(PROGRAM), "exec"), namespace)
            namespace["run_test"]()
        self.assertEqual(namespace["run_status"], "Stopped: set AI_VISION_PORT")
        self.assertFalse(self.runtime.vision_used)
        self.assertTrue(all(command == {"percent": 0, "brake": "brake"} for command in self.runtime.world.commands))

    def test_pose_fit_accepts_different_corner_start_and_winding(self):
        namespace = {"__name__": "vision_validation"}
        with patch.dict(sys.modules, {"vex": self.vex}):
            exec(compile(PROGRAM.read_text(encoding="utf-8"), str(PROGRAM), "exec"), namespace)
        item = self.runtime.vision._observe()[0]
        reference = namespace["observed_goal"](vex_object(item))
        for reverse in (False, True):
            for shift in range(4):
                with self.subTest(reverse=reverse, shift=shift):
                    candidate = deepcopy(item)
                    for key in ("corners_x", "corners_y"):
                        values = list(reversed(item[key])) if reverse else list(item[key])
                        candidate[key] = values[shift:] + values[:shift]
                    result = namespace["observed_goal"](vex_object(candidate))
                    self.assertAlmostEqual(result[0], reference[0], places=5)
                    self.assertAlmostEqual(result[1], reference[1], places=5)
                    self.assertAlmostEqual(namespace["clearance"](result[2]), namespace["clearance"](reference[2]), places=5)


class VisionProgramTests(unittest.TestCase):
    def test_normal_launcher_executes_real_vision_source_and_locks_noncentral_goal(self):
        # No replacement source and no mocked runtime: this catches main.py
        # reverting to the GPS program while the launch path stays unchanged.
        with patch("builtins.print") as printed:
            self.assertEqual(launch_simulator(["--headless", "--duration", "1", "--gps-mode", "ideal"]), 0)
        result = json.loads(printed.call_args.args[0])
        snapshot = result["snapshot"]
        self.assertIsNone(result["result"]["error"])
        self.assertEqual(snapshot["program"]["kind"], VISION_KIND)
        self.assertIsNone(snapshot["target"])
        self.assertIsNotNone(snapshot["vision"])
        self.assertEqual(snapshot["vision_tracking"]["id"], 4)
        self.assertEqual(snapshot["status"], "Approaching first goal")
        self.assertIn("Locked first visible goal: tag 4", snapshot["logs"])
        self.assertEqual(snapshot["world"]["collision_count"], 0)

    def test_gps_source_named_main_is_detected_and_rejected_for_default_launch(self):
        gps_source = (PROGRAM.parents[1] / "examples" / "gps_centre_test.py").read_text(encoding="utf-8")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "main.py"
            path.write_text(gps_source, encoding="utf-8")
            self.assertEqual(read_program(path)[1]["kind"], "coordinate_target")
            with patch("simulator.program.DEFAULT_PROGRAM", path.resolve()):
                with self.assertRaisesRegex(ValueError, "not the AI Vision"):
                    read_program(path, require_default_vision=True)

    def test_normal_launcher_uses_vision_and_preserves_requested_pose(self):
        with patch("simulate.run_headless", return_value={"result": {"error": None}}) as run, patch("builtins.print"):
            self.assertEqual(launch_simulator(["--headless", "--start-x", "400", "--start-y", "-700", "--heading", "15"]), 0)
        config, program, pose = run.call_args.args[:3]
        self.assertEqual(program, PROGRAM)
        self.assertEqual(config["layout"]["preset"], "override")
        self.assertEqual(pose, {"x_mm": 400, "y_mm": -700, "heading_deg": 15})

    def test_noncentral_goal_ids_from_translated_and_rotated_starts(self):
        # Each goal is away from the origin and each camera faces a different
        # direction. Targeting (0, 0), or accepting only ID 0, cannot pass this.
        scenarios = (("goal_neutral", 1, 650, 450, 90),
                     ("goal_neutral", 4, -700, 400, 180),
                     ("goal_red", 2, -500, -600, 270),
                     ("goal_blue", 3, 650, 400, 35))
        with tempfile.TemporaryDirectory() as directory:
            program = Path(directory) / "vision_validation.py"
            program.write_text(validation_source(), encoding="utf-8")
            for kind, tag_id, goal_x, goal_y, heading in scenarios:
                with self.subTest(kind=kind, tag_id=tag_id, heading=heading):
                    config = default_config()
                    target = make_element(kind, goal_x, goal_y, heading, element_id="noncentral")
                    target["tag_id"] = tag_id
                    config["layout"] = {"preset": "custom", "elements": [target]}
                    radians = math.radians(heading)
                    pose = {"x_mm": goal_x - 800 * math.sin(radians),
                            "y_mm": goal_y - 800 * math.cos(radians), "heading_deg": heading}
                    result = run_headless(config, program, pose, duration=20)
                    self.assertIsNone(result["result"]["error"])
                    snapshot, world = result["snapshot"], result["snapshot"]["world"]
                    self.assertIn("Tag: " + str(tag_id), snapshot["brain_lines"])
                    self.assertIsNone(snapshot["target"], "Vision must not publish a GPS coordinate target")
                    remaining = math.hypot(goal_x - world["x_mm"], goal_y - world["y_mm"])
                    self.assertLess(remaining, 600, "The robot must actually approach the noncentral goal")
                    self.assertEqual(world["collision_count"], 0)
                    self.assertTrue(snapshot["braking_settled"])
                    self.assertLess(abs(world["v_mm_s"]), 1)
                    # Square-on approaches to shorter goals lose the tag below
                    # the estimated level camera before the requested gap.
                    # Verify the real loss response instead of inventing vision.
                    self.assertEqual(snapshot["status"], "Stopped: target lost")
                    self.assertNotIn(tag_id, [item["id"] for item in snapshot["vision"]["objects"]])

    def test_first_visible_noncentral_goal_wins_with_centre_tag_also_visible(self):
        config = default_config()
        target = make_element("goal_neutral", 600, 500, element_id="first")
        centre = make_element("goal_center", 0, 850, element_id="other")
        config["layout"] = {"preset": "custom", "elements": [centre, target]}
        pose = {"x_mm": 400, "y_mm": -300, "heading_deg": 0}
        model = VisionModel(camera_world(400, -300, 0, [centre, target]), config["vision"])
        visible = model._observe()
        self.assertEqual(visible[0]["id"], 1)
        self.assertIn(0, [tag["id"] for tag in visible])
        with tempfile.TemporaryDirectory() as directory:
            program = Path(directory) / "vision_validation.py"
            program.write_text(validation_source(), encoding="utf-8")
            result = run_headless(config, program, pose, duration=20)
        self.assertIsNone(result["result"]["error"])
        snapshot, world = result["snapshot"], result["snapshot"]["world"]
        self.assertIn("Tag: 1", snapshot["brain_lines"])
        self.assertEqual(snapshot["status"], "Arrived: stopped before goal")
        self.assertEqual(world["collision_count"], 0)
        self.assertGreater(world["x_mm"], pose["x_mm"], "The selected goal is rightward, away from the centre tag")

    def test_first_centre_goal_approach_stays_clear_after_braking(self):
        before = PROGRAM.read_bytes()
        config = default_config()
        set_preset(config, "override")
        with tempfile.TemporaryDirectory() as directory:
            program = Path(directory) / "vision_validation.py"
            program.write_text(validation_source(), encoding="utf-8")
            for x, y, heading in ((0, -800, 0), (-500, -500, 45)):
                with self.subTest(x=x, y=y, heading=heading):
                    result = run_headless(config, program, {"x_mm": x, "y_mm": y, "heading_deg": heading}, duration=20)
                    self.assertIsNone(result["result"]["error"])
                    snapshot, world = result["snapshot"], result["snapshot"]["world"]
                    self.assertEqual(snapshot["status"], "Arrived: stopped before goal")
                    self.assertIn("Tag: 0", snapshot["brain_lines"])
                    self.assertEqual(world["collision_count"], 0)
                    self.assertTrue(snapshot["braking_settled"])
                    self.assertLess(abs(world["v_mm_s"]), 1)
                    self.assertLess(abs(world["omega_deg_s"]), .1)
                    if x == 0:
                        true_front_gap = -world["y_mm"] - config["robot"]["body_length_mm"] / 2 - 71.25
                        self.assertGreater(true_front_gap, 40)
                        self.assertLess(true_front_gap, 80)
        self.assertEqual(PROGRAM.read_bytes(), before)

    def test_no_visible_tag_calibrates_gps_before_search_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            program = Path(directory) / "vision_validation.py"
            program.write_text(validation_source(), encoding="utf-8")
            pose = {"x_mm": 0, "y_mm": -800, "heading_deg": 0}
            result = run_headless(default_config(), program, pose, duration=1)
            snapshot = result["snapshot"]
            self.assertEqual(snapshot["status"], "Calibrating GPS for search")
            self.assertEqual(snapshot["vision_search"]["swept_deg"], 0)
            for key, value in pose.items():
                self.assertEqual(snapshot["world"][key], value)

    def test_straight_mode_declines_an_off_axis_target(self):
        config = default_config()
        set_preset(config, "override")
        with tempfile.TemporaryDirectory() as directory:
            program = Path(directory) / "vision_validation.py"
            program.write_text(validation_source("straight"), encoding="utf-8")
            pose = {"x_mm": -300, "y_mm": -800, "heading_deg": 0}
            result = run_headless(config, program, pose, duration=2)
            self.assertEqual(result["snapshot"]["status"], "Stopped: goal is not straight ahead")
            for key, value in pose.items():
                self.assertEqual(result["snapshot"]["world"][key], value)

    def test_lost_target_brakes_and_does_not_select_another_goal(self):
        config = default_config()
        set_preset(config, "override")
        with tempfile.TemporaryDirectory() as directory:
            program = Path(directory) / "vision_validation.py"
            program.write_text(validation_source(), encoding="utf-8")
            session = SimulatorSession(config, program, {"x_mm": 0, "y_mm": -800, "heading_deg": 0},
                                       realtime=True, speed=2, gps_mode="ideal", duration=8)
            latest, injected, exited = None, False, False
            session.start()
            try:
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline and not exited:
                    for message in session.poll():
                        if message["type"] == "snapshot":
                            latest = message
                            if not injected and message["status"] == "Approaching first goal" and message["time_s"] > .3:
                                session.inject_vision_loss(2)
                                injected = True
                        elif message["type"] == "exit":
                            self.assertIsNone(message["error"])
                            exited = True
                    time.sleep(.01)
                self.assertTrue(exited, "Vision loss did not finish the test")
                self.assertTrue(injected)
                self.assertIsNotNone(latest)
                self.assertEqual(latest["status"], "Stopped: target lost")
                self.assertIn("Tag: 0", latest["brain_lines"])
                self.assertEqual(latest["world"]["collision_count"], 0)
                self.assertTrue(latest["braking_settled"])
                self.assertLess(abs(latest["world"]["v_mm_s"]), 1)
                self.assertLess(abs(latest["world"]["omega_deg_s"]), .1)
            finally:
                session.stop()


if __name__ == "__main__":
    unittest.main()
