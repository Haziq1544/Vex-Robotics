"""GPS-guided initial scanning exercised through the real robot program."""
import math
from pathlib import Path
import time
import unittest

from simulate import run_headless
from simulator.config import default_config
from simulator.field_elements import make_element
from simulator.runtime import SimulatorSession
from simulator.vision import VisionModel
from types import SimpleNamespace


PROGRAM = Path(__file__).resolve().parents[1] / "src" / "main.py"


class GpsVisionSearchTests(unittest.TestCase):
    def test_search_finds_noncentral_goal_behind_robot_then_approaches(self):
        config = default_config()
        goal = make_element("goal_neutral", 600, 500, element_id="behind_robot")
        goal["tag_id"] = 4
        config["layout"] = {"preset": "custom", "elements": [goal]}
        # At the measured 140 mm lens height, use an aligned approach so
        # the chosen face stays visible after the search turn.
        pose = {"x_mm": 600, "y_mm": -300, "heading_deg": 180}
        world = SimpleNamespace(**pose, time_s=0.0, elements=[goal])
        self.assertEqual(VisionModel(world, config["vision"])._observe(), [])

        result = run_headless(config, PROGRAM, pose, duration=40)

        self.assertIsNone(result["result"]["error"])
        snapshot = result["snapshot"]
        robot = snapshot["world"]
        self.assertEqual(snapshot["vision_tracking"]["id"], 4)
        self.assertIsNone(snapshot["target"], "Searching must not create a centre-coordinate target")
        self.assertIn("Locked first visible goal: tag 4", snapshot["logs"])
        self.assertGreater(snapshot["vision_search"]["swept_deg"], 100)
        self.assertLess(math.hypot(600 - robot["x_mm"], 500 - robot["y_mm"]), 650)
        self.assertEqual(robot["collision_count"], 0)
        self.assertTrue(snapshot["braking_settled"])
        self.assertIn(snapshot["status"], ("Arrived: stopped before goal", "Stopped: target lost"))

    def test_empty_field_finishes_one_scan_across_heading_wrap(self):
        pose = {"x_mm": 0, "y_mm": 0, "heading_deg": 350}
        result = run_headless(default_config(), PROGRAM, pose, duration=35)

        self.assertIsNone(result["result"]["error"])
        snapshot = result["snapshot"]
        robot = snapshot["world"]
        self.assertEqual(snapshot["status"], "Stopped: no goal found after full scan")
        self.assertIsNone(snapshot["vision_tracking"]["id"])
        self.assertIsNone(snapshot["target"])
        self.assertGreaterEqual(snapshot["vision_search"]["swept_deg"], 355)
        self.assertLess(snapshot["vision_search"]["swept_deg"], 370)
        # A bounded heading-counter alone could pass while the chassis didn't
        # turn: the wheel encoders also have to show actual in-place travel.
        self.assertGreater(abs(robot["left_position_deg"]), 1000)
        self.assertGreater(abs(robot["right_position_deg"]), 1000)
        self.assertAlmostEqual(robot["x_mm"], 0, delta=5)
        self.assertAlmostEqual(robot["y_mm"], 0, delta=5)
        self.assertEqual(robot["collision_count"], 0)
        self.assertTrue(snapshot["braking_settled"])

    def test_robot_that_fits_beside_wall_declines_unsafe_rotation(self):
        # The initial width fits inside the wall; its corners would hit the
        # wall partway through a full turn because the diagonal is larger.
        pose = {"x_mm": 1530, "y_mm": 0, "heading_deg": 0}
        result = run_headless(default_config(), PROGRAM, pose, duration=15)

        self.assertIsNone(result["result"]["error"])
        snapshot = result["snapshot"]
        self.assertEqual(snapshot["status"], "Stopped: too close to wall to search")
        self.assertIsNone(snapshot["vision_tracking"]["id"])
        self.assertEqual(snapshot["world"]["collision_count"], 0)
        for key, expected in pose.items():
            self.assertAlmostEqual(snapshot["world"][key], expected, places=5)

    def test_temporary_gps_loss_brakes_then_resumes_same_scan(self):
        session = SimulatorSession(default_config(), PROGRAM,
                                   {"x_mm": 0, "y_mm": 0, "heading_deg": 350},
                                   realtime=True, speed=2, gps_mode="ideal", duration=15)
        injected = False
        paused = []
        resumed = None
        injection_sweep = None
        session.start()
        try:
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline and resumed is None:
                for message in session.poll():
                    if message["type"] == "exit":
                        self.fail("Program exited before GPS recovery: " + str(message))
                    if message["type"] != "snapshot":
                        continue
                    status = message["status"]
                    sweep = message["vision_search"]["swept_deg"]
                    if not injected and status == "Searching for first goal" and sweep > 15:
                        injection_sweep = sweep
                        session.inject_gps_loss(1)
                        injected = True
                    elif injected and status == "GPS lost: search paused":
                        paused.append(message)
                    elif paused and status == "Searching for first goal" and sweep > injection_sweep + 10:
                        resumed = message
                time.sleep(.01)
            self.assertTrue(injected, "Initial scan never started")
            self.assertTrue(paused, "GPS loss did not brake and pause the search")
            self.assertIsNotNone(resumed, "Search did not resume after GPS recovered")
            self.assertLess(abs(paused[-1]["world"]["omega_deg_s"]), .1)
            self.assertGreaterEqual(resumed["vision_search"]["swept_deg"], injection_sweep)
            self.assertIsNone(resumed["vision_tracking"]["id"])
            self.assertEqual(resumed["world"]["collision_count"], 0)
        finally:
            session.stop()


if __name__ == "__main__":
    unittest.main()
