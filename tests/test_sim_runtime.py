"""GPS transforms and execution of the actual robot file through the VEX adapter."""
import hashlib
import math
from pathlib import Path
import tempfile
import time
import unittest

from simulate import run_headless
from simulator.config import default_config
from simulator.runtime import _Runtime, build_vex_module, SimulatorSession


PROGRAM = Path(__file__).resolve().parents[1] / "src" / "main.py"


class SensorAndAdapterTests(unittest.TestCase):
    def test_physical_mount_and_code_offsets_cancel_at_any_heading(self):
        config = default_config()
        config["robot"].update(gps_x_mm=35, gps_y_mm=-120, gps_heading_deg=180)
        for angle in (0, 45, 90, 180, 270, 359):
            runtime = _Runtime(config, {"x_mm": 500, "y_mm": -300, "heading_deg": angle}, "ideal")
            runtime.gps.configure(35, -120, 180)
            runtime.gps.update()
            reading = runtime.gps.read()
            self.assertAlmostEqual(reading["x_mm"], 500)
            self.assertAlmostEqual(reading["y_mm"], -300)
            self.assertAlmostEqual(reading["heading_deg"], angle)

    def test_incorrect_program_mount_is_not_silently_corrected(self):
        runtime = _Runtime(default_config(), {"x_mm": 0, "y_mm": 0, "heading_deg": 0}, "ideal")
        vex = build_vex_module(runtime)
        gps = vex.__dict__["Gps"](18, 0, -53.34, vex.__dict__["MM"], 0)
        runtime.gps.update()
        self.assertAlmostEqual(gps.heading(), 180)
        self.assertGreater(abs(gps.y_position()), 100)

    def test_latency_and_injected_loss(self):
        config = default_config()
        config["gps"].update(position_noise_mm=0, heading_noise_deg=0, latency_ms=100)
        runtime = _Runtime(config, {"x_mm": 0, "y_mm": 0, "heading_deg": 0}, "realistic")
        runtime.gps.update()
        self.assertEqual(runtime.gps.read()["quality"], 0)
        runtime.world.step(.1)
        runtime.gps.update()
        self.assertEqual(runtime.gps.read()["quality"], 100)
        runtime.gps.loss_until = runtime.world.time_s + .5
        self.assertEqual(runtime.gps.read()["quality"], 0)

    def test_motor_reversal_and_callable_accessors(self):
        runtime = _Runtime(default_config(), {"x_mm": 0, "y_mm": 0, "heading_deg": 0}, "ideal")
        vex = build_vex_module(runtime).__dict__
        left, right = vex["Motor"](9), vex["Motor"](10, vex["GearSetting"].RATIO_18_1, True)
        for motor in (left, right):
            motor.spin(vex["FORWARD"], 50, vex["PERCENT"])
        runtime.world.step(.5)
        self.assertGreater(runtime.world.y_mm, 0)
        self.assertAlmostEqual(runtime.world.x_mm, 0)
        self.assertGreater(left.velocity(), 0)
        self.assertGreater(right.position(), 0)
        screen = vex["Brain"]().screen
        screen.set_cursor(3, 1)
        screen.print("GPS ready")
        self.assertEqual(screen.row(), 3)
        self.assertEqual(runtime.lines[2], "GPS ready")


class ProgramExecutionTests(unittest.TestCase):
    def test_real_program_reaches_centre_from_different_starts(self):
        before = hashlib.sha256(PROGRAM.read_bytes()).digest()
        for x, y, angle, mode in ((-1000, -1000, 0, "ideal"),
                                  (1100, 700, 270, "ideal"),
                                  (-800, 1000, 180, "realistic")):
            with self.subTest(x=x, y=y, angle=angle, mode=mode):
                result = run_headless(default_config(), PROGRAM,
                                      {"x_mm": x, "y_mm": y, "heading_deg": angle}, gps_mode=mode)
                self.assertIsNone(result["result"]["error"])
                snapshot = result["snapshot"]
                self.assertEqual(snapshot["status"], "Arrived at centre")
                world = snapshot["world"]
                self.assertLess(math.hypot(world["x_mm"], world["y_mm"]), 110)
                self.assertLess(snapshot["time_s"], 46.6)
        self.assertEqual(hashlib.sha256(PROGRAM.read_bytes()).digest(), before)

    def test_same_seed_has_same_result(self):
        pose = {"x_mm": -900, "y_mm": -500, "heading_deg": 90}
        first = run_headless(default_config(), PROGRAM, pose, gps_mode="realistic")
        second = run_headless(default_config(), PROGRAM, pose, gps_mode="realistic")
        self.assertEqual(first, second)

    def test_program_failure_is_reported_and_braked(self):
        with tempfile.TemporaryDirectory() as directory:
            program = Path(directory) / "broken_robot.py"
            program.write_text("from vex import *\nm=Motor(Ports.PORT9)\nm.spin(FORWARD,50,PERCENT)\n"
                               "wait(100,MSEC)\nraise ValueError('intentional test error')\n")
            result = run_headless(default_config(), program, {"x_mm": 0, "y_mm": 0, "heading_deg": 0})
            self.assertEqual(result["result"]["reason"], "error")
            self.assertIn("intentional test error", result["result"]["error"])

    def test_pause_step_and_injected_recovery(self):
        session = SimulatorSession(default_config(), PROGRAM, realtime=True, speed=2, gps_mode="ideal")
        session.start()
        latest = {}

        def until(predicate, timeout=8):
            nonlocal latest
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                for message in session.poll():
                    if message["type"] == "snapshot":
                        latest = message
                        if predicate(message):
                            return message
                time.sleep(.01)
            self.fail("Timed out waiting for simulator state: " + str(latest.get("status")))

        try:
            until(lambda s: s["time_s"] > 2)
            session.pause()
            paused = until(lambda s: s["paused"])
            clock = paused["time_s"]
            unchanged = until(lambda s: s["paused"])
            self.assertEqual(unchanged["time_s"], clock)
            session.step()
            stepped = until(lambda s: s["time_s"] > clock)
            self.assertAlmostEqual(stepped["time_s"] - clock, .005)
            session.inject_gps_loss(.7)
            session.pause(False)
            lost = until(lambda s: s["status"] == "GPS lost: waiting for lock")
            resumed = until(lambda s: s["time_s"] > lost["time_s"] + .5
                            and s["status"] in ("Driving to centre", "Turning to centre"))
            self.assertEqual(resumed["gps"]["quality"], 100)
        finally:
            session.stop()
        self.assertFalse(session.alive)


if __name__ == "__main__":
    unittest.main()
