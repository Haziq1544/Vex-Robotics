"""Exercise the actual navigation functions with a clock and simulated VEX devices."""
import ast
import math
from pathlib import Path
import unittest


class Sim:
    def __init__(self, pose=None, quality=None):
        self.now = 0
        self.events = []
        self.pose = pose or (lambda t: (0, -1000, 90))
        self.quality = quality or (lambda t: 100)
        self.connected = True
        self.calibrating = False

        class Motor:
            def __init__(motor, port, *args):
                motor.port = port

            def stop(motor, mode):
                self.events.append((self.now, "stop", motor.port))

            def spin(motor, direction, speed, units):
                self.events.append((self.now, "spin", motor.port, direction, speed))

        class Gps:
            def __init__(gps, *args):
                pass

            def installed(gps):
                return self.connected

            def is_calibrating(gps):
                return self.calibrating

            def quality(gps):
                return self.quality(self.now)

            def x_position(gps, units):
                return self.pose(self.now)[0]

            def y_position(gps, units):
                return self.pose(self.now)[1]

            def heading(gps):
                return self.pose(self.now)[2]

        from types import SimpleNamespace
        env = {
            "Brain": lambda: SimpleNamespace(
                timer=SimpleNamespace(time=lambda units: self.now)),
            "Motor": Motor, "Gps": Gps,
            "GearSetting": SimpleNamespace(RATIO_18_1=18),
            "Ports": SimpleNamespace(PORT9=9, PORT10=10, PORT18=18),
            "MM": "mm", "MSEC": "ms", "BRAKE": "brake",
            "FORWARD": "forward", "REVERSE": "reverse", "PERCENT": "%",
            "wait": self.wait, "math": math, "print": lambda *args: None,
        }
        # Load declarations and real functions without executing the robot startup.
        source = Path(__file__).resolve().parents[1] / "src" / "main.py"
        tree = ast.parse(source.read_text())
        tree.body = [node for node in tree.body
                     if isinstance(node, (ast.Assign, ast.FunctionDef))
                     and not (isinstance(node, ast.Assign)
                              and any(isinstance(t, ast.Name)
                                      and t.id == "gps_screen_thread"
                                      for t in node.targets))]
        exec(compile(tree, str(source), "exec"), env)
        env["gps_status"] = "Ready"
        self.env = env

    def wait(self, duration, units):
        self.now += duration

    def run(self):
        self.env["autonomous"]()
        return self.env["run_status"]

    def spins(self, start=0, end=float("inf")):
        return [e for e in self.events
                if e[1] == "spin" and start <= e[0] < end]


class GpsRecoveryTests(unittest.TestCase):
    def test_lock_requires_uninterrupted_good_readings(self):
        sim = Sim(quality=lambda t: 100 if 100 <= t < 300 or t >= 320 else 90)
        self.assertIsNotNone(sim.env["wait_for_gps_lock"](2000))
        self.assertEqual(sim.now, 820)
        self.assertFalse(sim.spins())

    def test_transient_loss_brakes_then_uses_recovered_pose(self):
        sim = Sim(
            quality=lambda t: 90 if 600 <= t < 800 else 100,
            pose=lambda t: (0, -1000, 90) if t < 800 else
                           ((0, -300, 0) if t < 1500 else (0, 0, 0)))
        self.assertEqual(sim.run(), "Arrived at centre")
        self.assertTrue(sim.spins(500, 600))
        self.assertIn((600, "stop", 9), sim.events)
        self.assertIn((600, "stop", 10), sim.events)
        self.assertFalse(sim.spins(600, 1300))
        resumed = sim.spins(1300, 1320)
        self.assertEqual(len(resumed), 2)
        self.assertTrue(all(e[3] == "forward" for e in resumed))

    def test_permanent_loss_times_out_while_braked(self):
        sim = Sim(quality=lambda t: 100 if t < 600 else 90)
        self.assertEqual(sim.run(), "Stopped: GPS recovery timed out")
        self.assertEqual(sim.now, 2600)
        self.assertFalse(sim.spins(600))
        self.assertEqual([e[1] for e in sim.events[-2:]], ["stop", "stop"])

    def test_recovery_respects_overall_deadline(self):
        sim = Sim(quality=lambda t: 100 if t < 1400 else 90)
        sim.env["RUN_TIMEOUT_MS"] = 1000
        self.assertEqual(sim.run(), "Stopped: run timed out")
        self.assertEqual(sim.now, 1500)  # Initial 500 ms lock + 1000 ms run.
        self.assertFalse(sim.spins(1400))

    def test_loss_resets_arrival_confirmation(self):
        sim = Sim(pose=lambda t: (0, 0, 0),
                  quality=lambda t: 90 if 600 <= t < 800 else 100)
        self.assertEqual(sim.run(), "Arrived at centre")
        self.assertEqual(sim.now, 1800)  # Recovered at 1300; settle another 500.
        self.assertFalse(sim.spins())

    def test_recovery_resets_progress_clock(self):
        sim = Sim(quality=lambda t: 90 if 3000 <= t < 3100 else 100,
                  pose=lambda t: (0, -1000, 90) if t < 3700 else (0, 0, 0))
        self.assertEqual(sim.run(), "Arrived at centre")
        self.assertTrue(sim.spins(3600, 3700))
        self.assertTrue(all(e[4] <= 20 for e in sim.spins()))

    def test_no_initial_fix_never_moves(self):
        sim = Sim(quality=lambda t: 90)
        self.assertEqual(sim.run(), "Stopped: no GPS lock")
        self.assertEqual(sim.now, 5000)
        self.assertFalse(sim.spins())

    def test_invalid_pose_or_disconnected_sensor_never_resumes(self):
        for failure in ("bounds", "nan", "disconnected", "calibrating"):
            with self.subTest(failure=failure):
                sim = Sim()

                def pose(t):
                    if t >= 600:
                        if failure == "bounds":
                            return (3000, 0, 0)
                        if failure == "nan":
                            return (float("nan"), 0, 0)
                    return (0, -1000, 90)

                def quality(t):
                    if t >= 600:
                        sim.connected = failure != "disconnected"
                        sim.calibrating = failure == "calibrating"
                        return 0 if failure in ("disconnected", "calibrating") else 100
                    return 100

                sim.pose, sim.quality = pose, quality
                self.assertEqual(sim.run(), "Stopped: GPS recovery timed out")
                self.assertFalse(sim.spins(600))


if __name__ == "__main__":
    unittest.main()
