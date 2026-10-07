"""Run unmodified VEX Python in an isolated process with a cooperative virtual clock."""
from collections import deque
from enum import Enum, IntEnum
import math
import multiprocessing as mp
from pathlib import Path
import random
import sys
import threading
import time
import traceback
import types

from .physics import World


class SimulationHalt(BaseException):
    """Internal cancellation, deliberately outside robot code's Exception handlers."""


class UnsupportedVexAPI(RuntimeError):
    pass


class GPSModel:
    """Sampled optical-position model with independent physical and configured mounts."""
    def __init__(self, world, config, mode="realistic"):
        self.world, self.config, self.mode = world, config, mode
        self.rng = random.Random(config["simulation"]["seed"])
        robot = config["robot"]
        self.offset = (robot["gps_x_mm"], robot["gps_y_mm"], robot["gps_heading_deg"])
        self.calibrating_until = 0.0
        self.loss_until = 0.0
        self.next_sample = 0.0
        self.pending = deque()
        self.last = None

    def configure(self, x_mm, y_mm, heading_deg):
        self.offset = (x_mm, y_mm, heading_deg)

    def sensor_truth(self):
        r = self.config["robot"]
        h = math.radians(self.world.heading_deg)
        return (self.world.x_mm + r["gps_x_mm"] * math.cos(h) + r["gps_y_mm"] * math.sin(h),
                self.world.y_mm - r["gps_x_mm"] * math.sin(h) + r["gps_y_mm"] * math.cos(h),
                (self.world.heading_deg + r["gps_heading_deg"]) % 360)

    def update(self):
        now = self.world.time_s
        cfg = self.config["gps"]
        if now + 1e-9 >= self.next_sample:
            x, y, heading = self.sensor_truth()
            quality = 100
            if self.mode == "realistic":
                state = self.world.snapshot()
                # These are adjustable failure scenarios, not a camera/vision simulation.
                if abs(state["omega_deg_s"]) > cfg["turn_dropout_deg_s"]:
                    quality = 80
                h = math.radians(heading)
                distances = []
                for position, direction, half in (
                    (x, math.sin(h), self.config["field"]["width_mm"] / 2),
                    (y, math.cos(h), self.config["field"]["height_mm"] / 2),
                ):
                    if abs(direction) > 1e-9:
                        distances.append((math.copysign(half, direction) - position) / direction)
                if distances and min(distances) < cfg["wall_min_distance_mm"]:
                    quality = min(quality, 80)
                x += self.rng.gauss(0, cfg["position_noise_mm"])
                y += self.rng.gauss(0, cfg["position_noise_mm"])
                heading = (heading + self.rng.gauss(0, cfg["heading_noise_deg"])) % 360
            if now < self.loss_until or now < self.calibrating_until:
                quality = 0
            latency = cfg["latency_ms"] / 1000 if self.mode == "realistic" else 0
            self.pending.append((now + latency, (x, y, heading, quality)))
            self.next_sample = now + 1 / cfg["sample_hz"]
        while self.pending and self.pending[0][0] <= now + 1e-9:
            self.last = self.pending.popleft()[1]

    def read(self):
        if self.last is None:
            x, y, heading = self.sensor_truth()
            quality = 0
        else:
            x, y, heading, quality = self.last
        if self.world.time_s < self.calibrating_until or self.world.time_s < self.loss_until:
            quality = 0
        ox, oy, angle = self.offset
        robot_heading = (heading - angle) % 360
        h = math.radians(robot_heading)
        return {"x_mm": x - ox * math.cos(h) - oy * math.sin(h),
                "y_mm": y + ox * math.sin(h) - oy * math.cos(h),
                "heading_deg": robot_heading, "quality": quality,
                "sensor_x_mm": x, "sensor_y_mm": y, "sensor_heading_deg": heading}


class _Scheduler:
    """Only one robot thread runs at once; wait() yields to the virtual clock."""
    def __init__(self, runtime):
        self.runtime = runtime
        self.condition = threading.Condition()
        self.local = threading.local()
        self.tasks = {}
        self.active = None
        self.active_since = 0.0
        self.closed = False
        self.error: str | None = None
        self.main_done = False

    def spawn(self, function, args=(), main=False):
        with self.condition:
            ident = len(self.tasks)
            task = {"wake": self.runtime.world.time_s, "state": "waiting", "cancelled": False}
            self.tasks[ident] = task

        def entry():
            self.local.ident = ident
            try:
                with self.condition:
                    self._await_turn(task)
                function(*args)
            except SimulationHalt:
                pass
            except BaseException:
                self.error = traceback.format_exc()
            finally:
                with self.condition:
                    task["state"] = "done"
                    if self.active == ident:
                        self.active = None
                    if main:
                        self.main_done = True
                    self.condition.notify_all()

        thread = threading.Thread(target=entry, daemon=True, name="VEX-program-%s" % ident)
        thread.start()
        return ident

    def _await_turn(self, task):
        while task["state"] != "running" and not self.closed and not task["cancelled"]:
            self.condition.wait()
        if self.closed or task["cancelled"]:
            raise SimulationHalt()

    def wait(self, seconds):
        ident = self.local.ident
        with self.condition:
            task = self.tasks[ident]
            task["wake"] = self.runtime.world.time_s + max(seconds, 0.001)
            task["state"] = "waiting"
            self.active = None
            self.condition.notify_all()
            self._await_turn(task)

    def service(self):
        """True when all robot threads are asleep and physics may advance."""
        with self.condition:
            if self.active is not None:
                if time.monotonic() - self.active_since > 2:
                    raise RuntimeError("Robot code ran for 2 seconds without yielding. Add wait() to busy loops.")
                self.condition.wait(timeout=0.001)
                return False
            ready = [(task["wake"], ident) for ident, task in self.tasks.items()
                     if task["state"] == "waiting" and not task["cancelled"]
                     and task["wake"] <= self.runtime.world.time_s + 1e-9]
            if ready:
                _, ident = min(ready)
                self.tasks[ident]["state"] = "running"
                self.active, self.active_since = ident, time.monotonic()
                self.condition.notify_all()
                return False
            return True

    def cancel(self, ident):
        with self.condition:
            self.tasks[ident]["cancelled"] = True
            self.condition.notify_all()

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()


class _Runtime:
    def __init__(self, config, start_pose, gps_mode):
        self.config = config
        self.world = World(config, start_pose)
        self.gps = GPSModel(self.world, config, gps_mode)
        self.scheduler = _Scheduler(self)
        self.lines = [""] * 12
        self.logs = []
        self.log_revision = 0
        self.namespace = {}
        self.error: str | None = None

    def log(self, *args, sep=" ", end="\n", **kwargs):
        self.logs.extend((sep.join(str(a) for a in args) + end).rstrip("\n").splitlines())
        self.logs = self.logs[-200:]
        self.log_revision += 1

    def snapshot(self, paused=False):
        target = None
        if "TARGET_X_MM" in self.namespace and "TARGET_Y_MM" in self.namespace:
            target = {"x_mm": self.namespace["TARGET_X_MM"],
                      "y_mm": self.namespace["TARGET_Y_MM"],
                      "radius_mm": self.namespace.get("ARRIVAL_RADIUS_MM", 0)}
        return {"type": "snapshot", "time_s": self.world.time_s,
                "world": self.world.snapshot(), "gps": self.gps.read(),
                "brain_lines": list(self.lines), "logs": list(self.logs),
                "log_revision": self.log_revision, "target": target,
                "status": str(self.namespace.get("run_status", "Running program")),
                "error": self.error or self.scheduler.error, "paused": paused}

    def brake(self):
        for key in ("left_port", "right_port"):
            self.world.set_motor(self.config["robot"][key], {"percent": 0, "brake": "brake"})


def build_vex_module(runtime):
    """Create the documented subset used by this robot without shadowing the VEX SDK."""
    module = types.ModuleType("vex")

    class Units(str, Enum):
        MSEC = "msec"
        SEC = "sec"
        MM = "mm"
        INCHES = "inches"
        DEGREES = "degrees"
        TURNS = "turns"
        PERCENT = "percent"
        RPM = "rpm"

    class Direction(str, Enum):
        FORWARD = "forward"
        REVERSE = "reverse"

    class BrakeType(str, Enum):
        COAST = "coast"
        BRAKE = "brake"
        HOLD = "hold"

    class GearSetting(IntEnum):
        RATIO_36_1 = 100
        RATIO_18_1 = 200
        RATIO_6_1 = 600

    Ports = IntEnum("Ports", {"PORT%d" % n: n for n in range(1, 22)})

    def wait(duration, units=Units.MSEC):
        if units not in (Units.MSEC, Units.SEC):
            raise ValueError("wait units must be MSEC or SEC")
        runtime.scheduler.wait(float(duration) / 1000 if units == Units.MSEC else float(duration))

    class Timer:
        def __init__(self):
            self.origin = 0.0

        def time(self, units=Units.MSEC):
            seconds = runtime.world.time_s - self.origin
            return seconds if units == Units.SEC else seconds * 1000

        def reset(self):
            self.origin = runtime.world.time_s

        @staticmethod
        def system():
            return runtime.world.time_s * 1000

    class Screen:
        def __init__(self):
            self._row, self.col = 0, 0

        def clear_screen(self, *args):
            runtime.lines[:] = [""] * 12
            self._row, self.col = 0, 0

        def set_cursor(self, row, column):
            self._row, self.col = max(0, min(11, int(row) - 1)), max(0, int(column) - 1)

        def clear_line(self, number=None, *args):
            runtime.lines[self._row if number is None else max(0, min(11, int(number) - 1))] = ""

        def print(self, *args, sep=" ", **kwargs):
            text = sep.join(str(a) for a in args)
            line = runtime.lines[self._row].ljust(self.col)
            runtime.lines[self._row] = line[:self.col] + text + line[self.col + len(text):]
            self.col += len(text)

        def new_line(self):
            self._row, self.col = min(11, self._row + 1), 0

        def row(self):
            return self._row + 1

    class Brain:
        def __init__(self):
            self.timer, self.screen = Timer(), Screen()

    class Motor:
        def __init__(self, port, gears=GearSetting.RATIO_18_1, reverse=False):
            self.port = int(port)
            if self.port not in (runtime.config["robot"]["left_port"], runtime.config["robot"]["right_port"]):
                raise UnsupportedVexAPI("Motor on port %s has no physical model. Add it to the simulator first." % port)
            self.gears, self.reverse = gears, bool(reverse)
            self._velocity, self.velocity_units = 50, Units.PERCENT
            self.stopping = BrakeType.COAST
            self.zero = 0.0

        def installed(self):
            return True

        def set_velocity(self, velocity, units=Units.PERCENT):
            self._velocity, self.velocity_units = velocity, units

        def set_stopping(self, mode=BrakeType.COAST):
            self.stopping = mode

        def spin(self, direction, velocity=None, units=None):
            value = float(self._velocity if velocity is None else velocity)
            units = self.velocity_units if units is None else units
            if units == Units.RPM:
                value = value / runtime.config["robot"]["motor_free_rpm"] * 100
            elif units != Units.PERCENT:
                raise UnsupportedVexAPI("Motor velocity units supported: PERCENT and RPM")
            value *= -1 if direction == Direction.REVERSE else 1
            value *= -1 if self.reverse else 1
            runtime.world.set_motor(self.port, {"percent": max(-100, min(100, value)), "brake": "coast"})

        def stop(self, mode=None):
            runtime.world.set_motor(self.port, {"percent": 0, "brake": str((mode or self.stopping).value
                if isinstance(mode or self.stopping, Enum) else (mode or self.stopping))})

        def _shaft(self, suffix):
            side = "left" if self.port == runtime.config["robot"]["left_port"] else "right"
            value = runtime.world.snapshot()[side + suffix]
            return value * (-1 if self.reverse else 1)

        def position(self, units=Units.DEGREES):
            value = self._shaft("_position_deg") - self.zero
            return value / 360 if units == Units.TURNS else value

        def set_position(self, value, units=Units.DEGREES):
            self.zero = self._shaft("_position_deg") - value * (360 if units == Units.TURNS else 1)

        def reset_position(self):
            self.set_position(0)

        def velocity(self, units=Units.RPM):
            value = self._shaft("_rpm")
            return value / runtime.config["robot"]["motor_free_rpm"] * 100 if units == Units.PERCENT else value

    class Gps:
        def __init__(self, port, origin_x=0, origin_y=0, units=Units.MM, heading_offset=0):
            if int(port) != runtime.config["robot"]["gps_port"]:
                raise UnsupportedVexAPI("No GPS model on port %s (physical GPS port is %s)." %
                                        (port, runtime.config["robot"]["gps_port"]))
            factor = 25.4 if units == Units.INCHES else 1
            runtime.gps.configure(origin_x * factor, origin_y * factor, heading_offset)

        def installed(self):
            return True

        def calibrate(self):
            runtime.gps.calibrating_until = runtime.world.time_s + 1

        def is_calibrating(self):
            return runtime.world.time_s < runtime.gps.calibrating_until

        def quality(self):
            return runtime.gps.read()["quality"]

        def heading(self, units=Units.DEGREES):
            return runtime.gps.read()["heading_deg"]

        def x_position(self, units=Units.MM):
            return runtime.gps.read()["x_mm"] / (25.4 if units == Units.INCHES else 1)

        def y_position(self, units=Units.MM):
            return runtime.gps.read()["y_mm"] / (25.4 if units == Units.INCHES else 1)

        def set_origin(self, x, y, units=Units.MM):
            factor = 25.4 if units == Units.INCHES else 1
            runtime.gps.configure(x * factor, y * factor, runtime.gps.offset[2])

    class Thread:
        def __init__(self, callback, args=()):
            self.ident = runtime.scheduler.spawn(callback, args)

        def stop(self):
            runtime.scheduler.cancel(self.ident)

    class Competition:
        """This simulator runs the autonomous phase immediately after registration."""
        def __init__(self, driver, autonomous):
            self.thread = Thread(autonomous)

        @staticmethod
        def is_enabled():
            return True

        @staticmethod
        def is_autonomous():
            return True

        @staticmethod
        def is_driver_control():
            return False

    exports = {"Brain": Brain, "Motor": Motor, "Gps": Gps, "Thread": Thread,
               "Timer": Timer, "Competition": Competition, "Ports": Ports,
               "GearSetting": GearSetting, "BrakeType": BrakeType, "wait": wait,
               "TimeUnits": Units, "DistanceUnits": Units, "RotationUnits": Units,
               "VelocityUnits": Units, "DirectionType": Direction}
    for kind in (Units, Direction, BrakeType):
        exports.update({value.name: value for value in kind})
    module.__dict__.update(exports)
    module.__dict__["__all__"] = list(exports)
    return module


def _worker_main(connection, config, program_path, start_pose, realtime, speed, gps_mode, duration):
    runtime = None
    try:
        runtime = _Runtime(config, start_pose, gps_mode)
        program_path = Path(program_path).resolve()
        code = compile(program_path.read_text(encoding="utf-8-sig"), str(program_path), "exec")
        sys.modules["vex"] = build_vex_module(runtime)
        sys.path.insert(0, str(program_path.parent))
        runtime.namespace = {"__name__": "__main__", "__file__": str(program_path), "print": runtime.log}
        runtime.scheduler.spawn(lambda: exec(code, runtime.namespace), main=True)
        paused, stepping = False, False
        next_wall = time.monotonic()
        next_snapshot, next_sim_snapshot = 0.0, 0.0
        dt = config["simulation"]["step_ms"] / 1000
        reason = "stopped"
        while True:
            while connection.poll():
                command = connection.recv()
                if command["type"] == "stop":
                    return
                if command["type"] == "pause":
                    paused = command["paused"]
                    next_wall = time.monotonic()
                elif command["type"] == "step":
                    paused, stepping = True, True
                elif command["type"] == "speed":
                    speed = max(0.05, min(20, float(command["speed"])))
                    next_wall = time.monotonic()
                elif command["type"] == "gps_loss":
                    runtime.gps.loss_until = runtime.world.time_s + max(0, command["seconds"])
            now_wall = time.monotonic()
            if (realtime and now_wall >= next_snapshot) or (not realtime and runtime.world.time_s >= next_sim_snapshot):
                connection.send(runtime.snapshot(paused))
                next_snapshot = now_wall + 1 / 30
                next_sim_snapshot = runtime.world.time_s + 0.1
            if runtime.scheduler.error:
                runtime.error = runtime.scheduler.error
                reason = "error"
                break
            if paused and not stepping:
                time.sleep(0.005)
                continue
            if not runtime.scheduler.service():
                continue
            status = str(runtime.namespace.get("run_status", ""))
            if status.startswith(("Arrived", "Stopped:")):
                reason = "program_status"
                break
            if runtime.scheduler.main_done and all(t["state"] == "done" for t in runtime.scheduler.tasks.values()):
                reason = "program_finished"
                break
            if duration is not None and runtime.world.time_s >= duration:
                reason = "duration"
                break
            runtime.world.step(dt)
            runtime.gps.update()
            if stepping:
                stepping = False
                connection.send(runtime.snapshot(True))
            if realtime:
                next_wall += dt / speed
                delay = next_wall - time.monotonic()
                if delay > 0:
                    time.sleep(min(delay, 0.02))
                elif delay < -0.25:
                    next_wall = time.monotonic()
        runtime.brake()
        connection.send(runtime.snapshot(True))
        connection.send({"type": "exit", "reason": reason, "message": str(runtime.namespace.get("run_status", reason)),
                         "error": runtime.error})
    except BaseException:
        error = traceback.format_exc()
        if runtime:
            runtime.error = error
            runtime.brake()
            connection.send(runtime.snapshot(True))
        try:
            connection.send({"type": "exit", "reason": "error", "message": "Simulation error", "error": error})
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        if runtime:
            runtime.scheduler.close()
        connection.close()


class SimulatorSession:
    """Parent-side lifecycle. Robot code never executes in the Tk UI process."""
    def __init__(self, config, program_path, start_pose=None, realtime=True, speed=1.0,
                 gps_mode="realistic", duration=None):
        self.args = (config, str(program_path), start_pose, realtime, speed, gps_mode, duration)
        self.process = None
        self.connection = None

    @property
    def alive(self):
        return self.process is not None and self.process.is_alive()

    def start(self):
        if self.alive:
            raise RuntimeError("Simulation is already running")
        context = mp.get_context("spawn")
        self.connection, child = context.Pipe()
        self.process = context.Process(target=_worker_main, args=(child,) + self.args, daemon=True)
        self.process.start()
        child.close()

    def _send(self, command):
        if self.alive and self.connection:
            try:
                self.connection.send(command)
            except (BrokenPipeError, EOFError, OSError):
                pass

    def pause(self, paused=True):
        self._send({"type": "pause", "paused": bool(paused)})

    def step(self):
        self._send({"type": "step"})

    def set_speed(self, speed):
        self._send({"type": "speed", "speed": speed})

    def inject_gps_loss(self, seconds: float = 1.0):
        self._send({"type": "gps_loss", "seconds": seconds})

    def poll(self):
        messages = []
        if self.connection:
            try:
                while self.connection.poll():
                    messages.append(self.connection.recv())
            except (EOFError, OSError):
                pass
        return messages

    def stop(self):
        self._send({"type": "stop"})
        if self.process:
            self.process.join(0.25)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(1)
        if self.connection:
            self.connection.close()
            self.connection = None
