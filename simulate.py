"""Launch the local simulator. The VEX upload still uses src/main.py only."""
import argparse
import json
import math
from pathlib import Path
import time

from simulator.config import load_config
from simulator.runtime import SimulatorSession


ROOT = Path(__file__).resolve().parent


def run_headless(config, program, pose, duration=60, gps_mode="ideal"):
    session = SimulatorSession(config, program, pose, realtime=False,
                               gps_mode=gps_mode, duration=duration)
    latest, end = None, None
    session.start()
    wall_deadline = time.monotonic() + max(30, duration * 2)
    try:
        while True:
            for message in session.poll():
                if message["type"] == "snapshot":
                    latest = message
                elif message["type"] == "exit":
                    end = message
            if end is not None:
                break
            if not session.alive:
                for message in session.poll():
                    if message["type"] == "snapshot":
                        latest = message
                    elif message["type"] == "exit":
                        end = message
                break
            if time.monotonic() >= wall_deadline:
                raise RuntimeError("Simulator worker exceeded its wall-clock deadline")
            time.sleep(0.005)
    finally:
        session.stop()
    if end is None:
        end = {"reason": "error", "error": "Simulator process exited without a result"}
    return {"result": end, "snapshot": latest, "gps_mode": gps_mode,
            "seed": config["simulation"]["seed"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--program", type=Path, default=ROOT / "src" / "main.py")
    parser.add_argument("--config", type=Path, help="JSON model settings or partial overrides")
    parser.add_argument("--headless", action="store_true", help="Run without a window and print a JSON result")
    parser.add_argument("--duration", type=float, default=60, help="Maximum headless simulation seconds")
    parser.add_argument("--start-x", type=float, default=-1000, help="Starting X in mm")
    parser.add_argument("--start-y", type=float, default=-1000, help="Starting Y in mm")
    parser.add_argument("--heading", type=float, default=0, help="Clockwise heading; 0 faces +Y")
    parser.add_argument("--gps-mode", choices=("ideal", "realistic"), default="realistic")
    parser.add_argument("--speed", type=float, default=1, help="GUI playback speed")
    parser.add_argument("--output", type=Path, help="Save headless result as JSON")
    parser.add_argument("--smoke-test", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.duration, args.speed)):
        parser.error("Duration and speed must be finite and positive")
    try:
        config = load_config(args.config)
        if not args.program.is_file():
            raise ValueError("Program file not found: " + str(args.program))
        pose = {"x_mm": args.start_x, "y_mm": args.start_y, "heading_deg": args.heading}
        from simulator.physics import World
        World(config, pose)  # Validate the initial footprint before starting a process.
        if args.headless:
            result = run_headless(config, args.program, pose, args.duration, args.gps_mode)
            text = json.dumps(result, indent=2)
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(text + "\n", encoding="utf-8")
            print(text)
            return 1 if result["result"].get("error") else 0
        import tkinter as tk
        from simulator.ui import SimulatorApp, ModelSettings, enable_dpi_awareness
        enable_dpi_awareness()
        root = tk.Tk()
        app = SimulatorApp(root, config, args.program)
        app.start_x.set(str(args.start_x))
        app.start_y.set(str(args.start_y))
        app.start_heading.set(str(args.heading))
        app.gps_mode.set(args.gps_mode)
        app.speed.set(str(args.speed))
        app.apply_start()
        if args.smoke_test:
            failures = []
            root.report_callback_exception = lambda *exc: failures.append(str(exc[1]))
            def exercise():
                app.redraw()
                dialog = ModelSettings(app)
                dialog._read()
                dialog.window.destroy()
                app.start()
                root.after(800, app.toggle_pause)
                root.after(1100, app.step)
                root.after(1400, app.toggle_pause)
                root.after(1600, app.inject_dropout)
                def finish():
                    if app.snapshot is None:
                        failures.append("No worker snapshot reached the GUI")
                    elif app.snapshot.get("error"):
                        failures.append(app.snapshot["error"])
                    app.close()
                root.after(2200, finish)
            root.after(100, exercise)
            root.mainloop()
            if failures:
                raise RuntimeError("GUI callback failure: " + "; ".join(failures))
            print("GUI startup, settings, Run, Pause, Step and Stop smoke test passed")
        else:
            root.mainloop()
        return 0
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(1, "Simulator: %s\n" % error)


if __name__ == "__main__":
    raise SystemExit(main())
