# Ontario Skills Robot & Simulator

VEX V5 robot code and a Python desktop simulator. Physical testing is still in progress.

## Start here

- **Run the simulator:** double-click `launch_simulator.bat`, or select **Flex Simulator** in VS Code and press **F5**, then **Run**.
- **Edit/upload the robot program:** `src/main.py`.
- **Understand current progress:** [Project status](docs/PROJECT_STATUS.md).
- **Use the simulator:** [Simulator guide](simulator/README.md).

The desktop **Ontario Skills Workspace** shortcut opens the active project in
`Documents/vex-vscode-projects/Ontario_Skills_bot`. The separately copied desktop
folder is archived; do not use that copy for ongoing work.

## Folder map

| Location | Purpose |
| --- | --- |
| `src/main.py` | Current program uploaded to the VEX Brain and executed by the simulator. |
| `simulator/` | Desktop interface, physics, collisions, sensor models, and settings. |
| `examples/` | Preserved earlier robot experiments; not uploaded or launched by default. |
| `tests/` | Automated checks; some still target earlier experiments. |
| `docs/` | Current project status and development notes. |
| `.vscode/` | Existing VS Code launch settings and VEX project configuration. |
| `simulate.py` | Command-line entry point for desktop and headless simulation. |
| `launch_simulator.bat` | Windows launcher. |

## Current robot behaviour

The program confirms a visible AprilTag and turns to centre it. **Tag 2 only**
then receives short forward steps at 6% power, with braking and alignment checks
between steps. Other accepted IDs (0–4) centre and hold.

The Tag 2 stop threshold is **12 pixels wide OR 13 pixels high**, based on a
13 × 14 pixel hardware reading at approximately 240 mm front clearance.
This is an image-size threshold, not a guaranteed stopping distance. An image-edge
guard, target loss after forward motion, stale camera data, or time limits also
stop the approach. No visible tag means waiting stationary. **GPS searching and
distance estimation are not used by the current program.**

Centring was confirmed in physical testing. The latest approach still needs
physical validation. Robot dimensions and physics remain partly estimated.

| Device | Smart Port | Mount information |
| --- | ---: | --- |
| Left drive motor | 10 | Green cartridge |
| Right drive motor | 11 | Green cartridge; reversed in code |
| AI Vision | 17 | X 0, Y +139.7 mm; lens height 135.2 mm; forward and level |
| GPS | 18 | X +101.6, Y −162.56 mm; backward-facing, 180° offset; inactive in current test |

Offsets are measured from the drivetrain turning centre to each sensor lens.
Use `Ports.PORT10` and similar VEX constants in robot code; configuration JSON
uses printed Smart Port numbers. These representations differ internally.

## Simulator capabilities

- Scaled top-down field, editable starting pose, and robot trail.
- Run, pause, step, stop, reset, telemetry, and simulated Brain output.
- Acceleration, braking, traction, wall collisions, and fixed obstacles.
- Override match layout with **9 goals and 4 loaders**, empty field, and custom layouts.
- Geometric AprilTag observations and configurable GPS noise, delay, and loss.
- JSON model/layout save and load, plus headless execution without a window.

Stacker/cup code is retained but disabled. Scoring, moving mechanisms, and
obstacle route planning are not implemented. The preset is a match layout,
not a complete Robot Skills setup. Simulation results do not establish hardware accuracy.

## Preserved programs and tests

See [Examples](examples/README.md) for the GPS, geometric approach, and
centring-only baselines. Keep their paths unchanged: tests refer to them.

See [Test status](tests/README.md) before interpreting test results. Earlier
passing results predate the current camera-only approach. Tests were not run
during workspace cleanup, and no robot or simulator logic was changed.

## Web development

Web learning happens in the separate **Web Development Learning** workspace.
This repository currently contains no Flask server or browser interface.
Simulator packaging and web integration are future work after the fundamentals.
