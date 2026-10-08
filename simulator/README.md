# Robot simulator

Run and inspect `src/main.py` on your computer before testing on the VEX Brain. The simulator loads that file unchanged into a separate process and supplies simulated VEX devices. It does not download to, connect to, or move the real robot.

## Start the simulator

In VS Code, open this project folder, press **Ctrl+Shift+D**, choose **Flex Simulator** from the Run and Debug dropdown, and press **F5**. Then press **Run** in the simulator window. **Ctrl+F5** launches without the debugger. The launch configuration always opens `simulate.py`, whichever source file is currently selected. It uses the full installed desktop Python path, `C:/Users/1khan/AppData/Local/Python/pythoncore-3.14-64/python.exe`, because `LOCALAPPDATA` did not resolve in this VS Code session. If Python is moved or upgraded, update that path in `.vscode/launch.json`. Robot subprocess debugging is disabled for this launch because the simulation's cooperative clock is intended to run continuously. See the [VS Code launch documentation](https://code.visualstudio.com/docs/python/debugging).

Install **Python 3.10 or newer**, including **Tcl/Tk and IDLE** if you use the Windows installer. There are no third-party packages to install.

From the project folder, double-click `launch_simulator.bat`, or run:

```powershell
python simulate.py
```

If Windows uses the Python launcher instead, use `py simulate.py`. If the window cannot open because Tkinter is unavailable, install your Python distribution's Tk support; the headless mode below does not need a window.

Choose a starting position and heading, then press **Run**. Positions are in millimetres, with `(0, 0)` at the field centre. Heading `0°` faces `+Y`; headings increase clockwise, so `90°` faces `+X`.

- **Pause** freezes simulated time; stepping advances one physics update.
- **Stop**, edit and save `src/main.py`, then **Run** to load your changes.
- Read the simulated Brain screen and console for the program's status and errors.

The current robot program begins moving automatically after its simulated GPS calibration and lock. Its own stopping conditions still apply. The simulator finishes when the program reports arrival or a stopped result, and keeps the final display visible.

## GPS and robot settings

Use **ideal** GPS to check route logic with clean measurements. Use **realistic** GPS to explore the effects of measurement delay, noise and temporary loss of a position fix. These are illustrative sensor models, not a prediction of exactly when your physical sensor will lose its field-code view.

The simulator's **physical GPS mounting position** is independent of the offsets configured in `src/main.py`. Enter the measured physical mounting location in the simulator and keep the real robot's configuration in the code. Deliberate differences let you see how an incorrect mounting offset changes the reported robot position.

**Model settings** includes Chassis, GPS, Field & engine, and Physics tabs. Save JSON preserves your measured configuration; load it in the dialog or start with `python simulate.py --config flex-model.json`. Use the Controls sidebar scrollbar to reach all telemetry on smaller screens. The dashed robot outline is its collision footprint; the GPS cone is a schematic illustration, not a calibrated camera field of view.

Wheel size, track width, mass, traction and other robot settings begin as estimates. Measure your assembled robot and adjust these values before comparing distances or times with a physical run. Green motor cartridges and 1:1 external gearing alone do not determine the wheel size or the robot's turning behaviour.

Defaults are stored in `simulator/default_config.json`:

| Setting | Simulator default | Confidence |
| --- | --- | --- |
| Field interior | 3568.7 × 3568.7 mm (140.5 × 140.5 inches) | Official metal perimeter drawing |
| Robot collision body | 430 mm long × 400 mm wide | Build-diagram estimate |
| Drive wheel diameter | 101.6 mm (4 inches) | Assumption to measure |
| Track width | 330 mm | Estimate; distance between left and right wheel contact centres |
| Robot mass | 6.5 kg | Rough estimate, not a weighed robot |
| External drive ratio | 1:1 | Your stated gearing |
| Physics step | 5 ms | Simulation setting |

The field model includes perimeter walls, fixed goals and loaders. The robot uses a fixed collision rectangle rather than individually moving lift and claw parts.

## Field layouts and collision testing

The desktop launcher opens **Override (goals + loaders)**. In the **Field** tab, use the
layout dropdown to switch between that preset, an empty field, and a custom
layout. The preset matches the uploaded **head-to-head** overhead image, with
only goals and loaders enabled. It is not the separate Robot Skills starting setup.

The layout contains **9 anchored goals and 4 anchored loaders**. They block the
robot and cannot be pushed or rotated by it during a run. Floor tape remains as
a visual guide. Pins, cups/stackers, and wall-top toggles are not displayed.
Cup/stacker definitions and placement code are retained for later, but disabled
in the preset, palette, and runtime. Loading an older layout preserves its saved
stacker data without displaying or simulating those pieces.

While stopped, drag an element from the palette onto the field, or select a
palette item and click its destination. Drag existing elements to move them;
select one to rotate or delete it. Invalid placements (outside the walls or
overlapping another element, or placing a physical obstacle on the robot) are rejected. Editing is locked while
running or paused. Editing a preset creates a custom layout.

**Restore preset** puts the game elements back in their preset positions.
The main **Reset run** button restores the robot to the authored starting
positions of the current layout, so repeated collision tests use the same setup.
Field JSON save/load preserves custom starting layouts; model JSON also includes
the layout. Saved layouts contain authored positions independently of run results.

The unchanged `src/main.py` still targets `(0, 0)`. On Override this is occupied
by a fixed goal: expect the robot to contact it and eventually report its
existing no-progress stop instead of reaching the centre. Select **Empty field**
to reproduce the earlier centre-arrival test. No route planning has been added
to the robot program.

Official dimensions, coordinates, and simplifications are documented in
[FIELD_SOURCES.md](FIELD_SOURCES.md). These are 2D rigid footprints; they cannot
predict whether a raised mechanism can pass over an object. Scoring, vision,
obstacle avoidance, and cup/stacker behavior will be implemented later.

## Run without a window

From the project folder:

```powershell
python simulate.py --headless --duration 60 --start-x -1000 --start-y -1000 --heading 0 --gps-mode ideal
python simulate.py --headless --layout override --gps-mode ideal --duration 20
```

`--duration` is the maximum simulated time in seconds; arrival or a stopped result can finish the run earlier. Add `--output result.json` to save the JSON result, or use `--help` to see the available options. Repeating the same configuration and seed is intended to make local runs reproducible; changing the model settings changes the result.

Headless mode defaults to an empty field for compatibility with existing tests.
Use `--layout override` to select the game preset, or `--config saved-model.json`
to run a saved layout. `--layout` explicitly overrides the layout in a config.
The Override default start is `(-900, -900)` mm; the empty-field default remains
`(-1000, -1000)` mm. Override goals and loaders make some previous starting poses
invalid. The initial robot footprint must be clear of physical obstacles.

## What this can verify

The model includes finite acceleration, motor torque, wheel traction, wall and element contact, and simulated GPS measurements. It is useful for checking control flow, turning directions, GPS recovery, collision clearance, and whether a route reaches its target under the chosen assumptions.

It has not been validated against this physical robot. Wheel slip, sensor visibility, contact with game objects and other real conditions may behave differently. Reaching the target here does not establish that the robot will follow the same path on the competition field.

The VEX compatibility layer implements the APIs needed by the project, rather than the entire VEX runtime. Unsupported calls should produce an explicit error in the console. The original `src/main.py` remains the program to upload to the Brain; simulator files are for your computer only.

Currently supported: `Brain` text screen and timer, `Motor` construction/spin/stop/velocity/position, `Gps` mounting/position/heading/quality/calibration, cooperative `Thread`/`wait`, and basic autonomous `Competition` registration. Additional sensors, motor mechanisms, and drivetrain convenience classes need models before programs using those APIs can run. A program loop must call `wait()` regularly; a busy loop without it is reported instead of freezing the desktop window.

For model provenance, equations and calibration limits, read [PHYSICS_SOURCES.md](PHYSICS_SOURCES.md). Run the checks with `python -m unittest discover -s tests -v` from the project root.
