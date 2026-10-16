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

The active camera test confirms and centres a visible tag, then approaches **Tag 2 only** at 6% power in nominal 100 ms steps, braking at least 150 ms between steps. Other IDs centre and hold. It ends on a width of 12 pixels or height of 13 pixels, using a one-pixel margin before the user's 13 x 14 pixel reading at about 240 mm front clearance. That is a visual threshold, not a guaranteed distance. The geometric simulator may stop at a different gap from real hardware. Tag loss after forward motion, an image-edge guard, stale data, or a timeout also stops the run. No GPS search or distance calculation is used. The centring-only baseline is saved in `examples/vision_centering_test.py`; the old pose-based experiment remains in `examples/vision_approach_test.py`.

## GPS and robot settings

Use **ideal** GPS to check route logic with clean measurements. Use **realistic** GPS to explore the effects of measurement delay, noise and temporary loss of a position fix. These are illustrative sensor models, not a prediction of exactly when your physical sensor will lose its field-code view.

The simulator's **physical GPS mounting position** is independent of the offsets in robot code. The active camera approach test does not use GPS. The archived approach experiment uses GPS heading for a search sweep and X/Y for drift and wall clearance. Deliberate differences between physical and configured offsets let you test a biased GPS reading with that archived program.

**Model settings** includes AI Vision, Chassis, GPS, Field & engine, and Physics tabs. Save JSON preserves your measured configuration; load it in the dialog or start with `python simulate.py --config flex-model.json`. Use the Controls sidebar scrollbar to reach all telemetry on smaller screens. The dashed robot outline is its collision footprint; the GPS cone is a schematic illustration, not a calibrated camera field of view.

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

The active `src/main.py` selects the first visible goal tag and has no fixed
field-coordinate target. The previous centre test is archived in
`examples/gps_centre_test.py`; normal launches and uploads do not run it.

Official dimensions, coordinates, and simplifications are documented in
[FIELD_SOURCES.md](FIELD_SOURCES.md). These are 2D rigid footprints; they cannot
predict whether a raised mechanism can pass over an object. Scoring,
obstacle avoidance, and cup/stacker behavior will be implemented later.

## AI Vision and AprilTag approach

Launch **Flex Simulator** or **AI Vision Simulator** with F5, or use
`python simulate.py`. All normal launch paths run `src/main.py`, reloading it
on each Run. `--vision` remains an alias for the same program.

The **Vision** tab shows synthetic 320 by 240 pixel observations of visible goal
tags. The field shows the camera's viewing cone. Rotate or move the robot while
stopped to inspect the preview; use **Hide tags for 1 second** during a run to
test the controller's loss response. Loaders can block tags but are not detected
as tagged targets.

During a run, **Locked goal: ID ...** identifies the controller's chosen tag,
separately from the list of visible tags. The console records the actual program
loaded and the first acquired tag. A running program that has not created a
camera does not get synthetic preview detections. Before Run, the preview is
only shown for source declaring `PROGRAM_KIND = "ai_vision_first_goal"`.
If an editor undo or file replacement restores the old GPS code in `src/main.py`,
normal launches refuse it with an explicit message instead of driving to centre
under a vision label. Reload the intended saved source before running.

The example Override start is `(1400, -1200)` mm, heading `270°`, facing an
off-centre neutral goal (Tag 4). That default demonstrates detection and centring;
it does **not** demonstrate forward approach, which is enabled only for Tag 2.
Choose a valid starting pose facing a Tag 2 goal to inspect the current approach.
Run preserves your chosen pose. There is no fixed field-coordinate target or
route planning. With no visible tag, the current program waits stationary.

The active program uses the camera-only behaviour described above. The earlier
GPS-guided search and geometric 50 mm stopping-gap experiment is preserved in
`examples/vision_approach_test.py`; it is not executed by normal launches.
See [vision measurements and archived experiment](VISION_SOURCES.md) for details.

Current wiring is left motor 10, right motor 11, GPS 18, AI Vision 17. GPS model
offsets are **+101.6/−162.56 mm** with a rear-facing 180° heading offset. Camera
offsets are **0/+139.7 mm**, with lens height **135.2 mm**, forward and level.
Offsets are measured from the drivetrain turning centre. The active program
does not use GPS or a geometric distance estimate. Old saved model JSON and
archived programs may retain earlier settings; review them before comparison.

The same image-size threshold runs on hardware and in simulation, but the
geometric camera model has not been fitted to the real 13 × 14 pixel reading.
A simulated stop may therefore occur at a different front clearance.

On normal completion, the simulator advances braking physics for up to one
additional second. The final position and collision count include that travel;
headless results include `braking_settled`. This avoids reporting arrival while
the robot still has its pre-braking speed.

## Run without a window

From the project folder:

```powershell
python simulate.py --headless --duration 20 --start-x 1400 --start-y -1200 --heading 270
python simulate.py --headless --layout override --gps-mode ideal --duration 20
```

`--duration` is the maximum simulated time in seconds; arrival or a stopped result can finish the run earlier. Add `--output result.json` to save the JSON result, or use `--help` to see the available options. Repeating the same configuration and seed is intended to make local runs reproducible; changing the model settings changes the result.

Normal GUI and headless launches default to Override unless a config or explicit
layout is supplied. Custom `--program` headless runs keep the empty-field default.
Use `--layout override` to select the game preset, or `--config saved-model.json`
to run a saved layout. `--layout` explicitly overrides the layout in a config.
The default vision pose is described above; the empty-field default remains
`(-1000, -1000)` mm. Override goals and loaders make some starting poses
invalid. The initial robot footprint must be clear of physical obstacles.

## What this can verify

The model includes finite acceleration, motor torque, wheel traction, wall and element contact, and simulated GPS measurements. It is useful for checking control flow, turning directions, GPS recovery, collision clearance, and whether a route reaches its target under the chosen assumptions.

It has not been validated against this physical robot. Wheel slip, sensor visibility, contact with game objects and other real conditions may behave differently. Reaching the target here does not establish that the robot will follow the same path on the competition field.

The VEX compatibility layer implements the APIs needed by the project, rather than the entire VEX runtime. Unsupported calls should produce an explicit error in the console. `src/main.py` is the program to upload to the Brain after setting its camera port and calibration; simulator files are for your computer only.

Currently supported: `Brain` text screen and timer, `Motor` construction/spin/stop/velocity/position, `Gps` mounting/position/heading/quality/calibration, the AprilTag subset of `AiVision` with `Tagdesc`, cooperative `Thread`/`wait`, and basic autonomous `Competition` registration. Additional sensors, motor mechanisms, and drivetrain convenience classes need models before programs using those APIs can run. A program loop must call `wait()` regularly; a busy loop without it is reported instead of freezing the desktop window.

For model provenance, equations and calibration limits, read [PHYSICS_SOURCES.md](PHYSICS_SOURCES.md). Before running checks, read [current test status](../tests/README.md): several controller expectations still describe the archived experiment.
