# Ontario Skills Robot

VEX V5 Python autonomous code for a Flex robot, with a local Python simulator.

- `src/main.py`: approach the first visible goal using AI Vision AprilTags; no fixed centre target.
- `examples/gps_centre_test.py`: archived GPS test, excluded from normal launches and uploads.
- `simulator/`: top-down field view, drivetrain physics, simulated GPS/AI Vision and VEX API support.
- `tests/`: navigation, physics and simulator checks.

## Run the simulator

On this Windows setup, double-click `launch_simulator.bat`. Alternatively, with Python 3.10+ and Tkinter installed:

```sh
python simulate.py
```

In VS Code, select **Flex Simulator** in Run and Debug and press **F5**. The checked-in launch configuration uses the original computer's Python path; update its `python` setting to your installed interpreter on another computer.

Press **Run** inside the simulator. Save changes to `src/main.py`, then Stop and Run again to reload the robot program.

The **Field** tab offers **Override (goals + loaders)**,
**Empty field**, and custom layouts with drag-and-drop editing and JSON save/load.
Goals and loaders are fixed collision obstacles. Cup/stacker code is retained
but disabled: no stackers appear in the field, palette or simulation.

See [simulator instructions](simulator/README.md) and [model assumptions](simulator/PHYSICS_SOURCES.md). Robot dimensions, mass, friction and GPS behaviour are adjustable estimates awaiting physical calibration.

## AI Vision test

Both **Flex Simulator** and **AI Vision Simulator** run the same first-visible-goal
program. `python simulate.py` and the desktop batch launcher do too.
The **Vision** tab previews visible goal tags;
**Model settings → AI Vision** exposes the estimated camera mounting and
detection settings. Drag the robot or enter any valid starting position and
heading. The example start `(1400, -1200)` mm, heading `270°`, faces an off-centre
neutral goal. It is a starting pose, not a programmed destination.

The robot selects the first visible goal and steers toward it, aiming for a
**50 mm chassis-to-goal gap**. With multiple tags in the first frame, the widest
is selected. No visible tag means no motion; target loss brakes the robot.
The Vision tab shows **Locked goal: ID ...** during a run, and the console logs
the loaded program and first selected tag. The program label is read from the
saved source; a file named `main.py` is not automatically treated as AI Vision.
Normal launches reject `src/main.py` if it has reverted to the old GPS program.
Low tags may leave the estimated camera's view before the requested gap, causing
an earlier stop. The real camera port stays unset; the simulator uses a virtual
camera without claiming a physical port.

Read [vision calibration and limitations](simulator/VISION_SOURCES.md) before
hardware testing. This is tag-guided approach, without obstacle route planning.

## Run tests

```sh
python -m unittest discover -s tests -v
```

The robot code uses motor ports **9** and **10**, and GPS port **18**. Simulator files run on the computer; the VEX project uploads `src/main.py`.
