# Ontario Skills Robot

VEX V5 Python autonomous code for a Flex robot, with a local Python simulator.

- `src/main.py`: confirm and centre an AprilTag, then approach calibrated Tag 2 in short forward steps.
- `examples/vision_centering_test.py`: preserved centring-only program verified on the real robot.
- `examples/vision_approach_test.py`: archived vision/GPS approach experiment, excluded from normal launches and uploads.
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

Both **Flex Simulator** and **AI Vision Simulator** run the same Tag 2 approach
program. `python simulate.py` and the desktop batch launcher do too.
The **Vision** tab previews visible goal tags;
**Model settings → AI Vision** exposes the estimated camera mounting and
detection settings. Drag the robot or enter any valid starting position and
heading. The example start `(1400, -1200)` mm, heading `270°`, faces an off-centre
neutral goal. It is a starting pose, not a programmed destination.

The verified centring stage is preserved separately. This next step adds forward
motion only for **Tag 2**, whose real-camera stop sample is **13 x 14 pixels**
at approximately **240 mm front clearance**, centre `(168, 224)`. This was the
minimum full-tag visibility distance with the current mount, not a guarantee
of stopping clearance. The run brakes and ends as soon as width reaches 12 px
**or** height reaches 13 px, leaving a one-pixel margin before the sample size.
An 8-pixel image-edge guard also ends the run before the tag is clipped.

With no visible tag, the robot waits stationary. It selects the widest goal tag
in the first available frame and requires three consistent readings before
turning. IDs 0 through 4 are accepted; uncalibrated IDs only centre and hold.
It turns in place at 4–8% power. Tag 2 must be within 8 pixels of image centre
for three readings before each forward step. Each step commands 6% power for
a nominal 100 ms, followed by at least 150 ms braked to settle and a fresh
centring check. Sensor/display calls may delay the next timing check.
No distance in millimetres is estimated from the tag.

Before the first forward step, loss or ambiguity brakes and allows one second
to reacquire a nearby matching tag. After forward motion starts, loss,
ambiguity, or stale camera data ends the run without switching goals. A stop
threshold ends the run permanently; pixel jitter cannot restart movement.
The approach has a 30-second timeout, and the whole test has a 60-second limit.
GPS, tag corner fitting and the Distance Sensor are not used.
The real-camera image threshold also applies in the simulator. Its geometric
projection may reach that threshold at a different physical gap; simulator
geometry has not been calibrated to reproduce this hardware pixel sample.
The previous approach program is retained in `examples/vision_approach_test.py`.
The Vision tab shows **Locked goal: ID ...** during a run, and the console logs
the loaded program and first selected tag. The program label is read from the
saved source; a file named `main.py` is not automatically treated as AI Vision.
Normal launches reject `src/main.py` if it has reverted to the old GPS program.
The simulated Brain displays the detected ID, pixel position, size, horizontal
error, confirmation count, and current action. IDs can repeat across goal faces;
nearby image position is used to maintain a visual track, not a unique identity.

Temporary hardware settings for this test:

| Device | Smart Port | X right (+), mm | Y forward (+), mm |
| --- | ---: | ---: | ---: |
| Left drive motor | 10 | — | — |
| Right drive motor | 11 | — | — |
| GPS | 18 | +101.6 | −162.56 |
| AI Vision | 17 | 0 | +139.7 |

Sensor offsets are from the drivetrain turning centre to each lens. GPS faces
backward, so its heading offset is **180°**. These are the user's temporary test
measurements; camera height is 135.2 mm and tag size is 50 mm. Chassis dimensions
and braking still need physical calibration. Green motor cartridges and the existing motor
reversal settings are retained.

Read [vision calibration and limitations](simulator/VISION_SOURCES.md) before
hardware testing. This is tag-guided approach, without obstacle route planning.

## Run tests

```sh
python -m unittest discover -s tests -v
```

The VEX project uploads `src/main.py`; simulator files stay on the computer.
Use `Ports.PORT10`, `Ports.PORT11`, `Ports.PORT18` and `Ports.PORT17` in robot
code: VEX enum values are zero-based internally, whereas printed Smart Port
numbers and simulator model settings are 1–21.
