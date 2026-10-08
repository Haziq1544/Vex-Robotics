# Ontario Skills Robot

VEX V5 Python autonomous code for a Flex robot, with a local Python simulator.

- `src/main.py`: the program uploaded to the robot; currently a GPS-guided drive-to-centre test.
- `simulator/`: top-down field view, drivetrain physics, simulated GPS and VEX API support.
- `tests/`: navigation, physics and simulator checks.

## Run the simulator

On this Windows setup, double-click `launch_simulator.bat`. Alternatively, with Python 3.10+ and Tkinter installed:

```sh
python simulate.py
```

In VS Code, select **Flex Simulator** in Run and Debug and press **F5**. The checked-in launch configuration uses the original computer's Python path; update its `python` setting to your installed interpreter on another computer.

Press **Run** inside the simulator. Save changes to `src/main.py`, then Stop and Run again to reload the robot program.

On the simulator testing branch, the **Field** tab offers **Override (no pins)**,
**Empty field**, and custom layouts with drag-and-drop editing and JSON save/load.
Goals and loaders are fixed collision obstacles. Cup/stacker blocks remain
visual placeholders for later implementation. The current robot code targets
the centre goal's location, so contact and a no-progress stop are expected on
Override; choose Empty field for the original centre-arrival test.

See [simulator instructions](simulator/README.md) and [model assumptions](simulator/PHYSICS_SOURCES.md). Robot dimensions, mass, friction and GPS behaviour are adjustable estimates awaiting physical calibration.

## Run tests

```sh
python -m unittest discover -s tests -v
```

The robot code uses motor ports **9** and **10**, and GPS port **18**. Simulator files run on the computer; the VEX project uploads `src/main.py`.
