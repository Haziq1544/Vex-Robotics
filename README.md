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

On the simulator testing branch, the **Field** tab offers **Override (goals + loaders)**,
**Empty field**, and custom layouts with drag-and-drop editing and JSON save/load.
Goals and loaders are fixed collision obstacles. Cup/stacker code is retained
but disabled: no stackers appear in the field, palette or simulation. The current robot code targets
the centre goal's location, so contact and a no-progress stop are expected on
Override; choose Empty field for the original centre-arrival test.

See [simulator instructions](simulator/README.md) and [model assumptions](simulator/PHYSICS_SOURCES.md). Robot dimensions, mass, friction and GPS behaviour are adjustable estimates awaiting physical calibration.

## Run tests

```sh
python -m unittest discover -s tests -v
```

The robot code uses left motor port **10**, right motor port **9**, and GPS port **18**. Simulator files run on the computer; the VEX project uploads `src/main.py`.

GPS navigation accepts quality 90 or higher and still targets (0, 0). The minimum turn command is now 12%, an initial hardware tuning value. Motor connectivity is checked before navigation.

The simulator includes an editable 6% breakaway deadband and a 500 ms estimated-position phase at GPS quality 90 during optical loss, followed by quality 80. These are approximations, not measured VEX behavior. Program completion integrates up to two seconds of braking travel. Recovery-test expectations were updated, but tests have not been run for these changes.
