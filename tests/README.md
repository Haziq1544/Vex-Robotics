# Automated checks

These files check behaviour without being part of a normal robot upload or
simulator launch. Keep them: they document expected behaviour and help detect regressions.

## Current status — October 15, 2026

**The complete suite is not verified for the current Tag 2 approach.** Previous
passing counts apply to earlier versions. Workspace cleanup did not run or
modify test code, robot code, or simulator code.

| Files | Scope / maintenance needed |
| --- | --- |
| `test_sim_physics.py`, `test_element_physics.py` | Drivetrain motion, braking, walls, and anchored obstacle collisions. |
| `test_field_layouts.py` | Field geometry, layout validation/persistence, and archived GPS-program collisions. |
| `test_sim_runtime.py`, `test_gps_recovery.py` | Runtime, simulated GPS, and the archived GPS centre program. |
| `test_sim_vision.py` | Camera geometry and adapter checks, mixed with older controller expectations. Several tests still load `src/main.py` but expect removed pose-estimation functions, GPS constants, or old statuses. |
| `test_gps_vision_search.py` | Still loads `src/main.py` expecting GPS searching. Current code intentionally waits without a visible tag. |
| `test_brain_display.py` | Still expects the older six-row display and variables; current display differs. |
| `test_field_editor_ui.py` | Desktop editor checks; includes an outdated expected program label. |

## Planned maintenance

Separate archived-controller checks from current-controller checks. Preserve
useful assertions rather than weakening them to make results pass. Add coverage
for current tag confirmation, centring, Tag 2 pulses, stop thresholds, image-edge
guards, and loss handling when development resumes.

## Run manually

From the project root:

```powershell
python -m unittest discover -s tests -v
```

Some UI checks need a graphical display. The command may report failures from
the known outdated expectations above; this is not evidence that every failure
is harmless. Review each result. Tests are left for the owner to run unless requested.
