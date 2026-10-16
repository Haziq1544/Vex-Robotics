# Preserved robot experiments

These files are reference programs, not the default upload or simulator entry point.
The active program is `../src/main.py`. Preserve filenames because tests and
documentation refer to them.

| File | Purpose |
| --- | --- |
| `vision_centering_test.py` | Camera-only tag confirmation and centring. No forward movement. User-confirmed hardware baseline. |
| `vision_approach_test.py` | Earlier GPS-search and geometric-distance approach experiment. Retained for comparison; not the current controller. |
| `gps_centre_test.py` | GPS drive-to-centre experiment with recovery after brief loss of GPS. |

Each program contains its own settings. In particular, the archived geometric
approach retains a 140 mm camera height while current simulator defaults use
135.2 mm. Do not assume archived settings match the latest physical setup.

Normal simulator launches and VEX uploads use `src/main.py` only. Selecting an
example explicitly for headless simulation does not change the upload program.
