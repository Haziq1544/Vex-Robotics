# Project status

Reviewed October 15, 2026. This is a dated snapshot, not a live Git report.

## Working location

Active project: `C:/Users/1khan/Documents/vex-vscode-projects/Ontario_Skills_bot`.
Open it using **Ontario Skills Workspace** on the Desktop.

The old desktop copy is preserved as **Ontario_Skills_bot - Archived Copy**.
It has its own Git history and uncommitted changes. It is an archive, not the
active development folder. Nothing from it was merged into this project.

## Branches and checkpoints

| Branch | Purpose/status at cleanup |
| --- | --- |
| `main` | `52694a9`: GPS drive-to-centre baseline and field editor/collisions. |
| `sim_branch` | Also `52694a9`; its earlier changes were already merged into `main`. |
| `game_element_detection` | `7ece873`: earlier AI Vision and GPS-search experiment. |
| `sim_dev` | Active development; current working files saved as a checkpoint, followed by documentation cleanup. |

No branches were merged, deleted, or renamed during cleanup. Physical testing
must be completed before deciding whether to merge development work.
Use Git's current log/status for the latest commit IDs and remote synchronization.

## Current versus archived behaviour

| Program | Behaviour | Status |
| --- | --- | --- |
| `src/main.py` | Centre a tag; step toward calibrated Tag 2 only; stop by image size. | Current; approach needs physical validation. |
| `examples/vision_centering_test.py` | Confirm and centre a tag without forward motion. | Preserved baseline; user confirmed centring on hardware. |
| `examples/vision_approach_test.py` | GPS search plus geometric tag-position estimation and approach. | Archived experiment; hardware pose-fit issues led to simpler staged testing. |
| `examples/gps_centre_test.py` | Drive to field centre using GPS, with dropout recovery. | Archived test, updated to the temporary wiring. |

The simulator runs the actual selected Python program using a simulated subset
of the VEX API. It already supports running without a window. Tkinter owns the
desktop presentation; the physics and runtime are separate modules.

## Next robotics work

1. Physically validate the new Tag 2 approach and record the final pixel reading
   and actual front clearance. Keep changes small enough to diagnose individually.
2. Measure chassis dimensions, mass, wheel size, and braking response to improve
   calibration. The camera model has not been fitted to the hardware pixel sample.
3. Update tests deliberately: retain coverage for archived experiments and add
   appropriate coverage for the current approach. See `tests/README.md`.
4. Review and merge only when physical testing is complete and the owner requests it.

## Future web project

First learn HTML, CSS, HTTP/client–server concepts, frameworks, Flask, databases,
and JavaScript through small exercises and a blog application. Then package the
simulator for reuse and create a separate Flask web-server repository.

A browser interface will send commands and display state; the Python simulation
will continue to run on the local computer. Separate repositories alone do not
make microservices. Choose service boundaries when their benefits are understood.
