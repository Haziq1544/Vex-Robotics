# Model measurements and assumptions

The simulator uses millimetres for public coordinates and SI units internally.
Its field, robot drawing and collision rectangle use the same scale on both axes.

- The [official Override field specifications](https://www.vexrobotics.com/override-manual) show the metal perimeter's inside width as **140.50 inches**: **3568.7 mm**. See the [metal field drawing](https://content.vexrobotics.com/docs/2026-2027/override/online-manual/assets/image/MetalFieldSpecs.png). This is the default; the nominal “12-foot field” includes perimeter structure. Use the physical field's measured interior for calibration. The [portable drawing](https://content.vexrobotics.com/docs/2026-2027/override/online-manual/assets/image/PortableFieldSpecs.png) differs slightly.
- VEX specifies **200 RPM** for the green cartridge in its [11 W motor specifications](https://www.vexrobotics.com/276-4840.html). External gearing is **1:1**, as supplied by the user. A **1.05 N·m** stall torque is a gear-ratio estimate from the red cartridge's documented **2.1 N·m at 100 RPM** in [Understanding V5 Smart Motors](https://kb.vex.com/hc/en-us/articles/360060929971-Understanding-V5-Smart-Motors), not a measured torque curve of this robot.
- The supplied Flex build and sensor-build PDFs informed a rough **6.5 kg** mass estimate, **430 × 400 mm** collision footprint, **330 mm** drive track and **101.6 mm** powered-wheel diameter. These numbers are provisional, not dimensions or a measured total weight stated by the PDFs. The footprint is a simplified body centred at the drivetrain turning centre, including the displayed claw outline.
- The sensor PDF, pages 2–7, shows the GPS centred laterally and facing backward. Its **−53.34 mm** longitudinal offset remains the project's existing unverified estimate. The physical mount in the simulator is separate from the offsets passed to `Gps(...)` by the robot program.
- Tile grid pitch is an illustrative **600 mm**; the walls define the precise simulated boundary. Floor seams have no physical effect in this version.

The engine integrates wheel inertia and robot mass/yaw inertia; motor speed feedback is limited by a simple torque-speed envelope. Tire slip generates traction-limited forces. Braking, coasting, lateral friction, and oriented rectangular wall contacts are modelled. Friction and motor response values require experimental tuning. This is not a recreation of proprietary VEX motor firmware, nor a calibrated foam-tire model.

GPS is sampled with configurable delay/noise. In realistic mode, facing a wall too closely or exceeding a configured turn rate degrades quality; the dropout button supplies repeatable test failures. It does not render a camera image or decode field strips, so these conditions cannot predict exactly when the real GPS loses lock. Game elements do not occlude the simulated GPS or add vision detections.

The optional Override layout adds fixed convex goal/loader footprints.
Contact impulses account for robot translation/rotation;
position correction and movement-limited substeps prevent ordinary drive-speed
overlaps. Cup/stacker definitions are retained for later, but these pieces are
excluded from the active field, editor palette, rendering, and runtime.
Rounded goal edges are sampled with polygon segments. The
engine does not model vertical clearance, tipping, lifting or stacking. See
[FIELD_SOURCES.md](FIELD_SOURCES.md) for the official field geometry and placement.

For calibration, first measure wheel diameter, track width, actual mass and GPS mounting. Then compare straight runs, on-the-spot turns, and braking distances with logged real runs; adjust traction and response constants in the JSON settings. Preserve a copy of the measured configuration using the settings dialog's Save JSON button.
