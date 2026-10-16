# AI Vision test: measurements and limits

The active `src/main.py` confirms and centres an AprilTag, then approaches
calibrated Tag 2 in short steps using its observed image size to stop.
It does not estimate distance or use GPS. The verified centring-only baseline
is preserved in `examples/vision_centering_test.py`.
The earlier geometry-based approach experiment described below is archived in
`examples/vision_approach_test.py`. The simulator supplies the same
pixel-based observations through its VEX adapter. The robot program receives
no simulator coordinates, goal names, or hidden distances.

## Sources and calibration

The [VEX AI Vision API](https://api.vex.com/v5/home/python/Vision/AiVision.html)
defines a 320 by 240 pixel image, width-ordered snapshots, empty tuples when
nothing is detected, and a maximum of 24 returned objects. The installed
V5 Python SDK `V5_1_0_1_25/vexv5/stubs/vex.py` additionally documents
`tag.x`, `tag.y`, `tag_detection`, `set_tag_family`, and `timestamp`.
The latter is a sensor status-packet timestamp; the watchdog cannot prove that
the underlying camera image is fresh.

VEX's [sensor comparison](https://kb.vex.com/hc/en-us/articles/24173352365972-Comparing-the-AI-Vision-Sensor-to-the-V5-Vision-Sensor)
lists a 74-degree horizontal and 63-degree vertical field of view. These set
the ideal pinhole projection. The [AprilTag guide](https://kb.vex.com/hc/en-us/articles/30488063603988-Using-AprilTags-with-the-AI-Vision-Sensor)
and [printable tags](https://content.vexrobotics.com/kb/pdf/aim/aim-april-tags-sizes/AprilTags_Printable_Letter_REV2.pdf)
describe the supported Circle21h7 markers. This test explicitly selects that
family. It does not use the Override AI object classifier, which identifies
pins and cups rather than scoring goals.

The supplied `flex_sensors_build_instructions.pdf`, pages 22–25, places the
camera on the forward-facing claw/lift assembly. With the user's approval,
the pictured lift pose is held fixed. The user has since supplied the temporary
X/Y lens offsets below, measured from the drivetrain turning centre, and a
135.2 mm lens height above the floor with the camera facing forward and level.

| Setting | Value | Status |
| --- | ---: | --- |
| Right offset from turning centre | 0 mm | User test measurement |
| Forward offset | 139.7 mm (5.5 inches) | User test measurement |
| Camera height above floor | 135.2 mm (13.52 cm) | Latest user test measurement |
| Yaw / pitch | 0 / 0 degrees | User reports forward and level |
| Detection-square side | 50 mm (5 cm) | User test measurement; 5 x 5 cm tag |
| Tag tilt from vertical | 0 degrees | User reports upright, non-sloping tags |
| Update rate / delay | 20 Hz / 50 ms | Tunable model assumptions |
| Range limits | 50–2500 mm | Tunable assumptions |
| Minimum projected tag side | 4 pixels | Optimistic geometric threshold, not a reliable decoding specification |
| Maximum viewing angle to panel normal | 75 degrees | Tunable assumption |

Physical model settings are in `default_config.json` and **Model settings → AI
Vision**. The robot program's calibration constants are deliberately separate:
a wrong calibration should affect the estimated target position. For a matched
test, update both sets of camera offsets, orientation, tag size and field of
view, as well as the program's chassis width/length and simulator body size.

The [official goal drawing](https://content.vexrobotics.com/docs/2026-2027/override/online-manual/assets/image/GoalSpecs.png)
shows four tagged faces, a common upper part 82.5 mm high, base flats 142.5 mm
apart, top flats 88.8 mm apart, and tag centres 37.8 mm above the upper part's
base. With the neutral pedestals, tag heights are 178.0 mm for ID 0, 101.8 mm
for IDs 1 and 4, and 37.8 mm for IDs 2 and 3. The estimated horizontal tag-centre
offset remains derived from those dimensions, independently of tag orientation.
The user reports that the tag face stands vertically, so both the simulator's
`vision.tag_tilt_deg` and the robot's `TAG_TILT_DEG` now default to 0 rather than
assuming the tag follows the tapered goal body. Positive tilt tips the top inward.
Tag-centre heights and offsets still need physical verification. The drawing does not give
the detection-square size. See [FIELD_SOURCES.md](FIELD_SOURCES.md) for goal
footprints and placement. Loaders block sightlines but have no simulated tags.

## What is simulated

Each face's four tag corners are projected into the camera image and rounded
to integer pixels. Detection requires the tag to face the camera, fall within
the image and configured range, meet the pixel threshold, and have an
unobstructed centre and corners. Other goals and loaders occlude tags using
their full base footprint extruded to their height. This conservatively
overstates occlusion around tapered objects. The preview shows geometric tag
outlines, not rendered camera video or an AprilTag decoder.

Samples arrive at the configured rate and delay. **Hide tags for 1 second**
injects a detection dropout. No lighting, exposure, lens distortion, motion
blur, print damage, lift movement, or robot mechanism occlusion is modeled.
A synthetic four-pixel detection does not imply that real hardware can read
that tag at the same distance.

## Archived approach and stopping experiment

If the first camera frames contain no goal tag, the robot calibrates its GPS on
port 18 while braked, then performs one slow clockwise turn at 8% power. GPS
heading measures signed angular progress, including wraparound; X/Y detect
unwanted drift and check a full-chassis swept circle against field walls.
This does not infer the position of unseen goals or choose the field centre.
AI Vision keeps checking during the search, and its first goal tag triggers
braking of the search turn and a transition to approach.

A sweep ends after360° without a tag or 25 seconds including GPS startup. It
also ends on stalled rotation, more than 75 mm position drift, invalid heading
jumps, or insufficient wall clearance. GPS loss brakes immediately on the next
control sample and allows up to 2 seconds for a stable fix. Recovery preserves
the accumulated sweep. The robot does not drive forward during search; nearby
unseen obstacles can still intersect its rotating corners. These limits are
editable robot-code settings, and field/chassis dimensions require calibration.

The first nonempty snapshot selects its widest goal tag. The program tracks
that ID using nearby image position and similar apparent size. IDs repeat
across goals and their faces, so they are not unique physical identities.
After acquisition, ambiguous or missing matches stop the run; they do not trigger a search or
switch to a new ID. Closely overlapping identical tags can still be confused.

Distance is estimated from tag corners, known upright goal geometry and camera
calibration. The camera API does not directly measure it. A constrained panel
fit avoids unconstrained pose estimates from tiny tags. It fits either corner
winding and any cyclic starting corner. A conservative octagonal envelope is
used to estimate the nearest gap to the rectangular chassis.

The user has approved steering and a **50 mm** `STOP_CLEARANCE_MM`.
The controller turns toward the first target, approaches at
limited speed, and brakes at the gap plus a 15 mm model allowance. It stops on
target loss, pose failure, disconnection, stale status packets or timeout.
Normal simulator completion advances braking physics for up to one extra
second, and `braking_settled` reports whether motion fell below the small
settling thresholds. Final position and contact counts include that travel.

The example start is **X 1400, Y −1200 mm, heading 270 degrees**, facing an
off-centre goal on Override. Any valid user starting pose is retained. The robot
code receives no preferred field coordinate. This is a visual approach test, not route planning or general
obstacle avoidance. The original Override start can put a nearer goal in the
path to a farther visible tag. Use a clear approach corridor when testing this
controller. Collision bodies remain active; the simulator does not let the
robot pass through anchored goals or loaders.

With the user-supplied level camera at 135.2 mm high, lower tags can pass below the
vertical field of view before the requested 50 mm gap. In those cases the robot
approaches the correct goal but brakes early with `Stopped: target lost`.
The camera/lift geometry needs calibration before expecting the requested gap
at every goal. The simulator does not invent observations after a tag disappears.

The user confirmed that the real camera loses the low goal tag at a 50 mm
front clearance. The new camera-only approach uses the user's Tag 2 reading:
13 x 14 pixels, centre (168, 224), at about 240 mm front clearance, the minimum
full-tag visibility distance. It brakes at width >=12 OR height >=13 pixels,
with a one-pixel margin, or when a tag is within 8 pixels of an image edge.
Only Tag 2 has a stop sample; other IDs centre without driving forward.
Steps use 6% power for a nominal 100 ms, then brake at least 150 ms and reconfirm
centring. Image loss after the first step ends the run instead of switching goals.
This is a visual stop threshold, not an exact millimetre clearance. The simulator
uses the same threshold but may stop at a different gap because the ideal
geometric projection has not been matched to the measured hardware pixel size.
The reported tag top is roughly 70 mm above the goal base, which is not a
tag-centre height measurement and has not replaced the per-goal tag geometry.
The archived approach source retains its earlier calibration settings.

## Transfer to the physical robot

Current test wiring: left motor **Ports.PORT10**, right motor **Ports.PORT11**,
GPS **Ports.PORT18**, AI Vision **Ports.PORT17**. VEX internally numbers these
enum values from zero; the literal 17 would mean physical port 18. The simulator
now follows those SDK semantics and translates to printed port numbers only
inside its physical model. Configuration JSON stores printed numbers 10/11/18/17.
An unset camera port is still allowed only in virtual tests and blocks real
robot motion. Missing drive motors also stop startup.

GPS lens offsets are **X +101.6 mm, Y −162.56 mm**, with its rear-facing180°
heading offset confirmed by the user. Camera X/Y are **0/+139.7 mm**. These are
temporary test measurements, applied to both robot and simulated hardware.

Measure the tag square, fixed lift/camera pose and chassis envelope, set the
same calibration in the robot test, and verify tag corner readings, execution
timing and stopping distance at low speed. The stopping estimate has not been
validated on hardware. Other obstacles and an extended claw can require more
clearance than the current chassis rectangle models.

The VEX project uploads `src/main.py`; all normal simulator launches use that
same source. The hardware API calls are checked against the installed V5 SDK.
The pose solver's execution time on the real Brain and the actual stopping
distance remain unmeasured; passing the simulator is not a hardware validation.
The previous GPS centre test is archived at `examples/gps_centre_test.py` and
uses the same updated wiring when explicitly selected.
The simulator package is desktop-only and is not part of the robot program.
