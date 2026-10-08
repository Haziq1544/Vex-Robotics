# Override field measurements and simulation assumptions

The Override preset follows the supplied overhead image's **head-to-head starting
layout**, with every scoring pin removed. It retains nine goals, four loaders,
four wall-top toggles, and 36 visual-only cups. This is a collision-testing layout, not a
complete match or scoring simulator. It is not the distinct Robot Skills setup.

Dimensions below were checked against VEX's Appendix A engineering drawings on
October 8, 2026. The drawings linked below are primary sources hosted by VEX.
The [manual landing page](https://www.vexrobotics.com/override-manual) links the
[official game manual PDF](https://link.vex.com/docs/26-27/v5rc/game-manual).

## Coordinates and field size

The simulator's origin is the field centre. Positive X points right and positive
Y points up in the audience view; heading zero points up. Red is on the left and
blue is on the right.

The [field element location drawing](https://content.vexrobotics.com/docs/2026-2027/override/online-manual/assets/image/FieldElementLocations.png)
uses a bottom-left origin and a 3566.4 mm portable field interior. Subtracting
1783.2 mm converts its coordinates to centre origin. Interior placement offsets
are approximately 598.1 mm and 1196.1 mm. Published dimensions are rounded, so
symmetry is preserved instead of treating tenth-millimetre differences as real.

The existing simulator defaults to the
[metal field interior of 3568.7 mm](https://content.vexrobotics.com/docs/2026-2027/override/online-manual/assets/image/MetalFieldSpecs.png).
Interior goals and cups retain their measured centre offsets. Wall-mounted
elements and wall cups are positioned relative to the configured wall, rather
than scaling all objects to fit. For very small custom fields, use an empty or
custom layout instead of the competition preset.

## Goals

The [goal specification drawing](https://content.vexrobotics.com/docs/2026-2027/override/online-manual/assets/image/GoalSpecs.png)
gives 142.5 mm across opposite outside flats, an 81.9 mm corner radius measured
from the centre, an 88.8 mm top width, and a 60.1 mm opening. Goal heights are
82.5 mm for alliance goals, 146.5 mm for the four short neutral goals, and
222.7 mm for the centre neutral goal.

The collision outline approximates the rounded corners with polygon segments:
a circle of radius 81.9 mm clipped by the square X/Y limits of +/-71.25 mm.
These are solid, fixed footprints in 2D; openings and height clearance do not
permit the robot's rectangular body to pass through. A regular octagon of the
same width would underestimate the corner extent.

| Goal | X (mm) | Y (mm) |
| --- | ---: | ---: |
| Tall neutral, centre | 0 | 0 |
| Short neutral, upper left | -1196.1 | 598.1 |
| Short neutral, top left | -598.1 | 1196.1 |
| Short neutral, lower right | 1196.1 | -598.1 |
| Short neutral, bottom right | 598.1 | -1196.1 |
| Red, lower left | -1196.1 | -598.1 |
| Red, bottom left | -598.1 | -1196.1 |
| Blue, top right | 598.1 | 1196.1 |
| Blue, upper right | 1196.1 | 598.1 |

Locations come from the field element location drawing above. Anchoring plates,
small fasteners, and taper with height are not separate collision shapes.

## Cups, with pins excluded

The [cup specification drawing](https://content.vexrobotics.com/docs/2026-2027/override/online-manual/assets/image/CupSpecs.png)
gives an 80.2 mm maximum diameter, 59 mm waist diameter, and 164.5 mm height.
The drawing uses a 16-sided outline at radius 40.1 mm. Real cups are movable,
but in this testing phase the simulator keeps the cup/stacker blocks as visual
placeholders only. They do not move, block the robot, or participate in collisions.
Their physical behavior will be implemented later.

Cups have one opaque gray half and one transparent half; they are not red or
blue. Colored centres in the supplied diagram represent pins inside cups and
are absent from this preset. The manual's game primer specifies 24 cups starting
opaque-side-up along the perimeter and 12 starting clear-side-up in the interior.

The [scoring object location drawing](https://content.vexrobotics.com/docs/2026-2027/override/online-manual/assets/image/ScoringObjectLocations.png)
defines the retained 36 on-field cups:

- Four at every sign combination of (+/-1196.1, +/-1196.1) mm.
- Four at every sign combination of (+/-598.1, +/-598.1) mm.
- Four at (-598.1, 0), (598.1, 0), (0, -598.1), and (0, 598.1) mm.
- Twenty-four along the walls: each wall has two groups centred at along-wall
  offsets -598.1 and +598.1 mm. Each group has three cups with offsets -80.2, 0,
  and +80.2 mm. Each centre is 40.1 mm inside its wall, so its outside edge is
  tangent to that wall.

Off-field match loads are omitted. The
[spare scoring-element kit](https://www.vexrobotics.com/276-9255.html) lists a
combined weight of 0.15 kg for one cup and one pin; it does **not** establish an
individual cup mass. Future cup physics will need measured mass, friction,
and contact response; no cup dynamics are enabled now.

## Loaders and toggles

The [loader specification drawing](https://content.vexrobotics.com/docs/2026-2027/override/online-manual/assets/image/LoaderSpecs.png)
gives 95 mm depth into the field, 102.1 mm base width along the wall, and an 86 mm
chute opening. It shows low and high configurations of 365 mm and 583.5 mm total
height. The location drawing places the centre line of each loader 290.6 mm
from its nearest horizontal wall. Two red loaders are on the left wall; two
blue loaders are on the right.

For field half-width Hx and half-height Hy, loader centres are:

- Left: X = -Hx + 47.5 mm, Y = +/-(Hy - 290.6 mm).
- Right: X = Hx - 47.5 mm, Y = +/-(Hy - 290.6 mm).

Each is approximated by a fixed rectangle 95 mm across X and 102.1 mm across Y.
This is a conservative simplified chute-body footprint; it does not reproduce
the tapered top, individual mounting brackets, bolt heads, or height clearance.

The [toggle specification drawing](https://content.vexrobotics.com/docs/2026-2027/override/online-manual/assets/image/ToggleSpecs.png)
gives a 660.2 mm bar span and an axle 335.3 mm above the floor. One toggle is
centred on each wall. These are drawn for orientation; the existing wall
collision handles that boundary. They add no freestanding floor obstacle.
Toggle rotation, elevated mechanism contact, and scoring are not modeled.

## Accuracy limits

Measured dimensions and placement do not make contact dynamics exact. The
robot remains an estimated fixed rectangular footprint; robot contact response
needs physical calibration. A 2D test can verify overlap
handling and navigation clearance, but cannot establish real-world tipping,
mechanism contact, sensor visibility, or competition performance. No vision
sensor or AprilTag detection is added by the field preset.
