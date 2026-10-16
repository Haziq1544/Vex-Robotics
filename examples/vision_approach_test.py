"""VEXcode Python: approach the first visible goal, with no fixed field target.

Test wiring: left10, right11, GPS18, AI Vision17. Simulator
support is detected explicitly. Measure camera
mount, chassis size and tag size before relying on physical stopping distances.
If no goal is initially visible, GPS guides one clockwise search sweep in place.
The first returned goal tag is locked. Loss/ambiguity after acquisition stops
the test rather than switching targets. Repeated IDs cannot uniquely name
physical goals, so this is a continuous visual track, not global identification.
"""
from vex import *
import math

PROGRAM_KIND = "ai_vision_first_goal"
AI_VISION_PORT = Ports.PORT17
LEFT_MOTOR_PORT = Ports.PORT10
RIGHT_MOTOR_PORT = Ports.PORT11
GPS_PORT = Ports.PORT18
# User-approved steering and chassis-to-goal stopping gap.
APPROACH_MODE = "steer"  # "steer" or "straight"
STOP_CLEARANCE_MM = 50.0

# Test offsets and forward, level camera orientation supplied by the user.
# Reference: drivetrain turning centre; right +X, forward +Y.
CAMERA_X_MM = 0.0
CAMERA_Y_MM = 139.7  # 5.5 inches forward
CAMERA_HEIGHT_MM = 140.0  # lens 14 cm above the floor
CAMERA_YAW_DEG = 0.0
CAMERA_PITCH_DEG = 0.0
# User-supplied square size for this real-field test.
TAG_SIZE_MM = 50.0  # 5 x 5 cm
TAG_TILT_DEG = 0.0  # user confirms upright tag; 0 = vertical, positive = top inward
TAG_CENTRE_HEIGHTS_MM = {0: 178.0, 1: 101.8, 4: 101.8, 2: 37.8, 3: 37.8}
HORIZONTAL_FOV_DEG = 74.0
VERTICAL_FOV_DEG = 63.0
ROBOT_WIDTH_MM = 400.0
ROBOT_LENGTH_MM = 430.0
MAX_DRIVE_PERCENT = 15.0
MAX_TURN_PERCENT = 12.0
GOAL_TAG_IDS = (0, 1, 2, 3, 4)

# Existing rear-facing GPS on port 18; offsets relative to the turning centre.
GPS_X_OFFSET_MM = 101.6  # 4 inches right
GPS_Y_OFFSET_MM = -162.56  # 6.4 inches back
GPS_HEADING_OFFSET_DEG = 180.0
CAMERA_STARTUP_GRACE_MS = 150
SEARCH_TURN_PERCENT = 8.0
SEARCH_TIMEOUT_MS = 25000
SEARCH_GPS_CALIBRATION_MS = 10000
SEARCH_GPS_RECOVERY_MS = 2000
SEARCH_GPS_LOCK_MS = 200
SEARCH_NO_PROGRESS_MS = 2000
SEARCH_DRIFT_LIMIT_MM = 75.0
SEARCH_WALL_MARGIN_MM = 20.0
# Physical field bounds for the turn-clearance check, not a goal destination.
FIELD_HALF_WIDTH_MM = 1784.35
FIELD_HALF_HEIGHT_MM = 1784.35
APPROACH_TIMEOUT_MS = 30000

run_status = "Vision test ready"
vision_target_id = None
vision_clearance_mm = None
vision_bearing_deg = None
search_swept_deg = 0.0
search_phase = "Not needed"


def _distance(dx, dy):
    # VEX MicroPython's math subset does not provide math.hypot.
    return math.sqrt(dx * dx + dy * dy)


class GpsGoalSearch:
    """Measure one in-place sweep using GPS; goal positions are not assumed."""
    def __init__(self, gps):
        self.gps = gps
        self.started_ms = None
        self.origin = None
        self.previous_heading = None
        self.swept_deg = 0.0
        self.best_sweep = 0.0
        self.progress_ms = 0
        self.good_since = None
        self.bad_since = None

    def update(self, now_ms):
        """Return clockwise motor percentage and status; zero means brake."""
        if not self.gps.installed():
            return 0, "Stopped: GPS unavailable for search"
        if self.started_ms is None:
            self.started_ms = now_ms
            self.gps.calibrate()
        if self.gps.is_calibrating():
            if now_ms - self.started_ms >= SEARCH_GPS_CALIBRATION_MS:
                return 0, "Stopped: GPS calibration timed out"
            return 0, "Calibrating GPS for search"
        if now_ms - self.started_ms >= SEARCH_TIMEOUT_MS:
            return 0, "Stopped: goal search timed out"

        x, y = self.gps.x_position(MM), self.gps.y_position(MM)
        heading = self.gps.heading(DEGREES)
        valid = (self.gps.quality() >= 100 and -2000 <= x <= 2000 and
                 -2000 <= y <= 2000 and 0 <= heading < 360)
        if not valid:
            self.good_since = None
            if self.bad_since is None:
                self.bad_since = now_ms
            if now_ms - self.bad_since >= SEARCH_GPS_RECOVERY_MS:
                return 0, "Stopped: GPS recovery timed out"
            return 0, "GPS lost: search paused"

        # Account for the full rectangle's swept radius, including corner swing.
        radius = _distance(ROBOT_WIDTH_MM / 2, ROBOT_LENGTH_MM / 2) + SEARCH_WALL_MARGIN_MM
        if abs(x) + radius >= FIELD_HALF_WIDTH_MM or abs(y) + radius >= FIELD_HALF_HEIGHT_MM:
            return 0, "Stopped: too close to wall to search"
        if self.origin is None:
            self.origin = (x, y)
            self.previous_heading = heading
            self.progress_ms = now_ms
        elif _distance(x - self.origin[0], y - self.origin[1]) > SEARCH_DRIFT_LIMIT_MM:
            return 0, "Stopped: robot moved during search"
        if self.previous_heading is not None:
            delta = (heading - self.previous_heading + 180) % 360 - 180
            if abs(delta) > 30:
                return 0, "Stopped: GPS heading jumped"
            # Signed accumulation handles 359->0 and does not count jitter twice.
            self.swept_deg += delta
            self.previous_heading = heading
        if self.swept_deg >= 360:
            return 0, "Stopped: no goal found after full scan"

        if self.good_since is None:
            self.good_since = now_ms
        if now_ms - self.good_since < SEARCH_GPS_LOCK_MS:
            if self.bad_since is not None and now_ms - self.bad_since >= SEARCH_GPS_RECOVERY_MS:
                return 0, "Stopped: GPS recovery timed out"
            self.progress_ms = now_ms
            return 0, "Waiting for GPS search lock"
        self.bad_since = None
        if self.swept_deg > self.best_sweep + 3:
            self.best_sweep = self.swept_deg
            self.progress_ms = now_ms
        elif now_ms - self.progress_ms >= SEARCH_NO_PROGRESS_MS:
            return 0, "Stopped: search turn stalled"
        return SEARCH_TURN_PERCENT, "Searching for first goal"


def observed_goal(tag):
    # Fit the four pixel rays to the known, upright goal panel. Constraining
    # height/tilt avoids an unconstrained 6D pose becoming unstable on tiny tags.
    # This uses goal dimensions and tag ID, never a map or simulated position.
    tag_height = TAG_CENTRE_HEIGHTS_MM[tag.id]
    fx = 160 / math.tan(math.radians(HORIZONTAL_FOV_DEG) / 2)
    fy = 120 / math.tan(math.radians(VERTICAL_FOV_DEG) / 2)
    pitch = math.radians(CAMERA_PITCH_DEG)
    cp, sp = math.cos(pitch), math.sin(pitch)
    z = tag_height - CAMERA_HEIGHT_MM
    tilt = math.radians(TAG_TILT_DEG)
    pixels = list(zip(tag.tag.x, tag.tag.y))
    if len(pixels) != 4:
        raise ValueError("Need 4 corners, got %d" % len(pixels))
    signed_area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pixels, pixels[1:] + pixels[:1]))
    if abs(signed_area) < 1:
        raise ValueError("Collapsed or crossed corners")
    # Firmware corner start/winding is not assumed. The fit tries each start;
    # normalize winding to the visible panel projection before fitting.
    if signed_area > 0:
        pixels.reverse()
    best = None
    for angle_deg in range(0, 360, 4):
        angle = math.radians(angle_deg)
        nx, ny = math.sin(angle), math.cos(angle)
        offsets = [(TAG_SIZE_MM / 2 * (sx * ny - sy * nx * math.sin(tilt)),
                    TAG_SIZE_MM / 2 * (-sx * nx - sy * ny * math.sin(tilt)),
                    TAG_SIZE_MM / 2 * sy * math.cos(tilt))
                   for sx, sy in ((-1, 1), (1, 1), (1, -1), (-1, -1))]
        for shift in range(4):
            paired = pixels[shift:] + pixels[:shift]
            a11 = a12 = a22 = b1 = b2 = 0.0
            for (px, py), (dx, dy, dz) in zip(paired, offsets):
                u, v = (px - 160) / fx, (120 - py) / fy
                for aa, bb, rhs in ((fx, -u * cp * fx, (u * (cp * dy + sp * (z + dz)) - dx) * fx),
                                    (0, (-sp - v * cp) * fy,
                                     (v * (cp * dy + sp * (z + dz)) - cp * (z + dz) + sp * dy) * fy)):
                    a11 += aa * aa
                    a12 += aa * bb
                    a22 += bb * bb
                    b1 += aa * rhs
                    b2 += bb * rhs
            determinant = a11 * a22 - a12 * a12
            if abs(determinant) < 1e-8:
                continue
            x = (b1 * a22 - b2 * a12) / determinant
            y = (b2 * a11 - b1 * a12) / determinant
            if y < 30 or y > 5000 or nx * x + ny * y >= 0:
                continue
            error = 0.0
            for (px, py), (dx, dy, dz) in zip(paired, offsets):
                depth = cp * (y + dy) + sp * (z + dz)
                if depth <= 1:
                    error = float("inf")
                    break
                ex = 160 + fx * (x + dx) / depth
                ey = 120 - fy * (cp * (z + dz) - sp * (y + dy)) / depth
                error += (ex - px) ** 2 + (ey - py) ** 2
            if best is None or error < best[0]:
                best = (error, x, y, nx, ny)
    if best is None:
        raise ValueError("No forward pose fits tag")
    if best[0] > 16:
        # RMS pixel distance per corner; preserve the existing rejection limit.
        raise ValueError("Fit %.1f px; limit 2.0" % math.sqrt(best[0] / 4))
    _, x, y, nx, ny = best
    yaw = math.radians(CAMERA_YAW_DEG)
    tag_x, tag_y = x * math.cos(yaw) + y * math.sin(yaw), -x * math.sin(yaw) + y * math.cos(yaw)
    nx, ny = nx * math.cos(yaw) + ny * math.sin(yaw), -nx * math.sin(yaw) + ny * math.cos(yaw)
    # Goal top/base dimensions from official GoalSpecs; tag centre z37.8
    # above the base of its common upper part. Neutral pedestal doesn't change XY.
    face_offset = 71.25 - (37.8 - 10) * (71.25 - 44.4) / (82.5 - 10)
    gx, gy = tag_x + CAMERA_X_MM - face_offset * nx, tag_y + CAMERA_Y_MM - face_offset * ny
    # Conservative octagonal envelope of the goal's rounded corners.
    a, b = 71.25, math.sqrt(2) * 81.9 - 71.25
    local = ((a, b), (b, a), (-b, a), (-a, b), (-a, -b), (-b, -a), (b, -a), (a, -b))
    polygon = [(gx + x * ny + y * nx, gy - x * nx + y * ny) for x, y in local]
    return gx, gy, polygon


def _point_segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0, min(1, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / (dx * dx + dy * dy)))
    return _distance(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


def clearance(polygon):
    """Minimum planar gap between observed goal envelope and chassis rectangle."""
    w, l = ROBOT_WIDTH_MM / 2, ROBOT_LENGTH_MM / 2
    robot = [(-w, -l), (w, -l), (w, l), (-w, l)]
    separated = False
    for shape in (robot, polygon):
        for a, b in zip(shape, shape[1:] + shape[:1]):
            nx, ny = -(b[1] - a[1]), b[0] - a[0]
            r = [x * nx + y * ny for x, y in robot]
            g = [x * nx + y * ny for x, y in polygon]
            if min(g) > max(r) or min(r) > max(g):
                separated = True
    if not separated:
        return 0.0
    return min(_point_segment_distance(p, a, b) for first, second in ((robot, polygon), (polygon, robot))
               for p in first for a, b in zip(second, second[1:] + second[:1]))


def run_test():
    global run_status, vision_target_id, vision_clearance_mm, vision_bearing_deg
    global search_swept_deg, search_phase
    vision_target_id = vision_clearance_mm = vision_bearing_deg = None
    search_swept_deg, search_phase = 0.0, "Not needed"
    brain = Brain()
    left = Motor(LEFT_MOTOR_PORT, GearSetting.RATIO_18_1, False)
    right = Motor(RIGHT_MOTOR_PORT, GearSetting.RATIO_18_1, True)

    def brake():
        left.stop(BRAKE)
        right.stop(BRAKE)

    def display(status, details=()):
        global run_status
        run_status = status
        brain.screen.clear_screen()
        lines = ("AI Vision: first goal", status,
                 "Tag: %s" % vision_target_id,
                 "Gap estimate: %s mm" % (round(vision_clearance_mm) if vision_clearance_mm is not None else "--"),
                 "Camera port: %s" % (int(AI_VISION_PORT) + 1 if AI_VISION_PORT is not None else "virtual / unset"),
                 "Search sweep: %.0f / 360 deg" % search_swept_deg) + details
        # Avoid enumerate: unavailable on the Brain used for this test.
        row = 1
        for line in lines:
            brain.screen.set_cursor(row, 1)
            brain.screen.print(line)
            row += 1

    brake()
    if not left.installed() or not right.installed():
        display("Stopped: drive motor missing")
        return
    if AI_VISION_PORT is None and not getattr(AiVision, "SIMULATED", False):
        display("Stopped: set AI_VISION_PORT")
        return
    if APPROACH_MODE not in ("steer", "straight") or STOP_CLEARANCE_MM is None or STOP_CLEARANCE_MM <= 0:
        display("Stopped: set approach mode and gap")
        return
    if AI_VISION_PORT in (LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT, GPS_PORT):
        display("Stopped: camera port conflicts")
        return
    camera = AiVision(AI_VISION_PORT, AiVision.ALL_TAGS)
    camera.set_tag_family(AiVision.TAG_CIRCLE21H7)
    camera.tag_detection(True)
    gps = Gps(GPS_PORT, GPS_X_OFFSET_MM, GPS_Y_OFFSET_MM, MM, GPS_HEADING_OFFSET_DEG)
    search = GpsGoalSearch(gps)
    previous = None
    previous_timestamp = None
    last_fresh = brain.timer.time(MSEC)
    started = last_fresh
    approach_started = None
    settle_until = started
    try:
        while True:
            now = brain.timer.time(MSEC)
            if approach_started is not None and now - approach_started >= APPROACH_TIMEOUT_MS:
                display("Stopped: vision test timeout")
                return
            if not camera.installed():
                display("Stopped: camera disconnected")
                return
            timestamp = camera.timestamp()
            if timestamp == previous_timestamp:
                if brain.timer.time(MSEC) - last_fresh > 250:
                    display("Stopped: camera data stale")
                    return
                wait(20, MSEC)
                continue
            previous_timestamp = timestamp
            last_fresh = brain.timer.time(MSEC)
            objects = [obj for obj in camera.take_snapshot(AiVision.ALL_TAGS, 24) if obj.id in GOAL_TAG_IDS]
            if previous is None:
                if not objects:
                    if now - started < CAMERA_STARTUP_GRACE_MS:
                        turn, search_phase = 0, "Waiting for first camera frame"
                    else:
                        turn, search_phase = search.update(now)
                    search_swept_deg = search.swept_deg
                    if turn:
                        left.spin(FORWARD, turn, PERCENT)
                        right.spin(REVERSE, turn, PERCENT)
                    else:
                        brake()
                    display(search_phase)
                    if search_phase.startswith("Stopped:"):
                        return
                    wait(20, MSEC)
                    continue
                brake()
                selected = objects[0]  # API order: greatest width first in the first nonempty frame.
                vision_target_id = selected.id
                approach_started = now
                settle_until = now + (150 if search.started_ms is not None else 0)
                search_phase = "Goal acquired"
                print("Locked first visible goal: tag %s" % vision_target_id)
            else:
                previous_x, previous_y, previous_width = previous.centerX, previous.centerY, previous.width
                matches = [obj for obj in objects if obj.id == vision_target_id and
                           _distance(obj.centerX - previous_x, obj.centerY - previous_y) <= 45 and
                           0.4 <= obj.width / max(1, previous_width) <= 2.5]
                if not matches:
                    display("Stopped: target lost")
                    return
                matches.sort(key=lambda obj: _distance(obj.centerX - previous_x, obj.centerY - previous_y))
                if len(matches) > 1 and abs(matches[0].centerX - matches[1].centerX) < 5:
                    display("Stopped: target ambiguous")
                    return
                selected = matches[0]
            previous = selected
            # Do not move during a calibration we started, even if a tag appears.
            if search.started_ms is not None and gps.is_calibrating():
                brake()
                if now - search.started_ms >= SEARCH_GPS_CALIBRATION_MS:
                    display("Stopped: GPS calibration timed out")
                    return
                display("Goal found: waiting for GPS calibration")
                wait(20, MSEC)
                continue
            if now < settle_until:
                brake()
                display("Goal found: braking search turn")
                wait(20, MSEC)
                continue
            try:
                gx, gy, polygon = observed_goal(selected)
                vision_clearance_mm = clearance(polygon)
                vision_bearing_deg = math.degrees(math.atan2(gx, gy))
            except (ValueError, ZeroDivisionError) as error:
                brake()
                # Show the actual rejection rather than hiding every cause
                # behind the same status. Do not drive on a rejected estimate.
                display("Stopped: tag pose unreliable", (
                    "Pose: %s" % error,
                    "Image %dx%d px at %d,%d" % (selected.width, selected.height,
                                                selected.centerX, selected.centerY),
                    "Tag size %.1f; height %.1f mm" % (TAG_SIZE_MM, TAG_CENTRE_HEIGHTS_MM[selected.id])))
                print("Pose rejected: %s" % error)
                print("Tag corners X: %s" % str(selected.tag.x))
                print("Tag corners Y: %s" % str(selected.tag.y))
                return
            # Brake on the first close reading, with a small allowance for
            # frame delay and braking distance. This is still an estimate.
            if vision_clearance_mm <= STOP_CLEARANCE_MM + 15:
                display("Arrived: stopped before goal")
                return
            if gy <= 0 or abs(vision_bearing_deg) > 65:
                display("Stopped: target outside approach")
                return
            if APPROACH_MODE == "straight" and abs(vision_bearing_deg) > 4:
                display("Stopped: goal is not straight ahead")
                return
            turn = max(-MAX_TURN_PERCENT, min(MAX_TURN_PERCENT, vision_bearing_deg * 0.45)) if APPROACH_MODE == "steer" else 0
            drive = min(MAX_DRIVE_PERCENT, max(4, (vision_clearance_mm - STOP_CLEARANCE_MM) * 0.06))
            if abs(vision_bearing_deg) > 12:
                drive = 0
            left.spin(FORWARD, drive + turn, PERCENT)
            right.spin(FORWARD, drive - turn, PERCENT)
            display("Approaching first goal")
            wait(20, MSEC)
    finally:
        brake()


if __name__ == "__main__":
    run_test()
