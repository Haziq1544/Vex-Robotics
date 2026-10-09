# Ontario Skills: GPS-only drive-to-centre test.
# Press Run to calibrate GPS and automatically drive to centre once.
from vex import *
import math

brain = Brain()

# Green cartridge, external motor-to-wheel gearing 1:1.
DRIVE_CARTRIDGE = GearSetting.RATIO_18_1
# Verify these directions with the wheels lifted before the first driving test.
LEFT_REVERSED = False
RIGHT_REVERSED = True
left_motor = Motor(Ports.PORT9, DRIVE_CARTRIDGE, LEFT_REVERSED)
right_motor = Motor(Ports.PORT10, DRIVE_CARTRIDGE, RIGHT_REVERSED)

# Flex sensor build, pages 2-7: GPS is centred on the rear uprights and
# faces backward. Robot forward is toward the claw.
# Offsets are relative to the turning centre: right +X, forward +Y.
GPS_X_OFFSET_MM = 0
# The PDF gives no fore-aft dimension. Retain the existing 2.1-inch estimate;
# verify the horizontal distance from the turning centre to the GPS centre.
GPS_Y_OFFSET_MM = -53.34
GPS_HEADING_OFFSET_DEG = 180  # Rear-facing GPS; report the robot's forward heading.
GPS_MOUNT_CONFIGURED = True
gps = Gps(Ports.PORT18, GPS_X_OFFSET_MM, GPS_Y_OFFSET_MM,
          MM, GPS_HEADING_OFFSET_DEG)

# Standard GPS field coordinates put the arena centre at (0, 0).
TARGET_X_MM = 0
TARGET_Y_MM = 0
ARRIVAL_RADIUS_MM = 100
ARRIVAL_SETTLE_MS = 500
MIN_GPS_QUALITY = 100  # Require position calculated from the field-code strips.
GPS_LOCK_MS = 500
GPS_LOCK_TIMEOUT_MS = 5000
GPS_RECOVERY_TIMEOUT_MS = 2000  # Brake and allow a brief loss of optical lock.
FIELD_LIMIT_MM = 2000  # Reject readings outside the standard 12-foot field.
CONTROL_INTERVAL_MS = 20  # Nominal 50 Hz polling; sensor freshness is hardware-dependent.
DISPLAY_INTERVAL_MS = 20  # Target 50 screen updates/second, allowing margin over 30.
RUN_TIMEOUT_MS = 45000
PROGRESS_TIMEOUT_MS = 3000
DRIVE_PROGRESS_MM = 20
TURN_PROGRESS_DEG = 3
MAX_DRIVE_PERCENT = 50
MIN_DRIVE_PERCENT = 8
MAX_TURN_PERCENT = 20  # Slower initial setting to reduce GPS loss during turns.
MIN_TURN_PERCENT = 6
DISTANCE_GAIN = 0.04
TURN_GAIN = 0.35
STEERING_GAIN = 0.3
TURN_FINISH_DEG = 6
TURN_RESTART_DEG = 20

gps_status = "Not calibrated"
run_status = "Starting GPS calibration"
distance_to_target = None
heading_error = None


def stop_drive():
    """Brake both drive motors."""
    left_motor.stop(BRAKE)
    right_motor.stop(BRAKE)


def screen_line(row, message):
    """Replace one line on the Brain display."""
    brain.screen.set_cursor(row, 1)
    brain.screen.clear_line()
    brain.screen.print(message)


def gps_display():
    """Show GPS readings and the current test result without controlling motors."""
    fps_started = brain.timer.time(MSEC)
    frames = 0
    display_fps = 0
    while True:
        frame_started = brain.timer.time(MSEC)
        screen_line(1, "GPS centre test: (0, 0)")
        screen_line(2, run_status)
        if not gps.installed():
            screen_line(3, "GPS missing: check port 18")
            screen_line(4, "")
            screen_line(5, "")
        elif gps_status != "Ready":
            screen_line(3, gps_status)
            screen_line(4, "")
            screen_line(5, "")
        else:
            screen_line(3, "GPS quality: %d%%" % gps.quality())
            screen_line(4, "X: %.0f Y: %.0f mm" %
                        (gps.x_position(MM), gps.y_position(MM)))
            screen_line(5, "Heading: %.1f deg" % gps.heading())
        screen_line(6, "Distance: %.0f mm" % distance_to_target
                    if distance_to_target is not None else "Distance: --")
        screen_line(7, "Turn error: %.1f deg" % heading_error
                    if heading_error is not None else "Turn error: --")
        screen_line(8, "Display: %.0f FPS" % display_fps)
        frames += 1
        elapsed = brain.timer.time(MSEC) - fps_started
        if elapsed >= 1000:
            display_fps = frames * 1000 / elapsed
            frames = 0
            fps_started = brain.timer.time(MSEC)
        # Account for drawing time instead of adding a full delay after drawing.
        remaining = DISPLAY_INTERVAL_MS - (brain.timer.time(MSEC) - frame_started)
        wait(max(1, remaining), MSEC)


def initialize_gps():
    """Calibrate while stationary, allowing up to ten seconds."""
    global gps_status
    if not gps.installed():
        gps_status = "GPS missing: restart when connected"
        return
    gps_status = "Calibrating: keep robot still"
    gps.calibrate()
    wait(200, MSEC)
    started = brain.timer.time(MSEC)
    while gps.is_calibrating() and brain.timer.time(MSEC) - started < 10000:
        wait(20, MSEC)
    gps_status = "Calibration timeout: restart" if gps.is_calibrating() else "Ready"


def read_pose():
    """Return validated (x_mm, y_mm, heading_deg), or None without a GPS fix."""
    if (gps_status != "Ready" or not GPS_MOUNT_CONFIGURED or
            not gps.installed() or gps.is_calibrating() or
            gps.quality() < MIN_GPS_QUALITY):
        return None
    x = gps.x_position(MM)
    y = gps.y_position(MM)
    heading = gps.heading()
    # These comparisons also reject NaN and infinite readings.
    if not (-FIELD_LIMIT_MM <= x <= FIELD_LIMIT_MM and
            -FIELD_LIMIT_MM <= y <= FIELD_LIMIT_MM and 0 <= heading < 360):
        return None
    return x, y, heading


def wait_for_gps_lock(timeout_ms, run_deadline=None):
    """Stay braked until a valid fix lasts GPS_LOCK_MS, or a deadline expires."""
    stop_drive()
    lock_started = brain.timer.time(MSEC)
    good_since = None
    while True:
        now = brain.timer.time(MSEC)
        if (now - lock_started >= timeout_ms or
                (run_deadline is not None and now >= run_deadline)):
            return None
        pose = read_pose()
        if pose is None:
            good_since = None
        elif good_since is None:
            good_since = now
        elif now - good_since >= GPS_LOCK_MS:
            return pose
        wait(CONTROL_INTERVAL_MS, MSEC)


def target_errors(x, y, heading):
    """Return distance in mm and shortest signed turn in degrees to the target."""
    dx = TARGET_X_MM - x
    dy = TARGET_Y_MM - y
    distance = math.sqrt(dx * dx + dy * dy)
    # VEX: 0 degrees is +Y; headings increase clockwise toward +X.
    bearing = math.atan2(dx, dy) * 180 / math.pi
    error = (bearing - heading + 180) % 360 - 180
    return distance, error


def spin_motor(motor, speed):
    """Command a signed percentage; negative values mean reverse."""
    if speed == 0:
        motor.stop(BRAKE)
    else:
        motor.spin(FORWARD if speed > 0 else REVERSE, abs(speed), PERCENT)


def drive_to_centre():
    """Turn toward the centre, drive with GPS feedback, and return a result."""
    global run_status, distance_to_target, heading_error
    stop_drive()
    distance_to_target = None
    heading_error = None
    if gps_status != "Ready" or not GPS_MOUNT_CONFIGURED:
        return "Stopped: GPS not ready"

    run_status = "Waiting for GPS lock"
    if wait_for_gps_lock(GPS_LOCK_TIMEOUT_MS) is None:
        return "Stopped: no GPS lock"

    started = brain.timer.time(MSEC)
    run_deadline = started + RUN_TIMEOUT_MS
    turning = True
    previous_mode = None
    best_progress = None
    progress_time = started
    settled_since = None
    while True:
        now = brain.timer.time(MSEC)
        if now >= run_deadline:
            return "Stopped: run timed out"
        pose = read_pose()
        if pose is None:
            stop_drive()
            run_status = "GPS lost: waiting for lock"
            distance_to_target = None
            heading_error = None
            pose = wait_for_gps_lock(GPS_RECOVERY_TIMEOUT_MS, run_deadline)
            now = brain.timer.time(MSEC)
            if now >= run_deadline:
                return "Stopped: run timed out"
            if pose is None:
                return "Stopped: GPS recovery timed out"
            # Recompute from the recovered pose. Paused time is not driving time,
            # and an arrival needs a fresh uninterrupted period of valid fixes.
            turning = True
            previous_mode = None
            best_progress = None
            progress_time = now
            settled_since = None
        distance_to_target, heading_error = target_errors(*pose)

        if distance_to_target <= ARRIVAL_RADIUS_MM:
            stop_drive()
            run_status = "Checking centre position"
            if settled_since is None:
                settled_since = now
            if now - settled_since >= ARRIVAL_SETTLE_MS:
                return "Arrived at centre"
            previous_mode = None
            wait(CONTROL_INTERVAL_MS, MSEC)
            continue
        settled_since = None

        # Hysteresis prevents repeatedly switching between turning and driving.
        if turning and abs(heading_error) <= TURN_FINISH_DEG:
            turning = False
        elif not turning and abs(heading_error) >= TURN_RESTART_DEG:
            turning = True
        mode = "Turning" if turning else "Driving"
        metric = abs(heading_error) if turning else distance_to_target
        improvement = TURN_PROGRESS_DEG if turning else DRIVE_PROGRESS_MM
        if best_progress is None or mode != previous_mode:
            best_progress = metric
            progress_time = now
            previous_mode = mode
        elif metric <= best_progress - improvement:
            best_progress = metric
            progress_time = now
        elif now - progress_time >= PROGRESS_TIMEOUT_MS:
            return "Stopped: no progress"
        run_status = mode + " to centre"

        if turning:
            speed = min(MAX_TURN_PERCENT,
                        max(MIN_TURN_PERCENT, abs(heading_error) * TURN_GAIN))
            turn = speed if heading_error > 0 else -speed
            left_speed, right_speed = turn, -turn
        else:
            forward = min(MAX_DRIVE_PERCENT,
                          max(MIN_DRIVE_PERCENT, distance_to_target * DISTANCE_GAIN))
            correction = max(-forward / 2,
                             min(forward / 2, heading_error * STEERING_GAIN))
            left_speed = forward + correction
            right_speed = forward - correction
            # Preserve the steering ratio while limiting both wheel speeds.
            scale = max(1, max(abs(left_speed), abs(right_speed)) / MAX_DRIVE_PERCENT)
            left_speed /= scale
            right_speed /= scale
        spin_motor(left_motor, left_speed)
        spin_motor(right_motor, right_speed)
        wait(CONTROL_INTERVAL_MS, MSEC)


def autonomous():
    """Run one centre test and brake on every exit."""
    global run_status
    try:
        run_status = drive_to_centre()
    except Exception as error:
        run_status = "Stopped: program error"
        print("Centre test error:", error)
    finally:
        stop_drive()
        print(run_status)


# Keep still during calibration. Driving starts automatically after GPS lock.
stop_drive()
brain.screen.clear_screen()
gps_screen_thread = Thread(gps_display)
initialize_gps()
autonomous()
while True:
    # Keep the result visible; rerun the program to start another test.
    stop_drive()
    wait(100, MSEC)
