"""Stage 1: confirm an AprilTag and turn in place to centre it.

No forward drive, distance estimation, GPS search, or Distance Sensor use.
The previous approach experiment is saved in examples/vision_approach_test.py.
Run starts this camera test; stop the program on the Brain to end it early.
"""
from vex import *

# Keep the simulator's vision entry-point marker; mode describes this stage.
PROGRAM_KIND = "ai_vision_first_goal"
VISION_MODE = "tag_centering"
AI_VISION_PORT = Ports.PORT17
LEFT_MOTOR_PORT = Ports.PORT10
RIGHT_MOTOR_PORT = Ports.PORT11
GOAL_TAG_IDS = (0, 1, 2, 3, 4)

IMAGE_CENTRE_X = 160
CONFIRM_FRAMES = 3
CENTRE_TOLERANCE_PX = 8
CENTRE_RELEASE_PX = 12
CENTRE_CONFIRM_FRAMES = 3
TURN_GAIN = 0.15
MIN_TURN_PERCENT = 4.0
MAX_TURN_PERCENT = 8.0
MAX_TRACK_JUMP_PX = 60
REACQUIRE_MS = 1000
STALE_MS = 500
TEST_DURATION_MS = 60000

# Simulator telemetry; no clearance or GPS estimate is produced in this stage.
run_status = "Camera centring test ready"
vision_target_id = None
vision_clearance_mm = None
vision_bearing_deg = None
search_phase = "Camera only: no GPS search"
search_swept_deg = 0.0


def tag_reference(tag):
    # Copy numbers: a hardware snapshot may reuse its object storage.
    return (tag.id, tag.centerX, tag.centerY, tag.width)


def match_tag(objects, reference):
    """Associate a nearby detection; an ID alone does not identify a goal."""
    closest = None
    best = None
    second = None
    for obj in objects:
        if obj.id != reference[0]:
            continue
        dx, dy = abs(obj.centerX - reference[1]), abs(obj.centerY - reference[2])
        if dx > MAX_TRACK_JUMP_PX or dy > MAX_TRACK_JUMP_PX:
            continue
        if not 0.4 <= obj.width / max(1, reference[3]) <= 2.5:
            continue
        distance = dx + dy
        if best is None or distance < best:
            second, best, closest = best, distance, obj
        elif second is None or distance < second:
            second = distance
    if second is not None and second - best < 10:
        return None  # Same-ID faces cannot be distinguished confidently.
    return closest


def run_test():
    global run_status, vision_target_id
    vision_target_id = None
    brain = Brain()
    left = Motor(LEFT_MOTOR_PORT, GearSetting.RATIO_18_1, False)
    right = Motor(RIGHT_MOTOR_PORT, GearSetting.RATIO_18_1, True)
    reference = None
    confirmations = 0
    centred_frames = 0
    centred = False
    last_seen = 0
    image_description = "Image: --"
    error_description = "Centre error: --"

    def brake():
        left.stop(BRAKE)
        right.stop(BRAKE)

    def display(status):
        global run_status
        run_status = status
        brain.screen.clear_screen()
        lines = ("AI Vision: detect and centre", status,
                 "Locked tag: %s" % (vision_target_id if vision_target_id is not None else "--"),
                 image_description, error_description,
                 "Confirmed frames: %d / %d" % (confirmations, CONFIRM_FRAMES),
                 "Camera port: 17; no forward drive")
        row = 1
        for line in lines:
            brain.screen.set_cursor(row, 1)
            brain.screen.print(line)
            row += 1

    brake()
    try:
        if not left.installed() or not right.installed():
            display("Stopped: drive motor missing")
            return
        camera = AiVision(AI_VISION_PORT, AiVision.ALL_TAGS)
        camera.set_tag_family(AiVision.TAG_CIRCLE21H7)
        camera.tag_detection(True)
        started = brain.timer.time(MSEC)
        last_fresh = started
        previous_timestamp = None
        display("Waiting: place a tag in view")
        while brain.timer.time(MSEC) - started < TEST_DURATION_MS:
            now = brain.timer.time(MSEC)
            if not camera.installed():
                display("Stopped: camera disconnected")
                return
            timestamp = camera.timestamp()
            if timestamp == previous_timestamp:
                if now - last_fresh >= STALE_MS:
                    brake()
                    reference = None
                    vision_target_id = None
                    confirmations = centred_frames = 0
                    centred = False
                    image_description = "Image: --"
                    error_description = "Centre error: --"
                    display("Waiting: camera data stale")
                wait(20, MSEC)
                continue
            previous_timestamp = timestamp
            last_fresh = now
            objects = [obj for obj in camera.take_snapshot(AiVision.ALL_TAGS, 24)
                       if obj.id in GOAL_TAG_IDS and obj.width > 0 and obj.height > 0]
            selected = None
            if reference is not None:
                selected = match_tag(objects, reference)
            elif objects:
                selected = objects[0]  # Widest in the first available frame.

            if selected is None:
                brake()
                confirmations = centred_frames = 0
                centred = False
                image_description = "Image: no matching tag"
                error_description = "Centre error: --"
                if reference is not None and now - last_seen < REACQUIRE_MS:
                    display("Paused: tag lost or ambiguous")
                else:
                    reference = None
                    vision_target_id = None
                    display("Waiting: place a tag in view")
                wait(20, MSEC)
                continue

            reference = tag_reference(selected)
            last_seen = now
            confirmations = min(CONFIRM_FRAMES, confirmations + 1)
            error = selected.centerX - IMAGE_CENTRE_X
            image_description = "ID %d: (%d,%d), %dx%d px" % (
                selected.id, selected.centerX, selected.centerY, selected.width, selected.height)
            error_description = "Centre error: %+d px" % error
            if confirmations < CONFIRM_FRAMES:
                brake()
                display("Confirming tag: motors braked")
            else:
                if vision_target_id is None:
                    vision_target_id = selected.id
                    print("Locked confirmed tag: %s" % vision_target_id)
                # Hysteresis prevents small pixel jitter restarting the motors.
                if abs(error) <= CENTRE_TOLERANCE_PX or (centred and abs(error) <= CENTRE_RELEASE_PX):
                    brake()
                    centred_frames = min(CENTRE_CONFIRM_FRAMES, centred_frames + 1)
                    centred = centred_frames >= CENTRE_CONFIRM_FRAMES
                    display("Centred: holding position" if centred else "Confirming centre: braked")
                else:
                    centred = False
                    centred_frames = 0
                    turn = min(MAX_TURN_PERCENT, max(MIN_TURN_PERCENT, abs(error) * TURN_GAIN))
                    # Equal opposite wheel commands: turn only, never approach.
                    if error > 0:
                        left.spin(FORWARD, turn, PERCENT)
                        right.spin(REVERSE, turn, PERCENT)
                    else:
                        left.spin(REVERSE, turn, PERCENT)
                        right.spin(FORWARD, turn, PERCENT)
                    display("Turning to centre tag")
            wait(20, MSEC)
        display("Stopped: camera test finished")
    finally:
        brake()


if __name__ == "__main__":
    run_test()
