"""Stage 2: centre a tag, then approach calibrated Tag 2 in short steps.

Stopping uses the measured image size, not a distance or corner-pose estimate.
No GPS search or Distance Sensor use. Other IDs only centre in this stage.
The verified centring code is saved in examples/vision_centering_test.py.
Run starts this camera test; stop the program on the Brain to end it early.
"""
from vex import *

# Keep the simulator's vision entry-point marker; mode describes this stage.
PROGRAM_KIND = "ai_vision_first_goal"
VISION_MODE = "tag_approach"
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

# User's real-camera reading at the minimum full-tag visibility clearance.
# This is a calibration sample, NOT a live range measurement or guaranteed gap.
APPROACH_TAG_ID = 2
CALIBRATION_FRONT_GAP_MM = 240.0
CALIBRATION_TAG_WIDTH_PX = 13
CALIBRATION_TAG_HEIGHT_PX = 14
STOP_MARGIN_PX = 1
STOP_WIDTH_PX = CALIBRATION_TAG_WIDTH_PX - STOP_MARGIN_PX
STOP_HEIGHT_PX = CALIBRATION_TAG_HEIGHT_PX - STOP_MARGIN_PX
IMAGE_EDGE_MARGIN_PX = 8
APPROACH_SPEED_PERCENT = 6.0
APPROACH_PULSE_MS = 100
APPROACH_SETTLE_MS = 150
APPROACH_TIMEOUT_MS = 30000

# Simulator telemetry; no clearance or GPS estimate is produced in this stage.
run_status = "Camera approach test ready"
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
    approach_started = None
    forward_until = None
    settle_until = 0

    def brake():
        left.stop(BRAKE)
        right.stop(BRAKE)

    def display(status):
        global run_status
        run_status = status
        brain.screen.clear_screen()
        lines = ("AI Vision: centre and approach", status,
                 "Locked tag: %s" % (vision_target_id if vision_target_id is not None else "--"),
                 image_description, error_description,
                 "Confirmed frames: %d / %d" % (confirmations, CONFIRM_FRAMES),
                 "Camera 17; approach Tag %d only" % APPROACH_TAG_ID,
                 "Cal: %dx%d px at %.0f mm gap" % (CALIBRATION_TAG_WIDTH_PX,
                                                   CALIBRATION_TAG_HEIGHT_PX, CALIBRATION_FRONT_GAP_MM),
                 "Brake: width %d OR height %d px" % (STOP_WIDTH_PX, STOP_HEIGHT_PX))
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
            # Enforce the pulse limit even if camera timestamps stop changing.
            if forward_until is not None and now >= forward_until:
                brake()
                forward_until = None
                settle_until = now + APPROACH_SETTLE_MS
                centred = False
                centred_frames = 0
                display("Approach: braking between steps")
            if approach_started is not None and now - approach_started >= APPROACH_TIMEOUT_MS:
                brake()
                display("Stopped: approach time limit")
                return
            if not camera.installed():
                display("Stopped: camera disconnected")
                return
            timestamp = camera.timestamp()
            if timestamp == previous_timestamp:
                if now - last_fresh >= STALE_MS:
                    brake()
                    if approach_started is not None:
                        display("Stopped: approach camera stale")
                        return
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
                if approach_started is not None:
                    display("Stopped: approach target lost")
                    return
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
            can_approach = selected.id == APPROACH_TAG_ID
            if can_approach and (selected.width >= STOP_WIDTH_PX or selected.height >= STOP_HEIGHT_PX):
                brake()
                # A conservative single close reading ends this run. It cannot
                # restart on pixel jitter, even before the initial lock.
                display("Stopped: visual stop reached")
                return
            if can_approach and (selected.originX <= IMAGE_EDGE_MARGIN_PX or
                                 selected.originY <= IMAGE_EDGE_MARGIN_PX or
                                 selected.originX + selected.width >= 320 - IMAGE_EDGE_MARGIN_PX or
                                 selected.originY + selected.height >= 240 - IMAGE_EDGE_MARGIN_PX):
                brake()
                display("Stopped: tag near image edge")
                return
            if forward_until is not None:
                if abs(error) <= CENTRE_TOLERANCE_PX:
                    display("Approaching: short forward step")
                    wait(20, MSEC)
                    continue
                brake()
                forward_until = None
                settle_until = now + APPROACH_SETTLE_MS
                centred = False
                centred_frames = 0
            if now < settle_until:
                brake()
                display("Approach: braking between steps")
                wait(20, MSEC)
                continue
            if confirmations < CONFIRM_FRAMES:
                brake()
                display("Confirming tag: motors braked")
            else:
                if vision_target_id is None:
                    vision_target_id = selected.id
                    print("Locked confirmed tag: %s" % vision_target_id)
                # Preserve centring hysteresis for uncalibrated IDs. Forward
                # steps require the tighter +/-8 px alignment every time.
                if abs(error) <= CENTRE_TOLERANCE_PX or (not can_approach and centred and abs(error) <= CENTRE_RELEASE_PX):
                    brake()
                    centred_frames = min(CENTRE_CONFIRM_FRAMES, centred_frames + 1)
                    centred = centred_frames >= CENTRE_CONFIRM_FRAMES
                    if centred and can_approach:
                        if approach_started is None:
                            approach_started = now
                        forward_until = now + APPROACH_PULSE_MS
                        left.spin(FORWARD, APPROACH_SPEED_PERCENT, PERCENT)
                        right.spin(FORWARD, APPROACH_SPEED_PERCENT, PERCENT)
                        display("Approaching: short forward step")
                    elif centred:
                        display("Centred: this ID has no stop sample")
                    else:
                        display("Confirming centre: braked")
                else:
                    centred = False
                    centred_frames = 0
                    turn = min(MAX_TURN_PERCENT, max(MIN_TURN_PERCENT, abs(error) * TURN_GAIN))
                    # Re-centre in place before the next forward step.
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
