"""
Webcam -> MediaPipe hand landmarks -> finger bend -> servo angles -> Arduino.

Controls 6 servos (index, middle, ring, little, thumb, base rotation) via
PCA9685 through an Arduino Nano. The human->robot angle mapping runs
through the formal retargeting pipeline in kinematics.py (observe ->
normalize -> actuate through a calibrated tendon-transmission model) -
see that file for the underlying kinematic model (link lengths, forward
kinematics, DH-style joint chain, capstan transmission).
"""

import glob
import math
import os
import sys
import time

import cv2
import mediapipe as mp
import numpy as np
import serial

from kinematics import (
    TendonTransmission,
    normalize_bend,
    normalize_bend_piecewise,
    normalize_rotation,
    actuate,
    local_frame,
    to_local_frame,
    in_plane_angle_deg,
    roll_around_axis,
)

# ============================================================
# SETTINGS - edit these
# ============================================================

SERIAL_PORT = ""          # leave "" to auto-detect, or set e.g. "/dev/cu.usbserial-10"
BAUD_RATE = 115200

ENABLE_SERVO = True        # set False to test the camera/tracking only, no hand movement

# Your calibrated servo neutral positions (degrees)
# [index, middle, ring, little, thumb, base rotation]
NEUTRAL = [0, 0, 0, 0, 135, 90]

# +1 = servo increases from neutral as the finger bends (tightens string).
# -1 = servo decreases from neutral as the finger bends (reversed linkage).
DIRECTION = [1, 1, 1, 1]

# Thumb spins reversed relative to the mapping below.
THUMB_DIRECTION = -1

# Thumb range: neutral (135, hand fully open/stretched) down to 45 at
# full opposition (thumb brought closest to the other fingers), scaled
# proportionally to how far your real thumb bends. Confirmed safe by
# direct testing across the full 45-135 swing.
THUMB_MAX_MOVEMENT = 90

# Max degrees of movement away from neutral when each finger is fully
# bent (fist). [index, middle, ring, little] - each confirmed safe by
# direct incremental testing (stepping up in ~10-20 degree increments
# and watching/listening for strain), not assumed:
#   index:  150 (strain observed above this)
#   middle: 170 (180 - the servo's absolute limit - did nothing: stalled)
#   ring:   170 (re-tested on the replacement servo, clean up to here)
#   little: 170 (tested together with middle/ring, clean)
FINGER_MAX_MOVEMENT = [150, 170, 170, 170]

# ------------------------------------------------------------------
# Base/wrist rotation (6th servo, channel 7).
#
# Unlike the fingers (which only bend one way from straight), this
# servo is mounted at the arm's base and turns BOTH ways from center to
# mimic wrist pronation/supination (twisting your wrist left/right) -
# a bidirectional joint, handled by kinematics.py's normalize_rotation()
# instead of normalize_bend().
#
# Servo range confirmed safe by direct testing: full 0-180, neutral 90 -
# WRIST_MAX_MOVEMENT is 90 (the full swing either direction from center).
#
# WRIST_NEUTRAL_ANGLE/WRIST_ROTATION_RANGE_DEG measured directly off the
# on-screen WristAngle debug readout: ~90-100 deg facing the camera
# (neutral), ~170 deg at full rotation left, ~-170 deg at full rotation
# right. Right rotation passes smoothly through 0 down to -170 (not a
# short wrap past +-180), confirmed by testing - so the call below uses
# wrap=False (a direct signed delta), not the shortest-path wraparound.
# Real swing is ~75-95 deg each direction - not perfectly symmetric,
# which is normal; 85 is the average, a reasonable middle value.
WRIST_NEUTRAL_ANGLE = 95
WRIST_ROTATION_RANGE_DEG = 85   # real wrist-twist degrees mapped to full servo deflection
WRIST_DIRECTION = 1
WRIST_MAX_MOVEMENT = 90         # confirmed safe: full 0-180 swing from 90 neutral
WRIST_GAIN = 1.0

SMOOTHING = 0.25            # 0-1, higher = snappier but jittery
SEND_INTERVAL = 0.03        # seconds between serial sends (~33 Hz max)

# Tendon transmission objects (see kinematics.py). joint_range_deg ==
# servo_range_deg (ratio 1.0) for all of these because the joint-side
# range hasn't been measured separately from the servo-side range yet -
# only the servo angles have been empirically tuned so far. Measuring
# the actual joint angle at each extreme (e.g. with a protractor against
# the physical finger) and setting joint_range_deg to that measured
# value would make the transmission ratio - and therefore the kinematic
# model - physically accurate instead of a ratio-1 placeholder.
FINGER_TRANSMISSIONS = [TendonTransmission(servo_range_deg=m, joint_range_deg=m) for m in FINGER_MAX_MOVEMENT]
THUMB_TRANSMISSION = TendonTransmission(servo_range_deg=THUMB_MAX_MOVEMENT, joint_range_deg=THUMB_MAX_MOVEMENT)
WRIST_TRANSMISSION = TendonTransmission(servo_range_deg=WRIST_MAX_MOVEMENT, joint_range_deg=WRIST_MAX_MOVEMENT)


# ============================================================
# SERIAL SETUP
#
# A servo current spike (e.g. grip-lock snapping all 6 servos at once)
# can brown out the Arduino's USB connection - the port then vanishes,
# or macOS re-enumerates it under a new name (usbserial-10 ->
# usbserial-110), confirmed by direct testing. This used to crash the
# whole program (sys.exit on a missing/ambiguous port, no handling of
# a mid-run write failure), which is why a brief power hiccup mid-demo
# required a full manual restart. Now: never exit on a port problem -
# always degrade to camera-only and keep retrying in the background, so
# a transient brownout recovers automatically within about a second
# instead of killing the demo.
# ============================================================

def find_port():
    """Most-recently-active usbserial device, or None if none is present.
    Never exits - a missing/ambiguous port is a normal, recoverable
    runtime state here, not a fatal startup error."""
    paths = list(set(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.wchusbserial*")))
    if not paths:
        return None
    paths.sort(key=lambda p: os.path.getmtime(p) if os.path.exists(p) else 0, reverse=True)
    return paths[0]


def connect_serial(retries=1):
    """Try to (re)connect to the Arduino. Returns None on failure instead
    of raising/exiting, so callers can keep the camera loop running."""
    port = SERIAL_PORT or find_port()
    if port is None:
        return None
    for attempt in range(retries):
        try:
            s = serial.Serial(port, BAUD_RATE, timeout=1)
            time.sleep(2)  # Arduino resets on DTR toggle - give it time to boot
            print(f"Arduino connected on {port}.")
            return s
        except (serial.SerialException, OSError) as e:
            print(f"Serial connect on {port} failed ({e}).")
            time.sleep(0.5)
            port = SERIAL_PORT or find_port()
            if port is None:
                return None
    return None


ser = None
if ENABLE_SERVO:
    print("Connecting to Arduino ...")
    ser = connect_serial(retries=5)
    if ser is None:
        print("No Arduino connected yet - running camera/tracking only. "
              "It will connect automatically once the Nano is available.")
else:
    print("ENABLE_SERVO is False - running camera/tracking only, no serial.")


# ============================================================
# MEDIAPIPE SETUP
# ============================================================

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7,
)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Camera could not be opened")
    sys.exit(1)


# ============================================================
# FINGER BEND MATH
#
# Fingers: the real joint angle at the PIP, which tracks how bent the
# finger physically is much more faithfully than a fingertip-to-base
# distance ratio would.
#
# Thumb: the thumb tip's angle within the palm's OWN local coordinate
# frame (see thumb_local_angle() below), not a raw camera-space angle -
# this is what keeps thumb opposition and wrist rotation from bleeding
# into each other (see local_frame() in kinematics.py).
# ============================================================

# MediaPipe hand landmark indices:
# Index:  MCP=5  PIP=6  DIP=7
# Middle: MCP=9  PIP=10 DIP=11
# Ring:   MCP=13 PIP=14 DIP=15
# Little: MCP=17 PIP=18 DIP=19
FINGERS = [
    (5, 6, 7),
    (9, 10, 11),
    (13, 14, 15),
    (17, 18, 19),
]

# PIP joint angle (degrees) when the finger is fully straight vs. fully
# curled into a fist. Tune these if a finger never reaches 0% or 100%.
FINGER_STRAIGHT_ANGLE = 165
FINGER_BENT_ANGLE = 60

# Thumb tip's angle WITHIN THE PALM'S OWN LOCAL FRAME (see
# thumb_base_bend() below and local_frame() in kinematics.py) - NOT a
# raw camera-space angle, so wrist rotation doesn't leak into it.
# Selected live via MediaPipe's handedness detection.
#
# Right hand: precise 5-point calibration (open + touching each finger
# tip in turn), measured directly, not just the two extremes - thumb
# opposition sweeps a wide initial arc to first reach the index finger,
# then smaller, roughly even increments per finger after that, so a
# straight line between just open/opposed would misplace the index/
# middle/ring pinch positions. Used via normalize_bend_piecewise() for
# an accurate curve across the whole range - this is what makes a pinch
# grip (holding a pen, etc.) land correctly, not just open-vs-closed.
THUMB_PINCH_POINTS_RIGHT = [
    (30, 0.00),   # fully open
    (11, 0.25),   # touching index tip
    (3, 0.50),    # touching middle tip
    (-3, 0.75),   # touching ring tip
    (-10, 1.00),  # touching little tip / fully opposed
]

# Left hand: only the two extremes measured so far (right runs ~20 deg
# higher than left for the same pose - a real, consistent offset, not
# noise), so it still uses the simpler 2-point normalize_bend(). Swapped
# relative to the raw reading per direct testing.
THUMB_OPEN_ANGLE_LEFT = -30
THUMB_OPPOSED_ANGLE_LEFT = 10

# Extra multiplier on top of the angle mapping, for more headroom to tune.
THUMB_BEND_GAIN = 1.3


def joint_angle(a, b, c):
    """Angle at vertex b, in degrees, between rays b->a and b->c."""
    v1 = (a.x - b.x, a.y - b.y, a.z - b.z)
    v2 = (c.x - b.x, c.y - b.y, c.z - b.z)
    mag1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2 + v1[2] ** 2)
    mag2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2 + v2[2] ** 2)
    if mag1 == 0 or mag2 == 0:
        return 180.0
    dot = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
    cos_angle = max(-1.0, min(1.0, dot / (mag1 * mag2)))
    return math.degrees(math.acos(cos_angle))


def finger_bend(landmarks, mcp, pip, dip):
    """OBSERVE + NORMALIZE stages (see kinematics.py) for one finger's
    PIP joint."""
    angle = joint_angle(landmarks[mcp], landmarks[pip], landmarks[dip])
    return normalize_bend(angle, FINGER_STRAIGHT_ANGLE, FINGER_BENT_ANGLE)


def _xyz(landmark):
    return (landmark.x, landmark.y, landmark.z)


def build_palm_axes(landmarks, handedness_label):
    """Build the palm's local coordinate frame (see local_frame() in
    kinematics.py) - HANDEDNESS-CORRECTED.

    A cross product is direction-sensitive: a right hand facing the
    camera is the mirror image of a left hand facing the camera (or of
    a right hand facing AWAY from the camera), so computing the palm
    normal the same way regardless of which hand/orientation inverts
    the result for exactly one of those cases. Confirmed by testing:
    worked for the left hand, and for the right hand facing away, but
    was inverted for the right hand facing the camera - all three are
    "the same formula, opposite chirality" cases. Using MediaPipe's own
    handedness detection (instead of silently assuming one hand) fixes
    this at the source rather than patching around it per-orientation."""
    wrist = _xyz(landmarks[0])
    middle_mcp = _xyz(landmarks[9])
    index_mcp = _xyz(landmarks[5])
    pinky_mcp = _xyz(landmarks[17])
    if handedness_label == "Right":
        return local_frame(wrist, middle_mcp, pinky_mcp, index_mcp)
    return local_frame(wrist, middle_mcp, index_mcp, pinky_mcp)


def thumb_local_angle(landmarks, palm_axes):
    """Thumb tip's sweep angle (degrees) WITHIN THE PALM'S OWN LOCAL
    FRAME - not a raw camera-space angle. This is what decouples thumb
    opposition from wrist rotation: both used to be measured as angles
    anchored at the wrist in camera-space, so a pure wrist twist (which
    rotates the whole hand rigidly, thumb included) bled into the thumb
    reading too. Expressing the thumb tip in the palm's own coordinate
    frame instead factors the hand's orientation out first - see
    local_frame() in kinematics.py, and that module's rotation-invariance
    self-test."""
    wrist = _xyz(landmarks[0])
    thumb_tip = _xyz(landmarks[4])
    local_thumb = to_local_frame(thumb_tip, wrist, palm_axes)
    return in_plane_angle_deg(local_thumb)


def thumb_base_bend(landmarks, palm_axes, handedness_label):
    # Thumb-base (CMC) servo moves the thumb ACROSS the palm (opposition),
    # measured in the palm's own local frame so wrist rotation doesn't
    # bleed into it (see thumb_local_angle()). Calibration is per-hand.
    angle = thumb_local_angle(landmarks, palm_axes)
    if handedness_label == "Right":
        # Precise 5-point curve - no gain here, it would pull the servo
        # away from these exactly-measured finger-touch positions.
        return normalize_bend_piecewise(angle, THUMB_PINCH_POINTS_RIGHT)
    return normalize_bend(angle, THUMB_OPEN_ANGLE_LEFT, THUMB_OPPOSED_ANGLE_LEFT, THUMB_BEND_GAIN)


# Fixed external reference wrist_roll_angle() measures roll against.
# Deliberately NOT "world up" (0,-1,0): when you hold your hand facing
# the camera with fingers pointing up (the natural resting pose), the
# palm's forward axis IS roughly (0,-1,0) too - a reference parallel to
# the axis you're measuring roll around is a degenerate case (it always
# reads 0, found by testing). The camera's depth axis is naturally
# perpendicular to a hand held facing the camera, so it doesn't degrade
# in the common case.
ROLL_REFERENCE_AXIS = (0.0, 0.0, 1.0)


def wrist_roll_angle(palm_axes):
    """True axial wrist rotation (pronation/supination): the roll of the
    palm-local frame around its OWN forward (forearm) axis, relative to
    a fixed external reference - not just how tilted the hand looks in
    the 2D image, which also picks up moving your whole arm or bending
    your wrist sideways (radial/ulnar deviation), not just twisting.
    See roll_around_axis() in kinematics.py.

    Caveat: like any single-frame measurement against a fixed external
    reference, this isolates the twist cleanly when your forearm stays
    roughly steady, but can blend with other motion if you swing your
    whole arm through a large angle at the same time - keep your
    forearm roughly steady and twist at the wrist for the cleanest
    reading."""
    return roll_around_axis(palm_axes, ROLL_REFERENCE_AXIS)


# ============================================================
# GRIP LOCK
#
# Once your fingers close around an object, the object blocks the
# camera's view of your hand, so landmark tracking gets noisy or drops
# out right when you need it most. The fix: once the average finger
# bend crosses ENGAGE, commit to a full, firm closure (bend = 1.0 on
# every finger + thumb) instead of chasing noisy partial readings. This
# also maximizes grip force, which matters for heavier objects. It only
# releases once you clearly open your hand again (hysteresis avoids
# flicker near the threshold).
# ============================================================

GRIP_ENGAGE_THRESHOLD = 0.75
GRIP_RELEASE_THRESHOLD = 0.35

gripping = False


# ============================================================
# MAIN LOOP
# ============================================================

current_angles = np.array(NEUTRAL, dtype=float)
last_send = 0.0
last_reconnect_attempt = 0.0
RECONNECT_INTERVAL = 1.0  # don't hammer a missing port every frame

print("Press ESC in the video window to quit.")

while True:
    success, frame = cap.read()
    if not success:
        continue

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)

    if results.multi_hand_landmarks:
        hand = results.multi_hand_landmarks[0]

        handedness_label = "Right"
        if results.multi_handedness:
            handedness_label = results.multi_handedness[0].classification[0].label
        palm_axes = build_palm_axes(hand.landmark, handedness_label)

        finger_bends = [finger_bend(hand.landmark, *finger) for finger in FINGERS]
        thumb_bend = thumb_base_bend(hand.landmark, palm_axes, handedness_label)
        thumb_raw_angle = thumb_local_angle(hand.landmark, palm_axes)
        wrist_raw_angle = wrist_roll_angle(palm_axes)
        # wrap=False: confirmed by testing that your right rotation passes
        # smoothly through 0 down to -170 (not a short wrap past +-180),
        # so the shortest-path wraparound was flipping its sign to match
        # left instead of opposing it.
        wrist_offset = normalize_rotation(
            wrist_raw_angle, WRIST_NEUTRAL_ANGLE, WRIST_ROTATION_RANGE_DEG, WRIST_GAIN, wrap=False,
        )

        avg_bend = sum(finger_bends) / len(finger_bends)
        if not gripping and avg_bend >= GRIP_ENGAGE_THRESHOLD:
            gripping = True
        elif gripping and avg_bend <= GRIP_RELEASE_THRESHOLD:
            gripping = False

        if gripping:
            finger_bends = [1.0] * len(finger_bends)
            thumb_bend = 1.0

        target_angles = [
            int(max(0, min(180, actuate(
                bend, FINGER_TRANSMISSIONS[i], NEUTRAL[i], DIRECTION[i],
            ))))
            for i, bend in enumerate(finger_bends)
        ]
        target_angles.append(
            int(max(0, min(180, actuate(thumb_bend, THUMB_TRANSMISSION, NEUTRAL[4], THUMB_DIRECTION))))
        )
        target_angles.append(
            int(max(0, min(180, actuate(wrist_offset, WRIST_TRANSMISSION, NEUTRAL[5], WRIST_DIRECTION))))
        )
        target_angles = np.array(target_angles, dtype=float)

        current_angles = current_angles * (1 - SMOOTHING) + target_angles * SMOOTHING
        final_angles = [int(round(x)) for x in current_angles]

        now = time.time()
        if now - last_send >= SEND_INTERVAL:
            message = (
                f"{final_angles[0]},{final_angles[1]},{final_angles[2]},"
                f"{final_angles[3]},{final_angles[4]},{final_angles[5]}\n"
            )
            if ser is not None:
                try:
                    ser.write(message.encode())
                except (serial.SerialException, OSError) as e:
                    print(f"Serial write failed ({e}) - will reconnect.")
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
            elif ENABLE_SERVO and now - last_reconnect_attempt >= RECONNECT_INTERVAL:
                last_reconnect_attempt = now
                ser = connect_serial(retries=1)
            last_send = now

        mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

        text = (
            f"S1:{final_angles[0]} S2:{final_angles[1]} S3:{final_angles[2]} "
            f"S4:{final_angles[3]} S5:{final_angles[4]} S6:{final_angles[5]}"
        )
        cv2.putText(frame, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"ThumbAngle: {thumb_raw_angle:.0f}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(frame, f"WristAngle: {wrist_raw_angle:.0f}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(frame, f"Hand: {handedness_label}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        if gripping:
            cv2.putText(frame, "GRIPPING (locked)", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
    else:
        cv2.putText(frame, "NO HAND DETECTED", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    cv2.imshow("Robotic Hand Control", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
if ser is not None:
    ser.close()
print("System stopped")
