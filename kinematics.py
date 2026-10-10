"""
Forward kinematics and tendon-transmission model for an underactuated,
tendon-driven anthropomorphic hand.

This module is deliberately decoupled from the camera/serial live loop in
hand_robot.py - it IS the formal kinematic model: link lengths, joint
chains, forward kinematics, and the human -> robot motion retargeting
pipeline. Run it directly (`python kinematics.py`) for a standalone demo
and self-test that needs no camera, no Arduino, no servos.

KINEMATIC MODEL
===============
Each finger is modeled as a 3-revolute-joint planar serial chain (MCP,
PIP, DIP) - the same simplification standard in prosthetic/robotic hand
kinematics literature (out-of-plane abduction is neglected). In
Denavit-Hartenberg terms: alpha_i = 0 (coplanar links), a_i = link
length L_i, d_i = 0, theta_i = joint angle, with the MCP as the base
frame.

TRANSMISSION MODEL
===================
This hand is underactuated: ONE servo pulls ONE tendon that routes past
BOTH the PIP and DIP joints, so a single actuator drives a coupled pair
of joints rather than each independently - the same "N actuators for
N+k joints" underactuation used by real tendon-driven hands (e.g. the
Yale OpenHand / SDM Hand family). The servo-to-joint relationship is a
capstan (pulley) relationship:

    delta_s           = r_horn * delta_theta_servo   (tendon length pulled)
    delta_theta_joint = delta_s / r_joint             (resulting joint rotation)
    => delta_theta_joint = (r_horn / r_joint) * delta_theta_servo

(r_horn / r_joint) is the TRANSMISSION RATIO. The physical pulley radii
aren't easily measured on an assembled hand, so - same as commissioning
a real tendon-driven hand - it's calibrated empirically instead: run the
servo through its full range, measure the resulting joint range, and
divide.

RETARGETING PIPELINE
=====================
Observe (vision FK on the operator's hand) -> Normalize (map to a 0..1
bend fraction between calibrated extremes) -> Actuate (push that
fraction through the transmission model to a servo command). See
retarget() below - hand_robot.py's live loop calls this directly.

PALM-LOCAL REFERENCE FRAME
===========================
Thumb opposition (relative to the palm) and wrist rotation (the palm's
own orientation relative to the forearm/world) are anatomically distinct
motions, but a naive implementation measures both as angles anchored at
the wrist in raw camera-space, so a pure wrist twist - which rotates the
whole hand rigidly, thumb included - bleeds into the thumb reading too.
local_frame() builds a coordinate frame from the palm itself (forward
axis toward the fingers, normal axis out of the palm); to_local_frame()
then expresses a point (e.g. the thumb tip) inside that frame, which
factors the hand's own orientation out first. roll_around_axis()
isolates true axial wrist rotation the same way, against a fixed
external reference - see the rotation-invariance self-test below.
"""

import math

# ------------------------------------------------------------------
# Link lengths (phalanx proportions)
#
# Nominal average adult hand segment lengths in mm (proximal, middle,
# distal phalanx). The live system has no depth/biometric calibration
# (the camera gives relative landmark positions, not absolute mm), so
# these are standard anthropometric proportions for FK demonstration,
# not device-specific measurements - swap in measured values for a
# specific hand for a precise FK model.
# ------------------------------------------------------------------

PHALANX_LENGTHS_MM = {
    "index": (39.8, 22.4, 15.8),
    "middle": (44.6, 26.3, 17.3),
    "ring": (41.4, 25.6, 17.4),
    "little": (32.7, 18.0, 15.8),
}


def forward_kinematics_finger(link_lengths, joint_angles_deg):
    """
    Planar 3-revolute-joint forward kinematics for one finger.

    link_lengths: (L_proximal, L_middle, L_distal), mm.
    joint_angles_deg: (theta_mcp, theta_pip, theta_dip), degrees of
        flexion from the straight (0 deg) pose.

    Returns [MCP, PIP, DIP, TIP] as (x, y) mm positions in the finger's
    own base frame (MCP at the origin, straight finger along +x).
    """
    x, y = 0.0, 0.0
    heading = 0.0  # cumulative angle from +x axis
    points = [(x, y)]
    for length, theta_deg in zip(link_lengths, joint_angles_deg):
        heading += math.radians(theta_deg)
        x += length * math.cos(heading)
        y -= length * math.sin(heading)  # flexion curls "downward"
        points.append((x, y))
    return points


# ------------------------------------------------------------------
# Palm-local reference frame
#
# The thumb's opposition motion and the wrist's rotation are physically
# DIFFERENT things (anatomically: thumb opposition is relative to the
# palm; wrist rotation is the palm's own orientation relative to the
# forearm/world), but both were originally measured as angles anchored
# at the wrist in raw camera-space - so a pure wrist twist (which
# rotates the whole hand rigidly, thumb included) bled into the thumb
# opposition reading too, since that reading never factored out the
# hand's own orientation.
#
# The fix, same technique real hand-tracking SDKs use: build a local
# coordinate frame FROM the palm itself, then express other points (like
# the thumb tip) INSIDE that frame. A point's local-frame coordinates
# don't change when the frame rotates WITH the hand - e.g. if the thumb
# stays still relative to the palm while the wrist twists, the thumb's
# local coordinates stay constant even though its raw camera-space
# coordinates change completely. That's what decouples the two signals.
# ------------------------------------------------------------------

def _vec(a, b):
    return (b[0] - a[0], b[1] - a[1], b[2] - a[2])


def _normalize3(v):
    mag = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if mag == 0:
        return (0.0, 0.0, 0.0)
    return (v[0] / mag, v[1] / mag, v[2] / mag)


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def local_frame(origin, forward_point, plane_point_a, plane_point_b):
    """
    Build an orthonormal local coordinate frame anchored at `origin`.
    x_axis points toward forward_point (e.g. the palm's "forward"
    direction, wrist -> middle-finger base). z_axis is the normal of
    the plane spanned by (origin, plane_point_a, plane_point_b) - the
    palm's normal, roughly. y_axis completes a right-handed frame.
    Each point is an (x, y, z) tuple.
    """
    x_axis = _normalize3(_vec(origin, forward_point))
    normal = _cross(_vec(origin, plane_point_a), _vec(origin, plane_point_b))
    z_axis = _normalize3(normal)
    y_axis = _cross(z_axis, x_axis)
    return x_axis, y_axis, z_axis


def to_local_frame(point, origin, frame):
    """Express `point` in the local coordinate frame built by local_frame()."""
    x_axis, y_axis, z_axis = frame
    v = _vec(origin, point)
    return (_dot3(v, x_axis), _dot3(v, y_axis), _dot3(v, z_axis))


def in_plane_angle_deg(local_point):
    """Angle (degrees) of a local-frame point's (x, y) projection - its
    sweep angle within the reference plane, independent of whatever
    rotation the frame itself has undergone in camera-space."""
    x, y, _z = local_point
    return math.degrees(math.atan2(y, x))


def roll_around_axis(frame, reference):
    """
    Roll angle (degrees) of the frame's y_axis around its own x_axis,
    relative to `reference` (a fixed world direction not parallel to
    x_axis, e.g. "up"). This isolates rotation AROUND the frame's
    forward axis specifically - e.g. true wrist pronation/supination
    (twisting the forearm, which rotates the palm around its own
    forward direction) - from other motions that also move x_axis
    around in camera-space (moving your whole arm, bending your wrist
    sideways), which a simple "how tilted does the hand look in the 2D
    image" angle can't tell apart from genuine axial twisting.
    """
    x_axis, y_axis, _z_axis = frame
    ref_dot_x = _dot3(reference, x_axis)
    ref_perp = (
        reference[0] - ref_dot_x * x_axis[0],
        reference[1] - ref_dot_x * x_axis[1],
        reference[2] - ref_dot_x * x_axis[2],
    )
    ref_perp = _normalize3(ref_perp)
    if ref_perp == (0.0, 0.0, 0.0):
        return 0.0
    cos_roll = _dot3(ref_perp, y_axis)
    sin_roll = _dot3(_cross(ref_perp, y_axis), x_axis)
    return math.degrees(math.atan2(sin_roll, cos_roll))


# ------------------------------------------------------------------
# Tendon transmission model
# ------------------------------------------------------------------

class TendonTransmission:
    """Capstan relationship between servo rotation and the joint
    rotation it drives through a single tendon."""

    def __init__(self, servo_range_deg, joint_range_deg):
        self.servo_range_deg = servo_range_deg
        self.joint_range_deg = joint_range_deg
        self.transmission_ratio = (
            joint_range_deg / servo_range_deg if servo_range_deg else 0.0
        )

    def servo_to_joint(self, servo_deflection_deg):
        return servo_deflection_deg * self.transmission_ratio

    def joint_to_servo(self, joint_deflection_deg):
        if self.transmission_ratio == 0:
            return 0.0
        return joint_deflection_deg / self.transmission_ratio


# ------------------------------------------------------------------
# Human -> robot motion retargeting pipeline
#
# Three stages, each independently testable:
#   1. OBSERVE   - human_joint_angle_deg, measured via vision FK
#                  (see joint_angle() in hand_robot.py).
#   2. NORMALIZE - normalize_bend() maps the observed angle onto a
#                  0..1 bend fraction between calibrated extremes.
#   3. ACTUATE   - actuate() pushes that fraction through the
#                  transmission model to a servo command.
# retarget() composes all three for the common case; code that also
# needs the intermediate bend fraction (e.g. hand_robot.py's grip-lock,
# which looks at the AVERAGE bend across fingers) calls
# normalize_bend() and actuate() separately instead.
# ------------------------------------------------------------------

def normalize_bend(human_joint_angle_deg, straight_angle_deg, bent_angle_deg, gain=1.0):
    """Stage 2: observed angle -> 0..1 bend fraction. gain compensates
    for an operator whose real range of motion doesn't reach the full
    calibrated extreme on camera."""
    span = straight_angle_deg - bent_angle_deg
    bend_fraction = (straight_angle_deg - human_joint_angle_deg) / span * gain
    return max(0.0, min(1.0, bend_fraction))


def normalize_bend_piecewise(observed_angle_deg, control_points, gain=1.0):
    """Stage 2 variant using MULTIPLE calibration points instead of just
    two extremes - e.g. measured at several named poses (precision-pinch
    against each finger in turn), not just fully-open/fully-opposed.
    More accurate than normalize_bend() when the real angle-to-bend
    relationship isn't uniform across the range: for a thumb, opposition
    sweeps a wide initial arc to first reach the index finger, then
    smaller, roughly even increments to reach each finger after that -
    a straight line between just the two extremes misplaces those
    intermediate finger-touch positions (confirmed by direct
    measurement, not assumed).

    control_points: list of (angle_deg, bend_fraction) pairs - any
    order, any direction (angle increasing or decreasing as bend
    increases, whichever matches how the real measurement moves).
    bend_fraction runs 0.0 (straight/open) to 1.0 (fully bent/opposed).
    Interpolates linearly between the two points bracketing the
    observed angle; clamps to the nearest endpoint's bend outside the
    calibrated range.
    """
    points = sorted(control_points, key=lambda p: p[0])
    angle = observed_angle_deg

    if angle <= points[0][0]:
        bend = points[0][1]
    elif angle >= points[-1][0]:
        bend = points[-1][1]
    else:
        bend = points[-1][1]  # overwritten by the matching segment below
        for (a0, b0), (a1, b1) in zip(points, points[1:]):
            if a0 <= angle <= a1:
                t = (angle - a0) / (a1 - a0) if a1 != a0 else 0.0
                bend = b0 + t * (b1 - b0)
                break

    bend *= gain
    return max(0.0, min(1.0, bend))


def normalize_rotation(observed_angle_deg, neutral_angle_deg, full_range_deg, gain=1.0, wrap=True):
    """Stage 2 variant for BIDIRECTIONAL joints (e.g. base/wrist rotation,
    which turns left or right from a center position, unlike a finger
    which only bends one way from straight). Returns a signed fraction
    in -1..1: 0 at neutral, +-1 at the full calibrated rotation either
    way.

    wrap=True (default) takes the SHORTEST path around the atan2
    +-180 wraparound - correct when neutral sits near the discontinuity
    and a real rotation could plausibly cross it either way. wrap=False
    uses the raw signed difference instead - correct when one direction
    genuinely passes smoothly through the discontinuity as part of its
    normal range (confirmed by testing: assuming "shortest path" there
    flips that direction's sign to match the other one, which is wrong)."""
    delta = observed_angle_deg - neutral_angle_deg
    if wrap:
        delta = ((delta + 180) % 360) - 180
    normalized = delta / full_range_deg * gain
    return max(-1.0, min(1.0, normalized))


def actuate(bend_fraction, transmission, servo_neutral_deg, direction=1):
    """Stage 3: 0..1 bend fraction -> target servo angle, through the
    calibrated tendon transmission."""
    joint_deflection_deg = bend_fraction * transmission.joint_range_deg
    servo_deflection_deg = transmission.joint_to_servo(joint_deflection_deg)
    return servo_neutral_deg + direction * servo_deflection_deg


def retarget(human_joint_angle_deg, straight_angle_deg, bent_angle_deg,
             transmission, servo_neutral_deg, direction=1, gain=1.0):
    """All three stages composed: observed human angle -> target servo
    angle in degrees."""
    bend_fraction = normalize_bend(human_joint_angle_deg, straight_angle_deg, bent_angle_deg, gain)
    return actuate(bend_fraction, transmission, servo_neutral_deg, direction)


# ------------------------------------------------------------------
# Full kinematic chain (current hardware + planned DOF)
#
# Installed: 6 DOF - base rotation + 4 underactuated finger chains (one
# tendon each, coupling PIP+DIP) + thumb CMC opposition.
#
# Planned: 1 more DOF completing the chain:
#   - thumb MCP flexion: makes the thumb a true 2-DOF digit (CMC
#     opposition + MCP flexion), matching a real thumb.
# ------------------------------------------------------------------

KINEMATIC_CHAIN = [
    {"joint": "base_rotation", "type": "revolute", "installed": True,
     "role": "proximal joint - mounted at the arm's base, mimics wrist "
             "pronation/supination (bidirectional, not a one-way bend "
             "like the fingers)"},
    {"joint": "thumb_cmc_opposition", "type": "revolute", "installed": True,
     "role": "sweeps thumb across the palm"},
    {"joint": "thumb_mcp_flexion", "type": "revolute", "installed": False,
     "role": "curls the thumb tip - second thumb DOF"},
    {"joint": "index_mcp_pip_dip", "type": "underactuated (1 tendon, 2 joints)", "installed": True,
     "role": "coupled PIP+DIP flexion"},
    {"joint": "middle_mcp_pip_dip", "type": "underactuated (1 tendon, 2 joints)", "installed": True,
     "role": "coupled PIP+DIP flexion"},
    {"joint": "ring_mcp_pip_dip", "type": "underactuated (1 tendon, 2 joints)", "installed": True,
     "role": "coupled PIP+DIP flexion"},
    {"joint": "little_mcp_pip_dip", "type": "underactuated (1 tendon, 2 joints)", "installed": True,
     "role": "coupled PIP+DIP flexion"},
]


def print_chain_table():
    print(f"{'Joint':<24}{'Type':<38}{'Installed':<10}Role")
    for j in KINEMATIC_CHAIN:
        print(f"{j['joint']:<24}{j['type']:<38}{str(j['installed']):<10}{j['role']}")


# ------------------------------------------------------------------
# Self-test / standalone demo - no camera, no hardware required.
# ------------------------------------------------------------------

if __name__ == "__main__":
    print("=== Kinematic chain ===")
    print_chain_table()

    print("\n=== Forward kinematics self-test (index finger) ===")
    poses = {
        "straight": (0, 0, 0),
        "half-bent": (30, 30, 15),
        "full fist": (90, 90, 60),
    }
    for label, angles in poses.items():
        points = forward_kinematics_finger(PHALANX_LENGTHS_MM["index"], angles)
        tip = points[-1]
        print(f"{label:<10} joint angles {angles} -> fingertip = ({tip[0]:6.1f}, {tip[1]:6.1f}) mm")

    print("\n=== Transmission model self-test (index finger) ===")
    transmission = TendonTransmission(servo_range_deg=130, joint_range_deg=90)
    print(f"transmission ratio (joint deg / servo deg): {transmission.transmission_ratio:.3f}")
    for servo_deflection in (0, 65, 130):
        joint_deflection = transmission.servo_to_joint(servo_deflection)
        print(f"servo +{servo_deflection:3d} deg -> joint +{joint_deflection:5.1f} deg")

    print("\n=== Retargeting pipeline self-test (index finger) ===")
    for human_angle in (165, 120, 60):
        servo_angle = retarget(
            human_angle, straight_angle_deg=165, bent_angle_deg=60,
            transmission=transmission, servo_neutral_deg=0, direction=1,
        )
        print(f"human joint angle {human_angle:3d} deg -> servo command {servo_angle:6.1f} deg")

    print("\n=== Palm-local frame self-test: rotation invariance ===")
    # A synthetic hand: wrist at origin, palm roughly in the XY plane,
    # thumb tip off to one side. Then the SAME hand, rigidly rotated
    # (as if the wrist twisted) - the thumb's LOCAL angle should be
    # identical in both cases, proving wrist rotation doesn't leak into
    # the thumb reading.
    wrist = (0.0, 0.0, 0.0)
    middle_mcp = (0.0, -1.0, 0.0)
    index_mcp = (-0.3, -0.9, 0.0)
    pinky_mcp = (0.3, -0.9, 0.0)
    thumb_tip = (-0.4, -0.3, 0.1)

    def rotate_z(p, degrees):
        a = math.radians(degrees)
        return (
            p[0] * math.cos(a) - p[1] * math.sin(a),
            p[0] * math.sin(a) + p[1] * math.cos(a),
            p[2],
        )

    for twist_deg in (0, 30, 90, 180):
        w = rotate_z(wrist, twist_deg)
        m = rotate_z(middle_mcp, twist_deg)
        i = rotate_z(index_mcp, twist_deg)
        p = rotate_z(pinky_mcp, twist_deg)
        t = rotate_z(thumb_tip, twist_deg)

        frame = local_frame(w, m, i, p)
        local_thumb = to_local_frame(t, w, frame)
        thumb_angle = in_plane_angle_deg(local_thumb)
        print(f"wrist twisted {twist_deg:3d} deg -> thumb's local-frame angle = {thumb_angle:6.1f} deg (should stay constant)")
