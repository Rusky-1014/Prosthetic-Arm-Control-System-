# Robotic Hand — Camera Control

Your webcam watches your hand, a Python script on your Mac measures each
finger's (and thumb's) real joint angle plus your wrist's rotation,
retargets each through a calibrated tendon-transmission model, and sends
6 numbers over USB to an Arduino Nano, which drives 6 servos through a
PCA9685 to move the robotic hand and its base rotation.

```
Webcam -> MediaPipe (joint angles + wrist roll) -> kinematics.py (retargeting pipeline)
   -> servo angles -> USB serial -> Arduino Nano -> PCA9685 -> 6 servos -> hand
```

See `kinematics.py`'s module docstring (or just run `venv/bin/python
kinematics.py`) for the actual kinematic model behind this: link lengths,
forward kinematics, the tendon-transmission/underactuation model, and the
observe → normalize → actuate retargeting pipeline that `hand_robot.py`
runs on every camera frame.

## Files

- `kinematics.py` — the formal kinematic model. Standalone: no camera, no
  Arduino, no servos needed. Run it directly for a self-test/demo.
- `hand_robot.py` — the live controller: webcam + MediaPipe + serial,
  calling into `kinematics.py` for every joint.
- `arduino/hand_control/hand_control.ino` — upload this to the Nano. Reads
  `"s1,s2,s3,s4,s5,s6\n"` over serial and drives the 6 servo channels.
- `serial_test.py` — sends a couple of hard-coded positions to confirm the
  USB link and wiring work, before touching the camera.
- `demo_grasps.py` — standalone scripted demo cycling through named human
  grasp types (open hand, power grip, pinch, point). No camera needed.
- `requirements.txt` — pinned Python package versions.
- `venv/` — a ready-to-use Python virtual environment with everything
  already installed.

## One-time setup (already done for you)

The newest `mediapipe` on PyPI (1.0.x) dropped the simple hand-tracking API
this project uses. `venv/` is pinned to `mediapipe==0.10.21` (the classic,
reliable API) in a Python 3.12 environment — you don't need to reinstall
anything, just use `venv/bin/python` to run the scripts.

To rebuild it from scratch if ever needed:

```bash
cd ~/robotic_arm
python3.12 -m venv venv      # must be Python 3.11 or 3.12, NOT 3.13/3.14
venv/bin/pip install -r requirements.txt
```

## Step-by-step: how to actually use this

### 1. Upload the Arduino sketch

Open `arduino/hand_control/hand_control.ino` in the Arduino IDE.

- Install the library: Tools → Manage Libraries → search **"Adafruit PWM
  Servo Driver Library"** → Install.
- Select your board (Arduino Nano) and port.
- Click Upload.
- Open the Serial Monitor at **115200 baud** — you should see `READY`
  printed once, and the hand should move to its neutral position.

Leave the Serial Monitor **closed** afterwards — only one program can use
the serial port at a time, and the Python scripts need it.

### 2. Find your Nano's port

```bash
ls /dev/cu.*
```

Look for something like `/dev/cu.usbserial-10`. All the Python scripts
auto-detect this if there's only one match — set it manually (the `PORT`
/ `SERIAL_PORT` variable at the top of each file) only if you have
multiple USB serial devices plugged in.

### 3. Test the serial link (no camera yet)

```bash
cd ~/robotic_arm
venv/bin/python serial_test.py
```

The hand should: go to neutral → move slightly → return to neutral.

### 4. Test the camera tracking only (no servo movement)

In `hand_robot.py`, set `ENABLE_SERVO = False`, then:

```bash
venv/bin/python hand_robot.py
```

A window opens with your webcam feed, hand landmarks drawn on it, and
`S1`–`S6` + `ThumbAngle` + `WristAngle` numbers that change as you move
your hand. Press **ESC** to quit.

### 5. Run it for real

Set `ENABLE_SERVO = True` again, then run `hand_robot.py` the same way.
Bend your fingers/thumb → the hand follows proportionally. Make a full
fist → grip-lock engages (shows "GRIPPING (locked)" on screen) and holds a
firm, full closure even if the object blocks the camera's view of your
hand. Press **ESC** to stop.

### 6. Run the scripted grasp demo (no camera)

```bash
venv/bin/python demo_grasps.py
```

Cycles through: open hand → power grip → pinch → point → open hand.

## Current calibration

Every range below was found by direct incremental testing (stepping up in
~10-20° increments, watching/listening for strain at each step), not
assumed.

| Servo | Neutral | Direction | Max swing | Notes |
| --- | --- | --- | --- | --- |
| 1 — index | 0° | increasing | 150° | Strain observed above this |
| 2 — middle | 0° | increasing | 170° | 180° (the servo's absolute limit) did nothing - stalled |
| 3 — ring | 0° | increasing | 170° | Physically replaced servo, string re-tied at this 0° reference point, on **channel 8** (channels 2, 4, 6 all had bad board connections — confirmed with multiple servos) |
| 4 — little | 0° | increasing | 170° | Tested together with middle/ring, clean |
| 5 — thumb | 135° | **decreasing** | 90° (135°→45°) | 135° = hand fully open/stretched, 45° = thumb brought fully against the other fingers. CMC opposition only — thumb MCP flexion is a planned 7th servo |
| 6 — base rotation | 90° | increasing | 90° (0°→180°, full servo range) | **Channel 7** (channels 9, 12, 15 all turned out bad - see below). Bidirectional (turns both ways from center to mimic wrist twist), unlike the fingers. Eased in 2° steps in firmware since jumping straight to a new angle strains it under the arm's mechanical load. Confirmed safe across the servo's entire 0°-180° range by direct testing |

These live in `NEUTRAL`/`DIRECTION`/`FINGER_MAX_MOVEMENT`/`THUMB_MAX_MOVEMENT`/
`WRIST_*` at the top of `hand_robot.py`, the `TendonTransmission` objects
just below them, and `NEUTRAL[]`/`CHANNEL[]` in the Arduino sketch — keep
both files in sync if you ever recalibrate.

### Calibrating the base rotation servo's camera tracking (next step)

The servo's own range is confirmed (0°-180°, full swing). What's still a
guess is the camera-side mapping - how far you twist your real wrist vs.
how far the servo should turn. Same method used for the thumb:

1. Run `hand_robot.py`, center your wrist (no twist), and read the
   `WristAngle` number on screen.
2. Set `WRIST_NEUTRAL_ANGLE` in `hand_robot.py` to that number.
3. Twist your wrist fully one way, then the other, noting both
   `WristAngle` readings - that spread becomes `WRIST_ROTATION_RANGE_DEG`.

### Thumb and wrist no longer interfere with each other

Originally, both the thumb-opposition reading and the wrist-rotation
reading were angles measured in raw camera-space, anchored at the wrist -
so twisting your wrist (which rigidly rotates your whole hand, thumb
included) bled into the thumb reading too, and the wrist reading itself
was really measuring "how tilted does the hand look in the 2D image"
rather than true axial twist (so it also picked up moving your whole arm
or bending your wrist sideways).

Fixed with a palm-local reference frame (`local_frame()` in
`kinematics.py`, the same technique real hand-tracking SDKs use): thumb
opposition is now measured as the thumb tip's angle *inside* the palm's
own coordinate frame (`thumb_local_angle()`), which doesn't change when
the frame rotates with your wrist - proven by a rotation-invariance
self-test in `kinematics.py` (run `venv/bin/python kinematics.py`, see
"Palm-local frame self-test"). Wrist rotation is now measured as true
roll around the palm's own forward axis (`roll_around_axis()`), isolating
axial twist from other arm motion.

Because the thumb's metric changed units entirely, `THUMB_OPEN_ANGLE` /
`THUMB_OPPOSED_ANGLE` needed recalibrating against it - done, see below.

### Handedness: a real chirality bug, now fixed

Computing the palm's normal vector (needed for both the thumb and wrist
math above) uses a cross product, which is direction-sensitive: a right
hand facing the camera is the mirror image of a left hand facing the
camera (or of a right hand facing away). Using the same formula
regardless inverted the result for exactly one of those cases - found by
testing (worked for the left hand and for the right hand facing away,
broken for the right hand facing the camera).

Fixed by reading MediaPipe's own handedness detection
(`results.multi_handedness`) and correcting the palm-frame construction
accordingly (`build_palm_axes()` in `hand_robot.py`) - confirmed by
testing that identical hand geometry with the opposite handedness label
produces a precisely mirrored result. The on-screen `Hand: Left/Right`
readout shows what was detected.

### Thumb calibration (per-hand)

The two hands read consistently different ThumbAngle values for the same
physical pose - a real, repeatable offset, not noise - so each hand gets
its own calibration, selected live from the detected handedness.

**Right hand (primary target): precise 5-point curve.** Instead of just
open/opposed, this is calibrated at 5 named poses - open, and touching
each fingertip in turn - since the spacing between them isn't even:
there's a big sweep from open to first reaching the index finger, then
smaller, roughly even steps per finger after that. A straight line
between just the two extremes would misplace the index/middle/ring pinch
positions - which matters for anything held in a precision pinch (a pen,
etc.), not just a full open/close grip:

| Pose | ThumbAngle | Bend fraction | Servo angle |
| --- | --- | --- | --- |
| Fully open | 30° | 0.00 | 135° |
| Touching index tip | 11° | 0.25 | 112.5° |
| Touching middle tip | 3° | 0.50 | 90° |
| Touching ring tip | -3° | 0.75 | 67.5° |
| Touching little tip | -10° | 1.00 | 45° |

`THUMB_PINCH_POINTS_RIGHT` in `hand_robot.py` holds these;
`normalize_bend_piecewise()` in `kinematics.py` interpolates between
whichever two points bracket the live reading (clamped at the ends).

**Left hand: still the simpler 2-point curve** (only open/opposed
measured so far) - swapped relative to the raw ThumbAngle readings
(confirmed by testing: the thumb's local angle runs the opposite way for
the left hand's own geometry):

| Hand | Fully open | Fully opposed |
| --- | --- | --- |
| Left | -30° | 10° |

`thumb_base_bend()` dispatches between the two methods based on the same
handedness label as the chirality fix above.

## Tuning knobs (top of `hand_robot.py`)

| Variable | What it does |
| --- | --- |
| `FINGER_MAX_MOVEMENT` | Max degrees of travel from neutral, per finger `[index, middle, ring, little]` |
| `THUMB_MAX_MOVEMENT` | Max degrees of travel from thumb neutral |
| `WRIST_NEUTRAL_ANGLE` | The raw `WristAngle` reading at a centered/untwisted wrist - calibrated, 95° |
| `WRIST_ROTATION_RANGE_DEG` | Real wrist-twist degrees (either direction) that map to full servo deflection - calibrated, 85° |
| `WRIST_MAX_MOVEMENT` | Max servo degrees of travel from base-rotation neutral - 90° (full 0°-180° range), confirmed safe |
| `FINGER_STRAIGHT_ANGLE` / `FINGER_BENT_ANGLE` | Real joint-angle calibration for finger curl detection |
| `THUMB_PINCH_POINTS_RIGHT` | Right hand's 5-point thumb calibration curve (see Thumb calibration above) |
| `THUMB_OPEN_ANGLE_LEFT` / `THUMB_OPPOSED_ANGLE_LEFT` / `THUMB_BEND_GAIN` | Left hand's simpler 2-point thumb calibration |
| `GRIP_ENGAGE_THRESHOLD` / `GRIP_RELEASE_THRESHOLD` | When grip-lock engages/releases |
| `SMOOTHING` | 0–1. Higher = snappier but jittery, lower = smoother/slower |
| `SEND_INTERVAL` | Minimum seconds between serial messages |

## Known hardware history

**Ring finger**: the original servo showed slow movement + clicking (gear
strain), and channels 2, 4, and 6 on the PCA9685 all turned out to have
bad connections (confirmed by swapping servos between channels). That
servo has since been **physically replaced** and the string re-tied at
the new servo's 0° neutral, on **channel 8**. Re-tested from scratch,
confirmed clean across the full 0°-170° range.

**PCA9685 board**: channels 2, 4, 6, 9, 12, and 15 have all shown bad
connections at this point (confirmed with multiple different servos on
each, so it's the board/wiring, not any one servo). Treat any new channel
on this board as unverified until tested - channels confirmed good so
far: 0, 1, 3, 5, 7, 8.

**Base rotation firmware fix**: instant position jumps on the base
rotation servo looked like strain/failure (same symptom as a bad
channel), but turned out to be two separate firmware issues, not
hardware: (1) the main sketch was calling `setServo()` on every loop
iteration regardless of whether anything changed, flooding the PCA9685
with redundant I2C writes - fixed by only writing when a value actually
changes; (2) sudden large jumps demand a current spike that servo can't
sustain under the arm's mechanical load - fixed by easing it 2° at a time
instead of jumping straight to target. Both fixes are in
`hand_control.ino`'s `loop()`.

## Planned: thumb MCP flexion

One more servo left on the roadmap: **thumb MCP flexion** — gives the
thumb a real second DOF (opposition + flexion), not just opposition.
Already modeled in `kinematics.py`'s `KINEMATIC_CHAIN` (marked
`installed: False`), so wiring it in later follows the same pattern as
every joint already installed: see that file's docstring.

## What's not implemented yet

- **Per-hand open/closed calibration**: bend thresholds (`FINGER_STRAIGHT_ANGLE`
  etc.) are fixed constants, not calibrated per-session. Works well enough
  in practice, but a "show OPEN hand, show CLOSED hand" startup calibration
  step would be more robust to different users/camera setups.
- **Measured transmission ratios**: `TendonTransmission` objects currently
  use ratio 1.0 (servo-side range only) since the joint-side range hasn't
  been measured separately (e.g. with a protractor against the physical
  finger at each extreme). Doing that would make the kinematic model
  physically calibrated, not just a working placeholder.
