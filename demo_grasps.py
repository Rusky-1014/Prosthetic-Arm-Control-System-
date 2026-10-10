"""
Scripted demo: cycles the hand through named human grasp types.

Standalone - talks directly to the Arduino over serial, no camera needed.
This is the functional validation referenced in the kinematic writeup:
proof the model reproduces specific grasp types, not just "closes."

Servo order: index, middle, ring, little, thumb, base rotation.
Values below are derived from the current tuned constants in
hand_robot.py (NEUTRAL, FINGER_MAX_MOVEMENT, THUMB_MAX_MOVEMENT,
DIRECTION, THUMB_DIRECTION) - keep them in sync if those change. Base
rotation stays at neutral (90) in every grasp below - add rotated
variants once that servo's range is calibrated.
"""

import glob
import sys
import time

import serial

SERIAL_PORT = ""  # leave "" to auto-detect
BAUD_RATE = 115200
HOLD_SECONDS = 2.5

# name -> (index, middle, ring, little, thumb, base rotation)
GRASPS = {
    "Open hand / rest": (0, 0, 0, 0, 135, 90),
    "Power grip (full fist)": (150, 170, 170, 170, 45, 90),
    "Pinch (thumb to index)": (120, 0, 0, 0, 45, 90),
    "Point (index extended)": (0, 170, 170, 170, 45, 90),
}

SEQUENCE = [
    "Open hand / rest",
    "Power grip (full fist)",
    "Open hand / rest",
    "Pinch (thumb to index)",
    "Open hand / rest",
    "Point (index extended)",
    "Open hand / rest",
]


def find_port():
    candidates = glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.wchusbserial*")
    if not candidates:
        print("No serial port found. Is the Nano plugged in?")
        sys.exit(1)
    if len(candidates) > 1:
        print("Multiple ports found, set SERIAL_PORT manually:")
        for c in candidates:
            print(" -", c)
        sys.exit(1)
    return candidates[0]


def send(ser, angles):
    message = ",".join(str(a) for a in angles) + "\n"
    ser.write(message.encode())


def main():
    port = SERIAL_PORT or find_port()
    print(f"Connecting to {port} ...")
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    time.sleep(2)
    ser.read_all()
    print("Connected.\n")

    for name in SEQUENCE:
        angles = GRASPS[name]
        print(f"-> {name}  {angles}")
        send(ser, angles)
        time.sleep(HOLD_SECONDS)

    ser.close()
    print("\nDemo sequence complete.")


if __name__ == "__main__":
    main()
