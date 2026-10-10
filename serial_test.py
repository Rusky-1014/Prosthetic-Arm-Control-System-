"""
Quick sanity check: laptop -> USB -> Arduino Nano -> PCA9685 -> hand.

Run this BEFORE trying the camera script. It just sends a couple of
hard-coded positions so you can confirm the wiring/firmware works.
"""

import glob
import sys
import time

import serial

# CHANGE THIS to your Nano's port, or leave as "" to auto-detect.
PORT = ""


def find_port():
    candidates = glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.wchusbserial*")
    if not candidates:
        print("No serial port found. Plug in the Nano and run `ls /dev/cu.*` to check.")
        sys.exit(1)
    if len(candidates) > 1:
        print("Multiple ports found, pick one and set PORT manually:")
        for c in candidates:
            print(" -", c)
        sys.exit(1)
    return candidates[0]


def main():
    port = PORT or find_port()
    print(f"Connecting to {port} ...")

    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(2)  # let the Nano reset after the port opens

    print("Connected. Sending neutral position...")
    ser.write(b"0,0,0,0,135,90\n")
    time.sleep(1)

    print("Sending test position...")
    ser.write(b"30,30,30,30,105,90\n")
    time.sleep(1)

    print("Returning to neutral...")
    ser.write(b"0,0,0,0,135,90\n")
    time.sleep(1)

    ser.close()
    print("Done. If the hand moved and returned, the serial link is good.")


if __name__ == "__main__":
    main()
