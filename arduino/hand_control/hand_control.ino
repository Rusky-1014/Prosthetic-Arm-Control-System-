#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const int SERVOMIN = 150;
const int SERVOMAX = 600;

// Your calibrated neutral positions (boot-time pose only - hand_robot.py
// sends its own NEUTRAL once connected, keep these two in sync)
const int NEUTRAL[6] = {
    0,   // Servo 1 (index)
    0,   // Servo 2 (middle)
    0,   // Servo 3 (ring) - re-tied on the new servo at this position
    0,   // Servo 4 (little)
    135, // Servo 5 (thumb, SG90) - hand fully open/stretched position
    90   // Servo 6 (base/wrist rotation) - centered, recalibrate once tested
};

// Physical PCA9685 channel for each servo. Channels 2, 4, 6, 9, 12, and
// 15 have all shown bad connections on this board (confirmed by testing
// multiple different servos on each - it's the board, not the servos).
// Thumb on channel 5, ring on channel 8, base rotation now on channel 7.
const int CHANNEL[6] = {0, 1, 8, 3, 5, 7};

// Base rotation (index 5) carries the whole arm's mechanical load, so
// jumping straight to a new angle demands a current spike it can't
// sustain - confirmed directly: a simple 1-degree-at-a-time sweep moved
// it cleanly, instant jumps didn't. So only that servo eases toward its
// target gradually; the fingers/thumb (already tuned) stay instant.
float current5 = 90;
int target[6];
int lastSent[6] = {-1, -1, -1, -1, -1, -1};  // -1 = never written yet
unsigned long lastStepMs = 0;
const int STEP_DEGREES = 2;
const int STEP_INTERVAL_MS = 15;

String inputBuffer = "";

int angleToPulse(int angle) { return map(angle, 0, 180, SERVOMIN, SERVOMAX); }

void setServo(int channel, int angle) {
  angle = constrain(angle, 0, 180);
  pwm.setPWM(channel, 0, angleToPulse(angle));
}

void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setPWMFreq(50);

  delay(500);

  // Start at calibrated neutral
  for (int i = 0; i < 6; i++) {
    target[i] = NEUTRAL[i];
    setServo(CHANNEL[i], NEUTRAL[i]);
    lastSent[i] = NEUTRAL[i];
  }
  current5 = NEUTRAL[5];

  Serial.println("READY");
}

void handleLine(String data) {
  data.trim();
  int s1, s2, s3, s4, s5, s6;
  if (sscanf(data.c_str(), "%d,%d,%d,%d,%d,%d", &s1, &s2, &s3, &s4, &s5, &s6) == 6) {
    target[0] = s1;
    target[1] = s2;
    target[2] = s3;
    target[3] = s4;
    target[4] = s5;
    target[5] = s6;
    Serial.println("OK");
  }
}

void loop() {
  // Non-blocking serial read - buffer characters, parse on newline, so
  // this never stalls the base-rotation stepping below.
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleLine(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // Fingers + thumb: instant, but only write to the PCA9685 when the
  // value actually changed - no redundant I2C traffic.
  for (int i = 0; i < 5; i++) {
    if (target[i] != lastSent[i]) {
      setServo(CHANNEL[i], target[i]);
      lastSent[i] = target[i];
    }
  }

  // Base rotation: ease toward target a couple degrees at a time, same
  // change-only write rule.
  unsigned long now = millis();
  if (now - lastStepMs >= STEP_INTERVAL_MS) {
    lastStepMs = now;
    float diff = target[5] - current5;
    if (abs(diff) < 0.5) {
      current5 = target[5];
    } else if (diff > 0) {
      current5 += min((float)STEP_DEGREES, diff);
    } else {
      current5 -= min((float)STEP_DEGREES, -diff);
    }
    int angle5 = (int)round(current5);
    if (angle5 != lastSent[5]) {
      setServo(CHANNEL[5], angle5);
      lastSent[5] = angle5;
    }
  }
}
