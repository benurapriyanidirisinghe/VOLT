/*
  VOLT Arduino Nano servo firmware.

  Hardware:
  - Arduino Nano connected to Jetson over USB serial.
  - PCA9685 servo driver on I2C address 0x40.
  - 12 servos connected to PCA9685 channels 0..11.
  - Servo power must come from a separate high-current supply.
  - Jetson/Arduino/PCA9685/servo supply grounds must be common.

  Serial protocol, 115200 baud, newline terminated:
    RAD r0 r1 r2 r3 r4 r5 r6 r7 r8 r9 r10 r11
      Joint commands in radians, same order as volt_kinematics.JOINT_NAMES.
      0 rad maps to the servo's power-on zero position: 90 degrees.

    DEG d0 d1 d2 d3 d4 d5 d6 d7 d8 d9 d10 d11
      Absolute servo angles in degrees. Mostly useful for calibration.

    HOME
      Move every servo to 90 degrees.

    PING
      Replies with OK PONG.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t SERVO_COUNT = 12;
const uint32_t BAUD_RATE = 115200;
const uint16_t SERVO_FREQ_HZ = 50;
const uint16_t SERVO_PERIOD_US = 1000000UL / SERVO_FREQ_HZ;

// Tune these for your exact servos. Many servos are close to 500..2500 us,
// but 600..2400 us is a safer first power-up range.
const uint16_t SERVO_MIN_US[SERVO_COUNT] = {
  600, 600, 600,
  600, 600, 600,
  600, 600, 600,
  600, 600, 600
};
const uint16_t SERVO_MAX_US[SERVO_COUNT] = {
  2400, 2400, 2400,
  2400, 2400, 2400,
  2400, 2400, 2400,
  2400, 2400, 2400
};

// Match src/volt_description/scripts/volt_kinematics.py JOINT_NAMES:
// 0 front_left_shoulder, 1 front_left_leg, 2 front_left_foot,
// 3 front_right_shoulder, 4 front_right_leg, 5 front_right_foot,
// 6 rear_left_shoulder, 7 rear_left_leg, 8 rear_left_foot,
// 9 rear_right_shoulder, 10 rear_right_leg, 11 rear_right_foot.
const uint8_t SERVO_CHANNEL[SERVO_COUNT] = {
  0, 1, 2,
  3, 4, 5,
  6, 7, 8,
  9, 10, 11
};

// Change signs after calibration if a joint moves opposite the simulation.
const int8_t SERVO_DIRECTION[SERVO_COUNT] = {
  1, 1, 1,
  1, 1, 1,
  1, 1, 1,
  1, 1, 1
};

// Mechanical trim around the 90 degree neutral. Use this to make all legs
// physically match the simulated zero pose without changing the ROS model.
float servoTrimDeg[SERVO_COUNT] = {
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0
};

const float SERVO_NEUTRAL_DEG = 90.0;
const float SERVO_MIN_DEG = 0.0;
const float SERVO_MAX_DEG = 180.0;
const float RAD_TO_DEG = 57.2957795;

// Firmware-side smoothing protects servos if serial packets jump.
const float MAX_DEG_PER_SECOND = 360.0;
const uint16_t UPDATE_PERIOD_MS = 10;
const uint32_t COMMAND_TIMEOUT_MS = 600;

float targetDeg[SERVO_COUNT];
float currentDeg[SERVO_COUNT];
uint32_t lastCommandMs = 0;
uint32_t lastUpdateMs = 0;

char lineBuffer[192];
uint8_t lineLength = 0;

float clampFloat(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

uint16_t angleToPulseUs(uint8_t index, float angleDeg) {
  angleDeg = clampFloat(angleDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  float span = (float)(SERVO_MAX_US[index] - SERVO_MIN_US[index]);
  return SERVO_MIN_US[index] + (uint16_t)((angleDeg / 180.0) * span + 0.5);
}

uint16_t pulseUsToTicks(uint16_t pulseUs) {
  return (uint32_t)pulseUs * 4096UL / SERVO_PERIOD_US;
}

void writeServo(uint8_t index, float angleDeg) {
  uint16_t pulseUs = angleToPulseUs(index, angleDeg);
  pwm.setPWM(SERVO_CHANNEL[index], 0, pulseUsToTicks(pulseUs));
}

void setHomeTargets() {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    targetDeg[i] = SERVO_NEUTRAL_DEG + servoTrimDeg[i];
  }
}

void moveImmediatelyHome() {
  setHomeTargets();
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    currentDeg[i] = targetDeg[i];
    writeServo(i, currentDeg[i]);
  }
}

void updateServos() {
  uint32_t now = millis();
  if (now - lastUpdateMs < UPDATE_PERIOD_MS) {
    return;
  }

  float dt = (now - lastUpdateMs) * 0.001;
  lastUpdateMs = now;
  float maxStep = MAX_DEG_PER_SECOND * dt;

  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    float error = targetDeg[i] - currentDeg[i];
    if (error > maxStep) {
      error = maxStep;
    } else if (error < -maxStep) {
      error = -maxStep;
    }
    currentDeg[i] += error;
    writeServo(i, currentDeg[i]);
  }
}

bool parseFloatToken(char **cursor, float *value) {
  char *token = strtok_r(NULL, " ,\t", cursor);
  if (token == NULL) {
    return false;
  }
  *value = atof(token);
  return true;
}

bool parseCommand(char *line) {
  char *cursor = NULL;
  char *command = strtok_r(line, " ,\t", &cursor);
  if (command == NULL) {
    return false;
  }

  if (strcmp(command, "PING") == 0) {
    Serial.println(F("OK PONG"));
    return true;
  }

  if (strcmp(command, "HOME") == 0) {
    setHomeTargets();
    lastCommandMs = millis();
    Serial.println(F("OK HOME"));
    return true;
  }

  bool radiansMode = strcmp(command, "RAD") == 0;
  bool degreesMode = strcmp(command, "DEG") == 0;
  if (!radiansMode && !degreesMode) {
    Serial.println(F("ERR UNKNOWN_COMMAND"));
    return false;
  }

  float nextTargets[SERVO_COUNT];
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    float value = 0.0;
    if (!parseFloatToken(&cursor, &value)) {
      Serial.println(F("ERR MISSING_VALUE"));
      return false;
    }

    if (radiansMode) {
      // ROS joint zero maps to the servo's neutral 90 degree power-on pose.
      value = SERVO_NEUTRAL_DEG
        + servoTrimDeg[i]
        + SERVO_DIRECTION[i] * value * RAD_TO_DEG;
    } else {
      value = value + servoTrimDeg[i];
    }
    nextTargets[i] = clampFloat(value, SERVO_MIN_DEG, SERVO_MAX_DEG);
  }

  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    targetDeg[i] = nextTargets[i];
  }
  lastCommandMs = millis();
  Serial.println(F("OK"));
  return true;
}

void readSerialLines() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      lineBuffer[lineLength] = '\0';
      if (lineLength > 0) {
        parseCommand(lineBuffer);
      }
      lineLength = 0;
      continue;
    }

    if (lineLength < sizeof(lineBuffer) - 1) {
      lineBuffer[lineLength++] = c;
    } else {
      lineLength = 0;
      Serial.println(F("ERR LINE_TOO_LONG"));
    }
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);

  moveImmediatelyHome();
  lastCommandMs = millis();
  lastUpdateMs = millis();

  Serial.println(F("OK VOLT_PCA9685_READY"));
}

void loop() {
  readSerialLines();

  // If the Jetson stops sending commands, return gently to neutral. This keeps
  // power-on and communication-loss behavior predictable while you tune.
  if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    setHomeTargets();
  }

  updateServos();
}
