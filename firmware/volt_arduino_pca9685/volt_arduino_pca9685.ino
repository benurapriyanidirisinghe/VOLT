/*
  VOLT Arduino Nano PCA9685 physical-output firmware.

  Runtime protocol, 115200 baud, newline terminated:
    FRAME d0 d1 d2 d3 d4 d5 d6 d7 d8 d9 d10 d11
      Absolute physical servo angles in degrees, ordered by PCA9685 channel.

    SERVO channel degrees
      Move only one PCA9685 channel target.

    PING, ARM, HOLD, DISARM, DISABLE, STATUS

  Safety:
  - Startup leaves PCA9685 outputs disabled and does not move servos.
  - DISARM and timeout hold the last target; they do not command center.
  - Host ROS serial bridge owns neutral, trim, direction, and joint conversion.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <stdlib.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t CHANNEL_COUNT = 12;
const uint32_t BAUD_RATE = 115200;
const uint16_t SERVO_FREQ_HZ = 50;
const uint16_t SERVO_PERIOD_US = 1000000UL / SERVO_FREQ_HZ;

const uint16_t CHANNEL_MIN_US[CHANNEL_COUNT] = {
  600, 600, 600,
  600, 600, 600,
  600, 600, 600,
  600, 600, 600
};
const uint16_t CHANNEL_MAX_US[CHANNEL_COUNT] = {
  2400, 2400, 2400,
  2400, 2400, 2400,
  2400, 2400, 2400,
  2400, 2400, 2400
};

const float CHANNEL_MIN_DEG[CHANNEL_COUNT] = {
  70.0, 0.0, 0.0,
  70.0, 0.0, 30.0,
  50.0, 0.0, 30.0,
  50.0, 0.0, 0.0
};
const float CHANNEL_MAX_DEG[CHANNEL_COUNT] = {
  160.0, 180.0, 150.0,
  160.0, 180.0, 180.0,
  140.0, 180.0, 180.0,
  140.0, 180.0, 150.0
};

const float MAX_DEG_PER_SECOND = 30.0;
const uint16_t UPDATE_PERIOD_MS = 10;
const uint32_t COMMAND_TIMEOUT_MS = 750;
const bool ACK_FRAME_COMMANDS = false;

float targetDeg[CHANNEL_COUNT];
float currentDeg[CHANNEL_COUNT];
bool targetValid[CHANNEL_COUNT];
bool outputEnabled = false;
bool servoArmed = false;
bool timeoutWarned = false;
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

bool isFiniteFloat(float value) {
  return !isnan(value) && !isinf(value);
}

uint16_t angleToPulseUs(uint8_t channel, float angleDeg) {
  angleDeg = clampFloat(angleDeg, CHANNEL_MIN_DEG[channel], CHANNEL_MAX_DEG[channel]);
  float span = (float)(CHANNEL_MAX_US[channel] - CHANNEL_MIN_US[channel]);
  float pulse = CHANNEL_MIN_US[channel] + (angleDeg / 180.0f) * span;
  pulse = clampFloat(pulse, CHANNEL_MIN_US[channel], CHANNEL_MAX_US[channel]);
  return (uint16_t)(pulse + 0.5f);
}

uint16_t pulseUsToTicks(uint16_t pulseUs) {
  uint32_t ticks = (uint32_t)pulseUs * 4096UL / SERVO_PERIOD_US;
  if (ticks > 4095UL) {
    ticks = 4095UL;
  }
  return (uint16_t)ticks;
}

void writeChannel(uint8_t channel, float angleDeg) {
  if (!outputEnabled || channel >= CHANNEL_COUNT || !targetValid[channel]) {
    return;
  }
  uint16_t pulseUs = angleToPulseUs(channel, angleDeg);
  pwm.setPWM(channel, 0, pulseUsToTicks(pulseUs));
}

void disableOutputs() {
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    pwm.setPWM(channel, 0, 0);
  }
  outputEnabled = false;
}

void updateServos() {
  if (!outputEnabled) {
    return;
  }

  uint32_t now = millis();
  if (now - lastUpdateMs < UPDATE_PERIOD_MS) {
    return;
  }

  float dt = (now - lastUpdateMs) * 0.001f;
  lastUpdateMs = now;
  float maxStep = MAX_DEG_PER_SECOND * dt;

  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    if (!targetValid[channel]) {
      continue;
    }
    float error = targetDeg[channel] - currentDeg[channel];
    if (error > maxStep) {
      error = maxStep;
    } else if (error < -maxStep) {
      error = -maxStep;
    }
    currentDeg[channel] += error;
    writeChannel(channel, currentDeg[channel]);
  }
}

char *nextToken(char **cursor) {
  return strtok_r(NULL, " ,\t", cursor);
}

bool parseFloatStrict(char *token, float *value) {
  if (token == NULL || token[0] == '\0') {
    return false;
  }
  char *end = NULL;
  double parsed = strtod(token, &end);
  if (end == token || *end != '\0') {
    return false;
  }
  if (isnan(parsed) || isinf(parsed) || parsed < -1000.0 || parsed > 1000.0) {
    return false;
  }
  *value = (float)parsed;
  return isFiniteFloat(*value);
}

bool parseChannelStrict(char *token, uint8_t *channel) {
  float value = 0.0;
  if (!parseFloatStrict(token, &value)) {
    return false;
  }
  int integer = (int)value;
  if ((float)integer != value || integer < 0 || integer >= CHANNEL_COUNT) {
    return false;
  }
  *channel = (uint8_t)integer;
  return true;
}

bool ensureNoExtraTokens(char **cursor) {
  return nextToken(cursor) == NULL;
}

void setTargetChannel(uint8_t channel, float degrees) {
  degrees = clampFloat(degrees, CHANNEL_MIN_DEG[channel], CHANNEL_MAX_DEG[channel]);
  targetDeg[channel] = degrees;
  if (!targetValid[channel]) {
    currentDeg[channel] = degrees;
    targetValid[channel] = true;
  }
}

void printStatus() {
  Serial.print(F("OK STATUS ARMED="));
  Serial.print(servoArmed ? 1 : 0);
  Serial.print(F(" OUTPUT="));
  Serial.print(outputEnabled ? 1 : 0);
  Serial.print(F(" LAST_CMD_MS="));
  Serial.println(millis() - lastCommandMs);
}

bool handleFrame(char **cursor) {
  if (!servoArmed) {
    Serial.println(F("ERR NOT_ARMED"));
    return false;
  }

  float values[CHANNEL_COUNT];
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    char *token = nextToken(cursor);
    if (token == NULL) {
      Serial.println(F("ERR BAD_COUNT"));
      return false;
    }
    if (!parseFloatStrict(token, &values[channel])) {
      Serial.println(F("ERR BAD_VALUE"));
      return false;
    }
  }

  if (!ensureNoExtraTokens(cursor)) {
    Serial.println(F("ERR BAD_COUNT"));
    return false;
  }

  outputEnabled = true;
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    setTargetChannel(channel, values[channel]);
  }
  lastCommandMs = millis();
  timeoutWarned = false;
  if (ACK_FRAME_COMMANDS) {
    Serial.println(F("OK FRAME"));
  }
  return true;
}

bool handleServo(char **cursor) {
  if (!servoArmed) {
    Serial.println(F("ERR NOT_ARMED"));
    return false;
  }

  uint8_t channel = 0;
  float degrees = 0.0;
  if (!parseChannelStrict(nextToken(cursor), &channel)) {
    Serial.println(F("ERR BAD_CHANNEL"));
    return false;
  }
  if (!parseFloatStrict(nextToken(cursor), &degrees)) {
    Serial.println(F("ERR BAD_VALUE"));
    return false;
  }
  if (!ensureNoExtraTokens(cursor)) {
    Serial.println(F("ERR BAD_COUNT"));
    return false;
  }

  outputEnabled = true;
  setTargetChannel(channel, degrees);
  lastCommandMs = millis();
  timeoutWarned = false;
  Serial.print(F("OK SERVO "));
  Serial.print(channel);
  Serial.print(F(" "));
  Serial.println(targetDeg[channel], 2);
  return true;
}

bool parseCommand(char *line) {
  char *cursor = NULL;
  char *command = strtok_r(line, " ,\t", &cursor);
  if (command == NULL) {
    return false;
  }

  if (strcmp(command, "PING") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Serial.println(F("ERR BAD_COUNT"));
      return false;
    }
    Serial.println(F("OK PONG"));
    return true;
  }

  if (strcmp(command, "ARM") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Serial.println(F("ERR BAD_COUNT"));
      return false;
    }
    servoArmed = true;
    Serial.println(F("OK ARM"));
    return true;
  }

  if (strcmp(command, "HOLD") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Serial.println(F("ERR BAD_COUNT"));
      return false;
    }
    Serial.println(F("OK HOLD"));
    return true;
  }

  if (strcmp(command, "DISARM") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Serial.println(F("ERR BAD_COUNT"));
      return false;
    }
    servoArmed = false;
    Serial.println(F("OK DISARM"));
    return true;
  }

  if (strcmp(command, "DISABLE") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Serial.println(F("ERR BAD_COUNT"));
      return false;
    }
    servoArmed = false;
    disableOutputs();
    Serial.println(F("OK DISABLE"));
    return true;
  }

  if (strcmp(command, "STATUS") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Serial.println(F("ERR BAD_COUNT"));
      return false;
    }
    printStatus();
    return true;
  }

  if (strcmp(command, "FRAME") == 0) {
    return handleFrame(&cursor);
  }

  if (strcmp(command, "SERVO") == 0) {
    return handleServo(&cursor);
  }

  Serial.println(F("ERR UNKNOWN_COMMAND"));
  return false;
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

  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    targetDeg[channel] = 90.0;
    currentDeg[channel] = 90.0;
    targetValid[channel] = false;
  }

  disableOutputs();
  servoArmed = false;
  lastCommandMs = millis();
  lastUpdateMs = millis();

  Serial.println(F("OK VOLT_PCA9685_READY DISARMED OUTPUT_DISABLED"));
}

void loop() {
  readSerialLines();

  if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS && !timeoutWarned) {
    timeoutWarned = true;
    Serial.println(F("WARN COMMAND_TIMEOUT HOLDING"));
  }

  updateServos();
}
