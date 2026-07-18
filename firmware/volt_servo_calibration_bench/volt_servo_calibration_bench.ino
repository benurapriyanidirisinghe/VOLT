/*
  VOLT servo calibration bench.

  Upload this sketch while measuring each servo's safe mechanical range.

  Hardware:
  - Arduino Nano connected to PCA9685 over I2C.
  - Potentiometer outside legs to 5V and GND, wiper to A0.
  - Servo signal on the selected PCA9685 channel.
  - Servo power must come from a separate high-current supply.
  - Arduino/PCA9685/servo supply grounds must be common.

  Serial monitor:
  - 115200 baud
  - Newline ending

  Commands:
    CH n       Select PCA9685 channel 0..11.
    MIN        Store the current pot angle as this servo's minimum.
    CENTER     Store the current pot angle as this servo's center.
    MAX        Store the current pot angle as this servo's maximum.
    PRINT      Print arrays ready to paste into volt_arduino_pca9685.ino.
    HELP       Print command help.

  Workflow:
  1. Send CH 0.
  2. Turn the pot slowly until the servo is at its safe minimum. Send MIN.
  3. Turn to the neutral/center pose. Send CENTER.
  4. Turn to the safe maximum. Send MAX.
  5. Repeat CH 1 through CH 11.
  6. Send PRINT and copy the arrays into the main firmware.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t SERVO_COUNT = 12;
const uint8_t POT_PIN = A0;
const uint32_t BAUD_RATE = 115200;
const uint16_t SERVO_FREQ_HZ = 50;
const uint16_t SERVO_PERIOD_US = 1000000UL / SERVO_FREQ_HZ;

// Keep these the same as the main firmware while calibrating angle limits.
const uint16_t SERVO_MIN_US = 600;
const uint16_t SERVO_MAX_US = 2400;

// Limits for the bench sweep itself. Start conservative if the linkage is not
// installed yet. The values you store with MIN/CENTER/MAX come from this scale.
const float BENCH_MIN_DEG = 0.0;
const float BENCH_MAX_DEG = 180.0;

const uint16_t UPDATE_PERIOD_MS = 20;
const uint16_t PRINT_PERIOD_MS = 500;
const float MAX_DEG_PER_SECOND = 180.0;

uint8_t selectedChannel = 0;
float currentDeg = 90.0;
float targetDeg = 90.0;
uint32_t lastUpdateMs = 0;
uint32_t lastPrintMs = 0;

float minDeg[SERVO_COUNT];
float centerDeg[SERVO_COUNT];
float maxDeg[SERVO_COUNT];
bool hasMin[SERVO_COUNT];
bool hasCenter[SERVO_COUNT];
bool hasMax[SERVO_COUNT];

char lineBuffer[64];
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

uint16_t pulseUsToTicks(uint16_t pulseUs) {
  return (uint32_t)pulseUs * 4096UL / SERVO_PERIOD_US;
}

uint16_t angleToPulseUs(float angleDeg) {
  angleDeg = clampFloat(angleDeg, BENCH_MIN_DEG, BENCH_MAX_DEG);
  float span = (float)(SERVO_MAX_US - SERVO_MIN_US);
  return SERVO_MIN_US + (uint16_t)(((angleDeg - BENCH_MIN_DEG) / (BENCH_MAX_DEG - BENCH_MIN_DEG)) * span + 0.5);
}

float readPotAngleDeg() {
  uint16_t raw = analogRead(POT_PIN);
  return BENCH_MIN_DEG + ((float)raw / 1023.0) * (BENCH_MAX_DEG - BENCH_MIN_DEG);
}

void writeSelectedServo(float angleDeg) {
  uint16_t pulseUs = angleToPulseUs(angleDeg);
  pwm.setPWM(selectedChannel, 0, pulseUsToTicks(pulseUs));
}

void initializeStoredValues() {
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    minDeg[i] = 0.0;
    centerDeg[i] = 90.0;
    maxDeg[i] = 180.0;
    hasMin[i] = false;
    hasCenter[i] = false;
    hasMax[i] = false;
  }
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  CH n    - select PCA9685 channel 0..11"));
  Serial.println(F("  MIN     - store current angle as minimum"));
  Serial.println(F("  CENTER  - store current angle as center"));
  Serial.println(F("  MAX     - store current angle as maximum"));
  Serial.println(F("  PRINT   - print arrays for main firmware"));
}

void printOneValue(float value) {
  Serial.print(value, 1);
}

void printFloatArray(const __FlashStringHelper *name, float values[SERVO_COUNT]) {
  Serial.print(F("const float "));
  Serial.print(name);
  Serial.println(F("[SERVO_COUNT] = {"));
  for (uint8_t row = 0; row < 4; ++row) {
    Serial.print(F("  "));
    for (uint8_t col = 0; col < 3; ++col) {
      uint8_t index = row * 3 + col;
      printOneValue(values[index]);
      if (index < SERVO_COUNT - 1) {
        Serial.print(F(", "));
      }
    }
    Serial.println();
  }
  Serial.println(F("};"));
}

void printArrays() {
  Serial.println();
  printFloatArray(F("SERVO_CENTER_DEG"), centerDeg);
  printFloatArray(F("SERVO_MIN_DEG"), minDeg);
  printFloatArray(F("SERVO_MAX_DEG"), maxDeg);
  Serial.println();
}

void printStatus() {
  uint16_t pulseUs = angleToPulseUs(currentDeg);
  Serial.print(F("CH "));
  Serial.print(selectedChannel);
  Serial.print(F(" angle "));
  Serial.print(currentDeg, 1);
  Serial.print(F(" deg pulse "));
  Serial.print(pulseUs);
  Serial.print(F(" us"));

  Serial.print(F(" | saved min/center/max: "));
  if (hasMin[selectedChannel]) {
    Serial.print(minDeg[selectedChannel], 1);
  } else {
    Serial.print(F("--"));
  }
  Serial.print(F(" / "));
  if (hasCenter[selectedChannel]) {
    Serial.print(centerDeg[selectedChannel], 1);
  } else {
    Serial.print(F("--"));
  }
  Serial.print(F(" / "));
  if (hasMax[selectedChannel]) {
    Serial.print(maxDeg[selectedChannel], 1);
  } else {
    Serial.print(F("--"));
  }
  Serial.println();
}

void selectChannel(uint8_t channel) {
  if (channel >= SERVO_COUNT) {
    Serial.println(F("ERR channel must be 0..11"));
    return;
  }

  pwm.setPWM(selectedChannel, 0, 0);
  selectedChannel = channel;
  currentDeg = readPotAngleDeg();
  targetDeg = currentDeg;
  writeSelectedServo(currentDeg);

  Serial.print(F("OK selected CH "));
  Serial.println(selectedChannel);
}

void storeCurrentAs(const char *label) {
  float angle = currentDeg;

  if (strcmp(label, "MIN") == 0) {
    minDeg[selectedChannel] = angle;
    hasMin[selectedChannel] = true;
  } else if (strcmp(label, "CENTER") == 0) {
    centerDeg[selectedChannel] = angle;
    hasCenter[selectedChannel] = true;
  } else if (strcmp(label, "MAX") == 0) {
    maxDeg[selectedChannel] = angle;
    hasMax[selectedChannel] = true;
  }

  Serial.print(F("OK CH "));
  Serial.print(selectedChannel);
  Serial.print(F(" "));
  Serial.print(label);
  Serial.print(F(" = "));
  Serial.print(angle, 1);
  Serial.println(F(" deg"));
}

void parseCommand(char *line) {
  char *cursor = NULL;
  char *command = strtok_r(line, " ,\t", &cursor);
  if (command == NULL) {
    return;
  }

  if (strcmp(command, "CH") == 0) {
    char *token = strtok_r(NULL, " ,\t", &cursor);
    if (token == NULL) {
      Serial.println(F("ERR missing channel"));
      return;
    }
    selectChannel((uint8_t)atoi(token));
    return;
  }

  if (strcmp(command, "MIN") == 0 || strcmp(command, "CENTER") == 0 || strcmp(command, "MAX") == 0) {
    storeCurrentAs(command);
    return;
  }

  if (strcmp(command, "PRINT") == 0) {
    printArrays();
    return;
  }

  if (strcmp(command, "HELP") == 0) {
    printHelp();
    return;
  }

  Serial.println(F("ERR unknown command"));
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
      Serial.println(F("ERR line too long"));
    }
  }
}

void updateServoFromPot() {
  uint32_t now = millis();
  if (now - lastUpdateMs < UPDATE_PERIOD_MS) {
    return;
  }

  float dt = (now - lastUpdateMs) * 0.001;
  lastUpdateMs = now;

  targetDeg = readPotAngleDeg();
  float maxStep = MAX_DEG_PER_SECOND * dt;
  float error = targetDeg - currentDeg;
  if (error > maxStep) {
    error = maxStep;
  } else if (error < -maxStep) {
    error = -maxStep;
  }

  currentDeg += error;
  writeSelectedServo(currentDeg);
}

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);

  initializeStoredValues();
  currentDeg = readPotAngleDeg();
  targetDeg = currentDeg;
  writeSelectedServo(currentDeg);
  lastUpdateMs = millis();
  lastPrintMs = millis();

  Serial.println(F("OK VOLT_SERVO_CALIBRATION_BENCH_READY"));
  printHelp();
}

void loop() {
  readSerialLines();
  updateServoFromPot();

  uint32_t now = millis();
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    printStatus();
  }
}
