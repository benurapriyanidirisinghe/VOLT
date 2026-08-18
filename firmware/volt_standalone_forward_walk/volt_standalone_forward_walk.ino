/*
  VOLT standalone forward walk for Arduino Nano + PCA9685.

  This sketch does not use ROS and does not implement the VOLT protocol.
  It replays calibrated physical servo angles in PCA9685 channel order.

  Serial Monitor commands at 115200 baud:
    ARM       Enable the calibrated standing pose.
    RUN       Walk one finite forward cycle, then return to standing.
    RUN 1..3  Walk the requested finite number of cycles.
    STOP      Finish the current all-feet-down cycle, then return to standing.
    HOLD      Stop immediately and retain the current servo pulses.
    DISARM    Same immediate holding behavior as HOLD.
    DISABLE   Remove all servo pulses. The robot may collapse.
    STATUS
    HELP

  Safety behavior:
  - Startup emits no PWM and cannot move a servo.
  - ARM is explicit and RUN is rejected for two seconds after ARM.
  - RUN is finite; there is no autonomous endless loop.
  - Embedded frames are checked before ARM can succeed.
  - Direct-angle interpolation is capped at 18 deg/s.
  - STOP is preferred to HOLD because STOP completes the current crawl cycle.

  The angle tables were generated from the repository's calibrated
  spotmicro_video_walk/VOLT WALK trajectory at 0.004 m/s, hardware timing,
  open-loop support assumption, IK, and servo_calibration.yaml. Values are
  physical servo degrees multiplied by ten; directions and trims are already
  applied and must not be applied a second time.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <avr/pgmspace.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t CHANNEL_COUNT = 12;
const uint8_t FRAME_COUNT = 29;
const uint8_t MAX_RUN_CYCLES = 3;
const uint32_t BAUD_RATE = 115200;
const uint16_t SERVO_FREQ_HZ = 50;
const uint16_t SERVO_PERIOD_US = 1000000UL / SERVO_FREQ_HZ;
const uint16_t UPDATE_PERIOD_MS = 20;
const uint16_t ARM_SETTLE_MS = 2000;
const uint16_t MIN_RETURN_MS = 2000;
const float TABLE_MAX_DPS = 18.0;
const float RETURN_MAX_DPS = 10.0;

const uint16_t CHANNEL_MIN_DEG_X10[CHANNEL_COUNT] PROGMEM = {
  700, 0, 0, 700, 0, 300, 500, 0, 300, 500, 0, 0
};
const uint16_t CHANNEL_MAX_DEG_X10[CHANNEL_COUNT] PROGMEM = {
  1600, 1800, 1500, 1600, 1800, 1800,
  1400, 1800, 1800, 1400, 1800, 1500
};
const uint16_t SAFE_STAND_X10[CHANNEL_COUNT] PROGMEM = {
  1229, 886, 622, 1171, 914, 1109,
  1070, 999, 1272, 902, 801, 619
};

// Startup cycle: calibrated standing pose to the periodic crawl boundary.
const uint16_t STARTUP_FRAMES[FRAME_COUNT][CHANNEL_COUNT] PROGMEM = {
  {1229, 886, 622, 1171, 914, 1109, 1070, 999, 1272, 902, 801, 619},
  {1282, 1027, 761, 1224, 831, 1072, 1124, 1126, 1171, 955, 741, 617},
  {1282, 1028, 761, 1224, 830, 1072, 1124, 1127, 1171, 955, 740, 616},
  {1282, 1029, 757, 1224, 830, 1075, 1129, 1126, 1176, 955, 681, 712},
  {1282, 1028, 752, 1224, 831, 1081, 1137, 1124, 1183, 955, 620, 833},
  {1282, 1034, 757, 1224, 825, 1076, 1129, 1129, 1179, 955, 707, 727},
  {1282, 1038, 760, 1224, 821, 1072, 1124, 1133, 1178, 955, 764, 637},
  {1282, 1040, 760, 1224, 819, 1073, 1124, 1134, 1179, 955, 763, 636},
  {1282, 949, 751, 1224, 905, 1082, 1124, 1073, 1140, 955, 831, 657},
  {1282, 951, 751, 1224, 904, 1082, 1124, 1074, 1140, 955, 830, 657},
  {1287, 952, 748, 1224, 860, 991, 1124, 1074, 1145, 955, 830, 653},
  {1295, 952, 744, 1224, 833, 889, 1123, 1073, 1150, 955, 831, 648},
  {1287, 958, 750, 1224, 922, 1016, 1124, 1078, 1146, 955, 825, 653},
  {1282, 962, 754, 1224, 974, 1110, 1124, 1082, 1143, 955, 821, 656},
  {1282, 965, 754, 1224, 971, 1108, 1124, 1084, 1144, 955, 819, 656},
  {1176, 1002, 654, 1118, 804, 971, 1017, 1077, 1304, 849, 681, 728},
  {1176, 1003, 654, 1118, 803, 971, 1017, 1078, 1305, 849, 681, 727},
  {1176, 1003, 650, 1118, 802, 974, 1017, 1138, 1202, 844, 681, 722},
  {1176, 1002, 644, 1118, 802, 979, 1018, 1194, 1069, 836, 683, 716},
  {1176, 1007, 649, 1118, 797, 974, 1017, 1096, 1167, 844, 678, 719},
  {1176, 1011, 652, 1118, 793, 970, 1017, 1036, 1254, 849, 674, 721},
  {1176, 1013, 651, 1118, 790, 970, 1017, 1037, 1255, 849, 673, 720},
  {1176, 936, 657, 1118, 887, 992, 1017, 969, 1235, 849, 737, 754},
  {1176, 937, 657, 1118, 886, 991, 1017, 970, 1235, 849, 736, 754},
  {1176, 979, 749, 1113, 885, 993, 1017, 970, 1239, 849, 736, 750},
  {1176, 991, 849, 1105, 884, 997, 1018, 969, 1244, 849, 736, 745},
  {1176, 882, 717, 1113, 879, 991, 1017, 975, 1239, 849, 731, 749},
  {1176, 826, 621, 1118, 874, 987, 1017, 979, 1236, 849, 727, 752},
  {1176, 829, 622, 1118, 871, 986, 1017, 981, 1236, 849, 725, 751}
};
const uint16_t STARTUP_DURATIONS_MS[FRAME_COUNT - 1] PROGMEM = {
  800, 100, 540, 680, 600, 500, 195,
  520, 100, 520, 580, 720, 540, 195,
  940, 100, 580, 740, 560, 500, 195,
  540, 100, 520, 560, 740, 540, 195
};

// Periodic forward crawl. First and last frames are exactly equal.
const uint16_t LOOP_FRAMES[FRAME_COUNT][CHANNEL_COUNT] PROGMEM = {
  {1176, 829, 622, 1118, 871, 986, 1017, 981, 1236, 849, 725, 751},
  {1282, 996, 759, 1224, 824, 1072, 1124, 1119, 1164, 955, 728, 597},
  {1282, 997, 760, 1224, 823, 1072, 1124, 1119, 1165, 955, 727, 597},
  {1282, 998, 756, 1224, 823, 1076, 1129, 1119, 1170, 955, 667, 697},
  {1282, 998, 752, 1224, 824, 1081, 1137, 1117, 1176, 955, 610, 826},
  {1282, 1003, 757, 1224, 819, 1076, 1129, 1122, 1172, 955, 705, 725},
  {1282, 1007, 760, 1224, 814, 1073, 1124, 1126, 1170, 955, 764, 637},
  {1282, 1010, 761, 1224, 812, 1073, 1124, 1127, 1172, 955, 763, 636},
  {1282, 913, 739, 1224, 897, 1080, 1124, 1063, 1138, 955, 831, 657},
  {1282, 914, 740, 1224, 895, 1080, 1124, 1064, 1138, 955, 830, 657},
  {1287, 915, 737, 1224, 852, 988, 1124, 1064, 1142, 955, 830, 653},
  {1295, 916, 734, 1224, 828, 888, 1123, 1064, 1147, 955, 831, 648},
  {1287, 921, 740, 1224, 921, 1015, 1124, 1069, 1143, 955, 825, 653},
  {1282, 926, 744, 1224, 974, 1110, 1124, 1073, 1140, 955, 821, 656},
  {1282, 929, 745, 1224, 971, 1108, 1124, 1075, 1141, 955, 819, 656},
  {1176, 976, 659, 1118, 804, 971, 1017, 1072, 1294, 849, 681, 728},
  {1176, 977, 659, 1118, 803, 971, 1017, 1073, 1295, 849, 681, 727},
  {1176, 977, 655, 1118, 802, 974, 1017, 1133, 1195, 844, 681, 722},
  {1176, 976, 650, 1118, 802, 979, 1018, 1190, 1065, 836, 683, 716},
  {1176, 981, 654, 1118, 797, 974, 1017, 1095, 1166, 844, 678, 719},
  {1176, 986, 658, 1118, 793, 970, 1017, 1036, 1254, 849, 674, 721},
  {1176, 988, 657, 1118, 790, 970, 1017, 1037, 1255, 849, 673, 720},
  {1176, 903, 651, 1118, 887, 992, 1017, 969, 1235, 849, 737, 754},
  {1176, 905, 651, 1118, 886, 991, 1017, 970, 1235, 849, 736, 754},
  {1176, 948, 742, 1113, 885, 993, 1017, 970, 1239, 849, 736, 750},
  {1176, 972, 843, 1105, 884, 997, 1018, 969, 1244, 849, 736, 745},
  {1176, 879, 715, 1113, 879, 991, 1017, 975, 1239, 849, 731, 749},
  {1176, 826, 621, 1118, 874, 987, 1017, 979, 1236, 849, 727, 752},
  {1176, 829, 622, 1118, 871, 986, 1017, 981, 1236, 849, 725, 751}
};
const uint16_t LOOP_DURATIONS_MS[FRAME_COUNT - 1] PROGMEM = {
  940, 100, 560, 720, 580, 500, 195,
  540, 100, 520, 560, 720, 540, 195,
  940, 100, 560, 740, 580, 500, 195,
  540, 100, 520, 580, 720, 540, 195
};

enum MotionState {
  OUTPUT_DISABLED,
  STANDING,
  PLAYING_STARTUP,
  PLAYING_LOOP,
  RETURNING_TO_STAND,
  HOLDING
};

MotionState motionState = OUTPUT_DISABLED;
bool calibrationValid = false;
bool outputEnabled = false;
bool armed = false;
bool stopRequested = false;
uint8_t cyclesRemaining = 0;
uint8_t activeFrameIndex = 0;
uint32_t segmentStartMs = 0;
uint32_t segmentDurationMs = 0;
uint32_t lastUpdateMs = 0;
uint32_t runAllowedAfterMs = 0;

const uint16_t (*activeFrames)[CHANNEL_COUNT] = NULL;
const uint16_t *activeDurations = NULL;

float currentDeg[CHANNEL_COUNT];
float segmentStartDeg[CHANNEL_COUNT];
float segmentTargetDeg[CHANNEL_COUNT];

char lineBuffer[48];
uint8_t lineLength = 0;
bool discardUntilNewline = false;

uint16_t readWord(const uint16_t *address) {
  return pgm_read_word(address);
}

float clampFloat(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

void loadSafeStand(float output[CHANNEL_COUNT]) {
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    output[channel] = readWord(&SAFE_STAND_X10[channel]) * 0.1f;
  }
}

void loadFrame(
  const uint16_t frames[][CHANNEL_COUNT],
  uint8_t frameIndex,
  float output[CHANNEL_COUNT]
) {
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    output[channel] = readWord(&frames[frameIndex][channel]) * 0.1f;
  }
}

uint16_t angleToTicks(uint8_t channel, float degrees) {
  float low = readWord(&CHANNEL_MIN_DEG_X10[channel]) * 0.1f;
  float high = readWord(&CHANNEL_MAX_DEG_X10[channel]) * 0.1f;
  degrees = clampFloat(degrees, low, high);
  float pulseUs = 600.0f + (degrees / 180.0f) * 1800.0f;
  uint32_t ticks = (uint32_t)(pulseUs * 4096.0f / SERVO_PERIOD_US + 0.5f);
  if (ticks > 4095UL) {
    ticks = 4095UL;
  }
  return (uint16_t)ticks;
}

void writeCurrentFrame() {
  if (!outputEnabled) {
    return;
  }
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    pwm.setPWM(channel, 0, angleToTicks(channel, currentDeg[channel]));
  }
}

void disableOutputs() {
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    pwm.setPWM(channel, 0, 0);
  }
  outputEnabled = false;
  armed = false;
  stopRequested = false;
  cyclesRemaining = 0;
  motionState = OUTPUT_DISABLED;
}

bool validateFrame(
  const uint16_t frames[][CHANNEL_COUNT],
  uint8_t frameIndex
) {
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    uint16_t value = readWord(&frames[frameIndex][channel]);
    uint16_t low = readWord(&CHANNEL_MIN_DEG_X10[channel]);
    uint16_t high = readWord(&CHANNEL_MAX_DEG_X10[channel]);
    if (value < low || value > high) {
      return false;
    }
  }
  return true;
}

bool framesEqual(
  const uint16_t left[][CHANNEL_COUNT],
  uint8_t leftIndex,
  const uint16_t right[][CHANNEL_COUNT],
  uint8_t rightIndex
) {
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    if (
      readWord(&left[leftIndex][channel])
      != readWord(&right[rightIndex][channel])
    ) {
      return false;
    }
  }
  return true;
}

bool validateTable(
  const uint16_t frames[][CHANNEL_COUNT],
  const uint16_t durations[FRAME_COUNT - 1]
) {
  for (uint8_t frame = 0; frame < FRAME_COUNT; ++frame) {
    if (!validateFrame(frames, frame)) {
      return false;
    }
    if (frame == 0) {
      continue;
    }
    uint16_t durationMs = readWord(&durations[frame - 1]);
    if (durationMs < UPDATE_PERIOD_MS) {
      return false;
    }
    for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
      float delta = fabs(
        (float)readWord(&frames[frame][channel])
        - (float)readWord(&frames[frame - 1][channel])
      ) * 0.1f;
      float speed = delta * 1000.0f / durationMs;
      if (speed > TABLE_MAX_DPS + 0.05f) {
        return false;
      }
    }
  }
  return true;
}

bool validateCalibrationAndTrajectory() {
  if (
    !validateTable(STARTUP_FRAMES, STARTUP_DURATIONS_MS)
    || !validateTable(LOOP_FRAMES, LOOP_DURATIONS_MS)
    || !framesEqual(STARTUP_FRAMES, FRAME_COUNT - 1, LOOP_FRAMES, 0)
    || !framesEqual(LOOP_FRAMES, 0, LOOP_FRAMES, FRAME_COUNT - 1)
  ) {
    return false;
  }
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    if (
      readWord(&STARTUP_FRAMES[0][channel])
      != readWord(&SAFE_STAND_X10[channel])
    ) {
      return false;
    }
  }
  return true;
}

void printState() {
  Serial.print(F("OK STATUS STATE="));
  switch (motionState) {
    case OUTPUT_DISABLED:
      Serial.print(F("OUTPUT_DISABLED"));
      break;
    case STANDING:
      Serial.print(F("STANDING"));
      break;
    case PLAYING_STARTUP:
      Serial.print(F("STARTUP_CRAWL"));
      break;
    case PLAYING_LOOP:
      Serial.print(F("FORWARD_CRAWL"));
      break;
    case RETURNING_TO_STAND:
      Serial.print(F("RETURNING_TO_STAND"));
      break;
    case HOLDING:
      Serial.print(F("HOLDING"));
      break;
  }
  Serial.print(F(" ARMED="));
  Serial.print(armed ? 1 : 0);
  Serial.print(F(" OUTPUT="));
  Serial.print(outputEnabled ? 1 : 0);
  Serial.print(F(" CALIBRATION="));
  Serial.print(calibrationValid ? 1 : 0);
  Serial.print(F(" CYCLES_LEFT="));
  Serial.println(cyclesRemaining);
}

void beginNextTableSegment() {
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    segmentStartDeg[channel] = currentDeg[channel];
  }
  ++activeFrameIndex;
  loadFrame(activeFrames, activeFrameIndex, segmentTargetDeg);
  segmentDurationMs = readWord(&activeDurations[activeFrameIndex - 1]);
  segmentStartMs = millis();
}

void beginTable(
  MotionState state,
  const uint16_t frames[][CHANNEL_COUNT],
  const uint16_t durations[FRAME_COUNT - 1]
) {
  motionState = state;
  activeFrames = frames;
  activeDurations = durations;
  activeFrameIndex = 0;
  beginNextTableSegment();
}

void beginReturnToStand() {
  float safeStand[CHANNEL_COUNT];
  loadSafeStand(safeStand);
  float maximumDelta = 0.0f;
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    segmentStartDeg[channel] = currentDeg[channel];
    segmentTargetDeg[channel] = safeStand[channel];
    maximumDelta = max(
      maximumDelta,
      fabs(segmentTargetDeg[channel] - segmentStartDeg[channel])
    );
  }
  uint32_t requiredMs = (uint32_t)ceil(
    maximumDelta * 1000.0f / RETURN_MAX_DPS
  );
  segmentDurationMs = max((uint32_t)MIN_RETURN_MS, requiredMs);
  segmentDurationMs = (
    (segmentDurationMs + UPDATE_PERIOD_MS - 1) / UPDATE_PERIOD_MS
  ) * UPDATE_PERIOD_MS;
  segmentStartMs = millis();
  motionState = RETURNING_TO_STAND;
  cyclesRemaining = 0;
}

void completeTable() {
  if (cyclesRemaining > 0) {
    --cyclesRemaining;
  }
  if (stopRequested || cyclesRemaining == 0) {
    beginReturnToStand();
    return;
  }
  beginTable(PLAYING_LOOP, LOOP_FRAMES, LOOP_DURATIONS_MS);
}

void updateInterpolatedSegment(bool tableSegment) {
  uint32_t elapsed = millis() - segmentStartMs;
  float progress = (
    segmentDurationMs == 0
    ? 1.0f
    : clampFloat((float)elapsed / segmentDurationMs, 0.0f, 1.0f)
  );
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    currentDeg[channel] = (
      segmentStartDeg[channel]
      + (segmentTargetDeg[channel] - segmentStartDeg[channel]) * progress
    );
  }
  writeCurrentFrame();
  if (progress < 1.0f) {
    return;
  }

  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    currentDeg[channel] = segmentTargetDeg[channel];
  }
  if (!tableSegment) {
    motionState = STANDING;
    stopRequested = false;
    runAllowedAfterMs = millis();
    Serial.println(F("OK STANDING RUN_COMPLETE"));
    return;
  }
  if (activeFrameIndex + 1 < FRAME_COUNT) {
    beginNextTableSegment();
  } else {
    completeTable();
  }
}

void updateMotion() {
  if (!outputEnabled || millis() - lastUpdateMs < UPDATE_PERIOD_MS) {
    return;
  }
  lastUpdateMs = millis();
  if (
    motionState == PLAYING_STARTUP
    || motionState == PLAYING_LOOP
  ) {
    updateInterpolatedSegment(true);
  } else if (motionState == RETURNING_TO_STAND) {
    updateInterpolatedSegment(false);
  }
}

void armStandingPose() {
  if (!calibrationValid) {
    Serial.println(F("ERR CALIBRATION_OR_TRAJECTORY_INVALID"));
    return;
  }
  if (
    motionState == PLAYING_STARTUP
    || motionState == PLAYING_LOOP
    || motionState == RETURNING_TO_STAND
  ) {
    Serial.println(F("ERR MOTION_ACTIVE"));
    return;
  }
  if (motionState == HOLDING && outputEnabled) {
    armed = true;
    stopRequested = false;
    beginReturnToStand();
    Serial.println(F("OK ARM RETURNING_TO_STAND"));
    return;
  }

  loadSafeStand(currentDeg);
  outputEnabled = true;
  armed = true;
  stopRequested = false;
  cyclesRemaining = 0;
  motionState = STANDING;
  lastUpdateMs = millis();
  runAllowedAfterMs = millis() + ARM_SETTLE_MS;
  writeCurrentFrame();
  Serial.println(F("OK ARM STANDING WAIT_2_SECONDS_BEFORE_RUN"));
}

void startRun(uint8_t cycleCount) {
  if (!armed || !outputEnabled || motionState != STANDING) {
    Serial.println(F("ERR ARM_AND_STAND_REQUIRED"));
    return;
  }
  if ((int32_t)(millis() - runAllowedAfterMs) < 0) {
    Serial.println(F("ERR ARM_SETTLING"));
    return;
  }
  stopRequested = false;
  cyclesRemaining = cycleCount;
  beginTable(
    PLAYING_STARTUP,
    STARTUP_FRAMES,
    STARTUP_DURATIONS_MS
  );
  Serial.print(F("OK RUN FORWARD CYCLES="));
  Serial.println(cycleCount);
}

void holdImmediately() {
  armed = false;
  stopRequested = true;
  cyclesRemaining = 0;
  motionState = outputEnabled ? HOLDING : OUTPUT_DISABLED;
  Serial.print(F("OK HOLD OUTPUT="));
  Serial.println(outputEnabled ? 1 : 0);
}

bool noExtraToken(char **cursor) {
  return strtok_r(NULL, " \t", cursor) == NULL;
}

void handleCommand(char *line) {
  for (char *cursor = line; *cursor != '\0'; ++cursor) {
    *cursor = (char)toupper(*cursor);
  }
  char *save = NULL;
  char *command = strtok_r(line, " \t", &save);
  if (command == NULL) {
    return;
  }

  if (strcmp(command, "ARM") == 0 && noExtraToken(&save)) {
    armStandingPose();
    return;
  }
  if (
    (strcmp(command, "HOLD") == 0 || strcmp(command, "DISARM") == 0)
    && noExtraToken(&save)
  ) {
    holdImmediately();
    return;
  }
  if (strcmp(command, "DISABLE") == 0 && noExtraToken(&save)) {
    disableOutputs();
    Serial.println(F("OK DISABLE OUTPUT=0"));
    return;
  }
  if (strcmp(command, "STOP") == 0 && noExtraToken(&save)) {
    if (
      motionState == PLAYING_STARTUP
      || motionState == PLAYING_LOOP
    ) {
      stopRequested = true;
      Serial.println(F("OK STOP_QUEUED FINISHING_CURRENT_CYCLE"));
    } else if (motionState == STANDING) {
      Serial.println(F("OK STOPPED STANDING"));
    } else {
      Serial.println(F("OK STOPPED"));
    }
    return;
  }
  if (strcmp(command, "RUN") == 0) {
    char *countToken = strtok_r(NULL, " \t", &save);
    long count = 1;
    if (countToken != NULL) {
      char *end = NULL;
      count = strtol(countToken, &end, 10);
      if (*countToken == '\0' || *end != '\0') {
        Serial.println(F("ERR RUN_COUNT_MUST_BE_1_TO_3"));
        return;
      }
    }
    if (
      !noExtraToken(&save)
      || count < 1
      || count > MAX_RUN_CYCLES
    ) {
      Serial.println(F("ERR RUN_COUNT_MUST_BE_1_TO_3"));
      return;
    }
    startRun((uint8_t)count);
    return;
  }
  if (strcmp(command, "STATUS") == 0 && noExtraToken(&save)) {
    printState();
    return;
  }
  if (strcmp(command, "HELP") == 0 && noExtraToken(&save)) {
    Serial.println(
      F("OK COMMANDS ARM | RUN [1..3] | STOP | HOLD | DISARM | DISABLE | STATUS")
    );
    return;
  }
  Serial.println(F("ERR UNKNOWN_OR_MALFORMED_COMMAND"));
}

void readSerialLines() {
  while (Serial.available() > 0) {
    char value = (char)Serial.read();
    if (discardUntilNewline) {
      if (value == '\n') {
        discardUntilNewline = false;
        lineLength = 0;
      }
      continue;
    }
    if (value == '\r') {
      continue;
    }
    if (value == '\n') {
      lineBuffer[lineLength] = '\0';
      if (lineLength > 0) {
        handleCommand(lineBuffer);
      }
      lineLength = 0;
      continue;
    }
    if (lineLength + 1 < sizeof(lineBuffer)) {
      lineBuffer[lineLength++] = value;
    } else {
      lineLength = 0;
      discardUntilNewline = true;
      Serial.println(F("ERR LINE_TOO_LONG"));
    }
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();
  pwm.begin();
  Wire.setClock(400000L);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);

  loadSafeStand(currentDeg);
  disableOutputs();
  calibrationValid = validateCalibrationAndTrajectory();

  Serial.print(F("OK VOLT_STANDALONE_FORWARD_READY OUTPUT_DISABLED VALID="));
  Serial.println(calibrationValid ? 1 : 0);
  Serial.println(
    F("OK TYPE HELP; THEN ARM; WAIT 2 SECONDS; TYPE RUN 1")
  );
}

void loop() {
  readSerialLines();
  updateMotion();
}
