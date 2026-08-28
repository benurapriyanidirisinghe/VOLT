/*
  VOLT Arduino Nano PCA9685 physical-output firmware.

  Runtime protocol, 250000 baud, newline terminated:
    FRAME d0 d1 d2 d3 d4 d5 d6 d7 d8 d9 d10 d11
      Absolute physical servo angles in degrees, ordered by PCA9685 channel.

    SERVO channel degrees
      Move only one PCA9685 channel target.

    LED COLOR r g b, LED COLOR_B r g b, LED BRIGHTNESS 0-255, LED EFFECT name
    LED SPEED milliseconds, LED PIXEL index r g b
    LED CLEAR, LED OFF, LED STATUS, FACE expression, HOST SYNC

    PING, ARM, HOLD, DISARM, DISABLE, STATUS

  READY, PONG, and STATUS advertise:
    FW=VOLT_PCA9685 PROTO=2 MAX_DPS=240.0

  Safety:
  - Startup leaves PCA9685 outputs disabled and does not move servos.
  - HOLD, DISARM, and timeout reject new motion while maintaining the last
    output; they do not command center or remove holding torque.
  - Host ROS serial bridge owns neutral, trim, direction, and joint conversion.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <stdlib.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Face hardware configuration. The two 8-pixel strips have their DIN wires in
// parallel, so this deliberately owns one 8-pixel logical buffer, not 16.
const uint8_t LED_PIN = 6;  // Arduino D6; change only this constant to rewire.
const uint8_t NUM_FACE_LEDS = 8;
const uint8_t DEFAULT_FACE_BRIGHTNESS = 80;
// This hard ceiling limits current even if a host requests brightness 255.
const uint8_t FACE_BRIGHTNESS_LIMIT = 160;
// Normal DISARM keeps the face alive. Set true if the installation requires
// the LEDs to go dark whenever the motion outputs are disarmed/disabled.
const bool FACE_OFF_ON_DISARM = false;
const uint16_t MIN_FACE_SPEED_MS = 10;
const uint16_t MAX_FACE_SPEED_MS = 60000;
// 15 Hz, not 40: every show() masks interrupts for ~240 us against an ~80 us
// UART tolerance, so each transmission is a gamble that no frame byte is in
// flight.  15 Hz is visually indistinguishable for an 8-pixel face and takes
// 62% fewer gambles per second.
const uint16_t FACE_FRAME_PERIOD_MS = 67;

// Compile-time kill switch for the A/B timing test: build with 0 and every
// NeoPixel transmission is compiled out (the face logic still runs, so FACE
// commands still parse and acknowledge).  If serial corruption counters go
// quiet with this at 0 and return at 1, the corruption is show()'s interrupt
// blackout and nothing else.
#define FACE_LEDS_ENABLED 1

Adafruit_NeoPixel facePixels(
  NUM_FACE_LEDS,
  LED_PIN,
  NEO_GRB + NEO_KHZ800
);

enum FaceEffect : uint8_t {
  FACE_EFFECT_SOLID,
  FACE_EFFECT_BREATHE,
  FACE_EFFECT_BLINK,
  FACE_EFFECT_PULSE,
  FACE_EFFECT_RAINBOW,
  FACE_EFFECT_CHASE,
  FACE_EFFECT_SCANNER,
  FACE_EFFECT_SPARKLE,
  FACE_EFFECT_ALTERNATE,
  FACE_EFFECT_LOADING,
  FACE_EFFECT_OFF,
  FACE_EFFECT_PIXELS,
  FACE_EFFECT_HEARTBEAT,
  FACE_EFFECT_SUCCESS,
  FACE_EFFECT_ERROR,
  FACE_EFFECT_FADE_OFF,
  FACE_EFFECT_STARTUP
};

enum FaceExpression : uint8_t {
  FACE_EXPRESSION_CUSTOM,
  FACE_EXPRESSION_NEUTRAL,
  FACE_EXPRESSION_IDLE,
  FACE_EXPRESSION_HAPPY,
  FACE_EXPRESSION_EXCITED,
  FACE_EXPRESSION_LOVE,
  FACE_EXPRESSION_SAD,
  FACE_EXPRESSION_ANGRY,
  FACE_EXPRESSION_ALERT,
  FACE_EXPRESSION_THINKING,
  FACE_EXPRESSION_CONFUSED,
  FACE_EXPRESSION_SLEEPING,
  FACE_EXPRESSION_SUCCESS,
  FACE_EXPRESSION_ERROR,
  FACE_EXPRESSION_SCARED,
  FACE_EXPRESSION_PLAYFUL,
  FACE_EXPRESSION_SHUTDOWN,
  FACE_EXPRESSION_STARTUP
};

FaceEffect faceEffect = FACE_EFFECT_OFF;
FaceExpression faceExpression = FACE_EXPRESSION_STARTUP;
uint8_t faceColor[3] = {0, 120, 255};
uint8_t faceColorB[3] = {0, 120, 255};
uint8_t facePixelColors[NUM_FACE_LEDS][3];
uint8_t faceBrightness = DEFAULT_FACE_BRIGHTNESS;
uint8_t appliedFaceBrightness = 255;
uint16_t faceSpeedMs = 3000;
uint32_t faceEffectStartMs = 0;
uint32_t lastFaceFrameMs = 0;
uint32_t faceRandomState = 0x564F4C54UL;
uint32_t faceLastSparkleStep = 0xFFFFFFFFUL;
uint8_t faceSparklePixel = 0;
uint8_t faceSolidLevel = 255;
bool faceEnabled = false;
bool faceFrameDirty = true;
bool hostPingSeen = false;
bool hostSnapshotSeen = false;
bool hostSynced = false;

const uint8_t CHANNEL_COUNT = 12;
// Adafruit_NeoPixel masks interrupts for about 30 us per RGB pixel while
// show() transmits.  Eight pixels block the Nano UART receive ISR for about
// 240 us, which is roughly six character times at 250000 baud.  That blackout
// is no longer survived by running the wire slowly; it is survived by only
// transmitting the face inside a measured quiet window between host FRAMEs
// (see faceWindowSafe()).  With that guard in place the link runs at 250000
// baud, which divides exactly from the 16 MHz clock (UBRR=3, 0.0% error) and
// costs 40 us/byte instead of 174 us/byte.  The 4.3x headroom is what allows
// a higher FRAME rate and keeps long STATUS replies from stalling the servo
// loop.  115200 is deliberately not used: it carries 2.1% clock error here.
const uint32_t BAUD_RATE = 250000;
// A complete command is followed by a real idle interval before a face update.
// This avoids mistaking a momentarily empty Arduino RX ring for a quiet wire
// when a USB-UART packet is still arriving.
const uint16_t SERIAL_IDLE_BEFORE_FACE_US = 1000;
// PROTO 3 adds the binary FRAME path (magic 0xA5, sequence, 12 x uint16
// centidegrees, CRC-8).  The host uses binary only when the banner reports
// PROTO>=3, so this firmware remains compatible with an older bridge and an
// older firmware remains compatible with the new bridge.
const uint16_t PROTOCOL_VERSION = 3;
// 50 Hz is the conservative default every calibration to date was measured
// at.  The TD-8130MG is digital and should accept 250-333 Hz, which would
// cut command-to-pulse latency to <=4 ms and improve pulse resolution from
// ~0.49 deg/tick to ~0.10 deg/tick -- but the PCA9685 prescaler rounds, so
// the real period at 250 Hz is ~2% off nominal and every pulse-width constant
// effectively rescales.  Do not raise this without bench-verifying one leg
// against a protractor at stand, min, and max angles first.
const uint16_t SERVO_FREQ_HZ = 50;
const uint16_t SERVO_PERIOD_US = 1000000UL / SERVO_FREQ_HZ;

// ---- Binary FRAME path (PROTO 3) -----------------------------------------
// Layout after the magic byte: [seq][24 bytes: 12 x uint16 LE centideg][crc8]
// CRC-8 Dallas/Maxim (reflected poly 0x8C) over seq+payload.  On any failure
// the frame is dropped SILENTLY and a counter increments: printing an error
// per corrupt frame lengthens the very loop blackout that corrupts frames,
// which is the self-amplifying failure the old ASCII path suffered from.
const uint8_t BIN_FRAME_MAGIC = 0xA5;
const uint8_t BIN_FRAME_BODY_LEN = 26;   // seq + 24 payload + crc
// A binary frame stalled longer than this mid-body is a truncated stream,
// not jitter: at 250000 baud the whole body takes ~1.1 ms.
const uint32_t BIN_FRAME_STALL_US = 20000UL;

uint8_t binBuffer[BIN_FRAME_BODY_LEN];
uint8_t binLength = 0;
bool binActive = false;
bool binHaveLastSeq = false;
uint8_t binLastSeq = 0;

// ---- Link/loop instrumentation (STATUS-surfaced) -------------------------
uint32_t framesRxAscii = 0;
uint32_t framesRxBin = 0;
uint32_t binCrcFailCount = 0;
uint32_t binSeqGapCount = 0;
uint16_t maxLoopUs = 0;      // worst loop() duration since last STATUS
bool loopSampleSuppressed = false;  // set while a long reply is being printed
uint16_t maxI2cUs = 0;       // worst servo I2C write burst since last STATUS
uint32_t ledShowCount = 0;   // NeoPixel show() transmissions
bool frameJustApplied = false;

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

// ch2 (physically the FRONT-RIGHT foot) is 50.0, not 0.0. This is the servo
// that jammed: while ch2/ch5 were cross-wired, body_height 0.210 sent it
// 39.8 deg against a normal walking band of 54..79 -- 14 deg BELOW its usable
// travel, so it drove the linkage into its stop. Its exposed side is the MIN,
// because this channel runs dir -1 from a neutral of 0. 50.0 clears the worst
// emote (happy_dance, 54.1) and walking at 0.200 (53.7).
// It cannot live in servo_calibration.yaml: from_dict() enforces
// min_deg <= neutral_deg and this channel's neutral is 0.
const float CHANNEL_MIN_DEG[CHANNEL_COUNT] = {
  70.0, 0.0, 50.0,
  70.0, 0.0, 30.0,
  50.0, 0.0, 30.0,
  50.0, 0.0, 0.0
};
// ch5 (physically the FRONT-LEFT foot) is 130.0, not the nominal 180.0.
// Its exposed side is the MAX, because this channel runs dir +1 from a
// neutral of 180 -- the mirror of ch2 above. At body_height 0.210 it was
// commanded 140.4 against a normal band of 101..126. It did not visibly jam,
// but it was as far out of band as ch2 was, so it gets the matching guard.
// 130.0 clears the worst emote (happy_dance, 125.9) and walking at 0.200
// (126.3), while blocking the 137.6 that body_height 0.205 would command.
// Like ch2's, this cannot live in servo_calibration.yaml: from_dict()
// enforces neutral_deg <= max_deg and this channel's neutral is 180.
// Replace both with the measured mechanical stops once they are known.
const float CHANNEL_MAX_DEG[CHANNEL_COUNT] = {
  160.0, 180.0, 150.0,
  160.0, 180.0, 130.0,
  140.0, 180.0, 180.0,
  140.0, 180.0, 150.0
};

// Channel-ordered physical angles for the calibrated canonical standing pose.
// Keep this table aligned with servo_calibration.yaml when standing trims or
// centers change. It gives the first live frame a known, mechanically safe
// slew origin instead of jumping directly to an arbitrary first target.
const float CHANNEL_SAFE_START_DEG[CHANNEL_COUNT] = {
  122.863, 88.592, 55.115,
  117.137, 91.408, 118.051,
  107.013, 99.876, 118.051,
  90.237, 80.124, 61.939
};

// This is a fault ceiling, not a shaping filter.  The previous 120 deg/s sat
// only ~4% above the measured ~115 deg/s fast-trot peak, so ordinary swing
// transients clipped; a slew limiter that clips does not merely slow that
// step, it drops the leg behind its own gait phase and the error persists for
// the rest of the swing.  A ceiling should sit well clear of normal motion and
// catch only genuine faults, so it is raised to roughly 2x the observed peak.
// The TD-8130MG is rated near 375 deg/s at 6 V, so this stays inside the
// actuator.  Reduce to 30 deg/s for initial suspended calibration tests.
const float MAX_DEG_PER_SECOND = 240.0;
// PCA9685 output is 50 Hz, so rewriting faster than one 20 ms pulse period
// adds I2C load without improving physical response.
const uint16_t UPDATE_PERIOD_MS = 20;
const uint32_t COMMAND_TIMEOUT_MS = 750;
const bool ACK_FRAME_COMMANDS = false;
// A show() burst plus its reset needs ~300 us of masked interrupts.  Only
// start one when at least this much of the measured inter-FRAME gap remains,
// so the blackout cannot land on top of an arriving frame.
const uint16_t FACE_SHOW_GUARD_US = 2000;
// A corrupt FRAME means the wire is noisy, not that the host has stopped
// talking, so it must not be answered by the liveness timeout alone.  Holding
// position after this many consecutive rejects gives a diagnosable reason
// instead of a silent COMMAND_TIMEOUT disarm.
const uint8_t MAX_CONSECUTIVE_FRAME_REJECTS = 12;
// Rejected frames used to answer every failure with an ERR line, whose own
// transmission lengthened the next loop block and corrupted the next frame.
const uint16_t FRAME_ERROR_REPORT_INTERVAL_MS = 250;

float targetDeg[CHANNEL_COUNT];
float currentDeg[CHANNEL_COUNT];
bool targetValid[CHANNEL_COUNT];
bool channelOutputInitialized[CHANNEL_COUNT];
bool outputEnabled = false;
bool servoArmed = false;
bool timeoutWarned = false;
uint32_t lastCommandMs = 0;
uint32_t lastUpdateMs = 0;
// Measured host FRAME cadence, used to find a safe face-transmit window.
uint32_t lastFrameParsedUs = 0;
uint32_t frameIntervalUs = 0;
uint8_t consecutiveFrameRejects = 0;
uint32_t lastFrameErrorReportMs = 0;

char lineBuffer[192];
uint8_t lineLength = 0;
bool discardLineUntilNewline = false;
uint32_t lastSerialByteUs = 0;

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

uint8_t effectiveFaceBrightness() {
  return faceBrightness < FACE_BRIGHTNESS_LIMIT
    ? faceBrightness
    : FACE_BRIGHTNESS_LIMIT;
}

void setFaceColorValues(
  uint8_t r,
  uint8_t g,
  uint8_t b,
  uint8_t r2,
  uint8_t g2,
  uint8_t b2
) {
  faceColor[0] = r;
  faceColor[1] = g;
  faceColor[2] = b;
  faceColorB[0] = r2;
  faceColorB[1] = g2;
  faceColorB[2] = b2;
}

void setFaceEffectState(FaceEffect effect, uint16_t speedMs, bool enabled) {
  faceEffect = effect;
  faceSpeedMs = speedMs;
  faceEnabled = enabled;
  faceSolidLevel = 255;
  faceEffectStartMs = millis();
  lastFaceFrameMs = faceEffectStartMs - FACE_FRAME_PERIOD_MS;
  faceLastSparkleStep = 0xFFFFFFFFUL;
  faceFrameDirty = true;
}

void clearLogicalFacePixels() {
  for (uint8_t pixel = 0; pixel < NUM_FACE_LEDS; ++pixel) {
    facePixelColors[pixel][0] = 0;
    facePixelColors[pixel][1] = 0;
    facePixelColors[pixel][2] = 0;
  }
}

void printFaceEffectName(FaceEffect effect) {
  switch (effect) {
    case FACE_EFFECT_SOLID: Serial.print(F("solid")); break;
    case FACE_EFFECT_BREATHE: Serial.print(F("breathe")); break;
    case FACE_EFFECT_BLINK: Serial.print(F("blink")); break;
    case FACE_EFFECT_PULSE: Serial.print(F("pulse")); break;
    case FACE_EFFECT_RAINBOW: Serial.print(F("rainbow")); break;
    case FACE_EFFECT_CHASE: Serial.print(F("chase")); break;
    case FACE_EFFECT_SCANNER: Serial.print(F("scanner")); break;
    case FACE_EFFECT_SPARKLE: Serial.print(F("sparkle")); break;
    case FACE_EFFECT_ALTERNATE: Serial.print(F("alternate")); break;
    case FACE_EFFECT_LOADING: Serial.print(F("loading")); break;
    case FACE_EFFECT_OFF: Serial.print(F("off")); break;
    case FACE_EFFECT_PIXELS: Serial.print(F("pixels")); break;
    case FACE_EFFECT_HEARTBEAT: Serial.print(F("heartbeat")); break;
    case FACE_EFFECT_SUCCESS: Serial.print(F("success")); break;
    case FACE_EFFECT_ERROR: Serial.print(F("error")); break;
    case FACE_EFFECT_FADE_OFF: Serial.print(F("fade_off")); break;
    case FACE_EFFECT_STARTUP: Serial.print(F("startup")); break;
  }
}

void printFaceExpressionName(FaceExpression expression) {
  switch (expression) {
    case FACE_EXPRESSION_CUSTOM: Serial.print(F("custom")); break;
    case FACE_EXPRESSION_NEUTRAL: Serial.print(F("neutral")); break;
    case FACE_EXPRESSION_IDLE: Serial.print(F("idle")); break;
    case FACE_EXPRESSION_HAPPY: Serial.print(F("happy")); break;
    case FACE_EXPRESSION_EXCITED: Serial.print(F("excited")); break;
    case FACE_EXPRESSION_LOVE: Serial.print(F("love")); break;
    case FACE_EXPRESSION_SAD: Serial.print(F("sad")); break;
    case FACE_EXPRESSION_ANGRY: Serial.print(F("angry")); break;
    case FACE_EXPRESSION_ALERT: Serial.print(F("alert")); break;
    case FACE_EXPRESSION_THINKING: Serial.print(F("thinking")); break;
    case FACE_EXPRESSION_CONFUSED: Serial.print(F("confused")); break;
    case FACE_EXPRESSION_SLEEPING: Serial.print(F("sleeping")); break;
    case FACE_EXPRESSION_SUCCESS: Serial.print(F("success")); break;
    case FACE_EXPRESSION_ERROR: Serial.print(F("error")); break;
    case FACE_EXPRESSION_SCARED: Serial.print(F("scared")); break;
    case FACE_EXPRESSION_PLAYFUL: Serial.print(F("playful")); break;
    case FACE_EXPRESSION_SHUTDOWN: Serial.print(F("shutdown")); break;
    case FACE_EXPRESSION_STARTUP: Serial.print(F("startup")); break;
  }
}

void printFaceStatusFields() {
  Serial.print(F(" LED_ENABLED="));
  Serial.print(faceEnabled ? 1 : 0);
  Serial.print(F(" LED_COLOR="));
  Serial.print(faceColor[0]);
  Serial.print(',');
  Serial.print(faceColor[1]);
  Serial.print(',');
  Serial.print(faceColor[2]);
  Serial.print(F(" LED_COLOR_B="));
  Serial.print(faceColorB[0]);
  Serial.print(',');
  Serial.print(faceColorB[1]);
  Serial.print(',');
  Serial.print(faceColorB[2]);
  Serial.print(F(" LED_BRIGHTNESS="));
  Serial.print(faceBrightness);
  Serial.print(F(" LED_EFFECTIVE_BRIGHTNESS="));
  Serial.print(effectiveFaceBrightness());
  Serial.print(F(" LED_LIMIT="));
  Serial.print(FACE_BRIGHTNESS_LIMIT);
  Serial.print(F(" LED_EFFECT="));
  printFaceEffectName(faceEffect);
  Serial.print(F(" LED_SPEED_MS="));
  Serial.print(faceSpeedMs);
  Serial.print(F(" FACE="));
  printFaceExpressionName(faceExpression);
}

void printHostSyncFields() {
  Serial.print(F(" HOST_SYNC_REQUIRED=1 HOST_PING="));
  Serial.print(hostPingSeen ? 1 : 0);
  Serial.print(F(" HOST_SNAPSHOT="));
  Serial.print(hostSnapshotSeen ? 1 : 0);
  Serial.print(F(" HOST_SYNCED="));
  Serial.print(hostSynced ? 1 : 0);
}

void noteHostFaceMutation() {
  hostSnapshotSeen = true;
  hostSynced = false;
}

void setRenderedPixel(uint8_t pixel, const uint8_t color[3], uint8_t level) {
  uint8_t r = ((uint16_t)color[0] * level) / 255U;
  uint8_t g = ((uint16_t)color[1] * level) / 255U;
  uint8_t b = ((uint16_t)color[2] * level) / 255U;
  facePixels.setPixelColor(pixel, r, g, b);
}

void fillRenderedFace(const uint8_t color[3], uint8_t level) {
  for (uint8_t pixel = 0; pixel < NUM_FACE_LEDS; ++pixel) {
    setRenderedPixel(pixel, color, level);
  }
}

uint8_t triangleLevel(uint32_t elapsedMs, uint16_t periodMs) {
  uint32_t phase = elapsedMs % periodMs;
  uint16_t half = periodMs / 2U;
  if (half == 0) {
    return 255;
  }
  if (phase > half) {
    phase = periodMs - phase;
  }
  return (uint8_t)((phase * 255UL) / half);
}

uint32_t rainbowColor(uint8_t position) {
  position = 255 - position;
  if (position < 85) {
    return facePixels.Color(255 - position * 3, 0, position * 3);
  }
  if (position < 170) {
    position -= 85;
    return facePixels.Color(0, position * 3, 255 - position * 3);
  }
  position -= 170;
  return facePixels.Color(position * 3, 255 - position * 3, 0);
}

void applyFacePreset(FaceExpression expression) {
  faceExpression = expression;
  switch (expression) {
    case FACE_EXPRESSION_NEUTRAL:
      setFaceColorValues(80, 180, 255, 80, 180, 255);
      setFaceEffectState(FACE_EFFECT_BREATHE, 5000, true);
      break;
    case FACE_EXPRESSION_IDLE:
      setFaceColorValues(0, 120, 255, 0, 120, 255);
      setFaceEffectState(FACE_EFFECT_BREATHE, 3000, true);
      break;
    case FACE_EXPRESSION_HAPPY:
      setFaceColorValues(255, 180, 20, 255, 180, 20);
      setFaceEffectState(FACE_EFFECT_PULSE, 1200, true);
      break;
    case FACE_EXPRESSION_EXCITED:
      setFaceColorValues(0, 255, 255, 255, 0, 180);
      setFaceEffectState(FACE_EFFECT_ALTERNATE, 180, true);
      break;
    case FACE_EXPRESSION_LOVE:
      setFaceColorValues(255, 20, 80, 255, 20, 80);
      setFaceEffectState(FACE_EFFECT_HEARTBEAT, 1200, true);
      break;
    case FACE_EXPRESSION_SAD:
      setFaceColorValues(20, 50, 180, 20, 50, 180);
      setFaceEffectState(FACE_EFFECT_BREATHE, 4200, true);
      break;
    case FACE_EXPRESSION_ANGRY:
      setFaceColorValues(255, 0, 0, 255, 0, 0);
      setFaceEffectState(FACE_EFFECT_SCANNER, 120, true);
      break;
    case FACE_EXPRESSION_ALERT:
      setFaceColorValues(255, 0, 0, 255, 90, 0);
      setFaceEffectState(FACE_EFFECT_ALTERNATE, 180, true);
      break;
    case FACE_EXPRESSION_THINKING:
      setFaceColorValues(150, 40, 255, 150, 40, 255);
      setFaceEffectState(FACE_EFFECT_LOADING, 180, true);
      break;
    case FACE_EXPRESSION_CONFUSED:
      setFaceColorValues(170, 40, 255, 255, 180, 0);
      setFaceEffectState(FACE_EFFECT_ALTERNATE, 450, true);
      break;
    case FACE_EXPRESSION_SLEEPING:
      setFaceColorValues(0, 20, 80, 0, 20, 80);
      setFaceEffectState(FACE_EFFECT_BREATHE, 5000, true);
      break;
    case FACE_EXPRESSION_SUCCESS:
      setFaceColorValues(0, 255, 80, 0, 255, 80);
      setFaceEffectState(FACE_EFFECT_SUCCESS, 150, true);
      break;
    case FACE_EXPRESSION_ERROR:
      setFaceColorValues(255, 0, 0, 255, 0, 0);
      setFaceEffectState(FACE_EFFECT_ERROR, 100, true);
      break;
    case FACE_EXPRESSION_SCARED:
      setFaceColorValues(180, 220, 255, 180, 220, 255);
      setFaceEffectState(FACE_EFFECT_BLINK, 120, true);
      break;
    case FACE_EXPRESSION_PLAYFUL:
      setFaceColorValues(0, 255, 255, 255, 0, 180);
      setFaceEffectState(FACE_EFFECT_RAINBOW, 120, true);
      break;
    case FACE_EXPRESSION_SHUTDOWN:
      setFaceEffectState(FACE_EFFECT_FADE_OFF, 900, true);
      break;
    case FACE_EXPRESSION_STARTUP:
      setFaceColorValues(0, 255, 255, 0, 120, 255);
      setFaceEffectState(FACE_EFFECT_STARTUP, 100, true);
      break;
    case FACE_EXPRESSION_CUSTOM:
      break;
  }
  // setFaceEffectState does not own semantic preset identity.
  faceExpression = expression;
}

void turnFaceOff(FaceExpression expression) {
  faceExpression = expression;
  setFaceEffectState(FACE_EFFECT_OFF, faceSpeedMs, false);
  faceExpression = expression;
}

void updateFaceLeds() {
  uint32_t now = millis();
  uint32_t elapsed = now - faceEffectStartMs;

  if (faceEffect == FACE_EFFECT_FADE_OFF && elapsed >= faceSpeedMs) {
    turnFaceOff(FACE_EXPRESSION_SHUTDOWN);
    elapsed = 0;
  } else if (faceEffect == FACE_EFFECT_SUCCESS && elapsed >= 650UL) {
    faceEffect = FACE_EFFECT_SOLID;
    faceSolidLevel = 255;
    faceFrameDirty = true;
  } else if (faceEffect == FACE_EFFECT_ERROR && elapsed >= 650UL) {
    faceEffect = FACE_EFFECT_SOLID;
    faceSolidLevel = 40;
    faceFrameDirty = true;
  }

  bool staticEffect = faceEffect == FACE_EFFECT_SOLID
    || faceEffect == FACE_EFFECT_OFF
    || faceEffect == FACE_EFFECT_PIXELS;
  if (staticEffect && !faceFrameDirty) {
    return;
  }
  if (!faceFrameDirty && now - lastFaceFrameMs < FACE_FRAME_PERIOD_MS) {
    return;
  }
  lastFaceFrameMs = now;

  uint8_t brightness = effectiveFaceBrightness();
  if (brightness != appliedFaceBrightness) {
    facePixels.setBrightness(brightness);
    appliedFaceBrightness = brightness;
  }
  facePixels.clear();

  if (faceEnabled && faceEffect != FACE_EFFECT_OFF) {
    switch (faceEffect) {
      case FACE_EFFECT_SOLID:
        fillRenderedFace(faceColor, faceSolidLevel);
        break;
      case FACE_EFFECT_BREATHE: {
        uint8_t level = 24 + ((uint16_t)triangleLevel(elapsed, faceSpeedMs) * 231U) / 255U;
        fillRenderedFace(faceColor, level);
        break;
      }
      case FACE_EFFECT_BLINK: {
        uint16_t halfPeriod = faceSpeedMs / 2U;
        bool on = ((elapsed / (halfPeriod ? halfPeriod : 1U)) & 1U) == 0;
        fillRenderedFace(faceColor, on ? 255 : 0);
        break;
      }
      case FACE_EFFECT_PULSE: {
        uint8_t level = 80 + ((uint16_t)triangleLevel(elapsed, faceSpeedMs) * 175U) / 255U;
        fillRenderedFace(faceColor, level);
        break;
      }
      case FACE_EFFECT_RAINBOW: {
        uint8_t offset = (uint8_t)(elapsed / faceSpeedMs);
        for (uint8_t pixel = 0; pixel < NUM_FACE_LEDS; ++pixel) {
          facePixels.setPixelColor(
            pixel,
            rainbowColor((uint8_t)(pixel * (256U / NUM_FACE_LEDS) + offset * 8U))
          );
        }
        break;
      }
      case FACE_EFFECT_CHASE: {
        uint8_t head = (elapsed / faceSpeedMs) % NUM_FACE_LEDS;
        for (uint8_t pixel = 0; pixel < NUM_FACE_LEDS; ++pixel) {
          setRenderedPixel(pixel, faceColorB, pixel == head ? 64 : 18);
        }
        setRenderedPixel(head, faceColor, 255);
        break;
      }
      case FACE_EFFECT_SCANNER: {
        const uint8_t pathLength = NUM_FACE_LEDS * 2U - 2U;
        uint8_t position = (elapsed / faceSpeedMs) % pathLength;
        if (position >= NUM_FACE_LEDS) {
          position = pathLength - position;
        }
        if (position > 0) {
          setRenderedPixel(position - 1U, faceColor, 48);
        }
        setRenderedPixel(position, faceColor, 255);
        if (position + 1U < NUM_FACE_LEDS) {
          setRenderedPixel(position + 1U, faceColor, 48);
        }
        break;
      }
      case FACE_EFFECT_SPARKLE: {
        fillRenderedFace(faceColor, 20);
        uint32_t sparkleStep = elapsed / faceSpeedMs;
        if (sparkleStep != faceLastSparkleStep) {
          faceLastSparkleStep = sparkleStep;
          faceRandomState = faceRandomState * 1664525UL + 1013904223UL;
          faceSparklePixel = (faceRandomState >> 24) % NUM_FACE_LEDS;
        }
        setRenderedPixel(faceSparklePixel, faceColor, 255);
        break;
      }
      case FACE_EFFECT_ALTERNATE: {
        bool swapColors = ((elapsed / faceSpeedMs) & 1U) != 0;
        for (uint8_t pixel = 0; pixel < NUM_FACE_LEDS; ++pixel) {
          bool useSecond = ((pixel & 1U) != 0) ^ swapColors;
          setRenderedPixel(pixel, useSecond ? faceColorB : faceColor, 255);
        }
        break;
      }
      case FACE_EFFECT_LOADING: {
        uint8_t head = (elapsed / faceSpeedMs) % NUM_FACE_LEDS;
        setRenderedPixel(head, faceColor, 255);
        setRenderedPixel(
          (head + NUM_FACE_LEDS - 1U) % NUM_FACE_LEDS,
          faceColor,
          90
        );
        setRenderedPixel(
          (head + NUM_FACE_LEDS - 2U) % NUM_FACE_LEDS,
          faceColor,
          30
        );
        break;
      }
      case FACE_EFFECT_PIXELS:
        for (uint8_t pixel = 0; pixel < NUM_FACE_LEDS; ++pixel) {
          setRenderedPixel(pixel, facePixelColors[pixel], 255);
        }
        break;
      case FACE_EFFECT_HEARTBEAT: {
        uint32_t phase = elapsed % faceSpeedMs;
        uint32_t firstEnd = faceSpeedMs * 14UL / 100UL;
        uint32_t gapEnd = faceSpeedMs * 22UL / 100UL;
        uint32_t secondEnd = faceSpeedMs * 38UL / 100UL;
        uint8_t level = 18;
        if (phase < firstEnd) {
          level = triangleLevel(phase, (uint16_t)(firstEnd * 2UL));
        } else if (phase >= gapEnd && phase < secondEnd) {
          level = triangleLevel(
            phase - gapEnd,
            (uint16_t)((secondEnd - gapEnd) * 2UL)
          );
        }
        fillRenderedFace(faceColor, level);
        break;
      }
      case FACE_EFFECT_SUCCESS: {
        uint16_t phase = (uint16_t)elapsed;
        bool on = phase < 150U || (phase >= 300U && phase < 450U);
        fillRenderedFace(faceColor, on ? 255 : 0);
        break;
      }
      case FACE_EFFECT_ERROR: {
        uint16_t phase = (uint16_t)elapsed;
        bool on = phase < 100U
          || (phase >= 200U && phase < 300U)
          || (phase >= 400U && phase < 500U);
        fillRenderedFace(faceColor, on ? 255 : 0);
        break;
      }
      case FACE_EFFECT_FADE_OFF: {
        uint8_t level = 255U - (uint8_t)((elapsed * 255UL) / faceSpeedMs);
        fillRenderedFace(faceColor, level);
        break;
      }
      case FACE_EFFECT_STARTUP: {
        uint8_t head = (elapsed / faceSpeedMs) % NUM_FACE_LEDS;
        setRenderedPixel(head, faceColor, 255);
        setRenderedPixel((head + NUM_FACE_LEDS - 1U) % NUM_FACE_LEDS, faceColorB, 80);
        break;
      }
      case FACE_EFFECT_OFF:
        break;
    }
  }

  // One show transmits the 8-pixel buffer to both parallel strips. At most
  // 15 animation frames/s are sent, and static faces transmit only on change.
#if FACE_LEDS_ENABLED
  facePixels.show();
  ledShowCount++;
#endif
  faceFrameDirty = false;
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
  channelOutputInitialized[channel] = true;
}

// One PCA9685 register burst instead of twelve setPWM() transactions.  Each
// setPWM() is its own I2C START/addr/reg/4-bytes/STOP (~135 us at 400 kHz);
// twelve of them cost ~1.6 ms.  With the auto-increment bit set, consecutive
// LEDn registers accept a single stream -- but the AVR Wire buffer is 32
// bytes, so the 48 data bytes must go as TWO transactions (channels 0-5 from
// LED0_ON_L, channels 6-11 from LED6_ON_L), ~0.6 ms each.  A single 48-byte
// burst is not possible on stock AVR Wire.  Register and address names come
// from Adafruit_PWMServoDriver.h (PCA9685_LED0_ON_L etc are its macros).

void writeAllChannelsBurst() {
  uint16_t ticks[CHANNEL_COUNT];
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    ticks[channel] =
      pulseUsToTicks(angleToPulseUs(channel, currentDeg[channel]));
  }
  uint32_t startUs = micros();
  for (uint8_t half = 0; half < 2; ++half) {
    uint8_t first = half * 6;
    Wire.beginTransmission(PCA9685_I2C_ADDRESS);
    Wire.write((uint8_t)(PCA9685_LED0_ON_L + 4 * first));
    for (uint8_t channel = first; channel < first + 6; ++channel) {
      Wire.write((uint8_t)0x00);                    // ON low
      Wire.write((uint8_t)0x00);                    // ON high
      Wire.write((uint8_t)(ticks[channel] & 0xFF)); // OFF low
      Wire.write((uint8_t)(ticks[channel] >> 8));   // OFF high
    }
    Wire.endTransmission();
  }
  uint32_t burstUs = (uint32_t)(micros() - startUs);
  if (burstUs > maxI2cUs) {
    maxI2cUs = burstUs > 65535UL ? 65535U : (uint16_t)burstUs;
  }
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    channelOutputInitialized[channel] = true;
  }
}

void disableOutputs() {
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    pwm.setPWM(channel, 0, 0);
    channelOutputInitialized[channel] = false;
  }
  outputEnabled = false;
}

void holdCurrentPosition() {
  // Cancel any unfinished slew. Without this, HOLD could reject new commands
  // while the update loop continued moving toward the previous target.
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    if (targetValid[channel]) {
      targetDeg[channel] = currentDeg[channel];
    }
  }
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

  bool needWrite[CHANNEL_COUNT];
  uint8_t validCount = 0;
  bool anyWrite = false;
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    needWrite[channel] = false;
    if (!targetValid[channel]) {
      continue;
    }
    ++validCount;
    float error = targetDeg[channel] - currentDeg[channel];
    bool moved = fabs(error) > 0.001f;
    if (error > maxStep) {
      error = maxStep;
    } else if (error < -maxStep) {
      error = -maxStep;
    }
    currentDeg[channel] += error;
    if (moved || !channelOutputInitialized[channel]) {
      needWrite[channel] = true;
      anyWrite = true;
    }
  }
  if (!anyWrite) {
    return;
  }
  // The burst writes all twelve LED registers, so it is only correct once
  // every channel has a valid target (a FRAME provides all twelve at once).
  // Before that -- e.g. single-channel SERVO commands during bring-up -- the
  // legacy per-channel path avoids energizing untargeted outputs.
  if (validCount == CHANNEL_COUNT) {
    writeAllChannelsBurst();
    return;
  }
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    if (needWrite[channel]) {
      writeChannel(channel, currentDeg[channel]);
    }
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

bool parseLongStrict(char *token, long *value) {
  if (token == NULL || token[0] == '\0') {
    return false;
  }
  char *end = NULL;
  long parsed = strtol(token, &end, 10);
  if (end == token || *end != '\0') {
    return false;
  }
  *value = parsed;
  return true;
}

long clampLong(long value, long low, long high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

bool parseFaceEffect(char *token, FaceEffect *effect) {
  if (token == NULL) {
    return false;
  }
  if (strcmp(token, "solid") == 0) *effect = FACE_EFFECT_SOLID;
  else if (strcmp(token, "breathe") == 0) *effect = FACE_EFFECT_BREATHE;
  else if (strcmp(token, "blink") == 0) *effect = FACE_EFFECT_BLINK;
  else if (strcmp(token, "pulse") == 0) *effect = FACE_EFFECT_PULSE;
  else if (strcmp(token, "rainbow") == 0) *effect = FACE_EFFECT_RAINBOW;
  else if (strcmp(token, "chase") == 0) *effect = FACE_EFFECT_CHASE;
  else if (strcmp(token, "scanner") == 0) *effect = FACE_EFFECT_SCANNER;
  else if (strcmp(token, "sparkle") == 0) *effect = FACE_EFFECT_SPARKLE;
  else if (strcmp(token, "alternate") == 0) *effect = FACE_EFFECT_ALTERNATE;
  else if (strcmp(token, "loading") == 0) *effect = FACE_EFFECT_LOADING;
  else if (strcmp(token, "off") == 0) *effect = FACE_EFFECT_OFF;
  else return false;
  return true;
}

bool parseFaceExpression(char *token, FaceExpression *expression) {
  if (token == NULL) {
    return false;
  }
  if (strcmp(token, "neutral") == 0) *expression = FACE_EXPRESSION_NEUTRAL;
  else if (strcmp(token, "idle") == 0) *expression = FACE_EXPRESSION_IDLE;
  else if (strcmp(token, "happy") == 0) *expression = FACE_EXPRESSION_HAPPY;
  else if (strcmp(token, "excited") == 0) *expression = FACE_EXPRESSION_EXCITED;
  else if (strcmp(token, "love") == 0) *expression = FACE_EXPRESSION_LOVE;
  else if (strcmp(token, "sad") == 0) *expression = FACE_EXPRESSION_SAD;
  else if (strcmp(token, "angry") == 0) *expression = FACE_EXPRESSION_ANGRY;
  else if (strcmp(token, "alert") == 0) *expression = FACE_EXPRESSION_ALERT;
  else if (strcmp(token, "thinking") == 0) *expression = FACE_EXPRESSION_THINKING;
  else if (strcmp(token, "confused") == 0) *expression = FACE_EXPRESSION_CONFUSED;
  else if (strcmp(token, "sleeping") == 0) *expression = FACE_EXPRESSION_SLEEPING;
  else if (strcmp(token, "success") == 0) *expression = FACE_EXPRESSION_SUCCESS;
  else if (strcmp(token, "error") == 0) *expression = FACE_EXPRESSION_ERROR;
  else if (strcmp(token, "scared") == 0) *expression = FACE_EXPRESSION_SCARED;
  else if (strcmp(token, "playful") == 0) *expression = FACE_EXPRESSION_PLAYFUL;
  else if (strcmp(token, "shutdown") == 0) *expression = FACE_EXPRESSION_SHUTDOWN;
  else return false;
  return true;
}

FaceEffect preservePresetSequenceEffect(FaceEffect requestedEffect) {
  // The host applies a complete snapshot as FACE, COLOR, BRIGHTNESS, SPEED,
  // EFFECT. Its public pulse/blink names describe the preset in the GUI, while
  // these three expressions need richer internal one-shot/double-pulse states.
  if (
    faceExpression == FACE_EXPRESSION_LOVE
    && requestedEffect == FACE_EFFECT_PULSE
  ) {
    return FACE_EFFECT_HEARTBEAT;
  }
  if (
    faceExpression == FACE_EXPRESSION_SUCCESS
    && requestedEffect == FACE_EFFECT_BLINK
  ) {
    return FACE_EFFECT_SUCCESS;
  }
  if (
    faceExpression == FACE_EXPRESSION_ERROR
    && requestedEffect == FACE_EFFECT_BLINK
  ) {
    return FACE_EFFECT_ERROR;
  }
  return requestedEffect;
}

bool ensureNoExtraTokens(char **cursor) {
  return nextToken(cursor) == NULL;
}

void setTargetChannel(uint8_t channel, float degrees) {
  degrees = clampFloat(degrees, CHANNEL_MIN_DEG[channel], CHANNEL_MAX_DEG[channel]);
  targetDeg[channel] = degrees;
  if (!targetValid[channel]) {
    targetValid[channel] = true;
  }
}

void enableOutputsForMotion() {
  if (!outputEnabled) {
    // Do not let time spent disarmed inflate the first slew step.
    lastUpdateMs = millis();
  }
  outputEnabled = true;
}

void printCapabilityFields() {
  Serial.print(F(" FW=VOLT_PCA9685 PROTO="));
  Serial.print(PROTOCOL_VERSION);
  Serial.print(F(" MAX_DPS="));
  Serial.print(MAX_DEG_PER_SECOND, 1);
  Serial.print(F(" FACE_SUPPORTED=1 LED_COUNT="));
  Serial.print(NUM_FACE_LEDS);
  printHostSyncFields();
}

// Free SRAM between the heap break and the current stack top.  The binary
// parser, the counters, and the two NeoPixel strip buffers all share 2 KB;
// if this drops below ~300 bytes under load, stop adding features.
int freeSramBytes() {
  extern int __heap_start, *__brkval;
  int stackTop;
  return (int)&stackTop
    - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void printStatus() {
  // A STATUS reply is a few hundred bytes; beyond the 64-byte TX ring
  // Serial.print blocks at ~40 us/byte, so the loop that prints it is not a
  // representative loop.  Suppress that pass, otherwise LOOP_MAX_US reports
  // the cost of its own reporting (~16 ms) and hides the real worst case.
  loopSampleSuppressed = true;
  Serial.print(F("OK STATUS"));
  printCapabilityFields();
  Serial.print(F(" ARMED="));
  Serial.print(servoArmed ? 1 : 0);
  Serial.print(F(" OUTPUT="));
  Serial.print(outputEnabled ? 1 : 0);
  Serial.print(F(" LAST_CMD_MS="));
  Serial.print(millis() - lastCommandMs);
  // Link-health counters (stage-1 acceptance: CRC_FAIL and SEQ_GAP stay 0
  // over 60 s of walking with the face animating).  The two *_MAX_US maxima
  // are per-interval: printing resets them, so each STATUS reports the worst
  // case since the previous STATUS.
  Serial.print(F(" FRAMES_ASCII="));
  Serial.print(framesRxAscii);
  Serial.print(F(" FRAMES_BIN="));
  Serial.print(framesRxBin);
  Serial.print(F(" CRC_FAIL="));
  Serial.print(binCrcFailCount);
  Serial.print(F(" SEQ_GAP="));
  Serial.print(binSeqGapCount);
  Serial.print(F(" LOOP_MAX_US="));
  Serial.print(maxLoopUs);
  Serial.print(F(" BUS_MAX_US="));
  Serial.print(maxI2cUs);
  Serial.print(F(" LED_SHOWS="));
  Serial.print(ledShowCount);
  Serial.print(F(" SRAM_FREE="));
  Serial.print(freeSramBytes());
  maxLoopUs = 0;
  maxI2cUs = 0;
  printFaceStatusFields();
  Serial.println();
}

// A rejected FRAME carries two separate facts that used to be conflated.
// The host IS still talking, so the liveness timeout must not fire -- that
// silent COMMAND_TIMEOUT disarm was indistinguishable from a real host loss.
// But we also have no fresh target, so staying armed indefinitely on stale
// targets is not acceptable either.  Liveness is refreshed here; loss of
// usable targets is answered separately by a bounded reject count.
//
// The ERR reply is rate limited because answering every reject was itself
// self-amplifying: the reply lengthened the next loop block, which corrupted
// the next frame, which produced another reply.
bool rejectFrame(const __FlashStringHelper *reason) {
  uint32_t now = millis();
  lastCommandMs = now;
  timeoutWarned = false;

  if (consecutiveFrameRejects < 255) {
    ++consecutiveFrameRejects;
  }

  if (now - lastFrameErrorReportMs >= FRAME_ERROR_REPORT_INTERVAL_MS) {
    lastFrameErrorReportMs = now;
    Serial.print(F("ERR "));
    Serial.print(reason);
    Serial.print(F(" REJECTS="));
    Serial.println(consecutiveFrameRejects);
  }

  if (servoArmed && consecutiveFrameRejects >= MAX_CONSECUTIVE_FRAME_REJECTS) {
    holdCurrentPosition();
    servoArmed = false;
    timeoutWarned = true;
    Serial.println(F("WARN FRAME_LINK_DEGRADED HOLDING ARMED=0"));
  }
  return false;
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
      return rejectFrame(F("BAD_COUNT"));
    }
    if (!parseFloatStrict(token, &values[channel])) {
      return rejectFrame(F("BAD_VALUE"));
    }
  }

  if (!ensureNoExtraTokens(cursor)) {
    return rejectFrame(F("BAD_COUNT"));
  }

  enableOutputsForMotion();
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    setTargetChannel(channel, values[channel]);
  }

  // Track the host cadence so the face transmitter can find a gap that will
  // not collide with the next frame.  Gaps longer than 200 ms mean the stream
  // stopped rather than jittered, so they must not widen the estimate.
  noteFrameApplied();
  framesRxAscii++;
  if (ACK_FRAME_COMMANDS) {
    Serial.println(F("OK FRAME"));
  }
  return true;
}

// Shared bookkeeping for both frame encodings: host-cadence tracking for the
// face transmitter, liveness refresh, and the post-frame face slot flag.
void noteFrameApplied() {
  uint32_t nowUs = micros();
  if (lastFrameParsedUs != 0) {
    uint32_t delta = (uint32_t)(nowUs - lastFrameParsedUs);
    if (delta < 200000UL) {
      frameIntervalUs = frameIntervalUs == 0
        ? delta
        : (frameIntervalUs - (frameIntervalUs >> 2)) + (delta >> 2);
    } else {
      frameIntervalUs = 0;
    }
  }
  lastFrameParsedUs = nowUs;

  consecutiveFrameRejects = 0;
  lastCommandMs = millis();
  timeoutWarned = false;
  frameJustApplied = true;
}

// CRC-8 Dallas/Maxim, reflected polynomial 0x8C, init 0x00.  Bitwise on
// purpose: a 256-byte lookup table would cost 12% of the Nano's SRAM.
// Reference vector: crc8 of "123456789" is 0xA1.
uint8_t crc8Maxim(const uint8_t *data, uint8_t length) {
  uint8_t crc = 0x00;
  for (uint8_t index = 0; index < length; ++index) {
    uint8_t byte = data[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      uint8_t mix = (crc ^ byte) & 0x01;
      crc >>= 1;
      if (mix) {
        crc ^= 0x8C;
      }
      byte >>= 1;
    }
  }
  return crc;
}

// Complete binary frame body received: validate, count, apply.  Corruption is
// counted and dropped in silence -- see the BIN_FRAME_MAGIC comment.
void processBinaryFrame() {
  binActive = false;
  if (crc8Maxim(binBuffer, BIN_FRAME_BODY_LEN - 1)
      != binBuffer[BIN_FRAME_BODY_LEN - 1]) {
    binCrcFailCount++;
    return;
  }
  uint8_t sequence = binBuffer[0];
  if (binHaveLastSeq && (uint8_t)(sequence - binLastSeq) != 1) {
    binSeqGapCount++;
  }
  binHaveLastSeq = true;
  binLastSeq = sequence;

  // Counted HERE, on a frame that passed CRC, not at the end after the
  // arm check. FRAMES_BIN is what the host's link-health panel reads as
  // evidence the transport works, and a disarmed board is exactly when an
  // operator wants that evidence -- before arming, not after. Counting it
  // at the end meant a perfectly healthy link reported FRAMES_BIN=0
  // alongside CRC_FAIL=0 and SEQ_GAP=0, which reads as "nothing is
  // arriving" rather than "nothing is being applied, as intended".
  framesRxBin++;

  if (!servoArmed) {
    // The host is alive and streaming; refresh liveness but move nothing.
    // No print: a disarmed board answering 60 frames/s would flood the wire.
    lastCommandMs = millis();
    return;
  }

  float values[CHANNEL_COUNT];
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    uint16_t centideg = (uint16_t)binBuffer[1 + 2 * channel]
      | ((uint16_t)binBuffer[2 + 2 * channel] << 8);
    if (centideg > 18000U) {
      // Structurally valid but semantically impossible; treat as corruption
      // the CRC happened not to catch.
      binCrcFailCount++;
      return;
    }
    values[channel] = centideg * 0.01f;
  }

  enableOutputsForMotion();
  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    setTargetChannel(channel, values[channel]);
  }
  noteFrameApplied();
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

  enableOutputsForMotion();
  setTargetChannel(channel, degrees);
  lastCommandMs = millis();
  timeoutWarned = false;
  Serial.print(F("OK SERVO "));
  Serial.print(channel);
  Serial.print(F(" "));
  Serial.println(targetDeg[channel], 2);
  return true;
}

bool handleLed(char **cursor) {
  char *subcommand = nextToken(cursor);
  if (subcommand == NULL) {
    Serial.println(F("ERR LED BAD_COUNT"));
    return false;
  }

  if (strcmp(subcommand, "COLOR") == 0) {
    long values[3];
    for (uint8_t component = 0; component < 3; ++component) {
      if (!parseLongStrict(nextToken(cursor), &values[component])) {
        Serial.println(F("ERR LED BAD_VALUE"));
        return false;
      }
      values[component] = clampLong(values[component], 0, 255);
    }
    if (!ensureNoExtraTokens(cursor)) {
      Serial.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    FaceExpression expression = faceExpression;
    bool preserveExpression = expression != FACE_EXPRESSION_CUSTOM
      && expression != FACE_EXPRESSION_STARTUP
      && expression != FACE_EXPRESSION_SHUTDOWN;
    faceColor[0] = (uint8_t)values[0];
    faceColor[1] = (uint8_t)values[1];
    faceColor[2] = (uint8_t)values[2];
    // Preserve a preset's secondary color. This is essential for the
    // excited/alert/confused two-color effects after the host re-sends RGB A.
    if (!preserveExpression) {
      faceColorB[0] = faceColor[0];
      faceColorB[1] = faceColor[1];
      faceColorB[2] = faceColor[2];
      faceExpression = FACE_EXPRESSION_CUSTOM;
    }
    if (
      faceEffect == FACE_EFFECT_OFF
      || faceEffect == FACE_EFFECT_PIXELS
      || faceEffect == FACE_EFFECT_FADE_OFF
      || faceEffect == FACE_EFFECT_STARTUP
    ) {
      setFaceEffectState(FACE_EFFECT_SOLID, faceSpeedMs, true);
    } else {
      faceEnabled = true;
      faceFrameDirty = true;
      faceEffectStartMs = millis();
    }
    if (preserveExpression) {
      faceExpression = expression;
    }
    noteHostFaceMutation();
    Serial.print(F("OK LED COLOR "));
    Serial.print(faceColor[0]);
    Serial.print(' ');
    Serial.print(faceColor[1]);
    Serial.print(' ');
    Serial.println(faceColor[2]);
    return true;
  }

  if (strcmp(subcommand, "COLOR_B") == 0) {
    long values[3];
    for (uint8_t component = 0; component < 3; ++component) {
      if (!parseLongStrict(nextToken(cursor), &values[component])) {
        Serial.println(F("ERR LED BAD_VALUE"));
        return false;
      }
      values[component] = clampLong(values[component], 0, 255);
    }
    if (!ensureNoExtraTokens(cursor)) {
      Serial.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    faceColorB[0] = (uint8_t)values[0];
    faceColorB[1] = (uint8_t)values[1];
    faceColorB[2] = (uint8_t)values[2];
    // COLOR_B only tunes the secondary color. It deliberately preserves the
    // active expression, effect, and enabled/off state.
    faceFrameDirty = true;
    noteHostFaceMutation();
    Serial.print(F("OK LED COLOR_B "));
    Serial.print(faceColorB[0]);
    Serial.print(' ');
    Serial.print(faceColorB[1]);
    Serial.print(' ');
    Serial.println(faceColorB[2]);
    return true;
  }

  if (strcmp(subcommand, "BRIGHTNESS") == 0) {
    long value = 0;
    if (!parseLongStrict(nextToken(cursor), &value)) {
      Serial.println(F("ERR LED BAD_VALUE"));
      return false;
    }
    if (!ensureNoExtraTokens(cursor)) {
      Serial.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    faceBrightness = (uint8_t)clampLong(value, 0, 255);
    faceFrameDirty = true;
    noteHostFaceMutation();
    Serial.print(F("OK LED BRIGHTNESS "));
    Serial.print(faceBrightness);
    Serial.print(F(" EFFECTIVE="));
    Serial.println(effectiveFaceBrightness());
    return true;
  }

  if (strcmp(subcommand, "EFFECT") == 0) {
    FaceEffect requestedEffect;
    char *name = nextToken(cursor);
    if (!parseFaceEffect(name, &requestedEffect)) {
      Serial.println(F("ERR LED BAD_EFFECT"));
      return false;
    }
    if (!ensureNoExtraTokens(cursor)) {
      Serial.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    if (requestedEffect == FACE_EFFECT_OFF) {
      turnFaceOff(FACE_EXPRESSION_CUSTOM);
    } else {
      FaceExpression expression = faceExpression;
      bool preserveExpression = expression != FACE_EXPRESSION_CUSTOM
        && expression != FACE_EXPRESSION_STARTUP
        && expression != FACE_EXPRESSION_SHUTDOWN;
      FaceEffect activeEffect = preservePresetSequenceEffect(requestedEffect);
      setFaceEffectState(activeEffect, faceSpeedMs, true);
      // A public effect can tune an active preset without destroying its
      // semantic identity. Pixel/CLEAR/OFF remain explicit custom controls.
      faceExpression = preserveExpression
        ? expression
        : FACE_EXPRESSION_CUSTOM;
    }
    noteHostFaceMutation();
    Serial.print(F("OK LED EFFECT "));
    // Echo the requested public effect so the bridge can acknowledge its
    // in-flight command. STATUS separately exposes the resolved internal one.
    Serial.println(name);
    return true;
  }

  if (strcmp(subcommand, "SPEED") == 0) {
    long value = 0;
    if (!parseLongStrict(nextToken(cursor), &value)) {
      Serial.println(F("ERR LED BAD_VALUE"));
      return false;
    }
    if (!ensureNoExtraTokens(cursor)) {
      Serial.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    faceSpeedMs = (uint16_t)clampLong(
      value,
      MIN_FACE_SPEED_MS,
      MAX_FACE_SPEED_MS
    );
    faceEffectStartMs = millis();
    faceFrameDirty = true;
    noteHostFaceMutation();
    Serial.print(F("OK LED SPEED "));
    Serial.println(faceSpeedMs);
    return true;
  }

  if (strcmp(subcommand, "PIXEL") == 0) {
    long index = 0;
    long values[3];
    if (!parseLongStrict(nextToken(cursor), &index)) {
      Serial.println(F("ERR LED BAD_PIXEL"));
      return false;
    }
    for (uint8_t component = 0; component < 3; ++component) {
      if (!parseLongStrict(nextToken(cursor), &values[component])) {
        Serial.println(F("ERR LED BAD_VALUE"));
        return false;
      }
      values[component] = clampLong(values[component], 0, 255);
    }
    if (!ensureNoExtraTokens(cursor)) {
      Serial.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    index = clampLong(index, 0, NUM_FACE_LEDS - 1);
    if (faceEffect != FACE_EFFECT_PIXELS) {
      clearLogicalFacePixels();
    }
    facePixelColors[index][0] = (uint8_t)values[0];
    facePixelColors[index][1] = (uint8_t)values[1];
    facePixelColors[index][2] = (uint8_t)values[2];
    faceExpression = FACE_EXPRESSION_CUSTOM;
    setFaceEffectState(FACE_EFFECT_PIXELS, faceSpeedMs, true);
    faceExpression = FACE_EXPRESSION_CUSTOM;
    noteHostFaceMutation();
    Serial.print(F("OK LED PIXEL "));
    Serial.print(index);
    Serial.print(' ');
    Serial.print(facePixelColors[index][0]);
    Serial.print(' ');
    Serial.print(facePixelColors[index][1]);
    Serial.print(' ');
    Serial.println(facePixelColors[index][2]);
    return true;
  }

  if (strcmp(subcommand, "CLEAR") == 0) {
    if (!ensureNoExtraTokens(cursor)) {
      Serial.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    clearLogicalFacePixels();
    faceExpression = FACE_EXPRESSION_CUSTOM;
    setFaceEffectState(FACE_EFFECT_PIXELS, faceSpeedMs, true);
    faceExpression = FACE_EXPRESSION_CUSTOM;
    noteHostFaceMutation();
    Serial.println(F("OK LED CLEAR"));
    return true;
  }

  if (strcmp(subcommand, "OFF") == 0) {
    if (!ensureNoExtraTokens(cursor)) {
      Serial.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    turnFaceOff(FACE_EXPRESSION_CUSTOM);
    noteHostFaceMutation();
    Serial.println(F("OK LED OFF"));
    return true;
  }

  if (strcmp(subcommand, "STATUS") == 0) {
    if (!ensureNoExtraTokens(cursor)) {
      Serial.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    Serial.print(F("OK LED STATUS FACE_SUPPORTED=1"));
    printFaceStatusFields();
    printHostSyncFields();
    Serial.println();
    return true;
  }

  Serial.println(F("ERR LED UNKNOWN_COMMAND"));
  return false;
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
    hostPingSeen = true;
    Serial.print(F("OK PONG"));
    printCapabilityFields();
    Serial.println();
    return true;
  }

  if (strcmp(command, "HOST") == 0) {
    char *subcommand = nextToken(&cursor);
    if (subcommand == NULL || strcmp(subcommand, "SYNC") != 0) {
      Serial.println(F("ERR HOST UNKNOWN_COMMAND"));
      return false;
    }
    if (!ensureNoExtraTokens(&cursor)) {
      Serial.println(F("ERR HOST BAD_COUNT"));
      return false;
    }
    if (!hostPingSeen) {
      Serial.println(F("ERR HOST PING_REQUIRED"));
      return false;
    }
    if (!hostSnapshotSeen) {
      Serial.println(F("ERR HOST SNAPSHOT_REQUIRED"));
      return false;
    }
    hostSynced = true;
    if (faceEffect == FACE_EFFECT_STARTUP) {
      applyFacePreset(FACE_EXPRESSION_IDLE);
    }
    Serial.println(F("OK HOST SYNC HOST_SYNCED=1"));
    return true;
  }

  if (strcmp(command, "ARM") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Serial.println(F("ERR BAD_COUNT"));
      return false;
    }
    servoArmed = true;
    lastCommandMs = millis();
    timeoutWarned = false;
    Serial.println(F("OK ARM ARMED=1"));
    return true;
  }

  if (strcmp(command, "HOLD") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Serial.println(F("ERR BAD_COUNT"));
      return false;
    }
    // Keep generating the last requested PCA9685 pulses, but require a new
    // acknowledged ARM before accepting FRAME or SERVO commands.
    holdCurrentPosition();
    servoArmed = false;
    timeoutWarned = false;
    Serial.print(F("OK HOLD ARMED=0 OUTPUT="));
    Serial.println(outputEnabled ? 1 : 0);
    return true;
  }

  if (strcmp(command, "DISARM") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Serial.println(F("ERR BAD_COUNT"));
      return false;
    }
    holdCurrentPosition();
    servoArmed = false;
    timeoutWarned = false;
    if (FACE_OFF_ON_DISARM) {
      turnFaceOff(FACE_EXPRESSION_CUSTOM);
    }
    Serial.print(F("OK DISARM ARMED=0 OUTPUT="));
    Serial.println(outputEnabled ? 1 : 0);
    return true;
  }

  if (strcmp(command, "DISABLE") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Serial.println(F("ERR BAD_COUNT"));
      return false;
    }
    servoArmed = false;
    disableOutputs();
    timeoutWarned = false;
    if (FACE_OFF_ON_DISARM) {
      turnFaceOff(FACE_EXPRESSION_CUSTOM);
    }
    Serial.println(F("OK DISABLE ARMED=0 OUTPUT=0"));
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

  if (strcmp(command, "LED") == 0) {
    return handleLed(&cursor);
  }

  if (strcmp(command, "FACE") == 0) {
    FaceExpression expression;
    char *name = nextToken(&cursor);
    if (!parseFaceExpression(name, &expression)) {
      Serial.println(F("ERR FACE BAD_EXPRESSION"));
      return false;
    }
    if (!ensureNoExtraTokens(&cursor)) {
      Serial.println(F("ERR FACE BAD_COUNT"));
      return false;
    }
    applyFacePreset(expression);
    noteHostFaceMutation();
    Serial.print(F("OK FACE "));
    printFaceExpressionName(faceExpression);
    Serial.println();
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
  // A binary frame that stalls mid-body means the stream died or a byte was
  // lost; abandon it so the parser cannot wedge waiting for byte 26.
  if (binActive
      && (uint32_t)(micros() - lastSerialByteUs) > BIN_FRAME_STALL_US) {
    binActive = false;
    binCrcFailCount++;
  }
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    lastSerialByteUs = micros();
    if (binActive) {
      binBuffer[binLength++] = (uint8_t)c;
      if (binLength >= BIN_FRAME_BODY_LEN) {
        processBinaryFrame();
      }
      continue;
    }
    // The magic byte is only a frame start at line-idle; inside an ASCII line
    // it is ordinary (if invalid) text and the line parser rejects it.
    if (lineLength == 0
        && !discardLineUntilNewline
        && (uint8_t)c == BIN_FRAME_MAGIC) {
      binActive = true;
      binLength = 0;
      continue;
    }
    if (discardLineUntilNewline) {
      if (c == '\n') {
        discardLineUntilNewline = false;
        lineLength = 0;
      }
      continue;
    }
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
      discardLineUntilNewline = true;
      Serial.println(F("ERR LINE_TOO_LONG"));
    }
  }
}

bool serialIdleForFace() {
  return lineLength == 0
    && !discardLineUntilNewline
    && !binActive
    && Serial.available() == 0
    && (uint32_t)(micros() - lastSerialByteUs)
      >= SERIAL_IDLE_BEFORE_FACE_US;
}

// An idle wire is not enough on its own: show() masks interrupts for ~300 us,
// and at 250000 baud the next FRAME can begin inside that blackout and lose
// six characters.  When a steady host stream exists, only transmit while the
// measured gap to the next expected FRAME still has room to spare.
bool faceWindowSafe() {
  if (frameIntervalUs == 0) {
    // No measured stream (disarmed, idle, or first frames): the wire is quiet
    // and the plain idle test already governs.
    return true;
  }
  uint32_t since = (uint32_t)(micros() - lastFrameParsedUs);
  if (since >= frameIntervalUs) {
    // The next frame is already due; do not start a blackout now.
    return false;
  }
  return (frameIntervalUs - since) >= FACE_SHOW_GUARD_US;
}

void setup() {
  Serial.begin(BAUD_RATE);
  // begin()+show() makes the face dark immediately. Startup animation begins
  // only after PCA9685 initialization and does not block serial or servos.
#if FACE_LEDS_ENABLED
  facePixels.begin();
  facePixels.setBrightness(effectiveFaceBrightness());
  appliedFaceBrightness = effectiveFaceBrightness();
  facePixels.clear();
  facePixels.show();
#endif
  clearLogicalFacePixels();
  Wire.begin();
  pwm.begin();
  // A complete 12-channel update is too slow at the Wire default of 100 kHz
  // and can overflow the Nano's 64-byte UART RX ring. PCA9685 supports 400 kHz.
  Wire.setClock(400000L);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);

  // The register burst in writeAllChannelsBurst() depends on the MODE1
  // auto-increment bit.  Current Adafruit libraries happen to set it inside
  // setPWMFreq(), but that is their implementation detail -- assert it
  // explicitly so a library update cannot silently scramble all 12 outputs.
  Wire.beginTransmission(PCA9685_I2C_ADDRESS);
  Wire.write((uint8_t)PCA9685_MODE1);
  Wire.endTransmission();
  Wire.requestFrom((int)PCA9685_I2C_ADDRESS, 1);
  uint8_t mode1 = Wire.available() ? (uint8_t)Wire.read() : (uint8_t)0x00;
  if ((mode1 & MODE1_AI) == 0) {
    Wire.beginTransmission(PCA9685_I2C_ADDRESS);
    Wire.write((uint8_t)PCA9685_MODE1);
    Wire.write((uint8_t)(mode1 | MODE1_AI));
    Wire.endTransmission();
  }

  for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
    targetDeg[channel] = CHANNEL_SAFE_START_DEG[channel];
    currentDeg[channel] = CHANNEL_SAFE_START_DEG[channel];
    targetValid[channel] = false;
  }

  disableOutputs();
  servoArmed = false;
  lastCommandMs = millis();
  lastUpdateMs = millis();

  Serial.print(F("OK VOLT_PCA9685_READY"));
  printCapabilityFields();
  Serial.println(F(" DISARMED OUTPUT_DISABLED"));

  applyFacePreset(FACE_EXPRESSION_STARTUP);
}

void loop() {
  uint32_t loopStartUs = micros();
  readSerialLines();

  if (
    servoArmed
    && millis() - lastCommandMs > COMMAND_TIMEOUT_MS
    && !timeoutWarned
  ) {
    holdCurrentPosition();
    servoArmed = false;
    timeoutWarned = true;
    Serial.println(F("WARN COMMAND_TIMEOUT HOLDING ARMED=0"));
  }

  // Motion must not be starved by a busy wire.  Requiring Serial.available()==0
  // here meant that at a 28% serial duty cycle the servo update was skipped
  // whenever bytes were in flight, so the 20 ms interpolation tick ran late and
  // irregularly -- the direct cause of jerky walking and laggy emotes.  Only a
  // partially received line is a real reason to wait, because the I2C burst is
  // bounded (12 channels at 400 kHz is ~1.4 ms) and the 64-byte RX ring buffers
  // 2.5 ms at 250000 baud, so a full frame cannot be lost behind it.
  if (lineLength == 0 && !discardLineUntilNewline) {
    updateServos();
    // Face policy differs by regime.  While a host FRAME stream exists, the
    // only deterministic quiet window is the one that just opened: the frame
    // was parsed and its I2C write is done, so the next frame is a full host
    // period away and a ~240 us show() cannot collide with expected bytes.
    // faceWindowSafe()'s cadence *prediction* is kept only for the idle
    // regime, where there is no stream to collide with.
    if (serialIdleForFace()) {
      bool streaming = frameIntervalUs != 0;
      if (streaming ? frameJustApplied : faceWindowSafe()) {
        updateFaceLeds();
      }
    }
  }
  frameJustApplied = false;

  uint32_t loopUs = (uint32_t)(micros() - loopStartUs);
  if (loopSampleSuppressed) {
    loopSampleSuppressed = false;
  } else if (loopUs > maxLoopUs) {
    maxLoopUs = loopUs > 65535UL ? 65535U : (uint16_t)loopUs;
  }
}
