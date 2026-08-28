// VOLT servo board -- ESP32-S3 DevKitC-1 (N16R8), WiFi transport.
//
// This is the Arduino Nano firmware with one thing changed: the wire. Every
// byte of the protocol, every channel guard, the slew limit, the ARM
// handshake, the command timeout and the STATUS counters are the same code,
// because they are what the host was built and tested against. What moved is
// where those bytes come from -- a TCP socket instead of a UART.
//
// Consequences of that swap, and how they are handled:
//
//   The link can vanish silently. A cable that falls out stops delivering
//   bytes; a WiFi link that drops can leave a socket that looks open. The
//   existing COMMAND_TIMEOUT_MS (750 ms) already disarms on a starved
//   stream, and that is now the primary protection rather than a backstop.
//
//   The host drops frames rather than queueing them (see volt_net_link.py).
//   A burst of stale servo targets arriving after a stall is far worse than
//   a gap, and the sequence counter makes the gap visible.
//
//   Only one client at a time. A second connection would interleave frames
//   with the first and produce a servo stream neither host asked for, so it
//   is refused rather than accepted.
//
// N16R8 PIN WARNING: the octal PSRAM on this module uses GPIO 35, 36 and 37.
// They are NOT free. Using them for I2C or the LED strip produces a board
// that boots, runs, and then fails in ways that look like a wiring fault.

#include <WiFi.h>
#include <ESPmDNS.h>

// ---------------------------------------------------------------- config --
// Set these before flashing. Kept as plain constants rather than a captive
// portal: a robot that can be joined to an arbitrary network by anyone in
// range is not what this is for.
// Several may be listed. The board scans, then joins the STRONGEST one it
// can actually see, rather than the first that happens to be at the top --
// a robot carried between a bench and a floor should not need reflashing,
// and joining a weak AP when a strong one is present is how a 60 Hz stream
// starts stuttering.
struct WifiNetwork {
  const char *ssid;
  const char *password;
};

// Credentials live in wifi_credentials.h, which is gitignored. This
// repository is public: a password committed here is a password published,
// and rewriting history afterwards does not un-publish it. The tracked
// template is wifi_credentials_example.h, and the fallback below keeps a
// fresh clone compiling.
#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#else
#define VOLT_WIFI_NETWORKS {"CHANGE_ME", "CHANGE_ME"},
#warning "wifi_credentials.h not found - copy wifi_credentials_example.h and fill it in"
#endif

const WifiNetwork WIFI_NETWORKS[] = {
  VOLT_WIFI_NETWORKS
};
const uint8_t WIFI_NETWORK_COUNT =
  sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);

// 2.4 GHz only, deliberately: the S3 has no 5 GHz radio, so a 5 GHz-only AP
// is invisible no matter how close it is. The scan report says so rather
// than leaving that as a mystery.
const int8_t WIFI_WEAK_RSSI = -75;   // below this, warn that the link is marginal

// Which network was joined, for STATUS and for the scan report.
char wifiJoinedSsid[33] = "";
int32_t wifiJoinedRssi = 0;

// The host connects to this port. Must match the tcp:// endpoint the bridge
// is given (volt_wifi.launch.py / the VOLT WiFi Robot icon).
const uint16_t VOLT_TCP_PORT = 3333;

// Fixed hostname so the host can use volt-esp32.local instead of chasing a
// DHCP lease.
const char VOLT_HOSTNAME[] = "volt-esp32";

// I2C to the PCA9685. Any free GPIO works; these avoid the PSRAM pins.
const int PIN_I2C_SDA = 8;
const int PIN_I2C_SCL = 9;

// WiFi must not sleep. Power save adds tens of milliseconds of latency to a
// 60 Hz control stream and shows up as a stuttering gait.
const bool WIFI_DISABLE_SLEEP = true;

// Dead-peer detection is TCP KEEPALIVE, not silence.
//
// The obvious approach -- drop a client that has sent nothing for N seconds
// -- is wrong for this protocol, and measurably so. The host only streams
// frames while ARMED, and the bridge's own STATUS poll is gated behind
// protocol.armed too, so a perfectly healthy pre-arm console sends NOTHING,
// indefinitely. A silence timeout dropped it on a loop: connected, dropped,
// reconnected, handshake never finished, connected=1 ready=0 forever.
//
// Keepalive asks the question properly. An idle-but-alive host answers the
// probes in the kernel with no application traffic; a host that died without
// a FIN -- a crashed console, a yanked network, a slept laptop -- answers
// nothing, connected() goes false on its own, and the existing disconnect
// path handles it.
//
// 10 s idle, 3 s interval, 3 probes: a dead peer is gone in ~19 s against
// the minutes TCP takes by default. The servos are safe long before that
// either way -- COMMAND_TIMEOUT_MS disarms at 750 ms.
// Socket option numbers, spelled out rather than pulled in from
// <lwip/sockets.h>. That header shifts where the Arduino preprocessor
// inserts its auto-generated prototypes, which lands them ahead of this
// sketch's own enums and breaks the build in a place with no connection to
// networking. These are lwIP's stable ABI values.
const int VOLT_SOL_SOCKET = 0xfff;
const int VOLT_SO_KEEPALIVE = 0x0008;
const int VOLT_IPPROTO_TCP = 6;
const int VOLT_TCP_KEEPIDLE = 0x03;
const int VOLT_TCP_KEEPINTVL = 0x04;
const int VOLT_TCP_KEEPCNT = 0x05;

const int CLIENT_KEEPALIVE_IDLE_S = 10;
const int CLIENT_KEEPALIVE_INTERVAL_S = 3;
const int CLIENT_KEEPALIVE_COUNT = 3;


WiFiServer voltServer(VOLT_TCP_PORT);
WiFiClient voltClient;

// ------------------------------------------------------------ host link --
// A Stream that reads and writes the connected host, so all 193 existing
// Host.print()/read() sites work unchanged. Writes with no client attached
// are discarded rather than buffered: the board must never accumulate a
// backlog of replies for a host that has gone away.
class HostLink : public Stream {
 public:
  // Reads are BUFFERED, and this is the single most important thing in this
  // class. readSerialLines() is written for a UART: it calls available()
  // once per byte and reads one byte at a time, which on a UART is a
  // register poke. On a socket every one of those calls entered the lwIP
  // stack, so simply ASKING whether a byte was ready cost more than the byte
  // was worth -- readSerialLines() measured 38 ms with NO data pending at
  // all, and 47 ms under load, which by itself blew the 20 ms servo tick.
  //
  // It also corrupted the parser. Between available() saying yes and read()
  // being called, the socket could come back empty; read() then returned -1,
  // the caller cast it to (char) and got 0xFF, and that byte landed in the
  // ASCII line buffer. With lineLength no longer zero, the 0xA5 frame magic
  // stopped being recognised -- and since binary frames contain no newline,
  // the line never terminated and the parser stayed wedged. That is why the
  // board reported FRAMES_BIN=0 while the host sent hundreds of frames and
  // dropped none.
  //
  // One bulk read per refill fixes both: cheap, and available() and read()
  // can no longer disagree.
  int available() override {
    if (rxPos_ < rxUsed_) {
      return (int)(rxUsed_ - rxPos_);
    }
    refill();
    return (int)(rxUsed_ - rxPos_);
  }

  int read() override {
    if (rxPos_ >= rxUsed_) {
      refill();
      if (rxPos_ >= rxUsed_) {
        return -1;
      }
    }
    return rxBuffer_[rxPos_++];
  }

  int peek() override {
    if (rxPos_ >= rxUsed_) {
      refill();
      if (rxPos_ >= rxUsed_) {
        return -1;
      }
    }
    return rxBuffer_[rxPos_];
  }

  void resetInput() {
    rxPos_ = 0;
    rxUsed_ = 0;
  }

  // Discard, do NOT flush. Whatever is sitting here belongs to the host that
  // just went away, and a partial line handed to the next one corrupts its
  // first reply: a leftover "R" turned "OK PONG ..." into "ROK PONG ...",
  // which fails the host's startswith("OK PONG") check. The handshake then
  // never completes, so the console reports the board as unreachable while
  // the board is in fact answering every command perfectly.
  void resetOutput() {
    used_ = 0;
  }

  // Writes are BUFFERED to the end of the line. Print::print(F("...")) hands
  // over one byte at a time, and with TCP_NODELAY set every one of those
  // became its own packet -- a ~300 byte STATUS line turned into ~300 tiny
  // segments, overran the LwIP send buffers, and the reply vanished. PING
  // still worked because it is short, which is what made this look like a
  // STATUS bug rather than a transport one.
  //
  // A UART coalesces naturally; this restores that. One line, one send.
  size_t write(uint8_t value) override {
    if (!voltClient || !voltClient.connected()) return 1;
    if (used_ >= sizeof(buffer_)) {
      pushOut();
    }
    buffer_[used_++] = value;
    if (value == '\n') {
      pushOut();
    }
    return 1;
  }

  size_t write(const uint8_t *data, size_t size) override {
    if (!voltClient || !voltClient.connected()) return size;
    for (size_t index = 0; index < size; ++index) {
      write(data[index]);
    }
    return size;
  }

  void flush() override {
    pushOut();
    if (voltClient && voltClient.connected()) voltClient.flush();
  }

 private:
  void pushOut() {
    if (used_ == 0) return;
    if (voltClient && voltClient.connected()) {
      // Short, bounded retry: a momentarily full socket should not silently
      // truncate a status line, and must not block the servo loop either.
      // Single non-blocking attempt. The delay(1) retry this replaces put
      // up to 4 ms of the control loop into waiting for a socket, which is
      // the wrong trade: a status line is worth less than a servo tick, and
      // the host polls STATUS again anyway.
      voltClient.write(buffer_, used_);
    }
    used_ = 0;
  }

  void refill() {
    rxPos_ = 0;
    rxUsed_ = 0;
    if (!voltClient || !voltClient.connected()) {
      return;
    }
    const int pending = voltClient.available();
    if (pending <= 0) {
      return;
    }
    const size_t want =
      (size_t)pending > sizeof(rxBuffer_) ? sizeof(rxBuffer_) : (size_t)pending;
    const int got = voltClient.read(rxBuffer_, want);
    if (got > 0) {
      rxUsed_ = (size_t)got;
    }
  }

  uint8_t buffer_[512];
  size_t used_ = 0;
  // 1024 holds ~38 frames of backlog, far more than one 60 Hz tick, so a
  // burst after a scheduling hiccup is drained in one or two refills.
  uint8_t rxBuffer_[1024];
  size_t rxPos_ = 0;
  size_t rxUsed_ = 0;
};

HostLink Host;

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
// 100 Hz, not 50. Two wins: the wait between latching a new OFF value and
// the servo seeing a pulse edge halves (0-20 ms -> 0-10 ms), and output
// quantisation halves with it (0.488 -> 0.244 deg per tick).
//
// 100 is the ONLY clean step up. The Adafruit driver computes
// prescale = (uint8_t)((25e6/(f*4096)+0.5)-1); at 50 Hz that is 121 (real
// period 19.988 ms) and at 100 Hz exactly 60 (9.994 ms) -- the same -0.06%
// error, so every calibrated pulse width shifts by at most 0.24 deg, under
// the tick the build already lives with.
//
// Do NOT go to 250 Hz. Prescale rounds to 23, a real period of 3.932 ms
// against 4.0 assumed: -1.7%, which moves every commanded angle down by
// 1.1-4.1 deg. Commanding ch2's guard value of 50.0 would then deliver
// 48.1 deg, so the degree-domain clamp would stop bounding the physical
// angle -- and ch2 is the channel that jammed a leg once already.
const uint16_t SERVO_FREQ_HZ = 100;
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
// 250 ms here, against 20 ms on the UART build, and the difference is the
// transport rather than a relaxed standard.
//
// On a UART a mid-frame gap means a byte was genuinely lost, and 27 bytes at
// 250000 baud take about 1 ms, so 20 ms was already enormous. TCP does not
// lose bytes mid-stream -- it either delivers them in order or the
// connection dies -- so a gap here only means the segment boundary fell
// inside a frame and the rest is still in flight.
//
// That happens constantly: this loop measured 27-48 ms worst case with the
// face LEDs running, which is longer than the old 20 ms window all by
// itself. Every frame split across two segments was therefore abandoned and
// counted as a CRC failure, which is why a perfectly healthy link delivered
// FRAMES_BIN=0 while the host reported zero drops.
//
// A stall that really does mean the link is gone is already handled, and
// handled better, by COMMAND_TIMEOUT_MS disarming at 750 ms.
const uint32_t BIN_FRAME_STALL_US = 250000UL;

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
uint32_t maxLoopUs = 0;      // worst loop() duration since last STATUS
// uint32 deliberately: a uint16 saturates at 65535 us, and this loop was
// measured pinned at exactly that -- which reports "65.5 ms" for anything
// from 66 ms to a minute and hides the size of the problem.
// Per-section worst case, so a slow loop names its own cause instead of
// leaving the operator to guess between the radio, the servos and the LEDs.
uint32_t maxNetUs = 0;       // serviceNetwork()
uint32_t maxReadUs = 0;      // readSerialLines()
uint32_t maxServoUs = 0;     // updateServos() incl. the I2C burst
uint32_t maxFaceUs = 0;      // updateFaceLeds() incl. NeoPixel show()
bool loopSampleSuppressed = false;  // set while a long reply is being printed
uint16_t maxI2cUs = 0;       // worst servo I2C write burst since last STATUS
uint32_t ledShowCount = 0;   // NeoPixel show() transmissions
bool frameJustApplied = false;
// Set when a frame lands, so the next updateServos() runs immediately
// instead of waiting out the remainder of the free-running 20 ms tick.
// That tick is not synchronised with frame arrival, so on average half a
// period was being spent doing nothing with a target already in hand.
bool servoTickDueNow = false;

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
    case FACE_EFFECT_SOLID: Host.print(F("solid")); break;
    case FACE_EFFECT_BREATHE: Host.print(F("breathe")); break;
    case FACE_EFFECT_BLINK: Host.print(F("blink")); break;
    case FACE_EFFECT_PULSE: Host.print(F("pulse")); break;
    case FACE_EFFECT_RAINBOW: Host.print(F("rainbow")); break;
    case FACE_EFFECT_CHASE: Host.print(F("chase")); break;
    case FACE_EFFECT_SCANNER: Host.print(F("scanner")); break;
    case FACE_EFFECT_SPARKLE: Host.print(F("sparkle")); break;
    case FACE_EFFECT_ALTERNATE: Host.print(F("alternate")); break;
    case FACE_EFFECT_LOADING: Host.print(F("loading")); break;
    case FACE_EFFECT_OFF: Host.print(F("off")); break;
    case FACE_EFFECT_PIXELS: Host.print(F("pixels")); break;
    case FACE_EFFECT_HEARTBEAT: Host.print(F("heartbeat")); break;
    case FACE_EFFECT_SUCCESS: Host.print(F("success")); break;
    case FACE_EFFECT_ERROR: Host.print(F("error")); break;
    case FACE_EFFECT_FADE_OFF: Host.print(F("fade_off")); break;
    case FACE_EFFECT_STARTUP: Host.print(F("startup")); break;
  }
}

void printFaceExpressionName(FaceExpression expression) {
  switch (expression) {
    case FACE_EXPRESSION_CUSTOM: Host.print(F("custom")); break;
    case FACE_EXPRESSION_NEUTRAL: Host.print(F("neutral")); break;
    case FACE_EXPRESSION_IDLE: Host.print(F("idle")); break;
    case FACE_EXPRESSION_HAPPY: Host.print(F("happy")); break;
    case FACE_EXPRESSION_EXCITED: Host.print(F("excited")); break;
    case FACE_EXPRESSION_LOVE: Host.print(F("love")); break;
    case FACE_EXPRESSION_SAD: Host.print(F("sad")); break;
    case FACE_EXPRESSION_ANGRY: Host.print(F("angry")); break;
    case FACE_EXPRESSION_ALERT: Host.print(F("alert")); break;
    case FACE_EXPRESSION_THINKING: Host.print(F("thinking")); break;
    case FACE_EXPRESSION_CONFUSED: Host.print(F("confused")); break;
    case FACE_EXPRESSION_SLEEPING: Host.print(F("sleeping")); break;
    case FACE_EXPRESSION_SUCCESS: Host.print(F("success")); break;
    case FACE_EXPRESSION_ERROR: Host.print(F("error")); break;
    case FACE_EXPRESSION_SCARED: Host.print(F("scared")); break;
    case FACE_EXPRESSION_PLAYFUL: Host.print(F("playful")); break;
    case FACE_EXPRESSION_SHUTDOWN: Host.print(F("shutdown")); break;
    case FACE_EXPRESSION_STARTUP: Host.print(F("startup")); break;
  }
}

void printFaceStatusFields() {
  Host.print(F(" LED_ENABLED="));
  Host.print(faceEnabled ? 1 : 0);
  Host.print(F(" LED_COLOR="));
  Host.print(faceColor[0]);
  Host.print(',');
  Host.print(faceColor[1]);
  Host.print(',');
  Host.print(faceColor[2]);
  Host.print(F(" LED_COLOR_B="));
  Host.print(faceColorB[0]);
  Host.print(',');
  Host.print(faceColorB[1]);
  Host.print(',');
  Host.print(faceColorB[2]);
  Host.print(F(" LED_BRIGHTNESS="));
  Host.print(faceBrightness);
  Host.print(F(" LED_EFFECTIVE_BRIGHTNESS="));
  Host.print(effectiveFaceBrightness());
  Host.print(F(" LED_LIMIT="));
  Host.print(FACE_BRIGHTNESS_LIMIT);
  Host.print(F(" LED_EFFECT="));
  printFaceEffectName(faceEffect);
  Host.print(F(" LED_SPEED_MS="));
  Host.print(faceSpeedMs);
  Host.print(F(" FACE="));
  printFaceExpressionName(faceExpression);
}

void printHostSyncFields() {
  Host.print(F(" HOST_SYNC_REQUIRED=1 HOST_PING="));
  Host.print(hostPingSeen ? 1 : 0);
  Host.print(F(" HOST_SNAPSHOT="));
  Host.print(hostSnapshotSeen ? 1 : 0);
  Host.print(F(" HOST_SYNCED="));
  Host.print(hostSynced ? 1 : 0);
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
  // ONE transaction, not two. The split exists because the AVR Wire buffer
  // is 32 bytes and 12 channels need 50; the ESP32 core's I2C_BUFFER_LENGTH
  // is 128, so the whole burst fits. Two transactions also latched the front
  // legs 0.59 ms before the rear ones -- a skew the robot has no reason to
  // carry.
  {
    const uint8_t first = 0;
    Wire.beginTransmission(PCA9685_I2C_ADDRESS);
    Wire.write((uint8_t)(PCA9685_LED0_ON_L + 4 * first));
    for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
      Wire.write((uint8_t)0x00);                    // ON low
      Wire.write((uint8_t)0x00);                    // ON high
      Wire.write((uint8_t)(ticks[channel] & 0xFF)); // OFF low
      Wire.write((uint8_t)(ticks[channel] >> 8));   // OFF high
    }
    Wire.endTransmission();
  }
  for (uint8_t half = 0; half < 0; ++half) {
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
  // The tick is free-running and unsynchronised with frame arrival, so a
  // target that landed just after one tick used to sit unapplied for most
  // of a period. When a frame has just been parsed, run now instead --
  // dt still comes from the real elapsed time, so the slew limiter is
  // unchanged and simply gets a smaller step.
  if (!servoTickDueNow && now - lastUpdateMs < UPDATE_PERIOD_MS) {
    return;
  }
  servoTickDueNow = false;
  if (now == lastUpdateMs) {
    // Two frames inside one millisecond: dt would be zero and maxStep with
    // it, freezing the output. Wait for the clock to move.
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
  Host.print(F(" FW=VOLT_PCA9685 PROTO="));
  Host.print(PROTOCOL_VERSION);
  Host.print(F(" MAX_DPS="));
  Host.print(MAX_DEG_PER_SECOND, 1);
  Host.print(F(" FACE_SUPPORTED=1 LED_COUNT="));
  Host.print(NUM_FACE_LEDS);
  printHostSyncFields();
}

// Free SRAM between the heap break and the current stack top.  The binary
// parser, the counters, and the two NeoPixel strip buffers all share 2 KB;
// if this drops below ~300 bytes under load, stop adding features.
int freeSramBytes() {
  // Reported under the SRAM_FREE key the host already parses, so the link
  // health panel keeps working unchanged. On the S3 this is free heap: there
  // is no AVR-style stack-into-heap collision to watch for, and with 8 MB of
  // PSRAM the interesting number is whether the network stack is leaking.
  return (int)ESP.getFreeHeap();
}

void printStatus() {
  // A STATUS reply is a few hundred bytes; beyond the 64-byte TX ring
  // Host.print blocks at ~40 us/byte, so the loop that prints it is not a
  // representative loop.  Suppress that pass, otherwise LOOP_MAX_US reports
  // the cost of its own reporting (~16 ms) and hides the real worst case.
  loopSampleSuppressed = true;
  Host.print(F("OK STATUS"));
  printCapabilityFields();
  Host.print(F(" ARMED="));
  Host.print(servoArmed ? 1 : 0);
  Host.print(F(" OUTPUT="));
  Host.print(outputEnabled ? 1 : 0);
  Host.print(F(" LAST_CMD_MS="));
  Host.print(millis() - lastCommandMs);
  // Link-health counters (stage-1 acceptance: CRC_FAIL and SEQ_GAP stay 0
  // over 60 s of walking with the face animating).  The two *_MAX_US maxima
  // are per-interval: printing resets them, so each STATUS reports the worst
  // case since the previous STATUS.
  Host.print(F(" FRAMES_ASCII="));
  Host.print(framesRxAscii);
  Host.print(F(" FRAMES_BIN="));
  Host.print(framesRxBin);
  Host.print(F(" CRC_FAIL="));
  Host.print(binCrcFailCount);
  Host.print(F(" SEQ_GAP="));
  Host.print(binSeqGapCount);
  Host.print(F(" LOOP_MAX_US="));
  Host.print(maxLoopUs);
  Host.print(F(" BUS_MAX_US="));
  Host.print(maxI2cUs);
  Host.print(F(" LED_SHOWS="));
  Host.print(ledShowCount);
  Host.print(F(" SRAM_FREE="));
  Host.print(freeSramBytes());
  // Link quality travels with every STATUS. On a cable this had no
  // equivalent; on WiFi it is the number that predicts frame drops, and the
  // host's status parser accepts any [A-Z_]+=value pair, so it reaches the
  // console without a protocol change. RSSI is read live rather than cached:
  // the robot moves, and so does the number.
  // STATUS is space delimited and the host parses it with [A-Z_]+=([^\s]+),
  // so a value containing a space is silently truncated at the first word.
  // "NextGen Starlink 2.4GHz" arrived at the console as "NextGen". Spaces
  // become underscores here; the console shows the substituted name, which
  // is recognisable and, unlike the truncation, not a lie about which
  // network is carrying the servo stream.
  Host.print(F(" WIFI_SSID="));
  if (wifiJoinedSsid[0] == '\0') {
    Host.print('-');
  } else {
    for (const char *cursor = wifiJoinedSsid; *cursor != '\0'; ++cursor) {
      Host.print(*cursor == ' ' ? '_' : *cursor);
    }
  }
  Host.print(F(" WIFI_RSSI="));
  Host.print(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
  Host.print(F(" WIFI_IP="));
  if (WiFi.status() == WL_CONNECTED) {
    Host.print(WiFi.localIP());
  } else {
    Host.print(F("-"));
  }
  Host.print(F(" NET_MAX_US="));
  Host.print(maxNetUs);
  Host.print(F(" READ_MAX_US="));
  Host.print(maxReadUs);
  Host.print(F(" SERVO_MAX_US="));
  Host.print(maxServoUs);
  Host.print(F(" FACE_MAX_US="));
  Host.print(maxFaceUs);
  maxLoopUs = 0;
  maxI2cUs = 0;
  maxNetUs = 0;
  maxReadUs = 0;
  maxServoUs = 0;
  maxFaceUs = 0;
  printFaceStatusFields();
  Host.println();
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
    Host.print(F("ERR "));
    Host.print(reason);
    Host.print(F(" REJECTS="));
    Host.println(consecutiveFrameRejects);
  }

  if (servoArmed && consecutiveFrameRejects >= MAX_CONSECUTIVE_FRAME_REJECTS) {
    holdCurrentPosition();
    servoArmed = false;
    timeoutWarned = true;
    Host.println(F("WARN FRAME_LINK_DEGRADED HOLDING ARMED=0"));
  }
  return false;
}

bool handleFrame(char **cursor) {
  if (!servoArmed) {
    Host.println(F("ERR NOT_ARMED"));
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
    Host.println(F("OK FRAME"));
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
    Host.println(F("ERR NOT_ARMED"));
    return false;
  }

  uint8_t channel = 0;
  float degrees = 0.0;
  if (!parseChannelStrict(nextToken(cursor), &channel)) {
    Host.println(F("ERR BAD_CHANNEL"));
    return false;
  }
  if (!parseFloatStrict(nextToken(cursor), &degrees)) {
    Host.println(F("ERR BAD_VALUE"));
    return false;
  }
  if (!ensureNoExtraTokens(cursor)) {
    Host.println(F("ERR BAD_COUNT"));
    return false;
  }

  enableOutputsForMotion();
  setTargetChannel(channel, degrees);
  lastCommandMs = millis();
  timeoutWarned = false;
  Host.print(F("OK SERVO "));
  Host.print(channel);
  Host.print(F(" "));
  Host.println(targetDeg[channel], 2);
  return true;
}

bool handleLed(char **cursor) {
  char *subcommand = nextToken(cursor);
  if (subcommand == NULL) {
    Host.println(F("ERR LED BAD_COUNT"));
    return false;
  }

  if (strcmp(subcommand, "COLOR") == 0) {
    long values[3];
    for (uint8_t component = 0; component < 3; ++component) {
      if (!parseLongStrict(nextToken(cursor), &values[component])) {
        Host.println(F("ERR LED BAD_VALUE"));
        return false;
      }
      values[component] = clampLong(values[component], 0, 255);
    }
    if (!ensureNoExtraTokens(cursor)) {
      Host.println(F("ERR LED BAD_COUNT"));
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
    Host.print(F("OK LED COLOR "));
    Host.print(faceColor[0]);
    Host.print(' ');
    Host.print(faceColor[1]);
    Host.print(' ');
    Host.println(faceColor[2]);
    return true;
  }

  if (strcmp(subcommand, "COLOR_B") == 0) {
    long values[3];
    for (uint8_t component = 0; component < 3; ++component) {
      if (!parseLongStrict(nextToken(cursor), &values[component])) {
        Host.println(F("ERR LED BAD_VALUE"));
        return false;
      }
      values[component] = clampLong(values[component], 0, 255);
    }
    if (!ensureNoExtraTokens(cursor)) {
      Host.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    faceColorB[0] = (uint8_t)values[0];
    faceColorB[1] = (uint8_t)values[1];
    faceColorB[2] = (uint8_t)values[2];
    // COLOR_B only tunes the secondary color. It deliberately preserves the
    // active expression, effect, and enabled/off state.
    faceFrameDirty = true;
    noteHostFaceMutation();
    Host.print(F("OK LED COLOR_B "));
    Host.print(faceColorB[0]);
    Host.print(' ');
    Host.print(faceColorB[1]);
    Host.print(' ');
    Host.println(faceColorB[2]);
    return true;
  }

  if (strcmp(subcommand, "BRIGHTNESS") == 0) {
    long value = 0;
    if (!parseLongStrict(nextToken(cursor), &value)) {
      Host.println(F("ERR LED BAD_VALUE"));
      return false;
    }
    if (!ensureNoExtraTokens(cursor)) {
      Host.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    faceBrightness = (uint8_t)clampLong(value, 0, 255);
    faceFrameDirty = true;
    noteHostFaceMutation();
    Host.print(F("OK LED BRIGHTNESS "));
    Host.print(faceBrightness);
    Host.print(F(" EFFECTIVE="));
    Host.println(effectiveFaceBrightness());
    return true;
  }

  if (strcmp(subcommand, "EFFECT") == 0) {
    FaceEffect requestedEffect;
    char *name = nextToken(cursor);
    if (!parseFaceEffect(name, &requestedEffect)) {
      Host.println(F("ERR LED BAD_EFFECT"));
      return false;
    }
    if (!ensureNoExtraTokens(cursor)) {
      Host.println(F("ERR LED BAD_COUNT"));
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
    Host.print(F("OK LED EFFECT "));
    // Echo the requested public effect so the bridge can acknowledge its
    // in-flight command. STATUS separately exposes the resolved internal one.
    Host.println(name);
    return true;
  }

  if (strcmp(subcommand, "SPEED") == 0) {
    long value = 0;
    if (!parseLongStrict(nextToken(cursor), &value)) {
      Host.println(F("ERR LED BAD_VALUE"));
      return false;
    }
    if (!ensureNoExtraTokens(cursor)) {
      Host.println(F("ERR LED BAD_COUNT"));
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
    Host.print(F("OK LED SPEED "));
    Host.println(faceSpeedMs);
    return true;
  }

  if (strcmp(subcommand, "PIXEL") == 0) {
    long index = 0;
    long values[3];
    if (!parseLongStrict(nextToken(cursor), &index)) {
      Host.println(F("ERR LED BAD_PIXEL"));
      return false;
    }
    for (uint8_t component = 0; component < 3; ++component) {
      if (!parseLongStrict(nextToken(cursor), &values[component])) {
        Host.println(F("ERR LED BAD_VALUE"));
        return false;
      }
      values[component] = clampLong(values[component], 0, 255);
    }
    if (!ensureNoExtraTokens(cursor)) {
      Host.println(F("ERR LED BAD_COUNT"));
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
    Host.print(F("OK LED PIXEL "));
    Host.print(index);
    Host.print(' ');
    Host.print(facePixelColors[index][0]);
    Host.print(' ');
    Host.print(facePixelColors[index][1]);
    Host.print(' ');
    Host.println(facePixelColors[index][2]);
    return true;
  }

  if (strcmp(subcommand, "CLEAR") == 0) {
    if (!ensureNoExtraTokens(cursor)) {
      Host.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    clearLogicalFacePixels();
    faceExpression = FACE_EXPRESSION_CUSTOM;
    setFaceEffectState(FACE_EFFECT_PIXELS, faceSpeedMs, true);
    faceExpression = FACE_EXPRESSION_CUSTOM;
    noteHostFaceMutation();
    Host.println(F("OK LED CLEAR"));
    return true;
  }

  if (strcmp(subcommand, "OFF") == 0) {
    if (!ensureNoExtraTokens(cursor)) {
      Host.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    turnFaceOff(FACE_EXPRESSION_CUSTOM);
    noteHostFaceMutation();
    Host.println(F("OK LED OFF"));
    return true;
  }

  if (strcmp(subcommand, "STATUS") == 0) {
    if (!ensureNoExtraTokens(cursor)) {
      Host.println(F("ERR LED BAD_COUNT"));
      return false;
    }
    Host.print(F("OK LED STATUS FACE_SUPPORTED=1"));
    printFaceStatusFields();
    printHostSyncFields();
    Host.println();
    return true;
  }

  Host.println(F("ERR LED UNKNOWN_COMMAND"));
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
      Host.println(F("ERR BAD_COUNT"));
      return false;
    }
    hostPingSeen = true;
    Host.print(F("OK PONG"));
    printCapabilityFields();
    Host.println();
    return true;
  }

  if (strcmp(command, "HOST") == 0) {
    char *subcommand = nextToken(&cursor);
    if (subcommand == NULL || strcmp(subcommand, "SYNC") != 0) {
      Host.println(F("ERR HOST UNKNOWN_COMMAND"));
      return false;
    }
    if (!ensureNoExtraTokens(&cursor)) {
      Host.println(F("ERR HOST BAD_COUNT"));
      return false;
    }
    if (!hostPingSeen) {
      Host.println(F("ERR HOST PING_REQUIRED"));
      return false;
    }
    if (!hostSnapshotSeen) {
      Host.println(F("ERR HOST SNAPSHOT_REQUIRED"));
      return false;
    }
    hostSynced = true;
    if (faceEffect == FACE_EFFECT_STARTUP) {
      applyFacePreset(FACE_EXPRESSION_IDLE);
    }
    Host.println(F("OK HOST SYNC HOST_SYNCED=1"));
    return true;
  }

  if (strcmp(command, "ARM") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Host.println(F("ERR BAD_COUNT"));
      return false;
    }
    servoArmed = true;
    lastCommandMs = millis();
    timeoutWarned = false;
    Host.println(F("OK ARM ARMED=1"));
    return true;
  }

  if (strcmp(command, "HOLD") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Host.println(F("ERR BAD_COUNT"));
      return false;
    }
    // Keep generating the last requested PCA9685 pulses, but require a new
    // acknowledged ARM before accepting FRAME or SERVO commands.
    holdCurrentPosition();
    servoArmed = false;
    timeoutWarned = false;
    Host.print(F("OK HOLD ARMED=0 OUTPUT="));
    Host.println(outputEnabled ? 1 : 0);
    return true;
  }

  if (strcmp(command, "DISARM") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Host.println(F("ERR BAD_COUNT"));
      return false;
    }
    holdCurrentPosition();
    servoArmed = false;
    timeoutWarned = false;
    if (FACE_OFF_ON_DISARM) {
      turnFaceOff(FACE_EXPRESSION_CUSTOM);
    }
    Host.print(F("OK DISARM ARMED=0 OUTPUT="));
    Host.println(outputEnabled ? 1 : 0);
    return true;
  }

  if (strcmp(command, "DISABLE") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Host.println(F("ERR BAD_COUNT"));
      return false;
    }
    servoArmed = false;
    disableOutputs();
    timeoutWarned = false;
    if (FACE_OFF_ON_DISARM) {
      turnFaceOff(FACE_EXPRESSION_CUSTOM);
    }
    Host.println(F("OK DISABLE ARMED=0 OUTPUT=0"));
    return true;
  }

  if (strcmp(command, "STATUS") == 0) {
    if (!ensureNoExtraTokens(&cursor)) {
      Host.println(F("ERR BAD_COUNT"));
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
      Host.println(F("ERR FACE BAD_EXPRESSION"));
      return false;
    }
    if (!ensureNoExtraTokens(&cursor)) {
      Host.println(F("ERR FACE BAD_COUNT"));
      return false;
    }
    applyFacePreset(expression);
    noteHostFaceMutation();
    Host.print(F("OK FACE "));
    printFaceExpressionName(faceExpression);
    Host.println();
    return true;
  }

  if (strcmp(command, "FRAME") == 0) {
    return handleFrame(&cursor);
  }

  if (strcmp(command, "SERVO") == 0) {
    return handleServo(&cursor);
  }

  Host.println(F("ERR UNKNOWN_COMMAND"));
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
  while (Host.available() > 0) {
    char c = (char)Host.read();
    lastSerialByteUs = micros();
    // Evidence the host is alive, recorded where the byte is actually taken.
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
      Host.println(F("ERR LINE_TOO_LONG"));
    }
  }
}

bool serialIdleForFace() {
  return lineLength == 0
    && !discardLineUntilNewline
    && !binActive
    && Host.available() == 0
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

// Scanning blocks for seconds. That is fine at boot and during a join
// retry, when nothing is armed, and it is NOT fine while servos are being
// driven -- a stalled loop starves the 60 Hz stream and trips
// COMMAND_TIMEOUT_MS. Every caller must be one of those safe cases.
int scanAndReportNetworks() {
  Serial.println(F("[volt] scanning 2.4 GHz networks..."));
  const int found = WiFi.scanNetworks();
  if (found <= 0) {
    Serial.println(F("[volt] no networks visible."));
    Serial.println(F("[volt] the S3 radio is 2.4 GHz only -- a 5 GHz-only"));
    Serial.println(F("[volt] access point cannot be seen from here."));
    WiFi.scanDelete();
    return found;
  }

  Serial.print(F("[volt] "));
  Serial.print(found);
  Serial.println(F(" network(s) visible:"));
  Serial.println(F("       RSSI  CH  ENC  KNOWN  SSID"));
  for (int index = 0; index < found; ++index) {
    const int32_t rssi = WiFi.RSSI(index);
    Serial.print(F("      "));
    Serial.print(rssi);
    Serial.print(F("  "));
    Serial.print(WiFi.channel(index));
    Serial.print(WiFi.encryptionType(index) == WIFI_AUTH_OPEN
                   ? F("  open ") : F("  wpa  "));
    Serial.print(configuredIndexFor(WiFi.SSID(index).c_str()) >= 0
                   ? F("  yes  ") : F("   no   "));
    Serial.print(WiFi.SSID(index));
    if (rssi < WIFI_WEAK_RSSI) {
      Serial.print(F("   (weak)"));
    }
    Serial.println();
  }
  return found;
}

// Index into WIFI_NETWORKS for an SSID, or -1 when it is not configured.
int configuredIndexFor(const char *ssid) {
  if (ssid == nullptr) {
    return -1;
  }
  for (uint8_t index = 0; index < WIFI_NETWORK_COUNT; ++index) {
    if (strcmp(WIFI_NETWORKS[index].ssid, ssid) == 0) {
      return (int)index;
    }
  }
  return -1;
}

// Pick the configured network with the strongest signal that is actually
// on the air. Returns -1 when none of them are visible, which is a much
// more useful thing to report than a bare join timeout.
int bestVisibleNetwork(int scanCount, int32_t *rssiOut) {
  int bestConfigured = -1;
  int32_t bestRssi = -1000;
  for (int index = 0; index < scanCount; ++index) {
    const int configured = configuredIndexFor(WiFi.SSID(index).c_str());
    if (configured < 0) {
      continue;
    }
    const int32_t rssi = WiFi.RSSI(index);
    if (rssi > bestRssi) {
      bestRssi = rssi;
      bestConfigured = configured;
    }
  }
  if (bestConfigured >= 0 && rssiOut != nullptr) {
    *rssiOut = bestRssi;
  }
  return bestConfigured;
}

bool joinBestNetwork() {
  const int found = scanAndReportNetworks();
  int32_t rssi = 0;
  const int choice = bestVisibleNetwork(found, &rssi);
  WiFi.scanDelete();

  if (choice < 0) {
    Serial.println(F("[volt] none of the configured networks are visible."));
    Serial.println(F("[volt] check WIFI_NETWORKS in this sketch against the"));
    Serial.println(F("[volt] scan above; SSIDs are case sensitive."));
    wifiJoinedSsid[0] = '\0';
    wifiJoinedRssi = 0;
    return false;
  }

  Serial.print(F("[volt] joining "));
  Serial.print(WIFI_NETWORKS[choice].ssid);
  Serial.print(F(" at "));
  Serial.print(rssi);
  Serial.println(F(" dBm"));
  if (rssi < WIFI_WEAK_RSSI) {
    Serial.println(F("[volt] WARNING: weak signal. Expect frame drops and"));
    Serial.println(F("[volt] a 750 ms disarm if the link stalls."));
  }

  WiFi.begin(WIFI_NETWORKS[choice].ssid, WIFI_NETWORKS[choice].password);
  const uint32_t deadline = millis() + 15000UL;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
    delay(200);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[volt] join failed (wrong password?)"));
    wifiJoinedSsid[0] = '\0';
    wifiJoinedRssi = 0;
    return false;
  }
  strncpy(wifiJoinedSsid, WIFI_NETWORKS[choice].ssid,
          sizeof(wifiJoinedSsid) - 1);
  wifiJoinedSsid[sizeof(wifiJoinedSsid) - 1] = '\0';
  wifiJoinedRssi = WiFi.RSSI();
  return true;
}

void startNetwork() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(VOLT_HOSTNAME);
  if (WIFI_DISABLE_SLEEP) {
    // Power save parks the radio between beacons and adds tens of
    // milliseconds of jitter, which a 60 Hz servo stream shows as a stutter.
    WiFi.setSleep(false);
  }

  if (joinBestNetwork()) {
    Serial.print(F("[volt] ip "));
    Serial.print(WiFi.localIP());
    Serial.print(F("  host "));
    Serial.print(VOLT_HOSTNAME);
    Serial.print(F(".local  port "));
    Serial.println(VOLT_TCP_PORT);
    // Without this, volt-esp32.local does not resolve and the desktop icon's
    // default endpoint fails even though the board is up: setHostname() sets
    // the DHCP client name, which is not the same as answering mDNS.
    if (MDNS.begin(VOLT_HOSTNAME)) {
      MDNS.addService("volt", "tcp", VOLT_TCP_PORT);
      Serial.println(F("[volt] mDNS responder started"));
    } else {
      Serial.println(F("[volt] mDNS failed; use the IP above"));
    }
  } else {
    // Not fatal: the loop still runs, the servos hold their safe pose and
    // the USB console still reports. serviceNetwork() keeps retrying.
    Serial.println(F("[volt] not joined; retrying from loop()"));
  }
  voltServer.begin();
  voltServer.setNoDelay(true);
}

void enableClientKeepalive(WiFiClient &client) {
  int enable = 1;
  int idle = CLIENT_KEEPALIVE_IDLE_S;
  int interval = CLIENT_KEEPALIVE_INTERVAL_S;
  int count = CLIENT_KEEPALIVE_COUNT;
  client.setSocketOption(VOLT_SOL_SOCKET, VOLT_SO_KEEPALIVE,
                         &enable, sizeof(enable));
  client.setSocketOption(VOLT_IPPROTO_TCP, VOLT_TCP_KEEPIDLE,
                         &idle, sizeof(idle));
  client.setSocketOption(VOLT_IPPROTO_TCP, VOLT_TCP_KEEPINTVL,
                         &interval, sizeof(interval));
  client.setSocketOption(VOLT_IPPROTO_TCP, VOLT_TCP_KEEPCNT,
                         &count, sizeof(count));
}

void serviceNetwork() {
  if (WiFi.status() != WL_CONNECTED) {
    if (voltClient) {
      voltClient.stop();
      onHostDisconnected();
    }
    // Rescan rather than retrying one remembered SSID: the network may have
    // changed channel, or a second configured AP may now be the strong one.
    // Safe to block here -- reaching this branch means there is no host, so
    // nothing is armed and no servo stream is being starved.
    static uint32_t lastRetryMs = 0;
    if (millis() - lastRetryMs > 8000UL) {
      lastRetryMs = millis();
      WiFi.disconnect();
      joinBestNetwork();
    }
    return;
  }

  if (voltClient && !voltClient.connected()) {
    voltClient.stop();
    onHostDisconnected();
  }



  if (!voltClient || !voltClient.connected()) {
    WiFiClient incoming = voltServer.accept();
    if (incoming) {
      voltClient = incoming;
      voltClient.setNoDelay(true);
      // Zero send timeout. NetworkClient::write() otherwise loops around a
      // select() with a one-second timeout, so a momentarily full socket
      // could stall loop() for seconds -- far past the 750 ms disarm, with
      // the servos held at their last target the whole time.
      voltClient.setTimeout(0);
      // A fresh host has not armed anything yet, and must not inherit the
      // previous session's arm state.
      onHostDisconnected();
      Serial.print(F("[volt] host connected from "));
      Serial.println(voltClient.remoteIP());
    }
  } else {
    // One host only. A second stream would interleave frames with the first
    // and drive the servos with a mixture neither host asked for.
    WiFiClient extra = voltServer.accept();
    if (extra) {
      extra.println(F("ERR BUSY"));
      extra.stop();
    }
  }
}

void onHostDisconnected() {
  // Losing the host is exactly the condition COMMAND_TIMEOUT_MS covers, but
  // acting immediately is better than waiting 750 ms for a stream that is
  // provably gone.
  if (servoArmed) {
    holdCurrentPosition();
  }
  servoArmed = false;
  timeoutWarned = false;
  // Every parser buffer must be emptied, not just the binary one. A session
  // that ended mid-frame or mid-line leaves bytes here, and on a UART that
  // never mattered because the peer could not change without a board reset.
  // On a socket it does: the next host's first command gets those leftovers
  // prepended and comes back ERR UNKNOWN_COMMAND, which looks like a
  // protocol mismatch rather than a stale buffer.
  binActive = false;
  binLength = 0;
  binHaveLastSeq = false;
  lineLength = 0;
  discardLineUntilNewline = false;
  Host.resetInput();
  Host.resetOutput();
  hostPingSeen = false;
  hostSnapshotSeen = false;
  hostSynced = false;
}

void setup() {
  Serial.begin(115200);
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
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
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

  Host.print(F("OK VOLT_PCA9685_READY"));
  printCapabilityFields();
  Host.println(F(" DISARMED OUTPUT_DISABLED"));

  applyFacePreset(FACE_EXPRESSION_STARTUP);

  // WiFi comes up LAST, deliberately. Everything above puts the servos in a
  // known safe pose with outputs disabled; only then is the board reachable.
  // A board that accepted a frame before its channel guards were loaded
  // would be a board that can be told to break itself during boot.
  startNetwork();
}

void loop() {
  uint32_t loopStartUs = micros();
  uint32_t sectionUs = micros();
  serviceNetwork();
  { uint32_t d = micros() - sectionUs; if (d > maxNetUs) maxNetUs = d; }
  sectionUs = micros();
  readSerialLines();
  {
    uint32_t d = micros() - sectionUs;
    // Exclude the pass that printed a long reply. readSerialLines() calls
    // parseCommand(), which for STATUS writes ~500 bytes to the socket, and
    // charging that to the frame-read path reported 38 ms for work that
    // costs microseconds -- the same trap LOOP_MAX_US already dodges with
    // loopSampleSuppressed.
    if (!loopSampleSuppressed && d > maxReadUs) maxReadUs = d;
  }

  if (
    servoArmed
    && millis() - lastCommandMs > COMMAND_TIMEOUT_MS
    && !timeoutWarned
  ) {
    holdCurrentPosition();
    servoArmed = false;
    timeoutWarned = true;
    Host.println(F("WARN COMMAND_TIMEOUT HOLDING ARMED=0"));
  }

  // Motion must not be starved by a busy wire.  Requiring Host.available()==0
  // here meant that at a 28% serial duty cycle the servo update was skipped
  // whenever bytes were in flight, so the 20 ms interpolation tick ran late and
  // irregularly -- the direct cause of jerky walking and laggy emotes.  Only a
  // partially received line is a real reason to wait, because the I2C burst is
  // bounded (12 channels at 400 kHz is ~1.4 ms) and the 64-byte RX ring buffers
  // 2.5 ms at 250000 baud, so a full frame cannot be lost behind it.
  if (lineLength == 0 && !discardLineUntilNewline) {
    { uint32_t t = micros(); updateServos();
      uint32_t d = micros() - t; if (d > maxServoUs) maxServoUs = d; }
    // Face policy differs by regime.  While a host FRAME stream exists, the
    // only deterministic quiet window is the one that just opened: the frame
    // was parsed and its I2C write is done, so the next frame is a full host
    // period away and a ~240 us show() cannot collide with expected bytes.
    // faceWindowSafe()'s cadence *prediction* is kept only for the idle
    // regime, where there is no stream to collide with.
    if (serialIdleForFace()) {
      bool streaming = frameIntervalUs != 0;
      if (streaming ? frameJustApplied : faceWindowSafe()) {
        { uint32_t t = micros(); updateFaceLeds();
          uint32_t d = micros() - t; if (d > maxFaceUs) maxFaceUs = d; }
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
