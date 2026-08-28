#!/usr/bin/env python3

"""Pure state tracking for the VOLT Arduino serial protocol.

The ROS bridge owns transport and timing.  This module owns only confirmed
firmware state, which keeps safety-critical parsing directly unit testable.
"""

from dataclasses import dataclass, field
import math
import re


SAFE_STOP_COMMANDS = ("HOLD", "DISARM", "DISABLE")
EXPECTED_FIRMWARE_ID = "VOLT_PCA9685"
SUPPORTED_LED_EFFECTS = (
    "solid",
    "breathe",
    "blink",
    "pulse",
    "rainbow",
    "chase",
    "scanner",
    "sparkle",
    "alternate",
    "loading",
    "off",
)
SUPPORTED_FACE_EXPRESSIONS = (
    "neutral",
    "idle",
    "happy",
    "excited",
    "love",
    "sad",
    "angry",
    "alert",
    "thinking",
    "confused",
    "sleeping",
    "success",
    "error",
    "scared",
    "playful",
    "shutdown",
)
MIN_LED_SPEED_MS = 10
MAX_LED_SPEED_MS = 60000
RECOVERABLE_PARSE_ERRORS = (
    "ERR BAD_COUNT",
    "ERR BAD_VALUE",
    "ERR BAD_CHANNEL",
    "ERR LINE_TOO_LONG",
    "ERR UNKNOWN_COMMAND",
    # Face commands are independent of actuator state. A rejected visual
    # command must never tear down a confirmed servo stream.
    "ERR LED",
    "ERR FACE",
    "ERR HOST PING_REQUIRED",
    "ERR HOST SNAPSHOT_REQUIRED",
)
ARMABLE_MOTION_STATES = ("standing", "hold")
CRITICAL_STACK_TOPICS = (
    "/volt/status",
    "/volt/command_router_status",
    "/volt/serial_status",
    # These topics carry actuator authority. More than one publisher means a
    # second controller/router can bypass the ownership status interlock.
    "/joint_command_router/output",
    "/volt/joint_commands/motion",
    "/joint_group_position_controller/commands",
)
_STATUS_FIELD = re.compile(r"\b([A-Z_]+)=([^\s]+)")


class SerialLineBuffer:
    """Reassemble timeout-fragmented serial bytes into complete text lines."""

    def __init__(self, max_line_bytes=512):
        self.max_line_bytes = max(1, int(max_line_bytes))
        self._buffer = bytearray()

    @property
    def pending_bytes(self):
        return len(self._buffer)

    def reset(self):
        self._buffer.clear()

    def feed(self, data):
        """Return (complete_lines, overflowed), retaining any incomplete tail."""
        if data:
            self._buffer.extend(bytes(data))

        lines = []
        overflowed = False
        while True:
            newline = self._buffer.find(b"\n")
            if newline < 0:
                break
            raw_line = bytes(self._buffer[:newline])
            del self._buffer[:newline + 1]
            if len(raw_line) > self.max_line_bytes:
                overflowed = True
                continue
            line = raw_line.rstrip(b"\r").decode("ascii", errors="replace").strip()
            if line:
                lines.append(line)

        if len(self._buffer) > self.max_line_bytes:
            # The firmware caps commands at 191 bytes. A longer unterminated
            # response is corrupt and must never be interpreted as an ACK.
            self._buffer.clear()
            overflowed = True

        return lines, overflowed


def status_token(value):
    """Return one whitespace-free value for the bridge key=value status line."""
    text = str(value or "").strip()
    if not text:
        return "-"
    return re.sub(r"\s+", "_", text)


def format_frame_command(values):
    """Encode one Nano-safe FRAME that fits inside its 64-byte RX ring."""
    if len(values) != 12:
        raise ValueError("FRAME requires exactly 12 channel values")
    tokens = []
    for value in values:
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("FRAME values must be finite")
        tokens.append("%.0f" % value)
    command = "FRAME " + " ".join(tokens)
    if len((command + "\n").encode("ascii")) > 63:
        raise ValueError("FRAME exceeds the Nano's safe receive size")
    return command


# ---- Binary FRAME path (firmware PROTO >= 3) -------------------------------
# Wire layout: [0xA5 magic][seq][12 x uint16 LE centidegrees][crc8], 27 bytes
# against ~47 for the ASCII form.  Centidegrees end the whole-degree
# quantisation the ASCII %.0f encoding imposed -- the four shoulder channels
# command ~0.31 deg of amplitude and were flattened to a constant integer by
# it.  The CRC makes wire corruption detectable instead of parseable-garbage,
# and the firmware drops a bad frame silently and counts it (CRC_FAIL in
# STATUS), because printing an error per corrupt frame is itself the timing
# hazard that corrupts the next frame.

# Host-side mirror of the firmware's per-channel travel guards
# (CHANNEL_MIN_DEG / CHANNEL_MAX_DEG in volt_arduino_pca9685.ino).
#
# The firmware clamps silently -- it sends nothing back to say a channel was
# truncated -- so without this mirror a guard that eats commanded motion is
# invisible to the operator. That is how the front-right knee guard on ch2
# went unnoticed: at any stride over ~61 mm it holds that one knee at 50 deg
# while the gait asks for 46.5, so the front-right foot lands 3.5 deg short
# on every stride and no software layer says a word.
#
# test_serial_protocol.py parses the .ino and asserts these match, so the
# mirror cannot drift from the firmware it mirrors.
FIRMWARE_CHANNEL_MIN_DEG = (
    70.0, 0.0, 50.0,
    70.0, 0.0, 30.0,
    50.0, 0.0, 30.0,
    50.0, 0.0, 0.0,
)
FIRMWARE_CHANNEL_MAX_DEG = (
    160.0, 180.0, 150.0,
    160.0, 180.0, 130.0,
    140.0, 180.0, 180.0,
    140.0, 180.0, 150.0,
)


def firmware_guard_clips(frame, tolerance=0.05):
    """Channels this frame would have silently truncated in firmware.

    Returns a list of (channel, commanded_deg, guard_deg). ``tolerance``
    ignores clips smaller than the PCA9685's own 0.488 deg resolution step,
    which are not physically distinguishable anyway.
    """
    clips = []
    for channel, value in enumerate(frame):
        if channel >= len(FIRMWARE_CHANNEL_MIN_DEG):
            break
        low = FIRMWARE_CHANNEL_MIN_DEG[channel]
        high = FIRMWARE_CHANNEL_MAX_DEG[channel]
        if value < low - tolerance:
            clips.append((channel, float(value), low))
        elif value > high + tolerance:
            clips.append((channel, float(value), high))
    return clips


BINARY_FRAME_MAGIC = 0xA5
BINARY_PROTOCOL_MIN_VERSION = 3

# STATUS fields the firmware reports for link health, forwarded verbatim into
# the bridge's status line (lowercased) for the GUI DIAGNOSTICS tab.
# Names must be [A-Z_] only: _STATUS_FIELD is r"\b([A-Z_]+)=..." and silently
# fails to capture any field containing a digit.  I2C_MAX_US was unparseable
# for exactly that reason and reported as "?" in the GUI.
FIRMWARE_COUNTER_FIELDS = (
    "FRAMES_ASCII",
    "FRAMES_BIN",
    "CRC_FAIL",
    "SEQ_GAP",
    "LOOP_MAX_US",
    "BUS_MAX_US",
    "LED_SHOWS",
    "SRAM_FREE",
)


def crc8_maxim(data):
    """CRC-8 Dallas/Maxim (reflected poly 0x8C, init 0x00), bit-for-bit the
    firmware's crc8Maxim().  Reference vector: crc8(b"123456789") == 0xA1."""
    crc = 0x00
    for byte in bytes(data):
        for _ in range(8):
            mix = (crc ^ byte) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            byte >>= 1
    return crc


def format_binary_frame(values, sequence):
    """Encode one binary FRAME for firmware PROTO >= 3."""
    if len(values) != 12:
        raise ValueError("FRAME requires exactly 12 channel values")
    if not isinstance(sequence, int) or not 0 <= sequence <= 255:
        raise ValueError("binary FRAME sequence must be an integer 0..255")
    body = bytearray([sequence])
    for value in values:
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("FRAME values must be finite")
        centideg = int(round(min(max(value, 0.0), 180.0) * 100.0))
        body.append(centideg & 0xFF)
        body.append((centideg >> 8) & 0xFF)
    return bytes([BINARY_FRAME_MAGIC]) + bytes(body) + bytes([crc8_maxim(body)])


def _bounded_integer(value, low, high, field_name):
    """Return a clamped finite integer, rejecting ambiguous input types."""
    if isinstance(value, bool):
        raise ValueError("%s must be numeric" % field_name)
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s must be numeric" % field_name)
    if not math.isfinite(parsed):
        raise ValueError("%s must be finite" % field_name)
    return max(int(low), min(int(high), int(round(parsed))))


def format_face_command(expression):
    """Encode one validated firmware FACE preset command."""
    normalized = str(expression or "").strip().lower()
    if normalized not in SUPPORTED_FACE_EXPRESSIONS:
        raise ValueError("unsupported face expression: %s" % (normalized or "empty"))
    return "FACE %s" % normalized


def format_led_color_command(red, green, blue):
    """Encode an RGB command, clamping each channel to one byte."""
    channels = (
        _bounded_integer(red, 0, 255, "red"),
        _bounded_integer(green, 0, 255, "green"),
        _bounded_integer(blue, 0, 255, "blue"),
    )
    return "LED COLOR %d %d %d" % channels


def format_led_color_b_command(red, green, blue):
    """Encode the secondary RGB color used by alternating face effects."""
    channels = (
        _bounded_integer(red, 0, 255, "alternate red"),
        _bounded_integer(green, 0, 255, "alternate green"),
        _bounded_integer(blue, 0, 255, "alternate blue"),
    )
    return "LED COLOR_B %d %d %d" % channels


def format_led_brightness_command(brightness):
    return "LED BRIGHTNESS %d" % _bounded_integer(
        brightness, 0, 255, "brightness"
    )


def format_led_effect_command(effect):
    normalized = str(effect or "").strip().lower()
    if normalized not in SUPPORTED_LED_EFFECTS:
        raise ValueError("unsupported LED effect: %s" % (normalized or "empty"))
    return "LED EFFECT %s" % normalized


def format_led_speed_command(speed_ms):
    return "LED SPEED %d" % _bounded_integer(
        speed_ms, MIN_LED_SPEED_MS, MAX_LED_SPEED_MS, "LED speed"
    )


def motion_status_allows_arm(
    state,
    moving,
    step_in_place,
    controller_connected,
    age,
    timeout,
    arm_neutral_ready=False,
):
    """Return true only for a recent, connected, stopped stable-pose report."""
    normalized_state = str(state).strip().lower()
    # The firmware wakes at its calibrated WALK_POSE. With no physical joint
    # feedback, a named state alone cannot prove that the cached frame matches
    # that pose. Never ARM from sitting; Sit is entered only after an already
    # armed stream is active.
    stable_pose = (
        normalized_state in ARMABLE_MOTION_STATES
        and bool(arm_neutral_ready)
    )
    return (
        0.0 <= float(age) <= max(0.0, float(timeout))
        and bool(controller_connected)
        and stable_pose
        and not bool(moving)
        and not bool(step_in_place)
    )


def duplicate_stack_topics(publisher_counts):
    """Return critical VOLT topics that have more than one live publisher."""
    counts = dict(publisher_counts or {})
    return tuple(
        topic
        for topic in CRITICAL_STACK_TOPICS
        if int(counts.get(topic, 0)) > 1
    )


def guarded_pending_command(pending_command, motion_safe):
    """Replace an unsafe pending ARM retry with an immediate safe HOLD."""
    pending = str(pending_command or "").strip().upper()
    if pending == "ARM" and not motion_safe:
        return "HOLD"
    return pending


@dataclass
class ArduinoProtocolState:
    """Last state explicitly acknowledged by compatible Arduino firmware."""

    ready: bool = False
    armed: bool = False
    output_enabled: bool = False
    motion_inhibited: bool = True
    pending_command: str = ""
    last_response: str = ""
    last_error: str = ""
    firmware_id: str = ""
    protocol_version: int = 0
    max_dps: float = 0.0
    face_supported: bool = False
    face_status_seen: bool = False
    host_sync_required: bool = False
    host_ping_seen: bool = False
    host_snapshot_seen: bool = False
    host_synced: bool = False
    face_expression: str = ""
    led_enabled: bool = False
    led_color: tuple = ()
    led_color_b: tuple = ()
    led_brightness: int = -1
    led_effective_brightness: int = -1
    led_brightness_limit: int = -1
    face_led_count: int = 0
    led_effect: str = ""
    led_speed_ms: int = 0
    led_error: str = ""
    # Firmware link-health counters from the latest OK STATUS, keyed by the
    # names in FIRMWARE_COUNTER_FIELDS.
    firmware_counters: dict = field(default_factory=dict)

    @property
    def can_stream_frames(self):
        return self.ready and self.armed and not self.motion_inhibited

    def firmware_capability_satisfies(
        self,
        required_protocol_version,
        expected_firmware_id=EXPECTED_FIRMWARE_ID,
        minimum_max_dps=0.0,
    ):
        """Return true only for a complete, explicitly versioned identity."""
        try:
            required = max(0, int(required_protocol_version))
        except (TypeError, ValueError):
            return False
        if isinstance(minimum_max_dps, bool):
            return False
        try:
            minimum_dps = float(minimum_max_dps)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(minimum_dps) or minimum_dps < 0.0:
            return False
        if required == 0 and minimum_dps == 0.0:
            return True
        return (
            self.firmware_id == str(expected_firmware_id)
            and self.protocol_version >= required
            and math.isfinite(self.max_dps)
            and self.max_dps > 0.0
            and self.max_dps + 1e-9 >= minimum_dps
        )

    def reset(self):
        self.ready = False
        self.armed = False
        self.output_enabled = False
        self.motion_inhibited = True
        self.pending_command = ""
        self.last_response = ""
        self.last_error = ""
        self.firmware_id = ""
        self.protocol_version = 0
        self.max_dps = 0.0
        self._reset_face_confirmation()

    def _reset_face_confirmation(self):
        """Forget runtime LED state after a transport or firmware reset."""
        self.face_supported = False
        self.face_status_seen = False
        self.host_sync_required = False
        self.host_ping_seen = False
        self.host_snapshot_seen = False
        self.host_synced = False
        self.face_expression = ""
        self.led_enabled = False
        self.led_color = ()
        self.led_color_b = ()
        self.led_brightness = -1
        self.led_effective_brightness = -1
        self.led_brightness_limit = -1
        self.face_led_count = 0
        self.led_effect = ""
        self.led_speed_ms = 0
        self.led_error = ""

    def note_command_sent(self, command):
        """Record an outgoing state-changing command without assuming success."""
        verb = command.strip().upper().split(maxsplit=1)[0]
        if verb == "ARM":
            self.pending_command = verb
            # ARM never unlocks streaming until OK ARM or STATUS ARMED=1.
            self.motion_inhibited = True
        elif verb in SAFE_STOP_COMMANDS:
            self.pending_command = verb
            # Stop host FRAME traffic immediately, even before the stop ACK.
            self.motion_inhibited = True

    def _status_fields(self, line):
        return {key: value for key, value in _STATUS_FIELD.findall(line)}

    def _consume_output_field(self, line):
        output = self._status_fields(line).get("OUTPUT")
        if output in ("0", "1"):
            self.output_enabled = output == "1"

    def _consume_capability_fields(self, line, inferred_firmware_id=""):
        """Update only capability fields explicitly present in this response."""
        fields = self._status_fields(line)
        firmware_id = fields.get("FW") or fields.get("FIRMWARE_ID")
        if firmware_id:
            self.firmware_id = firmware_id
        elif inferred_firmware_id:
            # The legacy ready banner still carries a trustworthy identity in
            # its fixed prefix, but remains incompatible until it reports a
            # protocol version and MAX_DPS.
            self.firmware_id = inferred_firmware_id

        version = (
            fields.get("PROTO")
            or fields.get("PROTOCOL")
            or fields.get("PROTOCOL_VERSION")
            or fields.get("CAP_VERSION")
        )
        if version is not None:
            try:
                parsed_version = int(version)
                if str(parsed_version) != version or parsed_version < 0:
                    raise ValueError
                self.protocol_version = parsed_version
            except (TypeError, ValueError):
                self.protocol_version = 0

        max_dps = fields.get("MAX_DPS")
        if max_dps is not None:
            try:
                parsed_max_dps = float(max_dps)
                if not math.isfinite(parsed_max_dps) or parsed_max_dps <= 0.0:
                    raise ValueError
                self.max_dps = parsed_max_dps
            except (TypeError, ValueError):
                self.max_dps = 0.0

        face_supported = fields.get("FACE_SUPPORTED")
        if face_supported in ("0", "1"):
            self.face_supported = face_supported == "1"
        led_count = fields.get("LED_COUNT")
        if led_count is not None:
            try:
                parsed_count = int(led_count)
                if str(parsed_count) != led_count or parsed_count <= 0:
                    raise ValueError
                self.face_led_count = parsed_count
            except (TypeError, ValueError):
                self.face_led_count = 0
        self._consume_host_sync_fields(line)

    def _consume_host_sync_fields(self, line):
        """Track the optional visual host-snapshot handshake fields."""
        fields = self._status_fields(line)
        mappings = (
            ("HOST_SYNC_REQUIRED", "host_sync_required"),
            ("HOST_PING", "host_ping_seen"),
            ("HOST_SNAPSHOT", "host_snapshot_seen"),
            ("HOST_SYNCED", "host_synced"),
        )
        for field, attribute in mappings:
            value = fields.get(field)
            if value in ("0", "1"):
                setattr(self, attribute, value == "1")

    def _note_visual_mutation(self):
        """Mirror the firmware contract after an acknowledged visual change."""
        if self.host_sync_required:
            self.host_snapshot_seen = True
            self.host_synced = False

    @staticmethod
    def _parse_rgb(value):
        try:
            parts = tuple(int(part) for part in str(value).split(","))
        except (TypeError, ValueError):
            return ()
        if len(parts) != 3 or any(channel < 0 or channel > 255 for channel in parts):
            return ()
        return parts

    def _consume_face_fields(self, line):
        """Consume face fields shared by STATUS and LED STATUS responses."""
        self._consume_host_sync_fields(line)
        fields = self._status_fields(line)
        support = fields.get("FACE_SUPPORTED")
        if support in ("0", "1"):
            self.face_supported = support == "1"

        seen = False
        enabled = fields.get("LED_ENABLED")
        if enabled in ("0", "1"):
            self.led_enabled = enabled == "1"
            seen = True

        color = self._parse_rgb(fields.get("LED_COLOR"))
        if color:
            self.led_color = color
            seen = True

        color_b = self._parse_rgb(fields.get("LED_COLOR_B"))
        if color_b:
            self.led_color_b = color_b
            seen = True

        brightness = fields.get("LED_BRIGHTNESS")
        if brightness is not None:
            try:
                parsed = int(brightness)
                if str(parsed) != brightness or not 0 <= parsed <= 255:
                    raise ValueError
                self.led_brightness = parsed
                seen = True
            except (TypeError, ValueError):
                pass

        effective_brightness = fields.get("LED_EFFECTIVE_BRIGHTNESS")
        if effective_brightness is not None:
            try:
                parsed = int(effective_brightness)
                if str(parsed) != effective_brightness or not 0 <= parsed <= 255:
                    raise ValueError
                self.led_effective_brightness = parsed
                seen = True
            except (TypeError, ValueError):
                pass

        brightness_limit = fields.get("LED_LIMIT")
        if brightness_limit is not None:
            try:
                parsed = int(brightness_limit)
                if str(parsed) != brightness_limit or not 0 <= parsed <= 255:
                    raise ValueError
                self.led_brightness_limit = parsed
                seen = True
            except (TypeError, ValueError):
                pass

        effect = str(fields.get("LED_EFFECT", "")).strip().lower()
        # Firmware also reports internal preset states such as heartbeat,
        # success, startup, and fade_off. They are status values rather than
        # host-selectable LED EFFECT commands, so preserve any safe token.
        if effect and re.fullmatch(r"[a-z_]+", effect):
            self.led_effect = effect
            seen = True

        speed = fields.get("LED_SPEED_MS")
        if speed is not None:
            try:
                parsed = int(speed)
                if str(parsed) != speed or not MIN_LED_SPEED_MS <= parsed <= MAX_LED_SPEED_MS:
                    raise ValueError
                self.led_speed_ms = parsed
                seen = True
            except (TypeError, ValueError):
                pass

        expression = str(fields.get("FACE", "")).strip().lower()
        if expression:
            self.face_expression = expression
            seen = True

        if seen:
            self.face_supported = True
            self.face_status_seen = True

    def consume_response(self, line):
        """Consume one firmware line and return a short event name."""
        line = str(line).strip()
        if not line:
            return "empty"
        self.last_response = line

        if line.startswith("OK VOLT_PCA9685_READY"):
            # A ready banner can arrive after a Nano reset without the host's
            # serial file descriptor closing. Never retain pre-reset identity,
            # capability, or LED state.
            self.firmware_id = ""
            self.protocol_version = 0
            self.max_dps = 0.0
            self._reset_face_confirmation()
            self._consume_capability_fields(line, EXPECTED_FIRMWARE_ID)
            self._consume_face_fields(line)
            self.ready = True
            self.armed = "ARMED" in line and "DISARMED" not in line
            self.output_enabled = "OUTPUT_DISABLED" not in line
            self.motion_inhibited = not self.armed
            self.pending_command = ""
            self.last_error = ""
            return "ready"

        if line.startswith("OK PONG"):
            self._consume_capability_fields(line)
            self._consume_face_fields(line)
            self.ready = True
            self.last_error = ""
            return "ready"

        # No response other than the identity banner or PONG may establish
        # readiness. This prevents an unrelated serial device from unlocking.
        if line.startswith("OK FACE"):
            if not self.ready:
                return "untrusted"
            tokens = line.split()
            if len(tokens) == 3:
                self.face_expression = tokens[2].strip().lower()
                self.face_supported = True
                self.led_enabled = self.face_expression != "shutdown"
                self._note_visual_mutation()
                self.led_error = ""
                self.last_error = ""
                return "face"
            return "other"

        if line.startswith("OK LED STATUS"):
            if not self.ready:
                return "untrusted"
            self._consume_face_fields(line)
            self.face_supported = True
            return "led_status"

        if line.startswith("OK LED"):
            if not self.ready:
                return "untrusted"
            tokens = line.split()
            event = "led"
            try:
                operation = tokens[2]
                if operation == "COLOR" and len(tokens) == 6:
                    color = tuple(int(value) for value in tokens[3:6])
                    if any(value < 0 or value > 255 for value in color):
                        raise ValueError
                    self.led_color = color
                    self.led_enabled = True
                elif operation == "COLOR_B" and len(tokens) == 6:
                    color = tuple(int(value) for value in tokens[3:6])
                    if any(value < 0 or value > 255 for value in color):
                        raise ValueError
                    self.led_color_b = color
                elif operation == "BRIGHTNESS" and len(tokens) in (4, 5):
                    value = int(tokens[3])
                    if not 0 <= value <= 255:
                        raise ValueError
                    effective_value = self.led_effective_brightness
                    if len(tokens) == 5:
                        key, separator, effective = tokens[4].partition("=")
                        parsed_effective = int(effective)
                        if (
                            key != "EFFECTIVE"
                            or separator != "="
                            or not 0 <= parsed_effective <= 255
                        ):
                            raise ValueError
                        effective_value = parsed_effective
                    self.led_brightness = value
                    self.led_effective_brightness = effective_value
                elif operation == "EFFECT" and len(tokens) == 4:
                    effect = tokens[3].lower()
                    if effect not in SUPPORTED_LED_EFFECTS:
                        raise ValueError
                    self.led_effect = effect
                    self.led_enabled = effect != "off"
                elif operation == "SPEED" and len(tokens) == 4:
                    value = int(tokens[3])
                    if not MIN_LED_SPEED_MS <= value <= MAX_LED_SPEED_MS:
                        raise ValueError
                    self.led_speed_ms = value
                elif operation == "OFF" and len(tokens) == 3:
                    self.led_enabled = False
                    self.led_effect = "off"
                elif operation == "CLEAR" and len(tokens) == 3:
                    self.led_color = (0, 0, 0)
                elif operation == "PIXEL" and len(tokens) == 7:
                    pass
                else:
                    return "other"
            except (IndexError, TypeError, ValueError):
                return "other"
            self.face_supported = True
            self._note_visual_mutation()
            self.led_error = ""
            self.last_error = ""
            return event

        if line.startswith("OK HOST SYNC"):
            if not self.ready:
                return "untrusted"
            fields = self._status_fields(line)
            if fields.get("HOST_SYNCED") != "1":
                return "other"
            self.host_ping_seen = True
            self.host_snapshot_seen = True
            self.host_synced = True
            self.last_error = ""
            return "host_sync"

        if line.startswith("OK ARM"):
            if not self.ready:
                return "untrusted"
            self.armed = True
            if self.pending_command == "ARM":
                self.pending_command = ""
            self.motion_inhibited = bool(
                self.pending_command in SAFE_STOP_COMMANDS
            )
            self.last_error = ""
            return "armed"

        if line.startswith("OK HOLD"):
            if not self.ready:
                return "untrusted"
            self.armed = False
            self._consume_output_field(line)
            self.motion_inhibited = True
            if self.pending_command == "HOLD":
                self.pending_command = ""
            self.last_error = ""
            return "held"

        if line.startswith("OK DISARM"):
            if not self.ready:
                return "untrusted"
            self.armed = False
            self._consume_output_field(line)
            self.motion_inhibited = True
            if self.pending_command == "DISARM":
                self.pending_command = ""
            self.last_error = ""
            return "disarmed"

        if line.startswith("OK DISABLE"):
            if not self.ready:
                return "untrusted"
            self.armed = False
            self._consume_output_field(line)
            self.output_enabled = False
            self.motion_inhibited = True
            if self.pending_command == "DISABLE":
                self.pending_command = ""
            self.last_error = ""
            return "disabled"

        if line.startswith("OK STATUS"):
            if not self.ready:
                return "untrusted"
            self._consume_capability_fields(line)
            self._consume_face_fields(line)
            fields = self._status_fields(line)
            for counter in FIRMWARE_COUNTER_FIELDS:
                if counter in fields:
                    self.firmware_counters[counter] = fields[counter]
            if fields.get("ARMED") in ("0", "1"):
                self.armed = fields["ARMED"] == "1"
            if fields.get("OUTPUT") in ("0", "1"):
                self.output_enabled = fields["OUTPUT"] == "1"

            if self.armed and self.pending_command == "ARM":
                self.pending_command = ""
            elif not self.armed and self.pending_command in SAFE_STOP_COMMANDS:
                self.pending_command = ""

            self.motion_inhibited = (
                not self.armed
                or self.pending_command in SAFE_STOP_COMMANDS
                or self.pending_command == "ARM"
            )
            self.last_error = ""
            return "status"

        if line.startswith("WARN COMMAND_TIMEOUT"):
            self.armed = False
            self.motion_inhibited = True
            self.pending_command = ""
            self.last_error = line
            return "timeout"

        if line.startswith("ERR"):
            self.last_error = line
            if line.startswith(("ERR LED", "ERR FACE")):
                self.led_error = line
            if line.startswith(RECOVERABLE_PARSE_ERRORS):
                # FRAME parsing is atomic in the firmware: these errors reject
                # the bad line without changing its armed state or targets.
                # Keep a confirmed live stream running so the next valid frame
                # arrives before the watchdog. State-changing commands remain
                # pending and inhibited until their normal ACK/retry.
                self.motion_inhibited = (
                    not self.armed or bool(self.pending_command)
                )
                return "recoverable_error"

            self.armed = False
            self.motion_inhibited = True
            self.pending_command = ""
            return "error"

        return "other"
