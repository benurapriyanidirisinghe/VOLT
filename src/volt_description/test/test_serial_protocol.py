#!/usr/bin/env python3

import re
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import volt_serial_protocol as protocol
from volt_serial_protocol import (
    _STATUS_FIELD,
    BINARY_FRAME_MAGIC,
    BINARY_PROTOCOL_MIN_VERSION,
    FIRMWARE_COUNTER_FIELDS,
    crc8_maxim,
    format_binary_frame,
    ArduinoProtocolState,
    CRITICAL_STACK_TOPICS,
    EXPECTED_FIRMWARE_ID,
    SUPPORTED_FACE_EXPRESSIONS,
    SUPPORTED_LED_EFFECTS,
    SerialLineBuffer,
    duplicate_stack_topics,
    format_face_command,
    format_frame_command,
    format_led_brightness_command,
    format_led_color_b_command,
    format_led_color_command,
    format_led_effect_command,
    format_led_speed_command,
    guarded_pending_command,
    motion_status_allows_arm,
    status_token,
)


class FaceProtocolTests(unittest.TestCase):
    def test_visual_host_sync_contract_is_parsed_without_gating_servo_stream(self):
        state = ArduinoProtocolState()
        self.assertEqual(
            state.consume_response(
                "OK VOLT_PCA9685_READY FW=VOLT_PCA9685 PROTO=2 "
                "MAX_DPS=120.0 FACE_SUPPORTED=1 LED_COUNT=8 "
                "HOST_SYNC_REQUIRED=1 HOST_PING=0 HOST_SNAPSHOT=0 "
                "HOST_SYNCED=0 DISARMED OUTPUT_DISABLED"
            ),
            "ready",
        )
        self.assertTrue(state.ready)
        self.assertTrue(state.host_sync_required)
        self.assertFalse(state.host_ping_seen)
        self.assertFalse(state.host_snapshot_seen)
        self.assertFalse(state.host_synced)

        state.consume_response(
            "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0 "
            "HOST_SYNC_REQUIRED=1 HOST_PING=1 HOST_SNAPSHOT=0 HOST_SYNCED=0"
        )
        self.assertTrue(state.host_ping_seen)
        state.armed = True
        state.motion_inhibited = False

        self.assertEqual(state.consume_response("OK FACE happy"), "face")
        self.assertTrue(state.host_snapshot_seen)
        self.assertFalse(state.host_synced)
        self.assertTrue(state.can_stream_frames)
        self.assertEqual(
            state.consume_response("OK HOST SYNC HOST_SYNCED=1"),
            "host_sync",
        )
        self.assertTrue(state.host_synced)
        self.assertTrue(state.can_stream_frames)

        # A later visual mutation requires another terminal marker but cannot
        # inhibit a confirmed servo stream.
        self.assertEqual(state.consume_response("OK LED SPEED 500"), "led")
        self.assertFalse(state.host_synced)
        self.assertTrue(state.can_stream_frames)

    def test_host_sync_error_is_recoverable_during_servo_streaming(self):
        state = ArduinoProtocolState(
            ready=True,
            armed=True,
            output_enabled=True,
            motion_inhibited=False,
            host_sync_required=True,
        )
        self.assertEqual(
            state.consume_response("ERR HOST SNAPSHOT_REQUIRED"),
            "recoverable_error",
        )
        self.assertTrue(state.armed)
        self.assertTrue(state.can_stream_frames)

        corrupt_transport = ArduinoProtocolState(
            ready=True,
            armed=True,
            output_enabled=True,
            motion_inhibited=False,
        )
        self.assertEqual(
            corrupt_transport.consume_response("ERR HOST_RX_LINE_TOO_LONG"),
            "error",
        )
        self.assertFalse(corrupt_transport.armed)
        self.assertFalse(corrupt_transport.can_stream_frames)

    def test_every_supported_expression_and_effect_has_a_wire_command(self):
        for expression in SUPPORTED_FACE_EXPRESSIONS:
            with self.subTest(expression=expression):
                self.assertEqual(
                    format_face_command(expression),
                    "FACE %s" % expression,
                )
        for effect in SUPPORTED_LED_EFFECTS:
            with self.subTest(effect=effect):
                self.assertEqual(
                    format_led_effect_command(effect),
                    "LED EFFECT %s" % effect,
                )

    def test_face_and_led_commands_are_validated_and_clamped(self):
        self.assertEqual(format_face_command(" Happy "), "FACE happy")
        self.assertEqual(format_led_color_command(-10, 127.6, 999), "LED COLOR 0 128 255")
        self.assertEqual(
            format_led_color_b_command(300, 63.6, -1),
            "LED COLOR_B 255 64 0",
        )
        self.assertEqual(format_led_brightness_command(999), "LED BRIGHTNESS 255")
        self.assertEqual(format_led_effect_command(" Rainbow "), "LED EFFECT rainbow")
        self.assertEqual(format_led_speed_command(1), "LED SPEED 10")
        self.assertEqual(format_led_speed_command(99999), "LED SPEED 60000")

        for invalid in ("unknown", "", None):
            with self.subTest(expression=invalid), self.assertRaises(ValueError):
                format_face_command(invalid)
        with self.assertRaises(ValueError):
            format_led_effect_command("strobe")
        with self.assertRaises(ValueError):
            format_led_color_command(float("nan"), 0, 0)
        with self.assertRaises(ValueError):
            format_led_color_b_command(0, float("inf"), 0)

    def test_face_status_and_ack_are_tracked_without_changing_servo_state(self):
        state = ArduinoProtocolState()
        state.consume_response(
            "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0 "
            "FACE_SUPPORTED=1 LED_COUNT=8"
        )
        state.armed = True
        state.motion_inhibited = False

        self.assertEqual(state.consume_response("OK FACE happy"), "face")
        self.assertEqual(state.consume_response("OK LED COLOR 12 34 56"), "led")
        self.assertEqual(state.consume_response("OK LED COLOR_B 65 43 21"), "led")
        self.assertEqual(
            state.consume_response("OK LED BRIGHTNESS 80 EFFECTIVE=80"),
            "led",
        )
        self.assertEqual(state.led_brightness, 80)
        self.assertEqual(state.consume_response("OK LED EFFECT pulse"), "led")
        self.assertEqual(state.consume_response("OK LED SPEED 250"), "led")
        self.assertEqual(
            state.consume_response(
                "OK LED STATUS FACE_SUPPORTED=1 LED_ENABLED=1 "
                "LED_COLOR=12,34,56 LED_COLOR_B=65,43,21 "
                "LED_BRIGHTNESS=200 LED_EFFECTIVE_BRIGHTNESS=120 "
                "LED_LIMIT=120 "
                "LED_EFFECT=pulse LED_SPEED_MS=250 FACE=happy"
            ),
            "led_status",
        )

        self.assertTrue(state.face_supported)
        self.assertTrue(state.face_status_seen)
        self.assertEqual(state.face_expression, "happy")
        self.assertEqual(state.led_color, (12, 34, 56))
        self.assertEqual(state.led_color_b, (65, 43, 21))
        self.assertEqual(state.led_brightness, 200)
        self.assertEqual(state.led_effective_brightness, 120)
        self.assertEqual(state.led_brightness_limit, 120)
        self.assertEqual(state.face_led_count, 8)
        self.assertEqual(state.led_effect, "pulse")
        self.assertEqual(state.led_speed_ms, 250)
        self.assertTrue(state.armed)
        self.assertTrue(state.can_stream_frames)

    def test_led_error_is_recoverable_during_servo_streaming(self):
        state = ArduinoProtocolState(
            ready=True,
            armed=True,
            output_enabled=True,
            motion_inhibited=False,
        )
        self.assertEqual(
            state.consume_response("ERR LED BAD_EFFECT"),
            "recoverable_error",
        )
        self.assertEqual(state.led_error, "ERR LED BAD_EFFECT")
        self.assertTrue(state.can_stream_frames)

    def test_ready_banner_reset_invalidates_old_capabilities_and_face_status(self):
        state = ArduinoProtocolState()
        state.consume_response(
            "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0 "
            "FACE_SUPPORTED=1 LED_COUNT=8"
        )
        state.consume_response(
            "OK LED STATUS FACE_SUPPORTED=1 LED_ENABLED=1 "
            "LED_COLOR=0,120,255 LED_BRIGHTNESS=80 "
            "LED_EFFECT=breathe LED_SPEED_MS=3000 FACE=idle"
        )
        self.assertTrue(state.firmware_capability_satisfies(2))
        self.assertTrue(state.face_status_seen)

        state.consume_response(
            "OK VOLT_PCA9685_READY DISARMED OUTPUT_DISABLED"
        )

        self.assertFalse(state.firmware_capability_satisfies(2))
        self.assertFalse(state.face_supported)
        self.assertFalse(state.face_status_seen)
        self.assertEqual(state.face_expression, "")

    def test_malformed_led_ack_is_ignored_atomically(self):
        state = ArduinoProtocolState(ready=True, led_brightness=40)
        self.assertEqual(state.consume_response("OK LED"), "other")
        self.assertEqual(
            state.consume_response("OK LED BRIGHTNESS 80 EFFECTIVE=bad"),
            "other",
        )
        self.assertEqual(state.led_brightness, 40)

    def test_alternate_color_ack_preserves_off_state(self):
        state = ArduinoProtocolState(ready=True, led_enabled=False)
        self.assertEqual(
            state.consume_response("OK LED COLOR_B 255 0 180"),
            "led",
        )
        self.assertEqual(state.led_color_b, (255, 0, 180))
        self.assertFalse(state.led_enabled)

        self.assertEqual(
            state.consume_response("OK LED COLOR 0 120 255"),
            "led",
        )
        self.assertTrue(state.led_enabled)


class SerialLineBufferTests(unittest.TestCase):
    def test_fragmented_arm_ack_is_parsed_only_after_newline(self):
        lines = SerialLineBuffer()
        first, overflowed = lines.feed(b"OK A")
        self.assertEqual(first, [])
        self.assertFalse(overflowed)
        self.assertEqual(lines.pending_bytes, 4)

        complete, overflowed = lines.feed(b"RM ARMED=1\r\n")
        self.assertEqual(complete, ["OK ARM ARMED=1"])
        self.assertFalse(overflowed)
        self.assertEqual(lines.pending_bytes, 0)

        state = ArduinoProtocolState()
        state.consume_response("OK PONG")
        state.note_command_sent("ARM")
        self.assertEqual(state.consume_response(complete[0]), "armed")
        self.assertTrue(state.can_stream_frames)

    def test_arm_ack_survives_every_fragment_boundary(self):
        response = b"OK ARM ARMED=1\r\n"
        for split in range(1, len(response)):
            with self.subTest(split=split):
                lines = SerialLineBuffer()
                self.assertEqual(lines.feed(response[:split])[0], [])
                complete, overflowed = lines.feed(response[split:])
                self.assertEqual(complete, ["OK ARM ARMED=1"])
                self.assertFalse(overflowed)

    def test_multiple_lines_and_partial_tail_are_preserved(self):
        lines = SerialLineBuffer()
        complete, overflowed = lines.feed(
            b"OK PONG\r\nOK STATUS ARMED=1 OUT"
        )
        self.assertEqual(complete, ["OK PONG"])
        self.assertFalse(overflowed)
        complete, overflowed = lines.feed(b"PUT=1 LAST_CMD_MS=20\n")
        self.assertEqual(
            complete,
            ["OK STATUS ARMED=1 OUTPUT=1 LAST_CMD_MS=20"],
        )
        self.assertFalse(overflowed)

    def test_oversize_unterminated_response_is_dropped(self):
        lines = SerialLineBuffer(max_line_bytes=8)
        complete, overflowed = lines.feed(b"123456789")
        self.assertEqual(complete, [])
        self.assertTrue(overflowed)
        self.assertEqual(lines.pending_bytes, 0)


class FrameCommandFormattingTests(unittest.TestCase):
    def test_worst_case_frame_fits_nano_receive_ring(self):
        command = format_frame_command([180.0] * 12)
        self.assertEqual(command, "FRAME " + " ".join(["180"] * 12))
        self.assertLessEqual(len((command + "\n").encode("ascii")), 63)

    def test_frame_uses_compact_whole_degree_values(self):
        command = format_frame_command(
            [0.0, 1.1, 2.9, 10.25, 20.75, 30.0] * 2
        )
        tokens = command.split()
        self.assertEqual(tokens[0], "FRAME")
        self.assertEqual(len(tokens[1:]), 12)
        self.assertTrue(all("." not in token for token in tokens[1:]))

    def test_frame_rejects_wrong_value_count(self):
        with self.assertRaises(ValueError):
            format_frame_command([90.0] * 11)

    def test_frame_rejects_nonfinite_values(self):
        for value in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(value=value):
                frame = [90.0] * 12
                frame[4] = value
                with self.assertRaises(ValueError):
                    format_frame_command(frame)


class ArduinoProtocolStateTests(unittest.TestCase):
    def test_port_open_does_not_imply_firmware_ready(self):
        state = ArduinoProtocolState()
        self.assertFalse(state.ready)
        self.assertFalse(state.can_stream_frames)

    def test_ready_banner_confirms_identity_and_safe_start(self):
        state = ArduinoProtocolState()
        event = state.consume_response(
            "OK VOLT_PCA9685_READY DISARMED OUTPUT_DISABLED"
        )
        self.assertEqual(event, "ready")
        self.assertTrue(state.ready)
        self.assertFalse(state.armed)
        self.assertFalse(state.output_enabled)
        self.assertFalse(state.can_stream_frames)

    def test_pong_is_a_valid_handshake(self):
        state = ArduinoProtocolState()
        self.assertEqual(state.consume_response("OK PONG"), "ready")
        self.assertTrue(state.ready)
        self.assertFalse(state.firmware_capability_satisfies(2))

    def test_current_capability_is_parsed_from_identity_pong_and_status(self):
        state = ArduinoProtocolState()
        self.assertEqual(
            state.consume_response(
                "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0"
            ),
            "ready",
        )
        self.assertEqual(state.firmware_id, EXPECTED_FIRMWARE_ID)
        self.assertEqual(state.protocol_version, 2)
        self.assertEqual(state.max_dps, 120.0)
        self.assertTrue(state.firmware_capability_satisfies(2))
        self.assertFalse(state.firmware_capability_satisfies(3))

        state.consume_response(
            "OK STATUS FW=VOLT_PCA9685 PROTO=3 MAX_DPS=90.0 "
            "ARMED=0 OUTPUT=0 LAST_CMD_MS=10"
        )
        self.assertEqual(state.protocol_version, 3)
        self.assertEqual(state.max_dps, 90.0)
        self.assertTrue(state.firmware_capability_satisfies(3))

    def test_firmware_capability_must_cover_required_motion_velocity(self):
        state = ArduinoProtocolState()
        state.consume_response(
            "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0"
        )

        self.assertTrue(
            state.firmware_capability_satisfies(2, minimum_max_dps=120.0)
        )
        self.assertFalse(
            state.firmware_capability_satisfies(2, minimum_max_dps=120.1)
        )
        for invalid in (True, -1.0, float("nan"), float("inf"), "invalid"):
            with self.subTest(invalid=invalid):
                self.assertFalse(
                    state.firmware_capability_satisfies(
                        2,
                        minimum_max_dps=invalid,
                    )
                )

    def test_current_ready_banner_and_reset_track_capability(self):
        state = ArduinoProtocolState()
        state.consume_response(
            "OK VOLT_PCA9685_READY FW=VOLT_PCA9685 "
            "PROTO=2 MAX_DPS=120.0 DISARMED OUTPUT_DISABLED"
        )
        self.assertTrue(state.firmware_capability_satisfies(2))
        state.reset()
        self.assertEqual(state.firmware_id, "")
        self.assertEqual(state.protocol_version, 0)
        self.assertEqual(state.max_dps, 0.0)
        self.assertFalse(state.firmware_capability_satisfies(2))

    def test_arm_is_pending_until_ack(self):
        state = ArduinoProtocolState()
        state.consume_response("OK PONG")
        state.note_command_sent("ARM")
        self.assertFalse(state.armed)
        self.assertFalse(state.can_stream_frames)
        self.assertEqual(state.pending_command, "ARM")
        self.assertEqual(state.consume_response("OK ARM ARMED=1"), "armed")
        self.assertTrue(state.armed)
        self.assertTrue(state.can_stream_frames)

    def test_untrusted_arm_ack_cannot_unlock_streaming(self):
        state = ArduinoProtocolState()
        self.assertEqual(state.consume_response("OK ARM ARMED=1"), "untrusted")
        self.assertFalse(state.ready)
        self.assertFalse(state.armed)
        self.assertFalse(state.can_stream_frames)

    def test_hold_inhibits_frames_before_ack_and_preserves_output(self):
        state = ArduinoProtocolState()
        state.consume_response("OK PONG")
        state.note_command_sent("ARM")
        state.consume_response("OK ARM ARMED=1")
        state.output_enabled = True
        state.note_command_sent("HOLD")
        self.assertTrue(state.armed)
        self.assertFalse(state.can_stream_frames)
        state.output_enabled = False
        self.assertEqual(state.consume_response("OK HOLD ARMED=0 OUTPUT=1"), "held")
        self.assertFalse(state.armed)
        self.assertTrue(state.output_enabled)
        self.assertFalse(state.can_stream_frames)

    def test_disable_ack_clears_armed_and_output(self):
        state = ArduinoProtocolState(ready=True, armed=True, output_enabled=True)
        state.note_command_sent("DISABLE")
        state.consume_response("OK DISABLE ARMED=0 OUTPUT=0")
        self.assertFalse(state.armed)
        self.assertFalse(state.output_enabled)
        self.assertFalse(state.can_stream_frames)

    def test_status_can_confirm_arm(self):
        state = ArduinoProtocolState()
        state.consume_response("OK PONG")
        state.note_command_sent("ARM")
        state.consume_response("OK STATUS ARMED=1 OUTPUT=1 LAST_CMD_MS=20")
        self.assertTrue(state.armed)
        self.assertTrue(state.output_enabled)
        self.assertEqual(state.pending_command, "")
        self.assertTrue(state.can_stream_frames)

    def test_timeout_forces_safe_hold(self):
        state = ArduinoProtocolState(ready=True, armed=True, output_enabled=True)
        self.assertEqual(
            state.consume_response("WARN COMMAND_TIMEOUT HOLDING ARMED=0"),
            "timeout",
        )
        self.assertFalse(state.armed)
        self.assertTrue(state.output_enabled)
        self.assertFalse(state.can_stream_frames)

    def test_recoverable_parser_error_preserves_pending_safe_stop(self):
        state = ArduinoProtocolState(ready=True, armed=True, output_enabled=True)
        state.note_command_sent("HOLD")
        self.assertEqual(
            state.consume_response("ERR BAD_COUNT"),
            "recoverable_error",
        )
        self.assertTrue(state.armed)
        self.assertEqual(state.pending_command, "HOLD")
        self.assertTrue(state.motion_inhibited)
        self.assertFalse(state.can_stream_frames)

    def test_recoverable_parser_errors_keep_confirmed_stream_live(self):
        recoverable_errors = (
            "ERR BAD_COUNT",
            "ERR BAD_VALUE",
            "ERR BAD_CHANNEL",
            "ERR LINE_TOO_LONG",
            "ERR UNKNOWN_COMMAND",
        )
        for error in recoverable_errors:
            with self.subTest(error=error):
                state = ArduinoProtocolState(
                    ready=True,
                    armed=True,
                    output_enabled=True,
                    motion_inhibited=False,
                )
                self.assertEqual(
                    state.consume_response(error),
                    "recoverable_error",
                )
                self.assertTrue(state.armed)
                self.assertTrue(state.output_enabled)
                self.assertEqual(state.pending_command, "")
                self.assertFalse(state.motion_inhibited)
                self.assertTrue(state.can_stream_frames)
                self.assertEqual(state.last_error, error)

    def test_not_armed_error_is_fatal_to_confirmed_motion_state(self):
        state = ArduinoProtocolState(
            ready=True,
            armed=True,
            output_enabled=True,
            motion_inhibited=False,
        )
        self.assertEqual(state.consume_response("ERR NOT_ARMED"), "error")
        self.assertFalse(state.armed)
        self.assertEqual(state.pending_command, "")
        self.assertTrue(state.motion_inhibited)
        self.assertFalse(state.can_stream_frames)

    def test_status_tokens_are_parseable(self):
        self.assertEqual(status_token("ERR bad value"), "ERR_bad_value")
        self.assertEqual(status_token(""), "-")

    def test_recent_verified_walk_pose_allows_arm(self):
        for state in ("standing", "hold"):
            with self.subTest(state=state):
                self.assertTrue(
                    motion_status_allows_arm(
                        state,
                        False,
                        False,
                        True,
                        0.2,
                        3.0,
                        arm_neutral_ready=True,
                    )
                )

    def test_sitting_and_unverified_standing_never_allow_arm(self):
        self.assertFalse(
            motion_status_allows_arm(
                "sitting", False, False, True, 0.2, 3.0, True
            )
        )
        self.assertFalse(
            motion_status_allows_arm(
                "standing", False, False, True, 0.2, 3.0, False
            )
        )

    def test_transition_pose_blocks_arm(self):
        for state in ("standing_up", "sitting_down", "waiting", "hold"):
            with self.subTest(state=state):
                self.assertFalse(
                    motion_status_allows_arm(
                        state,
                        False,
                        False,
                        True,
                        0.2,
                        3.0,
                    )
                )

    def test_only_certified_neutral_hold_allows_arm(self):
        self.assertTrue(
            motion_status_allows_arm(
                "hold",
                False,
                False,
                True,
                0.2,
                3.0,
                arm_neutral_ready=True,
            )
        )
        self.assertFalse(
            motion_status_allows_arm(
                "hold",
                False,
                False,
                True,
                0.2,
                3.0,
                arm_neutral_ready=False,
            )
        )

    def test_duplicate_stack_topics_reports_only_multiple_publishers(self):
        counts = {topic: 1 for topic in CRITICAL_STACK_TOPICS}
        counts["/cmd_vel"] = 4
        self.assertEqual(duplicate_stack_topics(counts), ())
        counts["/volt/status"] = 2
        counts["/volt/serial_status"] = 3
        self.assertEqual(
            duplicate_stack_topics(counts),
            ("/volt/status", "/volt/serial_status"),
        )

        counts = {topic: 1 for topic in CRITICAL_STACK_TOPICS}
        counts["/joint_command_router/output"] = 2
        counts["/volt/joint_commands/motion"] = 2
        counts["/joint_group_position_controller/commands"] = 2
        self.assertEqual(
            duplicate_stack_topics(counts),
            (
                "/joint_command_router/output",
                "/volt/joint_commands/motion",
                "/joint_group_position_controller/commands",
            ),
        )

    def test_stale_motion_status_blocks_arm(self):
        self.assertFalse(
            motion_status_allows_arm("standing", False, False, True, 3.1, 3.0)
        )

    def test_active_or_disconnected_motion_status_blocks_arm(self):
        self.assertFalse(
            motion_status_allows_arm("standing", True, False, True, 0.1, 3.0)
        )
        self.assertFalse(
            motion_status_allows_arm("standing", False, True, True, 0.1, 3.0)
        )
        self.assertFalse(
            motion_status_allows_arm("standing", False, False, False, 0.1, 3.0)
        )

    def test_pending_arm_retry_becomes_hold_when_motion_is_unsafe(self):
        self.assertEqual(guarded_pending_command("ARM", False), "HOLD")
        self.assertEqual(guarded_pending_command("ARM", True), "ARM")
        self.assertEqual(guarded_pending_command("DISARM", False), "DISARM")


class BinaryFrameTests(unittest.TestCase):
    """The binary FRAME encoding must match the firmware bit for bit."""

    def test_crc8_maxim_reference_vector(self):
        # The canonical Dallas/Maxim check value.  If this fails, the host and
        # firmware disagree on the polynomial and every frame will be dropped.
        self.assertEqual(crc8_maxim(b"123456789"), 0xA1)

    def test_frame_layout_matches_firmware_contract(self):
        frame = format_binary_frame([118.051] * 12, 7)
        self.assertEqual(len(frame), 27)
        self.assertEqual(frame[0], BINARY_FRAME_MAGIC)
        self.assertEqual(frame[1], 7)
        # 118.051 deg -> 11805 centidegrees, little-endian.
        self.assertEqual(frame[2] | (frame[3] << 8), 11805)
        self.assertEqual(crc8_maxim(frame[1:26]), frame[26])

    def test_values_clamp_to_servo_range_and_reject_non_finite(self):
        low = format_binary_frame([-5.0] * 12, 0)
        high = format_binary_frame([200.0] * 12, 0)
        self.assertEqual(low[2] | (low[3] << 8), 0)
        self.assertEqual(high[2] | (high[3] << 8), 18000)
        with self.assertRaises(ValueError):
            format_binary_frame([float("nan")] * 12, 0)
        with self.assertRaises(ValueError):
            format_binary_frame([1.0] * 11, 0)
        with self.assertRaises(ValueError):
            format_binary_frame([1.0] * 12, 256)

    def test_centidegrees_preserve_shoulder_amplitude(self):
        # The ASCII %.0f encoding flattened the 0.31 deg shoulder sweep to a
        # single integer.  The binary path must keep sub-degree structure.
        lo = format_binary_frame([117.90] * 12, 0)
        hi = format_binary_frame([118.21] * 12, 1)
        self.assertNotEqual(lo[2:4], hi[2:4])

    def test_firmware_constants_match_host_contract(self):
        firmware = (
            Path(__file__).resolve().parents[3]
            / "firmware"
            / "volt_arduino_pca9685"
            / "volt_arduino_pca9685.ino"
        ).read_text(encoding="utf-8")
        self.assertIn("const uint16_t PROTOCOL_VERSION = %d;"
                      % BINARY_PROTOCOL_MIN_VERSION, firmware)
        self.assertIn("const uint8_t BIN_FRAME_MAGIC = 0xA5;", firmware)
        self.assertIn("const uint8_t BIN_FRAME_BODY_LEN = 26;", firmware)
        # Same reflected polynomial on both ends.
        self.assertIn("crc ^= 0x8C;", firmware)
        # Silent-drop contract: the binary reject path must never print.
        self.assertNotIn("ERR CRC", firmware)
        # The forwarded counter list now spans two boards with different
        # capabilities: the Nano has no radio, so WIFI_* exists only on the
        # ESP32. The contract is that every counter is emitted by SOME
        # firmware, and that transport-specific ones sit on the right board.
        root = Path(__file__).resolve().parents[3] / "firmware"
        sketches = {
            "arduino": firmware,
            "esp32": (
                root / "volt_esp32_pca9685" / "volt_esp32_pca9685.ino"
            ).read_text(encoding="utf-8"),
        }
        flat = {
            name: text.replace('F(" ', '').replace('")', '')
            for name, text in sketches.items()
        }
        for counter in FIRMWARE_COUNTER_FIELDS:
            emitters = [
                name for name, text in flat.items() if counter + "=" in text
            ]
            self.assertTrue(
                emitters,
                "%s is forwarded to the console but no firmware emits it"
                % counter,
            )
            # Board-specific fields, named explicitly. A suffix rule like
            # endswith("_MAX_US") would also catch LOOP_MAX_US and
            # BUS_MAX_US, which BOTH boards emit -- quietly dropping the
            # guarantee that the cabled board still reports them.
            esp32_only = counter.startswith("WIFI_") or counter in (
                "NET_MAX_US", "READ_MAX_US", "SERVO_MAX_US", "FACE_MAX_US",
            )
            if esp32_only:
                self.assertIn(
                    "esp32", emitters,
                    "%s is an ESP32 field but that firmware does not emit it"
                    % counter,
                )
            else:
                self.assertIn(
                    "arduino", emitters,
                    "%s must still be emitted by the cabled board" % counter,
                )
            # _STATUS_FIELD is r"\b([A-Z_]+)=..." so a digit anywhere in a
            # field name makes it silently unparseable. I2C_MAX_US was lost
            # to exactly that and reported as "?" in the GUI.
            self.assertRegex(counter, r"^[A-Z_]+$")
            self.assertTrue(
                _STATUS_FIELD.findall("%s=1" % counter),
                "%s is not capturable by the status regex" % counter,
            )


if __name__ == "__main__":
    unittest.main()


class FirmwareGuardMirrorTests(unittest.TestCase):
    """The host mirror of the firmware travel guard must not drift."""

    FIRMWARES = (
        ("volt_arduino_pca9685", "volt_arduino_pca9685.ino"),
        ("volt_esp32_pca9685", "volt_esp32_pca9685.ino"),
    )

    @staticmethod
    def _firmware_table(name, sketch=None):
        directory, filename = sketch or (
            "volt_arduino_pca9685", "volt_arduino_pca9685.ino"
        )
        source = (
            Path(__file__).resolve().parents[3]
            / "firmware" / directory / filename
        ).read_text()
        match = re.search(
            r"const float %s\[CHANNEL_COUNT\] = \{(.*?)\};" % name,
            source,
            re.S,
        )
        assert match, "%s not found in the firmware" % name
        return tuple(
            float(value)
            for value in re.findall(r"-?\d+\.?\d*", match.group(1))
        )

    def test_every_firmware_agrees_on_the_travel_guards(self):
        """The guards protect one robot, so they cannot differ per board.

        The ESP32 board replaces the Arduino on the same mechanism. A guard
        that is right on one and wrong on the other is a leg that jams only
        on the transport nobody tested that day.
        """
        root = Path(__file__).resolve().parents[3] / "firmware"
        present = [
            sketch for sketch in self.FIRMWARES
            if (root / sketch[0] / sketch[1]).is_file()
        ]
        self.assertTrue(present, "no firmware sketches found")
        for table in ("CHANNEL_MIN_DEG", "CHANNEL_MAX_DEG",
                      "CHANNEL_SAFE_START_DEG"):
            values = {
                sketch[0]: self._firmware_table(table, sketch)
                for sketch in present
            }
            self.assertEqual(
                1, len(set(values.values())),
                "%s differs between firmwares: %s" % (table, values),
            )

    def test_mirror_matches_the_firmware_tables(self):
        """A silent clamp is only visible if the mirror stays in sync.

        The firmware clamps to these and reports nothing back, so the host
        copy is the only way an operator learns that commanded travel is
        being discarded. If someone edits the .ino and not this table, the
        guard goes invisible again -- which is how the front-right knee
        clip stayed hidden.
        """
        self.assertEqual(
            self._firmware_table("CHANNEL_MIN_DEG"),
            protocol.FIRMWARE_CHANNEL_MIN_DEG,
        )
        self.assertEqual(
            self._firmware_table("CHANNEL_MAX_DEG"),
            protocol.FIRMWARE_CHANNEL_MAX_DEG,
        )

    def test_detects_the_front_right_knee_clip(self):
        """ch2 below 50 deg is the measured trot/run clip at full stride."""
        frame = [90.0] * 12
        frame[2] = 46.5
        clips = protocol.firmware_guard_clips(frame)
        self.assertEqual([(2, 46.5, 50.0)], clips)

    def test_clean_frame_reports_nothing(self):
        frame = [90.0] * 12
        frame[2] = 54.0
        self.assertEqual([], protocol.firmware_guard_clips(frame))
