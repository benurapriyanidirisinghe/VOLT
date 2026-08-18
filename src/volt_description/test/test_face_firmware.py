#!/usr/bin/env python3
"""Source-contract tests for the resource-constrained Nano face firmware."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[3]
FIRMWARE = ROOT / "firmware" / "volt_arduino_pca9685" / "volt_arduino_pca9685.ino"
SOURCE = FIRMWARE.read_text(encoding="utf-8")


def function_body(name: str) -> str:
    """Return a C++ function body using brace matching, not a fragile regex."""
    match = re.search(rf"\b{name}\s*\([^)]*\)\s*\{{", SOURCE)
    if match is None:
        raise AssertionError(f"missing firmware function: {name}")
    start = SOURCE.find("{", match.start())
    depth = 0
    for index in range(start, len(SOURCE)):
        if SOURCE[index] == "{":
            depth += 1
        elif SOURCE[index] == "}":
            depth -= 1
            if depth == 0:
                return SOURCE[start + 1 : index]
    raise AssertionError(f"unterminated firmware function: {name}")


class FaceFirmwareContractTest(unittest.TestCase):
    def test_parallel_strip_configuration_and_brightness_limit(self):
        self.assertIn("#include <Adafruit_NeoPixel.h>", SOURCE)
        self.assertRegex(SOURCE, r"const uint8_t LED_PIN\s*=\s*6\s*;")
        self.assertRegex(SOURCE, r"const uint8_t NUM_FACE_LEDS\s*=\s*8\s*;")
        self.assertRegex(
            SOURCE,
            r"const uint8_t DEFAULT_FACE_BRIGHTNESS\s*=\s*80\s*;",
        )
        self.assertIn("NEO_GRB + NEO_KHZ800", SOURCE)
        self.assertIn("effectiveFaceBrightness()", SOURCE)
        self.assertNotIn("Adafruit_NeoPixel facePixelsB", SOURCE)

    def test_baud_and_idle_guard_protect_uart_during_neopixel_show(self):
        # 250000 divides exactly from 16 MHz (UBRR=3, 0.0% error).  The show()
        # blackout is no longer survived by running the wire slowly -- it is
        # survived by only transmitting inside a measured inter-FRAME window,
        # which faceWindowSafe() enforces (asserted below).
        self.assertRegex(SOURCE, r"const uint32_t BAUD_RATE\s*=\s*250000\s*;")
        self.assertRegex(
            SOURCE,
            r"const uint16_t SERIAL_IDLE_BEFORE_FACE_US\s*=\s*1000\s*;",
        )
        reader = function_body("readSerialLines")
        face_idle = function_body("serialIdleForFace")
        loop = function_body("loop")
        self.assertIn("lastSerialByteUs = micros();", reader)
        self.assertIn(
            "micros() - lastSerialByteUs",
            face_idle,
        )
        self.assertIn("Serial.available() == 0", face_idle)
        self.assertLess(loop.index("updateServos();"), loop.index("serialIdleForFace()"))
        self.assertLess(loop.index("serialIdleForFace()"), loop.index("updateFaceLeds();"))
        # An idle wire alone is not enough at 250000 baud: the face may only
        # transmit when the measured gap to the next FRAME still has room.
        self.assertIn("faceWindowSafe()", loop)
        window = function_body("faceWindowSafe")
        self.assertIn("frameIntervalUs", window)
        self.assertIn("FACE_SHOW_GUARD_US", window)
        self.assertRegex(
            SOURCE,
            r"const uint16_t FACE_SHOW_GUARD_US\s*=\s*\d+\s*;",
        )

    def test_public_command_surface_is_present(self):
        for token in (
            '"COLOR"',
            '"COLOR_B"',
            '"BRIGHTNESS"',
            '"EFFECT"',
            '"SPEED"',
            '"PIXEL"',
            '"CLEAR"',
            '"OFF"',
            '"STATUS"',
            '"FACE"',
        ):
            self.assertIn(token, SOURCE)
        self.assertIn('F(" FACE_SUPPORTED=1 LED_COUNT=")', SOURCE)

    def test_every_required_effect_and_expression_is_recognized(self):
        effects = {
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
        }
        expressions = {
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
        }
        effect_parser = function_body("parseFaceEffect")
        expression_parser = function_body("parseFaceExpression")
        for name in effects:
            self.assertIn(f'"{name}"', effect_parser)
        for name in expressions:
            self.assertIn(f'"{name}"', expression_parser)

    def test_animation_loop_is_non_blocking_and_motion_has_priority(self):
        animation = function_body("updateFaceLeds")
        loop = function_body("loop")
        self.assertNotIn("delay(", animation)
        self.assertNotRegex(animation, r"\bwhile\s*\(")
        self.assertNotIn("new ", animation)
        self.assertNotIn("malloc(", animation)
        self.assertIn("millis()", animation)
        self.assertLess(loop.index("readSerialLines();"), loop.index("updateServos();"))
        self.assertLess(loop.index("updateServos();"), loop.index("updateFaceLeds();"))
        # The servo tick is deliberately NOT gated on an idle wire: doing so
        # starved the 20 ms interpolation whenever bytes were in flight.  Only
        # a partially received line may defer it.
        self.assertNotIn("Serial.available() == 0", loop)
        self.assertIn("lineLength == 0 && !discardLineUntilNewline", loop)

    def test_reset_clear_and_non_blocking_loading_until_host_sync(self):
        setup = function_body("setup")
        animation = function_body("updateFaceLeds")
        parser = function_body("parseCommand")
        self.assertLess(setup.index("facePixels.clear();"), setup.index("Wire.begin();"))
        self.assertLess(setup.index("facePixels.show();"), setup.index("Wire.begin();"))
        self.assertIn("applyFacePreset(FACE_EXPRESSION_STARTUP);", setup)
        self.assertNotIn("applyFacePreset(FACE_EXPRESSION_IDLE);", animation)
        self.assertIn('strcmp(command, "HOST") == 0', parser)
        self.assertIn('F("OK HOST SYNC HOST_SYNCED=1")', parser)
        self.assertIn('F("ERR HOST PING_REQUIRED")', parser)
        self.assertIn('F("ERR HOST SNAPSHOT_REQUIRED")', parser)

        # Visual synchronization intentionally remains separate from ARM.
        arm_start = parser.index('strcmp(command, "ARM") == 0')
        arm_end = parser.index('strcmp(command, "HOLD") == 0')
        arm_handler = parser[arm_start:arm_end]
        self.assertNotIn("hostSynced", arm_handler)
        self.assertIn("servoArmed = true;", arm_handler)

    def test_host_sync_status_and_mutation_contract(self):
        capability = function_body("printCapabilityFields")
        host_fields = function_body("printHostSyncFields")
        mutation = function_body("noteHostFaceMutation")
        led_handler = function_body("handleLed")
        parser = function_body("parseCommand")
        self.assertIn("printHostSyncFields();", capability)
        for field in (
            "HOST_SYNC_REQUIRED=1",
            "HOST_PING=",
            "HOST_SNAPSHOT=",
            "HOST_SYNCED=",
        ):
            self.assertIn(field, host_fields)
        self.assertIn("hostSnapshotSeen = true;", mutation)
        self.assertIn("hostSynced = false;", mutation)
        self.assertGreaterEqual(led_handler.count("noteHostFaceMutation();"), 8)
        self.assertIn("hostPingSeen = true;", parser)

    def test_host_snapshot_followups_preserve_preset_semantics(self):
        led_handler = function_body("handleLed")
        aliases = function_body("preservePresetSequenceEffect")
        self.assertIn("if (!preserveExpression)", led_handler)
        self.assertIn("faceColorB[0] = faceColor[0];", led_handler)
        self.assertIn("faceExpression = expression;", led_handler)
        self.assertIn('strcmp(subcommand, "COLOR_B") == 0', led_handler)
        self.assertIn('F("OK LED COLOR_B ")', led_handler)
        self.assertIn("faceColorB[0] = (uint8_t)values[0];", led_handler)
        self.assertIn(
            "FaceEffect activeEffect = preservePresetSequenceEffect(requestedEffect);",
            led_handler,
        )
        self.assertIn("Serial.println(name);", led_handler)
        self.assertNotIn('F(" ACTIVE=")', led_handler)
        self.assertIn("FACE_EXPRESSION_LOVE", aliases)
        self.assertIn("FACE_EFFECT_HEARTBEAT", aliases)
        self.assertIn("FACE_EXPRESSION_SUCCESS", aliases)
        self.assertIn("FACE_EFFECT_SUCCESS", aliases)
        self.assertIn("FACE_EXPRESSION_ERROR", aliases)
        self.assertIn("FACE_EFFECT_ERROR", aliases)


if __name__ == "__main__":
    unittest.main()
