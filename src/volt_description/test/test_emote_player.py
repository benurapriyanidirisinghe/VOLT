#!/usr/bin/env python3

"""Pure compatibility-client tests for ``volt_emote_player.py``."""

import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))

from volt_emote_engine import (  # noqa: E402
    BUILTIN_EMOTES,
    MAX_REPETITIONS,
    MAX_DEPTH_SCALE,
    MAX_SCALE,
    MAX_SPEED,
    MIN_REPETITIONS,
    MIN_SCALE,
    MIN_SPEED,
)
from volt_emote_player import (  # noqa: E402
    EmotePlayerError,
    VoltEmotePlayer,
    clamp_emote_options,
    correlated_emote_result,
    encode_cancel_request,
    encode_keepalive_request,
    encode_start_request,
    resolve_controller_request,
    status_gate_error,
)


class RequestMappingTests(unittest.TestCase):
    def test_legacy_action_and_cartesian_aliases(self):
        expected = {
            "stand_ready": ("action", "stand"),
            "sit": ("action", "sit"),
            "bow": ("emote", "bow"),
            "small_dance": ("emote", "happy_dance"),
            "look_left": ("emote", "look_left"),
            "look_right": ("emote", "look_right"),
        }
        for legacy_name, result in expected.items():
            with self.subTest(legacy_name=legacy_name):
                request = resolve_controller_request(legacy_name)
                self.assertEqual((request.kind, request.name), result)

    def test_canonical_cartesian_name_passes_through(self):
        request = resolve_controller_request("  PUSH_UPS  ")
        self.assertEqual((request.kind, request.name), ("emote", "push_ups"))

    def test_every_builtin_name_reaches_the_cartesian_controller_unchanged(self):
        for name in BUILTIN_EMOTES:
            with self.subTest(emote=name):
                request = resolve_controller_request(name)
                self.assertEqual((request.kind, request.name), ("emote", name))

    def test_custom_file_and_path_selectors_are_rejected(self):
        with self.assertRaisesRegex(EmotePlayerError, "emote_file"):
            resolve_controller_request("bow", "/tmp/custom.yaml")
        for selector in ("custom.yaml", "emotes/bow", r"emotes\bow"):
            with self.subTest(selector=selector):
                with self.assertRaisesRegex(EmotePlayerError, "paths"):
                    resolve_controller_request(selector)


class OptionAndProtocolTests(unittest.TestCase):
    def test_options_are_clamped_to_authoritative_engine_bounds(self):
        low = clamp_emote_options(-20, -3.0, 0.0, -1.0)
        self.assertEqual(low.repetitions, MIN_REPETITIONS)
        self.assertEqual(low.speed, MIN_SPEED)
        self.assertEqual(low.amplitude, MIN_SCALE)
        self.assertEqual(low.depth, MIN_SCALE)

        high = clamp_emote_options(50, 8.0, 9.0, 10.0)
        self.assertEqual(high.repetitions, MAX_REPETITIONS)
        self.assertEqual(high.speed, MAX_SPEED)
        self.assertEqual(high.amplitude, MAX_SCALE)
        self.assertEqual(high.depth, MAX_DEPTH_SCALE)

    def test_nonfinite_options_are_rejected_before_json_encoding(self):
        for value in (math.nan, math.inf, -math.inf, "not-a-number"):
            with self.subTest(value=value):
                with self.assertRaises(EmotePlayerError):
                    clamp_emote_options(1, value, 1.0, 1.0)

    def test_repetitions_use_the_engine_integer_type(self):
        for value in (True, 1.5, "2"):
            with self.subTest(value=value):
                with self.assertRaisesRegex(EmotePlayerError, "integer"):
                    clamp_emote_options(value, 1.0, 1.0, 1.0)

    def test_start_json_has_only_controller_schema_keys(self):
        options = clamp_emote_options(2, 1.25, 0.75, 1.5)
        payload = json.loads(encode_start_request("request-7", "bow", options))
        self.assertEqual(
            set(payload),
            {
                "command",
                "request_id",
                "name",
                "repetitions",
                "speed",
                "amplitude",
                "depth",
            },
        )
        self.assertEqual(payload["command"], "start")
        self.assertEqual(payload["request_id"], "request-7")
        self.assertEqual(payload["name"], "bow")
        self.assertEqual(payload["repetitions"], 2)

    def test_cancel_json_is_minimal_and_correlated(self):
        self.assertEqual(
            json.loads(encode_cancel_request("same-id")),
            {"command": "cancel", "request_id": "same-id"},
        )

    def test_keepalive_json_is_minimal_and_correlated(self):
        self.assertEqual(
            json.loads(encode_keepalive_request("same-id")),
            {"command": "keepalive", "request_id": "same-id"},
        )

    def test_result_requires_matching_request_id(self):
        status = {
            "emote_request_id": "ours",
            "emote_result": "queued",
        }
        self.assertEqual(correlated_emote_result(status, "ours"), "queued")
        self.assertIsNone(correlated_emote_result(status, "theirs"))
        status["emote_result"] = "unrecognized"
        self.assertIsNone(correlated_emote_result(status, "ours"))
        status["emote_result"] = "settling"
        self.assertEqual(correlated_emote_result(status, "ours"), "settling")


class StatusGateTests(unittest.TestCase):
    @staticmethod
    def ready_status():
        return {
            "command_owner": "MOTION",
            "state": "standing",
            "motion_active": False,
            "emote_active": False,
            "emote_pending": False,
            "physical_test_active": False,
            "emotes_available": ["bow", "happy_dance"],
        }

    def test_missing_stale_and_wrong_owner_fail_closed(self):
        self.assertIn("no valid", status_gate_error(None, 0.0, 3.0))
        status = self.ready_status()
        self.assertIn("stale", status_gate_error(status, 3.01, 3.0))
        status["command_owner"] = "HOLD"
        self.assertIn("enable MOTION", status_gate_error(status, 0.0, 3.0))

    def test_cartesian_gate_requires_standing_and_advertised_name(self):
        status = self.ready_status()
        status["state"] = "sitting"
        self.assertIn(
            "require standing",
            status_gate_error(status, 0.1, 3.0, emote_name="bow"),
        )
        status["state"] = "standing"
        self.assertIn(
            "not advertised",
            status_gate_error(status, 0.1, 3.0, emote_name="nod"),
        )

    def test_post_stop_cartesian_gate_requires_idle(self):
        status = self.ready_status()
        self.assertEqual(
            status_gate_error(
                status,
                0.1,
                3.0,
                emote_name="bow",
                require_idle=True,
            ),
            "",
        )
        for field in (
            "motion_active",
            "emote_active",
            "emote_pending",
            "physical_test_active",
        ):
            busy = self.ready_status()
            busy[field] = True
            with self.subTest(field=field):
                self.assertIn(
                    "idle standing",
                    status_gate_error(
                        busy,
                        0.1,
                        3.0,
                        emote_name="bow",
                        require_idle=True,
                    ),
                )

    def test_malformed_status_does_not_refresh_client_cache(self):
        player = VoltEmotePlayer.__new__(VoltEmotePlayer)
        player.last_status = {"old": True}
        player.last_status_time = 1.0
        player.status_generation = 4
        player.monotonic = lambda: 10.0
        VoltEmotePlayer.status_callback(
            player,
            SimpleNamespace(data="not-json"),
        )
        self.assertEqual(player.last_status, {"old": True})
        self.assertEqual(player.last_status_time, 1.0)
        self.assertEqual(player.status_generation, 4)


class CancellationAndSourceTests(unittest.TestCase):
    def test_interrupt_cleanup_sends_correlated_cancel_before_stop(self):
        events = []
        player = VoltEmotePlayer.__new__(VoltEmotePlayer)
        player.request_id = "active-request"
        player.publish_emote_json = lambda encoded: events.append(
            ("emote", json.loads(encoded))
        )
        player.spin_for = lambda duration: events.append(("spin", duration))
        player.publish_zero_stop = lambda: events.append(("stop", None))
        VoltEmotePlayer.cancel_and_stop(player)
        self.assertEqual(
            events[0],
            (
                "emote",
                {"command": "cancel", "request_id": "active-request"},
            ),
        )
        self.assertEqual(events[-1], ("stop", None))

    def test_player_has_no_manual_joint_or_owner_publication_path(self):
        source = (SCRIPTS / "volt_emote_player.py").read_text(encoding="utf-8")
        self.assertNotIn("Float64MultiArray", source)
        self.assertNotIn("/volt/joint_commands", source)
        self.assertNotIn("owner_publisher", source)
        self.assertNotIn('owner.data = "MANUAL"', source)
        self.assertNotIn("import yaml", source)
        self.assertNotIn("time.sleep", source)

    def test_launch_exposes_legacy_and_cartesian_option_arguments(self):
        source = (ROOT / "launch" / "emote_player.launch.py").read_text(
            encoding="utf-8"
        )
        for argument in (
            "emote",
            "emote_file",
            "speed_scale",
            "repetitions",
            "amplitude",
            "depth",
        ):
            self.assertIn('DeclareLaunchArgument(\n            "%s"' % argument, source)


if __name__ == "__main__":
    unittest.main()
