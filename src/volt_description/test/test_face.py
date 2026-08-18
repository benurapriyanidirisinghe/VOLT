#!/usr/bin/env python3

"""Focused pure tests for face mappings, persistence, and restoration."""

import json
import tempfile
import unittest
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
CONFIG = ROOT / "config" / "face_expressions.yaml"
sys.path.insert(0, str(SCRIPTS))

from volt_face import (  # noqa: E402
    FaceAutomation,
    FaceConfigError,
    FaceSettings,
    SUPPORTED_EFFECTS,
    default_face_config_path,
    default_face_settings,
    default_settings_path,
    load_face_catalog,
    load_face_settings,
    save_face_settings,
    settings_for_preset,
    validate_face_settings,
)


class FaceCatalogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog = load_face_catalog(CONFIG)

    def test_default_path_and_complete_preset_inventory(self):
        self.assertEqual(default_face_config_path(), CONFIG.resolve())
        self.assertEqual(
            set(self.catalog.presets),
            {
                "neutral", "idle", "happy", "excited", "love", "sad",
                "angry", "alert", "thinking", "confused", "sleeping",
                "success", "error", "scared", "playful", "shutdown",
            },
        )
        self.assertEqual(
            {preset.effect for preset in self.catalog.presets.values()}
            - set(SUPPORTED_EFFECTS),
            set(),
        )
        self.assertEqual(self.catalog.presets["shutdown"].effect, "off")
        self.assertEqual(
            self.catalog.presets["excited"].alternate_color,
            (255, 0, 180),
        )

    def test_actual_emotes_and_action_aliases_are_centrally_mapped(self):
        self.assertEqual(self.catalog.emote_mappings["push_ups"], "angry")
        self.assertEqual(self.catalog.emote_mappings["body_roll"], "playful")
        self.assertEqual(self.catalog.emote_mappings["heart"], "love")
        self.assertEqual(self.catalog.emote_mappings["happy_dance"], "excited")
        self.assertEqual(self.catalog.emote_mappings["sleep"], "sleeping")
        self.assertEqual(self.catalog.emote_mappings["wake_up"], "success")
        self.assertEqual(self.catalog.state_mappings["sitting"], "neutral")
        self.assertEqual(self.catalog.state_mappings["standing_up"], "success")
        self.assertEqual(self.catalog.state_mappings["walking"], "idle")
        self.assertEqual(self.catalog.state_mappings["calibration"], "thinking")

    def test_every_preset_is_bounded(self):
        for name, preset in self.catalog.presets.items():
            with self.subTest(expression=name):
                self.assertTrue(all(0 <= channel <= 255 for channel in preset.color))
                if preset.alternate_color is not None:
                    self.assertTrue(all(
                        0 <= channel <= 255
                        for channel in preset.alternate_color
                    ))
                self.assertTrue(0 <= preset.brightness <= 255)
                self.assertTrue(10 <= preset.speed_ms <= 60000)

    def test_preset_settings_use_configured_or_mirrored_alternate_color(self):
        excited = settings_for_preset(self.catalog, "excited")
        self.assertEqual(excited.color, (0, 255, 255))
        self.assertEqual(excited.alternate_color, (255, 0, 180))
        happy = settings_for_preset(self.catalog, "happy")
        self.assertEqual(happy.alternate_color, happy.color)

    def test_invalid_settings_fail_closed(self):
        cases = (
            {"expression": "missing"},
            {"expression": "idle", "color": [-1, 0, 0]},
            {"expression": "idle", "alternate_color": [0, 0, 256]},
            {"expression": "idle", "brightness": 256},
            {"expression": "idle", "speed_ms": 9},
            {"expression": "idle", "effect": "unknown"},
            {"expression": "idle", "automatic": 1},
        )
        for value in cases:
            with self.subTest(value=value):
                with self.assertRaises(FaceConfigError):
                    validate_face_settings(value, self.catalog)


class FacePersistenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog = load_face_catalog(CONFIG)

    def test_xdg_path_and_atomic_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(
                default_settings_path({"XDG_CONFIG_HOME": directory}),
                root / "volt" / "face_led_settings.json",
            )
            path = root / "nested" / "settings.json"
            settings = FaceSettings(
                enabled=False,
                automatic=False,
                locked=True,
                expression="love",
                color=(12, 34, 56),
                alternate_color=(65, 43, 21),
                brightness=73,
                effect="scanner",
                speed_ms=456,
            )
            self.assertEqual(save_face_settings(settings, self.catalog, path), path)
            self.assertEqual(load_face_settings(self.catalog, path), settings)
            payload = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(payload["schema_version"], 1)
            self.assertEqual(payload["color"], [12, 34, 56])
            self.assertEqual(payload["alternate_color"], [65, 43, 21])
            self.assertEqual(list(path.parent.glob("*.tmp")), [])

    def test_legacy_settings_without_alternate_color_derive_a_safe_default(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.json"
            path.write_text(
                json.dumps({
                    "schema_version": 1,
                    "expression": "happy",
                    "color": [12, 34, 56],
                    "brightness": 80,
                    "effect": "solid",
                    "speed_ms": 500,
                    "enabled": True,
                    "automatic": True,
                    "locked": False,
                }),
                encoding="utf-8",
            )
            loaded = load_face_settings(self.catalog, path)
            self.assertEqual(loaded.color, (12, 34, 56))
            self.assertEqual(loaded.alternate_color, loaded.color)

    def test_missing_or_corrupt_settings_restore_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.json"
            expected = default_face_settings(self.catalog)
            self.assertEqual(load_face_settings(self.catalog, path), expected)
            path.write_text("{broken", encoding="utf-8")
            self.assertEqual(load_face_settings(self.catalog, path), expected)


class FaceAutomationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog = load_face_catalog(CONFIG)

    def setUp(self):
        self.manual = FaceSettings(
            expression="sad",
            color=(9, 10, 11),
            alternate_color=(9, 10, 11),
            brightness=42,
            effect="solid",
            speed_ms=777,
        )
        self.automation = FaceAutomation(self.catalog, self.manual)

    def test_emote_start_and_end_restore_previous_manual_expression(self):
        start = self.automation.update(
            {"emote_active": True, "emote_name": "heart", "state": "standing"}
        )
        self.assertEqual(start.expression, "love")
        self.assertEqual(start.reason, "emote:heart")
        end = self.automation.update(
            {"emote_active": False, "emote_name": "", "state": "standing"}
        )
        self.assertEqual(end.expression, "sad")
        self.assertTrue(end.restored)
        self.assertIsNone(
            self.automation.update(
                {"emote_active": False, "emote_name": "", "state": "standing"}
            )
        )

    def test_manual_lock_blocks_emotes_and_walking_but_not_safety(self):
        locked = FaceSettings(**dict(self.manual.__dict__, locked=True))
        automation = FaceAutomation(self.catalog, locked)
        manual = automation.update(
            {"emote_active": True, "emote_name": "happy_dance", "motion_active": True}
        )
        self.assertIsNone(manual)
        emergency = automation.update(
            {"emergency_stop": True, "emote_active": True, "emote_name": "heart"}
        )
        self.assertEqual(emergency.expression, "error")
        self.assertTrue(emergency.safety_override)

    def test_low_voltage_has_safety_precedence_and_restores(self):
        low = self.automation.update(
            {"low_voltage": "1", "emote_active": True, "emote_name": "heart"}
        )
        self.assertEqual(low.expression, "alert")
        self.assertTrue(low.safety_override)
        emote = self.automation.update(
            {"low_voltage": "0", "emote_active": True, "emote_name": "heart"}
        )
        self.assertEqual(emote.expression, "love")
        restored = self.automation.update({"state": "standing"})
        self.assertEqual(restored.expression, "sad")
        self.assertTrue(restored.restored)

    def test_sit_stand_walk_and_calibration_transitions(self):
        self.assertEqual(self.automation.update({"state": "sitting_down"}).expression, "neutral")
        self.assertEqual(self.automation.update({"state": "standing_up"}).expression, "success")
        # A completed stand restores the prior manual face (idle by default in
        # production), so success cannot stick indefinitely.
        restored = self.automation.update({"state": "standing"})
        self.assertEqual(restored.expression, "sad")
        self.assertTrue(restored.restored)
        self.assertEqual(self.automation.update({"state": "standing", "motion_active": True}).expression, "idle")
        self.assertEqual(self.automation.update({"state": "standing", "command_owner": "CALIBRATION"}).expression, "thinking")

    def test_automatic_toggle_blocks_non_safety_changes(self):
        manual = FaceSettings(**dict(self.manual.__dict__, automatic=False))
        automation = FaceAutomation(self.catalog, manual)
        decision = automation.update(
            {"emote_active": True, "emote_name": "push_ups", "motion_active": True}
        )
        self.assertIsNone(decision)
        fault = automation.update({"critical_fault": "motor driver"})
        self.assertEqual(fault.expression, "error")


if __name__ == "__main__":
    unittest.main()
