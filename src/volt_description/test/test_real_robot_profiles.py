#!/usr/bin/env python3

"""Contract tests for validated real-robot tuning profiles.

Profiles overlay the two-gait engine: each names its target gait
(``trot`` or ``amble``), and applying one re-runs the engine's servo
budget sweep, so a profile can never demand joint speeds the TD-8130MG
cannot deliver under load.
"""

import copy
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from volt_gait_controller import (  # noqa: E402
    GAITS,
    VoltGaitController,
    apply_real_tuning_to_configs,
    canonical_gait_name,
)
from volt_real_profiles import (  # noqa: E402
    NUMERIC_BOUNDS,
    RealProfileError,
    TUNABLE_FIELDS,
    TUNABLE_GAITS,
    load_profiles,
    save_user_profile,
    smoothing_alpha,
    validate_tuning,
)


def shipped_profiles():
    return load_profiles(include_user=False)


class ProfileFileTests(unittest.TestCase):
    def test_shipped_profiles_load_and_validate(self):
        profiles = shipped_profiles()
        self.assertEqual(
            sorted(profiles),
            ["REAL_DIAGNOSTIC", "REAL_NORMAL", "REAL_SAFE", "SIMULATION"],
        )
        for name, tuning in profiles.items():
            validated = validate_tuning(tuning, allow_simulation=True)
            self.assertIn(validated["gait"], TUNABLE_GAITS, name)

    def test_every_shipped_profile_passes_the_servo_budget(self):
        for name, tuning in shipped_profiles().items():
            validated = validate_tuning(tuning)
            configs = apply_real_tuning_to_configs(GAITS, validated)
            gait = validated["gait"]
            self.assertIn(gait, configs, name)

    def test_diagnostic_profile_is_an_amble(self):
        tuning = shipped_profiles()["REAL_DIAGNOSTIC"]
        self.assertEqual(canonical_gait_name(tuning["gait"]), "amble")
        self.assertGreaterEqual(tuning["duty_factor"], 0.70)

    def test_load_and_normal_profiles_are_trots(self):
        profiles = shipped_profiles()
        for name in ("REAL_SAFE", "REAL_NORMAL"):
            self.assertEqual(
                canonical_gait_name(profiles[name]["gait"]), "trot", name
            )


class ValidationTests(unittest.TestCase):
    def base(self):
        return dict(shipped_profiles()["REAL_SAFE"])

    def test_unknown_field_is_rejected(self):
        tuning = self.base()
        tuning["warp_factor"] = 9.0
        with self.assertRaises(RealProfileError):
            validate_tuning(tuning)

    def test_missing_field_is_rejected(self):
        tuning = self.base()
        del tuning["cycle_duration"]
        with self.assertRaises(RealProfileError):
            validate_tuning(tuning)

    def test_unknown_gait_is_rejected(self):
        tuning = self.base()
        tuning["gait"] = "fast_trot"
        with self.assertRaises(RealProfileError):
            validate_tuning(tuning)

    def test_numeric_bounds_are_enforced(self):
        for field, (lower, upper) in NUMERIC_BOUNDS.items():
            tuning = self.base()
            tuning[field] = upper + abs(upper) * 0.5 + 1.0
            with self.assertRaises(RealProfileError, msg=field):
                validate_tuning(tuning)

    def test_boolean_values_are_rejected(self):
        tuning = self.base()
        tuning["cycle_duration"] = True
        with self.assertRaises(RealProfileError):
            validate_tuning(tuning)

    def test_amble_duty_floor(self):
        tuning = self.base()
        tuning["gait"] = "amble"
        tuning["duty_factor"] = 0.60
        with self.assertRaises(RealProfileError):
            validate_tuning(tuning)

    def test_trot_duty_range(self):
        tuning = self.base()
        tuning["gait"] = "trot"
        tuning["duty_factor"] = 0.85
        with self.assertRaises(RealProfileError):
            validate_tuning(tuning)

    def test_velocity_ceiling_cannot_bypass_the_firmware(self):
        tuning = self.base()
        tuning["max_joint_velocity_deg_s"] = 260.0
        with self.assertRaises(RealProfileError):
            validate_tuning(tuning)

    def test_smoothing_alpha_maps_amount_inversely(self):
        strong = dict(self.base(), smoothing_amount=0.80)
        weak = dict(self.base(), smoothing_amount=0.0)
        self.assertLess(smoothing_alpha(strong), smoothing_alpha(weak))


class ApplyChainTests(unittest.TestCase):
    def test_apply_overrides_the_named_gait_only(self):
        tuning = validate_tuning(shipped_profiles()["REAL_SAFE"])
        configs = apply_real_tuning_to_configs(GAITS, tuning)
        self.assertAlmostEqual(
            configs["trot"]["cycle_period"], tuning["cycle_duration"]
        )
        self.assertEqual(
            configs["amble"]["cycle_period"], GAITS["amble"]["cycle_period"]
        )

    def test_apply_rejects_a_budget_breaking_profile(self):
        tuning = validate_tuning(shipped_profiles()["REAL_NORMAL"])
        tuning = dict(tuning)
        tuning["cycle_duration"] = 0.60
        tuning["stride_length"] = 0.075
        with self.assertRaises(ValueError):
            apply_real_tuning_to_configs(GAITS, tuning)

    def test_engine_accepts_the_applied_tuning(self):
        controller = VoltGaitController(hardware_mode=True)
        tuning = validate_tuning(shipped_profiles()["REAL_DIAGNOSTIC"])
        applied = controller.set_real_tuning(tuning)
        self.assertEqual(applied["gait"], "amble")
        self.assertAlmostEqual(
            controller.gaits["amble"]["cycle_period"],
            tuning["cycle_duration"],
        )


class UserProfileTests(unittest.TestCase):
    def test_save_and_reload_roundtrip(self):
        base = validate_tuning(shipped_profiles()["REAL_SAFE"])
        with tempfile.TemporaryDirectory() as tmp:
            user_path = Path(tmp) / "user_profiles.yaml"
            with patch(
                "volt_real_profiles.user_profile_path",
                return_value=user_path,
            ):
                save_user_profile("MY_TEST", base)
                merged = load_profiles(include_user=True)
            self.assertIn("MY_TEST", merged)
            reloaded = validate_tuning(merged["MY_TEST"])
            for field in TUNABLE_FIELDS:
                self.assertEqual(reloaded[field], base[field], field)

    def test_user_file_cannot_bypass_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            user_path = Path(tmp) / "user_profiles.yaml"
            user_path.write_text(
                yaml.safe_dump({
                    "version": 1,
                    "profiles": {
                        "EVIL": dict(
                            shipped_profiles()["REAL_SAFE"],
                            max_joint_velocity_deg_s=500.0,
                        ),
                    },
                }),
                encoding="utf-8",
            )
            with patch(
                "volt_real_profiles.user_profile_path",
                return_value=user_path,
            ):
                # The loader itself validates and must refuse the file.
                with self.assertRaises(RealProfileError):
                    load_profiles(include_user=True)


if __name__ == "__main__":
    unittest.main()
