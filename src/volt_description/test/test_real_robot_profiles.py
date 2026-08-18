#!/usr/bin/env python3

"""Contract tests for validated real-robot tuning profiles and gait builders."""

import copy
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from volt_gait_controller import (  # noqa: E402
    DIAGNOSTIC_CRAWL,
    GAITS,
    REAL_SAFE_TROT,
    VoltGaitController,
    apply_real_tuning_to_configs,
    clamp_elliptical_offset,
    diagnostic_crawl_config,
    load_gait_configs,
    real_safe_trot_config,
)
from volt_kinematics import LEG_ORDER  # noqa: E402
from volt_motion_controller import VoltMotionController  # noqa: E402
from volt_real_profiles import (  # noqa: E402
    NUMERIC_BOUNDS,
    PROFILE_NAMES,
    RealProfileError,
    load_profiles,
    save_user_profile,
    validate_tuning,
)


EXPECTED_REAL_DIAGNOSTIC = {
    "gait": "diagnostic_crawl",
    "cycle_duration": 2.00,
    "stride_length": 0.035,
    "lateral_stride_width": 0.010,
    "step_height": 0.040,
    "duty_factor": 0.80,
    "body_height": 0.200,
    "body_x": 0.0,
    "body_y": 0.0,
    "body_roll_deg": 0.0,
    "body_pitch_deg": 0.0,
    "body_yaw_deg": 0.0,
    "max_joint_velocity_deg_s": 100.0,
    "max_joint_acceleration_deg_s2": 240.0,
    "smoothing_amount": 0.15,
    "touchdown_softness": 0.30,
    "stance_width": 0.104,
}


class RealProfileValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profiles = load_profiles(include_user=False)

    def test_required_builtin_profiles_are_present_and_valid(self):
        self.assertEqual(set(self.profiles), set(PROFILE_NAMES))
        for name in PROFILE_NAMES:
            with self.subTest(profile=name):
                self.assertEqual(
                    self.profiles[name],
                    validate_tuning(self.profiles[name]),
                )

    def test_real_diagnostic_defaults_are_exact(self):
        self.assertEqual(
            self.profiles["REAL_DIAGNOSTIC"],
            EXPECTED_REAL_DIAGNOSTIC,
        )

    def test_numeric_bounds_are_inclusive_and_outside_values_are_rejected(self):
        baseline = self.profiles["REAL_SAFE"]
        for field, (lower, upper) in NUMERIC_BOUNDS.items():
            with self.subTest(field=field, endpoint="lower"):
                candidate = dict(baseline, **{field: lower})
                self.assertEqual(validate_tuning(candidate)[field], float(lower))
            with self.subTest(field=field, endpoint="upper"):
                # Trot has a narrower gait-specific swing constraint at the
                # generic 0.90 duty endpoint; crawl is the valid consumer of
                # that inclusive top-level bound.
                endpoint_profile = (
                    self.profiles["REAL_DIAGNOSTIC"]
                    if field == "duty_factor"
                    else baseline
                )
                candidate = dict(endpoint_profile, **{field: upper})
                self.assertEqual(validate_tuning(candidate)[field], float(upper))
            for label, value in (
                ("below", math.nextafter(lower, -math.inf)),
                ("above", math.nextafter(upper, math.inf)),
            ):
                with self.subTest(field=field, endpoint=label):
                    with self.assertRaises(RealProfileError):
                        validate_tuning(dict(baseline, **{field: value}))

    def test_nonfinite_and_unknown_values_are_rejected(self):
        baseline = self.profiles["REAL_SAFE"]
        for field in NUMERIC_BOUNDS:
            for value in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(field=field, value=value):
                    with self.assertRaises(RealProfileError):
                        validate_tuning(dict(baseline, **{field: value}))

        with self.assertRaises(RealProfileError):
            validate_tuning(dict(baseline, unexpected_limit=1.0))

        for field in NUMERIC_BOUNDS:
            with self.subTest(field=field, value="boolean"):
                with self.assertRaises(RealProfileError):
                    validate_tuning(dict(baseline, **{field: True}))

    def test_hardware_attitude_bounds_match_the_real_gait_limit(self):
        self.assertEqual(NUMERIC_BOUNDS["body_roll_deg"], (-4.5, 4.5))
        self.assertEqual(NUMERIC_BOUNDS["body_pitch_deg"], (-4.5, 4.5))

    def test_user_profile_save_and_load_round_trip_uses_explicit_path(self):
        custom = dict(
            self.profiles["REAL_SAFE"],
            body_y=0.006,
            step_height=0.037,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profiles" / "user_profiles.yaml"
            # Seed the explicit test file with the required built-ins so the
            # normal public loader can validate the complete persisted file.
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                yaml.safe_dump(
                    {"version": 1, "profiles": self.profiles},
                    sort_keys=False,
                ),
                encoding="utf-8",
            )
            save_user_profile("floor_custom", custom, path=path)

            loaded = load_profiles(path=path, include_user=False)

        self.assertIn("FLOOR_CUSTOM", loaded)
        self.assertEqual(loaded["FLOOR_CUSTOM"], validate_tuning(custom))

    def test_user_overlay_cannot_replace_reserved_simulation_profile(self):
        changed = dict(self.profiles["SIMULATION"], cycle_duration=4.0)
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "volt_description" / "real_robot_profiles.yaml"
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(
                yaml.safe_dump({"version": 1, "profiles": {"SIMULATION": changed}}),
                encoding="utf-8",
            )
            with patch.dict(os.environ, {"XDG_CONFIG_HOME": directory}):
                with self.assertWarnsRegex(RuntimeWarning, "read-only"):
                    loaded = load_profiles()
                with self.assertRaisesRegex(RealProfileError, "reserved"):
                    save_user_profile(
                        "SIMULATION",
                        changed,
                        path=Path(directory) / "reserved.yaml",
                    )

        self.assertEqual(loaded["SIMULATION"], self.profiles["SIMULATION"])


class RealGaitBuilderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profiles = load_profiles(include_user=False)

    def test_simulation_and_hardware_gait_builders_are_separate(self):
        simulation_before = copy.deepcopy(GAITS["spotmicro_video_walk"])
        normal_trot_before = copy.deepcopy(GAITS["normal_trot"])

        diagnostic = diagnostic_crawl_config(
            self.profiles["REAL_DIAGNOSTIC"]
        )
        safe_trot = real_safe_trot_config(self.profiles["REAL_SAFE"])

        self.assertEqual(diagnostic["type"], "stable_crawl")
        self.assertEqual(safe_trot["type"], "real_safe_trot")
        self.assertEqual(
            self.profiles["SIMULATION"]["gait"],
            "spotmicro_video_walk",
        )
        self.assertEqual(diagnostic["real_tuning"]["gait"], DIAGNOSTIC_CRAWL)
        self.assertEqual(safe_trot["real_tuning"]["gait"], REAL_SAFE_TROT)
        self.assertEqual(GAITS["spotmicro_video_walk"], simulation_before)
        self.assertEqual(GAITS["normal_trot"], normal_trot_before)

    def test_diagnostic_crawl_timing_and_one_leg_semantics(self):
        tuning = self.profiles["REAL_DIAGNOSTIC"]
        config = diagnostic_crawl_config(tuning)
        per_leg_slot = tuning["cycle_duration"] / len(LEG_ORDER)

        self.assertEqual(tuple(config["leg_sequence"]), (
            "rear_right",
            "front_right",
            "rear_left",
            "front_left",
        ))
        self.assertEqual(set(config["leg_sequence"]), set(LEG_ORDER))
        self.assertAlmostEqual(
            config["swing_duration"],
            tuning["cycle_duration"] * (1.0 - tuning["duty_factor"]),
        )
        self.assertAlmostEqual(
            config["shift_duration"]
            + config["support_verify_duration"]
            + config["swing_duration"]
            + config["settle_duration"],
            per_leg_slot,
        )
        self.assertAlmostEqual(config["period"], tuning["cycle_duration"])
        self.assertAlmostEqual(config["hardware_time_scale"], 1.0)

        controller = VoltGaitController(hardware_mode=True)
        controller.set_gait(DIAGNOSTIC_CRAWL, 0.0)
        velocity = (0.25 * config["max_x"], 0.0, 0.0)
        observed_swing_legs = set()
        dt = 0.005
        sample_count = int(math.ceil(1.10 * config["period"] / dt))
        for index in range(1, sample_count + 1):
            controller.set_support_feedback({
                "command_ready": True,
                "tracking_available": False,
                "tracking_required": False,
                "contacts": None,
            })
            controller.step(index * dt, dt, velocity)
            debug = controller.debug_snapshot()
            swing = set(debug["swing_legs"])
            stance = set(debug["stance_legs"])
            self.assertLessEqual(len(swing), 1)
            if swing:
                observed_swing_legs.update(swing)
                self.assertEqual(len(stance), 3)
                self.assertFalse(swing & stance)
                self.assertEqual(swing | stance, set(LEG_ORDER))
        self.assertEqual(observed_swing_legs, set(LEG_ORDER))

    def test_real_safe_trot_preserves_duty_factor_and_clearance(self):
        tuning = self.profiles["REAL_SAFE"]
        config = real_safe_trot_config(tuning)

        self.assertEqual(config["type"], "real_safe_trot")
        self.assertAlmostEqual(config["period"], tuning["cycle_duration"])
        self.assertAlmostEqual(config["stance_ratio"], tuning["duty_factor"])
        self.assertAlmostEqual(
            config["swing_ratio"],
            1.0 - tuning["duty_factor"],
        )
        self.assertAlmostEqual(
            config["stance_ratio"] + config["swing_ratio"],
            1.0,
        )
        self.assertAlmostEqual(config["step_height"], tuning["step_height"])

        controller = VoltGaitController(hardware_mode=True)
        controller.set_gait(REAL_SAFE_TROT, 0.0)
        leg = "front_left"
        origin = (0.10, 0.104, -0.20)
        target = (0.14, 0.104, -0.20)
        controller.swing_origins[leg] = origin
        controller.swing_targets[leg] = target
        controller.swing_heights[leg] = tuning["step_height"]
        midpoint = controller.phase_trot_swing_step(leg, 0.5)
        self.assertAlmostEqual(
            midpoint[2] - origin[2],
            tuning["step_height"],
        )
        self.assertEqual(controller.phase_trot_swing_step(leg, 0.0), origin)
        self.assertEqual(controller.phase_trot_swing_step(leg, 1.0), target)

    def test_hardware_reset_preserves_tuned_stance_width(self):
        tuning = dict(self.profiles["REAL_SAFE"], stance_width=0.112)
        configs = apply_real_tuning_to_configs(load_gait_configs(), tuning)
        controller = VoltGaitController(configs, hardware_mode=True)
        controller.set_gait(REAL_SAFE_TROT, 0.0)

        controller.reset(1.0)

        for leg in LEG_ORDER:
            expected = 0.112 if "left" in leg else -0.112
            self.assertAlmostEqual(controller.feet[leg][1], expected)
            self.assertAlmostEqual(controller.world_feet[leg][1], expected)
            self.assertAlmostEqual(controller.swing_origins[leg][1], expected)
            self.assertAlmostEqual(controller.swing_targets[leg][1], expected)

    def test_zero_lateral_stride_is_a_supported_disabled_axis(self):
        self.assertEqual(
            clamp_elliptical_offset(0.02, 0.01, 0.035, 0.0),
            (0.02, 0.0),
        )
        tuning = dict(
            self.profiles["REAL_DIAGNOSTIC"],
            lateral_stride_width=0.0,
        )
        configs = apply_real_tuning_to_configs(load_gait_configs(), tuning)
        controller = VoltGaitController(configs, hardware_mode=True)
        controller.set_gait(DIAGNOSTIC_CRAWL, 0.0)

        target = controller.video_touchdown_world(
            "front_left",
            (0.01, 0.0, 0.0),
        )

        self.assertTrue(all(math.isfinite(value) for value in target))
        self.assertAlmostEqual(target[1], controller.nominal_foot("front_left")[1])

    def test_full_diagnostic_stride_is_included_in_atomic_ik_preflight(self):
        controller = VoltMotionController.__new__(VoltMotionController)
        for name in ("REAL_DIAGNOSTIC", "REAL_SAFE", "REAL_NORMAL"):
            with self.subTest(profile=name):
                controller.preflight_real_tuning(self.profiles[name])

        unreachable = dict(
            self.profiles["REAL_DIAGNOSTIC"],
            body_height=0.185,
            body_x=-0.025,
            body_y=-0.020,
            body_pitch_deg=-4.5,
            body_yaw_deg=-10.0,
            stance_width=0.130,
        )
        with self.assertRaises(RealProfileError):
            controller.preflight_real_tuning(unreachable)

        between_old_samples = dict(
            self.profiles["REAL_DIAGNOSTIC"],
            stride_length=0.05206108,
            lateral_stride_width=0.01676134,
            step_height=0.02846799,
            body_height=0.19695133,
            body_x=-0.01782362,
            body_y=-0.01601979,
            body_roll_deg=1.35244,
            body_pitch_deg=-1.54628,
            body_yaw_deg=4.45479,
            stance_width=0.12581118,
        )
        with self.assertRaises(RealProfileError):
            controller.preflight_real_tuning(between_old_samples)

    def test_apply_real_tuning_leaves_simulation_gaits_unchanged(self):
        simulation_gaits = (
            "spotmicro_video_walk",
            "spot_walk",
            "legacy_walk",
            "amble",
            "slow_trot",
            "normal_trot",
            "fast_trot",
        )
        for profile_name, changed_field in (
            ("REAL_DIAGNOSTIC", {"step_height": 0.039}),
            ("REAL_SAFE", {"step_height": 0.037}),
        ):
            with self.subTest(profile=profile_name):
                source = load_gait_configs()
                source_before = copy.deepcopy(source)
                tuning = dict(self.profiles[profile_name], **changed_field)

                updated = apply_real_tuning_to_configs(source, tuning)

                self.assertIsNot(updated, source)
                self.assertEqual(source, source_before)
                for gait_name in simulation_gaits:
                    self.assertEqual(
                        updated[gait_name],
                        source_before[gait_name],
                        gait_name,
                    )
                selected = tuning["gait"]
                self.assertEqual(updated[selected]["real_tuning"], tuning)
                self.assertAlmostEqual(
                    updated[selected]["step_height"],
                    changed_field["step_height"],
                )


if __name__ == "__main__":
    unittest.main()
