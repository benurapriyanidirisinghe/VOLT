#!/usr/bin/env python3

"""Pure safety contracts for the stopped-state FAST TROT sweep helper."""

import copy
import json
import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
CONFIG_PATH = ROOT / "config" / "physical_fast_trot.yaml"
sys.path.insert(0, str(SCRIPTS))

from volt_fast_trot_sweep import (  # noqa: E402
    HOLD_OBSERVE_ACKNOWLEDGEMENT,
    MAX_SWEEP_POINTS,
    SUPPORTED_PARAMETERS,
    SweepError,
    authorize_apply,
    build_sweep_plan,
    next_tuning_after_confirmation,
    status_safety_errors,
    tuning_json,
    tuning_matches,
    validate_complete_tuning,
    validate_safe_status,
    validate_value_list,
)
from volt_gait_controller import (  # noqa: E402
    FAST_TROT_PRESET_PARAMETER_NAMES,
    FAST_TROT_TUNING_BOUNDS,
    load_fast_trot_config,
)
from volt_kinematics import LEG_ORDER  # noqa: E402


CONFIG = load_fast_trot_config(CONFIG_PATH)
BASELINE = dict(CONFIG["presets"]["bench"])


def safe_status(tuning=None):
    return {
        "hardware_mode": True,
        "use_sim_time": False,
        "fast_trot_profile": "physical",
        "requested_gait": "fast_trot",
        "active_gait": "fast_trot",
        "state": "standing",
        "moving": False,
        "motion_active": False,
        "step_in_place": False,
        "pending_gait": None,
        "pending_pose_action": None,
        "physical_test_active": False,
        "physical_test_returning": False,
        "command_owner": "MOTION",
        "motion_authorized": True,
        "phase_transition_hold": False,
        "stance_legs": list(LEG_ORDER),
        "swing_legs": [],
        "requested_velocity": [0.0, 0.0, 0.0],
        "filtered_velocity": [0.0, 0.0, 0.0],
        "fast_trot_tuning": dict(tuning or BASELINE),
    }


class SweepPlanTests(unittest.TestCase):
    def test_supported_names_and_bounds_are_the_controller_contract(self):
        self.assertEqual(
            SUPPORTED_PARAMETERS,
            tuple(FAST_TROT_PRESET_PARAMETER_NAMES),
        )
        self.assertEqual(
            FAST_TROT_TUNING_BOUNDS,
            {
                "stride_scale": (0.50, 1.25),
                "step_height": (0.020, 0.050),
                "hardware_cycle_period": (0.50, 0.90),
                "hardware_speed_scale": (0.20, 0.75),
            },
        )

    def test_value_lists_are_finite_bounded_unique_and_short(self):
        parameter, values = validate_value_list(
            "step_height",
            [0.030, 0.034, 0.038],
        )
        self.assertEqual(parameter, "step_height")
        self.assertEqual(values, (0.030, 0.034, 0.038))

        invalid_lists = (
            [],
            [float("nan")],
            [float("inf")],
            [0.019],
            [0.051],
            [0.030, 0.030],
            [0.030] * (MAX_SWEEP_POINTS + 1),
        )
        for values in invalid_lists:
            with self.subTest(values=values):
                with self.assertRaises(SweepError):
                    validate_value_list("step_height", values)
        with self.assertRaises(SweepError):
            validate_value_list("not_a_parameter", [1.0])

    def test_plan_preserves_three_fields_and_changes_exactly_one(self):
        cases = {
            "stride_scale": [0.55, 0.60],
            "step_height": [0.030, 0.032],
            "hardware_cycle_period": [0.82, 0.86],
            "hardware_speed_scale": [0.30, 0.35],
        }
        for parameter, values in cases.items():
            with self.subTest(parameter=parameter):
                plan = build_sweep_plan(
                    parameter,
                    values,
                    BASELINE,
                    config=CONFIG,
                )
                self.assertEqual(plan.baseline, BASELINE)
                self.assertEqual(len(plan.points), len(values))
                for point, value in zip(plan.points, values):
                    self.assertEqual(point.tuning[parameter], value)
                    changed = {
                        name
                        for name in SUPPORTED_PARAMETERS
                        if point.tuning[name] != BASELINE[name]
                    }
                    self.assertEqual(changed, {parameter})

    def test_plan_rejects_baseline_points_and_cross_field_unsafe_values(self):
        with self.assertRaisesRegex(SweepError, "equals the baseline"):
            build_sweep_plan(
                "stride_scale",
                [BASELINE["stride_scale"]],
                BASELINE,
                config=CONFIG,
            )

        # This value is inside the scalar stride bound but the unchanged BENCH
        # speed/period cannot produce the requested stride. The same controller
        # validator used by the subscriber must reject it offline.
        with self.assertRaisesRegex(SweepError, "command envelope"):
            build_sweep_plan(
                "stride_scale",
                [0.65],
                BASELINE,
                config=CONFIG,
            )

    def test_complete_json_has_all_and_only_live_fields(self):
        encoded = tuning_json(BASELINE, config=CONFIG)
        decoded = json.loads(encoded)
        self.assertEqual(set(decoded), set(SUPPORTED_PARAMETERS))
        self.assertTrue(
            all(math.isfinite(value) for value in decoded.values())
        )
        self.assertNotIn("NaN", encoded)
        self.assertNotIn("Infinity", encoded)

        for malformed in (
            {},
            {"stride_scale": 0.5},
            {**BASELINE, "extra": 1.0},
            {**BASELINE, "step_height": float("nan")},
        ):
            with self.subTest(malformed=malformed):
                with self.assertRaises(SweepError):
                    validate_complete_tuning(malformed, config=CONFIG)

    def test_tuning_comparison_is_complete_and_tolerant_only_to_roundoff(self):
        close = dict(BASELINE)
        close["step_height"] += 0.5e-9
        far = dict(BASELINE)
        far["step_height"] += 2.0e-9
        self.assertTrue(tuning_matches(BASELINE, close))
        self.assertFalse(tuning_matches(BASELINE, far))
        self.assertFalse(tuning_matches(BASELINE, {"stride_scale": 0.5}))


class LiveGateTests(unittest.TestCase):
    def test_safe_status_returns_the_complete_baseline_echo(self):
        status = safe_status()
        self.assertEqual(status_safety_errors(status), [])
        self.assertEqual(
            validate_safe_status(
                status,
                expected_tuning=BASELINE,
                config=CONFIG,
            ),
            BASELINE,
        )

    def test_every_required_live_gate_fails_closed(self):
        mutations = {
            "hardware_mode": False,
            "use_sim_time": True,
            "fast_trot_profile": "simulation",
            "requested_gait": "spotmicro_video_walk",
            "active_gait": "spotmicro_video_walk",
            "state": "standing_up",
            "moving": True,
            "motion_active": True,
            "step_in_place": True,
            "pending_gait": "fast_trot",
            "pending_pose_action": "stand",
            "physical_test_active": True,
            "physical_test_returning": True,
            "command_owner": "HOLD",
            "motion_authorized": False,
            "phase_transition_hold": True,
            "stance_legs": list(LEG_ORDER[:-1]),
            "swing_legs": ["front_left"],
            "requested_velocity": [0.001, 0.0, 0.0],
            "filtered_velocity": [0.0, 0.0, 0.001],
        }
        for field, value in mutations.items():
            with self.subTest(field=field):
                status = safe_status()
                status[field] = value
                with self.assertRaises(SweepError):
                    validate_safe_status(status, config=CONFIG)

        for missing in tuple(safe_status()):
            with self.subTest(missing=missing):
                status = safe_status()
                del status[missing]
                with self.assertRaises(SweepError):
                    validate_safe_status(status, config=CONFIG)

    def test_baseline_echo_must_be_complete_and_match_expected_prior(self):
        status = safe_status()
        del status["fast_trot_tuning"]
        with self.assertRaisesRegex(SweepError, "baseline echo"):
            validate_safe_status(status, config=CONFIG)

        different = dict(BASELINE)
        different["step_height"] = 0.030
        with self.assertRaisesRegex(SweepError, "expected prior"):
            validate_safe_status(
                safe_status(different),
                expected_tuning=BASELINE,
                config=CONFIG,
            )

    def test_next_point_requires_confirmation_of_the_prior_point(self):
        plan = build_sweep_plan(
            "step_height",
            [0.030, 0.032],
            BASELINE,
            config=CONFIG,
        )
        first = next_tuning_after_confirmation(
            plan,
            0,
            safe_status(BASELINE),
            config=CONFIG,
        )
        self.assertEqual(first, plan.points[0].tuning)

        with self.assertRaisesRegex(SweepError, "expected prior"):
            next_tuning_after_confirmation(
                plan,
                1,
                safe_status(BASELINE),
                config=CONFIG,
            )
        second = next_tuning_after_confirmation(
            plan,
            1,
            safe_status(plan.points[0].tuning),
            config=CONFIG,
        )
        self.assertEqual(second, plan.points[1].tuning)

    def test_apply_requires_typed_hold_and_observe_acknowledgement(self):
        self.assertFalse(authorize_apply(False))
        with self.assertRaisesRegex(
            SweepError,
            "acknowledge-hold-observe",
        ):
            authorize_apply(True, "")
        with self.assertRaises(SweepError):
            authorize_apply(True, "yes")
        self.assertTrue(
            authorize_apply(
                True,
                HOLD_OBSERVE_ACKNOWLEDGEMENT,
            )
        )

    def test_source_contains_only_the_two_authorized_ros_topics(self):
        source = (
            SCRIPTS / "volt_fast_trot_sweep.py"
        ).read_text(encoding="utf-8")
        forbidden_topics = (
            "/cmd_vel",
            "/volt/gait",
            "/volt/action",
            "/volt/command_owner",
            "/volt/serial_command",
            "/volt/physical_test",
            "/volt/joint_commands",
            "/joint_command_router/output",
        )
        for topic in forbidden_topics:
            with self.subTest(topic=topic):
                self.assertNotIn(topic, source)
        self.assertIn('TUNING_TOPIC = "/volt/fast_trot_tuning"', source)
        self.assertIn('STATUS_TOPIC = "/volt/status"', source)


if __name__ == "__main__":
    unittest.main()
