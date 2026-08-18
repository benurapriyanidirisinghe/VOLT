#!/usr/bin/env python3

import math
import inspect
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from volt_fast_trot_diagnostic import (  # noqa: E402
    CycleStrideTracker,
    VoltFastTrotDiagnostic,
    check_command_jumps,
    check_diagonal_pairing,
    check_knee_branch_flip,
    check_serial_rate,
    check_stance_behavior,
    check_swing_clearance,
    consecutive_deltas,
    csv_columns,
    diagnostic_warning_checks,
    duplicate_publisher_conflicts,
    effective_serial_frame_rate,
    format_terminal_summary,
    parse_serial_status,
    should_emit_summary,
    status_body_transform,
    status_body_world,
    status_clamp_count,
    status_desired_feet,
    status_loop_metrics,
    status_per_leg_phase,
    throttled_warnings,
)
from volt_kinematics import JOINT_NAMES, LEG_ORDER, NOMINAL_FEET  # noqa: E402


class FastTrotDiagnosticPureTests(unittest.TestCase):
    def test_csv_schema_is_unique_and_contains_every_joint_and_foot(self):
        columns = csv_columns()
        self.assertEqual(len(columns), len(set(columns)))
        for leg_name in LEG_ORDER:
            for axis in ("x", "y", "z"):
                self.assertIn("%s_foot_%s_m" % (leg_name, axis), columns)
        self.assertEqual(
            len([column for column in columns if column.endswith("_rad")]),
            14,
        )
        self.assertEqual(
            len([
                column for column in columns
                if column.endswith("_servo_deg")
            ]),
            12,
        )
        for leg_name in LEG_ORDER:
            self.assertIn("%s_phase" % leg_name, columns)
        for joint_name in JOINT_NAMES:
            self.assertIn(
                "%s_servo_delta_deg" % joint_name,
                columns,
            )
        for metric in (
            "control_loop_rate_hz",
            "command_publish_rate_hz",
            "control_loop_dt_s",
            "control_loop_max_dt_s",
            "expected_control_rate_hz",
            "missed_deadlines",
        ):
            self.assertIn(metric, columns)

    def test_serial_status_parser_ignores_frame_tail(self):
        parsed = parse_serial_status(
            "connected=1 sent=25 rejected=0 frame=90.0 "
            "91.0 92.0"
        )
        self.assertEqual(parsed["connected"], "1")
        self.assertEqual(parsed["sent"], "25")
        self.assertEqual(parsed["rejected"], "0")
        self.assertEqual(parsed["frame"], "90.0")

    def test_cycle_stride_tracker_requires_signed_grounded_stance(self):
        tracker = CycleStrideTracker()

        def feet(x_offset):
            return {
                leg: (
                    NOMINAL_FEET[leg][0] + x_offset,
                    NOMINAL_FEET[leg][1],
                    NOMINAL_FEET[leg][2],
                )
                for leg in LEG_ORDER
            }

        tracker.update(0.1, feet(0.02), stance_legs=LEG_ORDER)
        tracker.update(0.2, feet(-0.02), stance_legs=LEG_ORDER)
        completed = tracker.update(0.3, feet(-0.02), stance_legs=[])
        self.assertAlmostEqual(completed, 0.04)
        self.assertTrue(math.isfinite(completed))

        airborne = {
            leg: (
                NOMINAL_FEET[leg][0] + 0.02,
                NOMINAL_FEET[leg][1],
                NOMINAL_FEET[leg][2] + 0.010,
            )
            for leg in LEG_ORDER
        }
        tracker.reset()
        tracker.update(0.1, airborne, stance_legs=LEG_ORDER)
        tracker.update(0.2, feet(-0.02), stance_legs=LEG_ORDER)
        self.assertEqual(
            tracker.update(0.3, feet(-0.02), stance_legs=[]),
            0.0,
        )

    def test_status_transform_and_clamp_totals_match_schema(self):
        transform = {
            "height": 0.188,
            "body_x": 0.0,
            "body_y": 0.0,
            "roll": 0.0,
            "pitch": 0.025,
            "yaw": 0.0,
        }
        status = {
            "gait_body_transform": transform,
            "joint_velocity_clamp_count": 17,
            "joint_acceleration_clamp_counts": {
                "joint_a": 2,
                "joint_b": 3,
            },
        }
        self.assertEqual(status_body_transform(status), transform)
        self.assertEqual(
            status_clamp_count(
                status,
                "joint_velocity_clamp_count",
                "joint_velocity_clamp_counts",
                "velocity_limit_clamps",
            ),
            17,
        )
        self.assertEqual(
            status_clamp_count(
                status,
                "joint_acceleration_clamp_count",
                "joint_acceleration_clamp_counts",
                "acceleration_limit_clamps",
            ),
            5,
        )
        invalid = dict(transform, pitch=float("nan"))
        self.assertEqual(
            status_body_transform({"gait_body_transform": invalid}),
            {},
        )
        self.assertEqual(
            status_body_world({
                "body_world": {"x": 1.2, "y": -0.3, "yaw": 0.1},
            }),
            {"x": 1.2, "y": -0.3, "yaw": 0.1},
        )
        self.assertEqual(
            status_body_world({
                "body_world": {"x": 1.2, "y": float("nan"), "yaw": 0.1},
            }),
            {},
        )

    def test_bridge_frame_rate_is_authoritative(self):
        recorder = VoltFastTrotDiagnostic.__new__(
            VoltFastTrotDiagnostic
        )
        recorder.serial_frame_rate = 3.0
        recorder.previous_serial_sent = 100.0
        recorder.previous_serial_ros = 9.0
        recorder.serial_status = {}
        recorder.now_seconds = lambda: 10.0

        recorder.serial_status_callback(
            SimpleNamespace(data="sent=125 frame_rate=25.00")
        )

        self.assertEqual(recorder.serial_frame_rate, 25.0)
        self.assertEqual(recorder.serial_status["frame_rate"], "25.00")

    def test_per_leg_phase_and_optional_metrics_are_backward_compatible(self):
        phases = status_per_leg_phase({"cycle_phase": 0.10})
        self.assertAlmostEqual(phases["front_left"], 0.10)
        self.assertAlmostEqual(phases["rear_right"], 0.10)
        self.assertAlmostEqual(phases["front_right"], 0.60)
        self.assertAlmostEqual(phases["rear_left"], 0.60)

        explicit = {
            leg: (0.2 if leg in ("front_left", "rear_right") else 0.7)
            for leg in LEG_ORDER
        }
        self.assertEqual(
            status_per_leg_phase({"per_leg_phase": explicit}),
            explicit,
        )
        missing = status_per_leg_phase({})
        self.assertTrue(all(math.isnan(value) for value in missing.values()))

        metrics = status_loop_metrics({
            "controller_loop_rate_hz": 99.0,
            "loop_metrics": {
                "command_publish_rate_hz": 30.0,
                "max_loop_dt_s": 0.014,
                "deadline_misses": 3,
            },
        })
        self.assertEqual(metrics["control_loop_rate_hz"], 99.0)
        self.assertEqual(metrics["command_publish_rate_hz"], 30.0)
        self.assertEqual(metrics["control_loop_max_dt_s"], 0.014)
        self.assertEqual(metrics["missed_deadlines"], 3.0)
        self.assertTrue(
            math.isnan(status_loop_metrics({})["control_loop_rate_hz"])
        )
        self.assertEqual(
            effective_serial_frame_rate(
                {"arduino_frame_rate": 28.0},
                float("nan"),
            ),
            28.0,
        )
        self.assertEqual(
            effective_serial_frame_rate(
                {"arduino_frame_rate": 28.0},
                29.5,
            ),
            29.5,
        )

    def test_desired_feet_accept_xyz_dict_and_missing_old_status(self):
        feet = status_desired_feet({
            "desired_feet": {
                "front_left": {"x": 0.1, "y": 0.2, "z": -0.18},
            },
        })
        self.assertEqual(feet["front_left"], [0.1, 0.2, -0.18])
        self.assertTrue(
            all(math.isnan(value) for value in feet["rear_right"])
        )
        missing = status_desired_feet({})
        self.assertTrue(
            all(
                math.isnan(value)
                for leg_name in LEG_ORDER
                for value in missing[leg_name]
            )
        )

    def test_jump_and_knee_branch_checks_use_consecutive_commands(self):
        previous_joints = {name: -0.8 for name in JOINT_NAMES}
        joints = dict(previous_joints)
        joints["front_left_shoulder"] = -0.5
        joints["rear_right_foot"] = 0.4
        previous_servos = {name: 90.0 for name in JOINT_NAMES}
        servos = dict(previous_servos)
        servos["front_left_shoulder"] = 105.0

        deltas = consecutive_deltas(
            servos,
            previous_servos,
            JOINT_NAMES,
        )
        self.assertEqual(deltas["front_left_shoulder"], 15.0)
        jump_codes = {
            code
            for code, _message in check_command_jumps(
                joints,
                servos,
                previous_joints,
                previous_servos,
                joint_threshold_rad=0.2,
                servo_threshold_deg=10.0,
            )
        }
        self.assertEqual(jump_codes, {"joint_jump", "servo_jump"})
        knee = check_knee_branch_flip(
            joints,
            previous_joints,
            sign_epsilon_rad=0.05,
            discontinuity_rad=0.3,
        )
        self.assertEqual(knee[0][0], "knee_branch_flip")
        self.assertIn("rear_right_foot", knee[0][1])

    def test_diagonal_pairing_checks_sets_and_phase_separation(self):
        correct_phases = {
            "front_left": 0.1,
            "rear_right": 0.1,
            "front_right": 0.6,
            "rear_left": 0.6,
        }
        correct = {
            "swing_legs": ["front_left", "rear_right"],
            "stance_legs": ["front_right", "rear_left"],
        }
        self.assertEqual(
            check_diagonal_pairing(correct, correct_phases),
            [],
        )
        wrong = dict(
            correct,
            swing_legs=["front_left", "front_right"],
        )
        warnings = check_diagonal_pairing(wrong, correct_phases)
        self.assertEqual(warnings[0][0], "diagonal_pairing")

        wrong_phases = dict(correct_phases, rear_right=0.3)
        warnings = check_diagonal_pairing(correct, wrong_phases)
        self.assertIn("not phase matched", warnings[0][1])

    def test_swing_clearance_lift_and_stance_ground_checks(self):
        phases = status_per_leg_phase({"cycle_phase": 0.02})
        previous = {
            leg: list(NOMINAL_FEET[leg])
            for leg in LEG_ORDER
        }
        current = {
            leg: list(NOMINAL_FEET[leg])
            for leg in LEG_ORDER
        }
        current["front_left"][0] += 0.003
        current["front_left"][2] += 0.002
        swing_status = {
            "swing_legs": ["front_left", "rear_right"],
            "stance_legs": ["front_right", "rear_left"],
        }
        warnings = check_swing_clearance(
            swing_status,
            current,
            previous,
            phases,
            liftoff_clearance_m=0.008,
            motion_threshold_m=0.001,
        )
        self.assertIn(
            "swing_before_clearance",
            {code for code, _message in warnings},
        )

        mid_phases = {
            "front_left": 0.2,
            "rear_right": 0.2,
            "front_right": 0.7,
            "rear_left": 0.7,
        }
        warnings = check_swing_clearance(
            swing_status,
            previous,
            previous,
            mid_phases,
            minimum_lift_m=0.020,
        )
        self.assertIn(
            "insufficient_lift",
            {code for code, _message in warnings},
        )

        moved = {leg: list(values) for leg, values in previous.items()}
        moved["front_right"][0] += 0.010
        stance_status = dict(
            swing_status,
            stance_max_ground_error=0.005,
            stance_ground_tolerance=0.002,
        )
        warnings = check_stance_behavior(
            stance_status,
            moved,
            previous,
            reposition_threshold_m=0.006,
        )
        self.assertEqual(
            {code for code, _message in warnings},
            {"stance_reposition", "stance_ground_error"},
        )

    def test_rate_conflict_and_keyed_throttle_helpers(self):
        self.assertEqual(check_serial_rate(25.0, 20.0, 35.0), [])
        self.assertEqual(
            check_serial_rate(10.0, 20.0, 35.0)[0][0],
            "serial_rate_low",
        )
        self.assertEqual(
            check_serial_rate(40.0, 20.0, 35.0)[0][0],
            "serial_rate_high",
        )
        conflicts = duplicate_publisher_conflicts(
            {
                "duplicate_command_topics": ["/joint_commands"],
                "command_publisher_counts": {
                    "/joint_command_router/output": 2,
                },
            },
            {"stack_conflict": "/volt/status"},
        )
        self.assertIn("/joint_commands", conflicts)
        self.assertIn("/joint_command_router/output(2)", conflicts)
        self.assertIn("/volt/status", conflicts)
        self.assertEqual(
            duplicate_publisher_conflicts({}, {"stack_conflict": "-"}),
            (),
        )

        warnings = [
            ("jump", "joint jumped"),
            ("rate", "rate low"),
        ]
        due, times = throttled_warnings(warnings, 10.0, {}, 2.0)
        self.assertEqual(due, warnings)
        self.assertEqual(
            throttled_warnings(warnings, 11.0, times, 2.0)[0],
            [],
        )
        self.assertEqual(
            throttled_warnings(warnings, 12.0, times, 2.0)[0],
            warnings,
        )

    def test_low_rate_summary_covers_motion_mapping_and_loop_health(self):
        status = {
            "cycle_phase": 0.1,
            "phase_name": "cycle",
            "swing_legs": ["front_left", "rear_right"],
            "stance_legs": ["front_right", "rear_left"],
            "control_loop_rate_hz": 99.5,
            "command_publish_rate_hz": 29.8,
            "control_loop_dt_s": 0.010,
            "control_loop_max_dt_s": 0.016,
            "expected_control_rate_hz": 100.0,
            "missed_deadlines": 2,
            "joint_limit_clamp_count": 1,
            "ik_projection_count": 3,
        }
        desired = {
            leg: list(NOMINAL_FEET[leg])
            for leg in LEG_ORDER
        }
        joints = {name: -0.5 for name in JOINT_NAMES}
        servos = {name: 90.0 for name in JOINT_NAMES}
        deltas = {name: 0.5 for name in JOINT_NAMES}
        summary = format_terminal_summary(
            status,
            desired,
            joints,
            servos,
            deltas,
            29.9,
        )
        self.assertIn("FAST_TROT phase=0.100", summary)
        self.assertIn("front_left phase=0.100 swing", summary)
        self.assertIn("desired_xyz=", summary)
        self.assertIn("canonical_rad", summary)
        self.assertIn("mapped_servo_deg(delta)", summary)
        self.assertIn("publish=29.8Hz loop=99.5Hz", summary)
        self.assertIn("missed=2", summary)
        self.assertIn("serial=29.9Hz", summary)
        self.assertIn("joint_limit=1", summary)
        self.assertIn("IK=3", summary)
        self.assertTrue(should_emit_summary(10.0, None, 1.0))
        self.assertFalse(should_emit_summary(10.5, 10.0, 1.0))
        self.assertTrue(should_emit_summary(11.0, 10.0, 1.0))
        self.assertFalse(should_emit_summary(11.0, 10.0, 0.0))

    def test_aggregate_checks_accept_old_status_and_node_is_passive(self):
        values = {name: -0.5 for name in JOINT_NAMES}
        warnings = diagnostic_warning_checks(
            {},
            values,
            values,
            {},
            {},
            status_desired_feet({}),
            {},
            float("nan"),
        )
        self.assertEqual(warnings, [])
        self.assertNotIn(
            "create_publisher(",
            inspect.getsource(VoltFastTrotDiagnostic),
        )


if __name__ == "__main__":
    unittest.main()
