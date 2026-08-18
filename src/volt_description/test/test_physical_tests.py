#!/usr/bin/env python3

"""ROS-free safety and Cartesian contracts for ``volt_physical_tests``."""

import json
import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from volt_kinematics import (  # noqa: E402
    FOOT_LIMIT,
    JOINT_NAMES,
    LEG_LIMIT,
    LEG_ORDER,
    NOMINAL_FEET,
    SHOULDER_LIMIT,
)
from volt_physical_tests import (  # noqa: E402
    CARTESIAN_TEST_MODES,
    DIAGONAL_PAIRS,
    DIAGONAL_PAIR_LIFT,
    EMERGENCY_STOP,
    PHYSICAL_FAST_TROT,
    SINGLE_LEG_LIFT,
    STAND,
    SUPPORT_STAND_ACKNOWLEDGEMENT,
    WEIGHT_SHIFT,
    ZERO_STRIDE_TROT,
    PhysicalTestError,
    authorize_execution,
    build_execution_plan,
    cartesian_frame_at,
    generate_cartesian_trajectory,
    physical_test_request_json,
    physical_stack_is_settled,
    physical_stack_status_errors,
    parse_key_value_status,
    serial_stack_ready,
    validate_cartesian_trajectory,
)


def lifted_legs(frame, tolerance=1e-9):
    return {
        leg
        for leg in LEG_ORDER
        if frame.feet[leg][2] > NOMINAL_FEET[leg][2] + tolerance
    }


class CliSafetyContractTests(unittest.TestCase):
    def test_execute_is_required_before_any_mode_is_authorized(self):
        with self.assertRaisesRegex(PhysicalTestError, "--execute"):
            authorize_execution(
                STAND,
                False,
                SUPPORT_STAND_ACKNOWLEDGEMENT,
            )

    def test_support_stand_phrase_is_typed_exactly_for_motion(self):
        for mode in (STAND, SINGLE_LEG_LIFT, ZERO_STRIDE_TROT):
            with self.subTest(mode=mode):
                with self.assertRaisesRegex(
                    PhysicalTestError,
                    "acknowledge-support-stand",
                ):
                    authorize_execution(mode, True, "")
                with self.assertRaises(PhysicalTestError):
                    authorize_execution(mode, True, "yes")
                self.assertTrue(
                    authorize_execution(
                        mode,
                        True,
                        SUPPORT_STAND_ACKNOWLEDGEMENT,
                    )
                )

    def test_emergency_stop_still_requires_execute_but_not_acknowledgement(self):
        with self.assertRaises(PhysicalTestError):
            build_execution_plan(EMERGENCY_STOP, False)
        plan = build_execution_plan(EMERGENCY_STOP, True)
        self.assertEqual(plan.mode, EMERGENCY_STOP)
        self.assertFalse(plan.disable_output)

    def test_disable_is_only_an_explicit_emergency_choice(self):
        plan = build_execution_plan(
            EMERGENCY_STOP,
            True,
            disable_output=True,
        )
        self.assertTrue(plan.disable_output)
        with self.assertRaisesRegex(PhysicalTestError, "emergency-stop"):
            build_execution_plan(
                STAND,
                True,
                SUPPORT_STAND_ACKNOWLEDGEMENT,
                disable_output=True,
            )

    def test_single_lift_requires_a_canonical_leg(self):
        with self.assertRaisesRegex(PhysicalTestError, "--leg"):
            build_execution_plan(
                SINGLE_LEG_LIFT,
                True,
                SUPPORT_STAND_ACKNOWLEDGEMENT,
            )
        plan = build_execution_plan(
            SINGLE_LEG_LIFT,
            True,
            SUPPORT_STAND_ACKNOWLEDGEMENT,
            leg="front_left",
        )
        self.assertEqual(plan.leg, "front_left")

    def test_request_envelope_has_finite_idempotency_fields(self):
        encoded = physical_test_request_json(
            "keepalive",
            SINGLE_LEG_LIFT,
            6.0,
            "request_1234",
            leg="rear_right",
        )
        payload = json.loads(encoded)
        self.assertEqual(
            set(payload),
            {"command", "mode", "leg", "duration", "request_id"},
        )
        self.assertEqual(payload["command"], "keepalive")
        self.assertEqual(payload["mode"], SINGLE_LEG_LIFT)
        self.assertEqual(payload["leg"], "rear_right")
        self.assertEqual(payload["request_id"], "request_1234")
        self.assertTrue(math.isfinite(payload["duration"]))

        for command in ("start", "keepalive", "cancel"):
            with self.subTest(command=command):
                payload = json.loads(
                    physical_test_request_json(
                        command,
                        STAND,
                        5.0,
                        "request_1234",
                    )
                )
                self.assertEqual(payload["command"], command)
                self.assertEqual(payload["leg"], "")

    def test_request_envelope_rejects_gaits_and_bad_ids(self):
        with self.assertRaises(PhysicalTestError):
            physical_test_request_json(
                "start",
                PHYSICAL_FAST_TROT,
                6.0,
                "request_1234",
            )
        with self.assertRaises(PhysicalTestError):
            physical_test_request_json(
                "start",
                STAND,
                5.0,
                "bad id",
            )

    def test_live_stack_status_gate_fails_closed(self):
        status = {
            "hardware_mode": True,
            "use_sim_time": False,
            "fast_trot_profile": "physical",
            "fast_trot_config_file": "/tmp/physical_fast_trot.yaml",
            "physical_tests_enabled": True,
            "command_owner": "MOTION",
            "motion_authorized": True,
            "state": "standing",
            "moving": False,
            "motion_active": False,
            "step_in_place": False,
            "physical_test_active": False,
            "physical_test_returning": False,
            "pending_gait": None,
            "pending_pose_action": None,
            "stance_legs": list(LEG_ORDER),
            "swing_legs": [],
            "requested_velocity": [0.0, 0.0, 0.0],
            "filtered_velocity": [0.0, 0.0, 0.0],
        }
        self.assertEqual(physical_stack_status_errors(status), [])
        self.assertTrue(physical_stack_is_settled(status))

        for name, unsafe in (
            ("hardware_mode", False),
            ("use_sim_time", True),
            ("fast_trot_profile", "simulation"),
            ("physical_tests_enabled", False),
            ("command_owner", "HOLD"),
            ("motion_authorized", False),
            ("moving", True),
            ("motion_active", True),
            ("step_in_place", True),
            ("physical_test_active", True),
            ("physical_test_returning", True),
            ("swing_legs", ["front_left"]),
            ("requested_velocity", [0.01, 0.0, 0.0]),
        ):
            with self.subTest(name=name):
                changed = dict(status)
                changed[name] = unsafe
                self.assertTrue(physical_stack_status_errors(changed))
                if name in (
                    "moving",
                    "motion_active",
                    "step_in_place",
                    "physical_test_active",
                    "physical_test_returning",
                    "swing_legs",
                ):
                    self.assertFalse(physical_stack_is_settled(changed))

        for name in tuple(status):
            with self.subTest(missing=name):
                changed = dict(status)
                del changed[name]
                self.assertTrue(physical_stack_status_errors(changed))

    def test_serial_status_requires_dry_run_or_confirmed_live_arm(self):
        dry_run = parse_key_value_status(
            "dry_run=1 hardware_enabled=0 calibration_valid=1 armed=0"
        )
        live = parse_key_value_status(
            "dry_run=0 hardware_enabled=1 connected=1 ready=1 armed=1 "
            "firmware_compatible=1 calibration_valid=1"
        )
        self.assertTrue(serial_stack_ready(dry_run))
        self.assertTrue(serial_stack_ready(live))
        live["armed"] = "0"
        self.assertFalse(serial_stack_ready(live))
        self.assertEqual(
            parse_key_value_status('{"owner":"MOTION"}')["owner"],
            "MOTION",
        )


class PureCartesianTrajectoryTests(unittest.TestCase):
    def assert_nominal_frame(self, frame):
        for leg in LEG_ORDER:
            self.assertEqual(frame.feet[leg], tuple(NOMINAL_FEET[leg]))

    def assert_smooth_endpoints(self, trajectory):
        self.assert_nominal_frame(trajectory[0])
        self.assert_nominal_frame(trajectory[-1])

        first_motion = max(
            math.dist(
                trajectory[1].feet[leg],
                trajectory[0].feet[leg],
            )
            for leg in LEG_ORDER
        )
        second_motion = max(
            math.dist(
                trajectory[2].feet[leg],
                trajectory[1].feet[leg],
            )
            for leg in LEG_ORDER
        )
        penultimate_motion = max(
            math.dist(
                trajectory[-2].feet[leg],
                trajectory[-3].feet[leg],
            )
            for leg in LEG_ORDER
        )
        last_motion = max(
            math.dist(
                trajectory[-1].feet[leg],
                trajectory[-2].feet[leg],
            )
            for leg in LEG_ORDER
        )
        self.assertLessEqual(first_motion, second_motion + 1e-12)
        self.assertLessEqual(last_motion, penultimate_motion + 1e-12)

    def test_every_mode_produces_finite_canonical_joint_commands(self):
        cases = (
            (STAND, None),
            (WEIGHT_SHIFT, None),
            (SINGLE_LEG_LIFT, "front_left"),
            (SINGLE_LEG_LIFT, "front_right"),
            (SINGLE_LEG_LIFT, "rear_left"),
            (SINGLE_LEG_LIFT, "rear_right"),
            (DIAGONAL_PAIR_LIFT, None),
        )
        limits = (SHOULDER_LIMIT, LEG_LIMIT, FOOT_LIMIT)
        for mode, leg in cases:
            with self.subTest(mode=mode, leg=leg):
                trajectory = generate_cartesian_trajectory(
                    mode,
                    leg=leg,
                )
                joints = validate_cartesian_trajectory(trajectory)
                self.assertEqual(len(joints), len(trajectory))
                for positions in joints:
                    self.assertEqual(len(positions), len(JOINT_NAMES))
                    self.assertTrue(
                        all(math.isfinite(value) for value in positions)
                    )
                    for index, value in enumerate(positions):
                        lower, upper = limits[index % 3]
                        self.assertGreaterEqual(value, lower - 1e-12)
                        self.assertLessEqual(value, upper + 1e-12)

    def test_weight_shift_keeps_all_four_feet_on_ground(self):
        trajectory = generate_cartesian_trajectory(WEIGHT_SHIFT)
        moved_horizontally = False
        for frame in trajectory:
            for leg in LEG_ORDER:
                self.assertAlmostEqual(
                    frame.feet[leg][2],
                    NOMINAL_FEET[leg][2],
                    places=12,
                )
                moved_horizontally = moved_horizontally or (
                    abs(frame.feet[leg][0] - NOMINAL_FEET[leg][0]) > 1e-6
                    or abs(frame.feet[leg][1] - NOMINAL_FEET[leg][1]) > 1e-6
                )
        self.assertTrue(moved_horizontally)
        self.assert_smooth_endpoints(trajectory)

    def test_single_leg_lift_moves_only_the_selected_leg(self):
        for selected in LEG_ORDER:
            with self.subTest(leg=selected):
                trajectory = generate_cartesian_trajectory(
                    SINGLE_LEG_LIFT,
                    leg=selected,
                )
                peak = max(
                    trajectory,
                    key=lambda frame: frame.feet[selected][2],
                )
                self.assertEqual(lifted_legs(peak), {selected})
                for frame in trajectory:
                    self.assertTrue(lifted_legs(frame).issubset({selected}))
                    for leg in set(LEG_ORDER) - {selected}:
                        self.assertEqual(
                            frame.feet[leg],
                            tuple(NOMINAL_FEET[leg]),
                        )
                self.assert_smooth_endpoints(trajectory)

    def test_diagonal_pair_lift_uses_only_canonical_trot_pairs(self):
        trajectory = generate_cartesian_trajectory(DIAGONAL_PAIR_LIFT)
        observed_pairs = set()
        allowed = {frozenset(pair) for pair in DIAGONAL_PAIRS}
        for frame in trajectory:
            lifted = frozenset(lifted_legs(frame))
            if lifted:
                observed_pairs.add(lifted)
                self.assertIn(lifted, allowed)
        self.assertEqual(observed_pairs, allowed)
        midpoint = cartesian_frame_at(
            DIAGONAL_PAIR_LIFT,
            5.0,
            10.0,
        )
        self.assertEqual(lifted_legs(midpoint), set())
        self.assert_smooth_endpoints(trajectory)

    def test_all_cartesian_modes_have_smooth_nominal_endpoints(self):
        for mode in CARTESIAN_TEST_MODES:
            leg = "front_left" if mode == SINGLE_LEG_LIFT else None
            with self.subTest(mode=mode):
                trajectory = generate_cartesian_trajectory(mode, leg=leg)
                self.assert_smooth_endpoints(trajectory)

    def test_input_validation_rejects_nonfinite_or_unsafe_shape_requests(self):
        with self.assertRaises(PhysicalTestError):
            cartesian_frame_at(SINGLE_LEG_LIFT, 0.0, 6.0)
        with self.assertRaises(PhysicalTestError):
            cartesian_frame_at(STAND, 0.0, float("nan"))
        with self.assertRaises(PhysicalTestError):
            generate_cartesian_trajectory(STAND, sample_rate=1000.0)


if __name__ == "__main__":
    unittest.main()
