#!/usr/bin/env python3

"""Pure regression tests for VOLT's canonical kinematics contract."""

import json
import math
import random
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from volt_kinematics import (  # noqa: E402
    FOOT_LIMIT,
    HIP_ORIGINS,
    JOINT_NAMES,
    LEG_LIMIT,
    LEG_ORDER,
    NOMINAL_FEET,
    SHOULDER_LIMIT,
    feet_to_joint_positions_diagnostic,
    forward_leg,
    joint_positions_to_feet,
)


EXPECTED_JOINT_ORDER = [
    "front_left_shoulder",
    "front_left_leg",
    "front_left_foot",
    "front_right_shoulder",
    "front_right_leg",
    "front_right_foot",
    "rear_left_shoulder",
    "rear_left_leg",
    "rear_left_foot",
    "rear_right_shoulder",
    "rear_right_leg",
    "rear_right_foot",
]


def body_target_from_shoulder(leg_name, shoulder_target):
    """Convert a forward_leg shoulder-frame point to a body-frame target."""
    hip = HIP_ORIGINS[leg_name]
    return tuple(
        shoulder_target[index] + hip[index]
        for index in range(3)
    )


class KinematicsTestCase(unittest.TestCase):
    """Shared assertions for finite output and diagnostic structure."""

    def assert_joint_limits(self, positions):
        self.assertEqual(len(positions), len(JOINT_NAMES))
        limits = (SHOULDER_LIMIT, LEG_LIMIT, FOOT_LIMIT)
        for index, value in enumerate(positions):
            lower, upper = limits[index % 3]
            self.assertGreaterEqual(value, lower - 1e-12)
            self.assertLessEqual(value, upper + 1e-12)

    def assert_safe_result(self, positions, diagnostics):
        self.assertEqual(len(positions), len(JOINT_NAMES))
        self.assertTrue(all(math.isfinite(value) for value in positions))
        self.assert_joint_limits(positions)
        self.assertIsInstance(diagnostics, dict)
        self.assertIsInstance(diagnostics.get("projected_targets"), list)

        # Reject NaN/Infinity in diagnostics as well as non-JSON types.
        json.dumps(diagnostics, allow_nan=False, sort_keys=True)

        details = self.leg_details(diagnostics)
        for leg_name in LEG_ORDER:
            self.assertIn(leg_name, details)
            self.assertIsInstance(details[leg_name], dict)
            self.reasons_for(diagnostics, leg_name)

    def leg_details(self, diagnostics):
        for key in ("legs", "per_leg", "leg_details"):
            details = diagnostics.get(key)
            if isinstance(details, dict):
                return details
        self.fail(
            "diagnostics must contain a per-leg mapping under "
            "'legs', 'per_leg', or 'leg_details'"
        )

    def reasons_for(self, diagnostics, leg_name):
        detail = self.leg_details(diagnostics)[leg_name]
        reasons = detail.get("reasons", detail.get("reason", []))
        if reasons is None:
            reasons = []
        if isinstance(reasons, str):
            reasons = [reasons]
        self.assertIsInstance(reasons, (list, tuple))
        self.assertTrue(all(isinstance(reason, str) for reason in reasons))
        return [reason.lower() for reason in reasons]

    def projected_names(self, diagnostics):
        names = []
        for item in diagnostics["projected_targets"]:
            if isinstance(item, str):
                names.append(item)
                continue
            if isinstance(item, dict):
                name = item.get("leg", item.get("name"))
                self.assertIsInstance(name, str)
                names.append(name)
                continue
            self.fail("projected_targets entries must be leg names or dicts")
        return names

    def assert_reason_mentions(self, diagnostics, leg_name, *tokens):
        combined = " ".join(self.reasons_for(diagnostics, leg_name))
        self.assertTrue(
            any(token in combined for token in tokens),
            "%s diagnostic reasons %r did not mention any of %r"
            % (leg_name, combined, tokens),
        )


class CanonicalKinematicsTests(KinematicsTestCase):
    def test_joint_names_are_in_the_exact_canonical_order(self):
        self.assertEqual(JOINT_NAMES, EXPECTED_JOINT_ORDER)

    def test_nominal_feet_produce_twelve_finite_unprojected_angles(self):
        positions, diagnostics = feet_to_joint_positions_diagnostic(
            NOMINAL_FEET
        )
        self.assert_safe_result(positions, diagnostics)
        self.assertEqual(self.projected_names(diagnostics), [])

    def test_nominal_left_and_right_geometry_is_mirrored_canonically(self):
        positions, diagnostics = feet_to_joint_positions_diagnostic(
            NOMINAL_FEET
        )
        self.assert_safe_result(positions, diagnostics)

        solved = {
            leg_name: positions[index * 3:index * 3 + 3]
            for index, leg_name in enumerate(LEG_ORDER)
        }
        for left, right in (
            ("front_left", "front_right"),
            ("rear_left", "rear_right"),
        ):
            self.assertAlmostEqual(solved[left][0], -solved[right][0])
            self.assertAlmostEqual(solved[left][1], solved[right][1])
            self.assertAlmostEqual(solved[left][2], solved[right][2])

            left_foot = forward_leg(left, solved[left])
            right_foot = forward_leg(right, solved[right])
            self.assertAlmostEqual(left_foot[0], right_foot[0])
            self.assertAlmostEqual(left_foot[1], -right_foot[1])
            self.assertAlmostEqual(left_foot[2], right_foot[2])

    def test_reachable_ik_targets_round_trip_through_forward_kinematics(self):
        requested_angles = (0.17, 0.68, -1.24)
        for leg_name in LEG_ORDER:
            with self.subTest(leg=leg_name):
                shoulder_target = forward_leg(leg_name, requested_angles)
                feet = dict(NOMINAL_FEET)
                feet[leg_name] = body_target_from_shoulder(
                    leg_name,
                    shoulder_target,
                )

                positions, diagnostics = feet_to_joint_positions_diagnostic(
                    feet
                )
                self.assert_safe_result(positions, diagnostics)
                self.assertNotIn(
                    leg_name,
                    self.projected_names(diagnostics),
                )

                offset = LEG_ORDER.index(leg_name) * 3
                solved_target = forward_leg(
                    leg_name,
                    positions[offset:offset + 3],
                )
                for expected, actual in zip(
                    shoulder_target,
                    solved_target,
                ):
                    self.assertAlmostEqual(expected, actual, places=9)

    def test_random_reachable_targets_are_finite_and_round_trip(self):
        generator = random.Random(4192)
        for sample in range(64):
            feet = {}
            expected_targets = {}
            for leg_name in LEG_ORDER:
                angles = (
                    generator.uniform(-0.25, 0.25),
                    generator.uniform(0.25, 1.00),
                    generator.uniform(-1.80, -0.50),
                )
                shoulder_target = forward_leg(leg_name, angles)
                expected_targets[leg_name] = shoulder_target
                feet[leg_name] = body_target_from_shoulder(
                    leg_name,
                    shoulder_target,
                )

            positions, diagnostics = feet_to_joint_positions_diagnostic(
                feet
            )
            self.assert_safe_result(positions, diagnostics)
            self.assertEqual(
                self.projected_names(diagnostics),
                [],
                "unexpected projection in random sample %d" % sample,
            )

            for index, leg_name in enumerate(LEG_ORDER):
                actual = forward_leg(
                    leg_name,
                    positions[index * 3:index * 3 + 3],
                )
                self.assertLess(
                    math.dist(expected_targets[leg_name], actual),
                    1e-9,
                )

    def test_body_translation_and_rotation_outputs_remain_finite(self):
        poses = (
            {},
            {"height": 0.17, "body_x": 0.015, "body_y": -0.010},
            {"roll": 0.20},
            {"pitch": -0.20},
            {"yaw": 0.30},
            {
                "height": 0.22,
                "body_x": -0.010,
                "body_y": 0.010,
                "roll": -0.18,
                "pitch": 0.16,
                "yaw": -0.25,
            },
        )
        for pose in poses:
            with self.subTest(pose=pose):
                positions, diagnostics = (
                    feet_to_joint_positions_diagnostic(
                        NOMINAL_FEET,
                        **pose,
                    )
                )
                self.assert_safe_result(positions, diagnostics)

    def test_whole_body_forward_kinematics_recovers_reachable_feet(self):
        pose = {
            "height": 0.19,
            "body_x": 0.008,
            "body_y": -0.006,
            "roll": 0.08,
            "pitch": -0.06,
            "yaw": 0.10,
        }
        positions, diagnostics = feet_to_joint_positions_diagnostic(
            NOMINAL_FEET,
            **pose,
        )
        self.assert_safe_result(positions, diagnostics)
        recovered = joint_positions_to_feet(positions, **pose)
        for leg_name in LEG_ORDER:
            self.assertLess(
                math.dist(recovered[leg_name], NOMINAL_FEET[leg_name]),
                1e-9,
            )


class KinematicsDiagnosticTests(KinematicsTestCase):
    def test_outside_maximum_reach_is_projected_and_reported(self):
        leg_name = "front_left"
        feet = dict(NOMINAL_FEET)
        feet[leg_name] = (1.40, 0.45, -1.20)

        positions, diagnostics = feet_to_joint_positions_diagnostic(feet)
        self.assert_safe_result(positions, diagnostics)
        self.assertIn(leg_name, self.projected_names(diagnostics))
        self.assert_reason_mentions(
            diagnostics,
            leg_name,
            "maximum",
            "max_reach",
            "outside",
            "workspace",
        )

    def test_target_too_close_to_hip_is_projected_and_reported(self):
        leg_name = "rear_right"
        feet = dict(NOMINAL_FEET)
        feet[leg_name] = HIP_ORIGINS[leg_name]

        positions, diagnostics = feet_to_joint_positions_diagnostic(feet)
        self.assert_safe_result(positions, diagnostics)
        self.assertIn(leg_name, self.projected_names(diagnostics))
        self.assert_reason_mentions(
            diagnostics,
            leg_name,
            "too_close",
            "minimum",
            "singular",
            "near",
        )

    def test_nonfinite_targets_never_produce_nonfinite_output(self):
        for bad_value in (
            float("nan"),
            float("inf"),
            float("-inf"),
        ):
            with self.subTest(value=bad_value):
                leg_name = "front_right"
                feet = dict(NOMINAL_FEET)
                original = feet[leg_name]
                feet[leg_name] = (
                    original[0],
                    bad_value,
                    original[2],
                )

                positions, diagnostics = (
                    feet_to_joint_positions_diagnostic(feet)
                )
                self.assert_safe_result(positions, diagnostics)
                self.assertIn(
                    leg_name,
                    self.projected_names(diagnostics),
                )
                self.assert_reason_mentions(
                    diagnostics,
                    leg_name,
                    "nonfinite",
                    "non_finite",
                    "non-finite",
                    "not finite",
                    "invalid",
                )

    def test_joint_limit_clamping_is_reported(self):
        leg_name = "front_left"
        outside_shoulder_limit = (
            SHOULDER_LIMIT[1] + 0.30,
            0.65,
            -1.10,
        )
        feet = dict(NOMINAL_FEET)
        feet[leg_name] = body_target_from_shoulder(
            leg_name,
            forward_leg(leg_name, outside_shoulder_limit),
        )

        positions, diagnostics = feet_to_joint_positions_diagnostic(feet)
        self.assert_safe_result(positions, diagnostics)
        self.assertAlmostEqual(
            positions[0],
            SHOULDER_LIMIT[1],
            places=9,
        )
        self.assert_reason_mentions(
            diagnostics,
            leg_name,
            "joint_limit",
            "joint limit",
            "clamp",
        )


if __name__ == "__main__":
    unittest.main()
