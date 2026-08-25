#!/usr/bin/env python3

import math
import re
import sys
import unittest
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from volt_kinematics import (
    JOINT_NAMES,
    LEG_FOOT_MID_POSE,
    LEG_FOOT_SIT_POSE,
    NOMINAL_FEET,
    SIT_POSE,
    WALK_POSE,
    feet_to_joint_positions,
    feet_to_joint_positions_diagnostic,
)
from volt_servo_calibration import (
    CalibrationError,
    ServoCalibrationTable,
    named_positions_from_ordered,
    ordered_positions_from_joint_state,
)


BASE = {
    "joint_order": JOINT_NAMES,
    "servos": {
        name: {
            "pca_channel": index,
            "direction": 1,
            "neutral_deg": 90.0,
            "trim_deg": 0.0,
            "min_deg": 0.0,
            "max_deg": 180.0,
            "min_pulse_us": 600,
            "max_pulse_us": 2400,
        }
        for index, name in enumerate(JOINT_NAMES)
    },
}


class ServoCalibrationTests(unittest.TestCase):
    def table(self, raw=None):
        return ServoCalibrationTable.from_dict(deepcopy(raw or BASE))

    def test_canonical_joint_order(self):
        self.assertEqual(JOINT_NAMES[0], "front_left_shoulder")
        self.assertEqual(JOINT_NAMES[-1], "rear_right_foot")
        self.assertEqual(len(JOINT_NAMES), 12)

    def test_validation_accepts_base(self):
        self.assertEqual(len(self.table().servos), 12)

    def test_missing_joint_detection(self):
        raw = deepcopy(BASE)
        raw["servos"].pop("rear_right_foot")
        with self.assertRaises(CalibrationError):
            self.table(raw)

    def test_duplicate_channel_detection(self):
        raw = deepcopy(BASE)
        raw["servos"]["rear_right_foot"]["pca_channel"] = 0
        with self.assertRaises(CalibrationError):
            self.table(raw)

    def test_positive_direction_conversion(self):
        table = self.table()
        self.assertAlmostEqual(
            table.ros_radians_to_servo_degrees("front_left_leg", 0.1),
            90.0 + math.degrees(0.1),
        )

    def test_negative_direction_conversion(self):
        raw = deepcopy(BASE)
        raw["servos"]["front_left_leg"]["direction"] = -1
        table = self.table(raw)
        self.assertAlmostEqual(
            table.ros_radians_to_servo_degrees("front_left_leg", 0.1),
            90.0 - math.degrees(0.1),
        )

    def test_inverse_conversion(self):
        table = self.table()
        self.assertAlmostEqual(
            table.servo_degrees_to_ros_radians("front_left_leg", 95.0),
            math.radians(5.0),
        )

    def test_trim_handling(self):
        raw = deepcopy(BASE)
        raw["servos"]["front_left_leg"]["trim_deg"] = 5.0
        table = self.table(raw)
        self.assertAlmostEqual(table.ros_radians_to_servo_degrees("front_left_leg", 0.0), 95.0)

    def test_trim_can_make_urdf_zero_unreachable_but_output_stays_safe(self):
        raw = deepcopy(BASE)
        raw["servos"]["rear_left_foot"]["neutral_deg"] = 180.0
        raw["servos"]["rear_left_foot"]["trim_deg"] = 9.0
        table = self.table(raw)
        self.assertEqual(
            table.ros_radians_to_servo_degrees("rear_left_foot", 0.0),
            180.0,
        )

    def test_clamping(self):
        raw = deepcopy(BASE)
        raw["servos"]["front_left_foot"]["neutral_deg"] = 10.0
        raw["servos"]["front_left_foot"]["max_deg"] = 20.0
        table = self.table(raw)
        self.assertEqual(table.ros_radians_to_servo_degrees("front_left_foot", 1.0), 20.0)

    def test_nan_rejection(self):
        with self.assertRaises(CalibrationError):
            self.table().ros_radians_to_servo_degrees("front_left_leg", float("nan"))

    def test_infinite_rejection(self):
        with self.assertRaises(CalibrationError):
            self.table().ros_radians_to_servo_degrees("front_left_leg", float("inf"))

    def test_joint_state_name_reordering(self):
        msg = SimpleNamespace(
            name=list(reversed(JOINT_NAMES)),
            position=list(reversed(range(12))),
        )
        self.assertEqual(ordered_positions_from_joint_state(msg), list(range(12)))

    def test_full_channel_frame_generation(self):
        raw = deepcopy(BASE)
        raw["servos"]["front_left_shoulder"]["pca_channel"] = 11
        raw["servos"]["rear_right_foot"]["pca_channel"] = 0
        table = self.table(raw)
        frame, details = table.channel_frame_from_positions({name: 0.0 for name in JOINT_NAMES})
        self.assertEqual(len(frame), 12)
        self.assertEqual(frame[11], 90.0)
        self.assertEqual(frame[0], 90.0)
        self.assertEqual(len(details), 12)

    def test_changing_one_joint_preserves_other_named_values(self):
        pose = {name: 0.0 for name in JOINT_NAMES}
        pose["front_left_leg"] = 0.05
        frame, _details = self.table().channel_frame_from_positions(pose)
        self.assertAlmostEqual(frame[1], 90.0 + math.degrees(0.05))
        for index in range(12):
            if index != 1:
                self.assertAlmostEqual(frame[index], 90.0)

    def test_named_positions_rejects_incomplete_arrays(self):
        with self.assertRaises(CalibrationError):
            named_positions_from_ordered([0.0])

    def test_invalid_direction_rejected(self):
        raw = deepcopy(BASE)
        raw["servos"]["front_left_leg"]["direction"] = 0
        with self.assertRaises(CalibrationError):
            self.table(raw)

    def test_wrong_joint_order_rejected(self):
        raw = deepcopy(BASE)
        raw["joint_order"] = list(reversed(JOINT_NAMES))
        with self.assertRaises(CalibrationError):
            self.table(raw)

    def test_unknown_joint_rejected(self):
        table = self.table()
        with self.assertRaises(CalibrationError):
            table.ros_radians_to_servo_degrees("bad_joint", 0.0)

    def test_channel_range_rejected(self):
        raw = deepcopy(BASE)
        raw["servos"]["front_left_leg"]["pca_channel"] = 16
        with self.assertRaises(CalibrationError):
            self.table(raw)

    def test_neutral_outside_range_rejected(self):
        raw = deepcopy(BASE)
        raw["servos"]["front_left_leg"]["neutral_deg"] = 200.0
        with self.assertRaises(CalibrationError):
            self.table(raw)

    def test_bad_pulse_width_rejected(self):
        raw = deepcopy(BASE)
        raw["servos"]["front_left_leg"]["min_pulse_us"] = 2500
        raw["servos"]["front_left_leg"]["max_pulse_us"] = 2400
        with self.assertRaises(CalibrationError):
            self.table(raw)

    def test_joint_state_missing_name_rejected(self):
        msg = SimpleNamespace(name=JOINT_NAMES[:-1], position=list(range(11)))
        with self.assertRaises(CalibrationError):
            ordered_positions_from_joint_state(msg)

    def test_frame_rejects_nonfinite_named_value(self):
        pose = {name: 0.0 for name in JOINT_NAMES}
        pose["front_left_leg"] = float("nan")
        with self.assertRaises(CalibrationError):
            self.table().channel_frame_from_positions(pose)

    def test_frame_rejects_missing_named_value(self):
        pose = {name: 0.0 for name in JOINT_NAMES}
        pose.pop("front_left_leg")
        with self.assertRaises(CalibrationError):
            self.table().channel_frame_from_positions(pose)

    def test_repo_directions_match_measured_robot(self):
        repo_table = ServoCalibrationTable.from_file(
            ROOT / "config" / "servo_calibration.yaml"
        )
        expected_directions = {
            "front_left_shoulder": -1,
            "front_left_leg": 1,
            "front_left_foot": -1,
            "front_right_shoulder": -1,
            "front_right_leg": -1,
            "front_right_foot": 1,
            "rear_left_shoulder": -1,
            "rear_left_leg": 1,
            "rear_left_foot": 1,
            "rear_right_shoulder": -1,
            "rear_right_leg": -1,
            "rear_right_foot": -1,
        }
        for joint_name in JOINT_NAMES:
            self.assertEqual(repo_table.servos[joint_name].direction, expected_directions[joint_name])

    def test_front_left_shoulder_matches_measured_physical_polarity(self):
        repo_table = ServoCalibrationTable.from_file(
            ROOT / "config" / "servo_calibration.yaml"
        )
        negative = repo_table.ros_radians_to_servo_degrees(
            "front_left_shoulder",
            -0.05,
        )
        zero = repo_table.ros_radians_to_servo_degrees(
            "front_left_shoulder",
            0.0,
        )
        positive = repo_table.ros_radians_to_servo_degrees(
            "front_left_shoulder",
            0.05,
        )
        self.assertGreater(negative, zero)
        self.assertGreater(zero, positive)

    def test_repo_pca_mapping_matches_measured_robot(self):
        repo_table = ServoCalibrationTable.from_file(
            ROOT / "config" / "servo_calibration.yaml"
        )
        expected_channels = {
            "front_left_shoulder": 3,
            "front_left_leg": 1,
            "front_left_foot": 2,
            "front_right_shoulder": 0,
            "front_right_leg": 4,
            "front_right_foot": 5,
            "rear_left_shoulder": 9,
            "rear_left_leg": 7,
            "rear_left_foot": 8,
            "rear_right_shoulder": 6,
            "rear_right_leg": 10,
            "rear_right_foot": 11,
        }
        for joint_name, channel in expected_channels.items():
            self.assertEqual(repo_table.servos[joint_name].pca_channel, channel)

    def test_walk_pose_is_canonical_nominal_kinematics(self):
        solved = feet_to_joint_positions(NOMINAL_FEET)
        for expected, actual in zip(solved, WALK_POSE):
            self.assertAlmostEqual(expected, actual, places=12)
        self.assertEqual(NOMINAL_FEET["front_left"][2], NOMINAL_FEET["front_right"][2])
        self.assertEqual(NOMINAL_FEET["front_left"][2], NOMINAL_FEET["rear_left"][2])
        self.assertEqual(NOMINAL_FEET["front_left"][2], NOMINAL_FEET["rear_right"][2])

    def test_repo_trims_preserve_previous_physical_stand(self):
        repo_table = ServoCalibrationTable.from_file(
            ROOT / "config" / "servo_calibration.yaml"
        )
        # front_right_foot and rear_left_foot previously carried -7.15 and
        # +9.18 deg of trim, and this list encoded the stand those trims
        # produced (-1.206 and -0.921 here). Those two trims were levelling a
        # robot whose servo channels the OLD firmware was driving wrongly;
        # with the firmware corrected they biased the correct legs instead,
        # leaving the FR+RL trot diagonal 16.0 mm out of level against a 20 mm
        # step height. Both are now 0.0, so those two joints reference the
        # canonical WALK_POSE. The other ten still carry ~0.02 deg of real
        # mounting trim and keep their measured entries.
        previously_leveled_pose = [
            0.050, 0.499, -1.085,
            -0.050, 0.499, -1.0812,
            0.050, 0.696, -1.0812,
            -0.050, 0.696, -1.081,
        ]
        for joint_name, canonical_rad, previous_rad in zip(
            JOINT_NAMES, WALK_POSE, previously_leveled_pose
        ):
            servo = repo_table.servos[joint_name]
            previous_output = servo.neutral_deg + servo.direction * math.degrees(previous_rad)
            previous_output = max(servo.min_deg, min(servo.max_deg, previous_output))
            migrated_output = repo_table.ros_radians_to_servo_degrees(
                joint_name, canonical_rad
            )
            self.assertAlmostEqual(previous_output, migrated_output, delta=0.006)

    def test_repo_calibration_does_not_clamp_canonical_sit_transitions(self):
        repo_table = ServoCalibrationTable.from_file(
            ROOT / "config" / "servo_calibration.yaml"
        )
        poses = {
            "leg_foot_mid": LEG_FOOT_MID_POSE,
            "leg_foot_sit": LEG_FOOT_SIT_POSE,
            "sit": SIT_POSE,
        }
        for pose_name, pose in poses.items():
            with self.subTest(pose=pose_name):
                _frame, details = repo_table.channel_frame_from_positions(
                    named_positions_from_ordered(pose)
                )
                clamped_joints = [
                    detail["joint"] for detail in details if detail["clamped"]
                ]
                self.assertEqual(clamped_joints, [])

    def test_repo_calibration_does_not_clamp_natural_cartesian_sit(self):
        repo_table = ServoCalibrationTable.from_file(
            ROOT / "config" / "servo_calibration.yaml"
        )
        pose, diagnostics = feet_to_joint_positions_diagnostic(
            NOMINAL_FEET,
            height=0.145,
            body_x=-0.020,
            pitch=math.radians(-10.0),
        )
        self.assertEqual(diagnostics["projected_targets"], [])
        _frame, details = repo_table.channel_frame_from_positions(
            named_positions_from_ordered(pose)
        )
        self.assertEqual(
            [detail["joint"] for detail in details if detail["clamped"]],
            [],
        )

    def test_firmware_safe_start_matches_calibrated_stand(self):
        repo_table = ServoCalibrationTable.from_file(
            ROOT / "config" / "servo_calibration.yaml"
        )
        expected, _details = repo_table.channel_frame_from_positions(
            dict(zip(JOINT_NAMES, WALK_POSE))
        )
        firmware_path = (
            ROOT.parents[1]
            / "firmware"
            / "volt_arduino_pca9685"
            / "volt_arduino_pca9685.ino"
        )
        firmware = firmware_path.read_text(encoding="utf-8")
        match = re.search(
            r"const float CHANNEL_SAFE_START_DEG\[CHANNEL_COUNT\] = \{(.*?)\};",
            firmware,
            re.DOTALL,
        )
        self.assertIsNotNone(match)
        actual = [float(value) for value in re.findall(r"-?\d+(?:\.\d+)?", match.group(1))]
        self.assertEqual(len(actual), 12)
        for expected_degrees, actual_degrees in zip(expected, actual):
            self.assertAlmostEqual(expected_degrees, actual_degrees, delta=0.001)


if __name__ == "__main__":
    unittest.main()
