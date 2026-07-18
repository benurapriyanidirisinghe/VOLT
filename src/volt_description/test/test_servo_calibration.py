#!/usr/bin/env python3

import math
import sys
import unittest
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from volt_kinematics import JOINT_NAMES
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
            "front_left_shoulder": 1,
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


if __name__ == "__main__":
    unittest.main()
