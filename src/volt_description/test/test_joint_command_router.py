#!/usr/bin/env python3

"""Pure tests for joint-command validation and router ownership gates."""

import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def install_ros_import_stubs():
    """Provide just enough message/Node surface to import without ROS setup."""
    rclpy_module = types.ModuleType("rclpy")
    rclpy_node_module = types.ModuleType("rclpy.node")

    class Node:
        pass

    rclpy_node_module.Node = Node
    rclpy_module.node = rclpy_node_module

    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")

    class Float64MultiArray:
        def __init__(self):
            self.data = []

    class String:
        def __init__(self):
            self.data = ""

    std_msgs_msg_module.Float64MultiArray = Float64MultiArray
    std_msgs_msg_module.String = String
    std_msgs_module.msg = std_msgs_msg_module

    sys.modules["rclpy"] = rclpy_module
    sys.modules["rclpy.node"] = rclpy_node_module
    sys.modules["std_msgs"] = std_msgs_module
    sys.modules["std_msgs.msg"] = std_msgs_msg_module


try:
    from volt_joint_command_router import (  # noqa: E402
        JointCommandRouter,
        validate_joint_values,
    )
except ModuleNotFoundError as error:
    if error.name not in {"rclpy", "rclpy.node", "std_msgs", "std_msgs.msg"}:
        raise
    install_ros_import_stubs()
    sys.modules.pop("volt_joint_command_router", None)
    from volt_joint_command_router import (  # noqa: E402
        JointCommandRouter,
        validate_joint_values,
    )

from volt_kinematics import JOINT_NAMES  # noqa: E402


class NullLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return 1


class JointValueValidationTests(unittest.TestCase):
    def test_valid_values_are_returned_as_a_new_float_list(self):
        values = tuple(range(len(JOINT_NAMES)))
        converted = validate_joint_values(values)
        self.assertEqual(converted, [float(value) for value in values])
        self.assertTrue(all(type(value) is float for value in converted))
        self.assertIsNot(converted, values)

    def test_custom_expected_count_is_supported(self):
        self.assertEqual(
            validate_joint_values((1, 2.5, -3), expected_count=3),
            [1.0, 2.5, -3.0],
        )

    def test_wrong_value_count_is_rejected(self):
        for count in (0, len(JOINT_NAMES) - 1, len(JOINT_NAMES) + 1):
            with self.subTest(count=count):
                with self.assertRaises(ValueError):
                    validate_joint_values([0.0] * count)

    def test_nonnumeric_values_are_rejected(self):
        for bad_value in ("not-a-number", None, object()):
            with self.subTest(value=repr(bad_value)):
                values = [0.0] * len(JOINT_NAMES)
                values[4] = bad_value
                with self.assertRaises(ValueError):
                    validate_joint_values(values)

    def test_nan_and_infinity_are_rejected(self):
        for bad_value in (
            float("nan"),
            float("inf"),
            float("-inf"),
        ):
            with self.subTest(value=bad_value):
                values = [0.0] * len(JOINT_NAMES)
                values[7] = bad_value
                with self.assertRaises(ValueError):
                    validate_joint_values(values)


class RouterSourceGateTests(unittest.TestCase):
    def make_router(self, owner):
        router = JointCommandRouter.__new__(JointCommandRouter)
        router.owner = owner
        router.last_pose = [-1.0] * len(JOINT_NAMES)
        router.last_owner_command_time = 0.0
        router.now_seconds = lambda: 42.0
        router.get_logger = lambda: NullLogger()
        router.published = []
        router.publish_pose = lambda pose: router.published.append(list(pose))
        return router

    def test_wrong_owner_cannot_forward_motion_source(self):
        router = self.make_router("HOLD")
        message = SimpleNamespace(data=[0.1] * len(JOINT_NAMES))
        router.source_callback(message, "MOTION")
        self.assertEqual(router.published, [])
        self.assertEqual(router.last_pose, [-1.0] * len(JOINT_NAMES))

    def test_matching_owner_forwards_valid_source(self):
        router = self.make_router("MOTION")
        values = [index / 10.0 for index in range(len(JOINT_NAMES))]
        router.source_callback(SimpleNamespace(data=values), "MOTION")
        self.assertEqual(router.published, [values])
        self.assertEqual(router.last_pose, values)
        self.assertEqual(router.last_owner_command_time, 42.0)

    def test_invalid_matching_source_is_not_forwarded(self):
        router = self.make_router("MOTION")
        values = [0.0] * len(JOINT_NAMES)
        values[3] = float("nan")
        router.source_callback(SimpleNamespace(data=values), "MOTION")
        self.assertEqual(router.published, [])
        self.assertEqual(router.last_pose, [-1.0] * len(JOINT_NAMES))

    def test_publish_pose_fans_out_identical_canonical_values(self):
        router = JointCommandRouter.__new__(JointCommandRouter)
        router.output_publisher = RecordingPublisher()
        router.controller_publisher = RecordingPublisher()
        pose = [index / 20.0 for index in range(len(JOINT_NAMES))]

        router.publish_pose(pose)

        self.assertEqual(len(router.output_publisher.messages), 1)
        self.assertEqual(len(router.controller_publisher.messages), 1)
        self.assertEqual(
            list(router.output_publisher.messages[0].data),
            pose,
        )
        self.assertEqual(
            list(router.controller_publisher.messages[0].data),
            pose,
        )

    def test_startup_hold_never_publishes_synthetic_zero_pose(self):
        router = self.make_router("MOTION")
        router.last_pose = None
        router.owner_callback(SimpleNamespace(data="HOLD"))
        self.assertEqual(router.owner, "HOLD")
        self.assertEqual(router.published, [])

    def test_stale_motion_owner_returns_to_hold_and_republishes_last_pose(self):
        router = self.make_router("MOTION")
        router.stale_timeout = 0.5
        router.last_owner_command_time = 40.0
        router.last_pose = [0.25] * len(JOINT_NAMES)
        router.output_publisher = RecordingPublisher()
        router.controller_publisher = RecordingPublisher()
        router.status_publisher = RecordingPublisher()
        router.timer_callback()
        self.assertEqual(router.owner, "HOLD")
        self.assertEqual(router.published, [[0.25] * len(JOINT_NAMES)])

    def test_existing_hold_periodically_reasserts_last_pose(self):
        router = self.make_router("HOLD")
        router.stale_timeout = 0.5
        router.last_pose = [0.125] * len(JOINT_NAMES)
        router.output_publisher = RecordingPublisher()
        router.controller_publisher = RecordingPublisher()
        router.status_publisher = RecordingPublisher()

        router.timer_callback()

        self.assertEqual(router.owner, "HOLD")
        self.assertEqual(router.published, [[0.125] * len(JOINT_NAMES)])


if __name__ == "__main__":
    unittest.main()
