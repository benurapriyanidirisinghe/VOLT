#!/usr/bin/env python3

"""Unit tests for the ROS-to-Arduino serial bridge without a ROS graph."""

import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))


class _Logger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message, *args, **kwargs):
        self.infos.append(str(message))

    def warning(self, message, *args, **kwargs):
        self.warnings.append(str(message))

    def error(self, message, *args, **kwargs):
        self.errors.append(str(message))


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Time:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class _Clock:
    def __init__(self, node):
        self.node = node

    def now(self):
        return _Time(self.node._test_now_nanoseconds)


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Node:
    """Small rclpy Node stand-in sufficient for bridge construction."""

    def __init__(self, name):
        self.name = name
        self._parameters = {}
        self._logger = _Logger()
        self._test_now_nanoseconds = 1_000_000_000
        self._publishers = []
        self._subscriptions = []
        self._timers = []
        self._publisher_info_counts = {}

    def declare_parameter(self, name, default_value):
        self._parameters[name] = default_value
        return _Parameter(default_value)

    def get_parameter(self, name):
        return _Parameter(self._parameters[name])

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock(self)

    def create_publisher(self, *args, **kwargs):
        publisher = _Publisher()
        self._publishers.append(publisher)
        return publisher

    def create_subscription(self, *args, **kwargs):
        subscription = types.SimpleNamespace(args=args, kwargs=kwargs)
        self._subscriptions.append(subscription)
        return subscription

    def create_timer(self, *args, **kwargs):
        timer = types.SimpleNamespace(args=args, kwargs=kwargs)
        self._timers.append(timer)
        return timer

    def get_publishers_info_by_topic(self, topic):
        return [
            object()
            for _index in range(self._publisher_info_counts.get(topic, 1))
        ]


class _Float64MultiArray:
    def __init__(self, data=None):
        self.data = list(data or [])


class _String:
    def __init__(self, data=""):
        self.data = data


class _ColorRGBA:
    def __init__(self, r=0.0, g=0.0, b=0.0, a=0.0):
        self.r = r
        self.g = g
        self.b = b
        self.a = a


class _UInt8:
    def __init__(self, data=0):
        self.data = data


class _UInt32:
    def __init__(self, data=0):
        self.data = data


class _SerialException(Exception):
    pass


def _load_bridge_module():
    """Load the production bridge while temporarily providing ROS stubs."""
    rclpy_module = types.ModuleType("rclpy")
    rclpy_node_module = types.ModuleType("rclpy.node")
    rclpy_node_module.Node = _Node
    rclpy_module.node = rclpy_node_module

    ament_module = types.ModuleType("ament_index_python")
    ament_packages_module = types.ModuleType("ament_index_python.packages")
    ament_packages_module.get_package_share_directory = lambda _name: str(ROOT)
    ament_module.packages = ament_packages_module

    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")
    std_msgs_msg_module.Float64MultiArray = _Float64MultiArray
    std_msgs_msg_module.String = _String
    std_msgs_msg_module.ColorRGBA = _ColorRGBA
    std_msgs_msg_module.UInt8 = _UInt8
    std_msgs_msg_module.UInt32 = _UInt32
    std_msgs_module.msg = std_msgs_msg_module

    serial_module = types.ModuleType("serial")
    serial_module.SerialException = _SerialException
    serial_module.Serial = Mock(side_effect=AssertionError("serial port opened in unit test"))

    stubs = {
        "rclpy": rclpy_module,
        "rclpy.node": rclpy_node_module,
        "ament_index_python": ament_module,
        "ament_index_python.packages": ament_packages_module,
        "std_msgs": std_msgs_module,
        "std_msgs.msg": std_msgs_msg_module,
        "serial": serial_module,
    }
    missing = object()
    previous = {name: sys.modules.get(name, missing) for name in stubs}
    sys.modules.update(stubs)
    module_name = "_volt_serial_bridge_under_test"
    module_path = SCRIPTS / "volt_serial_bridge.py"
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    try:
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
    finally:
        sys.modules.pop(module_name, None)
        for name, original in previous.items():
            if original is missing:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original
    return module, serial_module


BRIDGE_MODULE, SERIAL_MODULE = _load_bridge_module()

from volt_kinematics import JOINT_NAMES
from volt_servo_calibration import CalibrationError
from volt_serial_protocol import format_frame_command


EXPECTED_JOINT_NAMES = [
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


class SerialBridgeTests(unittest.TestCase):
    def setUp(self):
        self.serial_factory = Mock(
            side_effect=AssertionError("serial port opened in unit test")
        )
        SERIAL_MODULE.Serial = self.serial_factory
        self.bridge = BRIDGE_MODULE.VoltSerialBridge()

    @staticmethod
    def message(values):
        return _Float64MultiArray(values)

    def set_time(self, seconds):
        self.bridge._test_now_nanoseconds = int(seconds * 1_000_000_000)

    def set_ready_hardware(self):
        self.bridge.dry_run = False
        self.bridge.hardware_enabled = True
        self.bridge.connected = True
        self.bridge.protocol.consume_response(
            "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0"
        )

    def set_safe_motion_status(self, monotonic_time):
        self.bridge.motion_state = "standing"
        self.bridge.motion_moving = False
        self.bridge.motion_step_in_place = False
        self.bridge.motion_arm_neutral_ready = True
        self.bridge.motion_controller_connected = True
        self.bridge.last_motion_status_time = monotonic_time

    @staticmethod
    def hardware_motion_status(velocity_fields):
        status = {
            "state": "standing",
            "moving": False,
            "step_in_place": False,
            "arm_neutral_ready": True,
            "controller_connected": True,
            "hardware_mode": True,
        }
        status.update(velocity_fields)
        return _String(BRIDGE_MODULE.json.dumps(status))

    def set_recent_frame(self, seconds=1.0, stable=True):
        self.set_time(seconds)
        self.bridge.last_frame = [90.0] * len(JOINT_NAMES)
        self.bridge.last_command_time = seconds
        self.bridge.frame_sequence = max(
            self.bridge.frame_sequence + 1,
            self.bridge.motion_owner_frame_floor + 1,
        )
        self.bridge.last_frame_owner = "MOTION"
        self.bridge.last_frame_owner_epoch = self.bridge.owner_epoch
        self.bridge.frame_stable_reference = list(self.bridge.last_frame)
        self.bridge.frame_stable_since = (
            seconds - self.bridge.arm_frame_settle_time - 0.01
        )
        self.bridge.frame_stable_samples = 2 if stable else 1

    def capture_protocol_commands(self):
        commands = []

        def send(command):
            commands.append(command)
            self.bridge.protocol.note_command_sent(command)
            self.bridge.last_protocol_command_time = BRIDGE_MODULE.time.monotonic()
            return True

        self.bridge.send_protocol_command = Mock(side_effect=send)
        return commands

    def test_requires_exactly_twelve_canonical_radians(self):
        self.assertEqual(JOINT_NAMES, EXPECTED_JOINT_NAMES)
        self.assertEqual(self.bridge.calibration.joint_order, EXPECTED_JOINT_NAMES)
        self.assertTrue(self.bridge.calibration_valid)

        frame, details = self.bridge.build_frame(self.message([0.0] * 12))
        self.assertEqual(len(frame), 12)
        self.assertEqual(len(details), 12)

        for count in (0, 1, 11, 13):
            with self.subTest(count=count):
                with self.assertRaises(CalibrationError):
                    self.bridge.build_frame(self.message([0.0] * count))

        for count in (11, 13):
            with self.subTest(callback_count=count):
                before = self.bridge.frames_rejected
                self.bridge.command_callback(self.message([0.0] * count))
                self.assertEqual(self.bridge.frames_rejected, before + 1)
        self.assertEqual(self.bridge.frames_sent, 0)

    def test_actual_calibration_produces_pca_channel_order(self):
        radians = [0.004 * (index - 5) for index in range(12)]
        frame, details = self.bridge.build_frame(self.message(radians))
        expected = [None] * 12

        for joint_name, radians_value in zip(JOINT_NAMES, radians):
            servo = self.bridge.calibration.servos[joint_name]
            raw = (
                servo.neutral_deg
                + servo.trim_deg
                + servo.direction * math.degrees(radians_value)
            )
            expected[servo.pca_channel] = max(
                servo.min_deg,
                min(servo.max_deg, raw),
            )

        self.assertEqual(
            {servo.pca_channel for servo in self.bridge.calibration.servos.values()},
            set(range(12)),
        )
        for channel, expected_value in enumerate(expected):
            self.assertAlmostEqual(frame[channel], expected_value)
        self.assertEqual([item["joint"] for item in details], JOINT_NAMES)

    def test_negative_servo_direction_is_applied_exactly_once(self):
        joint_name = "front_right_leg"
        joint_index = JOINT_NAMES.index(joint_name)
        servo = self.bridge.calibration.servos[joint_name]
        self.assertEqual(servo.direction, -1)

        zero_frame, _ = self.bridge.build_frame(self.message([0.0] * 12))
        radians = [0.0] * 12
        radians[joint_index] = 0.1
        moved_frame, _ = self.bridge.build_frame(self.message(radians))

        expected_delta = servo.direction * math.degrees(radians[joint_index])
        actual_delta = (
            moved_frame[servo.pca_channel] - zero_frame[servo.pca_channel]
        )
        self.assertAlmostEqual(actual_delta, expected_delta)
        self.assertAlmostEqual(
            moved_frame[servo.pca_channel],
            servo.neutral_deg + servo.trim_deg + expected_delta,
        )

    def test_dry_run_callback_emits_valid_frame_without_serial(self):
        self.bridge.dry_run = True
        self.bridge.hardware_enabled = True
        self.bridge.connect = Mock(
            side_effect=AssertionError("dry-run attempted to connect")
        )
        self.bridge.send_protocol_command = Mock(
            side_effect=AssertionError("dry-run attempted to write")
        )
        self.set_time(2.0)

        self.bridge.command_callback(self.message([0.0] * 12))

        self.assertEqual(self.bridge.frames_sent, 1)
        self.assertEqual(self.bridge.frames_rejected, 0)
        self.assertEqual(len(self.bridge.last_frame), 12)
        expected_line = format_frame_command(self.bridge.last_frame)
        dry_run_log = next(
            entry for entry in self.bridge.get_logger().infos
            if "\nFRAME " in entry
        )
        frame_line = dry_run_log.rsplit("\n", 1)[-1]
        self.assertEqual(frame_line, expected_line)
        tokens = frame_line.split()
        self.assertEqual(tokens[0], "FRAME")
        self.assertEqual(len(tokens[1:]), 12)
        self.assertTrue(all(math.isfinite(float(token)) for token in tokens[1:]))
        self.bridge.connect.assert_not_called()
        self.bridge.send_protocol_command.assert_not_called()
        self.serial_factory.assert_not_called()

    def test_hardware_disabled_connect_never_calls_serial_factory(self):
        self.bridge.dry_run = False
        self.bridge.hardware_enabled = False

        self.assertFalse(self.bridge.connect())
        self.assertIsNone(self.bridge.serial_port)
        self.assertFalse(self.bridge.connected)
        self.serial_factory.assert_not_called()

    def test_invalid_calibration_blocks_connection_and_command_output(self):
        self.bridge.calibration_file = str(
            ROOT / "config" / "does_not_exist.yaml"
        )
        self.bridge.load_calibration()
        self.assertFalse(self.bridge.calibration_valid)
        self.assertIsNone(self.bridge.calibration)
        self.assertTrue(self.bridge.calibration_error)

        self.bridge.dry_run = False
        self.bridge.hardware_enabled = True
        self.assertFalse(self.bridge.connect())
        self.serial_factory.assert_not_called()

        self.bridge.connect = Mock(
            side_effect=AssertionError("invalid calibration attempted to connect")
        )
        self.bridge.send_protocol_command = Mock(
            side_effect=AssertionError("invalid calibration attempted output")
        )
        self.bridge.command_callback(self.message([0.0] * 12))

        self.assertEqual(self.bridge.frames_rejected, 1)
        self.assertEqual(self.bridge.frames_sent, 0)
        self.assertEqual(self.bridge.last_frame, [])
        self.bridge.connect.assert_not_called()
        self.bridge.send_protocol_command.assert_not_called()
        self.serial_factory.assert_not_called()

    def test_max_send_rate_is_enforced_without_sleeping(self):
        self.assertEqual(self.bridge.max_send_rate, 60.0)
        self.bridge.dry_run = True
        self.bridge.hardware_enabled = True
        self.bridge.connect = Mock(
            side_effect=AssertionError("rate-limited dry-run attempted to connect")
        )
        min_period = 1.0 / self.bridge.max_send_rate
        self.bridge.last_send_time = 10.0

        self.set_time(10.0 + 0.5 * min_period)
        self.bridge.command_callback(self.message([0.0] * 12))
        self.assertEqual(self.bridge.frames_sent, 0)

        self.set_time(10.0 + min_period + 1e-6)
        self.bridge.command_callback(self.message([0.0] * 12))
        self.assertEqual(self.bridge.frames_sent, 1)

        self.bridge.command_callback(self.message([0.0] * 12))
        self.assertEqual(self.bridge.frames_sent, 1)
        self.bridge.connect.assert_not_called()
        self.serial_factory.assert_not_called()

    def test_ideal_100_hz_input_is_deadline_limited_to_60_not_50_hz(self):
        self.bridge.dry_run = True
        self.bridge.hardware_enabled = True
        for index in range(100):
            self.set_time(20.0 + index * 0.01)
            self.bridge.command_callback(self.message([0.0] * 12))

        self.assertEqual(self.bridge.frames_sent, 60)
        self.assertLessEqual(self.bridge.max_send_rate, 100.0)

    def test_nonfinite_joint_commands_are_rejected_without_output(self):
        self.bridge.dry_run = True
        for value in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(value=value):
                values = [0.0] * 12
                values[4] = value
                before = self.bridge.frames_rejected
                self.bridge.command_callback(self.message(values))
                self.assertEqual(self.bridge.frames_rejected, before + 1)
        self.assertEqual(self.bridge.frames_sent, 0)
        self.assertEqual(self.bridge.last_frame, [])
        self.serial_factory.assert_not_called()

    def test_router_owner_parser_accepts_json_and_key_value_only(self):
        parse = BRIDGE_MODULE.parse_command_owner_status
        self.assertEqual(parse("owner=MOTION controller_connected=1"), "MOTION")
        self.assertEqual(parse('{"owner": "hold"}'), "HOLD")
        self.assertEqual(parse('{"OWNER": "calibration"}'), "CALIBRATION")
        for payload in (
            "",
            "controller_connected=1",
            "owner=ROOT",
            '{"owner": null}',
            '{"not_owner": "MOTION"}',
            "{broken",
        ):
            with self.subTest(payload=payload):
                self.assertIsNone(parse(payload))

    def test_arm_requires_safe_motion_owner_and_recent_frame(self):
        self.set_ready_hardware()
        commands = self.capture_protocol_commands()
        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=100.0):
            self.set_safe_motion_status(100.0)
            self.bridge.serial_command_callback(_String("ARM"))
            self.assertEqual(commands, [])
            self.assertIn("MOTION ownership", self.bridge.last_error)

            self.bridge.command_router_status_callback(
                _String("owner=MOTION controller_connected=1")
            )
            self.bridge.serial_command_callback(_String("ARM"))
            self.assertEqual(commands, [])
            self.assertIn("12-joint frame", self.bridge.last_error)

            self.set_recent_frame()
            self.bridge.serial_command_callback(_String("ARM"))

        self.assertEqual(commands, ["ARM"])
        self.assertEqual(self.bridge.protocol.pending_command, "ARM")

    def test_unversioned_generic_pong_cannot_arm_normal_hardware_but_hold_works(self):
        self.bridge.dry_run = False
        self.bridge.hardware_enabled = True
        self.bridge.connected = True
        self.bridge.protocol.consume_response("OK PONG")
        commands = self.capture_protocol_commands()
        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=102.0):
            self.set_safe_motion_status(102.0)
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.set_recent_frame(2.0)
            self.bridge.serial_command_callback(_String("ARM"))
            self.assertEqual(commands, [])
            self.assertIn("FW=VOLT_PCA9685", self.bridge.last_error)

            # Even if an old image was already armed, compatibility still
            # blocks motion while the fail-safe command path stays available.
            self.bridge.protocol.armed = True
            self.bridge.protocol.motion_inhibited = False
            self.assertFalse(self.bridge.hardware_stream_allowed())
            self.bridge.serial_command_callback(_String("HOLD"))

        self.assertEqual(commands, ["HOLD"])
        self.assertEqual(self.bridge.protocol.pending_command, "HOLD")
        self.assertFalse(self.bridge.hardware_stream_allowed())

    def test_protocol_below_configured_minimum_cannot_arm_normal_hardware(self):
        self.bridge.dry_run = False
        self.bridge.hardware_enabled = True
        self.bridge.connected = True
        self.bridge.protocol.consume_response(
            "OK PONG FW=VOLT_PCA9685 PROTO=1 MAX_DPS=120.0"
        )
        commands = self.capture_protocol_commands()
        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=102.5):
            self.set_safe_motion_status(102.5)
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.set_recent_frame(2.0)
            self.bridge.serial_command_callback(_String("ARM"))

        self.assertEqual(commands, [])
        self.assertFalse(self.bridge.firmware_capability_compatible())
        self.assertIn("PROTO>=2", self.bridge.last_error)

    def test_versioned_current_pong_can_arm_normal_hardware(self):
        self.bridge.dry_run = False
        self.bridge.hardware_enabled = True
        self.bridge.connected = True
        self.bridge.protocol.consume_response(
            "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0"
        )
        commands = self.capture_protocol_commands()
        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=103.0):
            self.set_safe_motion_status(103.0)
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.set_recent_frame(2.0)
            self.bridge.serial_command_callback(_String("ARM"))

        self.assertEqual(commands, ["ARM"])
        self.assertTrue(self.bridge.firmware_capability_compatible())

    def test_firmware_max_dps_must_cover_hardware_motion_velocity(self):
        self.bridge.dry_run = False
        self.bridge.hardware_enabled = True
        self.bridge.connected = True
        self.bridge.protocol.consume_response(
            "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=30.0"
        )
        limits = {joint_name: 80.0 for joint_name in JOINT_NAMES}
        limits[JOINT_NAMES[-1]] = 100.0
        commands = self.capture_protocol_commands()

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=104.0):
            self.bridge.motion_status_callback(
                self.hardware_motion_status(
                    {"effective_joint_velocity_limits_deg_s": limits}
                )
            )
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.set_recent_frame(2.0)

            self.assertEqual(self.bridge.motion_required_max_dps, 100.0)
            self.assertTrue(self.bridge.motion_required_max_dps_known)
            self.assertFalse(self.bridge.firmware_capability_compatible())
            self.bridge.serial_command_callback(_String("ARM"))
            self.assertEqual(commands, [])
            self.assertIn("MAX_DPS>=100.0", self.bridge.last_error)
            self.assertIn("reported MAX_DPS=30.0", self.bridge.last_error)

            self.bridge.protocol.consume_response(
                "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0"
            )
            self.assertTrue(self.bridge.firmware_capability_compatible())
            self.bridge.serial_command_callback(_String("ARM"))

        self.assertEqual(commands, ["ARM"])

    def test_hardware_motion_velocity_contract_fails_closed_when_malformed(self):
        self.set_ready_hardware()
        malformed_fields = (
            {},
            {"joint_velocity_limit_deg_s": 100.0},
            {"joint_velocity_limit_deg_s": True},
            {"joint_velocity_limit_deg_s": float("nan")},
            {
                "effective_joint_velocity_limits_deg_s": {
                    joint_name: 100.0 for joint_name in JOINT_NAMES[:-1]
                }
            },
        )

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=105.0):
            for fields in malformed_fields:
                with self.subTest(fields=fields):
                    self.bridge.motion_status_callback(
                        self.hardware_motion_status(fields)
                    )
                    self.assertTrue(self.bridge.motion_hardware_mode)
                    self.assertFalse(self.bridge.motion_required_max_dps_known)
                    self.assertTrue(math.isinf(self.bridge.motion_required_max_dps))
                    self.assertFalse(self.bridge.firmware_capability_compatible())

            self.bridge.publish_status()

        payload = self.bridge.status_publisher.messages[-1].data
        self.assertIn("required_max_dps=inf", payload)
        self.assertIn("required_max_dps_known=0", payload)
        self.assertIn("motion_hardware_mode=1", payload)
        self.assertIn("firmware_compatible=0", payload)

    def test_higher_live_velocity_contract_holds_incompatible_armed_firmware(self):
        self.set_ready_hardware()
        limits = {joint_name: 130.0 for joint_name in JOINT_NAMES}
        commands = self.capture_protocol_commands()
        self.bridge.read_available = Mock()

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=106.0):
            self.bridge.motion_status_callback(
                self.hardware_motion_status(
                    {
                        "effective_joint_velocity_limits_deg_s": {
                            joint_name: 100.0 for joint_name in JOINT_NAMES
                        }
                    }
                )
            )
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.assertTrue(self.bridge.firmware_capability_compatible())
            self.bridge.protocol.armed = True
            self.bridge.protocol.motion_inhibited = False

            self.bridge.motion_status_callback(
                self.hardware_motion_status(
                    {"effective_joint_velocity_limits_deg_s": limits}
                )
            )
            self.bridge.timer_callback()

        self.assertEqual(commands, ["HOLD"])
        self.assertTrue(self.bridge.protocol.motion_inhibited)
        self.assertIn("required_max_dps=130.0", self.bridge.last_error)
        self.assertIn("reported MAX_DPS=120.0", self.bridge.last_error)

    def test_stale_hardware_velocity_contract_holds_armed_firmware(self):
        self.set_ready_hardware()
        commands = self.capture_protocol_commands()
        self.bridge.read_available = Mock()

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=107.0):
            self.bridge.motion_status_callback(
                self.hardware_motion_status(
                    {
                        "effective_joint_velocity_limits_deg_s": {
                            joint_name: 100.0 for joint_name in JOINT_NAMES
                        }
                    }
                )
            )
            self.assertTrue(self.bridge.firmware_capability_compatible())
            self.bridge.protocol.armed = True
            self.bridge.protocol.motion_inhibited = False

        stale_time = 107.0 + self.bridge.motion_status_timeout + 0.01
        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=stale_time):
            self.bridge.command_owner = "MOTION"
            self.bridge.last_owner_status_time = stale_time
            self.bridge.timer_callback()

        self.assertEqual(commands, ["HOLD"])
        self.assertTrue(self.bridge.protocol.motion_inhibited)
        self.assertIn("velocity requirement is stale", self.bridge.last_error)

    def test_certified_neutral_hold_is_parsed_as_safe_motion_status(self):
        payload = _String(
            '{"state":"hold","moving":false,"step_in_place":false,'
            '"arm_neutral_ready":true,"controller_connected":true}'
        )
        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=105.0):
            self.bridge.motion_status_callback(payload)
            self.assertTrue(self.bridge.motion_safe_to_arm())

    def test_duplicate_stack_publishers_block_arm(self):
        self.set_ready_hardware()
        commands = self.capture_protocol_commands()
        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=110.0):
            self.set_safe_motion_status(110.0)
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.set_recent_frame(2.0)
            self.bridge._publisher_info_counts["/volt/status"] = 2

            self.bridge.serial_command_callback(_String("ARM"))

        self.assertEqual(commands, [])
        self.assertIn("duplicate VOLT stacks", self.bridge.last_error)
        self.assertEqual(
            self.bridge.stack_conflict_topics,
            ("/volt/status",),
        )

    def test_stale_or_nonfinite_frame_cannot_unlock_arm(self):
        self.set_ready_hardware()
        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=101.0):
            self.set_safe_motion_status(101.0)
            self.bridge.command_router_status_callback(_String("owner=MOTION"))

            self.set_recent_frame(2.0)
            self.bridge.last_command_time = 1.0
            self.assertFalse(self.bridge.frame_ready_to_arm())

            self.bridge.last_command_time = 2.0
            self.bridge.last_frame[4] = float("nan")
            self.assertFalse(self.bridge.frame_ready_to_arm())

            self.bridge.last_frame[4] = 90.0
            self.assertTrue(self.bridge.frame_ready_to_arm())

    def test_arm_frame_requires_new_stable_motion_owner_generation(self):
        self.set_ready_hardware()
        zeros = self.message([0.0] * len(JOINT_NAMES))

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=150.0):
            self.set_safe_motion_status(150.0)
            self.bridge.command_router_status_callback(_String("owner=HOLD"))
            self.set_time(1.0)
            self.bridge.command_callback(zeros)
            hold_frame_sequence = self.bridge.frame_sequence
            self.assertEqual(self.bridge.last_frame_owner, "HOLD")

            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            motion_epoch = self.bridge.owner_epoch
            self.assertEqual(
                self.bridge.motion_owner_frame_floor,
                hold_frame_sequence,
            )
            self.assertFalse(self.bridge.frame_ready_to_arm())

            self.set_time(1.10)
            self.bridge.command_callback(zeros)
            self.assertEqual(self.bridge.last_frame_owner, "MOTION")
            self.assertEqual(self.bridge.last_frame_owner_epoch, motion_epoch)
            self.assertEqual(self.bridge.frame_stable_samples, 1)
            self.assertFalse(self.bridge.frame_ready_to_arm())

            self.set_time(1.20)
            self.bridge.command_callback(zeros)
            self.assertEqual(self.bridge.frame_stable_samples, 2)
            self.assertFalse(self.bridge.frame_ready_to_arm())

            self.set_time(1.36)
            self.bridge.command_callback(zeros)
            self.assertTrue(self.bridge.frame_is_stable())
            self.assertTrue(self.bridge.frame_ready_to_arm())
            ready_sequence = self.bridge.frame_sequence
            self.bridge.publish_status()
            status = self.bridge.status_publisher.messages[-1].data
            self.assertIn("frame_ready=1", status)
            self.assertIn("frame_seq=%d" % ready_sequence, status)
            self.assertIn("frame_owner=MOTION", status)
            self.assertIn("frame_owner_epoch=%d" % motion_epoch, status)
            self.assertIn("frame_stable=1", status)
            self.assertIn("frame_stable_samples=3", status)

            self.bridge.command_router_status_callback(_String("owner=HOLD"))
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.assertGreater(self.bridge.owner_epoch, motion_epoch)
            self.assertEqual(
                self.bridge.motion_owner_frame_floor,
                ready_sequence,
            )
            self.assertFalse(self.bridge.frame_ready_to_arm())

    def test_out_of_tolerance_frame_restarts_stability_settle(self):
        self.set_ready_hardware()
        zeros = [0.0] * len(JOINT_NAMES)
        moved = list(zeros)
        moved[0] = 0.03

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=175.0):
            self.set_safe_motion_status(175.0)
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.set_time(1.0)
            self.bridge.command_callback(self.message(zeros))
            initial_stable_since = self.bridge.frame_stable_since

            self.set_time(1.30)
            self.bridge.command_callback(self.message(moved))
            self.assertGreater(
                self.bridge.frame_stable_since,
                initial_stable_since,
            )
            self.assertEqual(self.bridge.frame_stable_samples, 1)
            self.assertFalse(self.bridge.frame_ready_to_arm())

    def test_wire_token_change_restarts_stability_even_within_tolerance(self):
        self.set_ready_hardware()
        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=176.0):
            self.set_safe_motion_status(176.0)
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.bridge.cache_frame([89.49] * 12, [], 1.0)
            initial_stable_since = self.bridge.frame_stable_since
            self.bridge.cache_frame([89.51] * 12, [], 1.2)

        self.assertGreater(
            self.bridge.frame_stable_since,
            initial_stable_since,
        )
        self.assertEqual(self.bridge.frame_stable_samples, 1)
        self.assertFalse(self.bridge.frame_is_stable())

    def test_arm_ack_race_rechecks_moving_interlock_and_holds(self):
        self.set_ready_hardware()
        commands = self.capture_protocol_commands()

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=180.0):
            self.set_safe_motion_status(180.0)
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.set_recent_frame(2.0)
            self.bridge.protocol.note_command_sent("ARM")
            self.bridge.motion_moving = True
            self.bridge.handle_serial_line("OK ARM ARMED=1")

        self.assertEqual(commands, ["HOLD"])
        self.assertTrue(self.bridge.protocol.armed)
        self.assertTrue(self.bridge.protocol.motion_inhibited)
        self.assertEqual(self.bridge.protocol.pending_command, "HOLD")
        self.assertIn("interlocks", self.bridge.last_error)

    def test_arm_ack_race_rechecks_stale_frame_and_holds(self):
        self.set_ready_hardware()
        commands = self.capture_protocol_commands()

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=190.0):
            self.set_safe_motion_status(190.0)
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.set_recent_frame(2.0)
            self.set_time(2.0 + self.bridge.arm_frame_timeout + 0.01)
            self.bridge.protocol.note_command_sent("ARM")
            self.bridge.handle_serial_line("OK STATUS ARMED=1 OUTPUT=1")

        self.assertEqual(commands, ["HOLD"])
        self.assertTrue(self.bridge.protocol.motion_inhibited)
        self.assertEqual(self.bridge.protocol.pending_command, "HOLD")

    def test_failed_initial_cached_frame_immediately_holds(self):
        self.set_ready_hardware()
        commands = []

        def fail_frame(command):
            commands.append(command)
            if command.startswith("FRAME "):
                return False
            self.bridge.protocol.note_command_sent(command)
            return True

        self.bridge.send_protocol_command = Mock(side_effect=fail_frame)
        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=195.0):
            self.set_safe_motion_status(195.0)
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.set_recent_frame(2.0)
            self.bridge.protocol.note_command_sent("ARM")
            self.bridge.handle_serial_line("OK ARM ARMED=1")

        self.assertTrue(commands[0].startswith("FRAME "))
        self.assertEqual(commands[1], "HOLD")
        self.assertEqual(self.bridge.protocol.pending_command, "HOLD")
        self.assertTrue(self.bridge.protocol.motion_inhibited)

    def test_late_arm_ack_preserves_pending_disable(self):
        self.set_ready_hardware()
        self.bridge.protocol.note_command_sent("ARM")
        self.bridge.protocol.note_command_sent("DISABLE")
        commands = self.capture_protocol_commands()

        self.bridge.handle_serial_line("OK ARM ARMED=1")

        self.assertEqual(commands, [])
        self.assertTrue(self.bridge.protocol.armed)
        self.assertTrue(self.bridge.protocol.motion_inhibited)
        self.assertEqual(self.bridge.protocol.pending_command, "DISABLE")

    def test_ordinary_armed_status_does_not_apply_stopped_pose_gate(self):
        self.set_ready_hardware()
        self.bridge.protocol.armed = True
        self.bridge.protocol.motion_inhibited = False
        commands = self.capture_protocol_commands()

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=199.0):
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.bridge.motion_state = "standing"
            self.bridge.motion_moving = True
            self.bridge.motion_controller_connected = True
            self.bridge.last_motion_status_time = 199.0
            self.bridge.handle_serial_line(
                "OK STATUS ARMED=1 OUTPUT=1 LAST_CMD_MS=20"
            )

        self.assertEqual(commands, [])
        self.assertTrue(self.bridge.protocol.can_stream_frames)
        self.assertEqual(self.bridge.protocol.pending_command, "")

    def test_owner_departure_immediately_cancels_arm_and_sends_hold(self):
        self.set_ready_hardware()
        self.bridge.protocol.armed = True
        self.bridge.protocol.motion_inhibited = False
        self.bridge.arm_requested = True
        commands = self.capture_protocol_commands()

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=200.0):
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=200.1):
            self.bridge.command_router_status_callback(_String("owner=HOLD"))

        self.assertFalse(self.bridge.arm_requested)
        self.assertEqual(commands, ["HOLD"])
        self.assertEqual(self.bridge.protocol.pending_command, "HOLD")
        self.assertTrue(self.bridge.protocol.motion_inhibited)

    def test_stale_owner_status_sends_hold_before_more_frames(self):
        self.set_ready_hardware()
        self.bridge.protocol.armed = True
        self.bridge.protocol.motion_inhibited = False
        self.bridge.command_owner = "MOTION"
        self.bridge.last_owner_status_time = 300.0
        self.bridge.read_available = Mock()
        commands = self.capture_protocol_commands()

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=301.01):
            self.bridge.timer_callback()

        self.assertEqual(commands, ["HOLD"])
        self.assertTrue(self.bridge.protocol.motion_inhibited)
        self.assertIn("stale", self.bridge.last_error.lower())

    def test_live_frames_continue_while_armed_motion_is_moving(self):
        self.set_ready_hardware()
        self.bridge.protocol.armed = True
        self.bridge.protocol.motion_inhibited = False
        self.bridge.motion_state = "standing"
        self.bridge.motion_moving = True
        self.bridge.motion_controller_connected = True
        commands = self.capture_protocol_commands()
        self.set_time(2.0)

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=400.0):
            self.bridge.last_motion_status_time = 400.0
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.assertFalse(self.bridge.motion_safe_to_arm())
            self.bridge.command_callback(self.message([0.0] * 12))

        self.assertEqual(len(commands), 1)
        self.assertTrue(commands[0].startswith("FRAME "))
        self.assertEqual(self.bridge.frames_sent, 1)

    def test_cached_frame_requires_both_arm_pose_and_owner_interlocks(self):
        self.set_ready_hardware()
        self.bridge.protocol.armed = True
        self.bridge.protocol.motion_inhibited = False
        commands = self.capture_protocol_commands()

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=500.0):
            self.set_safe_motion_status(500.0)
            self.bridge.command_router_status_callback(_String("owner=MOTION"))
            self.set_recent_frame(2.0)
            self.bridge.command_owner = "HOLD"
            self.assertFalse(self.bridge.send_cached_frame())

            self.bridge.command_owner = "MOTION"
            self.assertTrue(self.bridge.send_cached_frame())

        self.assertEqual(len(commands), 1)
        self.assertTrue(commands[0].startswith("FRAME "))

    def test_explicit_calibration_mode_bypasses_motion_owner_interlocks(self):
        self.set_ready_hardware()
        self.bridge.protocol.reset()
        self.bridge.protocol.consume_response("OK PONG")
        self.bridge.require_motion_safe_to_arm = False
        commands = self.capture_protocol_commands()

        self.bridge.serial_command_callback(_String("ARM"))

        self.assertEqual(commands, ["ARM"])
        self.assertTrue(self.bridge.owner_allows_hardware_output())
        self.assertFalse(self.bridge.firmware_capability_compatible())
        self.assertTrue(self.bridge.firmware_capability_allows_motion())

    def test_explicit_calibration_arm_ack_does_not_require_cached_walk_frame(self):
        self.set_ready_hardware()
        self.bridge.require_motion_safe_to_arm = False
        commands = self.capture_protocol_commands()
        self.bridge.protocol.note_command_sent("ARM")

        self.bridge.handle_serial_line("OK ARM ARMED=1")

        self.assertEqual(commands, [])
        self.assertTrue(self.bridge.protocol.can_stream_frames)

    def test_servo_command_is_blocked_for_arbitrary_owner_in_normal_mode(self):
        self.set_ready_hardware()
        self.bridge.protocol.armed = True
        self.bridge.protocol.motion_inhibited = False
        self.bridge.command_owner = "MANUAL"
        self.bridge.last_owner_status_time = BRIDGE_MODULE.time.monotonic()
        commands = self.capture_protocol_commands()

        self.bridge.serial_command_callback(_String("SERVO 0 90"))

        self.assertEqual(commands, ["HOLD"])
        self.assertIn("SERVO blocked", self.bridge.last_error)

    def test_status_exposes_owner_freshness_and_effective_streaming(self):
        self.bridge.command_owner = "MOTION"
        self.bridge.protocol.consume_response(
            "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0"
        )
        self.bridge.protocol.armed = True
        self.bridge.protocol.motion_inhibited = False
        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=600.0):
            self.bridge.last_owner_status_time = 600.0
            self.bridge.publish_status()

        payload = self.bridge.status_publisher.messages[-1].data
        self.assertIn("owner=MOTION", payload)
        self.assertIn("owner_fresh=1", payload)
        self.assertIn("owner_required=1", payload)
        self.assertIn("owner_allowed=1", payload)
        self.assertIn("firmware_id=VOLT_PCA9685", payload)
        self.assertIn("protocol_version=2", payload)
        self.assertIn("required_protocol_version=2", payload)
        self.assertIn("max_dps=120.0", payload)
        self.assertIn("firmware_compatible=1", payload)
        self.assertIn("frame_ready=0", payload)
        self.assertIn("frame_age=-1.000", payload)
        self.assertIn("frame_seq=0", payload)
        self.assertIn("frame_owner=UNKNOWN", payload)
        self.assertIn("frame_stable=0", payload)
        self.assertIn("frame_stable_age=-1.000", payload)
        self.assertIn("frame_stable_samples=0", payload)
        self.assertIn("streaming=1", payload)

    def test_face_topics_use_standard_humble_message_types(self):
        subscriptions = {
            args[1]: args[0]
            for args, _kwargs in (
                (subscription.args, subscription.kwargs)
                for subscription in self.bridge._subscriptions
            )
        }
        self.assertIs(subscriptions["/volt/face/expression"], _String)
        self.assertIs(subscriptions["/volt/face/color"], _ColorRGBA)
        self.assertIs(
            subscriptions["/volt/face/alternate_color"],
            _ColorRGBA,
        )
        self.assertIs(subscriptions["/volt/face/brightness"], _UInt8)
        self.assertIs(subscriptions["/volt/face/effect"], _String)
        self.assertIs(subscriptions["/volt/face/speed"], _UInt32)

    def test_rapid_face_changes_are_clamped_deduplicated_and_coalesced(self):
        self.bridge.face_expression_callback(_String("happy"))
        self.bridge.face_expression_callback(_String("happy"))
        for expression in ("sad", "angry", "idle"):
            self.bridge.face_expression_callback(_String(expression))
        self.bridge.face_color_callback(_ColorRGBA(-1.0, 0.5, 2.0, 0.25))
        self.bridge.face_alternate_color_callback(
            _ColorRGBA(1.0, 0.0, 180.0 / 255.0, 1.0)
        )
        self.bridge.face_brightness_callback(_UInt8(80))
        self.bridge.face_effect_callback(_String("breathe"))
        self.bridge.face_speed_callback(_UInt32(1))

        self.assertEqual(self.bridge.face_desired["expression"], "idle")
        self.assertEqual(self.bridge.face_desired["color"], (0, 128, 255))
        self.assertEqual(
            self.bridge.face_desired["alternate_color"],
            (255, 0, 180),
        )
        self.assertEqual(self.bridge.face_desired["brightness"], 80)
        self.assertEqual(self.bridge.face_desired["effect"], "breathe")
        self.assertEqual(self.bridge.face_desired["speed"], 10)
        self.assertEqual(self.bridge.face_dirty.count("expression"), 1)
        self.assertEqual(len(self.bridge.face_dirty), 6)

    def test_shuffled_snapshot_callbacks_send_in_fixed_face_setting_order(self):
        self.set_ready_hardware()
        commands = self.capture_protocol_commands()

        # Deliberately reverse the GUI's semantic snapshot order. Independent
        # ROS subscriptions may deliver callbacks in any interleaving.
        shuffled_callbacks = (
            lambda: self.bridge.face_effect_callback(_String("pulse")),
            lambda: self.bridge.face_speed_callback(_UInt32(1200)),
            lambda: self.bridge.face_brightness_callback(_UInt8(80)),
            lambda: self.bridge.face_alternate_color_callback(
                _ColorRGBA(1.0, 0.0, 180.0 / 255.0, 1.0)
            ),
            lambda: self.bridge.face_color_callback(
                _ColorRGBA(1.0, 0.5, 0.0, 1.0)
            ),
            lambda: self.bridge.face_expression_callback(_String("sad")),
            lambda: self.bridge.face_expression_callback(_String("happy")),
        )
        for index, callback in enumerate(shuffled_callbacks):
            callback()
            # Even with a timer callback between every subscription callback,
            # the one-cycle barrier keeps the partial snapshot off the wire.
            self.assertFalse(self.bridge.service_face_queue(100.0 + index))
            self.assertEqual(commands, [])

        self.assertEqual(
            self.bridge.face_dirty,
            [
                "effect",
                "speed",
                "brightness",
                "alternate_color",
                "color",
                "expression",
            ],
        )
        self.assertEqual(len(self.bridge.face_dirty), 6)
        self.assertEqual(self.bridge.face_desired["expression"], "happy")

        acknowledgements = (
            "OK FACE happy",
            "OK LED COLOR 255 128 0",
            "OK LED COLOR_B 255 0 180",
            "OK LED BRIGHTNESS 80 EFFECTIVE=80",
            "OK LED SPEED 1200",
            "OK LED EFFECT pulse",
        )
        for index, acknowledgement in enumerate(acknowledgements):
            self.assertTrue(self.bridge.service_face_queue(106.0 + index))
            self.bridge.handle_serial_line(acknowledgement)

        self.assertEqual(
            commands,
            [
                "FACE happy",
                "LED COLOR 255 128 0",
                "LED COLOR_B 255 0 180",
                "LED BRIGHTNESS 80",
                "LED SPEED 1200",
                "LED EFFECT pulse",
            ],
        )
        self.assertEqual(self.bridge.face_dirty, [])

    def test_face_queue_waits_for_ready_and_only_sends_one_command_per_ack(self):
        self.bridge.dry_run = False
        self.bridge.hardware_enabled = True
        self.bridge.connected = True
        commands = self.capture_protocol_commands()
        self.bridge.face_expression_callback(_String("happy"))
        self.bridge.face_brightness_callback(_UInt8(80))

        self.assertFalse(self.bridge.send_next_face_command(100.0))
        self.bridge.handle_serial_line(
            "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0 FACE_SUPPORTED=1"
        )
        self.assertTrue(self.bridge.send_next_face_command(100.0))
        self.assertEqual(commands, ["FACE happy"])
        self.assertFalse(self.bridge.send_next_face_command(101.0))

        self.bridge.handle_serial_line("OK FACE happy")
        self.assertTrue(self.bridge.send_next_face_command(101.0))
        self.assertEqual(commands, ["FACE happy", "LED BRIGHTNESS 80"])
        self.bridge.handle_serial_line("OK LED BRIGHTNESS 80")
        self.assertTrue(self.bridge.send_next_face_command(102.0))
        self.assertEqual(commands[-1], "LED STATUS")

    def test_host_sync_follows_snapshot_acks_and_led_status_confirmation(self):
        self.bridge.dry_run = False
        self.bridge.hardware_enabled = True
        self.bridge.connected = True
        commands = self.capture_protocol_commands()
        self.bridge.handle_serial_line(
            "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0 "
            "FACE_SUPPORTED=1 HOST_SYNC_REQUIRED=1 HOST_PING=1 "
            "HOST_SNAPSHOT=0 HOST_SYNCED=0"
        )
        self.bridge.face_expression_callback(_String("happy"))

        self.assertTrue(self.bridge.send_next_face_command(100.0))
        self.assertEqual(commands, ["FACE happy"])
        self.bridge.handle_serial_line("OK FACE happy")
        self.assertTrue(self.bridge.send_next_face_command(101.0))
        self.assertEqual(commands, ["FACE happy", "LED STATUS"])

        # No terminal marker may precede the verification response.
        self.assertFalse(self.bridge.send_next_face_command(101.5))
        self.bridge.handle_serial_line(
            "OK LED STATUS FACE_SUPPORTED=1 HOST_SYNC_REQUIRED=1 "
            "HOST_PING=1 HOST_SNAPSHOT=1 HOST_SYNCED=0 LED_ENABLED=1 "
            "LED_COLOR=255,180,20 LED_BRIGHTNESS=80 "
            "LED_EFFECT=pulse LED_SPEED_MS=1600 FACE=happy"
        )
        self.assertTrue(self.bridge.send_next_face_command(103.0))
        self.assertEqual(commands[-1], "HOST SYNC")
        self.assertTrue(self.bridge.host_sync_inflight)
        self.assertFalse(self.bridge.face_is_synced())

        self.bridge.handle_serial_line("OK HOST SYNC HOST_SYNCED=1")
        self.assertFalse(self.bridge.host_sync_inflight)
        self.assertTrue(self.bridge.protocol.host_synced)
        self.assertTrue(self.bridge.face_is_synced())

    def test_new_ready_banner_replays_snapshot_and_timer_repings_host(self):
        self.set_ready_hardware()
        self.bridge.face_expression_callback(_String("thinking"))
        self.bridge.face_dirty = []
        commands = self.capture_protocol_commands()
        self.bridge.handle_serial_line(
            "OK VOLT_PCA9685_READY FW=VOLT_PCA9685 PROTO=2 "
            "MAX_DPS=120.0 FACE_SUPPORTED=1 LED_COUNT=8 "
            "HOST_SYNC_REQUIRED=1 HOST_PING=0 HOST_SNAPSHOT=0 "
            "HOST_SYNCED=0 DISARMED OUTPUT_DISABLED"
        )
        self.assertTrue(self.bridge.protocol.ready)
        self.assertFalse(self.bridge.protocol.host_ping_seen)
        self.assertEqual(self.bridge.face_dirty, ["expression"])

        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=100.0):
            self.bridge.last_ping_time = 0.0
            self.bridge.timer_callback()
        self.assertIn("PING", commands)

    def test_host_sync_is_visual_only_and_legacy_firmware_is_not_probed(self):
        self.set_ready_hardware()
        self.bridge.protocol.armed = True
        self.bridge.protocol.motion_inhibited = False
        self.bridge.face_expression_callback(_String("happy"))
        self.bridge.face_dirty = []
        self.bridge.face_status_pending = False
        self.bridge.face_status_request_time = 0.0
        self.bridge.protocol.face_status_seen = True
        commands = self.capture_protocol_commands()

        self.assertFalse(self.bridge.send_next_face_command(100.0))
        self.assertNotIn("HOST SYNC", commands)
        self.assertTrue(self.bridge.protocol.can_stream_frames)

        self.bridge.protocol.host_sync_required = True
        self.bridge.protocol.host_ping_seen = True
        self.bridge.protocol.host_snapshot_seen = True
        self.assertTrue(self.bridge.send_next_face_command(101.0))
        self.bridge.handle_serial_line("ERR HOST SNAPSHOT_REQUIRED")
        self.assertTrue(self.bridge.protocol.can_stream_frames)
        self.assertFalse(self.bridge.host_sync_inflight)
        self.assertEqual(
            self.bridge.host_sync_error,
            "ERR HOST SNAPSHOT_REQUIRED",
        )

    def test_unsynced_visual_host_state_does_not_block_arm(self):
        self.set_ready_hardware()
        self.bridge.protocol.host_sync_required = True
        self.bridge.protocol.host_ping_seen = True
        self.bridge.protocol.host_snapshot_seen = False
        self.bridge.protocol.host_synced = False
        self.bridge.require_motion_safe_to_arm = False
        self.bridge.arm_requested = True
        commands = self.capture_protocol_commands()

        self.assertTrue(self.bridge.send_arm_if_ready())
        self.assertEqual(commands, ["ARM"])

    def test_no_desired_snapshot_keeps_new_firmware_in_loading_state(self):
        self.bridge.dry_run = False
        self.bridge.hardware_enabled = True
        self.bridge.connected = True
        commands = self.capture_protocol_commands()
        self.bridge.handle_serial_line(
            "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0 "
            "FACE_SUPPORTED=1 HOST_SYNC_REQUIRED=1 HOST_PING=1 "
            "HOST_SNAPSHOT=0 HOST_SYNCED=0"
        )
        self.bridge.face_status_pending = False
        self.bridge.face_status_request_time = 0.0
        self.bridge.protocol.face_status_seen = True

        self.assertFalse(self.bridge.send_next_face_command(100.0))
        self.assertNotIn("HOST SYNC", commands)
        self.bridge.publish_status()
        payload = self.bridge.status_publisher.messages[-1].data
        self.assertIn("host_sync_state=loading", payload)
        self.assertIn("face_loading=1", payload)

    def test_brightness_ack_with_effective_field_clears_exact_inflight_value(self):
        self.set_ready_hardware()
        self.bridge.face_brightness_callback(_UInt8(80))
        commands = self.capture_protocol_commands()

        self.assertTrue(self.bridge.send_next_face_command(100.0))
        self.assertEqual(commands, ["LED BRIGHTNESS 80"])
        self.bridge.handle_serial_line("OK LED BRIGHTNESS 80 EFFECTIVE=80")

        self.assertEqual(self.bridge.face_inflight_key, "")
        self.assertEqual(self.bridge.protocol.led_brightness, 80)

    def test_stale_led_ack_cannot_acknowledge_newer_inflight_value(self):
        self.set_ready_hardware()
        self.bridge.face_brightness_callback(_UInt8(90))
        commands = self.capture_protocol_commands()
        self.assertTrue(self.bridge.send_next_face_command(100.0))
        self.assertEqual(commands, ["LED BRIGHTNESS 90"])

        self.bridge.handle_serial_line("OK LED BRIGHTNESS 80 EFFECTIVE=80")
        self.assertEqual(self.bridge.face_inflight_key, "brightness")
        self.assertEqual(self.bridge.face_inflight_command, "LED BRIGHTNESS 90")

        self.bridge.handle_serial_line("OK LED BRIGHTNESS 90 EFFECTIVE=90")
        self.assertEqual(self.bridge.face_inflight_key, "")

    def test_ready_banner_on_open_port_replays_desired_face_after_nano_reset(self):
        self.set_ready_hardware()
        self.bridge.face_expression_callback(_String("thinking"))
        self.bridge.face_effect_callback(_String("loading"))
        self.bridge.face_dirty = []
        self.bridge.protocol.consume_response(
            "OK LED STATUS FACE_SUPPORTED=1 LED_ENABLED=1 "
            "LED_COLOR=150,40,255 LED_BRIGHTNESS=80 "
            "LED_EFFECT=loading LED_SPEED_MS=180 FACE=thinking"
        )

        self.bridge.handle_serial_line(
            "OK VOLT_PCA9685_READY FW=VOLT_PCA9685 PROTO=2 "
            "MAX_DPS=120.0 FACE_SUPPORTED=1 LED_COUNT=8 "
            "DISARMED OUTPUT_DISABLED"
        )

        self.assertTrue(self.bridge.connected)
        self.assertTrue(self.bridge.protocol.ready)
        self.assertFalse(self.bridge.protocol.face_status_seen)
        self.assertEqual(self.bridge.protocol.face_expression, "")
        self.assertEqual(self.bridge.face_dirty, ["expression", "effect"])
        self.assertTrue(self.bridge.face_status_pending)

    def test_rejected_face_setting_stays_unsynced_until_explicit_republish(self):
        self.set_ready_hardware()
        self.bridge.face_expression_callback(_String("happy"))
        commands = self.capture_protocol_commands()
        self.assertTrue(self.bridge.send_next_face_command(100.0))
        self.bridge.handle_serial_line("ERR FACE BAD_EXPRESSION")
        self.bridge.handle_serial_line(
            "OK LED STATUS FACE_SUPPORTED=1 LED_ENABLED=1 "
            "LED_COLOR=0,120,255 LED_BRIGHTNESS=80 "
            "LED_EFFECT=breathe LED_SPEED_MS=3000 FACE=idle"
        )

        self.assertIn("expression", self.bridge.face_failed_keys)
        self.assertFalse(self.bridge.face_is_synced())
        self.assertEqual(
            self.bridge.protocol.led_error,
            "ERR FACE BAD_EXPRESSION",
        )
        self.bridge.face_expression_callback(_String("happy"))
        self.assertNotIn("expression", self.bridge.face_failed_keys)
        self.assertEqual(self.bridge.protocol.led_error, "")
        self.assertIn("expression", self.bridge.face_dirty)

    def test_legacy_unknown_face_command_reports_unsupported_without_polling(self):
        self.set_ready_hardware()
        self.bridge.protocol.armed = True
        self.bridge.protocol.motion_inhibited = False
        self.bridge.face_expression_callback(_String("happy"))
        commands = self.capture_protocol_commands()
        self.assertTrue(self.bridge.send_next_face_command(100.0))
        self.bridge.handle_serial_line("ERR UNKNOWN_COMMAND")
        self.assertTrue(self.bridge.protocol.can_stream_frames)
        self.assertEqual(self.bridge.face_inflight_key, "")

        self.assertTrue(self.bridge.send_next_face_command(101.0))
        self.assertEqual(commands, ["FACE happy", "LED STATUS"])
        self.bridge.handle_serial_line("ERR UNKNOWN_COMMAND")
        self.assertFalse(self.bridge.protocol.face_supported)
        self.assertFalse(self.bridge.face_status_pending)
        self.assertEqual(self.bridge.protocol.led_error, "")
        self.assertFalse(self.bridge.send_next_face_command(102.0))
        self.assertEqual(commands, ["FACE happy", "LED STATUS"])

    def test_known_face_firmware_does_not_misattribute_corrupt_frame_error(self):
        self.set_ready_hardware()
        self.bridge.protocol.consume_response(
            "OK PONG FW=VOLT_PCA9685 PROTO=2 MAX_DPS=120.0 "
            "FACE_SUPPORTED=1 LED_COUNT=8"
        )
        self.bridge.face_expression_callback(_String("happy"))
        commands = self.capture_protocol_commands()
        self.assertTrue(self.bridge.send_next_face_command(100.0))

        self.bridge.handle_serial_line("ERR UNKNOWN_COMMAND")

        self.assertEqual(commands, ["FACE happy"])
        self.assertEqual(self.bridge.face_inflight_key, "expression")
        self.assertEqual(self.bridge.face_failed_keys, set())
        self.assertTrue(self.bridge.protocol.face_supported)

        self.bridge.face_inflight_key = ""
        self.bridge.face_status_request_time = 99.0
        self.bridge.handle_serial_line("ERR UNKNOWN_COMMAND")
        self.assertTrue(self.bridge.protocol.face_supported)
        self.assertEqual(self.bridge.face_status_request_time, 99.0)

    def test_recoverable_parse_error_does_not_amplify_serial_traffic(self):
        self.set_ready_hardware()
        self.bridge.protocol.armed = True
        self.bridge.protocol.motion_inhibited = False
        commands = self.capture_protocol_commands()

        self.bridge.handle_serial_line("ERR BAD_COUNT")

        self.assertEqual(commands, [])
        self.assertTrue(self.bridge.protocol.can_stream_frames)

    def test_long_status_response_defers_frames_until_complete(self):
        self.set_ready_hardware()
        self.bridge.require_motion_safe_to_arm = False
        self.bridge.protocol.armed = True
        self.bridge.protocol.motion_inhibited = False
        self.bridge.serial_query_inflight = "LED STATUS"
        self.bridge.serial_query_since = 100.0
        commands = self.capture_protocol_commands()

        self.set_time(2.0)
        self.bridge.command_callback(self.message([0.0] * 12))
        self.assertEqual(commands, [])

        self.bridge.handle_serial_line(
            "OK LED STATUS FACE_SUPPORTED=1 LED_ENABLED=1 "
            "LED_COLOR=0,120,255 LED_COLOR_B=0,120,255 "
            "LED_BRIGHTNESS=80 LED_EFFECTIVE_BRIGHTNESS=80 LED_LIMIT=160 "
            "LED_EFFECT=breathe LED_SPEED_MS=3000 FACE=idle"
        )
        self.set_time(2.1)
        self.bridge.command_callback(self.message([0.0] * 12))
        self.assertEqual(len(commands), 1)
        self.assertTrue(commands[0].startswith("FRAME "))

    def test_face_inflight_does_not_block_frames_and_safe_stop_blocks_face(self):
        self.set_ready_hardware()
        self.bridge.require_motion_safe_to_arm = False
        self.bridge.protocol.armed = True
        self.bridge.protocol.motion_inhibited = False
        commands = self.capture_protocol_commands()
        self.bridge.face_expression_callback(_String("playful"))
        self.assertTrue(self.bridge.send_next_face_command(100.0))

        self.set_time(2.0)
        self.bridge.command_callback(self.message([0.0] * 12))
        self.assertEqual(commands[0], "FACE playful")
        self.assertTrue(commands[1].startswith("FRAME "))

        self.bridge.handle_serial_line("OK FACE playful")
        self.bridge.protocol.note_command_sent("HOLD")
        self.bridge.face_brightness_callback(_UInt8(80))
        self.assertFalse(self.bridge.send_next_face_command(101.0))
        self.assertNotIn("LED BRIGHTNESS 80", commands)

    def test_face_state_is_resynchronized_after_reconnect(self):
        self.bridge.face_expression_callback(_String("thinking"))
        self.bridge.face_alternate_color_callback(
            _ColorRGBA(1.0, 0.0, 180.0 / 255.0, 1.0)
        )
        self.bridge.face_effect_callback(_String("loading"))
        self.bridge.mark_face_for_resync()
        self.assertEqual(
            self.bridge.face_dirty,
            ["expression", "alternate_color", "effect"],
        )

        self.bridge.protocol.face_expression = "idle"
        self.bridge.protocol.led_color_b = (0, 0, 0)
        self.bridge.protocol.led_effect = "breathe"
        self.bridge.disconnect("test disconnect")
        self.assertEqual(self.bridge.protocol.face_expression, "")
        self.assertEqual(self.bridge.protocol.led_color_b, ())
        self.assertEqual(
            self.bridge.face_dirty,
            ["expression", "alternate_color", "effect"],
        )

    def test_face_errors_do_not_interrupt_active_servo_stream(self):
        self.set_ready_hardware()
        self.bridge.protocol.armed = True
        self.bridge.protocol.motion_inhibited = False
        self.bridge.face_expression_callback(_String("happy"))
        self.bridge.face_inflight_key = "expression"
        self.bridge.face_inflight_value = "happy"
        self.bridge.face_inflight_command = "FACE happy"

        self.bridge.handle_serial_line("ERR FACE BAD_EXPRESSION")

        self.assertTrue(self.bridge.protocol.can_stream_frames)
        self.assertEqual(self.bridge.face_inflight_key, "")
        self.assertEqual(self.bridge.face_commands_rejected, 1)

    def test_led_status_is_exposed_in_bridge_status(self):
        self.set_ready_hardware()
        self.bridge.protocol.consume_response(
            "OK LED STATUS FACE_SUPPORTED=1 LED_ENABLED=1 "
            "LED_COLOR=0,120,255 LED_COLOR_B=255,0,180 "
            "LED_BRIGHTNESS=200 LED_EFFECTIVE_BRIGHTNESS=120 LED_LIMIT=120 "
            "LED_EFFECT=breathe LED_SPEED_MS=1200 FACE=idle"
        )
        self.bridge.face_status_pending = False
        self.bridge.publish_status()
        payload = self.bridge.status_publisher.messages[-1].data
        self.assertIn("face_connected=1", payload)
        self.assertIn("face_supported=1", payload)
        self.assertIn("face_enabled=1", payload)
        self.assertIn("face_expression=idle", payload)
        self.assertIn("face_color=0,120,255", payload)
        self.assertIn("face_color_b=255,0,180", payload)
        self.assertIn("face_brightness=200", payload)
        self.assertIn("face_effective_brightness=120", payload)
        self.assertIn("face_brightness_limit=120", payload)
        self.assertIn("face_effect=breathe", payload)
        self.assertIn("face_speed=1200", payload)
        self.assertEqual(payload.count("firmware_compatible="), 1)
        self.assertEqual(payload.count("capability_required="), 1)

    def test_shutdown_prioritizes_hold_then_starts_face_fade(self):
        serial_port = Mock()
        writes = []

        def write(payload):
            writes.append(payload.decode("ascii").strip())
            return len(payload)

        serial_port.write.side_effect = write
        self.bridge.serial_port = serial_port
        self.bridge.connected = True
        self.bridge.shutdown()

        self.assertEqual(writes, ["HOLD", "FACE shutdown"])
        serial_port.flush.assert_called_once_with()
        serial_port.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
