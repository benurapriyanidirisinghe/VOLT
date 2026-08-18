#!/usr/bin/env python3

"""Pure tests for motion-controller command and gait-switch safety."""

import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from volt_gait_controller import GAITS  # noqa: E402
from volt_kinematics import (  # noqa: E402
    JOINT_NAMES,
    LEG_ORDER,
    NOMINAL_FEET,
    SIT_POSE,
    WALK_POSE,
    feet_to_joint_positions_diagnostic,
)
from volt_motion_controller import (  # noqa: E402
    HARDWARE_JOINT_VELOCITY_LIMIT,
    SIMULATION_JOINT_VELOCITY_LIMIT,
    VELOCITY_GATE_AWAIT_MOTION,
    VELOCITY_GATE_AWAIT_NEUTRAL,
    VELOCITY_GATE_OPEN,
    VoltMotionController,
    initial_motion_state,
)
from volt_physical_tests import physical_test_request_json  # noqa: E402


class NullLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message, *_args, **_kwargs):
        self.infos.append(str(message))

    def warning(self, message, *_args, **_kwargs):
        self.warnings.append(str(message))


class RecordingPublisher:
    def __init__(self, subscription_count=1):
        self.subscription_count = subscription_count
        self.messages = []

    def get_subscription_count(self):
        return self.subscription_count

    def publish(self, message):
        self.messages.append(message)


class FakeGaitController:
    def __init__(self, active=False, swing_legs=None):
        self.active = bool(active)
        self.swing_legs = list(swing_legs or [])
        self.stop_requests = 0
        self.set_gait_calls = []
        self.hold_calls = []
        self.step_calls = 0
        self.reset_calls = []
        self.current_feet = None

    def request_stop(self):
        self.stop_requests += 1

    def set_gait(self, gait_name, now):
        self.set_gait_calls.append((gait_name, now))

    def hold_current_feet(self, feet, now=None):
        self.hold_calls.append((feet, now))
        self.active = False
        self.swing_legs = []

    def reset(self, now):
        self.reset_calls.append(now)
        self.active = False
        self.swing_legs = []

    def nominal_feet(self):
        return {leg: tuple(NOMINAL_FEET[leg]) for leg in LEG_ORDER}

    def set_current_feet(self, feet):
        self.current_feet = {
            leg: tuple(feet[leg]) for leg in LEG_ORDER
        }

    def debug_snapshot(self):
        stance_legs = [
            leg for leg in LEG_ORDER if leg not in self.swing_legs
        ]
        return {
            "phase": 0.25,
            "phase_index": 2,
            "phase_name": "swing_rear_right",
            "phase_progress": 0.50,
            "step_state": "LIFT_AND_SWING",
            "swing_legs": list(self.swing_legs),
            "stance_legs": stance_legs,
            "body_shift": {"x": -0.01, "y": 0.01},
            "body_shift_x": -0.01,
            "body_shift_y": 0.01,
            "body_shift_target_x": -0.01,
            "body_shift_target_y": 0.01,
            "support_target_x": -0.01,
            "support_target_y": 0.01,
            "support_margin": 0.012,
            "shift_completion": 1.0,
            "support_polygon_valid": True,
            "lift_allowed": True,
            "warning": "",
        }


class BodyOffsetGaitController(FakeGaitController):
    def __init__(self):
        super().__init__(active=True)
        self.body_offsets = []

    def step(
        self,
        _now,
        _dt,
        _velocity,
        _step_in_place=False,
        body_offset=(0.0, 0.0),
    ):
        self.body_offsets.append(tuple(body_offset))
        adjustment = {
            "x": 0.001,
            "y": -0.002,
            "body_x_override": 0.005,
            "body_y_override": -0.004,
            "height": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
        }
        return NOMINAL_FEET, adjustment, True


def make_twist(x=0.0, y=0.0, yaw=0.0):
    return SimpleNamespace(
        linear=SimpleNamespace(x=x, y=y, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=yaw),
    )


def make_controller(gait_controller=None):
    """Build the state used by callback/filter tests without constructing Node."""
    controller = VoltMotionController.__new__(VoltMotionController)
    controller.gait_configs = {
        name: dict(config)
        for name, config in GAITS.items()
    }
    controller.gait_name = "spot_walk"
    controller.requested_gait = "spot_walk"
    controller.pending_gait = None
    controller.gait_controller = gait_controller or FakeGaitController()
    controller.motion_active = False
    controller.step_in_place = False
    controller.last_step_keepalive_time = 10.0
    controller.step_keepalive_timeout = 1.0
    controller.command_owner = "MOTION"
    controller.velocity_command_sequence = 4
    controller.resume_after_velocity_sequence = -1
    controller.velocity_gate_state = VELOCITY_GATE_OPEN
    controller.requested_velocity = [0.0, 0.0, 0.0]
    controller.filtered_velocity = [0.0, 0.0, 0.0]
    controller.last_velocity_time = 10.0
    controller.command_timeout = 0.60
    controller.hardware_mode = False
    controller.open_loop_hardware = False
    controller.enable_physical_tests = False
    controller.physical_test_keepalive_timeout = 0.75
    controller.physical_test = None
    controller.open_loop_warning = ""
    controller.pending_pose_action = None
    controller.transition = None
    controller.auto_ready_pending = False
    controller.auto_ready_requested = False
    controller.auto_ready_pose = False
    controller.state = "standing"
    controller.measured_positions = None
    controller.commanded_positions = None
    controller.commanded_velocities = None
    controller.body_height = 0.2
    controller.neutral_body_height = 0.2
    controller.body_x = 0.0
    controller.body_y = 0.0
    controller.body_roll = 0.0
    controller.body_pitch = 0.0
    controller.body_yaw = 0.0
    controller.warning = ""
    controller._test_now = 10.0
    controller.now_seconds = lambda: controller._test_now
    controller.logger = NullLogger()
    controller.get_logger = lambda: controller.logger
    return controller


def enable_physical_test_controller(controller):
    controller.hardware_mode = True
    controller.open_loop_hardware = True
    controller.enable_physical_tests = True
    controller.state = "standing"
    controller.commanded_positions = list(WALK_POSE)
    controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
    controller.standing_feet = {
        leg: tuple(NOMINAL_FEET[leg]) for leg in LEG_ORDER
    }
    controller.projected_targets = []
    controller.clamped_joints = []
    controller.ik_diagnostics = {"projected_targets": [], "legs": {}}
    return controller


class GaitRequestTests(unittest.TestCase):
    def test_walk_alias_request_activates_canonical_video_walk(self):
        controller = make_controller()
        controller.gait_name = "legacy_walk"
        controller.requested_gait = "legacy_walk"
        controller.velocity_command_sequence = 7

        controller.gait_callback(SimpleNamespace(data="  WALK  "))

        self.assertEqual(
            controller.requested_gait,
            "spotmicro_video_walk",
        )
        self.assertEqual(controller.gait_name, "spotmicro_video_walk")
        self.assertIsNone(controller.pending_gait)
        self.assertEqual(
            controller.gait_controller.set_gait_calls,
            [("spotmicro_video_walk", 10.0)],
        )
        self.assertEqual(controller.resume_after_velocity_sequence, 7)
        self.assertEqual(
            controller.velocity_gate_state,
            VELOCITY_GATE_AWAIT_NEUTRAL,
        )

    def test_active_gait_defers_request_and_requests_safe_stop(self):
        gait = FakeGaitController(
            active=True,
            swing_legs=["rear_right"],
        )
        controller = make_controller(gait)
        controller.step_in_place = True

        controller.gait_callback(SimpleNamespace(data="slow_trot"))

        self.assertEqual(controller.requested_gait, "slow_trot")
        self.assertEqual(controller.gait_name, "spot_walk")
        self.assertEqual(controller.pending_gait, "slow_trot")
        self.assertEqual(gait.stop_requests, 1)
        self.assertEqual(gait.set_gait_calls, [])
        self.assertFalse(controller.step_in_place)

    def test_video_walk_request_is_deferred_while_trot_leg_is_airborne(self):
        gait = FakeGaitController(
            active=True,
            swing_legs=["front_left", "rear_right"],
        )
        controller = make_controller(gait)
        controller.gait_name = "normal_trot"
        controller.requested_gait = "normal_trot"

        controller.gait_callback(
            SimpleNamespace(data="spotmicro_video_walk")
        )

        self.assertEqual(
            controller.requested_gait,
            "spotmicro_video_walk",
        )
        self.assertEqual(controller.gait_name, "normal_trot")
        self.assertEqual(
            controller.pending_gait,
            "spotmicro_video_walk",
        )
        self.assertEqual(gait.stop_requests, 1)
        self.assertEqual(gait.set_gait_calls, [])

    def test_spot_walk_attitude_is_bounded_before_fresh_motion(self):
        controller = make_controller()
        controller.gait_name = "normal_trot"
        controller.requested_gait = "normal_trot"
        controller.body_roll = 0.15
        controller.body_pitch = -0.15

        controller.select_gait("spot_walk", 10.0)

        limit_roll = controller.gait_configs["spot_walk"][
            "maximum_body_roll"
        ]
        limit_pitch = controller.gait_configs["spot_walk"][
            "maximum_body_pitch"
        ]
        self.assertAlmostEqual(controller.body_roll, limit_roll)
        self.assertAlmostEqual(controller.body_pitch, -limit_pitch)
        self.assertEqual(
            controller.velocity_gate_state,
            VELOCITY_GATE_AWAIT_NEUTRAL,
        )

    def test_fast_trot_selection_resets_unvalidated_manual_body_pose(self):
        controller = make_controller()
        controller.gait_name = "normal_trot"
        controller.requested_gait = "normal_trot"
        controller.body_height = 0.220
        controller.body_x = -0.025
        controller.body_y = 0.020
        controller.body_roll = -0.060
        controller.body_pitch = -0.080
        controller.body_yaw = 0.180

        controller.select_gait("fast_trot", 10.0)

        self.assertEqual(controller.body_height, controller.neutral_body_height)
        self.assertEqual(
            (
                controller.body_x,
                controller.body_y,
                controller.body_roll,
                controller.body_pitch,
                controller.body_yaw,
            ),
            (0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def test_held_nonzero_traffic_cannot_resume_after_immediate_switch(self):
        controller = make_controller()

        controller.gait_callback(SimpleNamespace(data="slow_trot"))
        for _ in range(5):
            controller.velocity_callback(make_twist(x=0.02))

        self.assertEqual(
            controller.velocity_gate_state,
            VELOCITY_GATE_AWAIT_NEUTRAL,
        )
        self.assertEqual(controller.requested_velocity, [0.0, 0.0, 0.0])

        controller.velocity_callback(make_twist())
        self.assertEqual(
            controller.velocity_gate_state,
            VELOCITY_GATE_AWAIT_MOTION,
        )
        controller.velocity_callback(make_twist(x=0.02))
        self.assertEqual(controller.velocity_gate_state, VELOCITY_GATE_OPEN)
        self.assertEqual(controller.requested_velocity[0], 0.02)

    def test_cancelling_queued_switch_discards_stale_velocity(self):
        gait = FakeGaitController(
            active=True,
            swing_legs=["rear_right"],
        )
        controller = make_controller(gait)
        controller.requested_velocity = [0.02, -0.01, 0.10]
        controller.filtered_velocity = [0.015, -0.005, 0.08]
        controller.velocity_command_sequence = 9

        controller.gait_callback(SimpleNamespace(data="slow_trot"))
        self.assertEqual(controller.pending_gait, "slow_trot")

        controller.gait_callback(SimpleNamespace(data="spot_walk"))

        self.assertIsNone(controller.pending_gait)
        self.assertEqual(controller.requested_gait, "spot_walk")
        self.assertEqual(controller.requested_velocity, [0.0, 0.0, 0.0])
        self.assertEqual(controller.filtered_velocity, [0.0, 0.0, 0.0])
        self.assertEqual(
            controller.velocity_gate_state,
            VELOCITY_GATE_AWAIT_NEUTRAL,
        )
        self.assertEqual(controller.resume_after_velocity_sequence, 9)


class VelocitySafetyTests(unittest.TestCase):
    def assert_ramped_toward_zero(self, before, after):
        for previous, current in zip(before, after):
            self.assertGreaterEqual(current, 0.0)
            self.assertLess(current, previous)

    def test_pending_gait_ramps_filtered_velocity_toward_zero(self):
        controller = make_controller()
        controller.pending_gait = "slow_trot"
        controller.requested_velocity = [0.004, 0.001, 0.05]
        controller.filtered_velocity = [0.004, 0.001, 0.05]
        controller.velocity_command_sequence = 5
        before = list(controller.filtered_velocity)

        controller.update_filtered_velocity(now=10.1, dt=0.1)

        self.assert_ramped_toward_zero(
            before,
            controller.filtered_velocity,
        )

    def test_hardware_scaled_low_spot_velocity_is_classified_as_motion(self):
        controller = make_controller()
        controller.hardware_mode = True
        config = controller.gait_configs["spot_walk"]
        controller.filtered_velocity = [
            config["max_x"]
            * config["hardware_speed_scale"]
            * 0.10,
            0.0,
            0.0,
        ]

        self.assertTrue(controller.filtered_motion_requested())

    def test_timed_out_command_ramps_filtered_velocity_toward_zero(self):
        gait = FakeGaitController(active=True)
        controller = make_controller(gait)
        controller.requested_velocity = [0.004, 0.001, 0.05]
        controller.filtered_velocity = [0.004, 0.001, 0.05]
        controller.velocity_command_sequence = 5
        controller.last_velocity_time = 1.0
        before = list(controller.filtered_velocity)

        controller.update_filtered_velocity(now=10.0, dt=0.1)

        self.assert_ramped_toward_zero(
            before,
            controller.filtered_velocity,
        )
        self.assertEqual(gait.stop_requests, 1)

    def test_nonfinite_twist_is_rejected_to_zero_without_new_sequence(self):
        invalid_commands = (
            make_twist(x=float("nan")),
            make_twist(y=float("inf")),
            make_twist(yaw=float("-inf")),
        )
        for message in invalid_commands:
            with self.subTest(message=message):
                controller = make_controller()
                controller.requested_velocity = [0.1, -0.1, 0.2]
                original_sequence = controller.velocity_command_sequence
                controller._test_now = 12.5

                controller.velocity_callback(message)

                self.assertEqual(
                    controller.requested_velocity,
                    [0.0, 0.0, 0.0],
                )
                self.assertEqual(
                    controller.velocity_command_sequence,
                    original_sequence,
                )
                self.assertEqual(controller.last_velocity_time, 12.5)
                self.assertIn("non-finite", controller.warning)
                self.assertTrue(controller.logger.warnings)


class SafeGaitSwitchTests(unittest.TestCase):
    def prepare_control_callback(self, controller):
        controller.last_update_time = 9.9
        controller.commanded_positions = [0.0] * len(JOINT_NAMES)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        controller.command_publisher = RecordingPublisher()
        controller.auto_ready_pending = False
        controller.state = "standing"
        controller.transition = None
        controller.step_in_place = False
        controller.update_filtered_velocity = lambda _now, _dt: None
        controller.gait_target = lambda _now, _dt: list(
            controller.commanded_positions
        )
        controller.smooth_joint_target = (
            lambda target, _dt: list(target)
        )
        controller.update_feedback_warning = lambda: None
        controller.publish_status = lambda _now: None

    def test_switch_waits_for_inactive_grounded_gait_and_requires_fresh_velocity(self):
        gait = FakeGaitController(
            active=True,
            swing_legs=["rear_right"],
        )
        controller = make_controller(gait)
        controller.pending_gait = "slow_trot"
        controller.requested_gait = "slow_trot"
        controller.velocity_command_sequence = 11
        controller.filtered_velocity = [0.003, 0.0, 0.0]
        self.prepare_control_callback(controller)

        controller.control_callback()

        self.assertEqual(controller.gait_name, "spot_walk")
        self.assertEqual(controller.pending_gait, "slow_trot")
        self.assertEqual(gait.set_gait_calls, [])

        # The gait controller's safety contract keeps active true until the
        # airborne foot has touched down and all four feet are in stance.
        gait.active = False
        gait.swing_legs = []
        controller.control_callback()

        self.assertEqual(controller.gait_name, "slow_trot")
        self.assertIsNone(controller.pending_gait)
        self.assertEqual(gait.set_gait_calls, [("slow_trot", 10.0)])
        self.assertEqual(controller.filtered_velocity, [0.0, 0.0, 0.0])
        self.assertEqual(controller.resume_after_velocity_sequence, 11)
        self.assertEqual(
            controller.velocity_gate_state,
            VELOCITY_GATE_AWAIT_NEUTRAL,
        )

        # Remove the control-callback stub and exercise the two-stage freshness
        # gate: old/nonzero traffic is ignored until neutral is observed.
        del controller.update_filtered_velocity
        controller.velocity_callback(make_twist(x=0.02))
        VoltMotionController.update_filtered_velocity(
            controller,
            now=10.1,
            dt=0.1,
        )
        self.assertEqual(controller.filtered_velocity, [0.0, 0.0, 0.0])

        controller.velocity_callback(make_twist())
        controller.velocity_callback(make_twist(x=0.02))
        VoltMotionController.update_filtered_velocity(
            controller,
            now=10.2,
            dt=0.1,
        )
        self.assertGreater(controller.filtered_velocity[0], 0.0)


class CommandOwnershipTests(unittest.TestCase):
    def prepare_control_callback(self, controller):
        controller.last_update_time = 9.9
        controller.last_status_time = 0.0
        controller.command_publisher = RecordingPublisher()
        controller.status_publisher = RecordingPublisher()
        controller.update_feedback_warning = lambda: None
        controller.publish_status = lambda _now: None

    def test_router_status_parser_accepts_router_format_and_json(self):
        self.assertEqual(
            VoltMotionController.parse_command_owner(
                "owner=MOTION controller_connected=1 pose_valid=1"
            ),
            "MOTION",
        )
        self.assertEqual(
            VoltMotionController.parse_command_owner(
                '{"owner": "calibration", "pose_valid": true}'
            ),
            "CALIBRATION",
        )
        self.assertIsNone(
            VoltMotionController.parse_command_owner("owner=INVALID")
        )

    def test_owner_loss_cancels_and_resyncs_every_latent_motion(self):
        gait = FakeGaitController(
            active=True,
            swing_legs=["rear_right"],
        )
        controller = make_controller(gait)
        measured = [0.01 * index for index in range(len(JOINT_NAMES))]
        controller.measured_positions = list(measured)
        controller.commanded_positions = [0.5] * len(JOINT_NAMES)
        controller.commanded_velocities = [0.2] * len(JOINT_NAMES)
        controller.requested_velocity = [0.02, 0.01, 0.1]
        controller.filtered_velocity = [0.01, 0.005, 0.05]
        controller.step_in_place = True
        controller.motion_active = True
        controller.pending_gait = "slow_trot"
        controller.requested_gait = "slow_trot"
        controller.pending_pose_action = "sit"
        controller.transition = {"start_time": 9.0}
        controller.state = "standing_up"

        controller.command_router_status_callback(
            SimpleNamespace(
                data="owner=HOLD controller_connected=1 pose_valid=1"
            )
        )

        self.assertEqual(controller.command_owner, "HOLD")
        self.assertEqual(controller.commanded_positions, measured)
        self.assertEqual(
            controller.commanded_velocities,
            [0.0] * len(JOINT_NAMES),
        )
        self.assertEqual(controller.requested_velocity, [0.0, 0.0, 0.0])
        self.assertEqual(controller.filtered_velocity, [0.0, 0.0, 0.0])
        self.assertFalse(controller.step_in_place)
        self.assertFalse(controller.motion_active)
        self.assertIsNone(controller.pending_gait)
        self.assertIsNone(controller.pending_pose_action)
        self.assertIsNone(controller.transition)
        self.assertEqual(controller.requested_gait, "spot_walk")
        self.assertEqual(controller.state, "hold")
        self.assertFalse(gait.active)
        self.assertEqual(len(gait.hold_calls), 1)

    def test_unowned_control_tick_only_republishes_measured_hold(self):
        gait = FakeGaitController(active=True, swing_legs=["front_left"])
        controller = make_controller(gait)
        controller.command_owner = "MANUAL"
        measured = [0.02] * len(JOINT_NAMES)
        controller.measured_positions = measured
        controller.commanded_positions = [0.6] * len(JOINT_NAMES)
        controller.commanded_velocities = [0.4] * len(JOINT_NAMES)
        controller.transition = {
            "start_time": 9.0,
            "start": [0.0] * len(JOINT_NAMES),
            "waypoints": [(4.0, [1.0] * len(JOINT_NAMES))],
            "final_state": "standing",
        }
        self.prepare_control_callback(controller)

        controller.control_callback()

        self.assertEqual(controller.commanded_positions, measured)
        self.assertIsNone(controller.transition)
        self.assertFalse(gait.active)
        self.assertEqual(len(controller.command_publisher.messages), 1)
        self.assertEqual(
            list(controller.command_publisher.messages[0].data),
            measured,
        )

    def test_reenable_requires_neutral_and_fresh_command_from_hold(self):
        controller = make_controller()
        controller.command_owner = "HOLD"
        controller.measured_positions = [0.0] * len(JOINT_NAMES)
        controller.commanded_positions = [0.4] * len(JOINT_NAMES)

        controller.command_router_status_callback(
            SimpleNamespace(data="owner=MOTION controller_connected=1")
        )
        self.assertEqual(controller.state, "hold")
        self.assertEqual(
            controller.velocity_gate_state,
            VELOCITY_GATE_AWAIT_NEUTRAL,
        )

        for _ in range(3):
            controller.velocity_callback(make_twist(x=0.02))
        self.assertEqual(controller.requested_velocity, [0.0, 0.0, 0.0])
        self.assertEqual(
            controller.velocity_gate_state,
            VELOCITY_GATE_AWAIT_NEUTRAL,
        )

        controller.velocity_callback(make_twist())
        controller.velocity_callback(make_twist(x=0.02))
        self.assertEqual(
            controller.velocity_gate_state,
            VELOCITY_GATE_AWAIT_NEUTRAL,
        )
        self.assertEqual(controller.requested_velocity, [0.0, 0.0, 0.0])
        self.assertIn("Stand Up", controller.warning)
        # Ownership alone never resumes a prior standing/gait state.
        self.assertEqual(controller.state, "hold")

    def test_body_pose_commands_are_not_cached_by_another_owner(self):
        controller = make_controller()
        controller.command_owner = "MANUAL"
        controller.body_x = 0.01
        controller.body_height = 0.2

        controller.body_pose_callback(
            SimpleNamespace(
                linear=SimpleNamespace(x=-0.02, y=0.01, z=0.18),
                angular=SimpleNamespace(x=0.1, y=-0.1, z=0.1),
            )
        )

        self.assertEqual(controller.body_x, 0.01)
        self.assertEqual(controller.body_height, 0.2)

    def test_spot_walk_body_pose_uses_shared_configured_attitude_limits(self):
        controller = make_controller()

        controller.body_pose_callback(
            SimpleNamespace(
                linear=SimpleNamespace(x=0.0, y=0.0, z=0.2),
                angular=SimpleNamespace(x=0.15, y=-0.15, z=0.0),
            )
        )

        config = controller.gait_configs["spot_walk"]
        self.assertAlmostEqual(
            controller.body_roll,
            config["maximum_body_roll"],
        )
        self.assertAlmostEqual(
            controller.body_pitch,
            -config["maximum_body_pitch"],
        )

    def test_fast_trot_rejects_external_body_pose_commands(self):
        controller = make_controller()
        controller.gait_name = "fast_trot"

        controller.body_pose_callback(
            SimpleNamespace(
                linear=SimpleNamespace(x=-0.025, y=0.020, z=0.220),
                angular=SimpleNamespace(x=-0.060, y=-0.080, z=0.180),
            )
        )

        self.assertEqual(controller.body_height, controller.neutral_body_height)
        self.assertEqual(
            (
                controller.body_x,
                controller.body_y,
                controller.body_roll,
                controller.body_pitch,
                controller.body_yaw,
            ),
            (0.0, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertIn("owns body posture", controller.warning)

        controller.body_pose_callback(
            SimpleNamespace(
                linear=SimpleNamespace(x=0.0, y=0.0, z=0.2),
                angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            )
        )
        self.assertEqual(controller.warning, "")


class PoseTransitionSafetyTests(unittest.TestCase):
    def install_parameters(self, controller):
        values = {
            "stand_duration": 4.0,
            "sit_duration": 4.7,
        }
        controller.get_parameter = lambda name: SimpleNamespace(
            value=values[name]
        )

    def test_sit_waits_for_gait_grounding_and_cancels_pending_switch(self):
        gait = FakeGaitController(
            active=True,
            swing_legs=["rear_left"],
        )
        controller = make_controller(gait)
        controller.commanded_positions = list(WALK_POSE)
        controller.measured_positions = list(WALK_POSE)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        controller.state = "standing"
        controller.step_in_place = True
        controller.pending_gait = "fast_trot"
        controller.requested_gait = "fast_trot"
        self.install_parameters(controller)

        controller.start_sit_transition()

        self.assertEqual(controller.pending_pose_action, "sit")
        self.assertIsNone(controller.pending_gait)
        self.assertEqual(controller.requested_gait, "spot_walk")
        self.assertIsNone(controller.transition)
        self.assertFalse(controller.step_in_place)
        self.assertEqual(gait.stop_requests, 1)

        gait.active = False
        gait.swing_legs = []
        controller.start_pending_pose_transition()
        self.assertIsNone(controller.pending_pose_action)
        self.assertIsNotNone(controller.transition)
        self.assertEqual(controller.state, "sitting_down")

    def test_natural_sit_is_planted_asymmetric_cartesian_ik_and_reversible(self):
        controller = make_controller()
        controller.commanded_positions = list(WALK_POSE)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        controller.state = "standing"
        controller.standing_feet = {
            leg: tuple(NOMINAL_FEET[leg]) for leg in LEG_ORDER
        }
        controller.natural_sit_height = 0.145
        controller.natural_sit_rearward_shift = 0.020
        controller.natural_sit_pitch = math.radians(-10.0)
        self.install_parameters(controller)

        controller.start_sit_transition()

        self.assertEqual(controller.state, "sitting_down")
        self.assertEqual(controller.transition["kind"], "cartesian")
        self.assertEqual(controller.transition["label"], "natural_sit")
        plan = controller.natural_sit_plan
        self.assertEqual(plan["sitting"]["feet"], plan["standing"]["feet"])
        self.assertAlmostEqual(plan["sitting"]["body"]["height"], 0.145)
        self.assertAlmostEqual(plan["sitting"]["body"]["body_x"], -0.020)
        self.assertAlmostEqual(
            plan["sitting"]["body"]["pitch"],
            math.radians(-10.0),
        )
        sitting, diagnostics = controller.solve_cartesian_pose(plan["sitting"])
        self.assertEqual(diagnostics["projected_targets"], [])
        self.assertTrue(all(
            not diagnostics["legs"][leg]["clamped_joints"]
            for leg in LEG_ORDER
        ))
        self.assertGreater(
            sitting[7] - sitting[1],
            math.radians(20.0),
        )

        for sample in range(95):
            target = controller.transition_target(10.0 + sample * 0.05)
            self.assertEqual(len(target), len(JOINT_NAMES))
            self.assertTrue(all(math.isfinite(value) for value in target))
            self.assertEqual(controller.projected_targets, [])
            self.assertEqual(controller.clamped_joints, [])

        final_target = controller.transition_target(14.71)
        self.assertTrue(controller.transition["filter_settling"])
        self.assertEqual(controller.state, "sitting_down")
        controller.commanded_positions = list(final_target)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        self.assertTrue(
            controller.complete_pose_transition_after_filter(final_target)
        )
        self.assertEqual(controller.state, "sitting")
        self.assertIsNone(controller.transition)

        controller.start_stand_transition()
        self.assertEqual(controller.state, "standing_up")
        self.assertEqual(controller.transition["label"], "natural_stand")
        stand_target = controller.transition_target(18.72)
        self.assertTrue(controller.transition["filter_settling"])
        controller.commanded_positions = list(stand_target)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        self.assertTrue(
            controller.complete_pose_transition_after_filter(stand_target)
        )
        self.assertEqual(controller.state, "standing")
        self.assertIsNone(controller.transition)
        self.assertIsNone(controller.natural_sit_plan)
        for expected, actual in zip(WALK_POSE, stand_target):
            self.assertAlmostEqual(expected, actual, places=12)

    def test_pose_transition_stays_active_until_filter_settles_and_stop_cancels(self):
        controller = make_controller()
        controller.commanded_positions = list(WALK_POSE)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        controller.state = "standing"
        controller.standing_feet = {
            leg: tuple(NOMINAL_FEET[leg]) for leg in LEG_ORDER
        }
        self.install_parameters(controller)
        controller.start_sit_transition()

        final_target = controller.transition_target(14.71)
        controller.commanded_positions = list(final_target)
        controller.commanded_positions[0] += math.radians(2.0)
        controller.commanded_velocities = [math.radians(2.0)] * len(JOINT_NAMES)
        self.assertFalse(
            controller.complete_pose_transition_after_filter(final_target)
        )
        self.assertEqual(controller.state, "sitting_down")
        self.assertIsNotNone(controller.transition)

        controller.stop_motion()
        self.assertIsNone(controller.transition)
        self.assertIsNone(controller.natural_sit_plan)
        self.assertEqual(controller.state, "hold")
        self.assertIn("STOP cancelled", controller.warning)

    def test_sit_transition_rejects_competing_gait_body_and_velocity(self):
        controller = make_controller()
        controller.commanded_positions = list(WALK_POSE)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        controller.state = "standing"
        controller.standing_feet = {
            leg: tuple(NOMINAL_FEET[leg]) for leg in LEG_ORDER
        }
        self.install_parameters(controller)
        controller.start_sit_transition()

        controller.gait_callback(SimpleNamespace(data="slow_trot"))
        self.assertEqual(controller.requested_gait, "spot_walk")
        controller.body_pose_callback(
            SimpleNamespace(
                linear=SimpleNamespace(x=0.01, y=0.0, z=0.2),
                angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            )
        )
        self.assertEqual(controller.body_x, 0.0)

        controller.velocity_callback(make_twist(x=0.02))
        self.assertIsNone(controller.transition)
        self.assertEqual(controller.state, "hold")
        self.assertEqual(controller.requested_velocity, [0.0, 0.0, 0.0])
        self.assertIn("cancelled the pose transition", controller.warning)

    def test_natural_stand_restores_captured_non_nominal_footprint(self):
        controller = make_controller()
        captured_feet = {
            leg: (
                NOMINAL_FEET[leg][0] + (0.003 if leg.startswith("front") else -0.002),
                NOMINAL_FEET[leg][1],
                NOMINAL_FEET[leg][2],
            )
            for leg in LEG_ORDER
        }
        controller.standing_feet = captured_feet
        start_pose = controller.current_cartesian_pose()
        start_target, _diagnostics = controller.solve_cartesian_pose(start_pose)
        controller.commanded_positions = list(start_target)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        controller.state = "standing"
        self.install_parameters(controller)

        controller.start_sit_transition()
        sit_target = controller.transition_target(14.71)
        controller.commanded_positions = list(sit_target)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        controller.complete_pose_transition_after_filter(sit_target)
        controller.start_stand_transition()
        stand_target = controller.transition_target(18.72)
        controller.commanded_positions = list(stand_target)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        controller.complete_pose_transition_after_filter(stand_target)

        self.assertEqual(controller.standing_feet, captured_feet)
        self.assertEqual(controller.gait_controller.current_feet, captured_feet)
        for expected, actual in zip(start_target, stand_target):
            self.assertAlmostEqual(expected, actual, places=12)

    def test_stand_is_rejected_without_motion_ownership(self):
        controller = make_controller()
        controller.command_owner = "CALIBRATION"
        controller.commanded_positions = list(SIT_POSE)
        self.install_parameters(controller)

        controller.start_stand_transition()

        self.assertIsNone(controller.transition)
        self.assertIn("MOTION", controller.warning)

    def test_step_cannot_latch_during_sitting_or_pose_transition(self):
        controller = make_controller()
        controller.state = "sitting"

        controller.action_callback(SimpleNamespace(data="step"))
        self.assertFalse(controller.step_in_place)
        self.assertIn("standing", controller.warning)

        controller.state = "standing_up"
        controller.transition = {"start_time": 10.0}
        controller.step_in_place = True
        controller.action_callback(SimpleNamespace(data="step"))
        self.assertFalse(controller.step_in_place)

        controller.state = "standing"
        controller.transition = None
        controller.pending_pose_action = None
        controller.velocity_gate_state = VELOCITY_GATE_AWAIT_NEUTRAL
        controller.action_callback(SimpleNamespace(data="step"))
        self.assertFalse(controller.step_in_place)

        controller.velocity_gate_state = VELOCITY_GATE_AWAIT_MOTION
        controller.action_callback(SimpleNamespace(data="step"))
        self.assertTrue(controller.step_in_place)
        self.assertEqual(controller.velocity_gate_state, VELOCITY_GATE_OPEN)

    def test_step_requires_keepalive_and_stops_safely_when_it_expires(self):
        gait = FakeGaitController(active=True, swing_legs=["rear_right"])
        controller = make_controller(gait)
        controller.state = "standing"
        controller.step_in_place = True
        controller.last_step_keepalive_time = 10.0

        self.assertFalse(controller.expire_step_if_stale(10.99))
        controller._test_now = 10.75
        controller.action_callback(SimpleNamespace(data="step_keepalive"))
        self.assertEqual(controller.last_step_keepalive_time, 10.75)
        self.assertFalse(controller.expire_step_if_stale(11.74))

        self.assertTrue(controller.expire_step_if_stale(11.76))
        self.assertFalse(controller.step_in_place)
        self.assertEqual(gait.stop_requests, 1)
        self.assertIn("keepalive expired", controller.warning)


class OpenLoopHardwareTests(unittest.TestCase):
    def test_fast_trot_jump_warning_requires_consecutive_active_gait_samples(self):
        controller = make_controller(FakeGaitController(active=False))
        controller.gait_name = "fast_trot"
        controller.commanded_positions = [0.0] * len(JOINT_NAMES)
        controller.sudden_joint_jump_deg = 10.0
        controller.last_fast_trot_raw_joint_target = [
            0.0 for _ in JOINT_NAMES
        ]
        emote_like_target = [0.0] * len(JOINT_NAMES)
        emote_like_target[0] = math.radians(20.0)

        controller.record_joint_command_deltas(
            emote_like_target,
            emote_like_target,
        )

        self.assertEqual(controller.maximum_raw_joint_jump_deg, 0.0)
        self.assertEqual(controller.last_fast_trot_raw_joint_target, [])
        self.assertEqual(controller.logger.warnings, [])

        # The first genuinely active sample establishes a source-local
        # baseline; only a later active fast-trot sample may raise the warning.
        controller.gait_controller.active = True
        controller.record_joint_command_deltas(
            emote_like_target,
            emote_like_target,
        )
        self.assertEqual(controller.maximum_raw_joint_jump_deg, 0.0)
        self.assertEqual(controller.logger.warnings, [])

        next_gait_target = list(emote_like_target)
        next_gait_target[0] += math.radians(20.0)
        controller.record_joint_command_deltas(
            next_gait_target,
            next_gait_target,
        )
        self.assertAlmostEqual(controller.maximum_raw_joint_jump_deg, 20.0)
        self.assertEqual(len(controller.logger.warnings), 1)
        self.assertIn("FAST TROT raw joint target", controller.logger.warnings[0])

    def test_fast_trot_restart_discards_partial_previous_cycle_metrics(self):
        gait = FakeGaitController(active=False)
        controller = make_controller(gait)
        controller.gait_name = "fast_trot"
        controller.reset_fast_trot_cycle_diagnostics()
        controller.fast_trot_diagnostic_active = True
        controller.fast_trot_completed_cycles = 3
        controller.fast_trot_stance_completion_pending = {"front_left"}
        controller.last_gait_body_transform = {
            "height": 0.2,
            "body_x": 0.0,
            "body_y": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }

        controller.update_fast_trot_cycle_diagnostics(
            WALK_POSE,
            WALK_POSE,
        )
        self.assertFalse(controller.fast_trot_diagnostic_active)

        gait.active = True
        controller.update_fast_trot_cycle_diagnostics(
            WALK_POSE,
            WALK_POSE,
        )
        self.assertTrue(controller.fast_trot_diagnostic_active)
        self.assertEqual(controller.fast_trot_completed_cycles, 0)
        self.assertEqual(
            controller.fast_trot_stance_completion_pending,
            set(),
        )

    def test_hardware_profile_never_exceeds_first_test_joint_rate(self):
        controller = make_controller()
        controller.max_joint_velocity = 4.0
        controller.hardware_mode = True
        for index in range(len(JOINT_NAMES)):
            self.assertLessEqual(
                controller.joint_velocity_limit(index),
                HARDWARE_JOINT_VELOCITY_LIMIT,
            )

        controller.hardware_mode = False
        self.assertEqual(
            controller.joint_velocity_limit(2),
            SIMULATION_JOINT_VELOCITY_LIMIT,
        )

    def test_fast_trot_uses_fixed_validated_acceleration_for_every_preset(self):
        controller = make_controller()
        controller.gait_name = "fast_trot"
        controller.gait_configs = {
            name: dict(config)
            for name, config in GAITS.items()
        }
        controller.hardware_mode = True
        controller.max_joint_acceleration = 60.0
        controller.gait_controller = SimpleNamespace(
            fast_trot_tuning=dict(
                GAITS["fast_trot"]["presets"]["bench"]
            )
        )

        expected = {
            name: GAITS["fast_trot"]["joint_acceleration_limit"]
            for name in ("bench", "floor_test", "wide")
        }
        observed = []
        for name, acceleration in expected.items():
            controller.gait_controller.fast_trot_tuning = dict(
                GAITS["fast_trot"]["presets"][name]
            )
            actual = controller.joint_acceleration_limit(0)
            observed.append(actual)
            self.assertAlmostEqual(actual, acceleration, places=12)
            self.assertLessEqual(actual, controller.max_joint_acceleration)
        self.assertEqual(len(set(observed)), 1)

    def test_open_loop_startup_state_is_firmware_safe_walk_pose(self):
        state, positions, velocities, warning = initial_motion_state(True)
        self.assertEqual(state, "standing")
        self.assertEqual(positions, list(WALK_POSE))
        self.assertEqual(velocities, [0.0] * len(JOINT_NAMES))
        self.assertIn("firmware-safe WALK_POSE", warning)

        state, positions, velocities, warning = initial_motion_state(False)
        self.assertEqual(state, "waiting")
        self.assertIsNone(positions)
        self.assertIsNone(velocities)
        self.assertEqual(warning, "")

    def test_open_loop_seed_remains_firmware_safe_without_feedback(self):
        gait = FakeGaitController(active=True, swing_legs=["rear_right"])
        controller = make_controller(gait)
        controller.open_loop_hardware = True
        controller.open_loop_warning = (
            "OPEN-LOOP HARDWARE: assuming firmware-safe WALK_POSE without "
            "measured joint feedback."
        )
        controller.state = "standing"
        controller.commanded_positions = list(WALK_POSE)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        controller.measured_positions = None

        controller.resync_motion_hold(10.0)

        self.assertEqual(controller.state, "standing")
        self.assertEqual(controller.commanded_positions, list(WALK_POSE))
        self.assertEqual(
            controller.commanded_velocities,
            [0.0] * len(JOINT_NAMES),
        )
        self.assertIn("OPEN-LOOP", controller.open_loop_warning)
        self.assertFalse(gait.active)
        self.assertEqual(len(gait.hold_calls), 1)

    def test_open_loop_ignores_simulator_joint_state_feedback(self):
        controller = make_controller()
        controller.open_loop_hardware = True
        controller.state = "standing"
        controller.commanded_positions = list(WALK_POSE)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        message = SimpleNamespace(
            name=list(JOINT_NAMES),
            position=[0.0] * len(JOINT_NAMES),
        )

        controller.joint_state_callback(message)

        self.assertIsNone(controller.measured_positions)
        self.assertEqual(controller.state, "standing")
        self.assertEqual(controller.commanded_positions, list(WALK_POSE))

    def test_only_exact_stopped_open_loop_walk_pose_certifies_neutral_arm(self):
        controller = make_controller()
        controller.open_loop_hardware = True
        controller.state = "hold"
        controller.commanded_positions = list(WALK_POSE)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)

        self.assertTrue(controller.arm_neutral_ready())

        controller.commanded_positions[0] += math.radians(1.0)
        self.assertFalse(controller.arm_neutral_ready())
        controller.commanded_positions = list(WALK_POSE)
        controller.motion_active = True
        self.assertFalse(controller.arm_neutral_ready())
        controller.motion_active = False
        controller.body_x = 0.001
        self.assertFalse(controller.arm_neutral_ready())

    def test_open_loop_owner_loss_holds_last_command_and_clears_gait(self):
        gait = FakeGaitController(active=True, swing_legs=["front_right"])
        controller = make_controller(gait)
        controller.open_loop_hardware = True
        controller.command_owner = "MOTION"
        controller.state = "standing"
        controller.measured_positions = None
        last_command = [0.03 * index for index in range(len(JOINT_NAMES))]
        controller.commanded_positions = list(last_command)
        controller.commanded_velocities = [0.1] * len(JOINT_NAMES)
        controller.motion_active = True

        controller.command_router_status_callback(
            SimpleNamespace(data="owner=HOLD controller_connected=1")
        )

        self.assertEqual(controller.state, "hold")
        self.assertEqual(controller.commanded_positions, last_command)
        self.assertFalse(gait.active)
        self.assertEqual(len(gait.hold_calls), 1)

    def test_hardware_owner_loss_restores_profile_com_offsets(self):
        controller = make_controller()
        controller.hardware_mode = True
        controller.open_loop_hardware = True
        controller.state = "standing"
        controller.commanded_positions = list(WALK_POSE)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        controller.applied_real_tuning = {
            "body_height": 0.195,
            "body_x": -0.004,
            "body_y": 0.002,
            "body_roll_deg": -0.5,
            "body_pitch_deg": 1.0,
            "body_yaw_deg": 0.25,
        }
        controller.neutral_body_height = 0.195
        controller.body_x = 0.02
        controller.body_roll = 0.1

        controller.cancel_motion_for_owner_loss(10.0, force_hold=True)

        self.assertAlmostEqual(controller.body_height, 0.195)
        self.assertAlmostEqual(controller.body_x, -0.004)
        self.assertAlmostEqual(controller.body_y, 0.002)
        self.assertAlmostEqual(controller.body_roll, math.radians(-0.5))
        self.assertAlmostEqual(controller.body_pitch, math.radians(1.0))
        self.assertAlmostEqual(controller.body_yaw, math.radians(0.25))
        self.assertFalse(controller.arm_neutral_ready())


class GaitBodyOffsetIntegrationTests(unittest.TestCase):
    def test_gait_target_passes_operator_offset_and_uses_absolute_override(self):
        gait = BodyOffsetGaitController()
        controller = make_controller(gait)
        controller.body_x = 0.020
        controller.body_y = 0.010
        controller.projected_targets = []
        controller.ik_diagnostics = {"projected_targets": [], "legs": {}}
        controller.debug_gait = False
        controller.last_debug_time = 0.0

        actual = controller.gait_target(now=10.0, dt=0.01)
        expected, _diagnostics = feet_to_joint_positions_diagnostic(
            NOMINAL_FEET,
            height=controller.body_height,
            # Override values are absolute operator offsets.  Support-shift
            # x/y are then added exactly once.
            body_x=0.006,
            body_y=-0.006,
            roll=controller.body_roll,
            pitch=controller.body_pitch,
            yaw=controller.body_yaw,
        )
        double_counted, _diagnostics = feet_to_joint_positions_diagnostic(
            NOMINAL_FEET,
            height=controller.body_height,
            body_x=0.021,
            body_y=0.008,
            roll=controller.body_roll,
            pitch=controller.body_pitch,
            yaw=controller.body_yaw,
        )

        self.assertEqual(gait.body_offsets, [(0.020, 0.010)])
        for value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(value, expected_value, places=10)
        self.assertTrue(
            any(
                abs(value - wrong_value) > 1e-4
                for value, wrong_value in zip(actual, double_counted)
            )
        )


class PhysicalTestControllerTests(unittest.TestCase):
    REQUEST_ID = "support_test_001"

    @classmethod
    def request(cls, command, mode="single-leg-lift", duration=4.0):
        return SimpleNamespace(
            data=physical_test_request_json(
                command,
                mode,
                duration,
                cls.REQUEST_ID,
                leg="front_left" if mode == "single-leg-lift" else None,
            )
        )

    def test_start_is_rejected_until_hardware_test_gate_is_enabled(self):
        controller = make_controller()
        controller.state = "standing"
        controller.commanded_positions = list(WALK_POSE)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)

        controller.physical_test_callback(self.request("start"))

        self.assertIsNone(controller.physical_test)
        self.assertIn("enable_physical_tests", controller.warning)

    def test_leased_test_runs_through_canonical_ik_and_returns_to_nominal(self):
        controller = enable_physical_test_controller(make_controller())

        controller.physical_test_callback(self.request("start"))
        self.assertIsNotNone(controller.physical_test)
        self.assertFalse(controller.arm_neutral_ready())

        controller._test_now = 12.0
        controller.physical_test_callback(self.request("keepalive"))
        joints = controller.physical_test_target(controller._test_now)
        self.assertEqual(len(joints), len(JOINT_NAMES))
        self.assertTrue(all(math.isfinite(value) for value in joints))
        self.assertGreater(
            controller.physical_test["current_feet"]["front_left"][2],
            NOMINAL_FEET["front_left"][2],
        )

        controller._test_now = 14.0
        controller.physical_test_callback(self.request("keepalive"))
        returned = controller.physical_test_target(controller._test_now)
        self.assertIsNotNone(controller.physical_test)
        self.assertTrue(controller.physical_test["filter_settling"])
        self.assertTrue(controller.motion_active)
        lagged = list(returned)
        lagged[0] += math.radians(2.0)
        controller.commanded_positions = lagged
        controller.commanded_velocities = [math.radians(4.0)] * len(JOINT_NAMES)
        self.assertFalse(
            controller.complete_physical_test_after_filter(returned, lagged)
        )
        self.assertIsNotNone(controller.physical_test)
        controller.commanded_positions = list(returned)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        self.assertTrue(
            controller.complete_physical_test_after_filter(returned, returned)
        )
        self.assertIsNone(controller.physical_test)
        self.assertEqual(controller.standing_feet, NOMINAL_FEET)

    def test_keepalive_timeout_causes_smooth_return_before_completion(self):
        controller = enable_physical_test_controller(make_controller())
        controller.physical_test_callback(self.request("start"))

        controller._test_now = 12.0
        controller.physical_test_target(controller._test_now)
        self.assertIsNotNone(
            controller.physical_test["cancel_start_time"]
        )
        self.assertIn("keepalive timeout", controller.warning)

        controller._test_now = 12.5
        controller.physical_test_target(controller._test_now)
        self.assertIsNotNone(controller.physical_test)
        controller._test_now = 13.0
        returned = controller.physical_test_target(controller._test_now)
        self.assertIsNotNone(controller.physical_test)
        controller.commanded_positions = list(returned)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        controller.complete_physical_test_after_filter(returned, returned)
        self.assertIsNone(controller.physical_test)

    def test_nonzero_velocity_cancels_test_and_is_not_cached(self):
        controller = enable_physical_test_controller(make_controller())
        controller.physical_test_callback(self.request("start"))

        controller._test_now = 10.1
        controller.velocity_callback(make_twist(x=0.03))

        self.assertEqual(controller.requested_velocity, [0.0, 0.0, 0.0])
        self.assertIsNotNone(
            controller.physical_test["cancel_start_time"]
        )
        self.assertIn("non-zero velocity", controller.warning)

    def test_body_pose_is_rejected_instead_of_deferred_through_test(self):
        controller = enable_physical_test_controller(make_controller())
        controller.physical_test_callback(self.request("start"))

        controller.body_pose_callback(make_twist(x=0.020))

        self.assertEqual(controller.body_x, 0.0)
        self.assertEqual(controller.body_y, 0.0)
        self.assertIn("Body-pose command rejected", controller.warning)

    def test_owner_loss_discards_test_without_advancing_trajectory(self):
        controller = enable_physical_test_controller(make_controller())
        controller.physical_test_callback(self.request("start"))

        controller.cancel_motion_for_owner_loss(10.1, force_hold=True)

        self.assertIsNone(controller.physical_test)
        self.assertFalse(controller.motion_active)


class StatusSchemaTests(unittest.TestCase):
    def test_status_contains_extended_gait_and_diagnostic_schema(self):
        gait = FakeGaitController(
            active=True,
            swing_legs=["rear_right"],
        )
        controller = make_controller(gait)
        controller.last_status_time = 0.0
        controller.state = "standing"
        controller.gait_name = "spotmicro_video_walk"
        controller.requested_gait = "slow_trot"
        controller.pending_gait = "slow_trot"
        controller.motion_active = True
        controller.step_in_place = False
        controller.projected_targets = ["front_left"]
        controller.clamped_joints = ["rear_right_foot"]
        controller.filtered_velocity = [0.01, 0.0, 0.02]
        controller.measured_positions = None
        controller.commanded_positions = None
        controller.command_publisher = RecordingPublisher(
            subscription_count=1,
        )
        controller.status_publisher = RecordingPublisher()
        controller.effective_gait_limits = lambda: {
            "spotmicro_video_walk": {
                "max_x": 0.01,
                "max_y": 0.005,
                "max_yaw": 0.18,
            },
        }
        controller.active_speed_scale = lambda _gait=None: 1.0

        controller.publish_status(1.0)

        self.assertEqual(len(controller.status_publisher.messages), 1)
        status_payload = controller.status_publisher.messages[0].data
        self.assertNotIn("Infinity", status_payload)
        self.assertNotIn("NaN", status_payload)
        status = json.loads(status_payload)
        required_fields = {
            "state",
            "requested_gait",
            "active_gait",
            "pending_gait",
            "pending_pose_action",
            "pose_transition_active",
            "pose_transition_kind",
            "pose_transition_progress",
            "pose_transition_settling",
            "moving",
            "motion_active",
            "step_in_place",
            "phase_index",
            "phase_name",
            "phase_progress",
            "cycle_phase",
            "swing_legs",
            "stance_legs",
            "body_shift",
            "body_shift_x",
            "body_shift_y",
            "body_shift_target_x",
            "body_shift_target_y",
            "support_target_x",
            "support_target_y",
            "support_margin",
            "shift_completion",
            "support_polygon_valid",
            "lift_allowed",
            "projected_targets",
            "clamped_joints",
            "filtered_velocity",
            "command_owner",
            "motion_authorized",
            "velocity_gate",
            "controller_connected",
            "joint_error",
            "warning",
            "gait_limits",
            "speed_scale",
            "hardware_mode",
            "open_loop_hardware",
            "requested_stride",
            "achieved_stride",
            "signed_stride",
            "stride_metric_valid",
            "stride_metric",
            "stance_grounded",
            "stance_max_ground_error",
            "stance_ground_tolerance",
            "requested_step_height",
            "achieved_step_height",
            "configured_cycle_period",
            "current_cycle_period",
            "phase_rate_scale",
            "phase_transition_hold",
            "joint_velocity_clamp_count",
            "joint_braking_clamp_count",
            "joint_acceleration_clamp_count",
            "tracking_required",
            "tracking_available",
            "tracking_ready",
            "tracking_assumed",
            "tracking_feedback_age",
        }
        self.assertTrue(required_fields.issubset(status))
        self.assertEqual(status["requested_gait"], "slow_trot")
        self.assertEqual(
            status["active_gait"],
            "spotmicro_video_walk",
        )
        self.assertEqual(status["pending_gait"], "slow_trot")
        self.assertEqual(status["swing_legs"], ["rear_right"])
        self.assertEqual(status["projected_targets"], ["front_left"])
        self.assertEqual(status["clamped_joints"], ["rear_right_foot"])
        self.assertTrue(status["support_polygon_valid"])
        self.assertTrue(status["lift_allowed"])
        self.assertEqual(status["shift_completion"], 1.0)
        self.assertTrue(status["controller_connected"])
        self.assertTrue(status["motion_active"])
        self.assertFalse(status["pose_transition_active"])
        self.assertEqual(status["command_owner"], "MOTION")
        self.assertTrue(status["motion_authorized"])
        self.assertIsNone(status["tracking_feedback_age"])
        self.assertTrue(
            all(math.isfinite(value) for value in status["filtered_velocity"])
        )


if __name__ == "__main__":
    unittest.main()
