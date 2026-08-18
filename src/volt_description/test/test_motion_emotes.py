#!/usr/bin/env python3

"""Pure controller-boundary regressions for ``/volt/emote`` playback."""

import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import rclpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from volt_emote_engine import CartesianEmoteEngine, load_builtin_catalog  # noqa: E402
from volt_gait_controller import GAITS  # noqa: E402
from volt_kinematics import (  # noqa: E402
    JOINT_NAMES,
    LEG_ORDER,
    NOMINAL_FEET,
    WALK_POSE,
    feet_to_joint_positions_diagnostic,
)
from volt_motion_controller import (  # noqa: E402
    DEFAULT_JOINT_ACCELERATION_LIMIT,
    SIMULATION_JOINT_VELOCITY_LIMIT,
    VELOCITY_GATE_OPEN,
    VoltMotionController,
)


class RecordingLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message, *_args, **_kwargs):
        self.infos.append(str(message))

    def warning(self, message, *_args, **_kwargs):
        self.warnings.append(str(message))


class FakeGaitController:
    def __init__(self, active=False):
        self.active = bool(active)
        self.swing_legs = []
        self.stop_requests = 0
        self.hold_calls = []

    def request_stop(self):
        self.stop_requests += 1

    def hold_current_feet(self, feet, now=None):
        self.hold_calls.append((feet, now))
        self.active = False
        self.swing_legs = []


def make_twist(x=0.0, y=0.0, yaw=0.0):
    return SimpleNamespace(
        linear=SimpleNamespace(x=x, y=y, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=yaw),
    )


def emote_message(command, request_id, **fields):
    payload = {"command": command, "request_id": request_id}
    payload.update(fields)
    return SimpleNamespace(data=json.dumps(payload))


def make_controller(catalog, gait_active=False):
    """Create all state needed by the production emote methods, without ROS."""
    controller = VoltMotionController.__new__(VoltMotionController)
    controller.emote_catalog = catalog
    controller.emote_engine = CartesianEmoteEngine(catalog)
    controller.pending_emote_request = None
    controller.active_emote_request = None
    controller.emote_request_id = ""
    controller.emote_result = "idle"
    controller.emote_message = ""
    controller.emote_progress = 0.0
    controller.emote_cancelled = False
    controller.emote_filter_settling = False
    controller.emote_keepalive_timeout = 0.75
    controller.emote_base_feet = {
        leg: tuple(NOMINAL_FEET[leg]) for leg in LEG_ORDER
    }
    controller.emote_base_body = {
        "height": 0.2,
        "body_x": 0.0,
        "body_y": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
    }
    controller.emote_swing_legs = []

    controller.gait_configs = {
        name: dict(config) for name, config in GAITS.items()
    }
    controller.gait_name = "spot_walk"
    controller.requested_gait = "spot_walk"
    controller.pending_gait = None
    controller.gait_controller = FakeGaitController(active=gait_active)
    controller.motion_active = bool(gait_active)
    controller.step_in_place = False
    controller.pending_pose_action = None
    controller.transition = None
    controller.physical_test = None
    controller.auto_ready_pending = False

    controller.command_owner = "MOTION"
    controller.state = "standing"
    controller.measured_positions = None
    controller.commanded_positions = list(WALK_POSE)
    controller.commanded_velocities = [0.0 for _ in JOINT_NAMES]
    controller.standing_feet = {
        leg: tuple(NOMINAL_FEET[leg]) for leg in LEG_ORDER
    }
    controller.last_gait_feet = dict(controller.standing_feet)
    controller.last_gait_body_transform = dict(controller.emote_base_body)

    controller.neutral_body_height = 0.2
    controller.body_height = 0.2
    controller.body_x = 0.0
    controller.body_y = 0.0
    controller.body_roll = 0.0
    controller.body_pitch = 0.0
    controller.body_yaw = 0.0

    controller.requested_velocity = [0.0, 0.0, 0.0]
    controller.filtered_velocity = [0.0, 0.0, 0.0]
    controller.velocity_command_sequence = 4
    controller.resume_after_velocity_sequence = -1
    controller.velocity_gate_state = VELOCITY_GATE_OPEN
    controller.velocity_message_count = 0
    controller.velocity_rate_window_count = 0
    controller.velocity_rate_window_start = 10.0
    controller.velocity_zero_transition_count = 0
    controller.last_velocity_was_neutral = True
    controller.last_velocity_time = 10.0

    controller.hardware_mode = False
    controller.open_loop_hardware = False
    controller.max_joint_velocity = 10.0
    controller.max_joint_acceleration = 100.0
    controller.applied_real_tuning = {
        "max_joint_velocity_deg_s": 60.0,
        "max_joint_acceleration_deg_s2": 120.0,
    }
    controller.warning = ""
    controller._test_now = 10.0
    controller.now_seconds = lambda: controller._test_now
    controller.logger = RecordingLogger()
    controller.get_logger = lambda: controller.logger
    controller.ik_records = []
    controller.record_ik_diagnostics = controller.ik_records.append
    return controller


def queue_and_start(
    controller,
    request_id="emote-1",
    name="push_ups",
    now=10.0,
    **options,
):
    controller._test_now = now
    fields = {"name": name}
    fields.update(options)
    controller.emote_callback(
        emote_message("start", request_id, **fields)
    )
    if controller.pending_emote_request is None:
        raise AssertionError(controller.emote_message)
    controller.gait_controller.active = False
    controller.motion_active = False
    if not controller.start_pending_emote(now):
        raise AssertionError(controller.emote_message)


def joint_distance(left, right):
    return sum(abs(float(a) - float(b)) for a, b in zip(left, right))


class MotionEmoteControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog = load_builtin_catalog()

    def test_request_schema_is_strict_and_cancel_is_correlated(self):
        controller = make_controller(self.catalog)

        controller.emote_callback(SimpleNamespace(data="[]"))
        self.assertEqual(controller.emote_result, "rejected")
        self.assertEqual(controller.emote_request_id, "")
        self.assertIsNone(controller.pending_emote_request)

        invalid = {
            "command": "start",
            "request_id": "bad-schema",
            "name": "push_ups",
            "unexpected": True,
        }
        controller.emote_callback(SimpleNamespace(data=json.dumps(invalid)))
        self.assertEqual(controller.emote_result, "rejected")
        self.assertEqual(controller.emote_request_id, "bad-schema")
        self.assertIn("unknown keys", controller.emote_message)

        controller.emote_callback(
            emote_message("start", "wanted", name="push_ups")
        )
        self.assertEqual(controller.emote_result, "queued")
        self.assertEqual(
            controller.pending_emote_request["request_id"], "wanted"
        )

        controller.emote_callback(emote_message("cancel", "stale"))
        self.assertEqual(controller.emote_result, "queued")
        self.assertEqual(controller.emote_request_id, "wanted")
        self.assertEqual(
            controller.pending_emote_request["request_id"], "wanted"
        )

        controller.emote_callback(emote_message("cancel", "wanted"))
        self.assertIsNone(controller.pending_emote_request)
        self.assertEqual(controller.emote_result, "cancelled")
        self.assertEqual(controller.emote_request_id, "wanted")

    def test_keepalive_renews_only_the_matching_strict_request(self):
        controller = make_controller(self.catalog)
        controller.emote_callback(
            emote_message("start", "leased", name="push_ups")
        )
        self.assertEqual(
            controller.pending_emote_request["last_keepalive_time"],
            10.0,
        )

        controller._test_now = 10.5
        controller.emote_callback(emote_message("keepalive", "leased"))
        self.assertEqual(
            controller.pending_emote_request["last_keepalive_time"],
            10.5,
        )

        controller._test_now = 10.6
        controller.emote_callback(emote_message("keepalive", "stale"))
        self.assertEqual(controller.emote_request_id, "leased")
        self.assertEqual(controller.emote_result, "queued")
        self.assertEqual(
            controller.pending_emote_request["last_keepalive_time"],
            10.5,
        )

        controller.emote_callback(
            emote_message("keepalive", "leased", name="not-allowed")
        )
        self.assertEqual(controller.emote_request_id, "leased")
        self.assertEqual(controller.emote_result, "queued")
        self.assertEqual(
            controller.pending_emote_request["last_keepalive_time"],
            10.5,
        )

    def test_keepalive_timeout_cancels_pending_or_returns_active_motion(self):
        pending = make_controller(self.catalog)
        pending.emote_callback(
            emote_message("start", "pending-lease", name="nod")
        )
        self.assertTrue(pending.expire_emote_if_stale(10.76))
        self.assertIsNone(pending.pending_emote_request)
        self.assertEqual(pending.emote_result, "cancelled")
        self.assertIn("lease expired", pending.warning.lower())

        active = make_controller(self.catalog)
        queue_and_start(active, request_id="active-lease", name="push_ups")
        self.assertTrue(active.expire_emote_if_stale(10.76))
        self.assertEqual(active.emote_result, "returning")
        self.assertEqual(active.emote_engine.state, "returning")
        self.assertIsNotNone(active.active_emote_request)

    def test_late_keepalive_cannot_overwrite_terminal_result(self):
        controller = make_controller(self.catalog)
        controller.emote_request_id = "finished"
        controller.emote_result = "completed"
        controller.emote_message = "Completed push_ups."

        controller.emote_callback(emote_message("keepalive", "finished"))

        self.assertEqual(controller.emote_request_id, "finished")
        self.assertEqual(controller.emote_result, "completed")
        self.assertEqual(controller.emote_message, "Completed push_ups.")

    def test_queued_request_starts_only_after_locomotion_is_idle(self):
        controller = make_controller(self.catalog, gait_active=True)
        controller.emote_callback(
            emote_message("start", "after-stop", name="nod")
        )

        self.assertEqual(controller.emote_result, "queued")
        self.assertGreater(controller.gait_controller.stop_requests, 0)
        self.assertFalse(controller.start_pending_emote(10.1))
        self.assertIsNotNone(controller.pending_emote_request)
        self.assertFalse(controller.emote_engine.active)

        controller.gait_controller.active = False
        controller.motion_active = False
        self.assertTrue(controller.start_pending_emote(10.2))
        self.assertIsNone(controller.pending_emote_request)
        self.assertEqual(
            controller.active_emote_request["request_id"], "after-stop"
        )
        self.assertEqual(controller.emote_result, "running")
        self.assertTrue(controller.emote_engine.active)

    def test_cartesian_frame_is_solved_through_canonical_ik(self):
        controller = make_controller(self.catalog)
        queue_and_start(controller)

        positions = controller.emote_target(10.8)
        expected, diagnostics = feet_to_joint_positions_diagnostic(
            controller.last_gait_feet,
            **controller.last_gait_body_transform,
        )

        self.assertEqual(len(positions), len(JOINT_NAMES))
        self.assertTrue(all(math.isfinite(value) for value in positions))
        self.assertEqual(positions, expected)
        self.assertEqual(diagnostics["projected_targets"], [])
        self.assertEqual(controller.ik_records[-1]["projected_targets"], [])
        self.assertAlmostEqual(
            controller.last_gait_body_transform["height"], 0.180
        )
        self.assertGreater(joint_distance(positions, WALK_POSE), 1e-3)

    def test_push_up_gui_maximum_reaches_25_mm_after_composed_preflight(self):
        controller = make_controller(self.catalog)
        queue_and_start(controller, request_id="pushup-25mm", depth=1.25)

        positions = controller.emote_target(10.8)

        self.assertAlmostEqual(
            controller.active_emote_request["depth"],
            1.25,
        )
        self.assertAlmostEqual(
            controller.last_gait_body_transform["height"],
            0.175,
        )
        self.assertEqual(
            controller.ik_records[-1]["projected_targets"],
            [],
        )
        self.assertTrue(all(math.isfinite(value) for value in positions))

    def test_push_up_gui_maximum_rejects_an_already_lower_stance(self):
        controller = make_controller(self.catalog)
        # The emote floor is 0.132 m, so the GUI maximum (60 mm) is legal from
        # a 0.195 m stand.  The contract under test is that it stops being
        # legal once the robot already stands low: 0.175 - 0.060 = 0.115 m.
        controller.body_height = 0.175
        settled, diagnostics = feet_to_joint_positions_diagnostic(
            controller.standing_feet,
            height=controller.body_height,
        )
        self.assertEqual(diagnostics["projected_targets"], [])
        controller.commanded_positions = settled
        controller._test_now = 10.0
        controller.emote_callback(
            emote_message(
                "start",
                "pushup-too-low",
                name="push_ups",
                depth=3.0,
            )
        )
        controller.gait_controller.active = False
        controller.motion_active = False

        self.assertFalse(controller.start_pending_emote(10.0))
        self.assertIsNone(controller.active_emote_request)
        self.assertEqual(controller.emote_result, "rejected")
        self.assertIn("height target", controller.emote_message)

    def test_nonzero_velocity_cancels_active_playback_into_return(self):
        controller = make_controller(self.catalog)
        queue_and_start(controller, request_id="velocity-cancel")
        controller.emote_target(10.4)
        controller._test_now = 10.4

        controller.velocity_callback(make_twist(x=0.02))

        self.assertEqual(controller.requested_velocity, [0.0, 0.0, 0.0])
        self.assertEqual(controller.last_velocity_time, 10.4)
        self.assertEqual(controller.emote_result, "returning")
        self.assertEqual(controller.emote_engine.state, "returning")
        self.assertEqual(controller.velocity_message_count, 0)
        self.assertEqual(
            controller.active_emote_request["request_id"],
            "velocity-cancel",
        )

    def test_stop_returns_smoothly_then_completes_as_cancelled(self):
        controller = make_controller(self.catalog)
        queue_and_start(controller, request_id="stop-cancel")
        lowered = controller.emote_target(10.8)
        controller._test_now = 10.8

        controller.stop_motion()

        self.assertEqual(controller.emote_result, "returning")
        self.assertEqual(controller.emote_engine.state, "returning")
        return_start = controller.emote_target(10.8)
        halfway = controller.emote_target(11.3)
        returned = controller.emote_target(11.8)

        self.assertEqual(return_start, lowered)
        self.assertLess(
            joint_distance(halfway, WALK_POSE),
            joint_distance(lowered, WALK_POSE),
        )
        self.assertGreater(joint_distance(halfway, WALK_POSE), 1e-4)
        self.assertLess(joint_distance(returned, WALK_POSE), 1e-9)
        self.assertEqual(controller.emote_result, "settling")
        self.assertTrue(controller.emote_filter_settling)
        self.assertIsNotNone(controller.active_emote_request)
        self.assertTrue(controller.motion_active)

        controller.commanded_velocities = [math.radians(5.0)] * len(JOINT_NAMES)
        lagged = list(returned)
        lagged[0] += math.radians(2.0)
        self.assertFalse(
            controller.complete_emote_after_filter(returned, lagged)
        )
        self.assertEqual(controller.emote_result, "settling")

        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        self.assertTrue(
            controller.complete_emote_after_filter(returned, returned)
        )
        self.assertEqual(controller.emote_result, "cancelled")
        self.assertEqual(controller.emote_request_id, "stop-cancel")
        self.assertIsNone(controller.active_emote_request)
        self.assertEqual(controller.emote_engine.state, "idle")
        self.assertFalse(controller.motion_active)

    def test_owner_loss_force_resets_emote_and_resynchronizes_hold(self):
        controller = make_controller(self.catalog)
        queue_and_start(controller, request_id="owner-loss", name="wave_left")
        controller.emote_target(10.5)
        controller.command_owner = "HOLD"
        controller.body_x = 0.01
        controller.body_roll = 0.05

        controller.cancel_motion_for_owner_loss(10.5, force_hold=True)

        self.assertFalse(controller.emote_engine.active)
        self.assertEqual(controller.emote_engine.state, "idle")
        self.assertIsNone(controller.active_emote_request)
        self.assertIsNone(controller.pending_emote_request)
        self.assertEqual(controller.emote_result, "cancelled")
        self.assertEqual(controller.emote_request_id, "owner-loss")
        self.assertIn("ownership changed", controller.emote_message.lower())
        self.assertEqual(controller.state, "hold")
        self.assertEqual(controller.body_x, 0.0)
        self.assertEqual(controller.body_roll, 0.0)
        self.assertEqual(len(controller.gait_controller.hold_calls), 1)

    def test_emote_conditioning_uses_real_profile_only_on_hardware(self):
        controller = make_controller(self.catalog)
        controller.emote_engine.start("push_ups", 10.0)
        knee_index = 2

        controller.hardware_mode = True
        self.assertAlmostEqual(
            controller.joint_velocity_limit(knee_index),
            math.radians(60.0),
        )
        self.assertAlmostEqual(
            controller.joint_acceleration_limit(knee_index),
            math.radians(120.0),
        )

        controller.hardware_mode = False
        self.assertAlmostEqual(
            controller.joint_velocity_limit(knee_index),
            SIMULATION_JOINT_VELOCITY_LIMIT,
        )
        self.assertAlmostEqual(
            controller.joint_acceleration_limit(knee_index),
            DEFAULT_JOINT_ACCELERATION_LIMIT,
        )

    def test_real_hardware_node_initializes_with_diagnostic_profile(self):
        physical_trot = ROOT / "config" / "physical_fast_trot.yaml"
        rclpy.init(args=[
            "--ros-args",
            "-p", "hardware_mode:=true",
            "-p", "open_loop_hardware:=true",
            "-p", "use_sim_time:=false",
            "-p", "physical_fast_trot_config_file:=%s" % physical_trot,
        ])
        node = None
        try:
            node = VoltMotionController()
            self.assertTrue(node.hardware_mode)
            self.assertEqual(node.active_real_profile, "REAL_DIAGNOSTIC")
            self.assertEqual(node.gait_name, "diagnostic_crawl")
            self.assertAlmostEqual(
                node.max_joint_velocity,
                math.radians(100.0),
            )
            self.assertAlmostEqual(
                node.max_joint_acceleration,
                math.radians(240.0),
            )
            self.assertAlmostEqual(node.joint_smoothing_alpha, 0.85)
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
