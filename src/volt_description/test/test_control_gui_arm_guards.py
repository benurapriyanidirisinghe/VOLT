#!/usr/bin/env python3

"""Adapter-level regression tests for the GUI's guided ARM command guards."""

from pathlib import Path
import ast
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import volt_control_gui as gui  # noqa: E402


class _Workflow:
    def __init__(self, active):
        self.active = bool(active)


class _GuardedWindow:
    def __init__(self, active=True):
        self.shutting_down = False
        self.arm_workflow = _Workflow(active)
        self.blocked = []
        self.cancelled = []
        self.actions = []
        self.both_layer_holds = []
        self.ros_node = type(
            "RosNode",
            (),
            {
                "action_publisher": object(),
                "publish_text": lambda owner, publisher, action: (
                    self.actions.append(action)
                ),
            },
        )()

    def arm_mutation_is_blocked(self, description):
        if not self.arm_workflow.active:
            return False
        self.blocked.append(description)
        return True

    def cancel_arm_workflow(self, reason, **_kwargs):
        self.cancelled.append(reason)

    def send_action(self, action):
        return gui.VoltControlWindow.send_action(self, action)

    def return_both_layers_to_hold(self, reason, serial_command="HOLD"):
        self.both_layer_holds.append((reason, serial_command))


class _SnapshotWindow:
    last_serial_status_fields = {
        "hardware_enabled": "1",
        "dry_run": "0",
        "connected": "1",
        "ready": "1",
    }
    last_motion_status_time = 9.9
    motion_state = "standing"
    motion_moving = False
    motion_step_in_place = False
    motion_arm_neutral_ready = False
    motion_controller_connected = True
    last_router_status_time = 9.9
    command_owner = "HOLD"
    router_pose_valid = False
    last_serial_status_time = 9.9


class _FakeControl:
    def __init__(self, text=""):
        self.text = text
        self.enabled = None
        self.message = ""
        self.tooltip = ""

    def currentText(self):
        return self.text

    def setEnabled(self, enabled):
        self.enabled = bool(enabled)

    def setText(self, message):
        self.message = str(message)

    def setStyleSheet(self, _style):
        pass

    def setToolTip(self, tooltip):
        self.tooltip = str(tooltip)


def _armable_snapshot(**overrides):
    values = {
        "now": 10.0,
        "motion_status_time": 9.9,
        "motion_state": "standing",
        "motion_moving": False,
        "motion_step_in_place": False,
        "motion_arm_neutral_ready": True,
        "motion_controller_connected": True,
        "router_status_time": 9.9,
        "router_owner": "MOTION",
        "router_pose_valid": True,
        "serial_status_time": 9.9,
        "hardware_enabled": True,
        "dry_run": False,
        "calibration_valid": True,
        "connected": True,
        "ready": True,
        "armed": False,
        "streaming": False,
        "pending": "",
        "bridge_motion_safe": True,
        "bridge_owner": "MOTION",
        "bridge_owner_fresh": True,
        "bridge_owner_allowed": True,
        "bridge_frame_ready": True,
        "bridge_frame_seq": 12,
    }
    values.update(overrides)
    return gui.ArmSnapshot(**values)


class GuiArmGuardTests(unittest.TestCase):
    def test_every_displayed_cartesian_emote_has_a_ready_catalog_mapping(self):
        displayed = [
            name for _caption, name in gui.DISPLAYED_CARTESIAN_EMOTES
        ]
        self.assertEqual(
            displayed,
            [
                "push_ups",
                "body_roll",
                "nod",
                "wave_left",
                "wave_right",
                "heart",
                "bow",
                "stretch",
                "happy_dance",
                "shake_no",
            ],
        )
        self.assertEqual(len(displayed), len(set(displayed)))
        for name in displayed:
            with self.subTest(name=name):
                self.assertEqual(
                    gui.emote_start_blocker(
                        name=name,
                        advertised=displayed,
                        command_owner="MOTION",
                        status_fresh=True,
                        controller_connected=True,
                        motion_state="standing",
                    ),
                    "",
                )

    def test_pushup_travel_maps_to_depth_without_changing_other_emotes(self):
        common = {
            "repetitions": 2,
            "speed": 1.2,
            "amplitude": 0.8,
            "depth": 1.4,
        }
        expected_depths = {
            10.0: 0.5,
            20.0: 1.0,
            25.0: 1.25,
        }
        for travel_mm, expected_depth in expected_depths.items():
            with self.subTest(travel_mm=travel_mm):
                options = gui.gui_emote_request_options(
                    name="push_ups",
                    pushup_travel_mm=travel_mm,
                    **common,
                )
                self.assertEqual(options["repetitions"], 2)
                self.assertAlmostEqual(options["speed"], 1.2)
                self.assertAlmostEqual(options["amplitude"], 0.8)
                self.assertAlmostEqual(options["depth"], expected_depth)

        for _caption, name in gui.DISPLAYED_CARTESIAN_EMOTES:
            if name == "push_ups":
                continue
            with self.subTest(name=name):
                options = gui.gui_emote_request_options(
                    name=name,
                    pushup_travel_mm=25.0,
                    **common,
                )
                self.assertAlmostEqual(options["depth"], 1.4)

    def test_pushup_travel_payload_is_clamped_to_gui_safety_range(self):
        low = gui.gui_emote_request_options(
            "push_ups", 1, 1.0, 1.0, 1.0, -100.0
        )
        high = gui.gui_emote_request_options(
            "push_ups", 1, 1.0, 1.0, 1.0, 100.0
        )
        self.assertAlmostEqual(low["depth"], 0.5)
        # 60 mm ceiling / 20 mm base keyframe = 3.0x
        self.assertAlmostEqual(high["depth"], 3.0)

    def test_start_emote_publishes_dedicated_pushup_travel_only_for_pushups(self):
        window = gui.VoltControlWindow.__new__(gui.VoltControlWindow)
        window.hardware_mode = False
        window.active_emote_request = None
        window.emote_was_active = False
        window.emote_notice = ""
        window.emote_notice_color = "#94a3b8"
        window.emote_notice_time = 0.0
        window.emote_status = _FakeControl()
        window.emote_repetitions = SimpleNamespace(value=lambda: 2.0)
        window.emote_speed = SimpleNamespace(value=lambda: 1.2)
        window.emote_amplitude = SimpleNamespace(value=lambda: 0.8)
        window.emote_depth = SimpleNamespace(value=lambda: 1.4)
        window.pushup_travel_mm = SimpleNamespace(value=lambda: 25.0)
        published = []
        window.ros_node = SimpleNamespace(
            emote_publisher=object(),
            publish_json=lambda _publisher, payload: published.append(payload)
            or True,
        )
        window.arm_mutation_is_blocked = lambda _description: False
        window.emote_button_blocker = lambda _name: ""
        window.latch_motion_until_neutral = lambda: None
        window.neutralize_motion_controls = lambda: None
        window.refresh_emote_controls = lambda _busy: None

        gui.VoltControlWindow.start_emote(window, "push_ups")
        self.assertAlmostEqual(published[-1]["depth"], 1.25)
        self.assertNotIn("pushup_travel_mm", published[-1])

        window.active_emote_request = None
        gui.VoltControlWindow.start_emote(window, "bow")
        self.assertAlmostEqual(published[-1]["depth"], 1.4)

    def test_pushup_travel_control_follows_emote_busy_and_arm_locks(self):
        window = gui.VoltControlWindow.__new__(gui.VoltControlWindow)
        window.controller_emote_busy = False
        window.active_emote_request = None
        window.arm_controls_locked = False
        window.emote_buttons_by_name = {}
        window.emote_action_buttons_by_name = {}
        window.emote_repetitions = _FakeControl()
        window.emote_speed = _FakeControl()
        window.emote_amplitude = _FakeControl()
        window.emote_depth = _FakeControl()
        window.pushup_travel_mm = _FakeControl()
        window.stop_emote_button = _FakeControl()

        gui.VoltControlWindow.refresh_emote_controls(window, False)
        self.assertTrue(window.pushup_travel_mm.enabled)
        gui.VoltControlWindow.refresh_emote_controls(window, True)
        self.assertFalse(window.pushup_travel_mm.enabled)
        window.arm_controls_locked = True
        gui.VoltControlWindow.refresh_emote_controls(window, False)
        self.assertFalse(window.pushup_travel_mm.enabled)

    def test_emote_button_gate_explains_catalog_and_runtime_blockers(self):
        ready = {
            "name": "bow",
            "advertised": ["bow"],
            "command_owner": "MOTION",
            "status_fresh": True,
            "controller_connected": True,
            "motion_state": "standing",
        }
        cases = (
            ({"advertised": []}, "did not advertise"),
            ({"command_owner": "HOLD"}, "ENABLE MOTION"),
            ({"status_fresh": False}, "fresh motion-controller status"),
            ({"controller_connected": False}, "not connected"),
            ({"motion_state": "sitting"}, "STANDING state"),
            ({"physical_busy": True}, "hardware diagnostic"),
            ({"emote_busy": True}, "current emote"),
        )
        for overrides, expected in cases:
            values = dict(ready)
            values.update(overrides)
            with self.subTest(expected=expected):
                self.assertIn(expected, gui.emote_start_blocker(**values))

    def test_pose_buttons_use_pose_state_instead_of_emote_catalog(self):
        common = {
            "advertised": [],
            "command_owner": "MOTION",
            "status_fresh": True,
            "controller_connected": True,
            "pose_action": True,
        }
        self.assertEqual(
            gui.emote_start_blocker(
                name="stand",
                motion_state="sitting",
                **common,
            ),
            "",
        )
        self.assertIn(
            "already STANDING",
            gui.emote_start_blocker(
                name="stand",
                motion_state="standing",
                **common,
            ),
        )
        self.assertIn(
            "transition",
            gui.emote_start_blocker(
                name="sit",
                motion_state="standing_up",
                **common,
            ),
        )

    def test_missing_catalog_fails_closed_and_external_emote_is_not_adopted(self):
        window = gui.VoltControlWindow.__new__(gui.VoltControlWindow)
        window.available_emotes = {"stale_emote"}
        window.emote_catalog_received = True
        window.emote_catalog_error = ""
        window.controller_emote_busy = False
        window.controller_emote_request_id = ""
        window.active_emote_request = None
        window.emote_was_active = False
        window.pending_emote_pose_action = None
        window.emote_notice = ""
        window.emote_notice_color = "#94a3b8"
        window.emote_notice_time = 0.0
        window.motion_state = "standing"
        window.emote_status = _FakeControl()
        busy_values = []
        window.refresh_emote_controls = busy_values.append

        gui.VoltControlWindow.update_emote_status(window, {})
        self.assertEqual(window.available_emotes, set())
        self.assertFalse(window.emote_catalog_received)
        self.assertIn("no emote catalog", window.emote_status.message)

        gui.VoltControlWindow.update_emote_status(
            window,
            {
                "emotes_available": ["bow"],
                "emote_active": True,
                "emote_name": "bow",
                "emote_state": "running",
                "emote_request_id": "another-client",
                "emote_progress": 0.5,
            },
        )
        self.assertIsNone(window.active_emote_request)
        self.assertTrue(window.controller_emote_busy)
        self.assertEqual(window.controller_emote_request_id, "another-client")
        self.assertTrue(busy_values[-1])

    def test_unacknowledged_emote_request_times_out_with_visible_error(self):
        window = gui.VoltControlWindow.__new__(gui.VoltControlWindow)
        window.available_emotes = set()
        window.emote_catalog_received = False
        window.emote_catalog_error = ""
        window.controller_emote_busy = False
        window.controller_emote_request_id = ""
        window.active_emote_request = {
            "request_id": "ours",
            "name": "heart",
            "started_at": 1.0,
        }
        window.emote_was_active = False
        window.pending_emote_pose_action = None
        window.emote_notice = ""
        window.emote_notice_color = "#94a3b8"
        window.emote_notice_time = 0.0
        window.motion_state = "standing"
        window.emote_status = _FakeControl()
        busy_values = []
        window.refresh_emote_controls = busy_values.append

        with patch.object(gui.time, "monotonic", return_value=4.0):
            gui.VoltControlWindow.update_emote_status(
                window,
                {"emotes_available": ["heart"]},
            )

        self.assertIsNone(window.active_emote_request)
        self.assertIn("no correlated controller acknowledgement", window.emote_status.message)
        self.assertFalse(busy_values[-1])

    def test_pending_emote_ui_state_expires_if_status_disappears(self):
        window = gui.VoltControlWindow.__new__(gui.VoltControlWindow)
        window.last_motion_status_time = 0.0
        window.active_emote_request = {
            "request_id": "lost",
            "name": "bow",
            "started_at": 1.0,
        }
        window.pending_emote_pose_action = {
            "name": "sit",
            "started_at": 1.0,
        }
        window.emote_notice = ""
        window.emote_notice_color = "#94a3b8"
        window.emote_notice_time = 0.0
        window.emote_status = _FakeControl()

        self.assertTrue(
            gui.VoltControlWindow.expire_emote_requests_without_status(
                window,
                now=4.0,
            )
        )
        self.assertIsNone(window.active_emote_request)
        self.assertIsNone(window.pending_emote_pose_action)
        self.assertIn("status was lost", window.emote_status.message)

    def test_simulation_profile_is_visible_but_read_only(self):
        window = gui.VoltControlWindow.__new__(gui.VoltControlWindow)
        window.arm_controls_locked = False
        window.real_profile_combo = _FakeControl("SIMULATION")
        window.real_tuning_value_controls = [_FakeControl(), _FakeControl()]
        window.apply_real_profile_button = _FakeControl()
        window.save_real_profile_button = _FakeControl()
        window.real_tuning_status = _FakeControl()

        gui.VoltControlWindow.refresh_real_profile_editability(window)

        self.assertTrue(all(
            control.enabled is False
            for control in window.real_tuning_value_controls
        ))
        self.assertFalse(window.apply_real_profile_button.enabled)
        self.assertFalse(window.save_real_profile_button.enabled)
        self.assertIn("read-only", window.real_tuning_status.message)

        window.real_profile_combo.text = "REAL_SAFE"
        gui.VoltControlWindow.refresh_real_profile_editability(window)
        self.assertTrue(all(
            control.enabled is True
            for control in window.real_tuning_value_controls
        ))
        self.assertTrue(window.apply_real_profile_button.enabled)
        self.assertTrue(window.save_real_profile_button.enabled)

    def test_reported_profiles_do_not_replace_user_overlay(self):
        shipped = gui.load_profiles(include_user=False)
        user_safe = dict(shipped["REAL_SAFE"], cycle_duration=1.45)
        reported_safe = dict(shipped["REAL_SAFE"], cycle_duration=1.10)

        merged = gui.merge_reported_real_profiles(
            {"REAL_SAFE": user_safe},
            {
                "REAL_SAFE": reported_safe,
                "REAL_NORMAL": shipped["REAL_NORMAL"],
                "BROKEN": {"gait": "invalid"},
            },
            {"REAL_SAFE"},
        )

        self.assertEqual(merged["REAL_SAFE"], user_safe)
        self.assertEqual(merged["REAL_NORMAL"], shipped["REAL_NORMAL"])
        self.assertNotIn("BROKEN", merged)

    def test_emote_requests_have_no_cross_topic_stop_race(self):
        source = Path(gui.__file__).read_text(encoding="utf-8")
        tree = ast.parse(source)
        methods = {
            node.name: ast.get_source_segment(source, node) or ""
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef)
        }
        for method_name in ("start_emote", "stop_emote"):
            with self.subTest(method=method_name):
                self.assertIn(
                    "self.neutralize_motion_controls()",
                    methods[method_name],
                )
                self.assertNotIn(
                    "self.stop_motion_controls()",
                    methods[method_name],
                )

    def test_duplicate_topic_helper_detects_multiple_status_authorities(self):
        counts = {topic: 1 for topic in gui.CRITICAL_STACK_TOPICS}
        self.assertEqual(gui.duplicate_stack_topics(counts), ())
        counts["/volt/command_router_status"] = 2
        self.assertEqual(
            gui.duplicate_stack_topics(counts),
            ("/volt/command_router_status",),
        )

    def test_button_step_is_blocked_while_waiting_for_arm_ack(self):
        window = _GuardedWindow(active=True)
        with patch.object(gui.rclpy, "ok", return_value=True):
            gui.VoltControlWindow.send_action(window, "step")
        self.assertEqual(window.actions, [])
        self.assertEqual(window.cancelled, [])
        self.assertEqual(window.blocked, ["STEP action"])

    def test_gamepad_step_is_blocked_while_waiting_for_arm_ack(self):
        window = _GuardedWindow(active=True)
        with patch.object(gui.rclpy, "ok", return_value=True):
            gui.VoltControlWindow.handle_gamepad_action(window, "step")
        self.assertEqual(window.actions, [])
        self.assertEqual(window.cancelled, [])
        self.assertEqual(window.blocked, ["Gamepad STEP"])

    def test_stop_cancels_instead_of_reaching_the_action_topic(self):
        window = _GuardedWindow(active=True)
        with patch.object(gui.rclpy, "ok", return_value=True):
            gui.VoltControlWindow.send_action(window, "stop")
        self.assertEqual(window.actions, [])
        self.assertEqual(len(window.cancelled), 1)

    def test_safe_hardware_button_always_returns_both_layers_to_hold(self):
        window = _GuardedWindow(active=False)
        with patch.object(gui.rclpy, "ok", return_value=True):
            gui.VoltControlWindow.send_hardware_safe_command(window, "DISARM")
        self.assertEqual(
            window.both_layer_holds,
            [("ROS ownership returned to HOLD; firmware DISARM requested.", "DISARM")],
        )

    def test_missing_calibration_and_frame_sequence_fail_closed(self):
        snapshot = gui.VoltControlWindow.arm_snapshot(
            _SnapshotWindow(),
            now=10.0,
        )
        self.assertFalse(snapshot.calibration_valid)
        self.assertEqual(snapshot.bridge_frame_seq, -1)

    def test_visual_host_sync_status_never_changes_arm_snapshot(self):
        window = _SnapshotWindow()
        window.last_serial_status_fields = dict(
            _SnapshotWindow.last_serial_status_fields
        )
        before = gui.VoltControlWindow.arm_snapshot(window, now=10.0)
        window.last_serial_status_fields.update(
            {
                "face_loading": "1",
                "host_sync": "0",
                "host_sync_required": "1",
                "host_sync_state": "error",
                "host_sync_error": "ERR_HOST_SYNC",
            }
        )
        after = gui.VoltControlWindow.arm_snapshot(window, now=10.0)

        self.assertEqual(after, before)
        self.assertNotIn("host_sync", gui.ArmSnapshot.__dataclass_fields__)

    def test_disabled_arm_button_lists_every_live_stack_blocker(self):
        snapshot = _armable_snapshot(
            hardware_enabled=False,
            dry_run=True,
            connected=False,
            ready=False,
            motion_state="standing_up",
            motion_moving=True,
            motion_arm_neutral_ready=False,
            bridge_motion_safe=False,
        )
        window = gui.VoltControlWindow.__new__(gui.VoltControlWindow)
        window.duplicate_stack_active = False
        window.arm_workflow = gui.GuidedArmWorkflow()
        window.hardware_arm_button = _FakeControl()
        window.arm_readiness_state = _FakeControl()
        window.arm_blockers_label = _FakeControl()
        window.arm_snapshot = lambda: snapshot

        gui.VoltControlWindow.refresh_arm_button(window)

        self.assertFalse(window.hardware_arm_button.enabled)
        self.assertEqual(
            window.hardware_arm_button.message,
            "ARM LOCKED — 8 BLOCKERS",
        )
        self.assertEqual(
            window.arm_readiness_state.message,
            "ARM LOCKED — 8 BLOCKERS",
        )
        for expected in (
            "live hardware mode is disabled",
            "the serial bridge is in dry-run",
            "the Arduino is not connected",
            "the Arduino firmware is not ready",
            "controller state is STANDING_UP",
            "the stopped pose is not certified",
            "still reports active movement",
            "has not certified motion_safe",
        ):
            with self.subTest(expected=expected):
                self.assertIn(expected, window.arm_blockers_label.message)
                self.assertIn(expected, window.hardware_arm_button.tooltip)

    def test_arm_button_only_enables_when_readiness_panel_is_clear(self):
        snapshot = _armable_snapshot()
        window = gui.VoltControlWindow.__new__(gui.VoltControlWindow)
        window.duplicate_stack_active = False
        window.arm_workflow = gui.GuidedArmWorkflow()
        window.hardware_arm_button = _FakeControl()
        window.arm_readiness_state = _FakeControl()
        window.arm_blockers_label = _FakeControl()
        window.arm_snapshot = lambda: snapshot

        gui.VoltControlWindow.refresh_arm_button(window)

        self.assertTrue(window.hardware_arm_button.enabled)
        self.assertEqual(
            window.hardware_arm_button.message,
            "ARM SYSTEM SAFELY",
        )
        self.assertEqual(
            window.arm_readiness_state.message,
            "ARM READY — ALL PRE-FLIGHT GATES PASSED",
        )
        self.assertIn("re-check every gate", window.arm_blockers_label.message)


if __name__ == "__main__":
    unittest.main()
