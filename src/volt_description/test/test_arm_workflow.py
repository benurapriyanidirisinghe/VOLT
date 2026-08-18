#!/usr/bin/env python3

"""Regression tests for the dependency-free guided ARM state machine."""

from dataclasses import replace
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from volt_arm_workflow import (  # noqa: E402
    ArmSnapshot,
    EFFECT_OWNER_HOLD,
    EFFECT_OWNER_MOTION,
    EFFECT_SERIAL_ARM,
    EFFECT_SERIAL_HOLD,
    EFFECT_SERIAL_STATUS,
    EFFECT_ZERO_STOP,
    GuidedArmWorkflow,
    STATE_ARMED,
    STATE_CLAIMING,
    STATE_FAILED,
    STATE_SETTLING,
    STATE_WAITING_ACK,
)


START_EFFECTS = (
    EFFECT_ZERO_STOP,
    EFFECT_OWNER_MOTION,
    EFFECT_SERIAL_STATUS,
)
HOLD_EFFECTS = (
    EFFECT_ZERO_STOP,
    EFFECT_OWNER_HOLD,
    EFFECT_SERIAL_HOLD,
)


def safe_hold_snapshot(now=10.0, status_time=9.9, **overrides):
    values = {
        "now": now,
        "motion_status_time": status_time,
        "motion_state": "standing",
        "motion_moving": False,
        "motion_step_in_place": False,
        "motion_arm_neutral_ready": True,
        "motion_controller_connected": True,
        "router_status_time": status_time,
        "router_owner": "HOLD",
        "router_pose_valid": False,
        "serial_status_time": status_time,
        "hardware_enabled": True,
        "dry_run": False,
        "calibration_valid": True,
        "connected": True,
        "ready": True,
        "armed": False,
        "streaming": False,
        "pending": "",
        "bridge_motion_safe": True,
        "bridge_owner": "HOLD",
        "bridge_owner_fresh": True,
        "bridge_owner_allowed": False,
        "bridge_frame_ready": False,
        "bridge_frame_seq": 7,
    }
    values.update(overrides)
    return ArmSnapshot(**values)


def confirmed_snapshot(now, status_time=None, **overrides):
    if status_time is None:
        status_time = now - 0.01
    values = {
        "router_owner": "MOTION",
        "router_pose_valid": True,
        "bridge_owner": "MOTION",
        "bridge_owner_fresh": True,
        "bridge_owner_allowed": True,
        "bridge_frame_ready": True,
        "bridge_frame_seq": max(
            8,
            int(round((float(now) - 10.0) * 20.0)) + 8,
        ),
    }
    values.update(overrides)
    return replace(
        safe_hold_snapshot(now=now, status_time=status_time),
        **values,
    )


def advance_to_waiting_ack(workflow):
    self_start = safe_hold_snapshot()
    workflow.start(self_start)
    effects = workflow.update(
        confirmed_snapshot(now=10.10, status_time=10.05)
    )
    if effects != (EFFECT_ZERO_STOP, EFFECT_SERIAL_STATUS):
        raise AssertionError("workflow did not enter settling")
    effects = workflow.update(
        confirmed_snapshot(now=10.40, status_time=10.35)
    )
    if effects != (EFFECT_SERIAL_ARM,):
        raise AssertionError("workflow did not request ARM")


class GuidedArmWorkflowTests(unittest.TestCase):
    def test_happy_path_orders_effects_and_requires_acknowledged_stream(self):
        workflow = GuidedArmWorkflow()
        self.assertEqual(workflow.start(safe_hold_snapshot()), START_EFFECTS)
        self.assertEqual(workflow.state, STATE_CLAIMING)

        self.assertEqual(
            workflow.update(
                confirmed_snapshot(now=10.10, status_time=10.05)
            ),
            (EFFECT_ZERO_STOP, EFFECT_SERIAL_STATUS),
        )
        self.assertEqual(workflow.state, STATE_SETTLING)
        self.assertEqual(
            workflow.update(
                confirmed_snapshot(now=10.20, status_time=10.15)
            ),
            (),
        )
        self.assertEqual(
            workflow.update(
                confirmed_snapshot(now=10.40, status_time=10.35)
            ),
            (EFFECT_SERIAL_ARM,),
        )
        self.assertEqual(workflow.state, STATE_WAITING_ACK)
        self.assertEqual(
            workflow.update(
                confirmed_snapshot(
                    now=10.45,
                    status_time=10.44,
                    pending="ARM",
                )
            ),
            (),
        )
        self.assertEqual(
            workflow.update(
                confirmed_snapshot(
                    now=10.50,
                    status_time=10.49,
                    armed=True,
                    streaming=True,
                )
            ),
            (),
        )
        self.assertEqual(workflow.state, STATE_ARMED)

    def test_all_green_start_never_publishes_arm_without_new_statuses(self):
        workflow = GuidedArmWorkflow()
        snapshot = confirmed_snapshot(now=10.0, status_time=9.9)
        self.assertEqual(workflow.start(snapshot), START_EFFECTS)
        self.assertEqual(workflow.update(snapshot), ())
        self.assertEqual(workflow.state, STATE_CLAIMING)

    def test_local_and_bridge_motion_ownership_are_both_required(self):
        workflow = GuidedArmWorkflow()
        workflow.start(safe_hold_snapshot())

        local_only = confirmed_snapshot(
            now=10.1,
            status_time=10.05,
            bridge_owner="HOLD",
            bridge_owner_allowed=False,
        )
        self.assertEqual(workflow.update(local_only), ())
        self.assertEqual(workflow.state, STATE_CLAIMING)

        bridge_only = confirmed_snapshot(
            now=10.2,
            status_time=10.15,
            router_owner="HOLD",
        )
        self.assertEqual(workflow.update(bridge_only), ())
        self.assertEqual(workflow.state, STATE_CLAIMING)

    def test_start_gate_rejects_every_live_hardware_prerequisite_failure(self):
        base = safe_hold_snapshot()
        cases = {
            "hardware off": {"hardware_enabled": False},
            "dry run": {"dry_run": True},
            "bad calibration": {"calibration_valid": False},
            "disconnected": {"connected": False},
            "not ready": {"ready": False},
            "already armed": {"armed": True},
            "pending": {"pending": "STATUS"},
            "moving": {"motion_moving": True},
            "stepping": {"motion_step_in_place": True},
            "transition": {"motion_state": "standing_up"},
            "unverified pose": {"motion_arm_neutral_ready": False},
            "controller offline": {"motion_controller_connected": False},
            "stale motion": {"motion_status_time": 6.0},
            "stale router": {"router_status_time": 8.0},
            "stale serial": {"serial_status_time": 8.0},
            "disabled owner": {"router_owner": "DISABLED"},
        }
        for label, changes in cases.items():
            with self.subTest(label=label):
                snapshot = replace(base, **changes)
                workflow = GuidedArmWorkflow()
                self.assertTrue(workflow.start_blockers(snapshot))
                self.assertEqual(workflow.start(snapshot), ())
                self.assertEqual(workflow.state, STATE_FAILED)

    def test_operator_details_expand_grouped_motion_and_router_blockers(self):
        workflow = GuidedArmWorkflow()
        snapshot = safe_hold_snapshot(
            hardware_enabled=False,
            dry_run=True,
            connected=False,
            ready=False,
            motion_state="standing_up",
            motion_moving=True,
            motion_arm_neutral_ready=False,
            bridge_motion_safe=False,
            router_owner="DISABLED",
        )

        self.assertEqual(
            workflow.start_blockers(snapshot),
            (
                "live hardware mode is disabled",
                "the serial bridge is in dry-run",
                "the Arduino is not connected",
                "the Arduino firmware is not ready",
                "the controller has not certified the stopped calibrated WALK_POSE",
                "the command router is unavailable or in an incompatible state",
            ),
        )
        details = workflow.start_blocker_details(snapshot)
        self.assertIn(
            "controller state is STANDING_UP; STANDING or HOLD is required",
            details,
        )
        self.assertIn(
            "the stopped pose is not certified as the calibrated WALK_POSE",
            details,
        )
        self.assertIn(
            "the motion controller still reports active movement",
            details,
        )
        self.assertIn(
            "the serial bridge has not certified motion_safe",
            details,
        )
        self.assertIn(
            "command-router owner is DISABLED; HOLD or MOTION is required",
            details,
        )

    def test_only_verified_walk_pose_is_armable(self):
        workflow = GuidedArmWorkflow()
        self.assertTrue(workflow.can_start(safe_hold_snapshot()))
        self.assertFalse(
            workflow.can_start(
                safe_hold_snapshot(motion_state="sitting")
            )
        )
        self.assertTrue(
            workflow.can_start(
                safe_hold_snapshot(
                    motion_state="hold",
                    motion_arm_neutral_ready=True,
                )
            )
        )
        for state in ("standing_up", "sitting_down", "walking"):
            with self.subTest(state=state):
                self.assertFalse(
                    workflow.can_start(
                        safe_hold_snapshot(motion_state=state)
                    )
                )
        for state in ("standing", "hold"):
            with self.subTest(state=state, verified=False):
                self.assertFalse(
                    workflow.can_start(
                        safe_hold_snapshot(
                            motion_state=state,
                            motion_arm_neutral_ready=False,
                        )
                    )
                )

    def test_pose_and_recent_frame_must_arrive_after_motion_claim(self):
        workflow = GuidedArmWorkflow(preparation_timeout=0.5)
        workflow.start(safe_hold_snapshot())
        missing_pose = confirmed_snapshot(
            now=10.1,
            status_time=10.05,
            router_pose_valid=False,
        )
        self.assertEqual(workflow.update(missing_pose), ())
        missing_frame = confirmed_snapshot(
            now=10.2,
            status_time=10.15,
            bridge_frame_ready=False,
        )
        self.assertEqual(workflow.update(missing_frame), ())
        self.assertEqual(
            workflow.update(
                replace(missing_frame, now=10.6, motion_status_time=10.55)
            ),
            HOLD_EFFECTS,
        )

    def test_new_safe_samples_are_required_after_the_second_stop(self):
        workflow = GuidedArmWorkflow()
        workflow.start(safe_hold_snapshot())
        confirmed = confirmed_snapshot(now=10.1, status_time=10.05)
        self.assertEqual(
            workflow.update(confirmed),
            (EFFECT_ZERO_STOP, EFFECT_SERIAL_STATUS),
        )
        self.assertEqual(
            workflow.update(replace(confirmed, now=10.5)),
            (),
        )
        self.assertEqual(workflow.state, STATE_SETTLING)
        self.assertEqual(
            workflow.update(
                confirmed_snapshot(now=10.55, status_time=10.54)
            ),
            (EFFECT_SERIAL_ARM,),
        )

    def test_new_frame_sequence_is_required_after_each_stop(self):
        workflow = GuidedArmWorkflow()
        workflow.start(safe_hold_snapshot(bridge_frame_seq=20))

        self.assertEqual(
            workflow.update(
                confirmed_snapshot(
                    now=10.10,
                    status_time=10.05,
                    bridge_frame_seq=20,
                )
            ),
            (),
        )
        self.assertEqual(workflow.state, STATE_CLAIMING)
        self.assertEqual(
            workflow.update(
                confirmed_snapshot(
                    now=10.15,
                    status_time=10.14,
                    bridge_frame_seq=21,
                )
            ),
            (EFFECT_ZERO_STOP, EFFECT_SERIAL_STATUS),
        )
        self.assertEqual(
            workflow.update(
                confirmed_snapshot(
                    now=10.50,
                    status_time=10.49,
                    bridge_frame_seq=21,
                )
            ),
            (),
        )
        self.assertEqual(workflow.state, STATE_SETTLING)
        self.assertEqual(
            workflow.update(
                confirmed_snapshot(
                    now=10.55,
                    status_time=10.54,
                    bridge_frame_seq=22,
                )
            ),
            (EFFECT_SERIAL_ARM,),
        )

    def test_arm_is_emitted_exactly_once_across_repeated_updates(self):
        workflow = GuidedArmWorkflow()
        advance_to_waiting_ack(workflow)
        observed = []
        for now in (10.41, 10.45, 10.55, 10.70):
            observed.extend(
                workflow.update(
                    confirmed_snapshot(
                        now=now,
                        status_time=now - 0.01,
                        pending="ARM",
                    )
                )
            )
        self.assertNotIn(EFFECT_SERIAL_ARM, observed)
        self.assertEqual(workflow.state, STATE_WAITING_ACK)

    def test_workflow_owned_status_pending_does_not_cancel_arming(self):
        workflow = GuidedArmWorkflow()
        workflow.start(safe_hold_snapshot())
        self.assertEqual(
            workflow.update(
                confirmed_snapshot(
                    now=10.10,
                    status_time=10.05,
                    pending="STATUS",
                )
            ),
            (EFFECT_ZERO_STOP, EFFECT_SERIAL_STATUS),
        )
        self.assertEqual(
            workflow.update(
                confirmed_snapshot(
                    now=10.40,
                    status_time=10.35,
                    pending="STATUS",
                )
            ),
            (EFFECT_SERIAL_ARM,),
        )
        self.assertEqual(
            workflow.update(
                confirmed_snapshot(
                    now=10.42,
                    status_time=10.41,
                    pending="STATUS",
                )
            ),
            (),
        )
        self.assertEqual(workflow.state, STATE_WAITING_ACK)

    def test_interlock_loss_while_waiting_ack_fails_to_both_hold_layers(self):
        workflow = GuidedArmWorkflow()
        advance_to_waiting_ack(workflow)
        effects = workflow.update(
            confirmed_snapshot(
                now=10.5,
                status_time=10.49,
                router_owner="HOLD",
                bridge_owner_allowed=False,
                pending="ARM",
                armed=True,
                streaming=True,
            )
        )
        self.assertEqual(effects, HOLD_EFFECTS)
        self.assertEqual(workflow.state, STATE_FAILED)

    def test_ack_timeout_and_armed_without_streaming_fail_closed(self):
        timeout_workflow = GuidedArmWorkflow(acknowledgement_timeout=0.2)
        advance_to_waiting_ack(timeout_workflow)
        self.assertEqual(
            timeout_workflow.update(
                confirmed_snapshot(
                    now=10.61,
                    status_time=10.60,
                    pending="ARM",
                )
            ),
            HOLD_EFFECTS,
        )

        inhibited_workflow = GuidedArmWorkflow()
        advance_to_waiting_ack(inhibited_workflow)
        self.assertEqual(
            inhibited_workflow.update(
                confirmed_snapshot(
                    now=10.5,
                    status_time=10.49,
                    armed=True,
                    streaming=False,
                )
            ),
            HOLD_EFFECTS,
        )

    def test_conflicting_pending_command_fails_closed(self):
        workflow = GuidedArmWorkflow()
        advance_to_waiting_ack(workflow)
        self.assertEqual(
            workflow.update(
                confirmed_snapshot(
                    now=10.5,
                    status_time=10.49,
                    pending="DISABLE",
                )
            ),
            HOLD_EFFECTS,
        )

    def test_cancel_is_immediate_idempotent_and_prevents_later_arm(self):
        workflow = GuidedArmWorkflow()
        workflow.start(safe_hold_snapshot())
        self.assertEqual(workflow.cancel(), HOLD_EFFECTS)
        self.assertEqual(workflow.cancel(), ())
        self.assertEqual(
            workflow.update(
                confirmed_snapshot(now=10.5, status_time=10.49)
            ),
            (),
        )


if __name__ == "__main__":
    unittest.main()
