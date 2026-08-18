#!/usr/bin/env python3

"""Pure fail-closed state machine for guided physical-robot arming."""

from dataclasses import dataclass


EFFECT_ZERO_STOP = "zero_stop"
EFFECT_OWNER_MOTION = "owner_motion"
EFFECT_SERIAL_STATUS = "serial_status"
EFFECT_SERIAL_ARM = "serial_arm"
EFFECT_OWNER_HOLD = "owner_hold"
EFFECT_SERIAL_HOLD = "serial_hold"

SAFE_HOLD_EFFECTS = (
    EFFECT_ZERO_STOP,
    EFFECT_OWNER_HOLD,
    EFFECT_SERIAL_HOLD,
)

STATE_IDLE = "idle"
STATE_CLAIMING = "claiming_motion"
STATE_SETTLING = "settling"
STATE_WAITING_ACK = "waiting_ack"
STATE_ARMED = "armed"
STATE_FAILED = "failed"
STATE_CANCELLED = "cancelled"

ACTIVE_STATES = (
    STATE_CLAIMING,
    STATE_SETTLING,
    STATE_WAITING_ACK,
)

TRANSIENT_FRAME_BLOCKERS = (
    "the bridge does not have a recent stable MOTION-owned frame",
    "the bridge has not reported a joint-frame sequence",
    "a new joint frame has not arrived after STOP",
)


@dataclass(frozen=True)
class ArmSnapshot:
    """One coherent view of all GUI, router, bridge, and controller gates."""

    now: float = 0.0
    motion_status_time: float = 0.0
    motion_state: str = "unknown"
    motion_moving: bool = True
    motion_step_in_place: bool = False
    motion_arm_neutral_ready: bool = False
    motion_controller_connected: bool = False
    router_status_time: float = 0.0
    router_owner: str = "UNKNOWN"
    router_pose_valid: bool = False
    serial_status_time: float = 0.0
    hardware_enabled: bool = False
    dry_run: bool = True
    calibration_valid: bool = False
    connected: bool = False
    ready: bool = False
    armed: bool = False
    streaming: bool = False
    pending: str = ""
    bridge_motion_safe: bool = False
    bridge_owner: str = "UNKNOWN"
    bridge_owner_fresh: bool = False
    bridge_owner_allowed: bool = False
    bridge_frame_ready: bool = False
    bridge_frame_seq: int = -1

    def timestamp_is_fresh(self, timestamp, timeout):
        age = float(self.now) - float(timestamp)
        return float(timestamp) > 0.0 and 0.0 <= age <= max(0.0, float(timeout))


class GuidedArmWorkflow:
    """Sequence STOP -> confirmed MOTION -> fresh frame -> acknowledged ARM."""

    def __init__(
        self,
        motion_timeout=3.0,
        router_timeout=1.0,
        serial_timeout=1.0,
        settle_duration=0.25,
        preparation_timeout=5.0,
        acknowledgement_timeout=2.0,
    ):
        self.motion_timeout = max(0.0, float(motion_timeout))
        self.router_timeout = max(0.0, float(router_timeout))
        self.serial_timeout = max(0.0, float(serial_timeout))
        self.settle_duration = max(0.0, float(settle_duration))
        self.preparation_timeout = max(0.1, float(preparation_timeout))
        self.acknowledgement_timeout = max(
            0.1,
            float(acknowledgement_timeout),
        )
        self.reset()

    @property
    def active(self):
        return self.state in ACTIVE_STATES

    def reset(self):
        if getattr(self, "active", False):
            return False
        self.state = STATE_IDLE
        self.reason = ""
        self.started_at = 0.0
        self.settle_started_at = 0.0
        self.arm_sent_at = 0.0
        self.claim_frame_seq = -1
        self.settle_frame_seq = -1
        return True

    @staticmethod
    def normalized_pending(snapshot):
        pending = str(snapshot.pending or "").strip().upper()
        return "" if pending in ("", "-") else pending

    def motion_is_safe(self, snapshot):
        motion_state = str(snapshot.motion_state).strip().lower()
        stable_pose = (
            motion_state in ("standing", "hold")
            and snapshot.motion_arm_neutral_ready
        )
        return (
            snapshot.timestamp_is_fresh(
                snapshot.motion_status_time,
                self.motion_timeout,
            )
            and snapshot.motion_controller_connected
            and stable_pose
            and not snapshot.motion_moving
            and not snapshot.motion_step_in_place
            and snapshot.bridge_motion_safe
        )

    def router_is_available(self, snapshot):
        return (
            snapshot.timestamp_is_fresh(
                snapshot.router_status_time,
                self.router_timeout,
            )
            and str(snapshot.router_owner).strip().upper() in ("HOLD", "MOTION")
        )

    def serial_is_fresh(self, snapshot):
        return snapshot.timestamp_is_fresh(
            snapshot.serial_status_time,
            self.serial_timeout,
        )

    def start_blockers(self, snapshot, allowed_pending=()):
        blockers = []
        allowed_pending = {
            str(command).strip().upper()
            for command in allowed_pending
        }
        if not self.serial_is_fresh(snapshot):
            blockers.append("serial status is missing or stale")
        if not snapshot.hardware_enabled:
            blockers.append("live hardware mode is disabled")
        if snapshot.dry_run:
            blockers.append("the serial bridge is in dry-run")
        if not snapshot.calibration_valid:
            blockers.append("servo calibration is invalid")
        if not snapshot.connected:
            blockers.append("the Arduino is not connected")
        if not snapshot.ready:
            blockers.append("the Arduino firmware is not ready")
        if snapshot.armed:
            blockers.append("the Arduino is already armed")
        if (
            self.normalized_pending(snapshot)
            and self.normalized_pending(snapshot) not in allowed_pending
        ):
            blockers.append("another Arduino command is pending")
        if not self.motion_is_safe(snapshot):
            blockers.append(
                "the controller has not certified the stopped calibrated WALK_POSE"
            )
        if not self.router_is_available(snapshot):
            blockers.append(
                "the command router is unavailable or in an incompatible state"
            )
        return tuple(blockers)

    def can_start(self, snapshot):
        return not self.start_blockers(snapshot)

    def start_blocker_details(self, snapshot, allowed_pending=()):
        """Return operator-facing detail without changing any start gate.

        ``start_blockers`` deliberately groups the controller and router
        interlocks into stable state-machine reasons.  The GUI needs to show
        which member of those grouped interlocks is currently false, so this
        method expands only those two reasons while retaining every blocker
        produced by ``start_blockers``.
        """
        blockers = list(
            self.start_blockers(
                snapshot,
                allowed_pending=allowed_pending,
            )
        )
        motion_summary = (
            "the controller has not certified the stopped calibrated WALK_POSE"
        )
        router_summary = (
            "the command router is unavailable or in an incompatible state"
        )

        if motion_summary in blockers:
            index = blockers.index(motion_summary)
            details = []
            if not snapshot.timestamp_is_fresh(
                snapshot.motion_status_time,
                self.motion_timeout,
            ):
                details.append("motion-controller status is missing or stale")
            if not snapshot.motion_controller_connected:
                details.append("the motion controller has no connected output route")
            motion_state = str(snapshot.motion_state).strip().lower() or "unknown"
            if motion_state not in ("standing", "hold"):
                details.append(
                    "controller state is %s; STANDING or HOLD is required"
                    % motion_state.upper()
                )
            if not snapshot.motion_arm_neutral_ready:
                details.append(
                    "the stopped pose is not certified as the calibrated WALK_POSE"
                )
            if snapshot.motion_moving:
                details.append("the motion controller still reports active movement")
            if snapshot.motion_step_in_place:
                details.append("step-in-place is still active")
            if not snapshot.bridge_motion_safe:
                details.append("the serial bridge has not certified motion_safe")
            blockers[index:index + 1] = details or [motion_summary]

        if router_summary in blockers:
            index = blockers.index(router_summary)
            details = []
            if not snapshot.timestamp_is_fresh(
                snapshot.router_status_time,
                self.router_timeout,
            ):
                details.append("command-router status is missing or stale")
            router_owner = str(snapshot.router_owner).strip().upper() or "UNKNOWN"
            if router_owner not in ("HOLD", "MOTION"):
                details.append(
                    "command-router owner is %s; HOLD or MOTION is required"
                    % router_owner
                )
            blockers[index:index + 1] = details or [router_summary]

        return tuple(blockers)

    def owner_is_confirmed(self, snapshot):
        return (
            str(snapshot.router_owner).strip().upper() == "MOTION"
            and snapshot.timestamp_is_fresh(
                snapshot.router_status_time,
                self.router_timeout,
            )
            and str(snapshot.bridge_owner).strip().upper() == "MOTION"
            and snapshot.bridge_owner_fresh
            and snapshot.bridge_owner_allowed
            and self.serial_is_fresh(snapshot)
        )

    def post_time_statuses_are_confirmed(self, snapshot, threshold):
        return (
            snapshot.motion_status_time > threshold
            and snapshot.router_status_time > threshold
            and snapshot.serial_status_time > threshold
        )

    def full_interlock_blockers(
        self,
        snapshot,
        allow_pending_arm=False,
        allow_pending_status=False,
        frame_after_seq=None,
    ):
        allowed_pending = []
        if allow_pending_arm:
            allowed_pending.append("ARM")
        if allow_pending_status:
            allowed_pending.append("STATUS")
        blockers = list(
            self.start_blockers(
                snapshot,
                allowed_pending=allowed_pending,
            )
        )
        if allow_pending_arm:
            blockers = [
                blocker
                for blocker in blockers
                if blocker != "the Arduino is already armed"
            ]
        if not self.owner_is_confirmed(snapshot):
            blockers.append("fresh MOTION ownership is not confirmed")
        if not snapshot.router_pose_valid:
            blockers.append("the router does not have a valid 12-joint pose")
        if not snapshot.bridge_frame_ready:
            blockers.append(
                "the bridge does not have a recent stable MOTION-owned frame"
            )
        try:
            frame_seq = int(snapshot.bridge_frame_seq)
        except (TypeError, ValueError):
            frame_seq = -1
        if frame_seq < 0:
            blockers.append("the bridge has not reported a joint-frame sequence")
        elif frame_after_seq is not None and frame_seq <= int(frame_after_seq):
            blockers.append("a new joint frame has not arrived after STOP")
        return tuple(blockers)

    def start(self, snapshot):
        if self.active:
            return ()
        blockers = self.start_blockers(snapshot)
        if blockers:
            self.state = STATE_FAILED
            self.reason = blockers[0]
            return ()
        self.state = STATE_CLAIMING
        self.reason = "Stopping motion and requesting MOTION ownership."
        self.started_at = float(snapshot.now)
        self.settle_started_at = 0.0
        self.arm_sent_at = 0.0
        try:
            self.claim_frame_seq = int(snapshot.bridge_frame_seq)
        except (TypeError, ValueError):
            self.claim_frame_seq = -1
        self.settle_frame_seq = -1
        return (
            EFFECT_ZERO_STOP,
            EFFECT_OWNER_MOTION,
            EFFECT_SERIAL_STATUS,
        )

    def fail(self, reason):
        if not self.active:
            return ()
        self.state = STATE_FAILED
        self.reason = str(reason)
        return SAFE_HOLD_EFFECTS

    def cancel(self, reason="Arming cancelled by the operator."):
        if not self.active:
            return ()
        self.state = STATE_CANCELLED
        self.reason = str(reason)
        return SAFE_HOLD_EFFECTS

    def update(self, snapshot):
        if not self.active:
            return ()

        if self.state in (STATE_CLAIMING, STATE_SETTLING):
            if float(snapshot.now) - self.started_at > self.preparation_timeout:
                return self.fail("Arming timed out before all interlocks confirmed.")
        elif (
            self.state == STATE_WAITING_ACK
            and float(snapshot.now) - self.arm_sent_at
            > self.acknowledgement_timeout
        ):
            return self.fail("Arduino ARM acknowledgement timed out.")

        if self.state == STATE_CLAIMING:
            blockers = self.start_blockers(
                snapshot,
                allowed_pending=("STATUS",),
            )
            if blockers:
                return self.fail(blockers[0])
            full_blockers = self.full_interlock_blockers(
                snapshot,
                allow_pending_status=True,
                frame_after_seq=self.claim_frame_seq,
            )
            if (
                not full_blockers
                and self.post_time_statuses_are_confirmed(
                    snapshot,
                    self.started_at,
                )
            ):
                self.state = STATE_SETTLING
                self.reason = "Ownership confirmed; verifying a stopped fresh frame."
                self.settle_started_at = float(snapshot.now)
                self.settle_frame_seq = int(snapshot.bridge_frame_seq)
                return (
                    EFFECT_ZERO_STOP,
                    EFFECT_SERIAL_STATUS,
                )
            return ()

        if self.state == STATE_SETTLING:
            blockers = self.full_interlock_blockers(
                snapshot,
                allow_pending_status=True,
                frame_after_seq=self.settle_frame_seq,
            )
            fatal_blockers = [
                blocker
                for blocker in blockers
                if blocker not in TRANSIENT_FRAME_BLOCKERS
            ]
            if fatal_blockers:
                return self.fail(fatal_blockers[0])
            if blockers:
                return ()
            if (
                float(snapshot.now) - self.settle_started_at
                < self.settle_duration
                or not self.post_time_statuses_are_confirmed(
                    snapshot,
                    self.settle_started_at,
                )
            ):
                return ()
            self.state = STATE_WAITING_ACK
            self.reason = "ARM requested; waiting for trusted firmware confirmation."
            self.arm_sent_at = float(snapshot.now)
            return (EFFECT_SERIAL_ARM,)

        blockers = self.full_interlock_blockers(
            snapshot,
            allow_pending_arm=True,
            allow_pending_status=True,
            frame_after_seq=self.settle_frame_seq,
        )
        if blockers:
            return self.fail(blockers[0])
        if snapshot.armed and snapshot.streaming:
            self.state = STATE_ARMED
            self.reason = "Arduino ARM confirmed; live servo streaming is enabled."
            return ()
        if snapshot.armed and not snapshot.streaming:
            return self.fail("Arduino armed without an allowed live stream.")
        return ()
