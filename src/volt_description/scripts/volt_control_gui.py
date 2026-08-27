#!/usr/bin/env python3

import fcntl
import json
import math
import os
import signal
import sys
import time
import uuid

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import rclpy
from geometry_msgs.msg import Twist
from PyQt5.QtCore import QEvent, QPointF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QColorDialog,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QInputDialog,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String, UInt8, UInt32

from volt_gait_controller import GAITS
from volt_kinematics import JOINT_NAMES, LEG_ORDER
from volt_real_profiles import (
    NUMERIC_BOUNDS,
    RealProfileError,
    load_profiles,
    save_user_profile,
    validate_tuning,
)
from volt_arm_workflow import (
    ArmSnapshot,
    EFFECT_OWNER_HOLD,
    EFFECT_OWNER_MOTION,
    EFFECT_SERIAL_ARM,
    EFFECT_SERIAL_HOLD,
    EFFECT_SERIAL_STATUS,
    EFFECT_ZERO_STOP,
    GuidedArmWorkflow,
    STATE_ARMED,
)
from volt_serial_protocol import (
    CRITICAL_STACK_TOPICS,
    duplicate_stack_topics,
    motion_status_allows_arm,
)
from volt_face import (
    FaceAutomation,
    FaceConfigError,
    SUPPORTED_EFFECTS,
    default_face_settings,
    load_face_catalog,
    load_face_settings,
    save_face_settings,
    settings_for_preset,
    validate_face_settings,
)

try:
    import pygame
except ImportError:
    pygame = None


GAIT_LIMITS = {
    name: (config["max_x"], config["max_y"], config["max_yaw"])
    for name, config in GAITS.items()
}

# Ordered slowest to fastest: STOP walks DOWN this list rather than halting a
# fast gait mid-cycle.
GAIT_SEQUENCE = (
    "amble",
    "trot",
    "run",
)
GAIT_DISPLAY_NAMES = {
    "amble": "AMBLE",
    "trot": "TROT",
    "run": "RUN",
}
DEFAULT_GAIT = "amble"
DEFAULT_SPEED_PERCENT = 20
MOTION_STATUS_TIMEOUT = 3.0
ROUTER_STATUS_TIMEOUT = 1.0
SERIAL_STATUS_TIMEOUT = 1.0
NEUTRAL_RELEASE_TIME = 0.25
STEP_KEEPALIVE_PERIOD = 0.25
EMOTE_REQUEST_ACK_TIMEOUT = 2.0
EMOTE_ERROR_DISPLAY_TIME = 5.0
PUSHUP_TRAVEL_MIN_MM = 10.0
PUSHUP_TRAVEL_MAX_MM = 60.0
PUSHUP_TRAVEL_DEFAULT_MM = 20.0
PUSHUP_TRAVEL_BASE_MM = 20.0

DISPLAYED_CARTESIAN_EMOTES = (
    ("PUSH-UPS", "push_ups"),
    ("BODY ROLL", "body_roll"),
    ("NOD / YES", "nod"),
    ("WAVE LEFT", "wave_left"),
    ("WAVE RIGHT", "wave_right"),
    ("HEART ❤️", "heart"),
    ("BOW", "bow"),
    ("STRETCH", "stretch"),
    ("HAPPY DANCE", "happy_dance"),
    ("SHAKE / NO", "shake_no"),
)
EMOTE_POSE_ACTIONS = (
    ("SIT", "sit"),
    ("STAND UP", "stand"),
)


def gui_emote_request_options(
    name,
    repetitions,
    speed,
    amplitude,
    depth,
    pushup_travel_mm,
):
    """Build GUI emote options, keeping push-up travel independent."""
    requested_depth = float(depth)
    if str(name).strip().lower() == "push_ups":
        travel_mm = max(
            PUSHUP_TRAVEL_MIN_MM,
            min(PUSHUP_TRAVEL_MAX_MM, float(pushup_travel_mm)),
        )
        requested_depth = travel_mm / PUSHUP_TRAVEL_BASE_MM
    return {
        "repetitions": int(round(float(repetitions))),
        "speed": float(speed),
        "amplitude": float(amplitude),
        "depth": requested_depth,
    }


def emote_start_blocker(
    name,
    advertised,
    command_owner,
    status_fresh,
    controller_connected,
    motion_state,
    physical_busy=False,
    emote_busy=False,
    controls_locked=False,
    duplicate_stack=False,
    pose_action=False,
):
    """Return the operator-facing reason an emote button must be disabled."""
    name = str(name).strip().lower()
    advertised = {
        str(value).strip().lower() for value in (advertised or ())
    }
    if duplicate_stack:
        return "Unavailable while duplicate VOLT stacks are active."
    if controls_locked:
        return "Unavailable while guided ARM is verifying the robot."
    if not pose_action and name not in advertised:
        return "The motion controller did not advertise this emote."
    if str(command_owner).strip().upper() != "MOTION":
        return "Press ENABLE MOTION to grant ROS command ownership."
    if not status_fresh:
        return "Waiting for a fresh motion-controller status."
    if not controller_connected:
        return "The motion controller is not connected to the joint controller."
    if physical_busy:
        return "Stop the active hardware diagnostic first."
    if emote_busy:
        return "Stop or wait for the current emote first."
    state = str(motion_state).strip().lower()
    if pose_action:
        if state in ("standing_up", "sitting_down"):
            return "Wait for the current pose transition to finish."
        target_state = "standing" if name == "stand" else "sitting"
        if state == target_state:
            return "The robot is already %s." % target_state.upper()
    elif state != "standing":
        return "Cartesian emotes require a fresh controller STANDING state."
    return ""


def merge_reported_real_profiles(local_profiles, reported_profiles, protected=()):
    """Merge controller profiles without replacing user-saved overlays."""
    merged = {
        str(name).strip().upper(): dict(values)
        for name, values in (local_profiles or {}).items()
    }
    protected_names = {
        str(name).strip().upper() for name in protected
    }
    if not isinstance(reported_profiles, dict):
        return merged
    for raw_name, raw_values in reported_profiles.items():
        name = str(raw_name).strip().upper()
        if not name or name in protected_names:
            continue
        try:
            merged[name] = validate_tuning(
                raw_values,
                allow_simulation=True,
            )
        except (RealProfileError, TypeError, ValueError):
            continue
    return merged


BALANCE_SENSITIVE_EMOTES = {
    "wave_left",
    "wave_right",
    "heart",
    "happy_dance",
}

# Default mapping for common Fantech / Xbox-style controllers.
BUTTON_ACTIONS = {
    0: "stand",       # A / Cross
    1: "sit",         # B / Circle
    2: "stop",        # X / Square
    3: "step",        # Y / Triangle
    4: "prev_gait",   # LB / L1
    5: "next_gait",   # RB / R1
    6: "reset_pose",  # Back / Select
    7: "drive_mode",  # Start / Options
    8: "stop",        # Left stick press
}

AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_RIGHT_X = 2
GAMEPAD_DEADZONE = 0.10
GAMEPAD_DEADZONE_RELEASE = 0.07


def parse_key_value_status(payload):
    """Parse the bridge/router key=value status while tolerating future JSON."""
    try:
        parsed = json.loads(payload)
    except (TypeError, ValueError):
        parsed = None
    if isinstance(parsed, dict):
        return {
            str(key): str(int(value)) if isinstance(value, bool) else str(value)
            for key, value in parsed.items()
        }
    fields = {}
    for part in str(payload).split():
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        fields[key] = value
    return fields


def arm_readiness_view(workflow, snapshot, duplicate_stack_active=False):
    """Build the persistent ARM-gate text from the authoritative workflow."""
    if duplicate_stack_active:
        return {
            "title": "ARM LOCKED — DUPLICATE VOLT STACK",
            "detail": (
                "• More than one critical VOLT status authority is active.\n"
                "• Stop the extra stack before attempting physical output."
            ),
            "color": "#fca5a5",
            "blockers": ("duplicate VOLT stack detected",),
        }

    if workflow.active:
        reason = str(workflow.reason or "Waiting for interlock status.")
        return {
            "title": "GUIDED ARM ACTIVE — OUTPUT STILL INHIBITED",
            "detail": "Current step: %s\nCANCEL ARM / HOLD remains available." % reason,
            "color": "#fbbf24",
            "blockers": (),
        }

    if snapshot.armed or workflow.state == STATE_ARMED:
        serial_fresh = workflow.serial_is_fresh(snapshot)
        if snapshot.armed and snapshot.streaming and serial_fresh:
            title = "SYSTEM ARMED — LIVE SERVO OUTPUT"
            detail = (
                "Firmware is armed and frames are streaming. Use HOLD SERVOS "
                "or DISARM ARDUINO before approaching the robot."
            )
            color = "#fb7185"
        elif not serial_fresh:
            title = "ARM STATE UNKNOWN — SERIAL STATUS STALE"
            detail = (
                "The last report was armed. Treat the robot as live; use HOLD "
                "or DISARM and restore bridge status."
            )
            color = "#fca5a5"
        else:
            title = "ARDUINO ARMED — STREAMING INHIBITED"
            detail = (
                "Firmware reports armed but the bridge is not streaming. Use "
                "HOLD or DISARM before troubleshooting."
            )
            color = "#fbbf24"
        return {
            "title": title,
            "detail": detail,
            "color": color,
            "blockers": (),
        }

    blockers = workflow.start_blocker_details(snapshot)
    if blockers:
        count = len(blockers)
        noun = "BLOCKER" if count == 1 else "BLOCKERS"
        return {
            "title": "ARM LOCKED — %d %s" % (count, noun),
            "detail": "Resolve every condition:\n%s" % "\n".join(
                "• %s" % blocker for blocker in blockers
            ),
            "color": "#fbbf24",
            "blockers": blockers,
        }

    return {
        "title": "ARM READY — ALL PRE-FLIGHT GATES PASSED",
        "detail": (
            "The button is enabled. Guided ARM will re-check every gate, STOP "
            "motion, confirm fresh ownership and frames, then request ARM once."
        ),
        "color": "#86efac",
        "blockers": (),
    }


def face_host_sync_view(fields):
    """Summarize the firmware loading/host-sync handshake for the Control tab."""
    fields = fields or {}

    def flag(name, fallback=None):
        if name not in fields:
            return fallback
        return fields.get(name) == "1"

    connected = flag("connected", False)
    ready = flag("ready", False)
    if not connected:
        return "OFFLINE — no Arduino link", "#94a3b8"
    if not ready:
        return "LOADING — firmware handshake", "#fbbf24"

    led_error = fields.get("led_error", "")
    if led_error not in ("", "-"):
        return "ERROR — %s" % led_error.replace("_", " "), "#fca5a5"

    host_sync_state = str(fields.get("host_sync_state", "")).strip().lower()
    host_sync_error = fields.get("host_sync_error", "")
    if host_sync_state == "error" or host_sync_error not in ("", "-"):
        detail = (
            host_sync_error.replace("_", " ")
            if host_sync_error not in ("", "-")
            else "HOST SYNC failed"
        )
        return "LOADING BLOCKED — %s" % detail, "#fca5a5"
    state_text = {
        "waiting_ping": "LOADING — waiting for host PING",
        "applying_snapshot": "LOADING — applying GUI LED snapshot",
        "verifying_snapshot": "LOADING — verifying LED snapshot",
        "finalizing": "LOADING — waiting for HOST SYNC acknowledgement",
        "loading": "LOADING — waiting for host synchronization",
    }
    if host_sync_state in state_text:
        return state_text[host_sync_state], "#fbbf24"
    if host_sync_state == "synced":
        return "READY — HOST SYNCED", "#86efac"
    if host_sync_state == "legacy":
        if flag("face_supported", False) and flag("face_synced", False):
            return "READY — LED SETTINGS SYNCED (legacy firmware)", "#86efac"
        if not flag("face_supported", False):
            return "UNAVAILABLE — legacy firmware has no Face LEDs", "#94a3b8"

    host_sync_required = flag("host_sync_required", False)
    host_synced = flag("host_synced", flag("host_sync", False))
    face_synced = flag("face_synced", False)
    if host_sync_required and not host_synced:
        if not flag("host_ping", False):
            return "LOADING — waiting for host PING", "#fbbf24"
        if not flag("host_snapshot", False):
            return "LOADING — waiting for GUI LED snapshot", "#fbbf24"
        if flag("host_sync_pending", False):
            return "LOADING — waiting for HOST SYNC acknowledgement", "#fbbf24"
        if not face_synced:
            return "LOADING — applying and verifying LED snapshot", "#7dd3fc"
        return "LOADING — completing HOST SYNC", "#7dd3fc"

    if flag("face_loading", False):
        return "LOADING — waiting for host synchronization", "#fbbf24"
    if face_synced and (host_synced or not host_sync_required):
        suffix = "HOST SYNCED" if host_sync_required else "LED SETTINGS SYNCED"
        return "READY — %s" % suffix, "#86efac"
    if flag("face_supported", False):
        return "SYNCING — verifying LED settings", "#7dd3fc"
    return "UNAVAILABLE — firmware does not report Face LEDs", "#94a3b8"


def arduino_connection_view(fields):
    """Return an explicit physical-link state independent of ARM readiness."""
    fields = fields or {}
    connected = fields.get("connected") == "1"
    ready = fields.get("ready") == "1"
    dry_run = fields.get("dry_run") == "1"
    hardware_enabled = fields.get("hardware_enabled") == "1"
    if connected and ready:
        return "CONNECTED — READY", "#86efac"
    if connected:
        return "CONNECTED — INITIALIZING FIRMWARE", "#fbbf24"
    if dry_run and not hardware_enabled:
        return "DISCONNECTED — hardware disabled / dry-run", "#94a3b8"
    if dry_run:
        return "DISCONNECTED — dry-run (expected)", "#94a3b8"
    if not hardware_enabled:
        return "DISCONNECTED — hardware mode disabled", "#94a3b8"
    return "DISCONNECTED — check USB / serial port", "#fca5a5"


class Joystick(QWidget):
    changed = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(250, 250)
        self.setFocusPolicy(Qt.StrongFocus)
        self.vector = QPointF(0.0, 0.0)
        self.dragging = False

    def set_vector(self, forward, horizontal):
        magnitude = math.hypot(forward, horizontal)
        if magnitude > 1.0:
            forward /= magnitude
            horizontal /= magnitude
        self.vector = QPointF(horizontal, -forward)
        self.changed.emit(forward, horizontal)
        self.update()

    def update_from_mouse(self, position):
        center = QPointF(self.width() / 2.0, self.height() / 2.0)
        radius = max(1.0, min(self.width(), self.height()) * 0.36)
        dx = (position.x() - center.x()) / radius
        dy = (position.y() - center.y()) / radius
        magnitude = math.hypot(dx, dy)
        if magnitude > 1.0:
            dx /= magnitude
            dy /= magnitude
        self.vector = QPointF(dx, dy)
        self.changed.emit(-dy, dx)
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = True
            self.update_from_mouse(event.localPos())

    def mouseMoveEvent(self, event):
        if self.dragging:
            self.update_from_mouse(event.localPos())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = False
            self.set_vector(0.0, 0.0)

    def keyPressEvent(self, event):
        keys = {
            Qt.Key_W: (1.0, 0.0),
            Qt.Key_S: (-1.0, 0.0),
            Qt.Key_A: (0.0, -1.0),
            Qt.Key_D: (0.0, 1.0),
        }
        if event.key() in keys:
            self.set_vector(*keys[event.key()])
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat() and event.key() in (
            Qt.Key_W,
            Qt.Key_A,
            Qt.Key_S,
            Qt.Key_D,
        ):
            self.set_vector(0.0, 0.0)

    def focusOutEvent(self, event):
        """Never retain a keyboard/mouse command after GUI focus is lost."""
        self.dragging = False
        self.set_vector(0.0, 0.0)
        super().focusOutEvent(event)

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        center = QPointF(self.width() / 2.0, self.height() / 2.0)
        radius = min(self.width(), self.height()) * 0.36

        painter.setPen(QPen(QColor("#3a4658"), 2))
        painter.setBrush(QColor("#17202d"))
        painter.drawEllipse(center, radius, radius)

        painter.setPen(QPen(QColor("#344054"), 1))
        painter.drawLine(
            QPointF(center.x() - radius, center.y()),
            QPointF(center.x() + radius, center.y()),
        )
        painter.drawLine(
            QPointF(center.x(), center.y() - radius),
            QPointF(center.x(), center.y() + radius),
        )

        knob = QPointF(
            center.x() + self.vector.x() * radius,
            center.y() + self.vector.y() * radius,
        )
        painter.setPen(QPen(QColor("#7dd3fc"), 3))
        painter.setBrush(QColor("#0ea5e9"))
        painter.drawEllipse(knob, 23, 23)


class VoltGuiNode(Node):
    def __init__(
        self,
        status_callback,
        router_status_callback,
        serial_status_callback,
    ):
        super().__init__("volt_control_gui")
        self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.action_publisher = self.create_publisher(String, "/volt/action", 10)
        self.gait_publisher = self.create_publisher(String, "/volt/gait", 10)
        self.owner_publisher = self.create_publisher(String, "/volt/command_owner", 10)
        self.serial_command_publisher = self.create_publisher(
            String,
            "/volt/serial_command",
            10,
        )
        self.real_tuning_publisher = self.create_publisher(
            String,
            "/volt/real_robot_tuning",
            10,
        )
        self.emote_publisher = self.create_publisher(
            String,
            "/volt/emote",
            10,
        )
        self.physical_test_publisher = self.create_publisher(
            String,
            "/volt/physical_test",
            10,
        )
        self.pose_publisher = self.create_publisher(
            Twist,
            "/volt/body_pose",
            10,
        )
        self.face_expression_publisher = self.create_publisher(
            String,
            "/volt/face/expression",
            10,
        )
        self.face_color_publisher = self.create_publisher(
            ColorRGBA,
            "/volt/face/color",
            10,
        )
        self.face_alternate_color_publisher = self.create_publisher(
            ColorRGBA,
            "/volt/face/alternate_color",
            10,
        )
        self.face_brightness_publisher = self.create_publisher(
            UInt8,
            "/volt/face/brightness",
            10,
        )
        self.face_effect_publisher = self.create_publisher(
            String,
            "/volt/face/effect",
            10,
        )
        self.face_speed_publisher = self.create_publisher(
            UInt32,
            "/volt/face/speed",
            10,
        )
        self.create_subscription(String, "/volt/status", status_callback, 10)
        self.create_subscription(
            String,
            "/volt/command_router_status",
            router_status_callback,
            10,
        )
        self.create_subscription(
            String,
            "/volt/serial_status",
            serial_status_callback,
            10,
        )

    def publish_text(self, publisher, text):
        if not rclpy.ok():
            return False
        message = String()
        message.data = text
        try:
            publisher.publish(message)
            return True
        except Exception:
            return False

    def critical_publisher_counts(self):
        return {
            topic: len(self.get_publishers_info_by_topic(topic))
            for topic in CRITICAL_STACK_TOPICS
        }

    def claim_motion_owner(self):
        return self.publish_text(self.owner_publisher, "MOTION")

    def set_command_owner(self, owner):
        owner = str(owner).strip().upper()
        if owner not in ("MOTION", "HOLD", "DISABLED"):
            return False
        return self.publish_text(self.owner_publisher, owner)

    def send_serial_command(self, command):
        allowed = ("ARM", "HOLD", "DISARM", "DISABLE", "STATUS", "PING")
        command = command.strip().upper()
        if command not in allowed:
            return False
        return self.publish_text(self.serial_command_publisher, command)

    def publish_json(self, publisher, payload):
        try:
            encoded = json.dumps(payload, sort_keys=True, allow_nan=False)
        except (TypeError, ValueError):
            return False
        return self.publish_text(publisher, encoded)

    def publish_face_expression(self, expression):
        return self.publish_text(
            self.face_expression_publisher,
            str(expression).strip().lower(),
        )

    def publish_face_color(self, color):
        if not rclpy.ok():
            return False
        try:
            red, green, blue = (
                max(0, min(255, int(component))) for component in color
            )
            message = ColorRGBA()
            message.r = red / 255.0
            message.g = green / 255.0
            message.b = blue / 255.0
            message.a = 1.0
            self.face_color_publisher.publish(message)
            return True
        except Exception:
            return False

    def publish_face_alternate_color(self, color):
        if not rclpy.ok():
            return False
        try:
            red, green, blue = (
                max(0, min(255, int(component))) for component in color
            )
            message = ColorRGBA()
            message.r = red / 255.0
            message.g = green / 255.0
            message.b = blue / 255.0
            message.a = 1.0
            self.face_alternate_color_publisher.publish(message)
            return True
        except Exception:
            return False

    def publish_face_brightness(self, brightness):
        if not rclpy.ok():
            return False
        try:
            message = UInt8()
            message.data = max(0, min(255, int(brightness)))
            self.face_brightness_publisher.publish(message)
            return True
        except Exception:
            return False

    def publish_face_speed(self, speed_ms):
        if not rclpy.ok():
            return False
        try:
            message = UInt32()
            message.data = max(10, min(60000, int(speed_ms)))
            self.face_speed_publisher.publish(message)
            return True
        except Exception:
            return False

    def publish_face_effect(self, effect):
        return self.publish_text(
            self.face_effect_publisher,
            str(effect).strip().lower(),
        )

    def publish_face_settings(self, settings):
        """Publish one bounded snapshot; the bridge deduplicates/resyncs it."""
        if not settings.enabled:
            return self.publish_face_effect("off")
        # FACE shutdown owns its smooth fade in firmware.  Following it with
        # COLOR/BRIGHTNESS/EFFECT would replace that state with immediate off.
        if str(settings.expression).strip().lower() == "shutdown":
            return self.publish_face_expression("shutdown")
        results = (
            self.publish_face_expression(settings.expression),
            self.publish_face_color(settings.color),
            self.publish_face_alternate_color(settings.alternate_color),
            self.publish_face_brightness(settings.brightness),
            self.publish_face_speed(settings.speed_ms),
            self.publish_face_effect(settings.effect),
        )
        return all(results)


class VoltControlWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VOLT Motion Control")
        self.resize(1280, 800)
        self.setMinimumSize(1120, 560)

        self.ros_node = VoltGuiNode(
            self.status_callback,
            self.router_status_callback,
            self.serial_status_callback,
        )
        self.shutting_down = False
        self.duplicate_stack_active = False
        self.duplicate_stack_topics = ()
        self.forward = 0.0
        self.horizontal = 0.0
        # Start on the stable crawl with zero controls. Command ownership remains
        # unclaimed until the operator explicitly presses ENABLE MOTION.
        self.current_gait = DEFAULT_GAIT
        self.gait_limits = dict(GAIT_LIMITS)
        try:
            shipped_profiles = load_profiles(include_user=False)
            self.real_profiles = load_profiles()
            self.user_real_profile_names = {
                name
                for name, values in self.real_profiles.items()
                if name not in shipped_profiles
                or values != shipped_profiles[name]
            }
        except (OSError, RealProfileError, ValueError):
            self.real_profiles = {}
            self.user_real_profile_names = set()
        self.real_tuning_dirty = False
        self.real_tuning_initialized = False
        self.body_profile_hydrated = False
        self.pending_real_tuning_request = ""
        self.active_real_profile = ""
        self.hardware_mode = False
        self.physical_tests_enabled = False
        self.active_physical_request = None
        self.physical_test_was_active = False
        self.active_emote_request = None
        self.emote_was_active = False
        self.available_emotes = set()
        self.emote_catalog_received = False
        self.emote_catalog_error = ""
        self.controller_emote_busy = False
        self.controller_emote_request_id = ""
        self.emote_notice = ""
        self.emote_notice_color = "#94a3b8"
        self.emote_notice_time = 0.0
        self.pending_emote_pose_action = None
        self.command_owner = "UNKNOWN"
        # The router refreshes MOTION ownership from the pose stream, so an
        # idle or stopped controller lets ownership lapse after stale_timeout
        # and the robot drops to HOLD on its own.  Re-assert the operator's
        # standing intent instead.  Gated on an explicit request and cleared
        # by every HOLD/DISABLE path, so a dead GUI still stops publishing and
        # the router still holds -- the liveness guarantee is unchanged.
        self.motion_ownership_requested = False
        self.last_owner_heartbeat = 0.0
        self.last_router_status_time = 0.0
        self.router_pose_valid = False
        self.last_serial_status_time = 0.0
        self.last_serial_status_fields = {}
        self.arm_workflow = GuidedArmWorkflow(
            motion_timeout=MOTION_STATUS_TIMEOUT,
            router_timeout=ROUTER_STATUS_TIMEOUT,
            serial_timeout=SERIAL_STATUS_TIMEOUT,
        )
        self.arm_workflow_notice = ""
        self.motion_neutral_latched = True
        self.motion_neutral_since = 0.0
        self.position_controller_connected = False
        self.open_loop_hardware = False
        self.motion_state = "unknown"
        self.motion_moving = True
        self.motion_step_in_place = False
        self.motion_arm_neutral_ready = False
        self.last_step_keepalive_publish_time = 0.0
        self.motion_controller_connected = False
        self.last_motion_status_time = 0.0
        self.gamepad = None
        self.gamepad_name = ""
        self.gamepad_buttons = {}
        self.gamepad_axis_active = {}
        self.gamepad_available = pygame is not None
        self.gamepad_enabled = True
        self.last_gamepad_scan = 0.0
        self.arm_controls_locked = False
        self.arm_frozen_pose = None
        self.last_face_motion_status = {}
        try:
            self.face_catalog = load_face_catalog()
            self.face_settings = load_face_settings(self.face_catalog)
            self.face_config_error = ""
        except (OSError, FaceConfigError, ValueError) as exc:
            self.face_catalog = None
            self.face_settings = None
            self.face_config_error = str(exc)
        self.face_automation = (
            FaceAutomation(self.face_catalog, self.face_settings)
            if self.face_catalog is not None and self.face_settings is not None
            else None
        )
        self.face_last_requested_settings = self.face_settings
        self.face_connection_active = False
        if self.gamepad_available:
            pygame.init()
            pygame.joystick.init()

        self.build_ui()
        self.stabilize_status_labels()
        self.apply_style()

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.spin_ros)
        self.spin_timer.start(10)

        self.command_timer = QTimer(self)
        self.command_timer.timeout.connect(self.publish_motion)
        self.command_timer.start(50)

        self.pose_timer = QTimer(self)
        self.pose_timer.timeout.connect(self.publish_pose)
        self.pose_timer.start(100)

        self.gamepad_timer = QTimer(self)
        self.gamepad_timer.timeout.connect(self.poll_gamepad)
        self.gamepad_timer.start(30)

        # Grace period before an unfocused window disarms.  Long enough to
        # survive notifications, window switches and a look at the simulator;
        # short enough that a genuinely abandoned console still ends safe.
        self.focus_hold_grace_seconds = 20.0
        self.focus_hold_timer = QTimer(self)
        self.focus_hold_timer.setSingleShot(True)
        self.focus_hold_timer.timeout.connect(self.focus_hold_timeout)
        self.arm_workflow_timer = QTimer(self)
        self.arm_workflow_timer.timeout.connect(self.advance_arm_workflow)
        self.arm_workflow_timer.setInterval(50)

        self.arm_status_timer = QTimer(self)
        self.arm_status_timer.timeout.connect(self.refresh_arm_status_freshness)
        self.arm_status_timer.start(250)

        self.diagnostic_timer = QTimer(self)
        self.diagnostic_timer.timeout.connect(self.publish_diagnostic_keepalive)
        self.diagnostic_timer.start(200)

        self.face_test_timer = QTimer(self)
        self.face_test_timer.setInterval(450)
        self.face_test_timer.timeout.connect(self.advance_face_test)
        self.face_test_steps = []

    def build_ui(self):
        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(16, 10, 16, 12)
        outer.setSpacing(9)

        header = QHBoxLayout()
        header.setSpacing(14)

        title = QLabel("VOLT")
        title.setObjectName("title")
        subtitle = QLabel("Quadruped motion console")
        subtitle.setObjectName("subtitle")
        header.addWidget(title)
        header.addWidget(subtitle)
        header.addStretch(1)
        outer.addLayout(header)

        self.main_tabs = QTabWidget()
        self.main_tabs.setObjectName("mainTabs")
        self.main_tabs.setDocumentMode(True)
        outer.addWidget(self.main_tabs, 1)

        def add_scroll_tab(title, object_name):
            scroll = QScrollArea()
            scroll.setObjectName(object_name)
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QFrame.NoFrame)
            page = QWidget()
            scroll.setWidget(page)
            self.main_tabs.addTab(scroll, title)
            return page

        control_page = add_scroll_tab("CONTROL", "controlScroll")
        self.control_page = control_page
        emotes_face_page = add_scroll_tab("EMOTES + FACE", "expressionsScroll")
        tuning_page = add_scroll_tab("TUNING", "tuningScroll")
        diagnostics_page = add_scroll_tab("DIAGNOSTICS", "diagnosticsScroll")

        workspace = QHBoxLayout(control_page)
        workspace.setContentsMargins(0, 8, 0, 0)
        workspace.setSpacing(14)

        emotes_face_layout = QGridLayout(emotes_face_page)
        emotes_face_layout.setContentsMargins(0, 8, 0, 0)
        emotes_face_layout.setHorizontalSpacing(14)
        emotes_face_layout.setVerticalSpacing(10)
        emotes_face_layout.setColumnStretch(0, 5)
        emotes_face_layout.setColumnStretch(1, 6)

        tuning_workspace = QHBoxLayout(tuning_page)
        tuning_workspace.setContentsMargins(0, 8, 0, 0)
        tuning_workspace.setSpacing(14)
        tuning_left = QVBoxLayout()
        tuning_left.setSpacing(10)
        tuning_right = QVBoxLayout()
        tuning_right.setSpacing(10)
        tuning_workspace.addLayout(tuning_left, 4)
        tuning_workspace.addLayout(tuning_right, 7)

        diagnostics_layout = QGridLayout(diagnostics_page)
        diagnostics_layout.setContentsMargins(0, 8, 0, 0)
        diagnostics_layout.setHorizontalSpacing(14)
        diagnostics_layout.setVerticalSpacing(10)
        diagnostics_layout.setColumnStretch(0, 3)
        diagnostics_layout.setColumnStretch(1, 2)

        left = QVBoxLayout()
        left.setSpacing(10)

        state_group = QGroupBox("Robot State")
        state_layout = QGridLayout(state_group)
        self.state_label = QLabel("WAITING")
        self.state_label.setObjectName("state")
        self.status_detail = QLabel("Waiting for controller status")
        self.status_detail.setWordWrap(True)
        state_layout.addWidget(self.state_label, 0, 0, 1, 3)
        state_layout.addWidget(self.status_detail, 1, 0, 1, 3)

        self.stand_button = QPushButton("STAND")
        self.stand_button.clicked.connect(lambda: self.send_action("stand"))
        self.sit_button = QPushButton("SIT")
        self.sit_button.clicked.connect(lambda: self.send_action("sit"))
        stop_button = QPushButton("STOP")
        stop_button.setObjectName("stop")
        stop_button.clicked.connect(lambda: self.send_action("stop"))
        state_layout.addWidget(self.stand_button, 2, 0)
        state_layout.addWidget(self.sit_button, 2, 1)
        state_layout.addWidget(stop_button, 2, 2)
        left.addWidget(state_group)

        owner_group = QGroupBox("ROS Command Ownership")
        owner_layout = QGridLayout(owner_group)
        self.owner_state = QLabel("ACTIVE OWNER: UNKNOWN")
        self.owner_state.setObjectName("ownerState")
        self.owner_state.setWordWrap(True)
        self.enable_motion_button = QPushButton("ENABLE MOTION")
        self.enable_motion_button.setObjectName("motionEnable")
        self.enable_motion_button.clicked.connect(self.enable_motion)
        hold_motion = QPushButton("HOLD")
        hold_motion.clicked.connect(self.hold_motion)
        disable_motion = QPushButton("DISABLE OUTPUT COMMANDS")
        disable_motion.setObjectName("motionDisable")
        disable_motion.clicked.connect(self.disable_motion)
        owner_layout.addWidget(self.owner_state, 0, 0, 1, 2)
        owner_layout.addWidget(self.enable_motion_button, 1, 0)
        owner_layout.addWidget(hold_motion, 1, 1)
        owner_layout.addWidget(disable_motion, 2, 0, 1, 2)
        left.addWidget(owner_group)

        gait_group = QGroupBox("Gait")
        gait_layout = QGridLayout(gait_group)
        self.gait_buttons = QButtonGroup(self)
        self.gait_buttons.setExclusive(True)
        self.gait_button_by_name = {}
        gait_choices = tuple(
            (gait, GAIT_DISPLAY_NAMES[gait])
            for gait in GAIT_SEQUENCE
        )
        for index, (gait, label) in enumerate(gait_choices):
            button = QPushButton(label)
            button.setCheckable(True)
            button.clicked.connect(
                lambda checked, name=gait: self.select_gait(name)
            )
            self.gait_buttons.addButton(button)
            self.gait_button_by_name[gait] = button
            gait_layout.addWidget(button, index // 3, index % 3)
            if gait == DEFAULT_GAIT:
                button.setChecked(True)

        gait_rows = (len(gait_choices) + 2) // 3
        self.step_button = QPushButton("STEP IN PLACE")
        self.step_button.setCheckable(True)
        self.step_button.clicked.connect(lambda: self.send_action("step"))
        gait_layout.addWidget(self.step_button, gait_rows, 0, 1, 3)
        self.gait_detail = QLabel(
            "Requested: VOLT WALK | Active: waiting | Phase: waiting"
        )
        self.gait_detail.setObjectName("gaitDetail")
        self.gait_detail.setWordWrap(True)
        gait_layout.addWidget(self.gait_detail, gait_rows + 1, 0, 1, 3)
        left.addWidget(gait_group)

        speed_group = QGroupBox("Motion")
        speed_layout = QFormLayout(speed_group)
        self.drive_mode = QComboBox()
        self.drive_mode.addItems(["Normal steering", "Crab / omnidirectional"])
        speed_layout.addRow("Drive mode", self.drive_mode)

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(10, 100)
        self.speed_slider.setValue(DEFAULT_SPEED_PERCENT)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(25)
        speed_layout.addRow("Speed limit", self.speed_slider)

        self.yaw_slider = QSlider(Qt.Horizontal)
        self.yaw_slider.setRange(-100, 100)
        self.yaw_slider.setValue(0)
        self.yaw_slider.setTickPosition(QSlider.TicksBelow)
        self.yaw_slider.setTickInterval(50)
        speed_layout.addRow("Yaw trim", self.yaw_slider)
        left.addWidget(speed_group)

        tuning_left.addStretch(1)

        controller_group = QGroupBox("Controller")
        controller_layout = QGridLayout(controller_group)
        self.controller_state = QLabel("NOT CONNECTED")
        self.controller_state.setObjectName("controllerState")
        self.controller_detail = QLabel(
            "Left stick: move/turn | Right stick X: yaw trim\n"
            "A stand | B sit | X stop | Y step | LB/RB gait"
        )
        self.controller_detail.setWordWrap(True)
        self.controller_enable = QPushButton("GAMEPAD ENABLED")
        self.controller_enable.setCheckable(True)
        self.controller_enable.setChecked(True)
        self.controller_enable.clicked.connect(self.toggle_gamepad)
        controller_layout.addWidget(self.controller_state, 0, 0, 1, 2)
        controller_layout.addWidget(self.controller_detail, 1, 0, 1, 2)
        controller_layout.addWidget(self.controller_enable, 2, 0, 1, 2)
        left.addStretch(1)
        workspace.addLayout(left, 4)

        center = QVBoxLayout()
        center.setSpacing(10)
        joystick_group = QGroupBox("Direction")
        joystick_layout = QVBoxLayout(joystick_group)
        self.joystick = Joystick()
        self.joystick.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.joystick.setMaximumSize(380, 380)
        self.joystick.changed.connect(self.joystick_changed)
        joystick_layout.addWidget(self.joystick, alignment=Qt.AlignCenter)
        hint = QLabel(
            "Drag the pad or use W A S D. Release to stop.\n"
            "Normal: left/right turns. Crab: left/right translates."
        )
        hint.setAlignment(Qt.AlignCenter)
        hint.setObjectName("hint")
        joystick_layout.addWidget(hint)
        joystick_group.setMaximumHeight(460)
        center.addWidget(joystick_group)

        safety = QFrame()
        safety.setObjectName("safety")
        safety_layout = QVBoxLayout(safety)
        safety_title = QLabel("Command safety")
        safety_title.setObjectName("safetyTitle")
        safety_text = QLabel(
            "Velocity stops automatically if GUI messages are lost.\n"
            "Use STOP before switching between tests."
        )
        safety_text.setWordWrap(True)
        safety_layout.addWidget(safety_title)
        safety_layout.addWidget(safety_text)
        center.addWidget(safety)
        center.addWidget(controller_group)
        center.addStretch(1)
        workspace.addLayout(center, 5)

        right = QVBoxLayout()
        right.setSpacing(10)

        hardware_group = QGroupBox("Arduino / Physical Robot")
        hardware_layout = QVBoxLayout(hardware_group)
        hardware_layout.setSpacing(8)

        self.hardware_state = QLabel("WAITING FOR SERIAL BRIDGE")
        self.hardware_state.setObjectName("hardwareState")
        self.hardware_state.setWordWrap(True)
        hardware_layout.addWidget(self.hardware_state)

        hardware_status = QGridLayout()
        hardware_status.setHorizontalSpacing(12)
        hardware_status.setVerticalSpacing(3)
        bridge_caption = QLabel("Serial bridge")
        bridge_caption.setObjectName("statusCaption")
        device_caption = QLabel("Arduino connection")
        device_caption.setObjectName("statusCaption")
        output_caption = QLabel("Servo output")
        output_caption.setObjectName("statusCaption")
        face_sync_caption = QLabel("Face LEDs / host sync")
        face_sync_caption.setObjectName("statusCaption")
        self.hardware_bridge = QLabel("WAITING FOR STATUS")
        self.hardware_device = QLabel("UNKNOWN — waiting for status")
        self.hardware_output = QLabel("Disabled")
        self.hardware_face_sync = QLabel("UNKNOWN — waiting for Arduino")
        self.hardware_face_sync.setWordWrap(True)
        hardware_status.addWidget(bridge_caption, 0, 0)
        hardware_status.addWidget(self.hardware_bridge, 0, 1)
        hardware_status.addWidget(device_caption, 1, 0)
        hardware_status.addWidget(self.hardware_device, 1, 1)
        hardware_status.addWidget(output_caption, 2, 0)
        hardware_status.addWidget(self.hardware_output, 2, 1)
        hardware_status.addWidget(face_sync_caption, 3, 0)
        hardware_status.addWidget(self.hardware_face_sync, 3, 1)
        hardware_status.setColumnStretch(1, 1)
        hardware_layout.addLayout(hardware_status)

        arm_gate = QFrame()
        arm_gate.setObjectName("armGate")
        arm_gate_layout = QVBoxLayout(arm_gate)
        arm_gate_layout.setContentsMargins(9, 7, 9, 7)
        arm_gate_layout.setSpacing(4)
        self.arm_readiness_state = QLabel("ARM LOCKED — WAITING FOR STATUS")
        self.arm_readiness_state.setObjectName("armReadinessState")
        self.arm_readiness_state.setWordWrap(True)
        self.arm_blockers_label = QLabel(
            "• serial status is missing or stale\n"
            "• Arduino connection state is unknown"
        )
        self.arm_blockers_label.setObjectName("armBlockers")
        self.arm_blockers_label.setWordWrap(True)
        self.arm_blockers_label.setSizePolicy(
            QSizePolicy.Preferred,
            QSizePolicy.Minimum,
        )
        self.arm_blockers_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        arm_gate_layout.addWidget(self.arm_readiness_state)
        arm_gate_layout.addWidget(self.arm_blockers_label)
        hardware_layout.addWidget(arm_gate)

        self.hardware_detail = QLabel(
            "Launch the serial bridge with hardware enabled. It reconnects "
            "automatically; arming unlocks only after the Arduino reports ready."
        )
        self.hardware_detail.setObjectName("hardwareDetail")
        self.hardware_detail.setWordWrap(True)
        hardware_layout.addWidget(self.hardware_detail)

        status_button = QPushButton("REQUEST STATUS")
        status_button.setObjectName("hardwareRefresh")
        status_button.setToolTip(
            "Request current firmware status. The bridge handles connection automatically."
        )
        status_button.clicked.connect(lambda: self.send_serial_command("STATUS"))
        hardware_layout.addWidget(status_button)

        safe_actions_caption = QLabel("Safe actions")
        safe_actions_caption.setObjectName("statusCaption")
        hardware_layout.addWidget(safe_actions_caption)

        hardware_actions = QGridLayout()
        hardware_actions.setHorizontalSpacing(8)
        hardware_actions.setVerticalSpacing(8)

        self.hardware_arm_button = QPushButton("ARM SYSTEM SAFELY")
        self.hardware_arm_button.setObjectName("hardwareArm")
        self.hardware_arm_button.setEnabled(False)
        self.hardware_arm_button.setToolTip(
            "Guided STOP, MOTION-ownership, fresh-frame, and Arduino ARM sequence."
        )
        self.hardware_arm_button.clicked.connect(
            self.arm_system_safely
        )

        hold_button = QPushButton("HOLD SERVOS")
        hold_button.setToolTip("Stop streaming motion and hold the current targets.")
        hold_button.clicked.connect(
            lambda: self.send_hardware_safe_command("HOLD")
        )

        disarm_button = QPushButton("DISARM ARDUINO")
        disarm_button.setObjectName("hardwareSafe")
        disarm_button.setToolTip("Stop accepting servo motion commands.")
        disarm_button.clicked.connect(
            lambda: self.send_hardware_safe_command("DISARM")
        )

        disable_button = QPushButton("DISABLE SERVO OUTPUTS")
        disable_button.setObjectName("hardwareDisable")
        disable_button.setToolTip("Disarm and switch off PCA9685 servo pulses.")
        disable_button.clicked.connect(
            lambda: self.send_hardware_safe_command("DISABLE")
        )

        hardware_actions.addWidget(hold_button, 0, 0)
        hardware_actions.addWidget(disarm_button, 0, 1)
        hardware_actions.addWidget(disable_button, 1, 0, 1, 2)
        hardware_actions.setColumnStretch(0, 1)
        hardware_actions.setColumnStretch(1, 1)
        hardware_layout.addLayout(hardware_actions)

        hardware_warning = QLabel(
            "ARM SYSTEM SAFELY sequences both safety layers but never bypasses "
            "their checks. Support the robot, clear the legs, and keep the "
            "power disconnect accessible."
        )
        hardware_warning.setObjectName("hardwareWarning")
        hardware_warning.setWordWrap(True)
        hardware_layout.addWidget(hardware_warning)
        hardware_layout.addWidget(self.hardware_arm_button)
        right.addWidget(hardware_group)

        pose_group = QGroupBox("Body Pose")
        pose_layout = QGridLayout(pose_group)
        pose_layout.setHorizontalSpacing(8)
        pose_layout.setVerticalSpacing(5)
        self.body_height = self.make_spinbox(0.175, 0.220, 0.200, 0.001, 3)
        self.body_x = self.make_spinbox(-0.025, 0.025, 0.0, 0.002, 3)
        self.body_y = self.make_spinbox(-0.020, 0.020, 0.0, 0.002, 3)
        self.roll = self.make_spinbox(-9.0, 9.0, 0.0, 1.0, 1)
        self.pitch = self.make_spinbox(-9.0, 9.0, 0.0, 1.0, 1)
        self.yaw = self.make_spinbox(-10.0, 10.0, 0.0, 1.0, 1)
        pose_fields = (
            ("Height (m)", self.body_height),
            ("Body X (m)", self.body_x),
            ("Body Y (m)", self.body_y),
            ("Roll (deg)", self.roll),
            ("Pitch (deg)", self.pitch),
            ("Yaw (deg)", self.yaw),
        )
        for index, (label_text, control) in enumerate(pose_fields):
            row = (index // 3) * 2
            column = index % 3
            label = QLabel(label_text)
            label.setObjectName("poseCaption")
            control.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            pose_layout.addWidget(label, row, column)
            pose_layout.addWidget(control, row + 1, column)
            pose_layout.setColumnStretch(column, 1)

        self.reset_pose_button = QPushButton("RESET BODY POSE")
        self.reset_pose_button.clicked.connect(self.reset_body_pose)
        pose_layout.addWidget(self.reset_pose_button, 4, 0, 1, 3)
        right.addWidget(pose_group)

        tuning_group = QGroupBox("Real Robot Tuning")
        tuning_layout = QGridLayout(tuning_group)
        tuning_layout.setHorizontalSpacing(8)
        tuning_layout.setVerticalSpacing(5)
        self.real_profile_combo = QComboBox()
        ordered_profiles = [
            name
            for name in (
                "REAL_DIAGNOSTIC",
                "REAL_SAFE",
                "REAL_NORMAL",
                "SIMULATION",
            )
            if name in self.real_profiles
        ]
        ordered_profiles.extend(
            sorted(set(self.real_profiles) - set(ordered_profiles))
        )
        self.real_profile_combo.addItems(ordered_profiles)
        self.real_gait_combo = QComboBox()
        self.real_gait_combo.addItem("Amble / Diagnostic", "amble")
        self.real_gait_combo.addItem("Trot / Load-Safe", "trot")
        tuning_layout.addWidget(QLabel("Profile"), 0, 0)
        tuning_layout.addWidget(self.real_profile_combo, 0, 1)
        tuning_layout.addWidget(QLabel("Profile gait"), 0, 2)
        tuning_layout.addWidget(self.real_gait_combo, 0, 3)

        self.real_cycle = self.make_tuning_spinbox(0.60, 6.00, 2.00, 0.05, 2, " s")
        self.real_stride_mm = self.make_tuning_spinbox(5.0, 75.0, 35.0, 1.0, 1, " mm")
        self.real_lateral_mm = self.make_tuning_spinbox(0.0, 30.0, 10.0, 1.0, 1, " mm")
        self.real_step_height_mm = self.make_tuning_spinbox(0.0, 45.0, 40.0, 1.0, 1, " mm")
        self.real_duty = self.make_tuning_spinbox(0.55, 0.90, 0.80, 0.01, 2, "")
        self.real_joint_velocity = self.make_tuning_spinbox(60.0, 120.0, 100.0, 5.0, 0, " deg/s")
        self.real_joint_acceleration = self.make_tuning_spinbox(60.0, 1200.0, 240.0, 20.0, 0, " deg/s²")
        self.real_smoothing = self.make_tuning_spinbox(0.0, 0.80, 0.15, 0.02, 2, "")
        self.real_touchdown_percent = self.make_tuning_spinbox(8.0, 35.0, 30.0, 1.0, 0, " %")
        self.real_stance_width_mm = self.make_tuning_spinbox(80.0, 130.0, 104.0, 1.0, 1, " mm")
        self.real_body_height_mm = self.make_tuning_spinbox(175.0, 220.0, 200.0, 1.0, 1, " mm")
        self.real_body_x_mm = self.make_tuning_spinbox(-25.0, 25.0, 0.0, 1.0, 1, " mm")
        self.real_body_y_mm = self.make_tuning_spinbox(-20.0, 20.0, 0.0, 1.0, 1, " mm")
        roll_bounds = NUMERIC_BOUNDS["body_roll_deg"]
        pitch_bounds = NUMERIC_BOUNDS["body_pitch_deg"]
        self.real_body_roll = self.make_tuning_spinbox(
            roll_bounds[0], roll_bounds[1], 0.0, 0.5, 1, " deg"
        )
        self.real_body_pitch = self.make_tuning_spinbox(
            pitch_bounds[0], pitch_bounds[1], 0.0, 0.5, 1, " deg"
        )
        self.real_body_yaw = self.make_tuning_spinbox(-10.0, 10.0, 0.0, 0.5, 1, " deg")
        tuning_fields = (
            ("Cycle duration", self.real_cycle),
            ("Forward stride", self.real_stride_mm),
            ("Lateral stride", self.real_lateral_mm),
            ("Swing clearance", self.real_step_height_mm),
            ("Duty factor", self.real_duty),
            ("Max joint velocity", self.real_joint_velocity),
            ("Max joint acceleration", self.real_joint_acceleration),
            ("Smoothing amount", self.real_smoothing),
            ("Touchdown softness", self.real_touchdown_percent),
            ("Stance half-width", self.real_stance_width_mm),
            ("Profile body height", self.real_body_height_mm),
            ("Profile body X", self.real_body_x_mm),
            ("Profile body Y", self.real_body_y_mm),
            ("Profile body roll", self.real_body_roll),
            ("Profile body pitch", self.real_body_pitch),
            ("Profile body yaw", self.real_body_yaw),
        )
        for index, (label_text, control) in enumerate(tuning_fields):
            row = 1 + index // 2
            column = (index % 2) * 2
            tuning_layout.addWidget(QLabel(label_text), row, column)
            tuning_layout.addWidget(control, row, column + 1)
        for column in (1, 3):
            tuning_layout.setColumnStretch(column, 1)

        tuning_buttons = QHBoxLayout()
        self.load_real_profile_button = QPushButton("LOAD PROFILE")
        self.load_real_profile_button.clicked.connect(self.load_selected_real_profile)
        self.reset_real_profile_button = QPushButton("RESET VALUES")
        self.reset_real_profile_button.clicked.connect(self.load_selected_real_profile)
        self.apply_real_profile_button = QPushButton("APPLY")
        self.apply_real_profile_button.clicked.connect(self.apply_real_tuning)
        self.save_real_profile_button = QPushButton("SAVE PROFILE")
        self.save_real_profile_button.clicked.connect(self.save_real_tuning_profile)
        for button in (
            self.load_real_profile_button,
            self.reset_real_profile_button,
            self.apply_real_profile_button,
            self.save_real_profile_button,
        ):
            tuning_buttons.addWidget(button)
        tuning_layout.addLayout(tuning_buttons, 9, 0, 1, 4)
        self.real_tuning_status = QLabel(
            "Load a profile, STOP and settle, then APPLY. Body pose controls above are part of the transaction."
        )
        self.real_tuning_status.setWordWrap(True)
        self.real_tuning_status.setObjectName("gaitDetail")
        tuning_layout.addWidget(self.real_tuning_status, 10, 0, 1, 4)
        tuning_right.addWidget(tuning_group)
        tuning_right.addStretch(1)

        self.real_tuning_value_controls = [
            self.real_gait_combo,
            self.real_cycle,
            self.real_stride_mm,
            self.real_lateral_mm,
            self.real_step_height_mm,
            self.real_duty,
            self.real_joint_velocity,
            self.real_joint_acceleration,
            self.real_smoothing,
            self.real_touchdown_percent,
            self.real_stance_width_mm,
            self.real_body_height_mm,
            self.real_body_x_mm,
            self.real_body_y_mm,
            self.real_body_roll,
            self.real_body_pitch,
            self.real_body_yaw,
        ]
        for control in self.real_tuning_value_controls:
            if isinstance(control, QComboBox):
                control.currentIndexChanged.connect(self.mark_real_tuning_dirty)
            else:
                control.valueChanged.connect(self.mark_real_tuning_dirty)
        self.real_profile_combo.currentTextChanged.connect(
            self.refresh_real_profile_editability
        )
        self.refresh_real_profile_editability()

        emote_group = QGroupBox("Robot Emotes & Poses")
        emote_layout = QGridLayout(emote_group)
        emote_layout.setHorizontalSpacing(8)
        emote_layout.setVerticalSpacing(5)
        self.emote_repetitions = self.make_tuning_spinbox(
            1.0, 5.0, 1.0, 1.0, 0, " ×"
        )
        self.emote_speed = self.make_tuning_spinbox(
            0.5, 2.0, 1.0, 0.1, 1, " ×"
        )
        self.emote_amplitude = self.make_tuning_spinbox(
            0.5, 2.0, 1.0, 0.1, 1, " ×"
        )
        self.emote_depth = self.make_tuning_spinbox(
            0.5, 3.0, 1.0, 0.1, 1, " ×"
        )
        self.pushup_travel_mm = self.make_tuning_spinbox(
            PUSHUP_TRAVEL_MIN_MM,
            PUSHUP_TRAVEL_MAX_MM,
            PUSHUP_TRAVEL_DEFAULT_MM,
            1.0,
            0,
            " mm",
        )
        self.pushup_travel_mm.setToolTip(
            "Vertical body travel used only by PUSH-UPS. The controller "
            "still applies IK and joint safety limits."
        )
        for column, (caption, control) in enumerate(
            (
                ("Repetitions", self.emote_repetitions),
                ("Speed", self.emote_speed),
                ("Amplitude", self.emote_amplitude),
                ("Depth", self.emote_depth),
            )
        ):
            emote_layout.addWidget(QLabel(caption), 0, column)
            emote_layout.addWidget(control, 1, column)

        pushup_travel_label = QLabel("Push-up travel")
        pushup_travel_label.setToolTip(self.pushup_travel_mm.toolTip())
        emote_layout.addWidget(pushup_travel_label, 2, 0)
        emote_layout.addWidget(self.pushup_travel_mm, 2, 1)
        pushup_travel_hint = QLabel(
            "PUSH-UPS only (10–25 mm); Depth remains for other emotes."
        )
        pushup_travel_hint.setObjectName("gaitDetail")
        pushup_travel_hint.setWordWrap(True)
        emote_layout.addWidget(pushup_travel_hint, 2, 2, 1, 2)

        emote_actions = (
            DISPLAYED_CARTESIAN_EMOTES[0],
            (EMOTE_POSE_ACTIONS[0][0], "action:%s" % EMOTE_POSE_ACTIONS[0][1]),
            (EMOTE_POSE_ACTIONS[1][0], "action:%s" % EMOTE_POSE_ACTIONS[1][1]),
        ) + DISPLAYED_CARTESIAN_EMOTES[1:]
        self.emote_start_buttons = []
        self.emote_buttons_by_name = {}
        self.emote_action_buttons_by_name = {}
        for index, (caption, command) in enumerate(emote_actions):
            button = QPushButton(caption)
            if command.startswith("action:"):
                action = command.partition(":")[2]
                button.clicked.connect(
                    lambda _checked=False, name=action: self.request_emote_pose_action(
                        name
                    )
                )
                button.setEnabled(False)
                self.emote_action_buttons_by_name[action] = button
            else:
                button.clicked.connect(
                    lambda _checked=False, name=command: self.start_emote(name)
                )
                button.setEnabled(False)
                self.emote_buttons_by_name[command] = button
            emote_layout.addWidget(button, 3 + index // 4, index % 4)
            self.emote_start_buttons.append(button)

        self.stop_emote_button = QPushButton("STOP EMOTE")
        self.stop_emote_button.setObjectName("stop")
        self.stop_emote_button.clicked.connect(self.stop_emote)
        self.stop_emote_button.setEnabled(False)
        emote_layout.addWidget(self.stop_emote_button, 6, 0, 1, 4)
        self.emote_status = QLabel(
            "Emotes run through Cartesian targets, IK, and the active joint safety limiter."
        )
        self.emote_status.setWordWrap(True)
        self.emote_status.setObjectName("gaitDetail")
        emote_layout.addWidget(self.emote_status, 7, 0, 1, 4)
        emotes_face_layout.addWidget(emote_group, 0, 0, alignment=Qt.AlignTop)

        face_group = QGroupBox("Face LEDs")
        face_layout = QGridLayout(face_group)
        face_layout.setHorizontalSpacing(8)
        face_layout.setVerticalSpacing(6)

        self.face_enable = QCheckBox("Enabled")
        self.face_auto = QCheckBox("Automatic during emotes")
        self.face_auto.setToolTip(
            "Select configured expressions automatically during emotes."
        )
        self.face_lock = QCheckBox("Lock current face")
        self.face_lock.setToolTip(
            "Keep the manual face during ordinary motion and emotes; safety "
            "alerts can still override it."
        )
        face_mode_row = QHBoxLayout()
        face_mode_row.setSpacing(16)
        face_mode_row.addWidget(self.face_enable)
        face_mode_row.addWidget(self.face_auto)
        face_mode_row.addWidget(self.face_lock)
        face_mode_row.addStretch(1)
        face_layout.addLayout(face_mode_row, 0, 0, 1, 4)

        self.face_expression_combo = QComboBox()
        if self.face_catalog is not None:
            self.face_expression_combo.addItems(list(self.face_catalog.presets))
        self.face_effect_combo = QComboBox()
        self.face_effect_combo.addItems(SUPPORTED_EFFECTS)
        face_layout.addWidget(QLabel("Expression"), 1, 0)
        face_layout.addWidget(self.face_expression_combo, 1, 1)
        face_layout.addWidget(QLabel("Effect"), 1, 2)
        face_layout.addWidget(self.face_effect_combo, 1, 3)

        self.face_color_button = QPushButton("CHOOSE COLOR")
        self.face_color_button.clicked.connect(self.choose_face_color)
        self.face_red = QSpinBox()
        self.face_green = QSpinBox()
        self.face_blue = QSpinBox()
        for channel in (self.face_red, self.face_green, self.face_blue):
            channel.setRange(0, 255)
            channel.valueChanged.connect(self.update_face_preview)
        face_layout.addWidget(self.face_color_button, 2, 0)
        for column, (caption, channel) in enumerate(
            (("R", self.face_red), ("G", self.face_green), ("B", self.face_blue)),
            start=1,
        ):
            channel_row = QHBoxLayout()
            channel_row.setSpacing(3)
            channel_row.addWidget(QLabel(caption))
            channel_row.addWidget(channel)
            face_layout.addLayout(channel_row, 2, column)

        self.face_preview = QLabel("LIVE PREVIEW")
        self.face_preview.setAlignment(Qt.AlignCenter)
        self.face_preview.setMinimumHeight(34)
        self.face_preview.setObjectName("facePreview")
        face_layout.addWidget(self.face_preview, 3, 0, 1, 4)

        self.face_brightness = QSlider(Qt.Horizontal)
        self.face_brightness.setRange(0, 255)
        self.face_brightness.setSingleStep(1)
        self.face_brightness_value = QLabel("80")
        self.face_brightness.valueChanged.connect(
            lambda value: self.face_brightness_value.setText(str(value))
        )
        face_layout.addWidget(QLabel("Brightness"), 4, 0)
        face_layout.addWidget(self.face_brightness, 4, 1, 1, 2)
        face_layout.addWidget(self.face_brightness_value, 4, 3)

        self.face_speed = QSlider(Qt.Horizontal)
        self.face_speed.setRange(10, 60000)
        self.face_speed.setSingleStep(10)
        self.face_speed.setPageStep(100)
        self.face_speed_value = QLabel("1200 ms")
        self.face_speed.valueChanged.connect(
            lambda value: self.face_speed_value.setText("%d ms" % value)
        )
        face_layout.addWidget(QLabel("Animation speed"), 5, 0)
        face_layout.addWidget(self.face_speed, 5, 1, 1, 2)
        face_layout.addWidget(self.face_speed_value, 5, 3)

        face_buttons = QHBoxLayout()
        self.face_apply_button = QPushButton("APPLY")
        self.face_apply_button.clicked.connect(self.apply_face_settings)
        self.face_off_button = QPushButton("OFF")
        self.face_off_button.setObjectName("stop")
        self.face_off_button.clicked.connect(self.stop_face_leds)
        self.face_restore_button = QPushButton("RESTORE DEFAULT")
        self.face_restore_button.clicked.connect(self.restore_default_face)
        self.face_test_button = QPushButton("TEST LEDS")
        self.face_test_button.clicked.connect(self.start_face_test)
        for button in (
            self.face_apply_button,
            self.face_off_button,
            self.face_restore_button,
            self.face_test_button,
        ):
            face_buttons.addWidget(button)
        face_layout.addLayout(face_buttons, 6, 0, 1, 4)

        preset_caption = QLabel("Expression presets")
        preset_caption.setObjectName("statusCaption")
        face_layout.addWidget(preset_caption, 7, 0, 1, 4)
        self.face_preset_buttons = []
        if self.face_catalog is not None:
            for index, expression in enumerate(self.face_catalog.presets):
                button = QPushButton(expression.replace("_", " ").upper())
                button.clicked.connect(
                    lambda _checked=False, name=expression: (
                        self.select_face_preset(name, publish=True)
                    )
                )
                face_layout.addWidget(button, 8 + index // 4, index % 4)
                self.face_preset_buttons.append(button)

        preset_rows = (
            (len(self.face_preset_buttons) + 3) // 4
            if self.face_preset_buttons
            else 1
        )
        self.face_status = QLabel("Waiting for face LED status.")
        self.face_status.setWordWrap(True)
        self.face_status.setObjectName("gaitDetail")
        face_layout.addWidget(self.face_status, 8 + preset_rows, 0, 1, 4)
        emotes_face_layout.addWidget(face_group, 0, 1, alignment=Qt.AlignTop)

        self.face_expression_combo.currentTextChanged.connect(
            self.face_expression_selected
        )
        self.face_enable.toggled.connect(self.face_enabled_changed)
        self.face_auto.toggled.connect(self.face_mode_changed)
        self.face_lock.toggled.connect(self.face_mode_changed)
        if self.face_settings is not None:
            self.set_face_controls(self.face_settings)
        else:
            face_group.setEnabled(False)
            self.face_status.setText(
                "Face configuration unavailable: %s" % self.face_config_error
            )
            self.face_status.setStyleSheet("color: #fca5a5;")

        diagnostic_group = QGroupBox("Hardware Gait Diagnostic")
        diagnostic_layout = QGridLayout(diagnostic_group)
        self.diagnostic_leg = QComboBox()
        for leg in LEG_ORDER:
            self.diagnostic_leg.addItem(leg.replace("_", " ").title(), leg)
        self.diagnostic_duration = self.make_tuning_spinbox(
            6.0, 20.0, 8.0, 1.0, 1, " s"
        )
        diagnostic_layout.addWidget(QLabel("Selected leg"), 0, 0)
        diagnostic_layout.addWidget(self.diagnostic_leg, 0, 1)
        diagnostic_layout.addWidget(QLabel("Duration"), 0, 2)
        diagnostic_layout.addWidget(self.diagnostic_duration, 0, 3)
        diagnostic_buttons = (
            ("A — STAND", lambda: self.send_action("stand")),
            ("B — SLOW SQUAT", lambda: self.start_physical_diagnostic("slow-squat")),
            ("C — SINGLE LEG LIFT", lambda: self.start_physical_diagnostic("single-leg-lift")),
            ("D — STEP ONE LEG", lambda: self.start_physical_diagnostic("single-leg-step")),
            ("E — SELECT AMBLE", lambda: self.select_diagnostic_gait("amble")),
            ("F — SELECT TROT", lambda: self.select_diagnostic_gait("trot")),
        )
        self.diagnostic_buttons = []
        for index, (label, callback) in enumerate(diagnostic_buttons):
            button = QPushButton(label)
            button.clicked.connect(callback)
            diagnostic_layout.addWidget(button, 1 + index // 2, (index % 2) * 2, 1, 2)
            self.diagnostic_buttons.append(button)
        self.run_gait_button = QPushButton("G — SELECT RUN")
        self.run_gait_button.clicked.connect(
            lambda: self.select_diagnostic_gait("run")
        )
        self.run_gait_button.setToolTip(
            "RUN is a fast trot: 1.30 Hz at duty 0.50, the fastest the servo "
            "budget accepts. Entry gates before driving it: zero CRC failures "
            "and sequence gaps over 60 s, clearance >= 8% of module span on "
            "all four feet, roll 1x below 2x. STOP decelerates through TROT "
            "rather than halting mid-cycle."
        )
        diagnostic_layout.addWidget(self.run_gait_button, 4, 0, 1, 4)
        self.stop_diagnostic_button = QPushButton("STOP DIAGNOSTIC")
        self.stop_diagnostic_button.setObjectName("stop")
        self.stop_diagnostic_button.clicked.connect(self.stop_physical_diagnostic)
        diagnostic_layout.addWidget(self.stop_diagnostic_button, 5, 0, 1, 4)
        self.diagnostic_status = QLabel(
            "A-D are finite leased Cartesian tests. E-F select a gait; use the joystick only after the selection is confirmed."
        )
        self.diagnostic_status.setWordWrap(True)
        self.diagnostic_status.setObjectName("gaitDetail")
        diagnostic_layout.addWidget(self.diagnostic_status, 6, 0, 1, 4)
        self.gait_diagnostics = QLabel("Gait diagnostics will appear here.")
        self.gait_diagnostics.setWordWrap(True)
        self.gait_diagnostics.setObjectName("gaitDetail")
        diagnostic_layout.addWidget(self.gait_diagnostics, 7, 0, 1, 4)
        diagnostics_layout.addWidget(
            diagnostic_group,
            0,
            0,
            alignment=Qt.AlignTop,
        )

        telemetry_group = QGroupBox("Commanded Telemetry — No Servo Feedback")
        telemetry_layout = QVBoxLayout(telemetry_group)
        self.commanded_telemetry = QLabel("Waiting for /volt/status.")
        self.commanded_telemetry.setWordWrap(True)
        self.commanded_telemetry.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.commanded_telemetry.setObjectName("gaitDetail")
        telemetry_layout.addWidget(self.commanded_telemetry)
        diagnostics_layout.addWidget(
            telemetry_group,
            0,
            1,
            alignment=Qt.AlignTop,
        )

        link_group = QGroupBox("Firmware Link Health")
        link_layout = QVBoxLayout(link_group)
        self.firmware_link_label = QLabel(
            "Waiting for firmware STATUS (polled every 5 s while connected)."
        )
        self.firmware_link_label.setWordWrap(True)
        self.firmware_link_label.setTextInteractionFlags(
            Qt.TextSelectableByMouse
        )
        self.firmware_link_label.setObjectName("gaitDetail")
        link_layout.addWidget(self.firmware_link_label)
        diagnostics_layout.addWidget(
            link_group,
            1,
            1,
            alignment=Qt.AlignTop,
        )
        emotes_face_layout.setRowStretch(1, 1)
        diagnostics_layout.setRowStretch(2, 1)
        right.addStretch(1)
        workspace.addLayout(right, 4)

        self.arm_mutation_controls = [
            self.stand_button,
            self.sit_button,
            self.enable_motion_button,
            self.step_button,
            self.drive_mode,
            self.speed_slider,
            self.yaw_slider,
            self.body_height,
            self.body_x,
            self.body_y,
            self.roll,
            self.pitch,
            self.yaw,
            self.reset_pose_button,
            self.real_profile_combo,
            self.load_real_profile_button,
            self.reset_real_profile_button,
            self.apply_real_profile_button,
            self.save_real_profile_button,
            self.diagnostic_leg,
            self.diagnostic_duration,
            self.joystick,
            self.emote_repetitions,
            self.emote_speed,
            self.emote_amplitude,
            self.emote_depth,
            self.pushup_travel_mm,
        ]
        self.arm_mutation_controls.extend(self.real_tuning_value_controls)
        self.arm_mutation_controls.extend(self.diagnostic_buttons)
        self.arm_mutation_controls.extend(self.emote_start_buttons)
        self.arm_mutation_controls.extend(self.gait_button_by_name.values())

    def make_spinbox(self, minimum, maximum, value, step, decimals):
        box = QDoubleSpinBox()
        box.setRange(minimum, maximum)
        box.setValue(value)
        box.setSingleStep(step)
        box.setDecimals(decimals)
        return box

    def make_tuning_spinbox(
        self,
        minimum,
        maximum,
        value,
        step,
        decimals,
        suffix,
    ):
        box = self.make_spinbox(minimum, maximum, value, step, decimals)
        box.setSuffix(suffix)
        box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        return box

    def set_face_controls(self, settings):
        if self.face_catalog is None:
            return
        settings = validate_face_settings(settings, self.face_catalog)
        controls = (
            self.face_enable,
            self.face_auto,
            self.face_lock,
            self.face_expression_combo,
            self.face_effect_combo,
            self.face_red,
            self.face_green,
            self.face_blue,
            self.face_brightness,
            self.face_speed,
        )
        previous = [control.blockSignals(True) for control in controls]
        try:
            self.face_enable.setChecked(settings.enabled)
            self.face_auto.setChecked(settings.automatic)
            self.face_lock.setChecked(settings.locked)
            self.face_expression_combo.setCurrentText(settings.expression)
            self.face_effect_combo.setCurrentText(settings.effect)
            self.face_red.setValue(settings.color[0])
            self.face_green.setValue(settings.color[1])
            self.face_blue.setValue(settings.color[2])
            self.face_brightness.setValue(settings.brightness)
            self.face_speed.setValue(settings.speed_ms)
        finally:
            for control, blocked in zip(controls, previous):
                control.blockSignals(blocked)
        self.face_brightness_value.setText(str(settings.brightness))
        self.face_speed_value.setText("%d ms" % settings.speed_ms)
        self.update_face_preview()

    def current_face_settings(self):
        if self.face_catalog is None:
            raise FaceConfigError("face catalog is unavailable")
        expression = self.face_expression_combo.currentText()
        color = (
            self.face_red.value(),
            self.face_green.value(),
            self.face_blue.value(),
        )
        preset = self.face_catalog.presets.get(expression)
        alternate_color = (
            preset.alternate_color
            if preset is not None and preset.alternate_color is not None
            else color
        )
        return validate_face_settings(
            {
                "enabled": self.face_enable.isChecked(),
                "automatic": self.face_auto.isChecked(),
                "locked": self.face_lock.isChecked(),
                "expression": expression,
                "color": color,
                "alternate_color": alternate_color,
                "brightness": self.face_brightness.value(),
                "effect": self.face_effect_combo.currentText(),
                "speed_ms": self.face_speed.value(),
            },
            self.face_catalog,
        )

    def persist_face_settings(self):
        if self.face_catalog is None or self.face_settings is None:
            return False
        try:
            save_face_settings(self.face_settings, self.face_catalog)
            return True
        except (OSError, FaceConfigError, ValueError) as exc:
            self.face_status.setText("Face settings were not saved: %s" % exc)
            self.face_status.setStyleSheet("color: #fca5a5;")
            return False

    def publish_face_request(self, settings):
        """Remember the effective snapshot so reconnect restores it exactly."""
        if self.face_catalog is None:
            return False
        settings = validate_face_settings(settings, self.face_catalog)
        self.face_last_requested_settings = settings
        return self.ros_node.publish_face_settings(settings)

    def update_face_preview(self, *_args):
        if not hasattr(self, "face_preview"):
            return
        red = self.face_red.value()
        green = self.face_green.value()
        blue = self.face_blue.value()
        luminance = (299 * red + 587 * green + 114 * blue) // 1000
        foreground = "#07111e" if luminance >= 135 else "#f8fafc"
        self.face_preview.setText("RGB %d, %d, %d" % (red, green, blue))
        self.face_preview.setStyleSheet(
            "background: rgb(%d, %d, %d); color: %s; border: 1px solid "
            "#64748b; border-radius: 6px; font-weight: 700;"
            % (red, green, blue, foreground)
        )

    def choose_face_color(self):
        selected = QColorDialog.getColor(
            QColor(
                self.face_red.value(),
                self.face_green.value(),
                self.face_blue.value(),
            ),
            self,
            "Choose face LED color",
        )
        if selected.isValid():
            self.face_red.setValue(selected.red())
            self.face_green.setValue(selected.green())
            self.face_blue.setValue(selected.blue())

    def face_expression_selected(self, expression):
        if self.face_catalog is None or expression not in self.face_catalog.presets:
            return
        try:
            current = self.current_face_settings()
            selected = settings_for_preset(
                self.face_catalog,
                expression,
                current,
            )
        except FaceConfigError:
            return
        self.set_face_controls(selected)

    def select_face_preset(self, expression, publish=False):
        if self.face_catalog is None or expression not in self.face_catalog.presets:
            return False
        self.face_expression_combo.setCurrentText(expression)
        if publish:
            self.apply_face_settings()
        return True

    def apply_face_settings(self):
        if self.face_catalog is None:
            return False
        try:
            settings = self.current_face_settings()
        except FaceConfigError as exc:
            self.face_status.setText("Invalid face settings: %s" % exc)
            self.face_status.setStyleSheet("color: #fca5a5;")
            return False
        self.face_settings = settings
        self.face_automation.set_settings(settings)
        self.persist_face_settings()
        published = self.publish_face_request(settings)
        self.face_status.setText(
            "%s requested; waiting for bridge synchronization."
            % settings.expression.replace("_", " ").title()
            if published
            else "Could not publish face LED settings."
        )
        self.face_status.setStyleSheet(
            "color: #7dd3fc;" if published else "color: #fca5a5;"
        )
        if (
            published
            and settings.enabled
            and self.last_face_motion_status
        ):
            self.apply_automatic_face(self.last_face_motion_status)
        return published

    def face_enabled_changed(self, enabled):
        if self.face_catalog is None:
            return
        try:
            self.face_settings = self.current_face_settings()
        except FaceConfigError:
            return
        self.face_automation.set_settings(self.face_settings)
        self.persist_face_settings()
        if enabled:
            self.publish_face_request(self.face_settings)
            self.face_status.setText("Face LEDs enabled; settings requested.")
            self.face_status.setStyleSheet("color: #7dd3fc;")
            if self.last_face_motion_status:
                self.apply_automatic_face(self.last_face_motion_status)
        else:
            self.face_test_timer.stop()
            self.face_test_steps = []
            self.publish_face_request(self.face_settings)
            self.face_status.setText("Face LEDs disabled (effect off requested).")
            self.face_status.setStyleSheet("color: #94a3b8;")

    def face_mode_changed(self, *_args):
        if self.face_catalog is None:
            return
        try:
            self.face_settings = self.current_face_settings()
        except FaceConfigError:
            return
        self.face_automation.set_settings(self.face_settings)
        self.persist_face_settings()
        if self.face_settings.enabled and (
            self.face_settings.locked or not self.face_settings.automatic
        ):
            self.publish_face_request(self.face_settings)
            self.face_status.setText(
                "Manual expression selected; safety overrides remain enabled."
            )
            self.face_status.setStyleSheet("color: #fbbf24;")
        if self.face_settings.enabled and self.last_face_motion_status:
            self.apply_automatic_face(self.last_face_motion_status)

    def stop_face_leds(self):
        if self.face_enable.isChecked():
            self.face_enable.setChecked(False)
        else:
            self.ros_node.publish_face_effect("off")

    def restore_default_face(self):
        if self.face_catalog is None:
            return
        self.face_settings = default_face_settings(self.face_catalog)
        self.face_automation.set_settings(self.face_settings)
        self.set_face_controls(self.face_settings)
        self.persist_face_settings()
        self.publish_face_request(self.face_settings)
        self.face_status.setText("Default idle face restored and requested.")
        self.face_status.setStyleSheet("color: #7dd3fc;")
        if self.last_face_motion_status:
            self.apply_automatic_face(self.last_face_motion_status)

    def start_face_test(self):
        if self.face_catalog is None or self.face_settings is None:
            return
        self.face_test_steps = [
            ((255, 0, 0), "red"),
            ((0, 255, 0), "green"),
            ((0, 0, 255), "blue"),
            ((255, 255, 255), "white"),
        ]
        self.face_test_button.setEnabled(False)
        self.face_status.setText("Testing solid red, green, blue, and white.")
        self.face_status.setStyleSheet("color: #7dd3fc;")
        self.advance_face_test()
        self.face_test_timer.start()

    def advance_face_test(self):
        if not self.face_test_steps:
            self.face_test_timer.stop()
            self.face_test_button.setEnabled(True)
            if self.face_settings.enabled:
                self.publish_face_request(self.face_settings)
                self.face_status.setText("LED test complete; prior face restored.")
            else:
                self.ros_node.publish_face_effect("off")
                self.face_status.setText("LED test complete; face restored off.")
            self.face_status.setStyleSheet("color: #86efac;")
            return
        color, name = self.face_test_steps.pop(0)
        self.ros_node.publish_face_color(color)
        self.ros_node.publish_face_brightness(
            max(40, self.face_settings.brightness)
        )
        self.ros_node.publish_face_effect("solid")
        self.face_status.setText("Testing solid %s." % name)

    def apply_automatic_face(self, status):
        if (
            self.face_automation is None
            or self.face_settings is None
            or not self.face_settings.enabled
            or self.face_test_timer.isActive()
        ):
            return
        decision = self.face_automation.update(status)
        if decision is None:
            return
        if decision.reason == "manual":
            requested = self.face_settings
        else:
            requested = settings_for_preset(
                self.face_catalog,
                decision.expression,
                self.face_settings,
            )
        self.publish_face_request(requested)
        if decision.safety_override:
            text = "SAFETY OVERRIDE: %s" % decision.expression.upper()
            color = "#fca5a5"
        elif decision.restored:
            text = "Automatic face complete; restored %s." % self.face_settings.expression
            color = "#86efac"
        elif decision.reason != "manual":
            text = "Automatic %s (%s)." % (
                decision.expression,
                decision.reason.replace(":", " "),
            )
            color = "#7dd3fc"
        else:
            return
        self.face_status.setText(text)
        self.face_status.setStyleSheet("color: %s;" % color)

    def refresh_firmware_link_health(self, fields):
        """Render the firmware's link counters on the DIAGNOSTICS tab.

        Stage-1 acceptance is fw_crc_fail and fw_seq_gap holding at zero over
        60 s of walking with the face animating, so those two lead the line
        and turn the label red when nonzero.
        """
        label = getattr(self, "firmware_link_label", None)
        if label is None or "fw_crc_fail" not in fields:
            return
        binary = fields.get("binary_frames", "0") == "1"
        crc_fail = fields.get("fw_crc_fail", "?")
        seq_gap = fields.get("fw_seq_gap", "?")
        text = (
            "encoding: %s\n"
            "CRC failures: %s    sequence gaps: %s\n"
            "frames rx: %s binary / %s ascii\n"
            "loop max: %s us    servo I2C max: %s us\n"
            "LED shows: %s    free SRAM: %s bytes"
            % (
                "BINARY (PROTO>=3)" if binary else "ASCII (legacy firmware)",
                crc_fail,
                seq_gap,
                fields.get("fw_frames_bin", "?"),
                fields.get("fw_frames_ascii", "?"),
                fields.get("fw_loop_max_us", "?"),
                fields.get("fw_bus_max_us", "?"),
                fields.get("fw_led_shows", "?"),
                fields.get("fw_sram_free", "?"),
            )
        )
        corrupt = crc_fail not in ("0", "?") or seq_gap not in ("0", "?")
        label.setText(text)
        label.setStyleSheet(
            "color: #f87171;" if corrupt else "color: #86efac;"
        )

    def refresh_face_status(self, fields):
        if not hasattr(self, "face_status") or self.face_catalog is None:
            return
        connected = fields.get("connected") == "1"
        supported = fields.get("face_supported") == "1"
        synced = fields.get("face_synced") == "1"
        connection_active = connected and supported
        reconnect = connection_active and not self.face_connection_active
        self.face_connection_active = connection_active
        if reconnect and self.face_last_requested_settings is not None:
            self.ros_node.publish_face_settings(
                self.face_last_requested_settings
            )
        error = fields.get("led_error", "")
        if error in ("", "-"):
            error = ""
        sync_text, sync_color = face_host_sync_view(fields)
        host_sync_state = str(fields.get("host_sync_state", "")).strip().lower()
        if not connected:
            text, color = "Face LEDs offline; waiting for Arduino.", "#fca5a5"
        elif error:
            text, color = "Face LED error: %s" % error.replace("_", " "), "#fca5a5"
        elif fields.get("face_loading") == "1" or host_sync_state in (
            "waiting_ping",
            "applying_snapshot",
            "verifying_snapshot",
            "finalizing",
            "loading",
            "error",
        ):
            text, color = sync_text, sync_color
        elif not supported:
            text, color = "Connected firmware does not advertise face LED support.", "#fbbf24"
        elif synced:
            text = (
                "SYNCED — %s | %s | RGB A %s / B %s | brightness %s | %s ms"
                % (
                    fields.get("face_expression", "unknown"),
                    fields.get("face_effect", "unknown"),
                    fields.get("face_color", "?"),
                    fields.get("face_color_b", "?"),
                    fields.get("face_brightness", "?"),
                    fields.get("face_speed", "?"),
                )
            )
            color = "#86efac"
        else:
            text = "Face command queued; waiting for firmware synchronization."
            color = "#7dd3fc"
        self.face_status.setText(text)
        self.face_status.setStyleSheet("color: %s;" % color)

    # Status text arrives at wildly different lengths -- "READY" one moment and
    # a three-clause blocker explanation the next.  Every one of these labels
    # word-wraps, so its height is a function of its content, and a longer
    # message reflowed the surrounding layout and grew the window while the
    # operator was reaching for a button.  Reserving a fixed number of text
    # lines makes the geometry independent of the message.  The line counts are
    # the worst case each label is actually given, so nothing is truncated.
    STATUS_LABEL_LINES = (
        ("status_detail", 2),
        ("hardware_detail", 2),
        ("arm_blockers_label", 3),
        ("arm_readiness_state", 1),
        ("emote_status", 2),
        ("face_status", 2),
        ("real_tuning_status", 2),
        ("diagnostic_status", 2),
        ("commanded_telemetry", 3),
        ("controller_detail", 2),
        ("hardware_face_sync", 2),
        ("gait_detail", 2),
        ("gait_diagnostics", 3),
        ("owner_state", 1),
        ("hardware_state", 1),
        ("controller_state", 1),
    )

    def stabilize_status_labels(self):
        """Give variable-length status text a fixed footprint."""
        for name, line_count in self.STATUS_LABEL_LINES:
            label = getattr(self, name, None)
            if label is None:
                continue
            label.setWordWrap(True)
            height = label.fontMetrics().lineSpacing() * line_count + 6
            label.setMinimumHeight(height)
            label.setMaximumHeight(height)
            label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
            # Ignored horizontally so a long single word cannot widen the
            # column either; the label takes whatever width the layout gives.
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)

    def apply_style(self):
        self.setStyleSheet(
            """
            /* VOLT console -- calm neutral surfaces, colour reserved for
               safety state so ARM/STOP/MOTION read instantly. */
            QWidget {
                background: #0f1419;
                color: #e6edf3;
                font-family: "Inter", "DejaVu Sans", sans-serif;
                font-size: 13px;
            }
            QMainWindow, QWidget#root { background: #0f1419; }

            QLabel { background: transparent; }
            QLabel#title {
                color: #e6edf3;
                font-size: 26px;
                font-weight: 800;
                letter-spacing: 3px;
            }
            QLabel#subtitle, QLabel#hint,
            QLabel#statusCaption, QLabel#poseCaption {
                color: #8b98a5;
                font-size: 12px;
            }
            QLabel#state {
                color: #58a6ff;
                font-size: 19px;
                font-weight: 700;
            }
            QLabel#controllerState, QLabel#ownerState, QLabel#hardwareState {
                color: #d29922;
                font-size: 14px;
                font-weight: 700;
            }
            QLabel#gaitDetail, QLabel#hardwareDetail { color: #9fb0c0; }
            QLabel#hardwareWarning, QLabel#armReadinessState {
                color: #d29922;
                font-size: 12px;
                font-weight: 700;
            }
            QLabel#armBlockers { color: #c9d6e2; font-size: 12px; }
            QLabel#safetyTitle { color: #d29922; font-weight: 700; }

            QFrame#armGate, QFrame#safety, QFrame#facePreview {
                background: #161c24;
                border: 1px solid #232c38;
                border-radius: 10px;
            }

            QTabWidget#mainTabs::pane {
                border: 1px solid #232c38;
                border-radius: 10px;
                background: #0f1419;
                top: -1px;
            }
            QTabWidget#mainTabs QScrollArea,
            QTabWidget#mainTabs QScrollArea > QWidget > QWidget {
                background: transparent;
                border: none;
            }
            QTabBar::tab {
                background: transparent;
                border: none;
                border-bottom: 2px solid transparent;
                color: #8b98a5;
                min-width: 132px;
                padding: 11px 20px;
                font-weight: 700;
                letter-spacing: 1px;
            }
            QTabBar::tab:hover { color: #e6edf3; }
            QTabBar::tab:selected {
                color: #58a6ff;
                border-bottom: 2px solid #58a6ff;
            }

            QGroupBox {
                background: #161c24;
                border: 1px solid #232c38;
                border-radius: 10px;
                margin-top: 15px;
                padding: 12px 10px 10px 10px;
                font-weight: 700;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
                color: #8b98a5;
                font-size: 11px;
                letter-spacing: 1px;
            }

            /* min-height keeps every control a comfortable target -- the old
               7px padding made buttons easy to miss while the layout moved. */
            QPushButton {
                background: #1c242e;
                border: 1px solid #2d3846;
                border-radius: 8px;
                padding: 9px 14px;
                min-height: 20px;
                font-weight: 600;
            }
            QPushButton:hover { background: #232d39; border-color: #3d4b5c; }
            QPushButton:pressed { background: #172029; }
            QPushButton:checked {
                background: #10395e;
                border-color: #58a6ff;
                color: #cfe6ff;
            }
            QPushButton:disabled {
                background: #141a21;
                border-color: #1f2731;
                color: #55606c;
            }

            QPushButton#stop, QPushButton#motionDisable,
            QPushButton#hardwareDisable {
                background: #3d1518;
                border-color: #f85149;
                color: #ffd7d5;
            }
            QPushButton#stop:hover, QPushButton#motionDisable:hover,
            QPushButton#hardwareDisable:hover { background: #5c1d21; }
            QPushButton#motionEnable {
                background: #123021;
                border-color: #3fb950;
                color: #b7f0c2;
            }
            QPushButton#motionEnable:hover { background: #17422c; }
            QPushButton#hardwareArm {
                background: #3a2a06;
                border-color: #d29922;
                color: #f5df9b;
            }
            QPushButton#hardwareArm:hover {
                background: #513a08;
                border-color: #e3ad2f;
            }
            QPushButton#hardwareSafe, QPushButton#hardwareRefresh {
                border-color: #58a6ff;
                color: #cfe6ff;
            }

            QComboBox, QDoubleSpinBox, QSpinBox {
                background: #10161d;
                border: 1px solid #2d3846;
                border-radius: 7px;
                padding: 7px 9px;
                min-height: 18px;
                selection-background-color: #10395e;
            }
            QComboBox:focus, QDoubleSpinBox:focus, QSpinBox:focus {
                border-color: #58a6ff;
            }
            QComboBox::drop-down { border: none; width: 20px; }
            QComboBox QAbstractItemView {
                background: #161c24;
                border: 1px solid #2d3846;
                selection-background-color: #10395e;
                outline: none;
            }

            QCheckBox { spacing: 8px; }
            QCheckBox::indicator {
                width: 16px; height: 16px;
                border: 1px solid #2d3846;
                border-radius: 4px;
                background: #10161d;
            }
            QCheckBox::indicator:checked {
                background: #58a6ff;
                border-color: #58a6ff;
            }

            QSlider::groove:horizontal {
                height: 4px;
                background: #232c38;
                border-radius: 2px;
            }
            QSlider::sub-page:horizontal {
                background: #58a6ff;
                border-radius: 2px;
            }
            QSlider::handle:horizontal {
                background: #e6edf3;
                width: 16px;
                margin: -7px 0;
                border-radius: 8px;
            }

            QScrollBar:vertical {
                background: transparent; width: 10px; margin: 0;
            }
            QScrollBar::handle:vertical {
                background: #2d3846; border-radius: 5px; min-height: 30px;
            }
            QScrollBar::handle:vertical:hover { background: #3d4b5c; }
            QScrollBar::add-line, QScrollBar::sub-line { height: 0; }
            QScrollBar::add-page, QScrollBar::sub-page { background: none; }

            QMessageBox { background: #161c24; }
            """
        )

    def joystick_changed(self, forward, horizontal):
        if self.arm_workflow.active or self.duplicate_stack_active:
            self.forward = 0.0
            self.horizontal = 0.0
            return
        self.forward = forward
        self.horizontal = horizontal

    def arm_mutation_is_blocked(self, description):
        """Keep every non-safe command frozen during the guided ARM sequence."""
        if self.duplicate_stack_active:
            self.arm_workflow_notice = (
                "%s blocked: duplicate VOLT stacks are active." % description
            )
            self.status_detail.setText(self.arm_workflow_notice)
            self.status_detail.setStyleSheet("color: #fca5a5;")
            return True
        if not self.arm_workflow.active:
            return False
        self.arm_workflow_notice = (
            "%s blocked while guided ARM is verifying a settled frame."
            % description
        )
        self.status_detail.setText(self.arm_workflow_notice)
        self.status_detail.setStyleSheet("color: #fbbf24;")
        return True

    def set_arm_input_lock(self, locked):
        locked = bool(locked)
        if locked == self.arm_controls_locked:
            return
        self.arm_controls_locked = locked
        if locked:
            self.arm_frozen_pose = self.current_body_pose()
        else:
            self.arm_frozen_pose = None
        for control in self.arm_mutation_controls:
            control.setEnabled(not locked)
        if hasattr(self, "real_tuning_value_controls"):
            self.refresh_real_profile_editability()
        if hasattr(self, "emote_buttons_by_name"):
            self.refresh_emote_controls(
                self.active_emote_request is not None
            )

    def current_body_pose(self):
        if self.arm_workflow.active and self.arm_frozen_pose is not None:
            return self.arm_frozen_pose
        return (
            self.body_x.value(),
            self.body_y.value(),
            self.body_height.value(),
            math.radians(self.roll.value()),
            math.radians(self.pitch.value()),
            math.radians(self.yaw.value()),
        )

    def select_gait(self, gait):
        if self.shutting_down or not rclpy.ok():
            return
        if self.arm_mutation_is_blocked("Gait change"):
            return
        if gait not in self.gait_limits:
            return
        # The speed limit is scoped per gait: a limit tuned for the slow
        # amble must not silently carry into the trot (or later RUN), and a
        # trot limit must not lobotomize the amble.  Save the outgoing gait's
        # slider, restore the incoming gait's last value (default: current).
        if not hasattr(self, "per_gait_speed_percent"):
            self.per_gait_speed_percent = {}
        self.per_gait_speed_percent[self.current_gait] = (
            self.speed_slider.value()
        )
        self.current_gait = gait
        restored = self.per_gait_speed_percent.get(gait)
        if restored is not None and restored != self.speed_slider.value():
            self.speed_slider.setValue(restored)
        self.ros_node.publish_text(self.ros_node.gait_publisher, gait)

    def mark_real_tuning_dirty(self, *_args):
        if not getattr(self, "real_tuning_initialized", False):
            return
        self.real_tuning_dirty = True
        self.real_tuning_status.setText(
            "Values changed locally; STOP and press APPLY to request controller validation."
        )
        self.real_tuning_status.setStyleSheet("color: #fbbf24;")

    def refresh_real_profile_editability(self, *_args):
        """Keep the simulator profile visible but impossible to edit/save."""
        if not hasattr(self, "real_tuning_value_controls"):
            return
        simulation = (
            self.real_profile_combo.currentText().strip().upper()
            == "SIMULATION"
        )
        editable = not self.arm_controls_locked and not simulation
        for control in self.real_tuning_value_controls:
            control.setEnabled(editable)
        self.apply_real_profile_button.setEnabled(editable)
        self.save_real_profile_button.setEnabled(editable)
        if simulation:
            self.real_tuning_status.setText(
                "SIMULATION is read-only; the proven simulator gait remains unchanged."
            )
            self.real_tuning_status.setStyleSheet("color: #94a3b8;")

    def real_tuning_request(self):
        gait = self.real_gait_combo.currentData()
        values = {
            "gait": str(gait),
            "cycle_duration": self.real_cycle.value(),
            "stride_length": self.real_stride_mm.value() / 1000.0,
            "lateral_stride_width": self.real_lateral_mm.value() / 1000.0,
            "step_height": self.real_step_height_mm.value() / 1000.0,
            "duty_factor": self.real_duty.value(),
            "body_height": self.real_body_height_mm.value() / 1000.0,
            "body_x": self.real_body_x_mm.value() / 1000.0,
            "body_y": self.real_body_y_mm.value() / 1000.0,
            "body_roll_deg": self.real_body_roll.value(),
            "body_pitch_deg": self.real_body_pitch.value(),
            "body_yaw_deg": self.real_body_yaw.value(),
            "max_joint_velocity_deg_s": self.real_joint_velocity.value(),
            "max_joint_acceleration_deg_s2": self.real_joint_acceleration.value(),
            "smoothing_amount": self.real_smoothing.value(),
            "touchdown_softness": self.real_touchdown_percent.value() / 100.0,
            "stance_width": self.real_stance_width_mm.value() / 1000.0,
        }
        return validate_tuning(values, allow_simulation=True)

    def set_real_tuning_controls(self, values, profile_name=None):
        values = validate_tuning(values, allow_simulation=True)
        controls = self.real_tuning_value_controls
        previous = [control.blockSignals(True) for control in controls]
        try:
            gait_index = self.real_gait_combo.findData(values["gait"])
            if gait_index >= 0:
                self.real_gait_combo.setCurrentIndex(gait_index)
            self.real_cycle.setValue(values["cycle_duration"])
            self.real_stride_mm.setValue(values["stride_length"] * 1000.0)
            self.real_lateral_mm.setValue(
                values["lateral_stride_width"] * 1000.0
            )
            self.real_step_height_mm.setValue(values["step_height"] * 1000.0)
            self.real_duty.setValue(values["duty_factor"])
            self.real_body_height_mm.setValue(values["body_height"] * 1000.0)
            self.real_body_x_mm.setValue(values["body_x"] * 1000.0)
            self.real_body_y_mm.setValue(values["body_y"] * 1000.0)
            self.real_body_roll.setValue(values["body_roll_deg"])
            self.real_body_pitch.setValue(values["body_pitch_deg"])
            self.real_body_yaw.setValue(values["body_yaw_deg"])
            self.real_joint_velocity.setValue(
                values["max_joint_velocity_deg_s"]
            )
            self.real_joint_acceleration.setValue(
                values["max_joint_acceleration_deg_s2"]
            )
            self.real_smoothing.setValue(values["smoothing_amount"])
            self.real_touchdown_percent.setValue(
                values["touchdown_softness"] * 100.0
            )
            self.real_stance_width_mm.setValue(values["stance_width"] * 1000.0)
        finally:
            for control, blocked in zip(controls, previous):
                control.blockSignals(blocked)
        if profile_name:
            index = self.real_profile_combo.findText(profile_name)
            if index >= 0:
                self.real_profile_combo.setCurrentIndex(index)
        self.real_tuning_initialized = True
        self.real_tuning_dirty = False
        self.refresh_real_profile_editability()

    def sync_live_body_pose_from_tuning(self, values):
        """Reflect a controller-applied profile in the existing live controls."""
        try:
            values = validate_tuning(values, allow_simulation=True)
        except (RealProfileError, TypeError, ValueError):
            return
        controls = (
            self.body_height,
            self.body_x,
            self.body_y,
            self.roll,
            self.pitch,
            self.yaw,
        )
        previous = [control.blockSignals(True) for control in controls]
        try:
            self.body_height.setValue(values["body_height"])
            self.body_x.setValue(values["body_x"])
            self.body_y.setValue(values["body_y"])
            self.roll.setValue(values["body_roll_deg"])
            self.pitch.setValue(values["body_pitch_deg"])
            self.yaw.setValue(values["body_yaw_deg"])
        finally:
            for control, blocked in zip(controls, previous):
                control.blockSignals(blocked)
        self.body_profile_hydrated = True

    def load_selected_real_profile(self):
        if self.arm_mutation_is_blocked("Real-profile load"):
            return
        name = self.real_profile_combo.currentText().strip().upper()
        profile = self.real_profiles.get(name)
        if profile is None:
            self.real_tuning_status.setText("Profile %s is unavailable." % name)
            self.real_tuning_status.setStyleSheet("color: #fca5a5;")
            return
        try:
            self.set_real_tuning_controls(profile, name)
        except (RealProfileError, TypeError, ValueError) as exc:
            self.real_tuning_status.setText("Could not load profile: %s" % exc)
            self.real_tuning_status.setStyleSheet("color: #fca5a5;")
            return
        if name == "SIMULATION":
            self.real_tuning_status.setText(
                "SIMULATION loaded read-only; the proven simulator gait remains unchanged."
            )
        else:
            self.real_tuning_status.setText(
                "%s loaded locally; press APPLY after STOP to make it active." % name
            )
        self.real_tuning_status.setStyleSheet("color: #94a3b8;")

    def save_real_tuning_profile(self):
        if self.arm_mutation_is_blocked("Real-profile save"):
            return
        if self.real_profile_combo.currentText().strip().upper() == "SIMULATION":
            self.refresh_real_profile_editability()
            return
        name, accepted = QInputDialog.getText(
            self,
            "Save V.O.L.T. tuning profile",
            "Profile name (A-Z, 0-9, _ or -):",
        )
        if not accepted:
            return
        try:
            values = self.real_tuning_request()
            path = save_user_profile(name, values)
            normalized = str(name).strip().upper()
            self.real_profiles[normalized] = values
            self.user_real_profile_names.add(normalized)
        except (OSError, RealProfileError, TypeError, ValueError) as exc:
            self.real_tuning_status.setText("Profile was not saved: %s" % exc)
            self.real_tuning_status.setStyleSheet("color: #fca5a5;")
            return
        if self.real_profile_combo.findText(normalized) < 0:
            self.real_profile_combo.addItem(normalized)
        self.real_profile_combo.setCurrentText(normalized)
        self.real_tuning_status.setText(
            "Saved %s to %s; values are not applied until APPLY is pressed."
            % (normalized, path)
        )
        self.real_tuning_status.setStyleSheet("color: #86efac;")

    def apply_real_tuning(self):
        if self.arm_mutation_is_blocked("Real-robot tuning"):
            return
        if self.real_profile_combo.currentText().strip().upper() == "SIMULATION":
            self.refresh_real_profile_editability()
            return
        try:
            values = self.real_tuning_request()
        except (RealProfileError, TypeError, ValueError) as exc:
            self.real_tuning_status.setText("Invalid values: %s" % exc)
            self.real_tuning_status.setStyleSheet("color: #fca5a5;")
            return
        request_id = "gui-%s" % uuid.uuid4().hex[:20]
        payload = {
            "request_id": request_id,
            "profile_name": self.real_profile_combo.currentText().strip().upper()
            or "CUSTOM",
            "values": values,
        }
        if self.ros_node.publish_json(
            self.ros_node.real_tuning_publisher,
            payload,
        ):
            self.pending_real_tuning_request = request_id
            self.real_tuning_status.setText(
                "Profile request sent; waiting for the controller's correlated acknowledgement."
            )
            self.real_tuning_status.setStyleSheet("color: #7dd3fc;")
        else:
            self.real_tuning_status.setText("Could not publish profile request.")
            self.real_tuning_status.setStyleSheet("color: #fca5a5;")

    def emote_busy(self):
        return bool(
            self.controller_emote_busy
            or self.active_emote_request is not None
        )

    def emote_button_blocker(self, name, pose_action=False):
        return emote_start_blocker(
            name=name,
            advertised=self.available_emotes,
            command_owner=self.command_owner,
            status_fresh=self.motion_status_is_fresh(),
            controller_connected=self.motion_controller_connected,
            motion_state=self.motion_state,
            physical_busy=self.active_physical_request is not None,
            emote_busy=(
                self.emote_busy()
                or self.pending_emote_pose_action is not None
            ),
            controls_locked=self.arm_controls_locked,
            duplicate_stack=self.duplicate_stack_active,
            pose_action=pose_action,
        )

    def set_emote_notice(self, message, color="#94a3b8"):
        self.emote_notice = str(message)
        self.emote_notice_color = str(color)
        self.emote_notice_time = time.monotonic()
        self.emote_status.setText(self.emote_notice)
        self.emote_status.setStyleSheet("color: %s;" % self.emote_notice_color)

    def request_emote_pose_action(self, action):
        """Request the SIT/STAND pose buttons while reporting failures here."""
        action = str(action).strip().lower()
        if action not in ("sit", "stand"):
            return
        if self.arm_mutation_is_blocked("%s pose" % action.upper()):
            self.set_emote_notice(
                self.arm_workflow_notice or "Pose request is currently locked.",
                "#fbbf24",
            )
            return
        blocker = self.emote_button_blocker(action, pose_action=True)
        if blocker:
            self.set_emote_notice("Blocked: %s" % blocker, "#fbbf24")
            return
        if self.shutting_down or not rclpy.ok():
            self.set_emote_notice(
                "Could not publish %s: ROS is not running." % action.upper(),
                "#fca5a5",
            )
            return
        self.latch_motion_until_neutral()
        self.neutralize_motion_controls()
        self.pending_emote_pose_action = {
            "name": action,
            "started_at": time.monotonic(),
        }
        self.send_action(action)
        self.set_emote_notice(
            "%s requested; waiting for controller acknowledgement."
            % action.replace("_", " ").title(),
            "#7dd3fc",
        )
        self.refresh_emote_controls(self.emote_busy())

    def start_emote(self, name):
        """Request one controller-owned finite emote without blocking Qt."""
        if self.arm_mutation_is_blocked("Robot emote"):
            self.set_emote_notice(
                self.arm_workflow_notice or "Emote request is currently locked.",
                "#fbbf24",
            )
            return
        name = str(name).strip().lower()
        blocker = self.emote_button_blocker(name)
        if blocker:
            self.set_emote_notice("Blocked: %s" % blocker, "#fbbf24")
            return
        if self.hardware_mode and name in BALANCE_SENSITIVE_EMOTES:
            answer = QMessageBox.warning(
                self,
                "Confirm balance-sensitive emote",
                (
                    "This emote unloads one or both front feet. Start with "
                    "V.O.L.T. supported, keep the power disconnect accessible, "
                    "and use the default 1.0× settings before floor testing."
                ),
                QMessageBox.Yes | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            if answer != QMessageBox.Yes:
                return

        request = {
            "request_id": "gui-emote-%s" % uuid.uuid4().hex[:16],
            "name": name,
            "started_at": time.monotonic(),
        }
        request.update(
            gui_emote_request_options(
                name=name,
                repetitions=self.emote_repetitions.value(),
                speed=self.emote_speed.value(),
                amplitude=self.emote_amplitude.value(),
                depth=self.emote_depth.value(),
                pushup_travel_mm=self.pushup_travel_mm.value(),
            )
        )
        payload = dict(request)
        payload.pop("started_at")
        payload["command"] = "start"
        self.latch_motion_until_neutral()
        self.neutralize_motion_controls()
        if not self.ros_node.publish_json(
            self.ros_node.emote_publisher,
            payload,
        ):
            self.set_emote_notice(
                "Could not publish the emote request.",
                "#fca5a5",
            )
            return
        self.active_emote_request = request
        self.emote_was_active = False
        request_description = name.replace("_", " ").title()
        if name == "push_ups":
            request_description += " at %.0f mm travel" % (
                request["depth"] * PUSHUP_TRAVEL_BASE_MM
            )
        self.set_emote_notice(
            "%s requested; locomotion will settle before playback starts."
            % request_description,
            "#7dd3fc",
        )
        self.refresh_emote_controls(True)

    def stop_emote(self):
        """Request one correlated smooth return without cross-topic races."""
        self.latch_motion_until_neutral()
        self.neutralize_motion_controls()
        active = self.active_emote_request
        request_id = (
            active.get("request_id", "") if active is not None else ""
        ) or (
            self.controller_emote_request_id
            if self.controller_emote_busy
            else ""
        )
        if request_id:
            published = self.ros_node.publish_json(
                self.ros_node.emote_publisher,
                {
                    "command": "cancel",
                    "request_id": request_id,
                },
            )
            if not published:
                self.ros_node.publish_text(
                    self.ros_node.action_publisher,
                    "stop",
                )
        else:
            self.ros_node.publish_text(
                self.ros_node.action_publisher,
                "stop",
            )
        self.set_emote_notice(
            "STOP requested; returning smoothly to the captured stand pose.",
            "#fbbf24",
        )

    def refresh_emote_controls(self, busy):
        busy = bool(busy or self.emote_busy())
        locked = bool(self.arm_controls_locked)
        for name, button in self.emote_buttons_by_name.items():
            blocker = self.emote_button_blocker(name)
            button.setEnabled(not blocker)
            button.setToolTip(blocker or "Start the %s emote." % name.replace("_", " "))
        for action, button in self.emote_action_buttons_by_name.items():
            blocker = self.emote_button_blocker(action, pose_action=True)
            button.setEnabled(not blocker)
            button.setToolTip(
                blocker or "Request the controller's %s pose." % action.upper()
            )
        for control in (
            self.emote_repetitions,
            self.emote_speed,
            self.emote_amplitude,
            self.emote_depth,
            self.pushup_travel_mm,
        ):
            control.setEnabled(not busy and not locked)
        self.stop_emote_button.setEnabled(bool(busy))

    def expire_emote_requests_without_status(self, now=None):
        """Release local pending UI state if controller status disappears."""
        if now is None:
            now = time.monotonic()
        if self.motion_status_is_fresh(now):
            return False

        expired = False
        local = self.active_emote_request
        if local is not None:
            last_response = float(
                local.get("last_status_at", local.get("started_at", now))
            )
            if now - last_response > EMOTE_REQUEST_ACK_TIMEOUT:
                self.active_emote_request = None
                expired = True
        pose = self.pending_emote_pose_action
        if (
            pose is not None
            and now - float(pose.get("started_at", now))
            > EMOTE_REQUEST_ACK_TIMEOUT
        ):
            self.pending_emote_pose_action = None
            expired = True
        if expired:
            self.set_emote_notice(
                "Request failed: motion-controller status was lost before "
                "a terminal acknowledgement. Check /volt/status and restart "
                "the single VOLT stack if needed.",
                "#fca5a5",
            )
        return expired

    def update_emote_status(self, status):
        """Correlate controller emote state with this GUI's request."""
        now = time.monotonic()
        advertised = status.get("emotes_available")
        if isinstance(advertised, (list, tuple)):
            self.available_emotes = {
                str(name).strip().lower()
                for name in advertised
                if str(name).strip()
            }
            self.emote_catalog_received = True
            self.emote_catalog_error = ""
        else:
            # Fail closed on controller restart/downgrade instead of retaining
            # buttons enabled from an earlier status authority.
            self.available_emotes = set()
            self.emote_catalog_received = False
            if "emotes_available" in status:
                self.emote_catalog_error = (
                    "Controller reported an invalid emote catalog; expected a list."
                )
            else:
                self.emote_catalog_error = (
                    "Controller status has no emote catalog; this controller "
                    "build cannot run Cartesian emotes."
                )
        active = bool(status.get("emote_active", False))
        pending = bool(status.get("emote_pending", False))
        name = str(status.get("emote_name", "")).strip().lower()
        state = str(status.get("emote_state", "idle")).strip().lower()
        result = str(status.get("emote_result", "idle")).strip().lower()
        request_id = str(status.get("emote_request_id", "")).strip()
        message = str(status.get("emote_message", "")).strip()
        try:
            progress = max(
                0.0,
                min(1.0, float(status.get("emote_progress", 0.0))),
            )
        except (TypeError, ValueError):
            progress = 0.0

        self.controller_emote_busy = bool(active or pending)
        self.controller_emote_request_id = (
            request_id if self.controller_emote_busy else ""
        )

        pose_view = None
        pose_request = self.pending_emote_pose_action
        if pose_request is not None:
            pose_name = pose_request["name"]
            target_state = "standing" if pose_name == "stand" else "sitting"
            transition_state = (
                "standing_up" if pose_name == "stand" else "sitting_down"
            )
            pose_display = pose_name.replace("_", " ").title()
            if self.motion_state == target_state:
                self.pending_emote_pose_action = None
                self.set_emote_notice(
                    "%s completed; controller reports %s."
                    % (pose_display, target_state.upper()),
                    "#86efac",
                )
                pose_view = (self.emote_notice, self.emote_notice_color)
            elif self.motion_state == transition_state:
                pose_view = (
                    "%s in progress; controller reports %s."
                    % (pose_display, transition_state.upper()),
                    "#7dd3fc",
                )
            elif now - pose_request["started_at"] > EMOTE_REQUEST_ACK_TIMEOUT:
                self.pending_emote_pose_action = None
                warning = str(status.get("warning", "")).strip()
                self.set_emote_notice(
                    "%s was not acknowledged by the controller%s."
                    % (pose_display, ": %s" % warning if warning else ""),
                    "#fca5a5",
                )
                pose_view = (self.emote_notice, self.emote_notice_color)
            else:
                pose_view = (
                    "%s requested; waiting for controller acknowledgement."
                    % pose_display,
                    "#7dd3fc",
                )

        local = self.active_emote_request
        correlated = bool(
            local is not None
            and request_id
            and request_id == local.get("request_id")
        )
        if correlated:
            local["last_status_at"] = now
        if correlated and (active or pending):
            self.emote_was_active = True
        terminal = result in ("completed", "cancelled", "rejected")
        correlated_terminal = bool(correlated and terminal)
        if correlated and terminal and not active and not pending:
            self.active_emote_request = None
            local = None

        if local is not None and not correlated:
            last_response = float(
                local.get("last_status_at", local.get("started_at", now))
            )
            if now - last_response > EMOTE_REQUEST_ACK_TIMEOUT:
                timed_out_name = str(local.get("name", "emote")).replace(
                    "_", " "
                ).title()
                self.active_emote_request = None
                local = None
                self.set_emote_notice(
                    "%s failed: no correlated controller acknowledgement. "
                    "Verify that /volt/emote has one subscriber and restart "
                    "the controller if its catalog is missing." % timed_out_name,
                    "#fca5a5",
                )

        busy = bool(active or pending or self.active_emote_request is not None)
        display_name = (name or (local or {}).get("name", "emote")).replace(
            "_", " "
        ).title()
        if pose_view is not None:
            self.emote_status.setText(pose_view[0])
            self.emote_status.setStyleSheet("color: %s;" % pose_view[1])
        elif active:
            if state == "returning":
                self.emote_status.setText(
                    "%s returning to stand — %.0f%%. %s"
                    % (display_name, 100.0 * progress, message)
                )
                self.emote_status.setStyleSheet("color: #fbbf24;")
            elif state == "settling":
                self.emote_status.setText(
                    "%s at neutral target; waiting for commanded joints "
                    "to settle. %s" % (display_name, message)
                )
                self.emote_status.setStyleSheet("color: #fbbf24;")
            else:
                self.emote_status.setText(
                    "%s running — %.0f%% commanded trajectory."
                    % (display_name, 100.0 * progress)
                )
                self.emote_status.setStyleSheet("color: #86efac;")
        elif pending:
            self.emote_status.setText(message or "%s queued." % display_name)
            self.emote_status.setStyleSheet("color: #7dd3fc;")
        elif correlated_terminal or (
            self.active_emote_request is None and request_id and terminal
        ):
            self.emote_status.setText(message or result.title())
            terminal_color = {
                "rejected": "#fca5a5",
                "cancelled": "#fbbf24",
                "completed": "#86efac",
            }[result]
            self.emote_status.setStyleSheet("color: %s;" % terminal_color)
        elif self.active_emote_request is not None:
            requested_name = str(
                self.active_emote_request.get("name", "emote")
            ).replace("_", " ").title()
            self.emote_status.setText(
                "%s request published; waiting for a correlated acknowledgement."
                % requested_name
            )
            self.emote_status.setStyleSheet("color: #7dd3fc;")
        elif (
            self.emote_notice
            and now - self.emote_notice_time <= EMOTE_ERROR_DISPLAY_TIME
        ):
            self.emote_status.setText(self.emote_notice)
            self.emote_status.setStyleSheet(
                "color: %s;" % self.emote_notice_color
            )
        elif self.emote_catalog_error:
            self.emote_status.setText(self.emote_catalog_error)
            self.emote_status.setStyleSheet("color: #fca5a5;")
        else:
            missing = [
                emote_name
                for _caption, emote_name in DISPLAYED_CARTESIAN_EMOTES
                if emote_name not in self.available_emotes
            ]
            if missing:
                self.emote_status.setText(
                    "Controller catalog is missing displayed emotes: %s."
                    % ", ".join(name.replace("_", " ") for name in missing)
                )
                self.emote_status.setStyleSheet("color: #fca5a5;")
            else:
                self.emote_status.setText(
                    "All %d Cartesian emotes are available; SIT/STAND use "
                    "the pose controller."
                    % len(DISPLAYED_CARTESIAN_EMOTES)
                )
                self.emote_status.setStyleSheet("color: #86efac;")
        self.refresh_emote_controls(busy)

    def physical_test_payload(self, command, active=None):
        active = active or self.active_physical_request
        if not active:
            return None
        return {
            "command": command,
            "mode": active["mode"],
            "duration": active["duration"],
            "request_id": active["request_id"],
            "leg": active["leg"],
        }

    def start_physical_diagnostic(self, mode):
        if self.arm_mutation_is_blocked("Hardware diagnostic"):
            return
        if not self.hardware_mode or not self.physical_tests_enabled:
            self.diagnostic_status.setText(
                "Blocked: launch hardware mode with enable_physical_tests:=true."
            )
            self.diagnostic_status.setStyleSheet("color: #fca5a5;")
            return
        if self.active_physical_request is not None:
            self.diagnostic_status.setText("Stop the active diagnostic first.")
            return
        answer = QMessageBox.warning(
            self,
            "Confirm finite physical diagnostic",
            (
                "Keep the power disconnect accessible. For first use, support "
                "V.O.L.T. on its stand; floor tests require a stable planted "
                "pose and a clear area. The controller will refuse motion unless "
                "MOTION ownership, neutral velocity, and a stopped stand are confirmed."
            ),
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return
        leg = self.diagnostic_leg.currentData() if mode in (
            "single-leg-lift",
            "single-leg-step",
        ) else ""
        active = {
            "request_id": "gui-test-%s" % uuid.uuid4().hex[:16],
            "mode": mode,
            "duration": self.diagnostic_duration.value(),
            "leg": str(leg),
            "started_at": time.monotonic(),
        }
        self.active_physical_request = active
        self.physical_test_was_active = False
        if not self.ros_node.publish_json(
            self.ros_node.physical_test_publisher,
            self.physical_test_payload("start", active),
        ):
            self.active_physical_request = None
            self.diagnostic_status.setText("Could not publish diagnostic request.")
            return
        self.diagnostic_status.setText(
            "Requested %s for %.1f s; waiting for controller acceptance."
            % (mode, active["duration"])
        )
        self.diagnostic_status.setStyleSheet("color: #7dd3fc;")

    def publish_diagnostic_keepalive(self):
        self.publish_emote_keepalive()
        active = self.active_physical_request
        if active is None or self.shutting_down or not rclpy.ok():
            return
        elapsed = time.monotonic() - active["started_at"]
        if elapsed > active["duration"] + 2.0:
            self.stop_physical_diagnostic()
            return
        self.ros_node.publish_json(
            self.ros_node.physical_test_publisher,
            self.physical_test_payload("keepalive"),
        )

    def publish_emote_keepalive(self):
        """Renew only while this GUI also receives fresh controller status."""
        active = self.active_emote_request
        if (
            active is None
            or not active.get("request_id")
            or self.shutting_down
            or not rclpy.ok()
            or not self.motion_status_is_fresh()
        ):
            return
        self.ros_node.publish_json(
            self.ros_node.emote_publisher,
            {
                "command": "keepalive",
                "request_id": active["request_id"],
            },
        )

    def stop_physical_diagnostic(self):
        if self.active_physical_request is not None:
            self.ros_node.publish_json(
                self.ros_node.physical_test_publisher,
                self.physical_test_payload("cancel"),
            )
            self.active_physical_request = None
        self.send_action("stop")
        self.diagnostic_status.setText(
            "STOP requested; controller is returning to the supported stand."
        )
        self.diagnostic_status.setStyleSheet("color: #fbbf24;")

    def select_diagnostic_gait(self, gait):
        if gait not in GAIT_SEQUENCE:
            return
        self.stop_motion_controls()
        self.select_gait(gait)
        self.diagnostic_status.setText(
            "%s requested. Wait for status confirmation, then begin with minimal joystick command."
            % GAIT_DISPLAY_NAMES[gait]
        )
        self.diagnostic_status.setStyleSheet("color: #7dd3fc;")

    def send_action(self, action):
        if self.shutting_down or not rclpy.ok():
            return
        if self.arm_workflow.active:
            if action == "stop":
                self.cancel_arm_workflow(
                    "Arming cancelled by the operator's STOP command."
                )
            else:
                self.arm_mutation_is_blocked(
                    "%s action" % str(action).strip().upper()
                )
            return
        if action != "stop" and self.arm_mutation_is_blocked(
            "%s action" % str(action).strip().upper()
        ):
            return
        if action in ("stop", "sit"):
            self.joystick.set_vector(0.0, 0.0)
            self.yaw_slider.setValue(0)
            # Decelerate through the slower gait rather than halting a fast
            # one mid-cycle: RUN carries the body on two diagonal feet with a
            # short stance, and stopping dead there drops the robot onto
            # whichever diagonal happens to be down.
            if self.current_gait == "run":
                self.select_gait("trot")
                self.status_detail.setText(
                    "Decelerating through TROT before stopping."
                )
                self.status_detail.setStyleSheet("color: #7dd3fc;")
        if action in ("stand", "sit", "step") and self.command_owner != "MOTION":
            self.status_detail.setText(
                "Action blocked: press ENABLE MOTION to grant ROS command ownership."
            )
            self.status_detail.setStyleSheet("color: #fbbf24;")
            return
        if action == "step" and self.motion_state != "standing":
            self.status_detail.setText(
                "Step-in-place blocked until the controller state is STANDING."
            )
            self.status_detail.setStyleSheet("color: #fbbf24;")
            return
        self.ros_node.publish_text(self.ros_node.action_publisher, action)

    def publish_zero_velocity(self):
        if not rclpy.ok():
            return
        try:
            self.ros_node.velocity_publisher.publish(Twist())
        except Exception:
            pass

    def latch_motion_until_neutral(self):
        self.motion_neutral_latched = True
        self.motion_neutral_since = 0.0

    def motion_inputs_are_neutral(self):
        return (
            abs(self.forward) <= 1e-6
            and abs(self.horizontal) <= 1e-6
            and self.yaw_slider.value() == 0
        )

    def neutral_latch_allows_motion(self, now):
        if not self.motion_neutral_latched:
            return True
        if not self.motion_inputs_are_neutral():
            self.motion_neutral_since = 0.0
            return False
        if self.motion_neutral_since <= 0.0:
            self.motion_neutral_since = now
            return False
        if now - self.motion_neutral_since < NEUTRAL_RELEASE_TIME:
            return False
        self.motion_neutral_latched = False
        self.motion_neutral_since = 0.0
        return True

    def neutralize_motion_controls(self):
        """Zero local and ROS velocity inputs without another control topic."""
        self.forward = 0.0
        self.horizontal = 0.0
        self.joystick.set_vector(0.0, 0.0)
        self.yaw_slider.setValue(0)
        self.publish_zero_velocity()
        self.last_step_keepalive_publish_time = 0.0

    def stop_motion_controls(self):
        self.neutralize_motion_controls()
        self.ros_node.publish_text(self.ros_node.action_publisher, "stop")

    def enable_motion(self):
        if self.shutting_down or not rclpy.ok():
            return
        if self.arm_mutation_is_blocked("Manual ownership change"):
            return
        self.latch_motion_until_neutral()
        self.stop_motion_controls()
        self.motion_ownership_requested = True
        self.last_owner_heartbeat = time.monotonic()
        self.ros_node.set_command_owner("MOTION")

    def publish_owner_heartbeat(self, now):
        """Keep MOTION ownership alive while the operator still wants it.

        The router ages ownership from the pose stream, so a gait that stops
        or an idle controller silently lets MOTION lapse and the bridge then
        blocks frames with "ownership is not fresh MOTION".  Re-asserting the
        operator's standing request at 5 Hz keeps a deliberate MOTION session
        alive without weakening the interlock: the request is set only by an
        explicit ENABLE MOTION and cleared by every HOLD/DISABLE path, and a
        GUI that dies stops publishing, so the router still falls back to HOLD
        within its own stale_timeout.
        """
        if not self.motion_ownership_requested:
            return
        if self.arm_workflow.active or self.duplicate_stack_active:
            # Never contend with the guided ARM sequence's own ownership steps.
            return
        if now - self.last_owner_heartbeat < 0.2:
            return
        self.last_owner_heartbeat = now
        self.ros_node.claim_motion_owner()

    def hold_motion(self):
        if self.shutting_down or not rclpy.ok():
            return
        if self.arm_workflow.active:
            self.cancel_arm_workflow("Arming cancelled by ROS HOLD.")
            return
        self.latch_motion_until_neutral()
        self.stop_motion_controls()
        self.motion_ownership_requested = False
        self.ros_node.set_command_owner("HOLD")

    def disable_motion(self):
        if self.shutting_down or not rclpy.ok():
            return
        if self.arm_workflow.active:
            self.cancel_arm_workflow(
                "Arming cancelled because ROS output commands were disabled.",
                owner_override="DISABLED",
            )
            return
        self.latch_motion_until_neutral()
        self.stop_motion_controls()
        self.motion_ownership_requested = False
        self.ros_node.set_command_owner("DISABLED")

    def changeEvent(self, event):
        """Turn every retained keyboard/mouse/gamepad input into a safe stop."""
        super().changeEvent(event)
        if (
            event.type() == QEvent.ActivationChange
            and not self.isActiveWindow()
            and not getattr(self, "shutting_down", True)
            and hasattr(self, "joystick")
        ):
            if self.arm_workflow.active:
                self.cancel_arm_workflow(
                    "Arming cancelled because the control window lost focus."
                )
                return
            # Losing focus is not by itself a reason to disarm.  The real
            # hazard is retained input -- a held key, a leaning gamepad stick --
            # continuing to drive the robot while the operator is elsewhere, and
            # that is fully answered by zeroing the commands and latching the
            # neutral requirement below.  Disarming outright on every focus
            # change meant a notification, a click on the Gazebo window, or a
            # glance at a terminal dropped the whole system to HOLD, which is
            # the "randomly disarms" behaviour.  An abandoned window is still
            # disarmed, just after a grace period rather than instantly.
            self.latch_motion_until_neutral()
            self.stop_motion_controls()
            snapshot = self.arm_snapshot()
            if snapshot.armed or self.arm_workflow.state == STATE_ARMED:
                self.focus_hold_timer.start(
                    int(self.focus_hold_grace_seconds * 1000.0)
                )
        elif (
            event.type() == QEvent.ActivationChange
            and self.isActiveWindow()
            and hasattr(self, "focus_hold_timer")
        ):
            # The operator came back before the grace period expired.
            self.focus_hold_timer.stop()

    def focus_hold_timeout(self):
        """Disarm only after the window has stayed unfocused."""
        if self.shutting_down or not rclpy.ok():
            return
        if self.isActiveWindow():
            return
        snapshot = self.arm_snapshot()
        if not (snapshot.armed or self.arm_workflow.state == STATE_ARMED):
            return
        self.return_both_layers_to_hold(
            "System returned to HOLD after the control window stayed "
            "unfocused for %.0f s." % self.focus_hold_grace_seconds
        )

    def send_serial_command(self, command):
        if self.shutting_down or not rclpy.ok():
            return False
        return self.ros_node.send_serial_command(command)

    def send_hardware_safe_command(self, command):
        command = str(command).strip().upper()
        if self.arm_workflow.active:
            self.cancel_arm_workflow(
                "Arming cancelled by %s." % command,
                serial_override=command,
            )
            return
        self.return_both_layers_to_hold(
            "ROS ownership returned to HOLD; firmware %s requested." % command,
            serial_command=command,
        )

    def return_both_layers_to_hold(self, reason, serial_command="HOLD"):
        """Apply a zero/STOP and both independent HOLD layers immediately."""
        self.latch_motion_until_neutral()
        self.stop_motion_controls()
        self.motion_ownership_requested = False
        self.ros_node.set_command_owner("HOLD")
        self.send_serial_command(serial_command)
        self.arm_workflow_notice = str(reason)

    def publish_motion(self):
        if self.shutting_down or not rclpy.ok():
            return
        now = time.monotonic()
        self.publish_owner_heartbeat(now)
        speed = self.speed_slider.value() / 100.0
        max_x, max_y, max_yaw = self.gait_limits[self.current_gait]
        message = Twist()
        motion_allowed = (
            self.motion_state == "standing"
            and self.command_owner == "MOTION"
            and self.motion_status_is_fresh(now)
            and self.router_status_is_fresh(now)
            and not self.arm_workflow.active
            and not self.duplicate_stack_active
        )
        if not motion_allowed:
            self.latch_motion_until_neutral()
        elif self.neutral_latch_allows_motion(now):
            message.linear.x = self.forward * max_x * speed

            if self.drive_mode.currentIndex() == 0:
                message.angular.z = -self.horizontal * max_yaw * speed
            else:
                message.linear.y = -self.horizontal * max_y * speed
                message.angular.z = (
                    self.yaw_slider.value() / 100.0 * max_yaw * speed
                )
        try:
            self.ros_node.velocity_publisher.publish(message)
        except Exception:
            return
        if (
            motion_allowed
            and self.motion_step_in_place
            and now - self.last_step_keepalive_publish_time
            >= STEP_KEEPALIVE_PERIOD
        ):
            self.ros_node.publish_text(
                self.ros_node.action_publisher,
                "step_keepalive",
            )
            self.last_step_keepalive_publish_time = now

    def publish_pose(self):
        if self.shutting_down or not rclpy.ok():
            return
        if (
            self.duplicate_stack_active
            or self.pending_real_tuning_request
            or self.active_emote_request is not None
            or self.active_physical_request is not None
        ):
            return
        pose = self.current_body_pose()
        message = Twist()
        message.linear.x = pose[0]
        message.linear.y = pose[1]
        message.linear.z = pose[2]
        message.angular.x = pose[3]
        message.angular.y = pose[4]
        message.angular.z = pose[5]
        try:
            self.ros_node.pose_publisher.publish(message)
        except Exception:
            return

    def reset_body_pose(self):
        if self.arm_mutation_is_blocked("Body-pose reset"):
            return
        self.body_height.setValue(0.200)
        self.body_x.setValue(0.0)
        self.body_y.setValue(0.0)
        self.roll.setValue(0.0)
        self.pitch.setValue(0.0)
        self.yaw.setValue(0.0)

    def toggle_gamepad(self, checked):
        self.gamepad_enabled = bool(checked)
        self.controller_enable.setText(
            "GAMEPAD ENABLED" if self.gamepad_enabled else "GAMEPAD DISABLED"
        )
        if not self.gamepad_enabled:
            if self.arm_workflow.active:
                self.cancel_arm_workflow(
                    "Arming cancelled because the gamepad was disabled."
                )
            self.latch_motion_until_neutral()
            self.stop_motion_controls()
        self.update_gamepad_status()

    def apply_deadzone(self, value, axis_index=None):
        value = float(value)
        magnitude = abs(value)
        active = bool(self.gamepad_axis_active.get(axis_index, False))
        threshold = (
            GAMEPAD_DEADZONE_RELEASE if active else GAMEPAD_DEADZONE
        )
        if magnitude < threshold:
            self.gamepad_axis_active[axis_index] = False
            return 0.0
        self.gamepad_axis_active[axis_index] = True
        scaled = (magnitude - threshold) / (1.0 - threshold)
        return math.copysign(min(1.0, scaled), value)

    def refresh_gamepad(self):
        if not self.gamepad_available:
            return
        now = time.monotonic()
        if now - self.last_gamepad_scan < 1.0:
            return
        self.last_gamepad_scan = now

        pygame.joystick.quit()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= 0:
            self.gamepad = None
            self.gamepad_name = ""
            self.gamepad_buttons = {}
            self.gamepad_axis_active = {}
            return

        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        self.gamepad_name = self.gamepad.get_name()
        self.gamepad_buttons = {
            index: False for index in range(self.gamepad.get_numbuttons())
        }
        self.gamepad_axis_active = {}

    def handle_gamepad_disconnect(self):
        """Stop immediately instead of retaining the last disconnected axis."""
        if self.arm_workflow.active:
            self.cancel_arm_workflow(
                "Arming cancelled because the gamepad disconnected."
            )
        self.latch_motion_until_neutral()
        self.gamepad = None
        self.gamepad_name = ""
        self.gamepad_buttons = {}
        self.gamepad_axis_active = {}
        self.stop_motion_controls()

    def update_gamepad_status(self):
        if not self.gamepad_available:
            self.controller_state.setText("PYGAME NOT INSTALLED")
            self.controller_state.setStyleSheet("color: #fca5a5;")
            return
        if not self.gamepad_enabled:
            self.controller_state.setText("DISABLED")
            self.controller_state.setStyleSheet("color: #94a3b8;")
            return
        if self.gamepad is None:
            self.controller_state.setText("NOT CONNECTED")
            self.controller_state.setStyleSheet("color: #fbbf24;")
            return
        self.controller_state.setText("CONNECTED: %s" % self.gamepad_name)
        self.controller_state.setStyleSheet("color: #86efac;")

    def axis_value(self, axis_index):
        if self.gamepad is None or axis_index >= self.gamepad.get_numaxes():
            return 0.0
        return self.apply_deadzone(
            self.gamepad.get_axis(axis_index),
            axis_index,
        )

    def choose_gait_offset(self, offset):
        if self.current_gait not in GAIT_SEQUENCE:
            index = 0
        else:
            index = GAIT_SEQUENCE.index(self.current_gait)
        self.select_gait(GAIT_SEQUENCE[(index + offset) % len(GAIT_SEQUENCE)])
        button = self.gait_button_by_name.get(self.current_gait)
        if button is not None:
            button.setChecked(True)

    def handle_gamepad_action(self, action):
        if self.arm_workflow.active:
            if action == "stop":
                self.send_action(action)
            else:
                self.arm_mutation_is_blocked(
                    "Gamepad %s" % str(action).strip().upper()
                )
            return
        if action == "prev_gait":
            self.choose_gait_offset(-1)
        elif action == "next_gait":
            self.choose_gait_offset(1)
        elif action == "reset_pose":
            self.reset_body_pose()
        elif action == "drive_mode":
            next_index = (self.drive_mode.currentIndex() + 1) % self.drive_mode.count()
            self.drive_mode.setCurrentIndex(next_index)
        else:
            self.send_action(action)

    def poll_gamepad(self):
        if not self.gamepad_available:
            self.update_gamepad_status()
            return

        try:
            pygame.event.pump()
            if self.gamepad is None or pygame.joystick.get_count() <= 0:
                if self.gamepad is not None:
                    self.handle_gamepad_disconnect()
                self.refresh_gamepad()
                self.update_gamepad_status()
                return

            if not self.gamepad_enabled:
                self.update_gamepad_status()
                return

            if self.arm_workflow.active:
                self.joystick.set_vector(0.0, 0.0)
                self.yaw_slider.setValue(0)
                for index in range(self.gamepad.get_numbuttons()):
                    pressed = bool(self.gamepad.get_button(index))
                    was_pressed = self.gamepad_buttons.get(index, False)
                    if (
                        pressed
                        and not was_pressed
                        and BUTTON_ACTIONS.get(index) == "stop"
                    ):
                        self.send_action("stop")
                    self.gamepad_buttons[index] = pressed
                self.update_gamepad_status()
                return

            left_x = self.axis_value(AXIS_LEFT_X)
            left_y = self.axis_value(AXIS_LEFT_Y)
            right_x = self.axis_value(AXIS_RIGHT_X)
            self.joystick.set_vector(-left_y, left_x)
            self.yaw_slider.setValue(int(-right_x * 100.0))

            for index in range(self.gamepad.get_numbuttons()):
                pressed = bool(self.gamepad.get_button(index))
                was_pressed = self.gamepad_buttons.get(index, False)
                if pressed and not was_pressed and index in BUTTON_ACTIONS:
                    self.handle_gamepad_action(BUTTON_ACTIONS[index])
                self.gamepad_buttons[index] = pressed

            if self.gamepad.get_numhats() > 0:
                hat_x, _hat_y = self.gamepad.get_hat(0)
                if hat_x != 0 and not self.gamepad_buttons.get("hat_x", False):
                    self.choose_gait_offset(1 if hat_x > 0 else -1)
                    self.gamepad_buttons["hat_x"] = True
                elif hat_x == 0:
                    self.gamepad_buttons["hat_x"] = False
        except pygame.error:
            self.handle_gamepad_disconnect()

        self.update_gamepad_status()

    def status_callback(self, message):
        if self.duplicate_stack_active:
            return
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        raw_state = str(status.get("state", "unknown")).strip().lower()
        self.motion_state = raw_state
        self.motion_moving = bool(
            status.get("motion_active", status.get("moving"))
        )
        self.motion_step_in_place = bool(status.get("step_in_place"))
        self.motion_arm_neutral_ready = bool(
            status.get("arm_neutral_ready", False)
        )
        self.motion_controller_connected = bool(status.get("controller_connected"))
        self.open_loop_hardware = bool(status.get("open_loop_hardware", False))
        self.hardware_mode = bool(status.get("hardware_mode", False))
        self.physical_tests_enabled = bool(
            status.get("physical_tests_enabled", False)
        )
        self.last_motion_status_time = time.monotonic()
        reported_profiles = status.get("real_tuning_profiles")
        if isinstance(reported_profiles, dict):
            self.real_profiles = merge_reported_real_profiles(
                self.real_profiles,
                reported_profiles,
                self.user_real_profile_names,
            )
            for name in self.real_profiles:
                if self.real_profile_combo.findText(name) < 0:
                    self.real_profile_combo.addItem(name)
        reported_profile = str(status.get("real_profile", "")).strip().upper()
        reported_tuning = status.get("real_tuning")
        if (
            isinstance(reported_tuning, dict)
            and (not self.real_tuning_initialized or not self.real_tuning_dirty)
        ):
            try:
                self.set_real_tuning_controls(
                    reported_tuning,
                    reported_profile or None,
                )
            except (RealProfileError, TypeError, ValueError):
                pass
        if isinstance(reported_tuning, dict) and not self.body_profile_hydrated:
            self.sync_live_body_pose_from_tuning(reported_tuning)
        if reported_profile:
            self.active_real_profile = reported_profile
        response_id = str(status.get("real_tuning_request_id", ""))
        if (
            self.pending_real_tuning_request
            and response_id == self.pending_real_tuning_request
        ):
            result = str(status.get("real_tuning_result", "")).lower()
            response_message = str(status.get("real_tuning_message", ""))
            if result == "applied":
                self.real_tuning_dirty = False
                if isinstance(reported_tuning, dict):
                    self.sync_live_body_pose_from_tuning(reported_tuning)
                self.real_tuning_status.setText(
                    "%s Controller applied %s."
                    % (response_message, reported_profile or "the profile")
                )
                self.real_tuning_status.setStyleSheet("color: #86efac;")
            elif result == "rejected":
                self.real_tuning_status.setText(
                    "Controller rejected the profile: %s" % response_message
                )
                self.real_tuning_status.setStyleSheet("color: #fca5a5;")
            self.pending_real_tuning_request = ""
        state = raw_state.replace("_", " ").upper()
        self.state_label.setText(state)
        route_connected = (
            "connected" if self.motion_controller_connected else "offline"
        )
        requested = str(
            status.get("requested_gait", status.get("gait", "unknown"))
        )
        active = str(status.get("active_gait", status.get("gait", "unknown")))
        pending = status.get("pending_gait")
        phase_index = status.get("phase_index", "?")
        phase_name = str(status.get("phase_name", "unknown"))
        phase_progress = float(status.get("phase_progress", 0.0))
        swing = list(status.get("swing_legs", []))
        stance = list(status.get("stance_legs", []))
        projected = status.get("projected_targets", [])
        if isinstance(projected, str):
            projected = [projected] if projected else []
        elif not isinstance(projected, (list, tuple)):
            projected = [str(projected)] if projected else []
        clamped = status.get("clamped_joints", [])
        if isinstance(clamped, str):
            clamped = [clamped] if clamped else []
        elif not isinstance(clamped, (list, tuple)):
            clamped = [str(clamped)] if clamped else []

        def status_number(name, default=0.0):
            try:
                return float(status.get(name, default))
            except (TypeError, ValueError):
                try:
                    return float(default)
                except (TypeError, ValueError):
                    return 0.0

        body_shift_x = status_number("body_shift_x")
        body_shift_y = status_number("body_shift_y")
        support_target_x = status_number(
            "body_shift_target_x",
            status.get("support_target_x", body_shift_x),
        )
        support_target_y = status_number(
            "body_shift_target_y",
            status.get("support_target_y", body_shift_y),
        )
        shift_completion = max(
            0.0,
            min(1.0, status_number("shift_completion")),
        )

        def status_boolean(name):
            value = status.get(name)
            if value in (True, 1, "1", "true", "True"):
                return "YES"
            if value in (False, 0, "0", "false", "False"):
                return "NO"
            return "UNKNOWN"

        support_valid = status_boolean("support_polygon_valid")
        lift_allowed = status_boolean("lift_allowed")
        limits = status.get("gait_limits")
        if isinstance(limits, dict):
            for gait_name, gait_limit in limits.items():
                if not isinstance(gait_limit, dict):
                    continue
                try:
                    self.gait_limits[gait_name] = (
                        float(
                            gait_limit.get(
                                "command_max_x",
                                gait_limit.get("max_x"),
                            )
                        ),
                        float(
                            gait_limit.get(
                                "command_max_y",
                                gait_limit.get("max_y"),
                            )
                        ),
                        float(
                            gait_limit.get(
                                "command_max_yaw",
                                gait_limit.get("max_yaw"),
                            )
                        ),
                    )
                except (KeyError, TypeError, ValueError):
                    continue
        if requested in self.gait_limits:
            self.current_gait = requested
            button = self.gait_button_by_name.get(requested)
            if button is not None:
                button.setChecked(True)

        def aggregate_diagnostic(value):
            if isinstance(value, dict):
                values = value.values()
            elif isinstance(value, (list, tuple)):
                values = value
            else:
                values = (value,)
            numeric = []
            for item in values:
                try:
                    number = float(item)
                except (TypeError, ValueError):
                    continue
                if math.isfinite(number):
                    numeric.append(number)
            return sum(numeric) if numeric else 0.0

        velocity_clamps = aggregate_diagnostic(
            status.get("joint_velocity_clamp_count", 0)
        )
        acceleration_clamps = aggregate_diagnostic(
            status.get("joint_acceleration_clamp_count", 0)
        )
        projection_count = aggregate_diagnostic(
            status.get("ik_projection_count", 0)
        )
        tracking_raw = status.get(
            "joint_tracking_error",
            status.get("joint_error"),
        )
        try:
            tracking_error = float(tracking_raw)
        except (TypeError, ValueError):
            tracking_error = float("nan")
        tracking_available = bool(
            status.get(
                "tracking_available",
                math.isfinite(tracking_error),
            )
        )
        tracking_assumed = bool(status.get("tracking_assumed", False))
        if tracking_assumed:
            tracking_text = "N/A (open-loop)"
        elif not tracking_available or not math.isfinite(tracking_error):
            tracking_text = "N/A (feedback unavailable)"
        else:
            tracking_text = "%.3f rad" % tracking_error
        arduino_frame_rate = status_number("arduino_frame_rate")
        cycle_period = status_number("cycle_period")
        swing = list(status.get("swing_legs", []))
        self.gait_diagnostics.setText(
            "Cycle period: %.2f s | Swing: %s\n"
            "Velocity clamps: %d | Acceleration clamps: %d | "
            "IK projections: %d\n"
            "Tracking error: %s | Arduino frames: %.1f Hz"
            % (
                cycle_period,
                " + ".join(swing) if swing else "none",
                int(velocity_clamps),
                int(acceleration_clamps),
                int(projection_count),
                tracking_text,
                arduino_frame_rate,
            )
        )

        error = status.get("joint_error")
        position_controller_text = (
            "not used (open-loop hardware)"
            if self.open_loop_hardware
            else (
                "connected"
                if self.position_controller_connected
                else "offline"
            )
        )
        detail = (
            "Command route: %s | position controller: %s | motion active: %s"
            % (
                route_connected,
                position_controller_text,
                "yes" if self.motion_moving else "no",
            )
        )
        if error is not None:
            detail += " | joint error %.3f rad" % float(error)
        detail += "\nIK projected targets: %s" % (
            ", ".join(projected) if projected else "none"
        )
        detail += " | clamped joints: %s" % (
            ", ".join(str(joint) for joint in clamped) if clamped else "none"
        )
        warning = status.get("warning")
        if warning or clamped:
            warning_text = str(warning) if warning else "joint command clamping"
            if clamped:
                warning_text += " [%s]" % ", ".join(str(joint) for joint in clamped)
            detail += "\nWARNING: " + warning_text
            self.status_detail.setStyleSheet("color: #fca5a5;")
        else:
            self.status_detail.setStyleSheet("color: #94a3b8;")
        self.status_detail.setText(detail)
        self.gait_detail.setText(
            "Requested: %s | Active: %s | Pending: %s\n"
            "Phase %s — %s: %.0f%% | Swing: %s | Stance: %s\n"
            "Body shift: (%+.3f, %+.3f) m → target (%+.3f, %+.3f) m | "
            "shift complete: %.0f%%\n"
            "Support valid: %s | Lift allowed: %s"
            % (
                GAIT_DISPLAY_NAMES.get(
                    requested,
                    requested.replace("_", " ").upper(),
                ),
                GAIT_DISPLAY_NAMES.get(
                    active,
                    active.replace("_", " ").upper(),
                ),
                (
                    GAIT_DISPLAY_NAMES.get(
                        str(pending),
                        str(pending).replace("_", " ").upper(),
                    )
                    if pending
                    else "NONE"
                ),
                phase_index,
                phase_name.replace("_", " ").upper(),
                100.0 * max(0.0, min(1.0, phase_progress)),
                ", ".join(swing) if swing else "none",
                ", ".join(stance) if stance else "none",
                body_shift_x,
                body_shift_y,
                support_target_x,
                support_target_y,
                100.0 * shift_completion,
                support_valid,
                lift_allowed,
            )
        )
        self.step_button.setChecked(self.motion_step_in_place)
        physical_active = bool(status.get("physical_test_active", False))
        physical_request_id = str(status.get("physical_test_request_id", ""))
        if self.active_physical_request is not None:
            expected_id = self.active_physical_request["request_id"]
            if physical_active and physical_request_id == expected_id:
                self.physical_test_was_active = True
                if bool(status.get("physical_test_settling", False)):
                    self.diagnostic_status.setText(
                        "Trajectory complete; waiting for conditioned joint "
                        "commands to settle at supported stand."
                    )
                else:
                    self.diagnostic_status.setText(
                        "Running %s — %.0f%% commanded trajectory complete."
                        % (
                            status.get("physical_test_mode", "diagnostic"),
                            100.0 * status_number("physical_test_progress"),
                        )
                    )
                self.diagnostic_status.setStyleSheet("color: #86efac;")
            elif self.physical_test_was_active and not physical_active:
                self.active_physical_request = None
                self.physical_test_was_active = False
                self.diagnostic_status.setText(
                    "Finite diagnostic completed and returned to commanded stand."
                )
                self.diagnostic_status.setStyleSheet("color: #86efac;")
            elif (
                not physical_active
                and time.monotonic()
                - self.active_physical_request["started_at"]
                > 2.0
                and str(status.get("warning", "")).startswith(
                    "Rejected physical test"
                )
            ):
                self.diagnostic_status.setText(str(status.get("warning")))
                self.diagnostic_status.setStyleSheet("color: #fca5a5;")
                self.active_physical_request = None
        self.update_emote_status(status)
        self.update_commanded_telemetry(status)
        self.last_face_motion_status = dict(status)
        self.apply_automatic_face(self.last_face_motion_status)

    def update_commanded_telemetry(self, status):
        """Render controller targets without implying physical feedback."""
        body = status.get("commanded_body_target", {})
        if not isinstance(body, dict):
            body = {}

        def finite(mapping, name, default=0.0):
            try:
                value = float(mapping.get(name, default))
            except (TypeError, ValueError):
                return float(default)
            return value if math.isfinite(value) else float(default)

        body_text = (
            "Body commanded: XYZ (%+.1f, %+.1f, %.1f) mm | "
            "RPY (%+.1f, %+.1f, %+.1f) deg"
            % (
                finite(body, "body_x") * 1000.0,
                finite(body, "body_y") * 1000.0,
                finite(body, "height", 0.2) * 1000.0,
                math.degrees(finite(body, "roll")),
                math.degrees(finite(body, "pitch")),
                math.degrees(finite(body, "yaw")),
            )
        )
        feet = status.get("commanded_foot_xyz", {})
        raw = status.get("raw_joint_target", [])
        filtered = status.get("filtered_joint_target", [])
        leg_lines = []
        for leg_index, leg in enumerate(LEG_ORDER):
            point = feet.get(leg, ()) if isinstance(feet, dict) else ()
            try:
                point_text = "(%+.0f,%+.0f,%+.0f)mm" % tuple(
                    float(value) * 1000.0 for value in point
                )
            except (TypeError, ValueError):
                point_text = "unavailable"
            start = leg_index * 3
            try:
                raw_degrees = [math.degrees(float(value)) for value in raw[start:start + 3]]
                filtered_degrees = [
                    math.degrees(float(value))
                    for value in filtered[start:start + 3]
                ]
                joint_text = "/".join(
                    "%.1f→%.1f" % pair
                    for pair in zip(raw_degrees, filtered_degrees)
                )
                if len(raw_degrees) != 3 or len(filtered_degrees) != 3:
                    joint_text = "unavailable"
            except (TypeError, ValueError):
                joint_text = "unavailable"
            phase = str(status.get("per_leg_phase", {}).get(leg, "-"))
            leg_state = "SWING" if leg in status.get("swing_legs", []) else "STANCE"
            leg_lines.append(
                "%s [%s phase %s] foot %s | joints raw→filtered ° %s"
                % (leg.replace("_", " ").title(), leg_state, phase, point_text, joint_text)
            )
        differences = status.get("raw_to_filtered_joint_error", {})
        maximum_difference = 0.0
        if isinstance(differences, dict):
            try:
                maximum_difference = max(
                    (abs(math.degrees(float(value))) for value in differences.values()),
                    default=0.0,
                )
            except (TypeError, ValueError):
                maximum_difference = 0.0
        serial_connected = self.last_serial_status_fields.get("connected", "0") == "1"
        footer = (
            "Profile %s | emote %s (%s) | max raw/filtered command "
            "difference %.2f° | ROS route %s | Arduino %s"
            % (
                status.get("real_profile", "unknown"),
                status.get("emote_name", "none") or "none",
                status.get("emote_state", "idle"),
                maximum_difference,
                "connected" if status.get("controller_connected") else "offline",
                "connected" if serial_connected else "offline/dry-run",
            )
        )
        self.commanded_telemetry.setText(
            "\n".join([body_text] + leg_lines + [footer])
        )

    def router_status_callback(self, message):
        if self.duplicate_stack_active:
            return
        fields = parse_key_value_status(message.data)
        owner = fields.get("owner", "UNKNOWN").strip().upper()
        if owner in ("MOTION", "MANUAL", "CALIBRATION", "HOLD", "DISABLED"):
            was_fresh_motion = (
                self.command_owner == "MOTION"
                and self.router_status_is_fresh()
            )
            self.command_owner = owner
            self.last_router_status_time = time.monotonic()
            if owner != "MOTION" or not was_fresh_motion:
                self.latch_motion_until_neutral()
        self.position_controller_connected = (
            fields.get("controller_connected") == "1"
        )
        self.router_pose_valid = fields.get("pose_valid") == "1"
        self.owner_state.setText("ACTIVE OWNER: %s" % self.command_owner)
        if self.command_owner == "MOTION":
            color = "#86efac"
        elif self.command_owner == "DISABLED":
            color = "#fca5a5"
        else:
            color = "#fbbf24"
        self.owner_state.setStyleSheet("color: %s;" % color)
        if self.last_face_motion_status:
            face_status = dict(self.last_face_motion_status)
            face_status["command_owner"] = self.command_owner
            self.last_face_motion_status = face_status
            self.apply_automatic_face(face_status)
        if self.arm_workflow.active:
            self.advance_arm_workflow()
        self.refresh_emote_controls(self.emote_busy())

    def serial_status_callback(self, message):
        if self.duplicate_stack_active:
            return
        fields = parse_key_value_status(message.data)
        self.last_serial_status_fields = dict(fields)
        self.last_serial_status_time = time.monotonic()
        self.refresh_firmware_link_health(fields)
        self.refresh_face_status(fields)
        if self.last_face_motion_status:
            face_status = dict(self.last_face_motion_status)
            for name in (
                "emergency_stop",
                "estop",
                "e_stop",
                "critical_fault",
                "fault",
                "low_voltage",
                "undervoltage",
            ):
                if name in fields:
                    face_status[name] = fields[name]
            self.apply_automatic_face(face_status)

        connected = fields.get("connected") == "1"
        ready = fields.get("ready") == "1"
        armed = fields.get("armed") == "1"
        streaming = fields.get("streaming") == "1"
        output_enabled = fields.get("output_enabled") == "1"
        dry_run = fields.get("dry_run") == "1"
        hardware_enabled = fields.get("hardware_enabled") == "1"
        calibration_valid = fields.get("calibration_valid") == "1"
        bridge_motion_safe = fields.get("motion_safe") == "1"
        bridge_owner = fields.get("owner", "UNKNOWN").strip().upper()
        bridge_owner_fresh = fields.get("owner_fresh") == "1"
        bridge_owner_allowed = fields.get("owner_allowed") == "1"
        pending = fields.get("pending", "-")
        waiting_for_ack = pending not in ("", "-")
        if not armed and self.arm_workflow.state == STATE_ARMED:
            self.arm_workflow.reset()
            self.set_arm_input_lock(False)
        ownership_safe_to_arm = (
            self.command_owner == "MOTION"
            and self.router_status_is_fresh()
            and bridge_owner == "MOTION"
            and bridge_owner_fresh
            and bridge_owner_allowed
        )
        motion_safe_to_arm = (
            self.motion_status_is_safe()
            and bridge_motion_safe
            and ownership_safe_to_arm
        )
        guided_arm_ready = self.arm_workflow.can_start(self.arm_snapshot())

        self.hardware_bridge.setText("ONLINE — status fresh")
        self.hardware_bridge.setStyleSheet("color: #86efac;")
        if not hardware_enabled:
            state = "HARDWARE MODE NOT ENABLED"
            color = "#fbbf24"
            device = (
                "CONNECTED — unused"
                if connected
                else "DISCONNECTED — hardware mode disabled"
            )
            output = "Locked (safe mode)"
            detail = (
                "Restart with start_serial_bridge:=true use_hardware:=true "
                "dry_run:=false to use the Arduino."
            )
        elif dry_run:
            state = "DRY RUN — NO SERVO OUTPUT"
            color = "#fbbf24"
            device = (
                "CONNECTED — unused"
                if connected
                else "DISCONNECTED — dry-run (expected)"
            )
            output = "Locked (dry run)"
            detail = (
                "The bridge is logging commands only. Restart with dry_run:=false "
                "when the supported robot is ready for a hardware test."
            )
        elif not calibration_valid:
            state = "CALIBRATION ERROR — ARM LOCKED"
            color = "#fca5a5"
            device = (
                "CONNECTED — READY"
                if ready
                else (
                    "CONNECTED — INITIALIZING"
                    if connected
                    else "DISCONNECTED"
                )
            )
            output = "Locked"
            detail = "Servo calibration is invalid. Correct it before arming."
        elif connected and ready and armed and streaming:
            state = "ARMED — LIVE SERVO OUTPUT"
            color = "#fb7185"
            device = "CONNECTED — READY"
            output = "Live frames"
            detail = (
                "Live hardware output is active. Use HOLD or DISARM before "
                "touching or lifting the robot."
            )
        elif connected and ready and armed:
            state = "MOTION BLOCKED — HARDWARE HOLD REQUIRED"
            color = "#fbbf24"
            device = "CONNECTED — READY"
            output = "Frames inhibited"
            if not ownership_safe_to_arm:
                detail = (
                    "ROS command ownership is not confirmed as fresh MOTION. "
                    "The bridge inhibits frames and sends firmware HOLD; ARM and "
                    "ROS ownership are independent safety layers."
                )
            elif waiting_for_ack:
                detail = "%s sent; waiting for firmware acknowledgement." % pending
            else:
                detail = "Firmware reports armed, but the bridge is blocking motion. Refresh status."
        elif self.arm_workflow.active:
            state = "GUIDED ARM — VERIFYING INTERLOCKS"
            color = "#fbbf24"
            device = "CONNECTED — READY"
            output = "Frames inhibited"
            detail = self.arm_workflow.reason
        elif connected and ready and pending == "ARM":
            state = "ARM REQUESTED — WAITING FOR ACK"
            color = "#fbbf24"
            device = "CONNECTED — READY"
            output = "Frames inhibited"
            detail = "ARM has not taken effect yet; servo frames remain safely blocked."
        elif connected and ready:
            device = "CONNECTED — READY"
            output = "Holding torque" if output_enabled else "Disarmed"
            if guided_arm_ready:
                state = "CONNECTED — READY FOR GUIDED ARM"
                color = "#86efac"
                detail = (
                    "Press ARM SYSTEM SAFELY. It will hold zero motion, request "
                    "MOTION ownership, verify a fresh valid 12-joint frame, and "
                    "then request Arduino ARM."
                )
            elif not ownership_safe_to_arm:
                state = "CONNECTED — WAITING FOR SAFE ARM CONDITIONS"
                color = "#fbbf24"
                detail = (
                    "The guided ARM button unlocks only for the fresh stopped "
                    "calibrated hardware WALK_POSE and an available command router."
                )
            elif motion_safe_to_arm:
                state = "CONNECTED — DISARMED"
                color = "#86efac"
                detail = (
                    "Arduino and a stable stopped pose are ready. Support the "
                    "robot and clear the legs before arming."
                )
            else:
                state = "CONNECTED — ARM LOCKED UNTIL CALIBRATED WALK POSE"
                color = "#fbbf24"
                detail = (
                    "Use Stand to reach the exact calibrated open-loop WALK_POSE. "
                    "Sitting and arbitrary HOLD poses remain ARM-locked."
                )
        elif connected:
            state = "CONNECTED — INITIALIZING"
            color = "#fbbf24"
            device = "CONNECTED — INITIALIZING FIRMWARE"
            output = "Disarmed"
            detail = "Serial is open, but the Arduino has not reported ready yet."
        else:
            state = "ARDUINO NOT CONNECTED"
            color = "#fca5a5"
            device = "DISCONNECTED"
            output = "Disarmed"
            detail = (
                "The bridge is searching automatically. Check the USB cable, "
                "serial port, permissions, and Arduino firmware."
            )

        error = fields.get("error")
        if error not in (None, "", "-"):
            detail += " Error: %s." % error.replace("_", " ")
        clamped = fields.get("clamped", "")
        if clamped not in ("", "-"):
            detail += " WARNING: servo output clamped for %s." % clamped.replace(
                ",",
                ", ",
            )
        if self.arm_workflow_notice:
            detail += " %s" % self.arm_workflow_notice

        device, device_color = arduino_connection_view(fields)
        face_sync_text, face_sync_color = face_host_sync_view(fields)
        self.hardware_state.setText(state)
        self.hardware_state.setStyleSheet("color: %s;" % color)
        self.hardware_device.setText(device)
        self.hardware_device.setStyleSheet(
            "color: %s; font-weight: 700;" % device_color
        )
        self.hardware_output.setText(output)
        self.hardware_output.setStyleSheet(
            "color: %s; font-weight: 700;" % ("#fb7185" if streaming else "#aebdd0")
        )
        self.hardware_face_sync.setText(face_sync_text)
        self.hardware_face_sync.setStyleSheet(
            "color: %s; font-weight: 700;" % face_sync_color
        )
        self.hardware_detail.setText(detail)
        self.refresh_arm_button()
        if self.arm_workflow.active:
            self.advance_arm_workflow()

    def arm_snapshot(self, now=None):
        if now is None:
            now = time.monotonic()
        fields = self.last_serial_status_fields

        def flag(name, default=False):
            fallback = "1" if default else "0"
            return fields.get(name, fallback) == "1"

        try:
            bridge_frame_seq = int(fields.get("frame_seq", "-1"))
        except (TypeError, ValueError):
            bridge_frame_seq = -1

        return ArmSnapshot(
            now=now,
            motion_status_time=self.last_motion_status_time,
            motion_state=self.motion_state,
            motion_moving=self.motion_moving,
            motion_step_in_place=self.motion_step_in_place,
            motion_arm_neutral_ready=self.motion_arm_neutral_ready,
            motion_controller_connected=self.motion_controller_connected,
            router_status_time=self.last_router_status_time,
            router_owner=self.command_owner,
            router_pose_valid=self.router_pose_valid,
            serial_status_time=self.last_serial_status_time,
            hardware_enabled=flag("hardware_enabled"),
            dry_run=flag("dry_run", default=True),
            calibration_valid=flag("calibration_valid"),
            connected=flag("connected"),
            ready=flag("ready"),
            armed=flag("armed"),
            streaming=flag("streaming"),
            pending=fields.get("pending", ""),
            bridge_motion_safe=flag("motion_safe"),
            bridge_owner=fields.get("owner", "UNKNOWN"),
            bridge_owner_fresh=flag("owner_fresh"),
            bridge_owner_allowed=flag("owner_allowed"),
            bridge_frame_ready=flag("frame_ready"),
            bridge_frame_seq=bridge_frame_seq,
        )

    def dispatch_arm_effects(
        self,
        effects,
        owner_override=None,
        serial_override=None,
    ):
        for effect in effects:
            if effect == EFFECT_ZERO_STOP:
                self.latch_motion_until_neutral()
                self.stop_motion_controls()
            elif effect == EFFECT_OWNER_MOTION:
                self.motion_ownership_requested = True
                self.last_owner_heartbeat = time.monotonic()
                self.ros_node.set_command_owner("MOTION")
            elif effect == EFFECT_OWNER_HOLD:
                self.motion_ownership_requested = False
                self.ros_node.set_command_owner(owner_override or "HOLD")
            elif effect == EFFECT_SERIAL_STATUS:
                self.send_serial_command("STATUS")
            elif effect == EFFECT_SERIAL_ARM:
                self.send_serial_command("ARM")
            elif effect == EFFECT_SERIAL_HOLD:
                self.send_serial_command(serial_override or "HOLD")

    def update_arm_readiness_display(self, snapshot=None):
        if snapshot is None:
            snapshot = self.arm_snapshot()
        view = arm_readiness_view(
            self.arm_workflow,
            snapshot,
            duplicate_stack_active=self.duplicate_stack_active,
        )
        self.arm_readiness_state.setText(view["title"])
        self.arm_readiness_state.setStyleSheet(
            "color: %s;" % view["color"]
        )
        self.arm_blockers_label.setText(view["detail"])
        self.fit_arm_readiness_text()
        return view

    def fit_arm_readiness_text(self):
        """Keep the full blocker list visible inside the scrolling page."""
        label = self.arm_blockers_label
        if (
            not hasattr(label, "heightForWidth")
            or not hasattr(self, "control_page")
        ):
            return
        fit_key = (label.text(), int(label.width()))
        if getattr(self, "_arm_readiness_fit_key", None) == fit_key:
            return
        self._arm_readiness_fit_key = fit_key
        label.setMinimumHeight(0)
        self.control_page.setMinimumHeight(0)
        width = max(1, int(label.width()))
        required_height = max(1, int(label.heightForWidth(width)))
        label.setMinimumHeight(required_height)
        layout = self.control_page.layout()
        if layout is not None:
            layout.invalidate()
            layout.activate()
        self.control_page.setMinimumHeight(
            self.control_page.minimumSizeHint().height()
        )

    def refresh_arm_button(self):
        snapshot = self.arm_snapshot()
        view = self.update_arm_readiness_display(snapshot)
        if self.duplicate_stack_active:
            self.hardware_arm_button.setText("DUPLICATE VOLT STACK — ARM LOCKED")
            self.hardware_arm_button.setEnabled(False)
            self.hardware_arm_button.setToolTip(
                "Stop the extra VOLT simulation or hardware launch. Exactly one "
                "controller, router, bridge, and GUI stack may run."
            )
            return
        if self.arm_workflow.active:
            self.hardware_arm_button.setText("CANCEL ARM / HOLD")
            self.hardware_arm_button.setEnabled(True)
            self.hardware_arm_button.setToolTip(
                "Cancel immediately, publish zero/STOP, return ROS ownership "
                "to HOLD, and send firmware HOLD."
            )
            return
        if snapshot.armed or self.arm_workflow.state == STATE_ARMED:
            serial_fresh = self.arm_workflow.serial_is_fresh(snapshot)
            if snapshot.armed and snapshot.streaming and serial_fresh:
                button_text = "SYSTEM ARMED"
                tooltip = (
                    "Live hardware is armed. Use HOLD SERVOS or DISARM ARDUINO."
                )
            elif not serial_fresh:
                button_text = "ARM STATE UNKNOWN — STATUS STALE"
                tooltip = (
                    "The last report was armed, but serial status is stale. "
                    "Use HOLD or DISARM and verify the bridge."
                )
            else:
                button_text = "ARDUINO ARMED — STREAM BLOCKED"
                tooltip = (
                    "Firmware reports armed without live streaming. Use HOLD "
                    "or DISARM before troubleshooting."
                )
            self.hardware_arm_button.setText(button_text)
            self.hardware_arm_button.setEnabled(False)
            self.hardware_arm_button.setToolTip(tooltip)
            return

        blockers = self.arm_workflow.start_blockers(snapshot)
        self.hardware_arm_button.setEnabled(not blockers)
        if blockers:
            count = len(view["blockers"])
            noun = "BLOCKER" if count == 1 else "BLOCKERS"
            self.hardware_arm_button.setText(
                "ARM LOCKED — %d %s" % (count, noun)
            )
            self.hardware_arm_button.setToolTip(
                "Locked until every listed condition is resolved:\n%s"
                % "\n".join("• %s" % item for item in view["blockers"])
            )
        else:
            self.hardware_arm_button.setText("ARM SYSTEM SAFELY")
            self.hardware_arm_button.setToolTip(
                "Guided STOP, fresh MOTION ownership, valid-frame verification, "
                "then one Arduino ARM request."
            )

    def arm_system_safely(self):
        if self.arm_workflow.active:
            self.cancel_arm_workflow("Arming cancelled by the operator.")
            return
        if self.shutting_down or not rclpy.ok():
            return
        if self.duplicate_stack_active:
            self.refresh_arm_button()
            return

        snapshot = self.arm_snapshot()
        blockers = self.arm_workflow.start_blockers(snapshot)
        if blockers:
            self.arm_workflow_notice = "ARM blocked: %s." % blockers[0]
            self.refresh_arm_button()
            return

        answer = QMessageBox.warning(
            self,
            "Confirm physical robot ARM",
            (
                "This can enable live servo movement.\n\n"
                "Confirm that the robot is supported off the floor, all legs "
                "are clear, the calibrated WALK_POSE matches the mechanism, "
                "and the servo-power disconnect is within reach.\n\n"
                "Continue with guided ARM?"
            ),
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return

        snapshot = self.arm_snapshot()
        blockers = self.arm_workflow.start_blockers(snapshot)
        if blockers:
            self.arm_workflow_notice = (
                "ARM cancelled after confirmation: %s." % blockers[0]
            )
            self.refresh_arm_button()
            return

        if not self.arm_workflow.active:
            self.arm_workflow.reset()
        effects = self.arm_workflow.start(snapshot)
        self.arm_workflow_notice = self.arm_workflow.reason
        self.set_arm_input_lock(self.arm_workflow.active)
        self.dispatch_arm_effects(effects)
        if self.arm_workflow.active:
            self.arm_workflow_timer.start()
        self.refresh_arm_button()

    def advance_arm_workflow(self):
        if not self.arm_workflow.active:
            self.arm_workflow_timer.stop()
            self.set_arm_input_lock(False)
            self.refresh_arm_button()
            return
        self.set_arm_input_lock(True)
        effects = self.arm_workflow.update(self.arm_snapshot())
        self.arm_workflow_notice = self.arm_workflow.reason
        self.dispatch_arm_effects(effects)
        if not self.arm_workflow.active:
            self.arm_workflow_timer.stop()
            self.set_arm_input_lock(False)
        self.refresh_arm_button()

    def cancel_arm_workflow(
        self,
        reason,
        owner_override=None,
        serial_override=None,
    ):
        effects = self.arm_workflow.cancel(reason)
        self.arm_workflow_notice = self.arm_workflow.reason
        self.dispatch_arm_effects(
            effects,
            owner_override=owner_override,
            serial_override=serial_override,
        )
        self.arm_workflow_timer.stop()
        self.set_arm_input_lock(False)
        self.refresh_arm_button()

    def refresh_arm_status_freshness(self):
        """Age out displayed bridge readiness even when no callback arrives."""
        if not self.refresh_stack_topology():
            return
        self.refresh_arm_button()
        self.expire_emote_requests_without_status()
        self.refresh_emote_controls(self.emote_busy())
        if self.last_serial_status_time <= 0.0:
            return
        age = time.monotonic() - self.last_serial_status_time
        if 0.0 <= age <= SERIAL_STATUS_TIMEOUT:
            return
        self.hardware_bridge.setText("Stale")
        self.hardware_bridge.setStyleSheet("color: #fca5a5;")
        self.hardware_state.setText("SERIAL STATUS STALE — ARM LOCKED")
        self.hardware_state.setStyleSheet("color: #fca5a5;")
        self.hardware_device.setText("UNKNOWN — serial status stale")
        self.hardware_device.setStyleSheet(
            "color: #fca5a5; font-weight: 700;"
        )
        self.hardware_output.setText("Unknown / inhibited")
        self.hardware_output.setStyleSheet("color: #fca5a5; font-weight: 700;")
        self.hardware_face_sync.setText("UNKNOWN — serial status stale")
        self.hardware_face_sync.setStyleSheet(
            "color: #fca5a5; font-weight: 700;"
        )
        self.hardware_detail.setText(
            "No recent /volt/serial_status update. Guided ARM is locked. "
            "Use HOLD or DISARM, then verify the bridge and ROS connection."
        )
        if hasattr(self, "face_status"):
            self.face_status.setText("Face LED status is stale; connection unknown.")
            self.face_status.setStyleSheet("color: #fca5a5;")

    def motion_status_is_safe(self):
        status_age = time.monotonic() - self.last_motion_status_time
        return motion_status_allows_arm(
            self.motion_state,
            self.motion_moving,
            self.motion_step_in_place,
            self.motion_controller_connected,
            status_age,
            MOTION_STATUS_TIMEOUT,
            self.motion_arm_neutral_ready,
        )

    def refresh_stack_topology(self):
        """Lock all motion mutations when status has multiple authorities."""
        try:
            duplicates = duplicate_stack_topics(
                self.ros_node.critical_publisher_counts()
            )
        except Exception as exc:
            duplicates = ("ROS graph inspection failed: %s" % exc,)

        if duplicates:
            first_detection = not self.duplicate_stack_active
            self.duplicate_stack_active = True
            self.duplicate_stack_topics = tuple(duplicates)
            if first_detection:
                if self.arm_workflow.active:
                    self.cancel_arm_workflow(
                        "Duplicate VOLT stacks detected; ARM cancelled."
                    )
                else:
                    self.return_both_layers_to_hold(
                        "Duplicate VOLT stacks detected; both safety layers "
                        "returned to HOLD."
                    )
                self.last_motion_status_time = 0.0
                self.last_router_status_time = 0.0
                self.last_serial_status_time = 0.0
                self.last_serial_status_fields = {}
            self.set_arm_input_lock(True)
            topic_text = ", ".join(self.duplicate_stack_topics)
            detail = (
                "More than one VOLT stack is publishing control status (%s). "
                "STOP/HOLD was sent and ARM is locked. Close the extra simulation "
                "or hardware launch, then wait for fresh single-stack status."
                % topic_text
            )
            self.state_label.setText("STACK CONFLICT")
            self.status_detail.setText(detail)
            self.status_detail.setStyleSheet("color: #fca5a5;")
            self.owner_state.setText("ACTIVE OWNER: CONFLICT — HOLD REQUESTED")
            self.owner_state.setStyleSheet("color: #fca5a5;")
            self.hardware_bridge.setText("Conflict")
            self.hardware_bridge.setStyleSheet("color: #fca5a5;")
            self.hardware_state.setText("DUPLICATE VOLT STACK — ARM LOCKED")
            self.hardware_state.setStyleSheet("color: #fca5a5;")
            self.hardware_device.setText("Unknown — conflicting reports")
            self.hardware_output.setText("HOLD requested")
            self.hardware_output.setStyleSheet(
                "color: #fca5a5; font-weight: 700;"
            )
            self.hardware_detail.setText(detail)
            self.refresh_arm_button()
            return False

        if self.duplicate_stack_active:
            self.duplicate_stack_active = False
            self.duplicate_stack_topics = ()
            self.set_arm_input_lock(False)
            self.last_motion_status_time = 0.0
            self.last_router_status_time = 0.0
            self.last_serial_status_time = 0.0
            self.last_serial_status_fields = {}
            self.motion_state = "unknown"
            self.motion_moving = True
            self.motion_arm_neutral_ready = False
            self.motion_controller_connected = False
            self.command_owner = "UNKNOWN"
            self.state_label.setText("WAITING")
            self.status_detail.setText(
                "Duplicate stack cleared; waiting for fresh single-stack status."
            )
            self.status_detail.setStyleSheet("color: #fbbf24;")
            self.owner_state.setText("ACTIVE OWNER: UNKNOWN")
            self.hardware_state.setText("WAITING FOR FRESH SERIAL STATUS")
            self.hardware_detail.setText(
                "The conflict cleared. ARM remains locked until every fresh "
                "single-stack interlock is confirmed."
            )
            self.refresh_arm_button()
        return True

    def motion_status_is_fresh(self, now=None):
        if now is None:
            now = time.monotonic()
        if self.last_motion_status_time <= 0.0:
            return False
        age = now - self.last_motion_status_time
        return 0.0 <= age <= MOTION_STATUS_TIMEOUT

    def router_status_is_fresh(self, now=None):
        if now is None:
            now = time.monotonic()
        if self.last_router_status_time <= 0.0:
            return False
        age = now - self.last_router_status_time
        return 0.0 <= age <= ROUTER_STATUS_TIMEOUT

    def spin_ros(self):
        if self.shutting_down or not rclpy.ok():
            return
        try:
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)
        except Exception:
            if self.shutting_down or not rclpy.ok():
                return
            raise

    def stop_timers(self):
        for timer in (
            self.spin_timer,
            self.command_timer,
            self.pose_timer,
            self.gamepad_timer,
            self.arm_workflow_timer,
            self.focus_hold_timer,
            self.arm_status_timer,
            self.diagnostic_timer,
            self.face_test_timer,
        ):
            timer.stop()

    def publish_shutdown_stop(self):
        if not rclpy.ok():
            return
        stop_action = String()
        stop_action.data = "stop"
        hold_owner = String()
        hold_owner.data = "HOLD"
        serial_hold = String()
        serial_hold.data = "HOLD"
        face_shutdown = String()
        face_shutdown.data = "shutdown"
        zero_velocity = Twist()
        try:
            self.ros_node.velocity_publisher.publish(zero_velocity)
            self.ros_node.action_publisher.publish(stop_action)
            self.ros_node.owner_publisher.publish(hold_owner)
            self.ros_node.serial_command_publisher.publish(serial_hold)
            self.ros_node.face_expression_publisher.publish(face_shutdown)
        except Exception as exc:
            self.ros_node.get_logger().warning(
                "Could not publish complete GUI shutdown HOLD sequence: %s" % exc
            )

    def shutdown(self):
        if self.shutting_down:
            return
        self.stop_timers()
        if self.arm_workflow.active:
            self.arm_workflow.cancel("Arming cancelled because the GUI is closing.")
        self.forward = 0.0
        self.horizontal = 0.0
        if self.face_catalog is not None and hasattr(self, "face_enable"):
            try:
                self.face_settings = self.current_face_settings()
                self.persist_face_settings()
            except (FaceConfigError, RuntimeError):
                pass
        self.publish_shutdown_stop()
        self.shutting_down = True
        if self.gamepad_available:
            pygame.joystick.quit()
            pygame.quit()
        try:
            self.ros_node.destroy_node()
        except Exception:
            pass

    def closeEvent(self, event):
        self.shutdown()
        event.accept()


def acquire_gui_instance_lock():
    runtime_root = os.environ.get("XDG_RUNTIME_DIR", "").strip()
    if not runtime_root or not os.path.isdir(runtime_root):
        runtime_root = "/tmp"
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    safe_domain = "".join(
        character if character.isalnum() or character in ("-", "_") else "_"
        for character in domain
    )
    lock_path = os.path.join(
        runtime_root,
        "volt-control-gui-%d-domain-%s.lock" % (os.getuid(), safe_domain),
    )
    lock_handle = open(lock_path, "a+", encoding="utf-8")
    try:
        fcntl.flock(
            lock_handle.fileno(),
            fcntl.LOCK_EX | fcntl.LOCK_NB,
        )
    except BlockingIOError:
        lock_handle.close()
        raise
    lock_handle.seek(0)
    lock_handle.truncate()
    lock_handle.write("%d\n" % os.getpid())
    lock_handle.flush()
    return lock_handle


def main():
    try:
        gui_lock = acquire_gui_instance_lock()
    except BlockingIOError:
        print(
            "ERROR: a VOLT control GUI is already running in this ROS domain. "
            "Use the existing window and stop the extra launch.",
            file=sys.stderr,
            flush=True,
        )
        return
    rclpy.init()
    application = QApplication(sys.argv)
    application.setFont(QFont("DejaVu Sans", 10))
    window = VoltControlWindow()
    signal.signal(signal.SIGINT, lambda *_args: window.close())
    window.show()
    try:
        exit_code = application.exec_()
    except KeyboardInterrupt:
        window.close()
        exit_code = 130
    if rclpy.ok():
        rclpy.shutdown()
    gui_lock.close()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
