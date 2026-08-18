#!/usr/bin/env python3

import json
import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from volt_gait_controller import (
    GAITS,
    VoltGaitController,
    apply_real_tuning_to_configs,
    canonical_gait_name,
    default_gait_config_path,
    limit_velocity_command,
    load_gait_configs,
    normalized_velocity_activity,
    validate_fast_trot_tuning,
)
from volt_kinematics import (
    JOINT_NAMES,
    JOINT_VELOCITY_LIMITS,
    FOOT_LIMIT,
    LEG_FOOT_MID_POSE,
    LEG_FOOT_SIT_POSE,
    LEG_ORDER,
    KinematicsError,
    NOMINAL_FEET,
    LEG_LIMIT,
    SHOULDER_LIMIT,
    SIT_POSE,
    WALK_POSE,
    clamp,
    feet_to_joint_positions_diagnostic,
    interpolate,
    joint_positions_to_feet,
)
from volt_physical_tests import (
    CARTESIAN_TEST_MODES,
    MAX_TEST_JOINT_SPEED,
    PhysicalTestError,
    cartesian_frame_at,
    physical_test_request_payload,
)
from volt_emote_engine import (
    CartesianEmoteEngine,
    EmoteStateError,
    EmoteValidationError,
    default_emote_config_path,
    load_builtin_catalog,
    validate_options,
)
from volt_real_profiles import (
    NUMERIC_BOUNDS,
    RealProfileError,
    default_profile_path,
    load_profiles,
    smoothing_alpha,
    validate_tuning,
)


# Default hardware gaits retain the conservative 30 deg/s cap. Physical
# fast_trot has its own validated 110 deg/s cap below the current 120 deg/s
# firmware source; its feedback-governed phase clock prevents that exception
# from turning into contact-phase lag.
SIMULATION_JOINT_VELOCITY_LIMIT = math.radians(118.0)
HARDWARE_JOINT_VELOCITY_LIMIT = math.radians(30.0)
DEFAULT_JOINT_ACCELERATION_LIMIT = 18.0
SIMULATION_FAST_TROT_JOINT_ACCELERATION_LIMIT = 60.0
COMMAND_OWNERS = {
    "MOTION",
    "MANUAL",
    "CALIBRATION",
    "HOLD",
    "DISABLED",
}
VELOCITY_GATE_OPEN = "open"
VELOCITY_GATE_AWAIT_NEUTRAL = "await_neutral"
VELOCITY_GATE_AWAIT_MOTION = "await_motion"
OPEN_LOOP_WARNING = (
    "OPEN-LOOP HARDWARE: assuming firmware-safe WALK_POSE without "
    "measured joint feedback."
)
ARM_NEUTRAL_TOLERANCE = math.radians(0.5)
ARM_NEUTRAL_ZERO_TOLERANCE = 1e-4
FINITE_MOTION_POSITION_TOLERANCE = math.radians(0.5)
FINITE_MOTION_VELOCITY_TOLERANCE = math.radians(1.0)
BODY_TARGET_FIELDS = (
    "height",
    "body_x",
    "body_y",
    "roll",
    "pitch",
    "yaw",
)


def initial_motion_state(open_loop_hardware):
    """Return an independent, finite startup command state."""
    if open_loop_hardware:
        return (
            "standing",
            list(WALK_POSE),
            [0.0 for _ in JOINT_NAMES],
            OPEN_LOOP_WARNING,
        )
    return "waiting", None, None, ""


class VoltMotionController(Node):
    def __init__(self):
        super().__init__("volt_motion_controller")

        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("command_timeout", 0.60)
        self.declare_parameter("step_keepalive_timeout", 1.0)
        self.declare_parameter("joint_error_limit", 0.45)
        self.declare_parameter("support_joint_error_limit", 0.15)
        self.declare_parameter("support_command_error_limit", 0.08)
        self.declare_parameter("support_feedback_timeout", 0.25)
        self.declare_parameter("minimum_workspace_margin", 0.010)
        self.declare_parameter("stand_duration", 4.0)
        self.declare_parameter("sit_duration", 4.7)
        self.declare_parameter("natural_sit_height", 0.145)
        self.declare_parameter("natural_sit_rearward_shift", 0.020)
        self.declare_parameter("natural_sit_pitch_deg", -10.0)
        self.declare_parameter("auto_ready_pose", False)
        self.declare_parameter("body_height", 0.200)
        self.declare_parameter("debug_gait", False)
        self.declare_parameter("diagnostic_log_rate", 1.0)
        self.declare_parameter("sudden_joint_jump_deg", 10.0)
        self.declare_parameter("max_joint_velocity", 4.0)
        self.declare_parameter("max_joint_acceleration", 60.0)
        self.declare_parameter("joint_smoothing_alpha", 0.12)
        # Deprecated names remain accepted for older launch/config files.
        self.declare_parameter("joint_smoothing_factor", -1.0)
        self.declare_parameter("smoothing_alpha", -1.0)
        self.declare_parameter(
            "gait_config_file",
            str(default_gait_config_path()),
        )
        self.declare_parameter("physical_fast_trot_config_file", "")
        self.declare_parameter(
            "real_robot_profiles_file",
            str(default_profile_path()),
        )
        self.declare_parameter(
            "emote_config_file",
            str(default_emote_config_path()),
        )
        self.declare_parameter("hardware_mode", False)
        self.declare_parameter("open_loop_hardware", False)
        self.declare_parameter("enable_physical_tests", False)
        self.declare_parameter("physical_test_keepalive_timeout", 0.75)
        self.declare_parameter("emote_keepalive_timeout", 0.75)

        self.control_rate = float(self.get_parameter("control_rate").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.step_keepalive_timeout = max(
            0.25,
            float(self.get_parameter("step_keepalive_timeout").value),
        )
        self.joint_error_limit = float(
            self.get_parameter("joint_error_limit").value
        )
        self.support_joint_error_limit = float(
            self.get_parameter("support_joint_error_limit").value
        )
        self.support_command_error_limit = float(
            self.get_parameter("support_command_error_limit").value
        )
        self.support_feedback_timeout = float(
            self.get_parameter("support_feedback_timeout").value
        )
        self.minimum_workspace_margin = float(
            self.get_parameter("minimum_workspace_margin").value
        )
        self.natural_sit_height = clamp(
            float(self.get_parameter("natural_sit_height").value),
            0.135,
            0.170,
        )
        self.natural_sit_rearward_shift = clamp(
            float(self.get_parameter("natural_sit_rearward_shift").value),
            0.005,
            0.025,
        )
        self.natural_sit_pitch = math.radians(clamp(
            float(self.get_parameter("natural_sit_pitch_deg").value),
            -20.0,
            -8.0,
        ))
        self.debug_gait = bool(self.get_parameter("debug_gait").value)
        self.diagnostic_log_rate = clamp(
            float(self.get_parameter("diagnostic_log_rate").value),
            0.1,
            5.0,
        )
        self.sudden_joint_jump_deg = clamp(
            float(self.get_parameter("sudden_joint_jump_deg").value),
            2.0,
            30.0,
        )
        self.max_joint_velocity = float(
            self.get_parameter("max_joint_velocity").value
        )
        self.max_joint_acceleration = float(
            self.get_parameter("max_joint_acceleration").value
        )
        configured_smoothing_alpha = float(
            self.get_parameter("joint_smoothing_alpha").value
        )
        for legacy_name in ("joint_smoothing_factor", "smoothing_alpha"):
            legacy_value = float(self.get_parameter(legacy_name).value)
            if legacy_value >= 0.0:
                configured_smoothing_alpha = legacy_value
                self.get_logger().warning(
                    "Parameter '%s' is deprecated; use joint_smoothing_alpha."
                    % legacy_name
                )
        self.joint_smoothing_alpha = clamp(
            configured_smoothing_alpha,
            0.02,
            1.0,
        )
        # Keep the old attribute for pure callers and downstream compatibility.
        self.joint_smoothing_factor = self.joint_smoothing_alpha
        self.hardware_mode = bool(self.get_parameter("hardware_mode").value)
        self.open_loop_hardware = bool(
            self.get_parameter("open_loop_hardware").value
        )
        self.use_sim_time = bool(
            self.get_parameter("use_sim_time").value
        )
        self.enable_physical_tests = bool(
            self.get_parameter("enable_physical_tests").value
        )
        self.physical_test_keepalive_timeout = clamp(
            float(
                self.get_parameter(
                    "physical_test_keepalive_timeout"
                ).value
            ),
            0.30,
            2.0,
        )
        self.emote_keepalive_timeout = clamp(
            float(self.get_parameter("emote_keepalive_timeout").value),
            0.30,
            2.0,
        )
        if self.open_loop_hardware and not self.hardware_mode:
            raise ValueError(
                "open_loop_hardware is only valid when hardware_mode is true"
            )
        if (
            self.hardware_mode
            and self.use_sim_time
        ):
            raise ValueError(
                "hardware_mode requires use_sim_time:=false so STOP, "
                "keepalive, and physical-test deadlines cannot pause with /clock"
            )
        gait_config_file = str(self.get_parameter("gait_config_file").value)
        physical_fast_trot_config_file = str(
            self.get_parameter("physical_fast_trot_config_file").value
        ).strip()
        if self.hardware_mode and not physical_fast_trot_config_file:
            raise ValueError(
                "hardware_mode requires a dedicated "
                "physical_fast_trot_config_file"
            )
        selected_fast_trot_file = (
            physical_fast_trot_config_file
            if self.hardware_mode and physical_fast_trot_config_file
            else gait_config_file
        )
        try:
            self.gait_configs = load_gait_configs(
                gait_config_file,
                selected_fast_trot_file,
            )
        except (OSError, ValueError) as exc:
            self.get_logger().fatal("Invalid gait configuration: %s" % exc)
            raise
        self.gait_config_file = gait_config_file
        self.fast_trot_config_file = selected_fast_trot_file
        self.real_profiles_file = str(
            self.get_parameter("real_robot_profiles_file").value
        ).strip()
        try:
            self.real_profiles = load_profiles(
                self.real_profiles_file,
                include_user=False,
            )
        except (OSError, RealProfileError, ValueError) as exc:
            self.get_logger().fatal("Invalid real-robot profiles: %s" % exc)
            raise
        self.active_real_profile = (
            "REAL_DIAGNOSTIC" if self.hardware_mode else "SIMULATION"
        )
        self.applied_real_tuning = dict(
            self.real_profiles[self.active_real_profile]
        )
        if self.hardware_mode:
            self.gait_configs = apply_real_tuning_to_configs(
                self.gait_configs,
                self.applied_real_tuning,
            )
            self.max_joint_velocity = math.radians(
                self.applied_real_tuning["max_joint_velocity_deg_s"]
            )
            self.max_joint_acceleration = math.radians(
                self.applied_real_tuning["max_joint_acceleration_deg_s2"]
            )
            self.joint_smoothing_alpha = smoothing_alpha(
                self.applied_real_tuning
            )
            self.joint_smoothing_factor = self.joint_smoothing_alpha
        self.real_tuning_request_id = "startup"
        self.real_tuning_result = "applied"
        self.real_tuning_message = "Loaded %s." % self.active_real_profile
        self.emote_config_file = str(
            self.get_parameter("emote_config_file").value
        ).strip()
        try:
            self.emote_catalog = load_builtin_catalog(
                self.emote_config_file,
                preflight=True,
            )
        except (OSError, EmoteValidationError, ValueError) as exc:
            self.get_logger().fatal("Invalid Cartesian emote catalog: %s" % exc)
            raise
        self.emote_engine = CartesianEmoteEngine(self.emote_catalog)

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            "/volt/joint_commands/motion",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/volt/status",
            10,
        )
        self.create_subscription(Twist, "/cmd_vel", self.velocity_callback, 10)
        self.create_subscription(String, "/volt/action", self.action_callback, 10)
        self.create_subscription(String, "/volt/gait", self.gait_callback, 10)
        self.create_subscription(
            String,
            "/volt/fast_trot_tuning",
            self.fast_trot_tuning_callback,
            10,
        )
        self.create_subscription(
            String,
            "/volt/real_robot_tuning",
            self.real_robot_tuning_callback,
            10,
        )
        self.create_subscription(
            String,
            "/volt/serial_status",
            self.serial_status_callback,
            10,
        )
        self.create_subscription(
            Twist,
            "/volt/body_pose",
            self.body_pose_callback,
            10,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            20,
        )
        self.create_subscription(
            String,
            "/volt/command_router_status",
            self.command_router_status_callback,
            10,
        )
        self.create_subscription(
            String,
            "/volt/physical_test",
            self.physical_test_callback,
            10,
        )
        self.create_subscription(
            String,
            "/volt/emote",
            self.emote_callback,
            10,
        )

        (
            self.state,
            initial_commanded_positions,
            initial_commanded_velocities,
            self.open_loop_warning,
        ) = initial_motion_state(self.open_loop_hardware)
        self.command_owner = "UNKNOWN"
        self.gait_name = (
            self.applied_real_tuning["gait"]
            if self.hardware_mode
            else canonical_gait_name("walk")
        )
        self.requested_gait = self.gait_name
        self.step_in_place = False
        self.last_step_keepalive_time = self.now_seconds()
        self.requested_velocity = [0.0, 0.0, 0.0]
        self.filtered_velocity = [0.0, 0.0, 0.0]
        self.last_velocity_time = self.now_seconds()
        self.last_update_time = self.now_seconds()
        self.expected_control_period = 1.0 / max(self.control_rate, 1.0)
        self.control_loop_rate_hz = 0.0
        self.command_publish_rate_hz = 0.0
        self.control_loop_dt_s = self.expected_control_period
        self.control_loop_max_dt_s = self.expected_control_period
        self.missed_deadlines = 0
        self.control_loop_window_start = self.last_update_time
        self.control_loop_window_count = 0
        self.command_publish_window_start = self.last_update_time
        self.command_publish_window_count = 0
        self.motion_active = False
        self.pending_gait = None
        self.gait_controller = VoltGaitController(
            self.gait_configs,
            hardware_mode=self.hardware_mode,
        )
        self.gait_controller.set_gait(
            self.gait_name,
            self.now_seconds(),
        )
        self.velocity_command_sequence = 0
        self.resume_after_velocity_sequence = -1
        self.velocity_gate_state = VELOCITY_GATE_OPEN
        self.pending_pose_action = None
        self.natural_sit_plan = None
        self.physical_test = None
        self.pending_emote_request = None
        self.active_emote_request = None
        self.emote_request_id = ""
        self.emote_result = "idle"
        self.emote_message = "No emote requested."
        self.emote_progress = 0.0
        self.emote_cancelled = False
        self.emote_filter_settling = False
        self.emote_base_feet = {
            leg: tuple(NOMINAL_FEET[leg]) for leg in LEG_ORDER
        }
        self.emote_base_body = {
            "height": 0.200,
            "body_x": 0.0,
            "body_y": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self.emote_swing_legs = []
        self.auto_ready_pose = bool(
            self.get_parameter("auto_ready_pose").value
        )
        self.auto_ready_requested = False
        self.auto_ready_pending = False

        initial_body_height = (
            self.applied_real_tuning["body_height"]
            if self.hardware_mode
            else float(self.get_parameter("body_height").value)
        )
        self.body_height = clamp(
            initial_body_height,
            0.175,
            0.220,
        )
        self.neutral_body_height = self.body_height
        self.body_x = (
            self.applied_real_tuning["body_x"] if self.hardware_mode else 0.0
        )
        self.body_y = (
            self.applied_real_tuning["body_y"] if self.hardware_mode else 0.0
        )
        self.body_roll = math.radians(
            self.applied_real_tuning["body_roll_deg"]
        ) if self.hardware_mode else 0.0
        self.body_pitch = math.radians(
            self.applied_real_tuning["body_pitch_deg"]
        ) if self.hardware_mode else 0.0
        self.body_yaw = math.radians(
            self.applied_real_tuning["body_yaw_deg"]
        ) if self.hardware_mode else 0.0

        self.measured_positions = None
        self.last_joint_state_time = None
        self.commanded_positions = initial_commanded_positions
        self.commanded_velocities = initial_commanded_velocities
        self.gait_command_lag = 0.0
        self.transition = None
        self.warning = ""
        self.projected_targets = []
        self.clamped_joints = []
        self.workspace_margin = None
        self.ik_diagnostics = {"projected_targets": [], "legs": {}}
        self.last_status_time = 0.0
        self.last_debug_time = 0.0
        self.command_timed_out = False
        self.velocity_message_count = 0
        self.velocity_zero_transition_count = 0
        self.last_velocity_was_neutral = True
        self.velocity_rate_window_start = self.now_seconds()
        self.velocity_rate_window_count = 0
        self.cmd_vel_receive_rate = 0.0
        self.joint_velocity_clamp_counts = [0 for _ in JOINT_NAMES]
        self.joint_braking_clamp_counts = [0 for _ in JOINT_NAMES]
        self.joint_acceleration_clamp_counts = [0 for _ in JOINT_NAMES]
        self.joint_delta_clamp_counts = [0 for _ in JOINT_NAMES]
        self.ik_projection_count = 0
        self.joint_limit_clamp_count = 0
        self.last_gait_feet = {
            leg: tuple(NOMINAL_FEET[leg]) for leg in LEG_ORDER
        }
        # Retain the final planted footprint after a physical trot stop.  A
        # loaded robot must not drag all four feet back to HOME on the next
        # idle controller tick.
        self.standing_feet = {
            leg: tuple(NOMINAL_FEET[leg]) for leg in LEG_ORDER
        }
        self.last_raw_joint_target = (
            list(initial_commanded_positions)
            if initial_commanded_positions is not None
            else []
        )
        self.last_filtered_joint_target = list(self.last_raw_joint_target)
        # Raw-jump diagnostics compare only consecutive samples actually owned
        # by the active fast-trot trajectory.  The selected gait name persists
        # while emotes and pose transitions run, so the global raw target is not
        # a valid fast-trot baseline.
        self.last_fast_trot_raw_joint_target = []
        self.joint_command_delta_deg = {
            name: 0.0 for name in JOINT_NAMES
        }
        self.maximum_joint_command_delta_deg = 0.0
        self.maximum_raw_joint_jump_deg = 0.0
        self.ik_branch_continuous = True
        self.last_gait_body_transform = {
            "height": self.body_height,
            "body_x": self.body_x,
            "body_y": self.body_y,
            "roll": self.body_roll,
            "pitch": self.body_pitch,
            "yaw": self.body_yaw,
        }
        self.fast_trot_completed_cycles = 0
        self.fast_trot_last_cycle_phase = None
        self.fast_trot_stance_start_x = {
            leg: None for leg in LEG_ORDER
        }
        self.fast_trot_stance_last_x = {
            leg: None for leg in LEG_ORDER
        }
        self.fast_trot_stance_direction = {
            leg: 1.0 for leg in LEG_ORDER
        }
        self.fast_trot_stance_max_ground_error = {
            leg: 0.0 for leg in LEG_ORDER
        }
        self.fast_trot_stance_active = {
            leg: False for leg in LEG_ORDER
        }
        self.fast_trot_stance_completion_pending = set()
        self.fast_trot_completed_stance_strides = {}
        self.fast_trot_completed_stance_ground_errors = {}
        self.fast_trot_swing_max_clearance = {
            leg: 0.0 for leg in LEG_ORDER
        }
        self.fast_trot_completed_swing_clearances = {}
        self.fast_trot_cycle_joint_ranges = [
            [float("inf"), float("-inf")] for _ in JOINT_NAMES
        ]
        self.fast_trot_achieved_stride = 0.0
        self.fast_trot_signed_stride = 0.0
        self.fast_trot_stance_grounded = False
        self.fast_trot_max_stance_ground_error = 0.0
        self.fast_trot_achieved_step_height = 0.0
        self.fast_trot_cycle_requested_stride = 0.0
        self.fast_trot_completed_requested_stride = 0.0
        self.fast_trot_cycle_max_abs_yaw = 0.0
        self.fast_trot_stride_metric_valid = True
        self.fast_trot_max_joint_excursion_deg = 0.0
        self.fast_trot_joint_excursions_deg = {
            name: 0.0 for name in JOINT_NAMES
        }
        self.fast_trot_diagnostic_active = False
        self.fast_trot_measurement_ready = False
        self.arduino_frame_rate = 0.0

        period = 1.0 / max(self.control_rate, 1.0)
        self.timer = self.create_timer(period, self.control_callback)
        if self.open_loop_warning:
            self.get_logger().warning(self.open_loop_warning)
        self.get_logger().info(
            "VOLT controller ready; waiting for joint states and command router."
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def emote_playback_active(self):
        """Return active playback while tolerating pure unit-test fixtures."""
        return bool(
            getattr(getattr(self, "emote_engine", None), "active", False)
            or getattr(self, "emote_filter_settling", False)
        )

    def emote_request_pending(self):
        return getattr(self, "pending_emote_request", None) is not None

    def velocity_callback(self, message):
        received_at = self.now_seconds()
        requested = [
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        ]
        if not all(math.isfinite(value) for value in requested):
            self.requested_velocity = [0.0, 0.0, 0.0]
            self.last_velocity_time = received_at
            self.warning = "Rejected non-finite velocity command."
            self.get_logger().warning(
                self.warning,
                throttle_duration_sec=2.0,
            )
            return
        neutral = self.velocity_is_neutral(requested)
        pose_motion_blocked = (
            self.transition is not None
            or self.pending_pose_action is not None
            or self.state != "standing"
        )
        if pose_motion_blocked:
            self.requested_velocity = [0.0, 0.0, 0.0]
            self.last_velocity_time = received_at
            self.require_neutral_velocity()
            if not neutral:
                if self.transition is not None:
                    self.stop_motion()
                    self.warning = (
                        "Non-zero velocity cancelled the pose transition; "
                        "holding until a fresh neutral-to-motion handshake."
                    )
                else:
                    self.warning = (
                        "Velocity rejected outside the stopped standing state; "
                        "Stand Up before locomotion."
                    )
            return
        if (
            (
                self.emote_playback_active()
                or self.emote_request_pending()
            )
            and not neutral
        ):
            self.requested_velocity = [0.0, 0.0, 0.0]
            self.last_velocity_time = received_at
            self.cancel_emote(
                "non-zero velocity received during emote",
                received_at,
            )
            return
        if getattr(self, "physical_test", None) is not None and not neutral:
            self.requested_velocity = [0.0, 0.0, 0.0]
            self.last_velocity_time = received_at
            self.cancel_physical_test(
                "non-zero velocity received during physical test",
                received_at,
            )
            return
        self.velocity_message_count = getattr(
            self,
            "velocity_message_count",
            0,
        ) + 1
        self.velocity_rate_window_count = getattr(
            self,
            "velocity_rate_window_count",
            0,
        ) + 1
        if neutral != getattr(self, "last_velocity_was_neutral", True):
            self.velocity_zero_transition_count = getattr(
                self,
                "velocity_zero_transition_count",
                0,
            ) + 1
        self.last_velocity_was_neutral = neutral
        rate_elapsed = received_at - getattr(
            self,
            "velocity_rate_window_start",
            received_at,
        )
        if rate_elapsed >= 1.0:
            self.cmd_vel_receive_rate = (
                self.velocity_rate_window_count / max(rate_elapsed, 1e-9)
            )
            self.velocity_rate_window_start = received_at
            self.velocity_rate_window_count = 0

        # Commands observed while another source owns the router must never be
        # cached and resumed later.  Ownership enable establishes a new
        # neutral/fresh-command handshake.
        if self.command_owner != "MOTION":
            self.requested_velocity = [0.0, 0.0, 0.0]
            self.last_velocity_time = received_at
            return

        self.velocity_command_sequence += 1
        self.last_velocity_time = received_at
        if self.velocity_gate_state == VELOCITY_GATE_AWAIT_NEUTRAL:
            self.requested_velocity = [0.0, 0.0, 0.0]
            if neutral:
                self.velocity_gate_state = VELOCITY_GATE_AWAIT_MOTION
            return
        if self.velocity_gate_state == VELOCITY_GATE_AWAIT_MOTION:
            self.requested_velocity = [0.0, 0.0, 0.0]
            if neutral:
                return
            self.velocity_gate_state = VELOCITY_GATE_OPEN
        self.requested_velocity = requested
        if (
            neutral
            and self.gait_controller.active
            and not self.step_in_place
        ):
            # Latch the controlled stop from the raw operator command. Waiting
            # for the low-pass velocity to decay could otherwise authorize an
            # extra stride after the joystick was released.
            self.gait_controller.request_stop()

    def gait_callback(self, message):
        requested = canonical_gait_name(message.data)
        if requested not in self.gait_configs:
            self.get_logger().warning("Unknown gait '%s'." % requested)
            return
        if (
            self.transition is not None
            or self.pending_pose_action is not None
            or self.state != "standing"
        ):
            self.warning = (
                "Gait selection rejected during a pose transition or outside "
                "the standing state."
            )
            self.get_logger().warning(
                self.warning,
                throttle_duration_sec=1.0,
            )
            return
        if getattr(self, "physical_test", None) is not None:
            self.warning = (
                "Gait selection rejected while a finite physical test is active."
            )
            self.get_logger().warning(
                self.warning,
                throttle_duration_sec=1.0,
            )
            return
        if self.emote_playback_active() or self.emote_request_pending():
            self.warning = "Gait selection rejected while an emote is active."
            self.get_logger().warning(
                self.warning,
                throttle_duration_sec=1.0,
            )
            return
        self.requested_gait = requested
        if requested != self.gait_name:
            self.step_in_place = False
            if self.motion_active or self.gait_controller.active:
                self.pending_gait = requested
                self.gait_controller.request_stop()
                self.get_logger().info(
                    "Stopping %s before switching to %s."
                    % (self.gait_name, requested)
                )
            else:
                self.select_gait(requested, self.now_seconds())
        elif self.pending_gait is not None:
            # Selecting the already-active gait cancels an obsolete queued
            # switch. The current stop/settle is still allowed to finish, but
            # the pre-switch velocity must never restart it automatically.
            self.pending_gait = None
            self.step_in_place = False
            self.require_neutral_velocity()
            self.get_logger().info(
                "Cancelled queued gait switch; waiting for neutral then "
                "a fresh velocity command."
            )

    def fast_trot_tuning_callback(self, message):
        """Apply one bounded request only while every gait foot is grounded."""
        try:
            tuning = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.warning = "Rejected malformed FAST TROT tuning JSON."
            self.get_logger().warning(self.warning)
            return
        if (
            self.gait_controller.active
            or self.state != "standing"
            or self.transition is not None
            or self.pending_pose_action is not None
            or getattr(self, "physical_test", None) is not None
            or self.emote_playback_active()
            or self.emote_request_pending()
        ):
            self.warning = (
                "FAST TROT tuning rejected while motion is active; "
                "STOP and wait for all feet to settle."
            )
            self.get_logger().warning(self.warning)
            return
        try:
            validated = validate_fast_trot_tuning(
                self.gait_configs["fast_trot"],
                tuning,
            )
            self.gait_controller.set_fast_trot_tuning(validated)
        except (TypeError, ValueError) as exc:
            self.warning = "Rejected FAST TROT tuning: %s" % exc
            self.get_logger().warning(self.warning)
            return
        self.reset_fast_trot_cycle_diagnostics()
        if self.warning.startswith("Rejected FAST TROT"):
            self.warning = ""
        self.get_logger().info(
            "Applied stopped-state FAST TROT tuning: %s"
            % json.dumps(validated, sort_keys=True)
        )

    def real_tuning_idle_reason(self):
        """Return why a full profile transaction cannot be applied now."""
        if self.state != "standing":
            return "real-profile tuning requires the stopped standing state"
        if (
            self.gait_controller.active
            or self.transition is not None
            or self.pending_pose_action is not None
            or self.pending_gait is not None
            or getattr(self, "physical_test", None) is not None
            or self.emote_playback_active()
            or self.emote_request_pending()
            or self.motion_active
            or self.step_in_place
        ):
            return "STOP and wait for all feet and transitions to settle"
        if not self.velocity_is_neutral(self.requested_velocity) or not (
            self.velocity_is_neutral(self.filtered_velocity)
        ):
            return "requested and filtered velocity must both be neutral"
        settle_reason = self.conditioned_command_settle_reason()
        if settle_reason:
            return settle_reason
        return ""

    def conditioned_command_settle_reason(self, expected_positions=None):
        """Describe residual commanded motion; this is not servo feedback."""
        positions = getattr(self, "commanded_positions", None)
        velocities = getattr(self, "commanded_velocities", None)
        if (
            positions is None
            or velocities is None
            or len(positions) != len(JOINT_NAMES)
            or len(velocities) != len(JOINT_NAMES)
            or not all(
                math.isfinite(float(value))
                for value in list(positions) + list(velocities)
            )
        ):
            return "conditioned joint command is not finite and complete"
        if max(abs(float(value)) for value in velocities) > (
            FINITE_MOTION_VELOCITY_TOLERANCE
        ):
            return "conditioned joint command is still moving"
        if expected_positions is not None:
            if len(expected_positions) != len(JOINT_NAMES) or not all(
                math.isfinite(float(value)) for value in expected_positions
            ):
                return "expected stopped joint target is invalid"
            if max(
                abs(float(actual) - float(expected))
                for actual, expected in zip(positions, expected_positions)
            ) > FINITE_MOTION_POSITION_TOLERANCE:
                return "conditioned joint command has not reached the stopped target"
        return ""

    def preflight_real_tuning(self, tuning):
        """Reject profile extremes that require IK projection or joint clamp."""
        stance_width = tuning["stance_width"]
        feet = {
            leg: (
                NOMINAL_FEET[leg][0],
                math.copysign(stance_width, NOMINAL_FEET[leg][1]),
                NOMINAL_FEET[leg][2],
            )
            for leg in LEG_ORDER
        }
        samples = [feet]
        reach_scale = 1.0 if tuning["gait"] == "diagnostic_crawl" else 0.5
        x_limit = reach_scale * tuning["stride_length"]
        y_limit = reach_scale * tuning["lateral_stride_width"]
        # Profile apply is an infrequent stopped-state transaction. Sampling
        # the complete ellipse at 1-degree intervals is worth the bounded
        # cost: axes/diagonals alone can miss a narrow IK boundary. A 1%
        # outward margin rejects profiles that only barely fit between samples.
        reach_margin = 1.01
        directions = ((0.0, 0.0),) + tuple(
            (
                reach_margin * math.cos(math.radians(degrees)),
                reach_margin * math.sin(math.radians(degrees)),
            )
            for degrees in range(360)
        )
        for leg in LEG_ORDER:
            for direction_x, direction_y in directions:
                for clearance in (0.0, tuning["step_height"]):
                    sample = {
                        name: tuple(point) for name, point in feet.items()
                    }
                    x, y, z = sample[leg]
                    sample[leg] = (
                        x + direction_x * x_limit,
                        y + direction_y * y_limit,
                        z + clearance,
                    )
                    samples.append(sample)
        for index, sample in enumerate(samples):
            _joints, diagnostics = feet_to_joint_positions_diagnostic(
                sample,
                height=tuning["body_height"],
                body_x=tuning["body_x"],
                body_y=tuning["body_y"],
                roll=math.radians(tuning["body_roll_deg"]),
                pitch=math.radians(tuning["body_pitch_deg"]),
                yaw=math.radians(tuning["body_yaw_deg"]),
            )
            if diagnostics["projected_targets"]:
                raise RealProfileError(
                    "IK preflight sample %d projected legs %s"
                    % (index, diagnostics["projected_targets"])
                )
            clamped = [
                joint
                for leg in LEG_ORDER
                for joint in diagnostics["legs"][leg]["clamped_joints"]
            ]
            if clamped:
                raise RealProfileError(
                    "IK preflight sample %d clamped %s" % (index, clamped)
                )

    def reject_real_tuning(self, request_id, reason):
        self.real_tuning_request_id = str(request_id)
        self.real_tuning_result = "rejected"
        self.real_tuning_message = str(reason)
        self.warning = "Rejected real-robot tuning: %s" % reason
        self.get_logger().warning(self.warning)

    def real_robot_tuning_callback(self, message):
        """Atomically validate and apply one stopped-state hardware profile."""
        try:
            decoded = json.loads(message.data)
            if not isinstance(decoded, dict):
                raise RealProfileError("request must be a JSON object")
        except (TypeError, ValueError, json.JSONDecodeError, RealProfileError) as exc:
            self.reject_real_tuning("", "malformed JSON: %s" % exc)
            return
        request_id = str(decoded.get("request_id", "")).strip()
        if not request_id or len(request_id) > 64:
            self.reject_real_tuning(request_id, "request_id must contain 1-64 characters")
            return
        requested_profile = str(decoded.get("profile_name", "CUSTOM")).strip().upper()
        values = decoded.get("values")
        try:
            tuning = validate_tuning(values, allow_simulation=True)
        except (RealProfileError, TypeError, ValueError) as exc:
            self.reject_real_tuning(request_id, str(exc))
            return

        if not self.hardware_mode:
            if requested_profile != "SIMULATION" or tuning["gait"] != "spotmicro_video_walk":
                self.reject_real_tuning(
                    request_id,
                    "real profiles require hardware_mode; simulation accepts only SIMULATION",
                )
                return
            if tuning != self.real_profiles["SIMULATION"]:
                self.reject_real_tuning(
                    request_id,
                    "SIMULATION is read-only so the proven simulator gait remains unchanged",
                )
                return
            self.active_real_profile = "SIMULATION"
            self.applied_real_tuning = dict(tuning)
            self.real_tuning_request_id = request_id
            self.real_tuning_result = "applied"
            self.real_tuning_message = (
                "Simulation profile acknowledged; hardware conditioning remains disabled."
            )
            return

        reason = self.real_tuning_idle_reason()
        if reason:
            self.reject_real_tuning(request_id, reason)
            return
        try:
            tuning = validate_tuning(tuning, allow_simulation=False)
            self.preflight_real_tuning(tuning)
            updated_configs = apply_real_tuning_to_configs(
                self.gait_configs,
                tuning,
            )
        except (KinematicsError, RealProfileError, TypeError, ValueError) as exc:
            self.reject_real_tuning(request_id, str(exc))
            return

        # Nothing below can partially move hardware: the controller is idle,
        # and all dictionaries/scalars are committed before gait selection.
        self.gait_configs = updated_configs
        self.gait_controller.gaits = dict(updated_configs)
        self.max_joint_velocity = math.radians(
            tuning["max_joint_velocity_deg_s"]
        )
        self.max_joint_acceleration = math.radians(
            tuning["max_joint_acceleration_deg_s2"]
        )
        self.joint_smoothing_alpha = smoothing_alpha(tuning)
        self.joint_smoothing_factor = self.joint_smoothing_alpha
        self.neutral_body_height = tuning["body_height"]
        self.body_height = tuning["body_height"]
        self.body_x = tuning["body_x"]
        self.body_y = tuning["body_y"]
        self.body_roll = math.radians(tuning["body_roll_deg"])
        self.body_pitch = math.radians(tuning["body_pitch_deg"])
        self.body_yaw = math.radians(tuning["body_yaw_deg"])
        self.applied_real_tuning = dict(tuning)
        matching_builtin = self.real_profiles.get(requested_profile)
        self.active_real_profile = (
            requested_profile
            if matching_builtin == tuning
            else (requested_profile if requested_profile not in self.real_profiles else "CUSTOM")
        )
        self.requested_gait = tuning["gait"]
        self.select_gait(tuning["gait"], self.now_seconds())
        profile_feet = self.gait_controller.nominal_feet()
        self.standing_feet = {
            leg: tuple(profile_feet[leg]) for leg in LEG_ORDER
        }
        self.gait_controller.set_current_feet(self.standing_feet)
        self.real_tuning_request_id = request_id
        self.real_tuning_result = "applied"
        self.real_tuning_message = "Applied %s." % self.active_real_profile
        if self.warning.startswith("Rejected real-robot tuning"):
            self.warning = ""
        self.get_logger().info(
            "Applied stopped-state real profile %s: %s"
            % (self.active_real_profile, json.dumps(tuning, sort_keys=True))
        )

    def reject_emote(self, request_id, reason):
        """Publish a correlated fail-closed result without changing targets."""
        current = (
            getattr(self, "active_emote_request", None)
            or getattr(self, "pending_emote_request", None)
        )
        if current is not None:
            self.warning = (
                "Ignored emote request %s while %s remains authoritative: %s"
                % (
                    str(request_id),
                    current.get("request_id", "active request"),
                    reason,
                )
            )
            self.get_logger().warning(self.warning)
            return
        self.emote_request_id = str(request_id)
        self.emote_result = "rejected"
        self.emote_message = str(reason)
        self.warning = "Rejected emote request: %s" % reason
        self.get_logger().warning(self.warning)

    def emote_start_idle_reason(self):
        """Return why a queued emote cannot enter its known standing pose."""
        if self.command_owner != "MOTION":
            return "emotes require MOTION command ownership"
        if self.state != "standing":
            return "emotes require the standing state"
        if self.commanded_positions is None:
            return "emotes require a finite commanded standing pose"
        if (
            self.transition is not None
            or self.pending_pose_action is not None
            or self.pending_gait is not None
            or getattr(self, "physical_test", None) is not None
        ):
            return "another finite motion or transition is active"
        if self.step_in_place or self.motion_active or self.gait_controller.active:
            return "locomotion is still returning all feet to support"
        if not self.velocity_is_neutral(self.requested_velocity) or not (
            self.velocity_is_neutral(self.filtered_velocity)
        ):
            return "requested and filtered velocity must both be neutral"
        try:
            expected_positions, diagnostics = feet_to_joint_positions_diagnostic(
                self.standing_feet,
                height=self.body_height,
                body_x=self.body_x,
                body_y=self.body_y,
                roll=self.body_roll,
                pitch=self.body_pitch,
                yaw=self.body_yaw,
            )
        except (KinematicsError, KeyError, TypeError, ValueError) as exc:
            return "standing target is invalid: %s" % exc
        if diagnostics["projected_targets"]:
            return "standing target requires IK projection"
        settle_reason = self.conditioned_command_settle_reason(
            expected_positions,
        )
        if settle_reason:
            return settle_reason
        return ""

    def emote_callback(self, message):
        """Validate, queue, or cancel one nonblocking Cartesian emote."""
        decoded = None
        try:
            decoded = json.loads(message.data)
            if not isinstance(decoded, dict):
                raise EmoteValidationError("request must be a JSON object")
            allowed = {
                "command",
                "request_id",
                "name",
                "repetitions",
                "speed",
                "amplitude",
                "depth",
            }
            unknown = sorted(set(decoded) - allowed)
            if unknown:
                raise EmoteValidationError(
                    "request has unknown keys: %s" % unknown
                )
            command = str(decoded.get("command", "")).strip().lower()
            request_id = str(decoded.get("request_id", "")).strip()
            if command not in ("start", "keepalive", "cancel"):
                raise EmoteValidationError(
                    "command must be start, keepalive, or cancel"
                )
            if not request_id or len(request_id) > 64:
                raise EmoteValidationError(
                    "request_id must contain 1-64 characters"
                )
            if command in ("keepalive", "cancel") and set(decoded) != {
                "command",
                "request_id",
            }:
                raise EmoteValidationError(
                    "%s request must contain only command and request_id"
                    % command
                )
        except (
            TypeError,
            ValueError,
            json.JSONDecodeError,
            EmoteValidationError,
        ) as exc:
            self.reject_emote(
                decoded.get("request_id", "") if isinstance(decoded, dict) else "",
                str(exc),
            )
            return

        if command in ("keepalive", "cancel"):
            active_id = (
                self.active_emote_request["request_id"]
                if self.active_emote_request is not None
                else ""
            )
            pending_id = (
                self.pending_emote_request["request_id"]
                if self.pending_emote_request is not None
                else ""
            )
            if request_id not in (active_id, pending_id):
                self.warning = (
                    "Ignored stale or mismatched emote %s request %s."
                    % (command, request_id)
                )
                self.get_logger().warning(self.warning)
                return
            if command == "keepalive":
                request = (
                    self.active_emote_request
                    if request_id == active_id
                    else self.pending_emote_request
                )
                request["last_keepalive_time"] = self.now_seconds()
                return
            self.cancel_emote("operator cancel request", self.now_seconds())
            return

        if self.emote_engine.active or self.active_emote_request is not None:
            self.reject_emote(request_id, "another emote is already active")
            return
        if self.pending_emote_request is not None:
            self.reject_emote(request_id, "another emote is already queued")
            return
        name = str(decoded.get("name", "")).strip().lower()
        try:
            options = validate_options(
                decoded.get("repetitions", 1),
                decoded.get("speed", 1.0),
                decoded.get("amplitude", 1.0),
                decoded.get("depth", 1.0),
            )
            if name not in self.emote_catalog.emotes:
                raise EmoteValidationError("unknown emote %r" % name)
        except (TypeError, ValueError, EmoteValidationError) as exc:
            self.reject_emote(request_id, str(exc))
            return
        if self.command_owner != "MOTION":
            self.reject_emote(request_id, "emotes require MOTION ownership")
            return
        if self.state != "standing":
            self.reject_emote(request_id, "emotes require the standing state")
            return
        if (
            self.transition is not None
            or self.pending_pose_action is not None
            or getattr(self, "physical_test", None) is not None
        ):
            self.reject_emote(
                request_id,
                "finish the active pose transition or diagnostic first",
            )
            return

        # Latch zero and ask the gait engine for a complete planted stop.  The
        # request starts only after the normal control loop certifies idle.
        self.stop_motion()
        self.cancel_pending_gait_switch()
        self.pending_emote_request = {
            "request_id": request_id,
            "name": name,
            "repetitions": options.repetitions,
            "speed": options.speed,
            "amplitude": options.amplitude,
            "depth": options.depth,
            "last_keepalive_time": self.now_seconds(),
        }
        self.emote_request_id = request_id
        self.emote_result = "queued"
        self.emote_message = "Stopping locomotion before %s." % name
        self.emote_progress = 0.0
        if self.warning.startswith("Rejected emote request"):
            self.warning = ""

    def force_reset_emote(self, reason=""):
        """Discard controller bookkeeping when the router has already held."""
        engine = getattr(self, "emote_engine", None)
        had_request = (
            getattr(self, "pending_emote_request", None) is not None
            or getattr(self, "active_emote_request", None) is not None
            or bool(getattr(engine, "active", False))
            or getattr(self, "emote_filter_settling", False)
        )
        request_id = getattr(self, "emote_request_id", "")
        catalog = getattr(self, "emote_catalog", None)
        if had_request and catalog is not None:
            self.emote_engine = CartesianEmoteEngine(catalog)
        self.pending_emote_request = None
        self.active_emote_request = None
        self.emote_progress = 0.0
        self.emote_cancelled = False
        self.emote_filter_settling = False
        self.emote_swing_legs = []
        if had_request:
            self.emote_request_id = request_id
            self.emote_result = "cancelled"
            self.emote_message = str(reason or "Emote cancelled.")

    def cancel_emote(self, reason, now=None):
        """Cancel a queued request or smoothly return an active emote."""
        if now is None:
            now = self.now_seconds()
        if getattr(self, "pending_emote_request", None) is not None:
            request_id = self.pending_emote_request["request_id"]
            self.pending_emote_request = None
            self.emote_request_id = request_id
            self.emote_result = "cancelled"
            self.emote_message = "Queued emote cancelled: %s." % reason
            self.emote_progress = 0.0
            return True
        if getattr(self, "emote_filter_settling", False):
            self.emote_cancelled = True
            self.emote_result = "settling"
            self.emote_message = (
                "Conditioning the cancelled emote back to captured stand."
            )
            return True
        engine = getattr(self, "emote_engine", None)
        if bool(getattr(engine, "active", False)):
            engine.cancel(float(now))
            self.emote_cancelled = True
            self.emote_result = "returning"
            self.emote_message = "Returning to stand: %s." % reason
            return True
        return False

    def expire_emote_if_stale(self, now):
        """Return or cancel when the requesting GUI/client loses its lease."""
        request = (
            getattr(self, "active_emote_request", None)
            or getattr(self, "pending_emote_request", None)
        )
        if request is None:
            return False
        last_keepalive = float(request.get("last_keepalive_time", now))
        timeout = getattr(self, "emote_keepalive_timeout", 0.75)
        if max(0.0, float(now) - last_keepalive) <= timeout:
            return False
        cancelled = self.cancel_emote("emote keepalive timeout", now)
        if cancelled:
            self.warning = (
                "Emote lease expired; queued motion was cancelled or active "
                "motion is returning to captured stand."
            )
        return cancelled

    def compose_emote_frame(self, frame, base_feet=None, base_body=None):
        """Apply a relative catalog frame to the captured planted stance."""
        base_feet = self.emote_base_feet if base_feet is None else base_feet
        base_body = self.emote_base_body if base_body is None else base_body
        feet = {
            leg: tuple(
                float(base_feet[leg][axis])
                + float(frame.feet[leg][axis])
                - float(NOMINAL_FEET[leg][axis])
                for axis in range(3)
            )
            for leg in LEG_ORDER
        }
        body = {
            "height": float(base_body["height"])
            + float(frame.body.height)
            - float(self.emote_catalog.base_body_height),
            "body_x": float(base_body["body_x"]) + float(frame.body.x),
            "body_y": float(base_body["body_y"]) + float(frame.body.y),
            "roll": float(base_body["roll"]) + float(frame.body.roll),
            "pitch": float(base_body["pitch"]) + float(frame.body.pitch),
            "yaw": float(base_body["yaw"]) + float(frame.body.yaw),
        }
        values = list(body.values()) + [
            value for point in feet.values() for value in point
        ]
        if not all(math.isfinite(value) for value in values):
            raise EmoteValidationError("composed emote target is non-finite")
        bounds = {
            # Emote composition may dip below the standing envelope while all
            # four feet stay planted; see MIN_EMOTE_BODY_HEIGHT_M in
            # volt_emote_engine.  Manual pose and gait limits are unchanged.
            "height": (0.132, 0.220),
            "body_x": (-0.030, 0.030),
            "body_y": (-0.026, 0.026),
            "roll": (-0.24, 0.24),
            "pitch": (-0.24, 0.24),
            "yaw": (-0.24, 0.24),
        }
        for name, (lower, upper) in bounds.items():
            if not lower <= body[name] <= upper:
                raise EmoteValidationError(
                    "%s target %.6f is outside [%.6f, %.6f]"
                    % (name, body[name], lower, upper)
                )
        return feet, body

    def preflight_pending_emote(self, request, base_feet, base_body):
        """Prove the full runtime request against its actual standing base."""
        engine = CartesianEmoteEngine(
            self.emote_catalog,
            preflight_on_start=False,
        )
        engine.start(
            request["name"],
            0.0,
            repetitions=request["repetitions"],
            speed=request["speed"],
            amplitude=request["amplitude"],
            depth=request["depth"],
        )
        definition = self.emote_catalog.emotes[request["name"]]
        duration = (
            definition.total_duration
            * request["repetitions"]
            / request["speed"]
        )
        intervals = max(1, int(math.ceil(duration / 0.05)))
        for index in range(intervals + 1):
            frame = engine.sample(duration * index / intervals)
            feet, body = self.compose_emote_frame(
                frame,
                base_feet=base_feet,
                base_body=base_body,
            )
            _joints, diagnostics = feet_to_joint_positions_diagnostic(
                feet,
                **body,
            )
            if diagnostics["projected_targets"]:
                raise EmoteValidationError(
                    "runtime preflight projected %s at sample %d"
                    % (diagnostics["projected_targets"], index)
                )
            clamped = [
                joint
                for leg in LEG_ORDER
                for joint in diagnostics["legs"][leg].get(
                    "clamped_joints",
                    [],
                )
            ]
            if clamped:
                raise EmoteValidationError(
                    "runtime preflight clamped %s at sample %d"
                    % (clamped, index)
                )

    def start_pending_emote(self, now):
        """Atomically start a queued request after locomotion has settled."""
        request = getattr(self, "pending_emote_request", None)
        if request is None:
            return False
        reason = self.emote_start_idle_reason()
        if reason:
            return False
        base_feet = {
            leg: tuple(self.standing_feet[leg]) for leg in LEG_ORDER
        }
        base_body = {
            "height": float(self.body_height),
            "body_x": float(self.body_x),
            "body_y": float(self.body_y),
            "roll": float(self.body_roll),
            "pitch": float(self.body_pitch),
            "yaw": float(self.body_yaw),
        }
        try:
            self.preflight_pending_emote(request, base_feet, base_body)
            if self.emote_engine.state == "complete":
                self.emote_engine.reset()
            self.emote_engine.start(
                request["name"],
                float(now),
                repetitions=request["repetitions"],
                speed=request["speed"],
                amplitude=request["amplitude"],
                depth=request["depth"],
            )
        except (
            KinematicsError,
            EmoteStateError,
            EmoteValidationError,
            TypeError,
            ValueError,
        ) as exc:
            self.pending_emote_request = None
            self.reject_emote(request["request_id"], str(exc))
            return False
        self.emote_base_feet = base_feet
        self.emote_base_body = base_body
        self.active_emote_request = request
        self.pending_emote_request = None
        self.emote_request_id = request["request_id"]
        self.emote_result = "running"
        self.emote_message = "Running %s." % request["name"]
        self.emote_progress = 0.0
        self.emote_cancelled = False
        self.emote_filter_settling = False
        self.require_neutral_velocity()
        self.get_logger().info(
            "Started Cartesian emote %s (%s)."
            % (request["name"], request["request_id"])
        )
        return True

    def emote_target(self, now):
        """Sample one emote frame and pass it through canonical IK."""
        frame = self.emote_engine.sample(float(now))
        feet, body = self.compose_emote_frame(frame)
        positions, diagnostics = feet_to_joint_positions_diagnostic(
            feet,
            **body,
        )
        projected = list(diagnostics["projected_targets"])
        clamped = [
            joint
            for leg in LEG_ORDER
            for joint in diagnostics["legs"][leg].get("clamped_joints", [])
        ]
        if projected or clamped:
            self.force_reset_emote(
                "Runtime IK guard rejected projected/clamped emote target."
            )
            raise KinematicsError(
                "emote target projected %s or clamped %s"
                % (projected, clamped)
            )
        self.record_ik_diagnostics(diagnostics)
        self.last_gait_feet = {
            leg: tuple(feet[leg]) for leg in LEG_ORDER
        }
        self.last_gait_body_transform = dict(body)
        self.emote_progress = float(frame.progress)
        self.emote_swing_legs = [
            leg
            for leg in LEG_ORDER
            if feet[leg][2] > self.emote_base_feet[leg][2] + 0.003
        ]
        complete = frame.state == "complete"
        self.motion_active = True
        if complete:
            if not getattr(self, "emote_filter_settling", False):
                self.emote_filter_settling = True
                self.emote_result = "settling"
                self.emote_message = (
                    "%s trajectory finished; conditioning the commanded "
                    "joints back to captured stand." % frame.emote
                )
            self.emote_swing_legs = []
            self.standing_feet = {
                leg: tuple(self.emote_base_feet[leg]) for leg in LEG_ORDER
            }
        return positions

    def complete_emote_after_filter(self, raw_target, filtered_target):
        """Publish terminal emote state only after joint conditioning settles."""
        if not getattr(self, "emote_filter_settling", False):
            return False
        velocities = getattr(self, "commanded_velocities", None)
        if velocities is None or len(velocities) != len(JOINT_NAMES):
            return False
        position_error = max(
            abs(float(raw) - float(filtered))
            for raw, filtered in zip(raw_target, filtered_target)
        )
        maximum_velocity = max(abs(float(value)) for value in velocities)
        if (
            position_error > FINITE_MOTION_POSITION_TOLERANCE
            or maximum_velocity > FINITE_MOTION_VELOCITY_TOLERANCE
        ):
            self.motion_active = True
            return False

        active = getattr(self, "active_emote_request", None)
        name = (
            active["name"]
            if active is not None
            else self.emote_engine.current_emote
        )
        request_id = (
            active["request_id"]
            if active is not None
            else self.emote_request_id
        )
        result = "cancelled" if self.emote_cancelled else "completed"
        self.active_emote_request = None
        self.emote_request_id = request_id
        self.emote_result = result
        self.emote_message = "%s %s and settled at stand." % (name, result)
        self.emote_progress = 1.0
        self.emote_swing_legs = []
        self.emote_filter_settling = False
        self.emote_engine.reset()
        self.emote_cancelled = False
        self.motion_active = False
        self.get_logger().info(self.emote_message)
        return True

    def physical_test_start_allowed(self):
        """Return a fail-closed reason, or an empty string when ready.

        The CLI acknowledgement is only the operator-side gate.  This
        controller independently requires an explicitly enabled hardware
        launch, MOTION ownership, a neutral planted standing pose, and no
        queued or active motion.
        """
        if not getattr(self, "enable_physical_tests", False):
            return "launch with enable_physical_tests:=true"
        if not getattr(self, "hardware_mode", False):
            return "physical tests require hardware_mode:=true"
        if getattr(self, "command_owner", "") != "MOTION":
            return "physical tests require MOTION command ownership"
        if getattr(self, "state", "") != "standing":
            return "physical tests require the stopped standing state"
        if getattr(self, "commanded_positions", None) is None:
            return "physical tests require a finite commanded standing pose"
        if (
            getattr(self, "transition", None) is not None
            or getattr(self, "pending_pose_action", None) is not None
            or getattr(self, "pending_gait", None) is not None
            or getattr(self, "step_in_place", False)
            or getattr(self, "motion_active", False)
            or bool(getattr(self.gait_controller, "active", False))
            or self.emote_playback_active()
            or self.emote_request_pending()
        ):
            return "physical tests require all motion and transitions to stop"
        if not self.velocity_is_neutral(
            getattr(self, "requested_velocity", [0.0, 0.0, 0.0])
        ) or not self.velocity_is_neutral(
            getattr(self, "filtered_velocity", [0.0, 0.0, 0.0])
        ):
            return "physical tests require neutral requested and filtered velocity"
        settle_reason = self.conditioned_command_settle_reason(WALK_POSE)
        if settle_reason:
            return "physical tests require settled WALK_POSE: %s" % settle_reason
        neutral_values = (
            getattr(self, "body_height", 0.0)
            - getattr(self, "neutral_body_height", 0.0),
            getattr(self, "body_x", 0.0),
            getattr(self, "body_y", 0.0),
            getattr(self, "body_roll", 0.0),
            getattr(self, "body_pitch", 0.0),
            getattr(self, "body_yaw", 0.0),
        )
        if any(abs(float(value)) > 1e-4 for value in neutral_values):
            return "physical tests require the neutral body pose"
        standing_feet = getattr(self, "standing_feet", NOMINAL_FEET)
        try:
            maximum_foot_error = max(
                math.dist(standing_feet[leg], NOMINAL_FEET[leg])
                for leg in LEG_ORDER
            )
        except (KeyError, TypeError, ValueError):
            return "physical tests require a valid canonical standing footprint"
        if maximum_foot_error > 0.003:
            return (
                "physical tests require the nominal planted footprint; "
                "return to the supported stand pose first"
            )
        return ""

    def physical_test_callback(self, message):
        """Accept one strict, leased Cartesian test request."""
        try:
            decoded = json.loads(message.data)
            if not isinstance(decoded, dict):
                raise PhysicalTestError("request must be a JSON object")
            request = physical_test_request_payload(
                decoded.get("command"),
                decoded.get("mode"),
                decoded.get("duration"),
                decoded.get("request_id"),
                leg=decoded.get("leg"),
            )
        except (
            TypeError,
            ValueError,
            json.JSONDecodeError,
            PhysicalTestError,
        ) as exc:
            self.warning = "Rejected physical test request: %s" % exc
            self.get_logger().warning(
                self.warning,
                throttle_duration_sec=1.0,
            )
            return

        now = self.now_seconds()
        active = getattr(self, "physical_test", None)
        command = request["command"]
        request_matches = (
            active is not None
            and request["request_id"] == active["request_id"]
            and request["mode"] == active["mode"]
            and request["leg"] == active["leg"]
            and abs(request["duration"] - active["duration"]) <= 1e-9
        )

        if command == "start":
            if active is not None:
                self.warning = (
                    "Rejected physical test start: another finite test is active."
                )
                self.get_logger().warning(
                    self.warning,
                    throttle_duration_sec=1.0,
                )
                return
            reason = self.physical_test_start_allowed()
            if reason:
                self.warning = "Rejected physical test start: %s." % reason
                self.get_logger().warning(
                    self.warning,
                    throttle_duration_sec=1.0,
                )
                return
            self.require_neutral_velocity()
            self.physical_test = {
                "request_id": request["request_id"],
                "mode": request["mode"],
                "leg": request["leg"],
                "duration": request["duration"],
                "start_time": now,
                "last_keepalive_time": now,
                "current_feet": {
                    leg: tuple(NOMINAL_FEET[leg]) for leg in LEG_ORDER
                },
                "cancel_start_time": None,
                "cancel_start_feet": None,
                "cancel_reason": "",
                "filter_settling": False,
            }
            self.warning = ""
            self.get_logger().warning(
                "PHYSICAL TEST STARTED: %s (%s); support stand is mandatory."
                % (request["mode"], request["request_id"])
            )
            return

        if not request_matches:
            self.warning = (
                "Ignored stale or mismatched physical test %s request."
                % command
            )
            self.get_logger().warning(
                self.warning,
                throttle_duration_sec=1.0,
            )
            return
        if command == "keepalive":
            if active["cancel_start_time"] is None:
                active["last_keepalive_time"] = now
            return
        self.cancel_physical_test("operator cancel request", now)

    def cancel_physical_test(self, reason, now=None):
        """Start a bounded return-to-stand without snapping a lifted leg."""
        active = getattr(self, "physical_test", None)
        if active is None:
            return False
        if active.get("cancel_start_time") is not None:
            return True
        if now is None:
            now = self.now_seconds()
        active["cancel_start_time"] = float(now)
        active["cancel_start_feet"] = {
            leg: tuple(active["current_feet"][leg]) for leg in LEG_ORDER
        }
        active["cancel_reason"] = str(reason)
        self.requested_velocity = [0.0, 0.0, 0.0]
        self.filtered_velocity = [0.0, 0.0, 0.0]
        self.warning = (
            "Physical test returning to supported stand: %s." % reason
        )
        return True

    def physical_test_target(self, now):
        """Generate one canonical IK target for the active finite test."""
        active = self.physical_test
        keepalive_age = max(0.0, now - active["last_keepalive_time"])
        if (
            active["cancel_start_time"] is None
            and keepalive_age
            > getattr(self, "physical_test_keepalive_timeout", 0.75)
        ):
            self.cancel_physical_test("keepalive timeout", now)

        if active["cancel_start_time"] is not None:
            cancel_duration = 1.0
            cancel_elapsed = max(0.0, now - active["cancel_start_time"])
            blend = min(1.0, cancel_elapsed / cancel_duration)
            feet = {
                leg: tuple(
                    interpolate(
                        active["cancel_start_feet"][leg],
                        NOMINAL_FEET[leg],
                        blend,
                    )
                )
                for leg in LEG_ORDER
            }
            complete = blend >= 1.0
        else:
            elapsed = max(0.0, now - active["start_time"])
            frame = cartesian_frame_at(
                active["mode"],
                elapsed,
                active["duration"],
                leg=active["leg"],
            )
            feet = {
                leg: tuple(frame.feet[leg]) for leg in LEG_ORDER
            }
            complete = elapsed >= active["duration"]

        active["current_feet"] = {
            leg: tuple(feet[leg]) for leg in LEG_ORDER
        }
        self.standing_feet = {
            leg: tuple(feet[leg]) for leg in LEG_ORDER
        }
        self.last_gait_feet = {
            leg: tuple(feet[leg]) for leg in LEG_ORDER
        }
        self.last_gait_body_transform = {
            "height": float(self.neutral_body_height),
            "body_x": 0.0,
            "body_y": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        joints, diagnostics = feet_to_joint_positions_diagnostic(
            feet,
            height=self.neutral_body_height,
        )
        self.record_ik_diagnostics(diagnostics)
        self.motion_active = True
        if complete:
            active["filter_settling"] = True
            self.standing_feet = {
                leg: tuple(NOMINAL_FEET[leg]) for leg in LEG_ORDER
            }
        return joints

    def complete_physical_test_after_filter(self, raw_target, filtered_target):
        """Keep a diagnostic lease active through its conditioned return."""
        active = getattr(self, "physical_test", None)
        if active is None or not active.get("filter_settling", False):
            return False
        settle_reason = self.conditioned_command_settle_reason(raw_target)
        position_error = max(
            abs(float(raw) - float(filtered))
            for raw, filtered in zip(raw_target, filtered_target)
        )
        if settle_reason or position_error > FINITE_MOTION_POSITION_TOLERANCE:
            self.motion_active = True
            return False
        completed_mode = active["mode"]
        self.physical_test = None
        self.motion_active = False
        self.get_logger().info(
            "Physical test '%s' completed and conditioned at supported stand."
            % completed_mode
        )
        return True

    def serial_status_callback(self, message):
        """Extract the bridge's measured FRAME rate for unified diagnostics."""
        frame_rate = None
        try:
            decoded = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            decoded = None
        if isinstance(decoded, dict):
            frame_rate = decoded.get("frame_rate")
        else:
            for token in str(message.data).split():
                key, separator, value = token.partition("=")
                if separator and key.strip().lower() == "frame_rate":
                    frame_rate = value
                    break
        try:
            frame_rate = float(frame_rate)
        except (TypeError, ValueError):
            return
        if math.isfinite(frame_rate) and frame_rate >= 0.0:
            self.arduino_frame_rate = frame_rate

    def reset_fast_trot_cycle_diagnostics(self):
        self.fast_trot_completed_cycles = 0
        self.fast_trot_last_cycle_phase = None
        self.fast_trot_stance_start_x = {
            leg: None for leg in LEG_ORDER
        }
        self.fast_trot_stance_last_x = {
            leg: None for leg in LEG_ORDER
        }
        self.fast_trot_stance_direction = {
            leg: 1.0 for leg in LEG_ORDER
        }
        self.fast_trot_stance_max_ground_error = {
            leg: 0.0 for leg in LEG_ORDER
        }
        self.fast_trot_stance_active = {
            leg: False for leg in LEG_ORDER
        }
        self.fast_trot_stance_completion_pending = set()
        self.fast_trot_completed_stance_strides = {}
        self.fast_trot_completed_stance_ground_errors = {}
        self.fast_trot_swing_max_clearance = {
            leg: 0.0 for leg in LEG_ORDER
        }
        self.fast_trot_completed_swing_clearances = {}
        self.fast_trot_cycle_joint_ranges = [
            [float("inf"), float("-inf")] for _ in JOINT_NAMES
        ]
        self.fast_trot_achieved_stride = 0.0
        self.fast_trot_signed_stride = 0.0
        self.fast_trot_stance_grounded = False
        self.fast_trot_max_stance_ground_error = 0.0
        self.fast_trot_achieved_step_height = 0.0
        self.fast_trot_cycle_requested_stride = 0.0
        self.fast_trot_completed_requested_stride = 0.0
        self.fast_trot_cycle_max_abs_yaw = 0.0
        self.fast_trot_stride_metric_valid = True
        self.fast_trot_max_joint_excursion_deg = 0.0
        self.fast_trot_joint_excursions_deg = {
            name: 0.0 for name in JOINT_NAMES
        }
        self.fast_trot_diagnostic_active = False
        self.fast_trot_measurement_ready = False
        self.joint_velocity_clamp_counts = [0 for _ in JOINT_NAMES]
        self.joint_braking_clamp_counts = [0 for _ in JOINT_NAMES]
        self.joint_acceleration_clamp_counts = [0 for _ in JOINT_NAMES]
        self.joint_delta_clamp_counts = [0 for _ in JOINT_NAMES]
        self.ik_projection_count = 0
        self.joint_limit_clamp_count = 0

    def body_pose_callback(self, message):
        values = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.linear.z),
            float(message.angular.x),
            float(message.angular.y),
            float(message.angular.z),
        )
        if not all(math.isfinite(value) for value in values):
            self.warning = "Rejected non-finite body-pose command."
            return
        if self.command_owner != "MOTION":
            return
        if (
            self.transition is not None
            or self.pending_pose_action is not None
            or self.state != "standing"
        ):
            current = (
                self.body_x,
                self.body_y,
                self.body_height,
                self.body_roll,
                self.body_pitch,
                self.body_yaw,
            )
            if any(
                abs(float(requested) - float(active)) > 1e-9
                for requested, active in zip(values, current)
            ):
                self.warning = (
                    "Body-pose command rejected during a pose transition or "
                    "outside the standing state."
                )
                self.get_logger().warning(
                    self.warning,
                    throttle_duration_sec=1.0,
                )
            return
        if getattr(self, "physical_test", None) is not None:
            self.warning = (
                "Body-pose command rejected while a finite physical test "
                "owns the neutral body pose."
            )
            self.get_logger().warning(
                self.warning,
                throttle_duration_sec=1.0,
            )
            return
        if self.emote_playback_active() or self.emote_request_pending():
            self.warning = (
                "Body-pose command rejected while the emote engine owns "
                "the Cartesian target."
            )
            self.get_logger().warning(
                self.warning,
                throttle_duration_sec=1.0,
            )
            return
        if self.gait_name == "fast_trot":
            changed = self.enforce_fast_trot_body_pose()
            non_neutral_request = any(
                abs(value) > 1e-9
                for value in (
                    values[0],
                    values[1],
                    values[2] - self.neutral_body_height,
                    values[3],
                    values[4],
                    values[5],
                )
            )
            if changed or non_neutral_request:
                self.warning = (
                    "FAST TROT owns body posture; ignored external "
                    "body-pose command."
                )
            elif self.warning.startswith("FAST TROT owns body posture"):
                self.warning = ""
            return
        self.body_x = clamp(values[0], -0.025, 0.025)
        self.body_y = clamp(values[1], -0.020, 0.020)
        self.body_height = clamp(values[2], 0.175, 0.220)
        self.body_roll, self.body_pitch = self.bounded_body_attitude(
            values[3],
            values[4],
        )
        self.body_yaw = clamp(values[5], -0.18, 0.18)

    def bounded_body_attitude(self, roll, pitch, gait_name=None):
        """Apply the active gait's body-attitude limits before gait startup."""
        gait_name = gait_name or self.gait_name
        gait_config = self.gait_configs.get(gait_name, {})
        roll_limit = min(
            0.16,
            float(gait_config.get("maximum_body_roll", 0.16)),
        )
        pitch_limit = min(
            0.16,
            float(gait_config.get("maximum_body_pitch", 0.16)),
        )
        return (
            clamp(float(roll), -roll_limit, roll_limit),
            clamp(float(pitch), -pitch_limit, pitch_limit),
        )

    def enforce_fast_trot_body_pose(self):
        """Keep manual body-pose offsets out of the validated trot workspace."""
        if self.gait_name != "fast_trot":
            return False
        safe_values = (
            self.neutral_body_height,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        current_values = (
            self.body_height,
            self.body_x,
            self.body_y,
            self.body_roll,
            self.body_pitch,
            self.body_yaw,
        )
        changed = any(
            abs(float(current) - float(safe)) > 1e-12
            for current, safe in zip(current_values, safe_values)
        )
        (
            self.body_height,
            self.body_x,
            self.body_y,
            self.body_roll,
            self.body_pitch,
            self.body_yaw,
        ) = safe_values
        return changed

    def action_callback(self, message):
        action = message.data.strip().lower()
        if action == "stand":
            self.start_stand_transition()
        elif action == "sit":
            self.start_sit_transition()
        elif action == "stop":
            self.stop_motion()
        elif action == "step_keepalive":
            if (
                self.step_in_place
                and self.command_owner == "MOTION"
                and self.state == "standing"
            ):
                self.last_step_keepalive_time = self.now_seconds()
        elif action == "step":
            step_allowed = (
                self.command_owner == "MOTION"
                and self.state == "standing"
                and self.transition is None
                and self.pending_pose_action is None
                and self.pending_gait is None
                and getattr(self, "physical_test", None) is None
                and not self.emote_playback_active()
                and not self.emote_request_pending()
                and self.velocity_gate_state != VELOCITY_GATE_AWAIT_NEUTRAL
            )
            if not step_allowed:
                self.step_in_place = False
                self.warning = (
                    "Step action requires MOTION ownership and an idle "
                    "standing pose."
                )
                return
            if self.velocity_gate_state == VELOCITY_GATE_AWAIT_MOTION:
                # A STEP click is itself the fresh motion command, but only
                # after a neutral Twist has completed the first gate stage.
                self.velocity_gate_state = VELOCITY_GATE_OPEN
            self.step_in_place = not self.step_in_place
            if self.step_in_place:
                self.last_step_keepalive_time = self.now_seconds()
                if self.warning.startswith("Step-in-place keepalive expired"):
                    self.warning = ""
            else:
                self.gait_controller.request_stop()
        elif action == "debug_on":
            self.debug_gait = True
            self.get_logger().info("Gait debug logging enabled.")
        elif action == "debug_off":
            self.debug_gait = False
            self.get_logger().info("Gait debug logging disabled.")
        else:
            self.get_logger().warning("Unknown action '%s'." % action)

    @staticmethod
    def velocity_is_neutral(values):
        return (
            abs(float(values[0])) <= 1e-4
            and abs(float(values[1])) <= 1e-4
            and abs(float(values[2])) <= 1e-3
        )

    @staticmethod
    def parse_command_owner(status_text):
        """Extract a validated router owner from key/value or JSON status."""
        text = str(status_text).strip()
        if not text:
            return None
        try:
            decoded = json.loads(text)
        except (TypeError, ValueError, json.JSONDecodeError):
            decoded = None
        if isinstance(decoded, dict):
            owner = str(decoded.get("owner", "")).strip().upper()
        else:
            owner = ""
            for token in text.replace(",", " ").split():
                key, separator, value = token.partition("=")
                if separator and key.strip().lower() == "owner":
                    owner = value.strip().upper()
                    break
        return owner if owner in COMMAND_OWNERS else None

    def command_router_status_callback(self, message):
        owner = self.parse_command_owner(message.data)
        if owner is None:
            self.get_logger().warning(
                "Ignoring command-router status without a valid owner.",
                throttle_duration_sec=2.0,
            )
            return
        previous_owner = self.command_owner
        self.command_owner = owner
        if owner == previous_owner:
            return
        if owner == "MOTION":
            self.require_neutral_velocity()
            self.resync_motion_hold(self.now_seconds())
            self.get_logger().info(
                "MOTION ownership enabled; waiting for neutral then fresh velocity."
            )
        else:
            self.cancel_motion_for_owner_loss(
                self.now_seconds(),
                force_hold=previous_owner == "MOTION",
            )
            self.get_logger().info(
                "Motion suspended because command owner is %s." % owner
            )

    def require_neutral_velocity(self):
        self.requested_velocity = [0.0, 0.0, 0.0]
        self.filtered_velocity = [0.0, 0.0, 0.0]
        self.velocity_gate_state = VELOCITY_GATE_AWAIT_NEUTRAL
        self.resume_after_velocity_sequence = self.velocity_command_sequence

    def cancel_pending_gait_switch(self):
        self.pending_gait = None
        self.requested_gait = self.gait_name

    def restore_configured_body_pose(self):
        """Restore the backend's stopped base pose after ownership is lost.

        A hardware profile may intentionally include a small COM offset.  The
        router's HOLD state must clear transient GUI/emote offsets without
        silently changing that applied profile.  Simulation retains its
        historical neutral-height, zero-offset behavior.
        """
        self.body_height = self.neutral_body_height
        if not getattr(self, "hardware_mode", False):
            self.body_x = 0.0
            self.body_y = 0.0
            self.body_roll = 0.0
            self.body_pitch = 0.0
            self.body_yaw = 0.0
            return

        tuning = getattr(self, "applied_real_tuning", {})
        self.body_height = float(
            tuning.get("body_height", self.neutral_body_height)
        )
        self.body_x = float(tuning.get("body_x", 0.0))
        self.body_y = float(tuning.get("body_y", 0.0))
        self.body_roll = math.radians(
            float(tuning.get("body_roll_deg", 0.0))
        )
        self.body_pitch = math.radians(
            float(tuning.get("body_pitch_deg", 0.0))
        )
        self.body_yaw = math.radians(
            float(tuning.get("body_yaw_deg", 0.0))
        )

    def cancel_motion_for_owner_loss(self, now, force_hold=False):
        """Atomically discard all motion that could resume under a new owner."""
        self.require_neutral_velocity()
        self.step_in_place = False
        self.motion_active = False
        self.pending_pose_action = None
        self.transition = None
        self.natural_sit_plan = None
        self.physical_test = None
        self.force_reset_emote(
            "Command ownership changed; router HOLD is authoritative."
        )
        self.auto_ready_pending = False
        self.cancel_pending_gait_switch()
        self.restore_configured_body_pose()
        self.resync_motion_hold(now)
        if force_hold and self.commanded_positions is not None:
            self.state = "hold"

    def current_hold_feet(self, positions=None):
        if positions is None:
            positions = self.measured_positions
        if positions is None:
            return None
        return joint_positions_to_feet(
            positions,
            height=self.body_height,
            body_x=self.body_x,
            body_y=self.body_y,
            roll=self.body_roll,
            pitch=self.body_pitch,
            yaw=self.body_yaw,
        )

    def resync_motion_hold(self, now):
        """Make finite measured feedback the sole restart pose."""
        hold_positions = (
            self.measured_positions
            if self.measured_positions is not None
            else self.commanded_positions
        )
        if hold_positions is None:
            self.commanded_velocities = (
                None
                if self.commanded_positions is None
                else [0.0 for _ in JOINT_NAMES]
            )
            self.motion_active = False
            return
        # Measured feedback always wins.  Explicit open-loop hardware has no
        # feedback, so its last finite rate-limited command is the only safe
        # restart seed and remains visibly flagged as assumed in status.
        self.commanded_positions = list(hold_positions)
        self.commanded_velocities = [0.0 for _ in JOINT_NAMES]
        preserve_open_loop_standing = (
            self.open_loop_hardware
            and self.measured_positions is None
            and self.state == "standing"
        )
        if not preserve_open_loop_standing:
            self.state = "hold"
        self.motion_active = False
        try:
            feet = self.current_hold_feet(hold_positions)
            self.standing_feet = {
                leg: tuple(feet[leg]) for leg in LEG_ORDER
            }
            self.gait_controller.hold_current_feet(feet, now)
        except (AttributeError, KinematicsError, ValueError) as exc:
            # AttributeError is retained as a compatibility fallback for an
            # externally supplied older gait controller.  The in-tree gait
            # controller implements the atomic hold API.
            self.gait_controller.request_stop()
            self.warning = "Could not resync gait hold: %s" % exc

    def joint_state_callback(self, message):
        # Ignition may be used as a shadow visualizer while this controller
        # drives the physical robot. Hobby servos provide no feedback, so
        # simulator JointState data must never be mistaken for physical state.
        if self.open_loop_hardware:
            return
        by_name = dict(zip(message.name, message.position))
        if not all(name in by_name for name in JOINT_NAMES):
            return
        measured = [float(by_name[name]) for name in JOINT_NAMES]
        if not all(math.isfinite(value) for value in measured):
            self.warning = "Rejected non-finite joint feedback."
            return
        self.measured_positions = measured
        self.last_joint_state_time = self.now_seconds()
        if self.commanded_positions is None:
            self.commanded_positions = list(self.measured_positions)
            self.commanded_velocities = [0.0 for _ in JOINT_NAMES]
            self.state = "hold"
            self.get_logger().info("Joint feedback received; holding current pose.")
            if self.auto_ready_pose and not self.auto_ready_requested:
                self.auto_ready_requested = True
                self.auto_ready_pending = True
                self.get_logger().info("Auto-starting walk-ready pose.")
        elif self.command_owner != "MOTION":
            # Manual/calibration sources may move the robot.  Follow measured
            # feedback without filtering so a later handoff starts exactly
            # where the hardware actually is.
            self.resync_motion_hold(self.now_seconds())

    def stop_motion(self):
        self.require_neutral_velocity()
        self.step_in_place = False
        self.pending_pose_action = None
        if self.transition is not None:
            self.transition = None
            self.natural_sit_plan = None
            self.state = "hold"
            self.warning = (
                "STOP cancelled the pose transition; holding the current "
                "conditioned command."
            )
        self.gait_controller.request_stop()
        now = self.now_seconds()
        self.cancel_physical_test("STOP command", now)
        self.cancel_emote("STOP command", now)

    def expire_step_if_stale(self, now):
        """Stop persistent bench stepping if its GUI/operator lease disappears."""
        if (
            self.step_in_place
            and now - self.last_step_keepalive_time > self.step_keepalive_timeout
        ):
            self.step_in_place = False
            self.gait_controller.request_stop()
            self.warning = (
                "Step-in-place keepalive expired; completing touchdown and stopping."
            )
            return True
        return False

    def start_stand_transition(self):
        if self.commanded_positions is None:
            self.warning = "Cannot stand until joint feedback is available."
            return
        if self.command_owner != "MOTION":
            self.warning = "Stand action requires MOTION command ownership."
            return
        if self.transition is not None:
            self.warning = "STOP the active pose transition before Stand Up."
            return
        if (
            self.state == "standing"
            and not self.motion_active
            and not self.step_in_place
            and not self.gait_controller.active
            and getattr(self, "physical_test", None) is None
            and not self.emote_playback_active()
            and not self.emote_request_pending()
        ):
            self.get_logger().info("Stand ignored: already at stopped stand.")
            return
        self.stop_motion()
        self.cancel_pending_gait_switch()
        if getattr(self, "physical_test", None) is not None:
            self.pending_pose_action = "stand"
            self.get_logger().info(
                "Stand transition queued until the physical test returns "
                "to its supported neutral pose."
            )
            return
        if self.emote_playback_active():
            self.pending_pose_action = "stand"
            self.get_logger().info(
                "Stand transition queued until the emote returns to stand."
            )
            return
        if self.gait_controller.active:
            self.pending_pose_action = "stand"
            self.get_logger().info(
                "Stand transition queued until the active gait is grounded."
            )
            return
        self.begin_stand_transition()

    def begin_stand_transition(self):
        duration = float(self.get_parameter("stand_duration").value)
        natural_plan = getattr(self, "natural_sit_plan", None)
        if self.state == "sitting" and isinstance(natural_plan, dict):
            try:
                sitting_target, _diagnostics = self.solve_cartesian_pose(
                    natural_plan["sitting"]
                )
                close_to_natural_sit = max(
                    abs(float(current) - float(expected))
                    for current, expected in zip(
                        self.commanded_positions,
                        sitting_target,
                    )
                ) < 0.35
            except (KinematicsError, KeyError, TypeError, ValueError):
                close_to_natural_sit = False
            if close_to_natural_sit:
                scale = duration / 4.0
                waypoints = [
                    (0.40 * scale, natural_plan["sitting"]),
                    (1.55 * scale, natural_plan["bend"]),
                    (2.45 * scale, natural_plan["shift"]),
                    (3.55 * scale, natural_plan["standing"]),
                    (4.00 * scale, natural_plan["standing"]),
                ]
                if self.begin_cartesian_transition(
                    waypoints,
                    "standing",
                    "natural_stand",
                    start_pose=natural_plan["sitting"],
                ):
                    self.state = "standing_up"
                    self.get_logger().info(
                        "Standing from the planted natural sit through Cartesian IK."
                    )
                return
        sit_error = max(
            abs(current - sit)
            for current, sit in zip(self.commanded_positions, SIT_POSE)
        )
        if sit_error < 0.35:
            scale = duration / 4.0
            waypoints = [
                (0.45 * scale, SIT_POSE),
                (1.20 * scale, LEG_FOOT_SIT_POSE),
                (2.55 * scale, LEG_FOOT_MID_POSE),
                (4.00 * scale, WALK_POSE),
            ]
        else:
            waypoints = [(duration, WALK_POSE)]
        self.begin_transition(waypoints, "standing")
        self.state = "standing_up"
        self.get_logger().info("Standing up.")

    def start_sit_transition(self):
        if self.commanded_positions is None:
            self.warning = "Cannot sit until joint feedback is available."
            return
        if self.command_owner != "MOTION":
            self.warning = "Sit action requires MOTION command ownership."
            return
        if self.transition is not None:
            self.warning = "STOP the active pose transition before Sit."
            return
        if self.state == "sitting":
            self.get_logger().info("Sit ignored: already sitting.")
            return
        if self.state != "standing":
            self.warning = "Sit requires the stopped standing state."
            return
        self.stop_motion()
        self.cancel_pending_gait_switch()
        if getattr(self, "physical_test", None) is not None:
            self.pending_pose_action = "sit"
            self.get_logger().info(
                "Sit transition queued until the physical test returns "
                "to its supported neutral pose."
            )
            return
        if self.emote_playback_active():
            self.pending_pose_action = "sit"
            self.get_logger().info(
                "Sit transition queued until the emote returns to stand."
            )
            return
        if self.gait_controller.active:
            self.pending_pose_action = "sit"
            self.get_logger().info(
                "Sit transition queued until the active gait is grounded."
            )
            return
        settle_reason = self.sit_start_settle_reason()
        if settle_reason:
            self.pending_pose_action = "sit"
            self.warning = "Sit waiting: %s." % settle_reason
            return
        self.begin_sit_transition()

    def sit_start_settle_reason(self):
        """Require the conditioned planted stand before beginning Sit."""
        try:
            expected, diagnostics = self.solve_cartesian_pose(
                self.current_cartesian_pose()
            )
        except (KinematicsError, KeyError, TypeError, ValueError) as exc:
            return "standing target is invalid: %s" % exc
        if diagnostics["projected_targets"]:
            return "standing target requires IK projection"
        return self.conditioned_command_settle_reason(expected)

    def begin_sit_transition(self):
        duration = float(self.get_parameter("sit_duration").value)
        scale = duration / 4.7
        standing = self.current_cartesian_pose()
        standing_body = standing["body"]
        shift_body = dict(standing_body)
        shift_body["body_x"] = clamp(
            standing_body["body_x"]
            - getattr(self, "natural_sit_rearward_shift", 0.020),
            -0.025,
            0.025,
        )
        final_body = dict(shift_body)
        final_body["height"] = min(
            standing_body["height"],
            getattr(self, "natural_sit_height", 0.145),
        )
        final_body["pitch"] = getattr(
            self,
            "natural_sit_pitch",
            math.radians(-10.0),
        )
        bend_body = self.interpolate_body_target(
            shift_body,
            final_body,
            0.45,
        )

        def pose(body):
            return {
                "body": dict(body),
                "feet": {
                    leg: tuple(standing["feet"][leg])
                    for leg in LEG_ORDER
                },
            }

        shift = pose(shift_body)
        bend = pose(bend_body)
        sitting = pose(final_body)
        waypoints = [
            (0.70 * scale, standing),
            (1.45 * scale, shift),
            (2.45 * scale, bend),
            (3.85 * scale, sitting),
            (4.70 * scale, sitting),
        ]
        if not self.begin_cartesian_transition(
            waypoints,
            "sitting",
            "natural_sit",
            start_pose=standing,
        ):
            return
        self.natural_sit_plan = {
            "standing": standing,
            "shift": shift,
            "bend": bend,
            "sitting": sitting,
        }
        self.state = "sitting_down"
        self.get_logger().info(
            "Sitting naturally with planted feet through Cartesian IK."
        )

    def start_pending_pose_transition(self):
        action = self.pending_pose_action
        if action == "sit":
            reason = self.sit_start_settle_reason()
            if reason:
                self.warning = "Sit waiting: %s." % reason
                return
        self.pending_pose_action = None
        if action == "stand":
            self.begin_stand_transition()
        elif action == "sit":
            self.begin_sit_transition()

    def select_gait(self, gait_name, now):
        self.gait_name = gait_name
        self.pending_gait = None
        self.step_in_place = False
        # Clamp while grounded, before a fresh velocity can start the new gait.
        # Joint filtering then makes a tighter profile limit continuous rather
        # than changing attitude on the first airborne frame.
        self.body_roll, self.body_pitch = self.bounded_body_attitude(
            self.body_roll,
            self.body_pitch,
            gait_name,
        )
        if gait_name == "fast_trot":
            self.enforce_fast_trot_body_pose()
        if self.warning.startswith("FAST TROT owns body posture"):
            self.warning = ""
        self.gait_controller.set_gait(gait_name, now)
        if gait_name == "fast_trot":
            self.reset_fast_trot_cycle_diagnostics()
        self.require_neutral_velocity()
        self.get_logger().info("Selected %s gait." % gait_name)

    def begin_transition(self, waypoints, final_state):
        start = (
            list(self.measured_positions)
            if self.measured_positions is not None
            else list(self.commanded_positions)
        )
        self.transition = {
            "kind": "joint",
            "start_time": self.now_seconds(),
            "start": start,
            "waypoints": [(float(t), list(pose)) for t, pose in waypoints],
            "final_state": final_state,
            "filter_settling": False,
        }

    def current_cartesian_pose(self):
        """Capture the planted controller target used to begin Sit/Stand."""
        feet = getattr(self, "standing_feet", None)
        if not isinstance(feet, dict) or any(leg not in feet for leg in LEG_ORDER):
            nominal = getattr(self.gait_controller, "nominal_feet", None)
            feet = nominal() if callable(nominal) else NOMINAL_FEET
        return {
            "body": {
                "height": float(self.body_height),
                "body_x": float(self.body_x),
                "body_y": float(self.body_y),
                "roll": float(self.body_roll),
                "pitch": float(self.body_pitch),
                "yaw": float(self.body_yaw),
            },
            "feet": {leg: tuple(feet[leg]) for leg in LEG_ORDER},
        }

    @staticmethod
    def interpolate_body_target(start, target, proportion):
        values = interpolate(
            [float(start[name]) for name in BODY_TARGET_FIELDS],
            [float(target[name]) for name in BODY_TARGET_FIELDS],
            proportion,
        )
        return dict(zip(BODY_TARGET_FIELDS, values))

    @classmethod
    def interpolate_cartesian_pose(cls, start, target, proportion):
        return {
            "body": cls.interpolate_body_target(
                start["body"],
                target["body"],
                proportion,
            ),
            "feet": {
                leg: tuple(interpolate(
                    start["feet"][leg],
                    target["feet"][leg],
                    proportion,
                ))
                for leg in LEG_ORDER
            },
        }

    @staticmethod
    def solve_cartesian_pose(pose):
        return feet_to_joint_positions_diagnostic(
            pose["feet"],
            **pose["body"],
        )

    def preflight_cartesian_transition(self, start_pose, waypoints):
        """Sample a complete pose transition and reject any IK projection."""
        previous_time = 0.0
        previous_pose = start_pose
        for end_time, end_pose in waypoints:
            end_time = float(end_time)
            if not math.isfinite(end_time) or end_time <= previous_time:
                raise KinematicsError(
                    "Cartesian transition times must be finite and increasing"
                )
            for sample_index in range(13):
                pose = self.interpolate_cartesian_pose(
                    previous_pose,
                    end_pose,
                    sample_index / 12.0,
                )
                _positions, diagnostics = self.solve_cartesian_pose(pose)
                clamped = [
                    joint
                    for leg in LEG_ORDER
                    for joint in diagnostics["legs"][leg].get(
                        "clamped_joints",
                        (),
                    )
                ]
                if diagnostics["projected_targets"] or clamped:
                    raise KinematicsError(
                        "Cartesian %s projected=%s clamped=%s"
                        % (
                            "Sit/Stand preflight",
                            diagnostics["projected_targets"],
                            clamped,
                        )
                    )
                joint_limits = (
                    SHOULDER_LIMIT,
                    LEG_LIMIT,
                    FOOT_LIMIT,
                )
                joint_margin = min(
                    min(angle - limits[0], limits[1] - angle)
                    for leg in LEG_ORDER
                    for angle, limits in zip(
                        diagnostics["legs"][leg]["raw_angles"],
                        joint_limits,
                    )
                )
                reach_margin = min(
                    min(
                        diagnostics["legs"][leg]["input_reach"]
                        - diagnostics["legs"][leg]["minimum_reach"],
                        diagnostics["legs"][leg]["maximum_reach"]
                        - diagnostics["legs"][leg]["input_reach"],
                    )
                    for leg in LEG_ORDER
                )
                if joint_margin < math.radians(4.0) or reach_margin < 0.005:
                    raise KinematicsError(
                        "Sit/Stand preflight margin is too small "
                        "(joint %.1f deg, reach %.1f mm)"
                        % (
                            math.degrees(joint_margin),
                            reach_margin * 1000.0,
                        )
                    )
            previous_time = end_time
            previous_pose = end_pose

    def begin_cartesian_transition(
        self,
        waypoints,
        final_state,
        label,
        start_pose=None,
    ):
        """Begin a sampled, preflighted body/foot transition through IK."""
        if start_pose is None:
            start_pose = self.current_cartesian_pose()
        try:
            self.preflight_cartesian_transition(start_pose, waypoints)
            final_target, _diagnostics = self.solve_cartesian_pose(
                waypoints[-1][1]
            )
        except (KinematicsError, KeyError, TypeError, ValueError) as exc:
            self.warning = "Rejected %s transition: %s" % (label, exc)
            self.get_logger().warning(self.warning)
            return False
        self.transition = {
            "kind": "cartesian",
            "label": str(label),
            "start_time": self.now_seconds(),
            "start_pose": start_pose,
            "waypoints": list(waypoints),
            "final_target": list(final_target),
            "final_state": str(final_state),
            "filter_settling": False,
        }
        return True

    def transition_target(self, now):
        transition = self.transition
        elapsed = now - transition["start_time"]
        segment_start_time = 0.0
        if transition.get("kind") == "cartesian":
            segment_start = transition["start_pose"]
            for segment_end_time, segment_end in transition["waypoints"]:
                if elapsed <= segment_end_time:
                    duration = max(
                        segment_end_time - segment_start_time,
                        1e-6,
                    )
                    pose = self.interpolate_cartesian_pose(
                        segment_start,
                        segment_end,
                        (elapsed - segment_start_time) / duration,
                    )
                    positions, diagnostics = self.solve_cartesian_pose(pose)
                    self.last_gait_body_transform = dict(pose["body"])
                    self.last_gait_feet = {
                        leg: tuple(pose["feet"][leg]) for leg in LEG_ORDER
                    }
                    self.record_ik_diagnostics(diagnostics)
                    return positions
                segment_start_time = segment_end_time
                segment_start = segment_end
            transition["filter_settling"] = True
            self.last_gait_body_transform = dict(segment_start["body"])
            self.last_gait_feet = {
                leg: tuple(segment_start["feet"][leg]) for leg in LEG_ORDER
            }
            return list(transition["final_target"])

        segment_start = transition["start"]

        for segment_end_time, segment_end in transition["waypoints"]:
            if elapsed <= segment_end_time:
                duration = max(segment_end_time - segment_start_time, 1e-6)
                proportion = (elapsed - segment_start_time) / duration
                return interpolate(segment_start, segment_end, proportion)
            segment_start_time = segment_end_time
            segment_start = segment_end

        transition["filter_settling"] = True
        transition["final_target"] = list(segment_start)
        return list(segment_start)

    def complete_pose_transition_after_filter(self, raw_target):
        """Finish Sit/Stand only after the conditioned command has settled."""
        transition = getattr(self, "transition", None)
        if not transition or not transition.get("filter_settling", False):
            return False
        if self.conditioned_command_settle_reason(raw_target):
            return False
        final_state = transition["final_state"]
        self.transition = None
        self.state = final_state
        self.motion_active = False
        if final_state == "standing":
            now = self.now_seconds()
            self.gait_controller.reset(now)
            if transition.get("kind") == "cartesian":
                nominal_feet = transition["waypoints"][-1][1]["feet"]
            else:
                nominal_feet = getattr(
                    self.gait_controller,
                    "nominal_feet",
                    lambda: NOMINAL_FEET,
                )()
            self.standing_feet = {
                leg: tuple(nominal_feet[leg]) for leg in LEG_ORDER
            }
            set_current_feet = getattr(
                self.gait_controller,
                "set_current_feet",
                None,
            )
            if callable(set_current_feet):
                set_current_feet(self.standing_feet)
            self.natural_sit_plan = None
        return True

    def standing_target(self):
        self.last_gait_body_transform = {
            "height": float(self.body_height),
            "body_x": float(self.body_x),
            "body_y": float(self.body_y),
            "roll": float(self.body_roll),
            "pitch": float(self.body_pitch),
            "yaw": float(self.body_yaw),
        }
        self.last_gait_feet = {
            leg: tuple(point)
            for leg, point in getattr(
                self,
                "standing_feet",
                NOMINAL_FEET,
            ).items()
        }
        positions, diagnostics = feet_to_joint_positions_diagnostic(
            getattr(self, "standing_feet", NOMINAL_FEET),
            height=self.body_height,
            body_x=self.body_x,
            body_y=self.body_y,
            roll=self.body_roll,
            pitch=self.body_pitch,
            yaw=self.body_yaw,
        )
        self.record_ik_diagnostics(diagnostics)
        return positions

    def record_ik_diagnostics(self, diagnostics):
        self.ik_diagnostics = diagnostics
        self.projected_targets = list(diagnostics["projected_targets"])
        self.clamped_joints = [
            joint
            for leg_name in LEG_ORDER
            for joint in diagnostics["legs"][leg_name].get(
                "clamped_joints",
                [],
            )
        ]
        if (
            getattr(self, "gait_name", "") == "fast_trot"
            and getattr(
                getattr(self, "gait_controller", None),
                "active",
                False,
            )
        ):
            self.ik_projection_count = getattr(
                self,
                "ik_projection_count",
                0,
            ) + len(self.projected_targets)
            self.joint_limit_clamp_count = getattr(
                self,
                "joint_limit_clamp_count",
                0,
            ) + len(self.clamped_joints)
        reach_margins = []
        for leg_name in LEG_ORDER:
            diagnostic = diagnostics["legs"][leg_name]
            reach = float(diagnostic["input_reach"])
            reach_margins.append(min(
                reach - float(diagnostic["minimum_reach"]),
                float(diagnostic["maximum_reach"]) - reach,
            ))
        self.workspace_margin = min(reach_margins)

    def gait_support_feedback(self, now):
        """Build the pure-gait lift interlock input from prior control output."""
        measured = getattr(self, "measured_positions", None)
        commanded = getattr(self, "commanded_positions", None)
        last_feedback = getattr(self, "last_joint_state_time", None)
        feedback_age = (
            0.0
            if last_feedback is None and measured is not None
            else (
                float("inf")
                if last_feedback is None
                else max(0.0, now - last_feedback)
            )
        )
        feedback_timeout = getattr(self, "support_feedback_timeout", 0.25)
        tracking_available = (
            measured is not None
            and commanded is not None
            and feedback_age <= feedback_timeout
        )
        tracking_error = None
        if tracking_available:
            tracking_error = max(
                abs(command - actual)
                for command, actual in zip(commanded, measured)
            )
        tracking_limit = getattr(self, "support_joint_error_limit", 0.15)
        command_limit = getattr(self, "support_command_error_limit", 0.08)
        tracking_assumed = bool(getattr(self, "open_loop_hardware", False))
        return {
            "command_ready": (
                getattr(self, "gait_command_lag", 0.0) <= command_limit
            ),
            "command_error": getattr(self, "gait_command_lag", 0.0),
            "tracking_required": not tracking_assumed,
            "tracking_available": tracking_available,
            "tracking_ready": (
                tracking_error is not None
                and tracking_error <= tracking_limit
            ),
            "tracking_error": tracking_error,
            "tracking_error_limit": tracking_limit,
            "feedback_age": feedback_age,
            "tracking_assumed": tracking_assumed,
            # Reserved for a future mapping of leg name -> contact boolean.
            "contacts": None,
        }

    def gait_target(self, now, dt):
        if self.gait_name == "fast_trot":
            self.enforce_fast_trot_body_pose()
        if not self.gait_controller.active:
            seed_positions = (
                self.commanded_positions
                if self.commanded_positions is not None
                else self.measured_positions
            )
            if seed_positions is not None:
                self.gait_controller.set_current_feet(
                    joint_positions_to_feet(
                        seed_positions,
                        height=self.body_height,
                        body_x=self.body_x,
                        body_y=self.body_y,
                        roll=self.body_roll,
                        pitch=self.body_pitch,
                        yaw=self.body_yaw,
                    )
                )
        set_feedback = getattr(
            self.gait_controller,
            "set_support_feedback",
            None,
        )
        if callable(set_feedback):
            set_feedback(self.gait_support_feedback(now))
        feet, body_adjustment, active = self.gait_controller.step(
            now,
            dt,
            tuple(self.filtered_velocity),
            self.step_in_place,
            body_offset=(self.body_x, self.body_y),
        )
        self.motion_active = active
        if isinstance(body_adjustment, dict):
            body_x = body_adjustment.get(
                "body_x_override",
                self.body_x,
            ) + body_adjustment.get("x", 0.0)
            body_y = body_adjustment.get(
                "body_y_override",
                self.body_y,
            ) + body_adjustment.get("y", 0.0)
            body_height = self.body_height + body_adjustment.get("height", 0.0)
            body_roll = self.body_roll + body_adjustment.get("roll", 0.0)
            body_pitch = self.body_pitch + body_adjustment.get("pitch", 0.0)
        else:
            body_x = self.body_x + body_adjustment[0]
            body_y = self.body_y + body_adjustment[1]
            body_height = self.body_height
            body_roll = self.body_roll
            body_pitch = self.body_pitch
        body_roll, body_pitch = self.bounded_body_attitude(
            body_roll,
            body_pitch,
        )
        self.last_gait_feet = {
            leg: tuple(feet[leg]) for leg in LEG_ORDER
        }
        if not active:
            self.standing_feet = {
                leg: tuple(feet[leg]) for leg in LEG_ORDER
            }
        self.last_gait_body_transform = {
            "height": float(body_height),
            "body_x": float(body_x),
            "body_y": float(body_y),
            "roll": float(body_roll),
            "pitch": float(body_pitch),
            "yaw": float(self.body_yaw),
        }
        joints, diagnostics = feet_to_joint_positions_diagnostic(
            feet,
            height=body_height,
            body_x=body_x,
            body_y=body_y,
            roll=body_roll,
            pitch=body_pitch,
            yaw=self.body_yaw,
        )
        self.record_ik_diagnostics(diagnostics)
        self.log_gait_debug(now, feet, joints)
        return joints

    def joint_velocity_limit(self, index):
        if getattr(self, "physical_test", None) is not None:
            return min(
                getattr(self, "max_joint_velocity", 4.0),
                JOINT_VELOCITY_LIMITS[index],
                MAX_TEST_JOINT_SPEED,
            )
        if self.hardware_mode and self.emote_playback_active():
            return min(
                getattr(self, "max_joint_velocity", 4.0),
                JOINT_VELOCITY_LIMITS[index],
                math.radians(
                    float(
                        self.applied_real_tuning[
                            "max_joint_velocity_deg_s"
                        ]
                    )
                ),
            )
        if self.hardware_mode:
            gait_configs = getattr(self, "gait_configs", GAITS)
            gait = gait_configs.get(getattr(self, "gait_name", ""), {})
            if gait.get("type") == "physical_trot":
                joint_kind = index % 3
                per_joint_name = (
                    "shoulder_joint_velocity_limit",
                    "upper_leg_joint_velocity_limit",
                    "knee_joint_velocity_limit",
                )[joint_kind]
                profile_limit = math.radians(
                    min(
                        gait["joint_velocity_limit"],
                        gait[per_joint_name],
                    )
                )
            elif "real_tuning" in gait:
                profile_limit = math.radians(
                    float(
                        getattr(self, "applied_real_tuning", {}).get(
                            "max_joint_velocity_deg_s",
                            math.degrees(HARDWARE_JOINT_VELOCITY_LIMIT),
                        )
                    )
                )
            else:
                profile_limit = HARDWARE_JOINT_VELOCITY_LIMIT
        else:
            gait_configs = getattr(self, "gait_configs", GAITS)
            gait = gait_configs.get(getattr(self, "gait_name", ""), {})
            # The fast-trot simulator runs at the deliberately shorter
            # Cartesian cycle period.  Let each URDF joint limit (and the
            # node-wide max_joint_velocity guard) remain authoritative
            # instead of additionally imposing the legacy all-joint
            # 118 deg/s profile cap.  Other gaits retain that cap.
            if gait.get("type") == "physical_trot":
                profile_limit = getattr(self, "max_joint_velocity", 4.0)
            else:
                profile_limit = SIMULATION_JOINT_VELOCITY_LIMIT
        return min(
            getattr(self, "max_joint_velocity", 4.0),
            JOINT_VELOCITY_LIMITS[index],
            profile_limit,
        )

    def joint_acceleration_limit(self, index):
        """Return the backend-aware canonical joint acceleration guard."""
        del index  # Reserved for future per-joint acceleration limits.
        gait_configs = getattr(self, "gait_configs", GAITS)
        gait = gait_configs.get(getattr(self, "gait_name", ""), {})
        if self.hardware_mode and self.emote_playback_active():
            profile_limit = math.radians(
                float(
                    self.applied_real_tuning[
                        "max_joint_acceleration_deg_s2"
                    ]
                )
            )
        elif gait.get("type") == "physical_trot":
            profile_limit = float(
                gait.get(
                    "joint_acceleration_limit",
                    SIMULATION_FAST_TROT_JOINT_ACCELERATION_LIMIT,
                )
            )
        elif self.hardware_mode and "real_tuning" in gait:
            profile_limit = math.radians(
                float(
                    getattr(self, "applied_real_tuning", {}).get(
                        "max_joint_acceleration_deg_s2",
                        math.degrees(DEFAULT_JOINT_ACCELERATION_LIMIT),
                    )
                )
            )
        else:
            profile_limit = DEFAULT_JOINT_ACCELERATION_LIMIT
        return min(
            getattr(
                self,
                "max_joint_acceleration",
                DEFAULT_JOINT_ACCELERATION_LIMIT,
            ),
            profile_limit,
        )

    def smooth_joint_target(self, target, dt):
        """Apply IK smoothing, velocity limits, and acceleration limits.

        The IK smoothing step is exponential:
        smoothedAngle = previousAngle + alpha * (targetAngle - previousAngle).
        A small alpha, default 0.12, removes sharp knee/shoulder snaps before
        the velocity and acceleration guards protect the simulated servos.
        """
        target = list(target)
        if len(target) != len(JOINT_NAMES):
            raise KinematicsError(
                "joint target must contain exactly %d values" % len(JOINT_NAMES)
            )
        if not all(math.isfinite(float(value)) for value in target):
            raise KinematicsError("joint target contains a non-finite value")
        if self.commanded_positions is None:
            return target
        if self.commanded_velocities is None:
            self.commanded_velocities = [0.0 for _ in target]

        gait_configs = getattr(self, "gait_configs", GAITS)
        gait = gait_configs.get(getattr(self, "gait_name", ""), {})
        default_alpha = getattr(
            self,
            "joint_smoothing_alpha",
            self.joint_smoothing_factor,
        )
        # Per-gait joint_smoothing_alpha is an explicit override. The old
        # internal joint_tracking_alpha key remains accepted by the preserved
        # trot configurations.
        if self.emote_playback_active():
            tracking_alpha = default_alpha
        else:
            tracking_alpha = gait.get(
                "joint_smoothing_alpha",
                gait.get("joint_tracking_alpha", default_alpha),
            )
        tracking_alpha = clamp(tracking_alpha, 0.02, 1.0)

        smoothed = []
        for index, raw_target in enumerate(target):
            current = self.commanded_positions[index]
            acceleration_limit = self.joint_acceleration_limit(index)
            blended_target = current + (
                raw_target - current
            ) * tracking_alpha

            unconstrained_velocity = (
                blended_target - current
            ) / max(dt, 1e-6)
            remaining_error = raw_target - current
            acceleration_step = acceleration_limit * max(dt, 1e-6)
            # Discrete stopping guard.  Unlike sqrt(2*a*error), this reserves
            # the distance travelled during the next controller interval,
            # preventing the limiter itself from creating a target overshoot.
            braking_velocity_limit = math.sqrt(
                max(
                    0.0,
                    acceleration_step * acceleration_step
                    + 2.0 * acceleration_limit * abs(remaining_error),
                )
            ) - acceleration_step
            configured_velocity_limit = self.joint_velocity_limit(index)
            velocity_limit = min(
                configured_velocity_limit,
                braking_velocity_limit,
            )
            desired_velocity = clamp(
                unconstrained_velocity,
                -velocity_limit,
                velocity_limit,
            )
            if abs(unconstrained_velocity) > configured_velocity_limit + 1e-12:
                counts = getattr(
                    self,
                    "joint_velocity_clamp_counts",
                    [0 for _ in JOINT_NAMES],
                )
                counts[index] += 1
                self.joint_velocity_clamp_counts = counts
            if (
                braking_velocity_limit
                < configured_velocity_limit - 1e-12
                and abs(unconstrained_velocity)
                > braking_velocity_limit + 1e-12
            ):
                counts = getattr(
                    self,
                    "joint_braking_clamp_counts",
                    [0 for _ in JOINT_NAMES],
                )
                counts[index] += 1
                self.joint_braking_clamp_counts = counts

            current_velocity = self.commanded_velocities[index]
            velocity_step = acceleration_limit * dt
            unconstrained_velocity_step = desired_velocity - current_velocity
            constrained_velocity_step = clamp(
                unconstrained_velocity_step,
                -velocity_step,
                velocity_step,
            )
            if (
                abs(
                    constrained_velocity_step
                    - unconstrained_velocity_step
                )
                > 1e-12
            ):
                counts = getattr(
                    self,
                    "joint_acceleration_clamp_counts",
                    [0 for _ in JOINT_NAMES],
                )
                counts[index] += 1
                self.joint_acceleration_clamp_counts = counts
            next_velocity = current_velocity + constrained_velocity_step

            next_position = current + next_velocity * dt
            if self.hardware_mode and gait.get("type") == "physical_trot":
                maximum_delta = math.radians(
                    gait["max_joint_command_delta_deg"]
                )
                bounded_position = clamp(
                    next_position,
                    current - maximum_delta,
                    current + maximum_delta,
                )
                if abs(bounded_position - next_position) > 1e-12:
                    counts = getattr(
                        self,
                        "joint_delta_clamp_counts",
                        [0 for _ in JOINT_NAMES],
                    )
                    counts[index] += 1
                    self.joint_delta_clamp_counts = counts
                    next_position = bounded_position
                    next_velocity = (next_position - current) / max(dt, 1e-6)

            self.commanded_velocities[index] = next_velocity
            smoothed.append(next_position)
        return smoothed

    def update_fast_trot_cycle_diagnostics(self, raw_target, filtered_target):
        """Measure signed, grounded stride after smoothing and rate limits."""
        self.last_raw_joint_target = list(raw_target)
        self.last_filtered_joint_target = list(filtered_target)
        if self.gait_name != "fast_trot":
            self.fast_trot_diagnostic_active = False
            return
        if not self.gait_controller.active:
            self.fast_trot_diagnostic_active = False
            return
        if not getattr(self, "fast_trot_diagnostic_active", False):
            self.reset_fast_trot_cycle_diagnostics()
            self.fast_trot_diagnostic_active = True

        debug = self.gait_controller.debug_snapshot()
        phase = float(debug.get("phase", 0.0)) % 1.0
        self.fast_trot_last_cycle_phase = phase

        try:
            commanded_feet = joint_positions_to_feet(
                filtered_target,
                **self.last_gait_body_transform,
            )
        except (KinematicsError, TypeError, ValueError):
            commanded_feet = None

        stance_legs = set(debug.get("stance_legs", []))
        if float(debug.get("startup_scale", 0.0)) < 1.0 - 1e-6:
            return
        if not self.fast_trot_measurement_ready:
            self.fast_trot_measurement_ready = True
            self.fast_trot_stance_active = {
                leg: leg in stance_legs for leg in LEG_ORDER
            }
            return

        ground_tolerance = float(
            self.gait_configs["fast_trot"]["stance_ground_tolerance"]
        )
        planned_velocity = debug.get(
            "planned_velocity",
            (0.0, 0.0, 0.0),
        )
        try:
            planned_yaw = abs(float(planned_velocity[2]))
        except (IndexError, TypeError, ValueError):
            planned_yaw = float("inf")
        self.fast_trot_cycle_max_abs_yaw = max(
            self.fast_trot_cycle_max_abs_yaw,
            planned_yaw,
        )
        if commanded_feet is not None:
            try:
                forward_direction = math.copysign(
                    1.0,
                    float(planned_velocity[0]),
                )
            except (IndexError, TypeError, ValueError):
                forward_direction = 1.0
            for leg_name in LEG_ORDER:
                is_stance = leg_name in stance_legs
                was_stance = self.fast_trot_stance_active[leg_name]
                x_value = float(commanded_feet[leg_name][0])
                z_value = float(commanded_feet[leg_name][2])
                if was_stance and not is_stance:
                    start_x = self.fast_trot_stance_start_x[leg_name]
                    last_x = self.fast_trot_stance_last_x[leg_name]
                    if (
                        start_x is not None
                        and last_x is not None
                        and math.isfinite(start_x)
                        and math.isfinite(last_x)
                    ):
                        direction = self.fast_trot_stance_direction[
                            leg_name
                        ]
                        signed_stride = direction * (start_x - last_x)
                        self.fast_trot_completed_stance_strides[leg_name] = (
                            signed_stride
                        )
                        self.fast_trot_completed_stance_ground_errors[
                            leg_name
                        ] = self.fast_trot_stance_max_ground_error[
                            leg_name
                        ]
                        self.fast_trot_stance_completion_pending.add(
                            leg_name
                        )
                    self.fast_trot_stance_start_x[leg_name] = None
                    self.fast_trot_stance_last_x[leg_name] = None
                    self.fast_trot_stance_max_ground_error[leg_name] = 0.0
                    self.fast_trot_swing_max_clearance[leg_name] = 0.0
                elif not was_stance and is_stance:
                    self.fast_trot_completed_swing_clearances[leg_name] = (
                        self.fast_trot_swing_max_clearance[leg_name]
                    )
                if is_stance:
                    if not was_stance:
                        self.fast_trot_stance_start_x[leg_name] = x_value
                        self.fast_trot_stance_direction[
                            leg_name
                        ] = forward_direction
                        self.fast_trot_stance_max_ground_error[
                            leg_name
                        ] = 0.0
                    self.fast_trot_stance_last_x[leg_name] = x_value
                    ground_error = abs(
                        z_value - NOMINAL_FEET[leg_name][2]
                    )
                    self.fast_trot_stance_max_ground_error[leg_name] = max(
                        self.fast_trot_stance_max_ground_error[leg_name],
                        ground_error,
                    )
                else:
                    self.fast_trot_swing_max_clearance[leg_name] = max(
                        self.fast_trot_swing_max_clearance[leg_name],
                        z_value - NOMINAL_FEET[leg_name][2],
                    )
                self.fast_trot_stance_active[leg_name] = is_stance

        completed_stance_cycle = (
            len(self.fast_trot_stance_completion_pending)
            == len(LEG_ORDER)
        )
        if completed_stance_cycle:
            strides = [
                self.fast_trot_completed_stance_strides[leg]
                for leg in LEG_ORDER
                if leg in self.fast_trot_completed_stance_strides
            ]
            ground_errors = [
                self.fast_trot_completed_stance_ground_errors[leg]
                for leg in LEG_ORDER
                if leg in self.fast_trot_completed_stance_ground_errors
            ]
            swing_clearances = [
                self.fast_trot_completed_swing_clearances[leg]
                for leg in LEG_ORDER
                if leg in self.fast_trot_completed_swing_clearances
            ]
            excursions = [
                bounds[1] - bounds[0]
                for bounds in self.fast_trot_cycle_joint_ranges
                if all(math.isfinite(value) for value in bounds)
            ]
            if len(strides) == len(LEG_ORDER):
                self.fast_trot_signed_stride = min(strides)
            if len(ground_errors) == len(LEG_ORDER):
                self.fast_trot_max_stance_ground_error = max(
                    ground_errors
                )
                self.fast_trot_stance_grounded = (
                    self.fast_trot_max_stance_ground_error
                    <= ground_tolerance
                )
            if len(swing_clearances) == len(LEG_ORDER):
                self.fast_trot_achieved_step_height = min(
                    swing_clearances
                )
            self.fast_trot_achieved_stride = (
                max(0.0, self.fast_trot_signed_stride)
                if self.fast_trot_stance_grounded
                else 0.0
            )
            self.fast_trot_completed_requested_stride = (
                self.fast_trot_cycle_requested_stride
            )
            self.fast_trot_stride_metric_valid = (
                self.fast_trot_cycle_max_abs_yaw
                <= self.fast_trot_stride_metric_yaw_threshold()
            )
            if len(excursions) == len(JOINT_NAMES):
                excursion_degrees = [
                    math.degrees(value) for value in excursions
                ]
                self.fast_trot_joint_excursions_deg = {
                    name: value
                    for name, value in zip(
                        JOINT_NAMES,
                        excursion_degrees,
                    )
                }
                self.fast_trot_max_joint_excursion_deg = max(
                    excursion_degrees,
                    default=0.0,
                )
            self.fast_trot_completed_cycles += 1
            self.fast_trot_stance_completion_pending.clear()
            self.fast_trot_cycle_joint_ranges = [
                [float("inf"), float("-inf")] for _ in JOINT_NAMES
            ]
            self.fast_trot_cycle_requested_stride = 0.0
            self.fast_trot_cycle_max_abs_yaw = 0.0

        for index, value in enumerate(filtered_target):
            bounds = self.fast_trot_cycle_joint_ranges[index]
            bounds[0] = min(bounds[0], float(value))
            bounds[1] = max(bounds[1], float(value))
        self.fast_trot_cycle_requested_stride = max(
            self.fast_trot_cycle_requested_stride,
            float(debug.get("requested_stride", 0.0)),
        )

    def fast_trot_stride_metric_yaw_threshold(self):
        """Bound signed body-X stride reporting to near-straight motion."""
        config = self.gait_configs["fast_trot"]
        yaw_limit = self.gait_velocity_limits(
            "fast_trot",
            config,
            True,
        )[2]
        return max(0.01, 0.10 * abs(float(yaw_limit)))

    def log_gait_debug(self, now, feet, joints):
        if not self.debug_gait:
            return
        minimum_period = 1.0 / max(
            getattr(self, "diagnostic_log_rate", 1.0),
            0.1,
        )
        if now - getattr(self, "last_debug_time", 0.0) < minimum_period:
            return
        self.last_debug_time = now
        debug = self.gait_controller.debug_snapshot()
        feet_debug = {
            leg: tuple(round(value, 3) for value in feet[leg])
            for leg in LEG_ORDER
        }
        joint_debug = {
            name: round(angle, 3)
            for name, angle in zip(JOINT_NAMES, joints)
        }
        self.get_logger().info(
            "time=%.3f gait=%s phase=%d/%s state=%s progress=%.3f "
            "shift=(%.4f,%.4f) swing=%s stance=%s feet=%s joints=%s"
            % (
                now,
                self.gait_name,
                debug.get("phase_index", 0),
                debug.get("phase_name", "cycle"),
                debug.get("step_state", "CYCLE"),
                debug.get("phase_progress", debug["phase"]),
                debug.get("body_shift_x", 0.0),
                debug.get("body_shift_y", 0.0),
                debug["swing_legs"],
                debug["stance_legs"],
                feet_debug,
                joint_debug,
            )
        )

    def update_filtered_velocity(self, now, dt):
        command_timed_out = (
            now - self.last_velocity_time > self.command_timeout
        )
        self.command_timed_out = command_timed_out
        if (
            command_timed_out
            and self.gait_controller.active
            and not self.step_in_place
        ):
            self.gait_controller.request_stop()
        if self.pending_gait is not None:
            desired = [0.0, 0.0, 0.0]
        elif self.velocity_gate_state != VELOCITY_GATE_OPEN:
            desired = [0.0, 0.0, 0.0]
        elif self.velocity_command_sequence <= self.resume_after_velocity_sequence:
            desired = [0.0, 0.0, 0.0]
        elif command_timed_out:
            desired = [0.0, 0.0, 0.0]
        else:
            desired = list(self.requested_velocity)

        gait = self.gait_configs[self.gait_name]
        speed_scale = self.active_speed_scale(gait)
        limits = list(self.gait_velocity_limits(self.gait_name, gait, True))
        desired = limit_velocity_command(desired, limits)
        if gait.get("type") in ("spot_walk", "stable_crawl"):
            lateral = abs(desired[1]) / max(limits[1], 1e-9)
            turning = abs(desired[2]) / max(limits[2], 1e-9)
            if lateral > 0.05 and turning > 0.05:
                combined = math.hypot(lateral, turning)
                if combined > 0.65:
                    reduction = 0.65 / combined
                    desired[1] *= reduction
                    desired[2] *= reduction
        acceleration = gait.get("acceleration")
        if acceleration is None:
            rates = [
                max(0.18, gait["max_x"] * 3.0),
                max(0.12, gait["max_y"] * 3.0),
                max(1.20, gait["max_yaw"] * 3.0),
            ]
        else:
            rates = [
                max(0.01, acceleration * speed_scale),
                max(0.008, acceleration * 0.55 * speed_scale),
                max(0.10, gait["max_yaw"] * 2.0 * speed_scale),
            ]
        smoothing = clamp(gait.get("smoothing_factor", 0.15), 0.02, 1.0)
        for index in range(3):
            # Apply the low-pass blend before the acceleration guard.  The old
            # order slew-limited the command and then multiplied that tiny step
            # by ``smoothing``, unintentionally reducing acceleration by another
            # 64-72 percent for the tuned trots.
            smoothed_desired = self.filtered_velocity[index] + (
                desired[index] - self.filtered_velocity[index]
            ) * smoothing
            difference = smoothed_desired - self.filtered_velocity[index]
            maximum_change = rates[index] * dt
            self.filtered_velocity[index] += clamp(
                difference,
                -maximum_change,
                maximum_change,
            )

    def active_speed_scale(self, gait=None):
        """Return the active profile's simulation/hardware command scale."""
        gait = gait or self.gait_configs[self.gait_name]
        if gait.get("type") == "physical_trot":
            if self.hardware_mode:
                gait_controller = getattr(self, "gait_controller", None)
                tuning = getattr(gait_controller, "fast_trot_tuning", None)
                if isinstance(tuning, dict):
                    return float(tuning["hardware_speed_scale"])
                return float(gait["presets"]["bench"]["hardware_speed_scale"])
            return float(gait["simulation_speed_scale"])
        if self.hardware_mode:
            return gait.get(
                "hardware_speed_scale",
                self.gait_configs["spot_walk"]["hardware_speed_scale"],
            )
        return gait.get(
            "simulation_speed_scale",
            self.gait_configs["spot_walk"]["simulation_speed_scale"],
        )

    def gait_velocity_limits(self, gait_name, gait=None, effective=True):
        """Return backend-aware Twist limits without mixing stride scaling."""
        gait = gait or self.gait_configs[gait_name]
        if gait.get("type") == "physical_trot":
            if self.hardware_mode:
                command_max_x = gait["hardware_max_x"]
                gait_controller = getattr(self, "gait_controller", None)
                tuning = getattr(gait_controller, "fast_trot_tuning", None)
                period = (
                    float(tuning["hardware_cycle_period"])
                    if isinstance(tuning, dict)
                    else float(
                        gait["presets"]["bench"]["hardware_cycle_period"]
                    )
                )
            else:
                command_max_x = gait["simulation_max_x"]
                period = gait["simulation_cycle_period"]
            command_limits = (
                command_max_x,
                gait["max_y"],
                gait["max_yaw_step"] / (
                    gait["stance_ratio"] * period
                ),
            )
        else:
            command_limits = (
                gait["max_x"],
                gait["max_y"],
                gait["max_yaw"],
            )
        if not effective:
            return command_limits
        scale = self.active_speed_scale(gait)
        return tuple(value * scale for value in command_limits)

    def effective_gait_limits(self):
        limits = {}
        for name, gait in self.gait_configs.items():
            scale = self.active_speed_scale(gait)
            command_limits = self.gait_velocity_limits(name, gait, False)
            effective_limits = tuple(
                value * scale for value in command_limits
            )
            limits[name] = {
                "max_x": effective_limits[0],
                "max_y": effective_limits[1],
                "max_yaw": effective_limits[2],
                "command_max_x": command_limits[0],
                "command_max_y": command_limits[1],
                "command_max_yaw": command_limits[2],
                "speed_scale": scale,
            }
        return limits

    def filtered_motion_requested(self):
        """Classify filtered velocity using gait-compatible units."""
        gait = self.gait_configs[self.gait_name]
        if gait.get("type") == "stable_crawl":
            return (
                math.hypot(
                    self.filtered_velocity[0],
                    self.filtered_velocity[1],
                )
                > gait["command_deadband_linear"]
                or abs(self.filtered_velocity[2])
                > gait["command_deadband_yaw"]
            )
        if gait.get("type") == "spot_walk":
            scale = self.active_speed_scale(gait)
            effective = {
                "max_x": gait["max_x"] * scale,
                "max_y": gait["max_y"] * scale,
                "max_yaw": gait["max_yaw"] * scale,
            }
            return (
                normalized_velocity_activity(
                    effective,
                    self.filtered_velocity,
                )
                > gait["velocity_deadband"]
            )
        return (
            abs(self.filtered_velocity[0]) > 0.0015
            or abs(self.filtered_velocity[1]) > 0.0015
            or abs(self.filtered_velocity[2]) > 0.025
        )

    def update_loop_timing(self, now, raw_dt):
        """Track real callback cadence without changing the bounded control dt."""
        raw_dt = max(0.0, float(raw_dt))
        expected_period = getattr(
            self,
            "expected_control_period",
            1.0 / max(getattr(self, "control_rate", 100.0), 1.0),
        )
        self.expected_control_period = expected_period
        if not hasattr(self, "control_loop_window_start"):
            self.control_loop_window_start = now
        if not hasattr(self, "command_publish_window_start"):
            self.command_publish_window_start = now
        if not hasattr(self, "command_publish_window_count"):
            self.command_publish_window_count = 0
        self.control_loop_dt_s = raw_dt
        self.control_loop_max_dt_s = max(
            getattr(self, "control_loop_max_dt_s", 0.0),
            raw_dt,
        )
        if raw_dt > 1.5 * expected_period:
            self.missed_deadlines = getattr(self, "missed_deadlines", 0) + max(
                1,
                int(raw_dt / expected_period) - 1,
            )
        self.control_loop_window_count = getattr(
            self,
            "control_loop_window_count",
            0,
        ) + 1
        window_start = self.control_loop_window_start
        elapsed = now - window_start
        if elapsed >= 1.0:
            self.control_loop_rate_hz = (
                self.control_loop_window_count / max(elapsed, 1e-9)
            )
            self.control_loop_window_start = now
            self.control_loop_window_count = 0

        publish_start = self.command_publish_window_start
        publish_elapsed = now - publish_start
        if publish_elapsed >= 1.0:
            self.command_publish_rate_hz = (
                self.command_publish_window_count
                / max(publish_elapsed, 1e-9)
            )
            self.command_publish_window_start = now
            self.command_publish_window_count = 0

    def note_command_published(self):
        self.command_publish_window_count = getattr(
            self,
            "command_publish_window_count",
            0,
        ) + 1

    def record_joint_command_deltas(self, raw_target, filtered_target):
        previous_filtered = getattr(self, "commanded_positions", None)
        if previous_filtered is None:
            filtered_deltas = [0.0 for _ in JOINT_NAMES]
        else:
            filtered_deltas = [
                math.degrees(abs(current - previous))
                for current, previous in zip(
                    filtered_target,
                    previous_filtered,
                )
            ]
        self.joint_command_delta_deg = dict(
            zip(JOINT_NAMES, filtered_deltas)
        )
        self.maximum_joint_command_delta_deg = max(
            filtered_deltas,
            default=0.0,
        )

        fast_trot_sample = (
            self.gait_name == "fast_trot"
            and bool(getattr(self.gait_controller, "active", False))
        )
        previous_raw = getattr(
            self,
            "last_fast_trot_raw_joint_target",
            [],
        )
        raw_deltas = (
            [
                math.degrees(abs(current - previous))
                for current, previous in zip(raw_target, previous_raw)
            ]
            if (
                fast_trot_sample
                and len(previous_raw) == len(JOINT_NAMES)
            )
            else [0.0 for _ in JOINT_NAMES]
        )
        self.maximum_raw_joint_jump_deg = max(raw_deltas, default=0.0)
        self.last_fast_trot_raw_joint_target = (
            list(raw_target) if fast_trot_sample else []
        )
        knee_indices = (2, 5, 8, 11)
        self.ik_branch_continuous = all(
            float(raw_target[index]) <= FOOT_LIMIT[1] + 1e-9
            for index in knee_indices
        )
        if (
            fast_trot_sample
            and len(previous_raw) == len(JOINT_NAMES)
            and self.maximum_raw_joint_jump_deg
            > getattr(self, "sudden_joint_jump_deg", 10.0)
        ):
            self.get_logger().warning(
                "FAST TROT raw joint target jumped %.1f deg; phase will "
                "remain downstream-limited."
                % self.maximum_raw_joint_jump_deg,
                throttle_duration_sec=1.0,
            )
        if fast_trot_sample and not self.ik_branch_continuous:
            self.get_logger().error(
                "FAST TROT knee branch continuity check failed.",
                throttle_duration_sec=1.0,
            )

    def control_callback(self):
        now = self.now_seconds()
        raw_dt = now - self.last_update_time
        self.update_loop_timing(now, raw_dt)
        dt = clamp(raw_dt, 0.001, 0.100)
        self.last_update_time = now

        if self.commanded_positions is None:
            self.publish_status(now)
            return

        if self.command_owner != "MOTION":
            # This branch intentionally does not call transition_target(),
            # update_filtered_velocity(), or gait_target(): no phase clock may
            # advance while the motion source does not own the router.
            self.cancel_motion_for_owner_loss(now)
            if self.command_publisher.get_subscription_count() > 0:
                message = Float64MultiArray()
                message.data = list(self.commanded_positions)
                self.command_publisher.publish(message)
                self.note_command_published()
            self.update_feedback_warning()
            self.publish_status(now)
            return

        if self.command_publisher.get_subscription_count() == 0:
            self.warning = "Joint command router is not connected."
            self.publish_status(now)
            return
        if self.warning == "Joint command router is not connected.":
            self.warning = ""

        if self.auto_ready_pending and self.state == "hold":
            self.auto_ready_pending = False
            self.start_stand_transition()

        if (
            self.pending_pose_action is not None
            and not self.gait_controller.active
            and self.transition is None
            and getattr(self, "physical_test", None) is None
            and not self.emote_playback_active()
            and not self.emote_request_pending()
        ):
            self.start_pending_pose_transition()

        self.expire_step_if_stale(now)
        self.update_filtered_velocity(now, dt)
        self.expire_emote_if_stale(now)
        self.start_pending_emote(now)

        try:
            if self.transition is not None:
                target = self.transition_target(now)
                self.motion_active = True
            elif self.state == "standing":
                if self.emote_playback_active():
                    target = self.emote_target(now)
                elif getattr(self, "physical_test", None) is not None:
                    target = self.physical_test_target(now)
                else:
                    moving = (
                        self.filtered_motion_requested()
                        or self.step_in_place
                    )
                    if moving or self.gait_controller.active:
                        target = self.gait_target(now, dt)
                    else:
                        target = self.standing_target()
                        self.motion_active = False
            else:
                target = list(self.commanded_positions)
                self.motion_active = False
        except (
            KinematicsError,
            EmoteStateError,
            EmoteValidationError,
            ValueError,
        ) as exc:
            self.warning = "Kinematics rejected target: %s" % exc
            target = list(self.commanded_positions)
            self.motion_active = False

        raw_target = list(target)
        try:
            target = self.smooth_joint_target(target, dt)
        except (KinematicsError, ValueError) as exc:
            self.warning = "Joint filter rejected target: %s" % exc
            target = list(self.commanded_positions)
            self.commanded_velocities = [0.0 for _ in JOINT_NAMES]
        if self.gait_controller.active:
            self.gait_command_lag = max(
                abs(raw - filtered)
                for raw, filtered in zip(raw_target, target)
            )
        else:
            self.gait_command_lag = 0.0
        self.record_joint_command_deltas(raw_target, target)
        self.update_fast_trot_cycle_diagnostics(raw_target, target)
        self.commanded_positions = target
        self.complete_pose_transition_after_filter(raw_target)
        self.complete_emote_after_filter(raw_target, target)
        self.complete_physical_test_after_filter(raw_target, target)

        if (
            self.pending_gait is not None
            and self.pending_pose_action is None
            and self.transition is None
            and not self.gait_controller.active
            and getattr(self, "physical_test", None) is None
            and not self.emote_playback_active()
            and not self.emote_request_pending()
        ):
            self.select_gait(self.pending_gait, now)

        message = Float64MultiArray()
        message.data = target
        self.command_publisher.publish(message)
        self.note_command_published()

        self.update_feedback_warning()
        self.publish_status(now)

    def update_feedback_warning(self):
        if self.measured_positions is None or self.commanded_positions is None:
            return
        maximum_error = max(
            abs(command - measured)
            for command, measured in zip(
                self.commanded_positions,
                self.measured_positions,
            )
        )
        if maximum_error > self.joint_error_limit:
            self.warning = "Large joint tracking error: %.2f rad" % maximum_error
        elif self.warning.startswith("Large joint"):
            self.warning = ""

    def arm_neutral_ready(self):
        """Certify only the stopped calibrated WALK_POSE for open-loop ARM."""
        positions = getattr(self, "commanded_positions", None)
        velocities = getattr(self, "commanded_velocities", None)
        requested_velocity = getattr(self, "requested_velocity", ())
        filtered_velocity = getattr(self, "filtered_velocity", ())
        gait_controller = getattr(self, "gait_controller", None)
        if not (
            getattr(self, "open_loop_hardware", False)
            and getattr(self, "measured_positions", None) is None
            and str(getattr(self, "state", "")).strip().lower()
            in ("standing", "hold")
            and positions is not None
            and velocities is not None
            and len(positions) == len(WALK_POSE)
            and len(velocities) == len(WALK_POSE)
            and all(math.isfinite(value) for value in positions)
            and all(math.isfinite(value) for value in velocities)
            and max(
                abs(value - neutral)
                for value, neutral in zip(positions, WALK_POSE)
            )
            <= ARM_NEUTRAL_TOLERANCE
            and max(abs(value) for value in velocities)
            <= ARM_NEUTRAL_ZERO_TOLERANCE
            and all(
                math.isfinite(value)
                and abs(value) <= ARM_NEUTRAL_ZERO_TOLERANCE
                for value in requested_velocity
            )
            and all(
                math.isfinite(value)
                and abs(value) <= ARM_NEUTRAL_ZERO_TOLERANCE
                for value in filtered_velocity
            )
            and not getattr(self, "motion_active", False)
            and not getattr(self, "step_in_place", False)
            and getattr(self, "transition", None) is None
            and getattr(self, "pending_pose_action", None) is None
            and getattr(self, "physical_test", None) is None
            and not self.emote_playback_active()
            and not self.emote_request_pending()
            and getattr(self, "active_emote_request", None) is None
            and not bool(getattr(gait_controller, "active", False))
            and abs(
                getattr(self, "body_height", 0.0)
                - getattr(self, "neutral_body_height", 0.0)
            )
            <= ARM_NEUTRAL_ZERO_TOLERANCE
            and abs(getattr(self, "body_x", 0.0))
            <= ARM_NEUTRAL_ZERO_TOLERANCE
            and abs(getattr(self, "body_y", 0.0))
            <= ARM_NEUTRAL_ZERO_TOLERANCE
            and abs(getattr(self, "body_roll", 0.0))
            <= ARM_NEUTRAL_ZERO_TOLERANCE
            and abs(getattr(self, "body_pitch", 0.0))
            <= ARM_NEUTRAL_ZERO_TOLERANCE
            and abs(getattr(self, "body_yaw", 0.0))
            <= ARM_NEUTRAL_ZERO_TOLERANCE
        ):
            return False
        return True

    def publish_status(self, now):
        if now - self.last_status_time < 0.10:
            return
        self.last_status_time = now
        maximum_error = None
        if self.measured_positions is not None and self.commanded_positions is not None:
            maximum_error = max(
                abs(command - measured)
                for command, measured in zip(
                    self.commanded_positions,
                    self.measured_positions,
                )
            )
        support_feedback = self.gait_support_feedback(now)
        debug = self.gait_controller.debug_snapshot()
        body_shift = debug.get(
            "body_shift",
            {"x": 0.0, "y": 0.0},
        )
        clamped_joints = list(getattr(self, "clamped_joints", []))
        workspace_margin = getattr(self, "workspace_margin", None)
        minimum_workspace_margin = getattr(
            self,
            "minimum_workspace_margin",
            0.010,
        )
        gait_warning = str(debug.get("warning", ""))
        clamp_warning = (
            "IK joint limits clamped: %s" % ", ".join(clamped_joints)
            if clamped_joints
            else ""
        )
        workspace_warning = (
            "IK workspace margin %.4f m is below %.4f m"
            % (workspace_margin, minimum_workspace_margin)
            if (
                workspace_margin is not None
                and workspace_margin < minimum_workspace_margin
            )
            else ""
        )
        fast_trot_tuning = dict(
            getattr(
                self.gait_controller,
                "fast_trot_tuning",
                {},
            )
        )
        completed_cycles = getattr(self, "fast_trot_completed_cycles", 0)
        if completed_cycles > 0:
            requested_stride = getattr(
                self,
                "fast_trot_completed_requested_stride",
                0.0,
            )
            achieved_stride = getattr(
                self,
                "fast_trot_achieved_stride",
                0.0,
            )
        else:
            requested_stride = float(debug.get("requested_stride", 0.0))
            achieved_stride = 0.0
        stride_ratio = (
            achieved_stride / requested_stride
            if requested_stride > 1e-9 and completed_cycles > 0
            else 1.0
        )
        signed_stride = float(
            getattr(self, "fast_trot_signed_stride", 0.0)
        )
        stance_grounded = bool(
            getattr(self, "fast_trot_stance_grounded", False)
        )
        stance_ground_error = float(
            getattr(
                self,
                "fast_trot_max_stance_ground_error",
                0.0,
            )
        )
        stride_metric_valid = bool(
            getattr(self, "fast_trot_stride_metric_valid", True)
        )
        stride_warning = (
            (
                "FAST TROT stance is not grounded "
                "(maximum commanded height error %.1f mm)"
                % (stance_ground_error * 1000.0)
            )
            if (
                self.gait_name == "fast_trot"
                and completed_cycles > 0
                and not stance_grounded
            )
            else (
                "FAST TROT achieved signed stride %.1f mm is below 80%% "
                "of requested %.1f mm"
                % (signed_stride * 1000.0, requested_stride * 1000.0)
                if (
                    self.gait_name == "fast_trot"
                    and completed_cycles > 0
                    and requested_stride > 1e-9
                    and stride_metric_valid
                    and stride_ratio < 0.80
                )
                else ""
            )
        )
        timing_warning = (
            "Control loop missed %d deadlines; latest dt %.1f ms"
            % (
                getattr(self, "missed_deadlines", 0),
                1000.0 * getattr(self, "control_loop_dt_s", 0.0),
            )
            if (
                self.gait_name == "fast_trot"
                and getattr(self, "control_loop_dt_s", 0.0)
                > 1.5 * getattr(
                    self,
                    "expected_control_period",
                    1.0 / max(getattr(self, "control_rate", 100.0), 1.0),
                )
            )
            else ""
        )
        joint_excursions_deg = dict(
            getattr(self, "fast_trot_joint_excursions_deg", {})
        )
        joint_range_usage_percent = {}
        for index, name in enumerate(JOINT_NAMES):
            if index % 3 == 0:
                lower, upper = SHOULDER_LIMIT
            elif index % 3 == 1:
                lower, upper = LEG_LIMIT
            else:
                lower, upper = FOOT_LIMIT
            joint_range_usage_percent[name] = (
                joint_excursions_deg.get(name, 0.0)
                / math.degrees(upper - lower)
                * 100.0
            )
        velocity_clamp_counts = list(
            getattr(
                self,
                "joint_velocity_clamp_counts",
                [0 for _ in JOINT_NAMES],
            )
        )
        braking_clamp_counts = list(
            getattr(
                self,
                "joint_braking_clamp_counts",
                [0 for _ in JOINT_NAMES],
            )
        )
        acceleration_clamp_counts = list(
            getattr(
                self,
                "joint_acceleration_clamp_counts",
                [0 for _ in JOINT_NAMES],
            )
        )
        delta_clamp_counts = list(
            getattr(
                self,
                "joint_delta_clamp_counts",
                [0 for _ in JOINT_NAMES],
            )
        )
        command_age = max(0.0, now - self.last_velocity_time)
        physical_test = getattr(self, "physical_test", None)
        if physical_test is None:
            physical_test_mode = ""
            physical_test_request_id = ""
            physical_test_progress = 0.0
            physical_test_keepalive_age = None
            physical_test_returning = False
            physical_test_settling = False
        else:
            physical_test_mode = physical_test["mode"]
            physical_test_request_id = physical_test["request_id"]
            physical_test_progress = clamp(
                (now - physical_test["start_time"])
                / max(physical_test["duration"], 1e-9),
                0.0,
                1.0,
            )
            physical_test_keepalive_age = max(
                0.0,
                now - physical_test["last_keepalive_time"],
            )
            physical_test_returning = (
                physical_test["cancel_start_time"] is not None
            )
            physical_test_settling = bool(
                physical_test.get("filter_settling", False)
            )
        engine = getattr(self, "emote_engine", None)
        engine_status = (
            engine.status()
            if engine is not None
            else {
                "state": "idle",
                "active": False,
                "emote": "",
                "repetitions": 1,
                "speed": 1.0,
                "amplitude": 1.0,
                "depth": 1.0,
            }
        )
        emote_filter_settling = bool(
            getattr(self, "emote_filter_settling", False)
        )
        emote_active = bool(engine_status["active"]) or emote_filter_settling
        pending_emote = getattr(self, "pending_emote_request", None)
        emote_pending = pending_emote is not None
        leased_emote_request = (
            getattr(self, "active_emote_request", None) or pending_emote
        )
        emote_keepalive_age = (
            max(
                0.0,
                now
                - float(
                    leased_emote_request.get("last_keepalive_time", now)
                ),
            )
            if leased_emote_request is not None
            else None
        )
        if emote_filter_settling:
            active_request = getattr(self, "active_emote_request", None) or {}
            emote_name = str(
                active_request.get("name", engine_status["emote"])
            )
            emote_state = "settling"
        elif emote_active:
            emote_name = str(engine_status["emote"])
            emote_state = str(engine_status["state"])
        elif emote_pending:
            emote_name = str(pending_emote["name"])
            emote_state = "queued"
        else:
            emote_name = ""
            emote_state = str(engine_status["state"])
        emote_swing_legs = (
            list(getattr(self, "emote_swing_legs", ()))
            if emote_active
            else []
        )
        emote_stance_legs = [
            leg for leg in LEG_ORDER if leg not in emote_swing_legs
        ]
        displayed_phase_name = (
            "emote_%s" % emote_name
            if emote_active
            else debug.get("phase_name", "cycle")
        )
        displayed_phase_progress = (
            float(getattr(self, "emote_progress", 0.0))
            if emote_active
            else debug.get("phase_progress", 0.0)
        )
        displayed_per_leg_phase = (
            {
                leg: (
                    "emote_swing"
                    if leg in emote_swing_legs
                    else "emote_support"
                )
                for leg in LEG_ORDER
            }
            if emote_active
            else debug.get("per_leg_phase", {})
        )
        pose_transition = getattr(self, "transition", None)
        pose_transition_active = pose_transition is not None
        pose_transition_duration = (
            float(pose_transition["waypoints"][-1][0])
            if pose_transition_active and pose_transition.get("waypoints")
            else 0.0
        )
        pose_transition_progress = (
            clamp(
                (now - float(pose_transition["start_time"]))
                / max(pose_transition_duration, 1e-6),
                0.0,
                1.0,
            )
            if pose_transition_active
            else 0.0
        )
        status = {
            "joint_names": list(JOINT_NAMES),
            "state": self.state,
            "requested_gait": self.requested_gait,
            "active_gait": self.gait_name,
            "pending_gait": self.pending_gait,
            "pending_pose_action": self.pending_pose_action,
            "pose_transition_active": pose_transition_active,
            "pose_transition_kind": (
                str(pose_transition.get("label", pose_transition.get("kind", "")))
                if pose_transition_active
                else ""
            ),
            "pose_transition_progress": pose_transition_progress,
            "pose_transition_settling": bool(
                pose_transition_active
                and pose_transition.get("filter_settling", False)
            ),
            # Retained for older GUI/serial consumers.
            "gait": self.gait_name,
            "moving": self.motion_active,
            "motion_active": self.motion_active,
            "step_in_place": self.step_in_place,
            "physical_tests_enabled": bool(
                getattr(self, "enable_physical_tests", False)
            ),
            "physical_test_active": physical_test is not None,
            "physical_test_mode": physical_test_mode,
            "physical_test_request_id": physical_test_request_id,
            "physical_test_progress": physical_test_progress,
            "physical_test_keepalive_age": physical_test_keepalive_age,
            "physical_test_returning": physical_test_returning,
            "physical_test_settling": physical_test_settling,
            "emote_config_file": getattr(self, "emote_config_file", ""),
            "emotes_available": list(
                getattr(engine, "available_emotes", ())
            ),
            "emote_active": emote_active,
            "emote_pending": emote_pending,
            "emote_name": emote_name,
            "emote_state": emote_state,
            "emote_progress": float(getattr(self, "emote_progress", 0.0)),
            "emote_returning": emote_state == "returning",
            "emote_settling": emote_filter_settling,
            "emote_keepalive_age": emote_keepalive_age,
            "emote_keepalive_timeout": float(
                getattr(self, "emote_keepalive_timeout", 0.75)
            ),
            "emote_request_id": getattr(self, "emote_request_id", ""),
            "emote_result": getattr(self, "emote_result", "idle"),
            "emote_message": getattr(self, "emote_message", ""),
            "emote_options": {
                "repetitions": engine_status["repetitions"],
                "speed": engine_status["speed"],
                "amplitude": engine_status["amplitude"],
                "depth": engine_status["depth"],
            },
            "phase_index": debug.get("phase_index", 0),
            "phase_name": displayed_phase_name,
            "phase_progress": displayed_phase_progress,
            "step_state": debug.get("step_state", "CYCLE"),
            "cycle_phase": debug["phase"],
            "gait_phase": debug["phase"],
            "per_leg_phase": displayed_per_leg_phase,
            "swing_legs": (
                emote_swing_legs if emote_active else debug["swing_legs"]
            ),
            "stance_legs": (
                emote_stance_legs if emote_active else debug["stance_legs"]
            ),
            "body_shift": body_shift,
            "body_shift_x": debug.get("body_shift_x", body_shift["x"]),
            "body_shift_y": debug.get("body_shift_y", body_shift["y"]),
            "body_shift_target_x": debug.get(
                "body_shift_target_x",
                0.0,
            ),
            "body_shift_target_y": debug.get(
                "body_shift_target_y",
                0.0,
            ),
            "support_target_x": debug.get("support_target_x", 0.0),
            "support_target_y": debug.get("support_target_y", 0.0),
            "support_margin": debug.get("support_margin", 0.0),
            "support_clearance": debug.get("support_clearance", 0.0),
            "shift_completion": debug.get("shift_completion", 0.0),
            "support_polygon_valid": debug.get(
                "support_polygon_valid",
                True,
            ),
            "lift_allowed": debug.get("lift_allowed", False),
            "projected_targets": list(self.projected_targets),
            "clamped_joints": clamped_joints,
            "workspace_margin": workspace_margin,
            "minimum_workspace_margin": minimum_workspace_margin,
            "filtered_velocity": list(self.filtered_velocity),
            "requested_velocity": list(self.requested_velocity),
            "command_age": command_age,
            "command_timed_out": bool(
                getattr(self, "command_timed_out", False)
            ),
            "cmd_vel_receive_rate": float(
                getattr(self, "cmd_vel_receive_rate", 0.0)
            ),
            "cmd_vel_message_count": int(
                getattr(self, "velocity_message_count", 0)
            ),
            "cmd_vel_zero_transition_count": int(
                getattr(self, "velocity_zero_transition_count", 0)
            ),
            "command_owner": self.command_owner,
            "motion_authorized": self.command_owner == "MOTION",
            "velocity_gate": self.velocity_gate_state,
            "controller_connected": self.command_publisher.get_subscription_count() > 0,
            "joint_error": maximum_error,
            "warning": "; ".join(
                warning
                for warning in (
                    self.open_loop_warning,
                    self.warning,
                    gait_warning,
                    clamp_warning,
                    workspace_warning,
                    stride_warning,
                    timing_warning,
                )
                if warning
            ),
            "gait_limits": self.effective_gait_limits(),
            "speed_scale": self.active_speed_scale(),
            "hardware_mode": self.hardware_mode,
            "use_sim_time": bool(
                getattr(self, "use_sim_time", False)
            ),
            "gait_config_file": getattr(self, "gait_config_file", ""),
            "fast_trot_config_file": getattr(
                self,
                "fast_trot_config_file",
                "",
            ),
            "fast_trot_profile": (
                "physical"
                if self.hardware_mode
                else "simulation"
            ),
            "real_robot_profiles_file": getattr(
                self,
                "real_profiles_file",
                "",
            ),
            "real_profile": getattr(
                self,
                "active_real_profile",
                "SIMULATION",
            ),
            "real_tuning": dict(
                getattr(self, "applied_real_tuning", {})
            ),
            "real_tuning_profiles": {
                name: dict(values)
                for name, values in getattr(self, "real_profiles", {}).items()
            },
            "real_tuning_bounds": {
                name: list(bounds)
                for name, bounds in NUMERIC_BOUNDS.items()
            },
            "real_tuning_request_id": getattr(
                self,
                "real_tuning_request_id",
                "",
            ),
            "real_tuning_result": getattr(
                self,
                "real_tuning_result",
                "",
            ),
            "real_tuning_message": getattr(
                self,
                "real_tuning_message",
                "",
            ),
            "open_loop_hardware": self.open_loop_hardware,
            "arm_neutral_ready": self.arm_neutral_ready(),
            "joint_velocity_limit_deg_s": min(
                math.degrees(self.joint_velocity_limit(index))
                for index in range(len(JOINT_NAMES))
            ),
            "joint_acceleration_limit_deg_s2": min(
                math.degrees(self.joint_acceleration_limit(index))
                for index in range(len(JOINT_NAMES))
            ),
            "effective_joint_velocity_limits_deg_s": {
                name: math.degrees(self.joint_velocity_limit(index))
                for index, name in enumerate(JOINT_NAMES)
            },
            "effective_joint_acceleration_limits_deg_s2": {
                name: math.degrees(self.joint_acceleration_limit(index))
                for index, name in enumerate(JOINT_NAMES)
            },
            "fast_trot_tuning": fast_trot_tuning,
            "fast_trot_presets": {
                name: dict(values)
                for name, values in self.gait_configs["fast_trot"][
                    "presets"
                ].items()
            },
            "requested_step_height": float(
                fast_trot_tuning.get("step_height", 0.0)
            ),
            "achieved_step_height": float(
                getattr(self, "fast_trot_achieved_step_height", 0.0)
            ),
            "requested_stride": requested_stride,
            "planned_stride": float(debug.get("achieved_stride", 0.0)),
            "achieved_stride": achieved_stride,
            "signed_stride": signed_stride,
            "stride_achievement_ratio": stride_ratio,
            "stride_metric_valid": stride_metric_valid,
            "stride_metric": "signed_body_x_near_straight",
            "stance_grounded": stance_grounded,
            "stance_max_ground_error": stance_ground_error,
            "stance_ground_tolerance": float(
                self.gait_configs["fast_trot"][
                    "stance_ground_tolerance"
                ]
            ),
            "configured_cycle_period": float(
                debug.get("configured_cycle_period", 0.0)
            ),
            "current_cycle_period": float(
                debug.get("current_cycle_period", 0.0)
            ),
            "phase_rate_scale": float(
                debug.get("phase_rate_scale", 1.0)
            ),
            "phase_transition_hold": bool(
                debug.get("phase_transition_hold", False)
            ),
            "current_swing_pair": debug.get(
                "current_swing_pair",
                "none",
            ),
            "maximum_joint_excursion_deg": float(
                getattr(
                    self,
                    "fast_trot_max_joint_excursion_deg",
                    0.0,
                )
            ),
            "joint_excursions_deg": joint_excursions_deg,
            "joint_range_usage_percent": joint_range_usage_percent,
            "joint_velocity_clamp_counts": dict(
                zip(JOINT_NAMES, velocity_clamp_counts)
            ),
            "joint_braking_clamp_counts": dict(
                zip(JOINT_NAMES, braking_clamp_counts)
            ),
            "joint_acceleration_clamp_counts": dict(
                zip(JOINT_NAMES, acceleration_clamp_counts)
            ),
            "joint_delta_clamp_counts": dict(
                zip(JOINT_NAMES, delta_clamp_counts)
            ),
            "joint_velocity_clamp_count": sum(velocity_clamp_counts),
            "joint_braking_clamp_count": sum(braking_clamp_counts),
            "joint_acceleration_clamp_count": sum(
                acceleration_clamp_counts
            ),
            "joint_delta_clamp_count": sum(delta_clamp_counts),
            "joint_limit_clamp_count": int(
                getattr(self, "joint_limit_clamp_count", 0)
            ),
            "ik_projection_count": int(
                getattr(self, "ik_projection_count", 0)
            ),
            "joint_tracking_error": maximum_error,
            "tracking_required": bool(
                support_feedback["tracking_required"]
            ),
            "tracking_available": bool(
                support_feedback["tracking_available"]
            ),
            "tracking_ready": bool(
                support_feedback["tracking_ready"]
            ),
            "tracking_assumed": bool(
                support_feedback["tracking_assumed"]
            ),
            "tracking_feedback_age": (
                float(support_feedback["feedback_age"])
                if math.isfinite(
                    float(support_feedback["feedback_age"])
                )
                else None
            ),
            "arduino_frame_rate": float(
                getattr(self, "arduino_frame_rate", 0.0)
            ),
            "expected_control_rate_hz": float(
                getattr(self, "control_rate", 100.0)
            ),
            "control_loop_rate_hz": float(
                getattr(self, "control_loop_rate_hz", 0.0)
            ),
            "command_publish_rate_hz": float(
                getattr(self, "command_publish_rate_hz", 0.0)
            ),
            "control_loop_dt_s": float(
                getattr(self, "control_loop_dt_s", 0.0)
            ),
            "control_loop_max_dt_s": float(
                getattr(self, "control_loop_max_dt_s", 0.0)
            ),
            "missed_deadlines": int(
                getattr(self, "missed_deadlines", 0)
            ),
            "joint_command_delta_deg": dict(
                getattr(self, "joint_command_delta_deg", {})
            ),
            "maximum_joint_command_delta_deg": float(
                getattr(self, "maximum_joint_command_delta_deg", 0.0)
            ),
            "maximum_raw_joint_jump_deg": float(
                getattr(self, "maximum_raw_joint_jump_deg", 0.0)
            ),
            "ik_branch_continuous": bool(
                getattr(self, "ik_branch_continuous", True)
            ),
            "fast_trot_completed_cycles": completed_cycles,
            "feet": debug.get("feet", {}),
            "commanded_foot_xyz": dict(
                getattr(self, "last_gait_feet", {})
            ),
            "body_world": debug.get("body_world", {}),
            "gait_body_transform": dict(
                getattr(self, "last_gait_body_transform", {})
            ),
            "commanded_body_target": dict(
                getattr(self, "last_gait_body_transform", {})
            ),
            "input_velocity": debug.get("input_velocity", [0.0, 0.0, 0.0]),
            "planned_velocity": debug.get(
                "planned_velocity",
                [0.0, 0.0, 0.0],
            ),
            "raw_joint_target": list(
                getattr(self, "last_raw_joint_target", [])
            ),
            "filtered_joint_target": list(
                getattr(self, "last_filtered_joint_target", [])
            ),
            "raw_to_filtered_joint_error": {
                name: float(raw) - float(filtered)
                for name, raw, filtered in zip(
                    JOINT_NAMES,
                    getattr(self, "last_raw_joint_target", []),
                    getattr(self, "last_filtered_joint_target", []),
                )
            },
        }
        message = String()
        message.data = json.dumps(status, allow_nan=False)
        self.status_publisher.publish(message)


def main():
    rclpy.init()
    node = VoltMotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motion()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
