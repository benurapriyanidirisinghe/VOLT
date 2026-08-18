#!/usr/bin/env python3

"""Safe, stopped-state, one-variable FAST TROT tuning sweeps.

The module is dry-run by default and has a ROS-independent planning layer.
Live application creates exactly one publisher: complete tuning JSON is sent
to the motion controller's existing tuning interface. The tool never creates
motion, ownership, hardware-protocol, or joint-command publishers.
"""

import argparse
import json
import math
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

from volt_gait_controller import (
    FAST_TROT_PRESET_PARAMETER_NAMES,
    FAST_TROT_TUNING_BOUNDS,
    load_fast_trot_config,
    validate_fast_trot_tuning,
)
from volt_kinematics import LEG_ORDER


SUPPORTED_PARAMETERS = tuple(FAST_TROT_PRESET_PARAMETER_NAMES)
MAX_SWEEP_POINTS = 8
TUNING_TOPIC = "/volt/fast_trot_tuning"
STATUS_TOPIC = "/volt/status"
HOLD_OBSERVE_ACKNOWLEDGEMENT = "I WILL HOLD AND OBSERVE VOLT"
DEFAULT_STATUS_TIMEOUT = 4.0
DEFAULT_CONFIRMATION_TIMEOUT = 4.0
DEFAULT_OBSERVE_SECONDS = 2.0
STATUS_FRESHNESS_TIMEOUT = 1.0
TUNING_ABSOLUTE_TOLERANCE = 1e-9


class SweepError(ValueError):
    """Raised when a sweep plan or live safety state is invalid."""


@dataclass(frozen=True)
class SweepPoint:
    """One complete controller tuning with exactly one selected field varied."""

    index: int
    value: float
    tuning: dict


@dataclass(frozen=True)
class SweepPlan:
    """Immutable baseline and ordered complete tuning requests."""

    parameter: str
    values: tuple
    baseline: dict
    points: tuple


def _finite_float(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise SweepError("%s must be numeric" % label) from exc
    if not math.isfinite(result):
        raise SweepError("%s must be finite" % label)
    return result


def validate_value_list(parameter, values):
    """Return a finite, unique, bounded list for one supported field."""
    parameter = str(parameter).strip()
    if parameter not in SUPPORTED_PARAMETERS:
        raise SweepError(
            "parameter must be one of %s" % (SUPPORTED_PARAMETERS,)
        )
    try:
        values = tuple(values)
    except TypeError as exc:
        raise SweepError("values must be an iterable") from exc
    if not values:
        raise SweepError("at least one sweep value is required")
    if len(values) > MAX_SWEEP_POINTS:
        raise SweepError(
            "a sweep may contain at most %d points" % MAX_SWEEP_POINTS
        )
    lower, upper = FAST_TROT_TUNING_BOUNDS[parameter]
    validated = tuple(
        _finite_float(value, "%s value %d" % (parameter, index))
        for index, value in enumerate(values)
    )
    for value in validated:
        if not lower <= value <= upper:
            raise SweepError(
                "%s values must be in [%.3f, %.3f]"
                % (parameter, lower, upper)
            )
    if len(set(validated)) != len(validated):
        raise SweepError("sweep values must not contain duplicates")
    return parameter, validated


def validate_complete_tuning(tuning, config=None):
    """Validate an exact, complete four-field tuning without clamping."""
    if not isinstance(tuning, dict):
        raise SweepError("fast_trot_tuning echo must be a mapping")
    expected = set(SUPPORTED_PARAMETERS)
    actual = set(tuning)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing:
        raise SweepError("fast_trot_tuning echo is missing: %s" % missing)
    if unknown:
        raise SweepError(
            "fast_trot_tuning echo has unknown fields: %s" % unknown
        )

    result = {}
    for name in SUPPORTED_PARAMETERS:
        value = _finite_float(tuning[name], name)
        lower, upper = FAST_TROT_TUNING_BOUNDS[name]
        if not lower <= value <= upper:
            raise SweepError(
                "%s must be in [%.3f, %.3f]" % (name, lower, upper)
            )
        result[name] = value
    if config is not None:
        try:
            result = validate_fast_trot_tuning(config, result)
        except (TypeError, ValueError) as exc:
            raise SweepError(str(exc)) from exc
    return dict(result)


def _source_physical_config_path():
    return (
        Path(__file__).resolve().parents[1]
        / "config"
        / "physical_fast_trot.yaml"
    )


def load_default_sweep_config():
    """Load the dedicated physical profile for ROS-free source-tree use."""
    source_path = _source_physical_config_path()
    if source_path.is_file():
        return load_fast_trot_config(source_path)
    try:
        from ament_index_python.packages import get_package_share_directory

        installed_path = (
            Path(get_package_share_directory("volt_description"))
            / "config"
            / "physical_fast_trot.yaml"
        )
    except (ImportError, LookupError) as exc:
        raise SweepError(
            "cannot locate config/physical_fast_trot.yaml"
        ) from exc
    try:
        return load_fast_trot_config(installed_path)
    except (OSError, ValueError) as exc:
        raise SweepError(str(exc)) from exc


def build_sweep_plan(parameter, values, baseline, config=None):
    """Build complete candidates that vary exactly one field.

    When ``config`` is omitted, the repository's dedicated physical profile is
    loaded. Every candidate is passed through the controller's authoritative
    ``validate_fast_trot_tuning`` function, including its cross-field command
    envelope constraint.
    """
    parameter, values = validate_value_list(parameter, values)
    if config is None:
        config = load_default_sweep_config()
    baseline = validate_complete_tuning(baseline, config=config)

    points = []
    for index, value in enumerate(values):
        if abs(value - baseline[parameter]) <= TUNING_ABSOLUTE_TOLERANCE:
            raise SweepError(
                "sweep point %d equals the baseline %s"
                % (index, parameter)
            )
        candidate = dict(baseline)
        candidate[parameter] = value
        candidate = validate_complete_tuning(candidate, config=config)
        changed = [
            name
            for name in SUPPORTED_PARAMETERS
            if abs(candidate[name] - baseline[name])
            > TUNING_ABSOLUTE_TOLERANCE
        ]
        if changed != [parameter]:
            raise SweepError(
                "each sweep point must change exactly %s" % parameter
            )
        points.append(SweepPoint(index, value, candidate))
    return SweepPlan(
        parameter=parameter,
        values=values,
        baseline=baseline,
        points=tuple(points),
    )


def tuning_matches(first, second, tolerance=TUNING_ABSOLUTE_TOLERANCE):
    """Return whether two complete finite tuning mappings match."""
    try:
        first = validate_complete_tuning(first)
        second = validate_complete_tuning(second)
    except SweepError:
        return False
    tolerance = _finite_float(tolerance, "tolerance")
    if tolerance < 0.0:
        raise SweepError("tolerance cannot be negative")
    return all(
        abs(first[name] - second[name]) <= tolerance
        for name in SUPPORTED_PARAMETERS
    )


def tuning_json(tuning, config=None):
    """Serialize one complete validated tuning with no NaN extension."""
    return json.dumps(
        validate_complete_tuning(tuning, config=config),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )


def _zero_vector(status, name, tolerance=1e-6):
    value = status.get(name)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return False
    try:
        values = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return False
    return all(
        math.isfinite(item) and abs(item) <= tolerance
        for item in values
    )


def status_safety_errors(status):
    """Return fail-closed reasons that block a live tuning publication."""
    if not isinstance(status, dict):
        return ["status must be a JSON object"]
    errors = []

    exact_values = (
        ("hardware_mode", True),
        ("use_sim_time", False),
        ("fast_trot_profile", "physical"),
        ("requested_gait", "fast_trot"),
        ("active_gait", "fast_trot"),
        ("state", "standing"),
        ("moving", False),
        ("motion_active", False),
        ("step_in_place", False),
        ("physical_test_active", False),
        ("physical_test_returning", False),
        ("command_owner", "MOTION"),
        ("motion_authorized", True),
        ("phase_transition_hold", False),
    )
    for field, expected in exact_values:
        if field not in status:
            errors.append("%s must equal %r" % (field, expected))
            continue
        actual = status[field]
        if isinstance(expected, bool):
            matches = actual is expected
        else:
            matches = actual == expected
        if not matches:
            errors.append("%s must equal %r" % (field, expected))

    for field in ("pending_gait", "pending_pose_action"):
        if field not in status or status[field] not in (None, ""):
            errors.append("%s must be empty" % field)

    # Newer controllers may expose these direct flags. Missing fields are
    # tolerated because the existing status contract provides equivalent
    # stopped-state gates above.
    for field in ("gait_active", "transition_active"):
        if field in status and status[field] is not False:
            errors.append("%s must be false" % field)

    stance_legs = status.get("stance_legs")
    if (
        not isinstance(stance_legs, (list, tuple))
        or len(stance_legs) != len(LEG_ORDER)
        or set(stance_legs) != set(LEG_ORDER)
    ):
        errors.append("all four canonical legs must be in stance")
    swing_legs = status.get("swing_legs")
    if not isinstance(swing_legs, (list, tuple)) or len(swing_legs) != 0:
        errors.append("swing_legs must be empty")

    for field in ("requested_velocity", "filtered_velocity"):
        if not _zero_vector(status, field):
            errors.append("%s must be a finite zero vector" % field)

    if "fast_trot_tuning" not in status:
        errors.append("fast_trot_tuning baseline echo is missing")
    else:
        try:
            validate_complete_tuning(status["fast_trot_tuning"])
        except SweepError as exc:
            errors.append(str(exc))
    return errors


def validate_safe_status(status, expected_tuning=None, config=None):
    """Return the complete baseline/echo only when every live gate is safe."""
    errors = status_safety_errors(status)
    tuning = None
    if isinstance(status, dict) and "fast_trot_tuning" in status:
        try:
            tuning = validate_complete_tuning(
                status["fast_trot_tuning"],
                config=config,
            )
        except SweepError as exc:
            if str(exc) not in errors:
                errors.append(str(exc))
    if (
        tuning is not None
        and expected_tuning is not None
        and not tuning_matches(tuning, expected_tuning)
    ):
        errors.append("status has not echoed the expected prior tuning")
    if errors:
        raise SweepError("; ".join(errors))
    return tuning


def next_tuning_after_confirmation(plan, point_index, status, config=None):
    """Authorize one point only after status echoes its expected predecessor."""
    if not isinstance(plan, SweepPlan):
        raise SweepError("plan must be a SweepPlan")
    try:
        point_index = int(point_index)
    except (TypeError, ValueError) as exc:
        raise SweepError("point_index must be an integer") from exc
    if not 0 <= point_index < len(plan.points):
        raise SweepError("point_index is outside the sweep plan")
    expected_prior = (
        plan.baseline
        if point_index == 0
        else plan.points[point_index - 1].tuning
    )
    validate_safe_status(
        status,
        expected_tuning=expected_prior,
        config=config,
    )
    return dict(plan.points[point_index].tuning)


def authorize_apply(apply, acknowledgement=""):
    """Require an explicit observe/hold statement only for live application."""
    if not bool(apply):
        return False
    if (
        str(acknowledgement or "").strip()
        != HOLD_OBSERVE_ACKNOWLEDGEMENT
    ):
        raise SweepError(
            "type --acknowledge-hold-observe '%s'"
            % HOLD_OBSERVE_ACKNOWLEDGEMENT
        )
    return True


def validate_runtime_options(
    observe_seconds,
    status_timeout,
    confirmation_timeout,
):
    observe_seconds = _finite_float(observe_seconds, "observe_seconds")
    status_timeout = _finite_float(status_timeout, "status_timeout")
    confirmation_timeout = _finite_float(
        confirmation_timeout,
        "confirmation_timeout",
    )
    if not 0.5 <= observe_seconds <= 30.0:
        raise SweepError("observe_seconds must be in [0.5, 30.0]")
    if not 1.0 <= status_timeout <= 15.0:
        raise SweepError("status_timeout must be in [1.0, 15.0]")
    if not 1.0 <= confirmation_timeout <= 15.0:
        raise SweepError("confirmation_timeout must be in [1.0, 15.0]")
    return observe_seconds, status_timeout, confirmation_timeout


def create_argument_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Plan or apply a stopped-state one-variable FAST TROT tuning "
            "sweep. Default: dry-run with no publication."
        )
    )
    parser.add_argument(
        "--parameter",
        required=True,
        choices=SUPPORTED_PARAMETERS,
        help="The only tuning field allowed to change in this run.",
    )
    parser.add_argument(
        "--values",
        required=True,
        nargs="+",
        type=float,
        metavar="VALUE",
        help=(
            "One to %d finite values within the controller validator bounds."
            % MAX_SWEEP_POINTS
        ),
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Publish the confirmed sequence; omission is a no-publish dry-run.",
    )
    parser.add_argument(
        "--acknowledge-hold-observe",
        "--acknowledge",
        default="",
        metavar="TEXT",
        help=(
            "Required with --apply; type exactly: %s"
            % HOLD_OBSERVE_ACKNOWLEDGEMENT
        ),
    )
    parser.add_argument(
        "--observe-seconds",
        type=float,
        default=DEFAULT_OBSERVE_SECONDS,
        help="Stopped-state observation time after each confirmed point.",
    )
    parser.add_argument(
        "--status-timeout",
        type=float,
        default=DEFAULT_STATUS_TIMEOUT,
        help="Seconds to wait for an initial status/baseline echo.",
    )
    parser.add_argument(
        "--confirmation-timeout",
        type=float,
        default=DEFAULT_CONFIRMATION_TIMEOUT,
        help="Seconds to wait for the controller to echo each tuning.",
    )
    return parser


def _run_ros(
    parameter,
    values,
    apply,
    observe_seconds,
    status_timeout,
    confirmation_timeout,
):
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        try:
            from rclpy.signals import SignalHandlerOptions
        except ImportError:  # pragma: no cover - older rclpy fallback.
            SignalHandlerOptions = None
    except ImportError as exc:  # pragma: no cover - depends on ROS environment.
        raise RuntimeError(
            "ROS 2 Python packages are unavailable; source the workspace"
        ) from exc

    node_suffix = uuid.uuid4().hex[:8]

    class FastTrotSweepNode(Node):
        def __init__(self):
            super().__init__("volt_fast_trot_sweep_" + node_suffix)
            self.tuning_publisher = self.create_publisher(
                String,
                TUNING_TOPIC,
                10,
            )
            self.create_subscription(
                String,
                STATUS_TOPIC,
                self.status_callback,
                10,
            )
            self.status_sequence = 0
            self.latest_status = None
            self.latest_status_error = ""
            self.latest_status_time = 0.0

        def status_callback(self, message):
            self.status_sequence += 1
            self.latest_status_time = time.monotonic()
            try:
                decoded = json.loads(message.data)
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                self.latest_status = None
                self.latest_status_error = "invalid /volt/status JSON: %s" % exc
                return
            if not isinstance(decoded, dict):
                self.latest_status = None
                self.latest_status_error = "/volt/status must be a JSON object"
                return
            self.latest_status = decoded
            self.latest_status_error = ""

        def wait_for_initial_status(self):
            deadline = time.monotonic() + status_timeout
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.latest_status is not None:
                    return dict(self.latest_status)
                if self.latest_status_error:
                    raise SweepError(self.latest_status_error)
            raise SweepError(
                "no valid /volt/status baseline within %.1f seconds"
                % status_timeout
            )

        def current_status(self):
            if self.latest_status_error:
                raise SweepError(self.latest_status_error)
            if self.latest_status is None:
                raise SweepError("no valid /volt/status is available")
            age = time.monotonic() - self.latest_status_time
            if not 0.0 <= age <= STATUS_FRESHNESS_TIMEOUT:
                raise SweepError("/volt/status is stale")
            return dict(self.latest_status)

        def wait_for_tuning_subscriber(self):
            deadline = time.monotonic() + status_timeout
            while rclpy.ok() and time.monotonic() < deadline:
                if self.tuning_publisher.get_subscription_count() > 0:
                    return
                rclpy.spin_once(self, timeout_sec=0.05)
            raise SweepError(
                "no motion-controller subscriber on %s" % TUNING_TOPIC
            )

        def publish_tuning(self, tuning, config):
            message = String()
            message.data = tuning_json(tuning, config=config)
            self.tuning_publisher.publish(message)

        def wait_for_confirmation(
            self,
            expected,
            config,
            after_sequence,
            require_safe=True,
        ):
            deadline = time.monotonic() + confirmation_timeout
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.status_sequence <= after_sequence:
                    continue
                status = self.current_status()
                try:
                    if require_safe:
                        echo = validate_safe_status(
                            status,
                            config=config,
                        )
                    else:
                        echo = validate_complete_tuning(
                            status.get("fast_trot_tuning"),
                            config=config,
                        )
                except SweepError:
                    if require_safe:
                        raise
                    continue
                if tuning_matches(echo, expected):
                    return dict(status)
            raise SweepError(
                "controller did not echo tuning within %.1f seconds"
                % confirmation_timeout
            )

        def observe_confirmed_tuning(self, expected, config):
            deadline = time.monotonic() + observe_seconds
            checked_sequence = self.status_sequence
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                status = self.current_status()
                if self.status_sequence != checked_sequence:
                    validate_safe_status(
                        status,
                        expected_tuning=expected,
                        config=config,
                    )
                    checked_sequence = self.status_sequence
            validate_safe_status(
                self.current_status(),
                expected_tuning=expected,
                config=config,
            )

        def live_config(self, status):
            config_path = str(
                status.get("fast_trot_config_file", "")
            ).strip()
            if not config_path:
                raise SweepError(
                    "status does not echo fast_trot_config_file"
                )
            try:
                return load_fast_trot_config(config_path)
            except (OSError, ValueError) as exc:
                raise SweepError(
                    "cannot validate against live FAST TROT config: %s" % exc
                ) from exc

        def print_plan(self, plan):
            print("DRY RUN - no tuning was published")
            print("parameter: %s" % plan.parameter)
            print(
                "baseline: %s"
                % json.dumps(plan.baseline, sort_keys=True)
            )
            for point in plan.points:
                print(
                    "point %d: %s=%.9g complete=%s"
                    % (
                        point.index + 1,
                        plan.parameter,
                        point.value,
                        json.dumps(point.tuning, sort_keys=True),
                    )
                )

        def execute(self):
            initial_status = self.wait_for_initial_status()
            config = self.live_config(initial_status)
            baseline = validate_complete_tuning(
                initial_status.get("fast_trot_tuning"),
                config=config,
            )
            plan = build_sweep_plan(
                parameter,
                values,
                baseline,
                config=config,
            )
            if not apply:
                self.print_plan(plan)
                return True

            self.wait_for_tuning_subscriber()
            validate_safe_status(
                self.current_status(),
                expected_tuning=plan.baseline,
                config=config,
            )
            baseline_restore_needed = False
            try:
                for point in plan.points:
                    candidate = next_tuning_after_confirmation(
                        plan,
                        point.index,
                        self.current_status(),
                        config=config,
                    )
                    sequence_before_publish = self.status_sequence
                    baseline_restore_needed = True
                    self.publish_tuning(candidate, config)
                    self.get_logger().info(
                        "Published point %d/%d: %s=%.9g"
                        % (
                            point.index + 1,
                            len(plan.points),
                            plan.parameter,
                            point.value,
                        )
                    )
                    self.wait_for_confirmation(
                        candidate,
                        config,
                        sequence_before_publish,
                    )
                    self.observe_confirmed_tuning(candidate, config)
                return True
            finally:
                if baseline_restore_needed and rclpy.ok():
                    sequence_before_restore = self.status_sequence
                    self.publish_tuning(plan.baseline, config)
                    self.get_logger().info(
                        "Published original FAST TROT baseline."
                    )
                    try:
                        self.wait_for_confirmation(
                            plan.baseline,
                            config,
                            sequence_before_restore,
                            require_safe=False,
                        )
                    except SweepError as exc:
                        self.get_logger().error(
                            "Baseline restore was not confirmed: %s" % exc
                        )
                        raise SweepError(
                            "baseline restore was not confirmed"
                        ) from exc

    if SignalHandlerOptions is None:
        rclpy.init(args=[])
    else:
        rclpy.init(
            args=[],
            signal_handler_options=SignalHandlerOptions.NO,
        )
    node = FastTrotSweepNode()
    succeeded = False
    try:
        succeeded = node.execute()
    except KeyboardInterrupt:
        node.get_logger().warning(
            "Sweep interrupted; baseline restore requested."
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if succeeded else 1


def main(argv=None):
    parser = create_argument_parser()
    args = parser.parse_args(argv)
    try:
        parameter, values = validate_value_list(
            args.parameter,
            args.values,
        )
        authorize_apply(
            args.apply,
            args.acknowledge_hold_observe,
        )
        (
            observe_seconds,
            status_timeout,
            confirmation_timeout,
        ) = validate_runtime_options(
            args.observe_seconds,
            args.status_timeout,
            args.confirmation_timeout,
        )
    except SweepError as exc:
        parser.error(str(exc))

    try:
        return _run_ros(
            parameter,
            values,
            args.apply,
            observe_seconds,
            status_timeout,
            confirmation_timeout,
        )
    except (RuntimeError, SweepError) as exc:
        print("volt_fast_trot_sweep: %s" % exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
