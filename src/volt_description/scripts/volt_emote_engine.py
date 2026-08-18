#!/usr/bin/env python3

"""Pure Cartesian emote loading, validation, sampling, and cancellation.

This module deliberately has no ROS or GUI dependency.  A controller supplies a
monotonic timestamp, samples :class:`CartesianEmoteEngine`, and feeds the
returned absolute body target and foot targets through the normal VOLT IK and
joint-conditioning path.

YAML angles are expressed in degrees for readability.  Runtime frames use the
canonical kinematics units: metres and radians.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Mapping, Tuple

import yaml
from yaml.constructor import ConstructorError
from yaml.resolver import BaseResolver

from volt_kinematics import (
    JOINT_NAMES,
    LEG_ORDER,
    NOMINAL_FEET,
    NOMINAL_HEIGHT,
    feet_to_joint_positions_diagnostic,
    smootherstep,
)


SCHEMA_VERSION = 1
BUILTIN_EMOTES = (
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
    "look_left",
    "look_right",
)

MIN_REPETITIONS = 1
MAX_REPETITIONS = 5
MIN_SPEED = 0.5
MAX_SPEED = 2.0
MIN_SCALE = 0.5
# Raised from 1.5 after sweeping every catalog emote through the controller's
# own preflight (feet_to_joint_positions_diagnostic, projected_targets empty)
# at the 0.195 m standing profile.  The binding emotes are wave_left/right at
# 2.3x -- they lift a foot 40 mm while shifting body_y, so they run out of
# workspace long before the pure body-pose emotes (bow 6.6x, shake_no 8.1x,
# look_* 11.1x).  2.0x keeps ~15% margin under that worst case and applies
# uniformly, so no emote can be driven into workspace projection.
MAX_SCALE = 2.0
# Depth scales only the vertical body travel, which the leg IK has far more
# room for than the lateral/roll motion `amplitude` drives.  A sweep of
# inverse_leg_diagnostic over the planted-foot stance clamps at 85 mm of
# downward travel (joint_limit_clamping); 3.0 x the 20 mm push-up base is
# 60 mm, leaving 20 mm of margin.  Amplitude deliberately keeps the narrower
# MAX_SCALE because the other emotes were not authored for 3x sideways swing.
MAX_DEPTH_SCALE = 3.0

# These match the current controller's conservative manual body-pose envelope.
# MIN_BODY_HEIGHT_M bounds the standing base the robot starts an emote from.
MIN_BODY_HEIGHT_M = 0.175
MAX_BODY_HEIGHT_M = 0.220
# An emote may dip below the standing envelope: all four feet stay planted, so
# the leg is not reaching, it is folding.  A sweep of inverse_leg_diagnostic
# over the planted stance is clean to 0.120 m and only clamps at 0.115 m, and
# 3x depth on the deepest authored emote (push_ups, 20 mm base) is 60 mm, and
# two real_robot_profiles stand at 0.195 rather than 0.200, so the deepest
# reachable frame is 0.135.  The floor is set just under that, still 12 mm
# above the 0.120 m the IK sweep clears and 17 mm above where it clamps.  The
# base height above is deliberately NOT widened -- the robot still stands,
# walks and takes manual pose commands inside the original envelope.
MIN_EMOTE_BODY_HEIGHT_M = 0.132
# The envelope has to admit the peaks 2.0x actually produces: heart reaches
# x=0.024 and wave reaches y=0.020, both of which sat exactly on the old
# limits.  Widened with margin, and still far inside the measured single-axis
# IK limits at 0.195 m (x 0.080 m, y 0.075 m).
MAX_BODY_X_M = 0.030
MAX_BODY_Y_M = 0.026
# 2.0x drives bow to 12 deg pitch (0.209 rad) and shake_no to 11 deg yaw
# (0.192 rad), both over the old 0.18 rad ceiling.  0.24 rad is 13.75 deg,
# comfortably inside the measured single-axis IK limits (roll 26 deg,
# pitch 22 deg, yaw 40 deg+) at the same standing height.
MAX_BODY_ANGLE_RAD = 0.24
# heart and wave_* lift a foot 40 mm at 1x, so 2.0x needs 80 mm and both of
# these sat just under it.  The per-emote IK sweep runs those two clean to
# 2.3x (a 92 mm lift) -- they are the binding emotes precisely because a
# lifted foot leaves the workspace before a body-pose-only emote does -- so
# 90 mm axis / 100 mm norm admits 2.0x while staying inside what IK clears.
MAX_FOOT_OFFSET_AXIS_M = 0.090
MAX_FOOT_OFFSET_NORM_M = 0.100

MIN_SEGMENT_DURATION_S = 0.05
MAX_SEGMENT_DURATION_S = 10.0
MAX_EMOTE_DURATION_S = 60.0
MAX_KEYFRAMES = 128
MAX_EMOTES = 64
DEFAULT_PREFLIGHT_SAMPLE_PERIOD_S = 0.05
DEFAULT_RETURN_DURATION_S = 1.0

_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")
_ROOT_KEYS = {"schema_version", "base_body_height_m", "emotes"}
_EMOTE_KEYS = {"description", "keyframes"}
_KEYFRAME_KEYS = {"duration_s", "easing", "body", "feet"}
_BODY_KEYS = {
    "height_offset_m",
    "x_m",
    "y_m",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
}


class EmoteValidationError(ValueError):
    """Raised when configuration, options, or sampled targets are unsafe."""


class EmoteStateError(RuntimeError):
    """Raised when a playback request conflicts with the current engine state."""


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects duplicate mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in mapping
        except TypeError as exc:
            raise ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                "found an unhashable mapping key",
                key_node.start_mark,
            ) from exc
        if duplicate:
            raise ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                "found duplicate key %r" % key,
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _finite_float(value, label):
    if isinstance(value, bool):
        raise EmoteValidationError("%s must be a finite number" % label)
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise EmoteValidationError("%s must be a finite number" % label) from exc
    if not math.isfinite(result):
        raise EmoteValidationError("%s must be finite" % label)
    return result


def _mapping(value, label):
    if not isinstance(value, dict):
        raise EmoteValidationError("%s must be a mapping" % label)
    return value


def _require_exact_keys(mapping, expected, label):
    keys = set(mapping)
    missing = sorted(expected - keys, key=repr)
    unknown = sorted(keys - expected, key=repr)
    if missing:
        raise EmoteValidationError("%s is missing keys: %s" % (label, missing))
    if unknown:
        raise EmoteValidationError("%s has unknown keys: %s" % (label, unknown))


def _reject_unknown_keys(mapping, allowed, label):
    unknown = sorted(set(mapping) - set(allowed), key=repr)
    if unknown:
        raise EmoteValidationError("%s has unknown keys: %s" % (label, unknown))


def _bounded(value, lower, upper, label):
    value = _finite_float(value, label)
    if value < lower or value > upper:
        raise EmoteValidationError(
            "%s %.6f is outside [%.6f, %.6f]"
            % (label, value, lower, upper)
        )
    return value


@dataclass(frozen=True)
class BodyOffset:
    """Body target relative to the catalog's neutral body target."""

    height: float = 0.0
    x: float = 0.0
    y: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class RelativePose:
    """Normalized body and foot offsets, all in metres/radians."""

    body: BodyOffset
    foot_offsets: Tuple[Tuple[float, float, float], ...]


@dataclass(frozen=True)
class EmoteKeyframe:
    """One target; duration is the transition time from the prior target."""

    duration: float
    easing: str
    pose: RelativePose


@dataclass(frozen=True)
class EmoteDefinition:
    """Validated, immutable definition ready for deterministic sampling."""

    name: str
    description: str
    keyframes: Tuple[EmoteKeyframe, ...]
    total_duration: float


@dataclass(frozen=True)
class EmoteCatalog:
    """A validated collection of definitions sharing one neutral height."""

    base_body_height: float
    emotes: Mapping[str, EmoteDefinition]


@dataclass(frozen=True)
class EmoteOptions:
    """Runtime request options after validation."""

    repetitions: int = 1
    speed: float = 1.0
    amplitude: float = 1.0
    depth: float = 1.0


@dataclass(frozen=True)
class BodyTarget:
    """Absolute body target in the units consumed by VOLT kinematics."""

    height: float
    x: float
    y: float
    roll: float
    pitch: float
    yaw: float

    def ik_kwargs(self):
        """Return keyword arguments accepted by VOLT whole-body IK."""
        return {
            "height": self.height,
            "body_x": self.x,
            "body_y": self.y,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
        }


@dataclass(frozen=True)
class CartesianEmoteFrame:
    """One timer-sampled controller target.

    ``feet`` contains absolute body/world support-coordinate targets keyed in
    canonical ``LEG_ORDER``.  Angles in ``body`` are radians.
    """

    emote: str
    state: str
    progress: float
    body: BodyTarget
    feet: Mapping[str, Tuple[float, float, float]]

    def solve_ik(self):
        """Return canonical joints and diagnostics using the in-tree IK."""
        return feet_to_joint_positions_diagnostic(
            dict(self.feet),
            **self.body.ik_kwargs(),
        )


@dataclass(frozen=True)
class EmotePreflightReport:
    """Summary proving that sampled targets reached finite, unprojected IK."""

    emote: str
    samples: int
    requested_duration: float
    maximum_absolute_joint: float


def neutral_relative_pose():
    return RelativePose(
        body=BodyOffset(),
        foot_offsets=tuple((0.0, 0.0, 0.0) for _ in LEG_ORDER),
    )


def _pose_is_neutral(pose, tolerance=1e-12):
    values = (
        pose.body.height,
        pose.body.x,
        pose.body.y,
        pose.body.roll,
        pose.body.pitch,
        pose.body.yaw,
    )
    values += tuple(
        value for offset in pose.foot_offsets for value in offset
    )
    return all(abs(value) <= tolerance for value in values)


def validate_options(repetitions=1, speed=1.0, amplitude=1.0, depth=1.0):
    """Validate and normalize the public runtime option envelope."""
    if isinstance(repetitions, bool) or not isinstance(repetitions, int):
        raise EmoteValidationError("repetitions must be an integer")
    if not MIN_REPETITIONS <= repetitions <= MAX_REPETITIONS:
        raise EmoteValidationError(
            "repetitions must be in [%d, %d]"
            % (MIN_REPETITIONS, MAX_REPETITIONS)
        )
    return EmoteOptions(
        repetitions=repetitions,
        speed=_bounded(speed, MIN_SPEED, MAX_SPEED, "speed"),
        amplitude=_bounded(amplitude, MIN_SCALE, MAX_SCALE, "amplitude"),
        depth=_bounded(depth, MIN_SCALE, MAX_DEPTH_SCALE, "depth"),
    )


def _validated_options(options):
    if not isinstance(options, EmoteOptions):
        raise EmoteValidationError("options must be an EmoteOptions instance")
    return validate_options(
        options.repetitions,
        options.speed,
        options.amplitude,
        options.depth,
    )


def _parse_body(value, label):
    body = _mapping(value, label)
    _reject_unknown_keys(body, _BODY_KEYS, label)
    degree = math.pi / 180.0
    return BodyOffset(
        height=_finite_float(body.get("height_offset_m", 0.0), label + ".height_offset_m"),
        x=_finite_float(body.get("x_m", 0.0), label + ".x_m"),
        y=_finite_float(body.get("y_m", 0.0), label + ".y_m"),
        roll=_finite_float(body.get("roll_deg", 0.0), label + ".roll_deg") * degree,
        pitch=_finite_float(body.get("pitch_deg", 0.0), label + ".pitch_deg") * degree,
        yaw=_finite_float(body.get("yaw_deg", 0.0), label + ".yaw_deg") * degree,
    )


def _parse_feet(value, label):
    feet = _mapping(value, label)
    _reject_unknown_keys(feet, set(LEG_ORDER), label)
    offsets = []
    for leg_name in LEG_ORDER:
        raw = feet.get(leg_name, (0.0, 0.0, 0.0))
        if isinstance(raw, (str, bytes)):
            raise EmoteValidationError("%s.%s must be an XYZ sequence" % (label, leg_name))
        try:
            raw = tuple(raw)
        except TypeError as exc:
            raise EmoteValidationError(
                "%s.%s must be an XYZ sequence" % (label, leg_name)
            ) from exc
        if len(raw) != 3:
            raise EmoteValidationError(
                "%s.%s must contain exactly three XYZ offsets"
                % (label, leg_name)
            )
        offsets.append(tuple(
            _finite_float(component, "%s.%s[%d]" % (label, leg_name, index))
            for index, component in enumerate(raw)
        ))
    return tuple(offsets)


def _scale_pose(pose, options):
    amplitude = options.amplitude
    depth = options.depth
    return RelativePose(
        body=BodyOffset(
            height=pose.body.height * depth,
            x=pose.body.x * amplitude,
            y=pose.body.y * amplitude,
            roll=pose.body.roll * amplitude,
            pitch=pose.body.pitch * amplitude,
            yaw=pose.body.yaw * amplitude,
        ),
        foot_offsets=tuple(
            tuple(component * amplitude for component in offset)
            for offset in pose.foot_offsets
        ),
    )


def _interpolate_pose(start, target, blend):
    blend = _finite_float(blend, "interpolation blend")
    if blend < -1e-12 or blend > 1.0 + 1e-12:
        raise EmoteValidationError("interpolation blend must be in [0, 1]")
    blend = max(0.0, min(1.0, blend))

    def mix(first, second):
        return first + (second - first) * blend

    return RelativePose(
        body=BodyOffset(
            height=mix(start.body.height, target.body.height),
            x=mix(start.body.x, target.body.x),
            y=mix(start.body.y, target.body.y),
            roll=mix(start.body.roll, target.body.roll),
            pitch=mix(start.body.pitch, target.body.pitch),
            yaw=mix(start.body.yaw, target.body.yaw),
        ),
        foot_offsets=tuple(
            tuple(mix(first, second) for first, second in zip(start_offset, end_offset))
            for start_offset, end_offset in zip(start.foot_offsets, target.foot_offsets)
        ),
    )


def _validate_output_pose(pose, base_body_height, label):
    height = base_body_height + pose.body.height
    _bounded(
        height,
        MIN_EMOTE_BODY_HEIGHT_M,
        MAX_BODY_HEIGHT_M,
        label + " body height",
    )
    _bounded(pose.body.x, -MAX_BODY_X_M, MAX_BODY_X_M, label + " body x")
    _bounded(pose.body.y, -MAX_BODY_Y_M, MAX_BODY_Y_M, label + " body y")
    for name, value in (
        ("roll", pose.body.roll),
        ("pitch", pose.body.pitch),
        ("yaw", pose.body.yaw),
    ):
        _bounded(
            value,
            -MAX_BODY_ANGLE_RAD,
            MAX_BODY_ANGLE_RAD,
            "%s body %s" % (label, name),
        )
    if len(pose.foot_offsets) != len(LEG_ORDER):
        raise EmoteValidationError("%s has the wrong number of foot offsets" % label)
    for leg_name, offset in zip(LEG_ORDER, pose.foot_offsets):
        if len(offset) != 3:
            raise EmoteValidationError("%s %s offset is not XYZ" % (label, leg_name))
        for axis, value in zip("xyz", offset):
            _bounded(
                value,
                -MAX_FOOT_OFFSET_AXIS_M,
                MAX_FOOT_OFFSET_AXIS_M,
                "%s %s %s offset" % (label, leg_name, axis),
            )
        if math.dist((0.0, 0.0, 0.0), offset) > MAX_FOOT_OFFSET_NORM_M:
            raise EmoteValidationError(
                "%s %s offset exceeds %.3f m"
                % (label, leg_name, MAX_FOOT_OFFSET_NORM_M)
            )


def _frame_from_pose(emote, state, progress, pose, base_body_height):
    _validate_output_pose(pose, base_body_height, "%s %s frame" % (emote, state))
    feet = {
        leg_name: tuple(
            nominal + delta
            for nominal, delta in zip(NOMINAL_FEET[leg_name], offset)
        )
        for leg_name, offset in zip(LEG_ORDER, pose.foot_offsets)
    }
    return CartesianEmoteFrame(
        emote=emote,
        state=state,
        progress=_bounded(progress, 0.0, 1.0, "frame progress"),
        body=BodyTarget(
            height=base_body_height + pose.body.height,
            x=pose.body.x,
            y=pose.body.y,
            roll=pose.body.roll,
            pitch=pose.body.pitch,
            yaw=pose.body.yaw,
        ),
        feet=MappingProxyType(feet),
    )


def sample_definition_pose(definition, elapsed_s, options=None):
    """Sample one repetition in definition time and return a relative pose."""
    if not isinstance(definition, EmoteDefinition):
        raise EmoteValidationError("definition must be an EmoteDefinition")
    if options is None:
        options = EmoteOptions()
    options = _validated_options(options)
    elapsed_s = _finite_float(elapsed_s, "elapsed_s")
    elapsed_s = max(0.0, min(definition.total_duration, elapsed_s))
    if elapsed_s <= 0.0:
        return _scale_pose(definition.keyframes[0].pose, options)

    segment_start_time = 0.0
    segment_start = definition.keyframes[0].pose
    for keyframe in definition.keyframes[1:]:
        segment_end_time = segment_start_time + keyframe.duration
        if elapsed_s <= segment_end_time:
            proportion = (
                (elapsed_s - segment_start_time) / keyframe.duration
            )
            if keyframe.easing != "smootherstep":
                raise EmoteValidationError(
                    "unsupported easing '%s'" % keyframe.easing
                )
            blend = smootherstep(proportion)
            return _scale_pose(
                _interpolate_pose(segment_start, keyframe.pose, blend),
                options,
            )
        segment_start_time = segment_end_time
        segment_start = keyframe.pose
    return _scale_pose(definition.keyframes[-1].pose, options)


def preflight_emote(
    definition,
    base_body_height=NOMINAL_HEIGHT,
    options=None,
    sample_period_s=DEFAULT_PREFLIGHT_SAMPLE_PERIOD_S,
):
    """Reject sampled targets requiring workspace projection or joint clamping."""
    if options is None:
        options = EmoteOptions()
    options = _validated_options(options)
    base_body_height = _bounded(
        base_body_height,
        MIN_BODY_HEIGHT_M,
        MAX_BODY_HEIGHT_M,
        "base_body_height_m",
    )
    sample_period_s = _bounded(sample_period_s, 0.01, 0.5, "sample_period_s")
    intervals = max(1, int(math.ceil(definition.total_duration / sample_period_s)))
    maximum_joint = 0.0
    for index in range(intervals + 1):
        elapsed = definition.total_duration * index / intervals
        pose = sample_definition_pose(definition, elapsed, options)
        frame = _frame_from_pose(
            definition.name,
            "preflight",
            index / intervals,
            pose,
            base_body_height,
        )
        try:
            joints, diagnostics = frame.solve_ik()
        except Exception as exc:
            raise EmoteValidationError(
                "%s IK failed at %.3f s: %s"
                % (definition.name, elapsed, exc)
            ) from exc
        if len(joints) != len(JOINT_NAMES) or not all(
            math.isfinite(value) for value in joints
        ):
            raise EmoteValidationError(
                "%s IK was not 12 finite joints at %.3f s"
                % (definition.name, elapsed)
            )
        projected = list(diagnostics.get("projected_targets", []))
        if projected:
            raise EmoteValidationError(
                "%s requires IK projection for %s at %.3f s"
                % (definition.name, projected, elapsed)
            )
        maximum_joint = max(maximum_joint, *(abs(value) for value in joints))
    return EmotePreflightReport(
        emote=definition.name,
        samples=intervals + 1,
        requested_duration=(
            definition.total_duration * options.repetitions / options.speed
        ),
        maximum_absolute_joint=maximum_joint,
    )


def _parse_definition(name, value, base_body_height):
    if not isinstance(name, str) or not _NAME_PATTERN.fullmatch(name):
        raise EmoteValidationError("invalid emote name %r" % name)
    data = _mapping(value, "emote %s" % name)
    _require_exact_keys(data, _EMOTE_KEYS, "emote %s" % name)
    description = data["description"]
    if not isinstance(description, str) or not description.strip():
        raise EmoteValidationError("emote %s description must be non-empty" % name)
    raw_keyframes = data["keyframes"]
    if not isinstance(raw_keyframes, list):
        raise EmoteValidationError("emote %s keyframes must be a list" % name)
    if not 2 <= len(raw_keyframes) <= MAX_KEYFRAMES:
        raise EmoteValidationError(
            "emote %s must contain 2..%d keyframes" % (name, MAX_KEYFRAMES)
        )

    keyframes = []
    total_duration = 0.0
    for index, raw_keyframe in enumerate(raw_keyframes):
        label = "emote %s keyframe %d" % (name, index)
        keyframe = _mapping(raw_keyframe, label)
        _require_exact_keys(keyframe, _KEYFRAME_KEYS, label)
        duration = _finite_float(keyframe["duration_s"], label + ".duration_s")
        if index == 0:
            if abs(duration) > 1e-12:
                raise EmoteValidationError(
                    "%s first duration_s must be exactly 0" % label
                )
        else:
            duration = _bounded(
                duration,
                MIN_SEGMENT_DURATION_S,
                MAX_SEGMENT_DURATION_S,
                label + ".duration_s",
            )
            total_duration += duration
        easing = keyframe["easing"]
        if easing != "smootherstep":
            raise EmoteValidationError(
                "%s easing must be 'smootherstep'" % label
            )
        pose = RelativePose(
            body=_parse_body(keyframe["body"], label + ".body"),
            foot_offsets=_parse_feet(keyframe["feet"], label + ".feet"),
        )
        _validate_output_pose(pose, base_body_height, label)
        keyframes.append(EmoteKeyframe(duration, easing, pose))

    if total_duration <= 0.0 or total_duration > MAX_EMOTE_DURATION_S:
        raise EmoteValidationError(
            "emote %s duration must be in (0, %.1f] s"
            % (name, MAX_EMOTE_DURATION_S)
        )
    if not _pose_is_neutral(keyframes[0].pose):
        raise EmoteValidationError("emote %s must start at the neutral pose" % name)
    if not _pose_is_neutral(keyframes[-1].pose):
        raise EmoteValidationError("emote %s must end at the neutral pose" % name)
    return EmoteDefinition(
        name=name,
        description=description.strip(),
        keyframes=tuple(keyframes),
        total_duration=total_duration,
    )


def load_emote_catalog(path, preflight=True, require_builtins=False):
    """Load one strict YAML catalog.

    Duplicate or unknown keys, non-finite values, unsupported easing, unsafe
    bounds, and unexpected sampled IK projection are rejected before use.
    """
    path = Path(path).expanduser().resolve()
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.load(handle, Loader=_UniqueKeyLoader)
    except (OSError, yaml.YAMLError) as exc:
        raise EmoteValidationError("could not load emote YAML %s: %s" % (path, exc)) from exc
    root = _mapping(data, "emote YAML root")
    _require_exact_keys(root, _ROOT_KEYS, "emote YAML root")
    version = root["schema_version"]
    if isinstance(version, bool) or not isinstance(version, int) or version != SCHEMA_VERSION:
        raise EmoteValidationError(
            "schema_version must be %d" % SCHEMA_VERSION
        )
    base_height = _bounded(
        root["base_body_height_m"],
        MIN_BODY_HEIGHT_M,
        MAX_BODY_HEIGHT_M,
        "base_body_height_m",
    )
    raw_emotes = _mapping(root["emotes"], "emotes")
    if not 1 <= len(raw_emotes) <= MAX_EMOTES:
        raise EmoteValidationError("emotes must contain 1..%d entries" % MAX_EMOTES)
    definitions = {
        name: _parse_definition(name, value, base_height)
        for name, value in raw_emotes.items()
    }
    if require_builtins:
        missing = sorted(set(BUILTIN_EMOTES) - set(definitions))
        if missing:
            raise EmoteValidationError("catalog is missing built-ins: %s" % missing)
    catalog = EmoteCatalog(base_height, MappingProxyType(definitions))
    if preflight:
        for definition in definitions.values():
            preflight_emote(definition, base_height)
    return catalog


def default_emote_config_path():
    """Resolve the catalog in both source/symlink and normal ROS installs."""
    source = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "cartesian_emotes.yaml"
    )
    if source.is_file():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory("volt_description"))
            / "config"
            / "cartesian_emotes.yaml"
        )
    except (ImportError, LookupError):
        return source


def load_builtin_catalog(path=None, preflight=True):
    """Load the shipped catalog and require the complete built-in inventory."""
    return load_emote_catalog(
        default_emote_config_path() if path is None else path,
        preflight=preflight,
        require_builtins=True,
    )


class CartesianEmoteEngine:
    """Nonblocking, caller-clocked emote playback state machine."""

    def __init__(
        self,
        catalog,
        return_duration_s=DEFAULT_RETURN_DURATION_S,
        preflight_on_start=True,
    ):
        if not isinstance(catalog, EmoteCatalog):
            raise EmoteValidationError("catalog must be an EmoteCatalog")
        self.catalog = catalog
        self.return_duration = _bounded(
            return_duration_s,
            0.20,
            5.0,
            "return_duration_s",
        )
        self.preflight_on_start = bool(preflight_on_start)
        self._state = "idle"
        self._definition = None
        self._options = EmoteOptions()
        self._start_time = 0.0
        self._return_start_time = 0.0
        self._return_start_pose = neutral_relative_pose()
        self._current_pose = neutral_relative_pose()
        self._last_sample_time = None

    @property
    def state(self):
        return self._state

    @property
    def active(self):
        return self._state in ("running", "returning")

    @property
    def current_emote(self):
        return self._definition.name if self._definition is not None else ""

    @property
    def available_emotes(self):
        return tuple(sorted(self.catalog.emotes))

    def _timestamp(self, value, label="now"):
        result = _finite_float(value, label)
        if self._last_sample_time is not None and result < self._last_sample_time - 1e-12:
            raise EmoteStateError("%s moved backwards" % label)
        return result

    def start(
        self,
        name,
        now,
        repetitions=1,
        speed=1.0,
        amplitude=1.0,
        depth=1.0,
    ):
        """Start one request and return its neutral first frame.

        An active emote must be cancelled and allowed to return before another
        request can start; this prevents target-controller overlap.
        """
        if self.active:
            raise EmoteStateError("an emote is already active")
        if name not in self.catalog.emotes:
            raise EmoteValidationError("unknown emote %r" % name)
        now = self._timestamp(now)
        options = validate_options(repetitions, speed, amplitude, depth)
        definition = self.catalog.emotes[name]
        if self.preflight_on_start:
            preflight_emote(definition, self.catalog.base_body_height, options)
        self._definition = definition
        self._options = options
        self._start_time = now
        self._state = "running"
        self._current_pose = neutral_relative_pose()
        self._last_sample_time = now
        return self.sample(now)

    def sample(self, now):
        """Return the target for ``now`` without sleeping or blocking."""
        now = self._timestamp(now)
        self._last_sample_time = now
        if self._state in ("idle", "complete") or self._definition is None:
            self._current_pose = neutral_relative_pose()
            return _frame_from_pose(
                self.current_emote,
                self._state,
                1.0 if self._state == "complete" else 0.0,
                self._current_pose,
                self.catalog.base_body_height,
            )

        if self._state == "returning":
            elapsed = max(0.0, now - self._return_start_time)
            progress = min(1.0, elapsed / self.return_duration)
            self._current_pose = _interpolate_pose(
                self._return_start_pose,
                neutral_relative_pose(),
                smootherstep(progress),
            )
            if elapsed + 1e-12 >= self.return_duration:
                progress = 1.0
                self._state = "complete"
                self._current_pose = neutral_relative_pose()
            return _frame_from_pose(
                self.current_emote,
                self._state,
                progress,
                self._current_pose,
                self.catalog.base_body_height,
            )

        single_wall_duration = self._definition.total_duration / self._options.speed
        total_wall_duration = single_wall_duration * self._options.repetitions
        elapsed = max(0.0, now - self._start_time)
        if elapsed + 1e-12 >= total_wall_duration:
            self._state = "complete"
            self._current_pose = neutral_relative_pose()
            return _frame_from_pose(
                self.current_emote,
                self._state,
                1.0,
                self._current_pose,
                self.catalog.base_body_height,
            )

        repetition_elapsed = elapsed % single_wall_duration
        definition_elapsed = repetition_elapsed * self._options.speed
        self._current_pose = sample_definition_pose(
            self._definition,
            definition_elapsed,
            self._options,
        )
        return _frame_from_pose(
            self.current_emote,
            "running",
            elapsed / total_wall_duration,
            self._current_pose,
            self.catalog.base_body_height,
        )

    def cancel(self, now):
        """Begin a smootherstep return from the current target to neutral."""
        if not self.active:
            return False
        now = self._timestamp(now)
        if self._state == "returning":
            return True
        frame = self.sample(now)
        if frame.state != "running":
            return False
        self._return_start_pose = self._current_pose
        self._return_start_time = now
        self._state = "returning"
        return True

    def reset(self):
        """Reset completed/idle bookkeeping; never interrupt active playback."""
        if self.active:
            raise EmoteStateError("cancel the active emote before reset")
        self._state = "idle"
        self._definition = None
        self._options = EmoteOptions()
        self._current_pose = neutral_relative_pose()
        self._last_sample_time = None

    def status(self):
        """Return a serialization-friendly state summary for a controller."""
        return {
            "state": self.state,
            "active": self.active,
            "emote": self.current_emote,
            "repetitions": self._options.repetitions,
            "speed": self._options.speed,
            "amplitude": self._options.amplitude,
            "depth": self._options.depth,
        }
