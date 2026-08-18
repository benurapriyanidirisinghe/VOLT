#!/usr/bin/env python3

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from volt_kinematics import JOINT_NAMES


class CalibrationError(ValueError):
    pass


@dataclass(frozen=True)
class ServoCalibration:
    joint_name: str
    pca_channel: int
    direction: int
    neutral_deg: float
    trim_deg: float
    min_deg: float
    max_deg: float
    min_pulse_us: int
    max_pulse_us: int


def _finite_float(value, field, joint_name):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise CalibrationError("%s.%s is not numeric" % (joint_name, field)) from exc
    if not math.isfinite(result):
        raise CalibrationError("%s.%s is not finite" % (joint_name, field))
    return result


def _int_value(value, field, joint_name):
    if isinstance(value, bool):
        raise CalibrationError("%s.%s is not an integer" % (joint_name, field))
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise CalibrationError("%s.%s is not an integer" % (joint_name, field)) from exc
    if result != value and not (isinstance(value, str) and str(result) == value):
        raise CalibrationError("%s.%s is not an integer" % (joint_name, field))
    return result


class ServoCalibrationTable:
    def __init__(self, joint_order, servos):
        self.joint_order = list(joint_order)
        self.servos = dict(servos)
        self.by_channel = {servo.pca_channel: servo for servo in self.servos.values()}

    @classmethod
    def from_file(cls, path):
        path = Path(path).expanduser()
        with path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
        return cls.from_dict(raw)

    @classmethod
    def from_dict(cls, raw):
        if not isinstance(raw, dict):
            raise CalibrationError("calibration file must be a mapping")
        joint_order = raw.get("joint_order")
        servos_raw = raw.get("servos")
        if joint_order != JOINT_NAMES:
            raise CalibrationError("joint_order must exactly match canonical JOINT_NAMES")
        if not isinstance(servos_raw, dict):
            raise CalibrationError("servos must be a mapping")
        if set(servos_raw.keys()) != set(JOINT_NAMES):
            missing = sorted(set(JOINT_NAMES) - set(servos_raw.keys()))
            extra = sorted(set(servos_raw.keys()) - set(JOINT_NAMES))
            raise CalibrationError("servo joint mismatch missing=%s extra=%s" % (missing, extra))

        servos = {}
        channels = []
        for joint_name in JOINT_NAMES:
            entry = servos_raw[joint_name]
            if not isinstance(entry, dict):
                raise CalibrationError("%s entry must be a mapping" % joint_name)
            channel = _int_value(entry.get("pca_channel"), "pca_channel", joint_name)
            direction = _int_value(entry.get("direction"), "direction", joint_name)
            neutral = _finite_float(entry.get("neutral_deg"), "neutral_deg", joint_name)
            trim = _finite_float(entry.get("trim_deg", 0.0), "trim_deg", joint_name)
            min_deg = _finite_float(entry.get("min_deg"), "min_deg", joint_name)
            max_deg = _finite_float(entry.get("max_deg"), "max_deg", joint_name)
            min_pulse = _int_value(entry.get("min_pulse_us"), "min_pulse_us", joint_name)
            max_pulse = _int_value(entry.get("max_pulse_us"), "max_pulse_us", joint_name)

            if not 0 <= channel <= 15:
                raise CalibrationError("%s pca_channel must be 0..15" % joint_name)
            if direction not in (-1, 1):
                raise CalibrationError("%s direction must be 1 or -1" % joint_name)
            if not min_deg < max_deg:
                raise CalibrationError("%s min_deg must be less than max_deg" % joint_name)
            if not min_pulse < max_pulse:
                raise CalibrationError("%s min_pulse_us must be less than max_pulse_us" % joint_name)
            if min_pulse < 100 or max_pulse > 3000:
                raise CalibrationError("%s pulse widths outside conservative bounds" % joint_name)
            # Keep the physical neutral itself valid.  A mounting/leveling trim
            # may make URDF zero unreachable on a real joint (for example a
            # foot whose neutral is already 180 degrees), so the effective
            # neutral is allowed outside this range. Runtime output is always
            # clamped by ros_radians_to_servo_degrees().
            if not min_deg <= neutral <= max_deg:
                raise CalibrationError("%s neutral_deg outside safe range" % joint_name)

            channels.append(channel)
            servos[joint_name] = ServoCalibration(
                joint_name=joint_name,
                pca_channel=channel,
                direction=direction,
                neutral_deg=neutral,
                trim_deg=trim,
                min_deg=min_deg,
                max_deg=max_deg,
                min_pulse_us=min_pulse,
                max_pulse_us=max_pulse,
            )

        if len(channels) != len(set(channels)):
            raise CalibrationError("PCA channels must be unique")
        if len(channels) != len(JOINT_NAMES):
            raise CalibrationError("exactly 12 channels are required")
        expected_channels = set(range(len(JOINT_NAMES)))
        if set(channels) != expected_channels:
            raise CalibrationError(
                "PCA channels must be the exact 12-channel FRAME range 0..11"
            )
        return cls(joint_order, servos)

    def ros_radians_to_servo_degrees(self, joint_name, radians):
        if joint_name not in self.servos:
            raise CalibrationError("unknown joint '%s'" % joint_name)
        radians = float(radians)
        if not math.isfinite(radians):
            raise CalibrationError("%s radians is not finite" % joint_name)
        servo = self.servos[joint_name]
        raw = servo.neutral_deg + servo.trim_deg + servo.direction * math.degrees(radians)
        clamped = max(servo.min_deg, min(servo.max_deg, raw))
        return clamped

    def servo_degrees_to_ros_radians(self, joint_name, degrees):
        if joint_name not in self.servos:
            raise CalibrationError("unknown joint '%s'" % joint_name)
        degrees = float(degrees)
        if not math.isfinite(degrees):
            raise CalibrationError("%s degrees is not finite" % joint_name)
        servo = self.servos[joint_name]
        return math.radians((degrees - servo.neutral_deg - servo.trim_deg) / servo.direction)

    def channel_frame_from_positions(self, named_joint_positions):
        frame = [None] * len(JOINT_NAMES)
        details = []
        for joint_name in JOINT_NAMES:
            if joint_name not in named_joint_positions:
                raise CalibrationError("missing joint command '%s'" % joint_name)
            servo = self.servos[joint_name]
            raw = servo.neutral_deg + servo.trim_deg + servo.direction * math.degrees(
                float(named_joint_positions[joint_name])
            )
            servo_deg = self.ros_radians_to_servo_degrees(joint_name, named_joint_positions[joint_name])
            if servo.pca_channel >= len(frame):
                raise CalibrationError("channel %d cannot fit 12-channel FRAME" % servo.pca_channel)
            if frame[servo.pca_channel] is not None:
                raise CalibrationError("duplicate channel %d in frame" % servo.pca_channel)
            frame[servo.pca_channel] = servo_deg
            details.append({
                "joint": joint_name,
                "ros_rad": float(named_joint_positions[joint_name]),
                "servo_deg": servo_deg,
                "pca_channel": servo.pca_channel,
                "clamped": abs(raw - servo_deg) > 1e-9,
            })
        if any(value is None for value in frame):
            raise CalibrationError("incomplete channel frame")
        return frame, details


def ordered_positions_from_joint_state(message):
    by_name = dict(zip(message.name, message.position))
    missing = [name for name in JOINT_NAMES if name not in by_name]
    if missing:
        raise CalibrationError("joint state missing names: %s" % missing)
    return [by_name[name] for name in JOINT_NAMES]


def named_positions_from_ordered(values):
    if len(values) != len(JOINT_NAMES):
        raise CalibrationError("expected %d joint values, got %d" % (len(JOINT_NAMES), len(values)))
    result = {}
    for name, value in zip(JOINT_NAMES, values):
        value = float(value)
        if not math.isfinite(value):
            raise CalibrationError("%s command is not finite" % name)
        result[name] = value
    return result
