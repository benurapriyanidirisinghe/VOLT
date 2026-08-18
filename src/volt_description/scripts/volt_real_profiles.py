#!/usr/bin/env python3

"""Validated real-robot gait-conditioning profiles for V.O.L.T.

The values in this module stay in canonical SI units.  Servo calibration,
direction, channel order, and degree conversion remain exclusively owned by
``volt_serial_bridge.py``.
"""

import math
import os
import warnings
from pathlib import Path

import yaml


PROFILE_NAMES = (
    "SIMULATION",
    "REAL_DIAGNOSTIC",
    "REAL_SAFE",
    "REAL_NORMAL",
)
RESERVED_USER_PROFILE_NAMES = frozenset({"SIMULATION"})

TUNABLE_FIELDS = (
    "gait",
    "cycle_duration",
    "stride_length",
    "lateral_stride_width",
    "step_height",
    "duty_factor",
    "body_height",
    "body_x",
    "body_y",
    "body_roll_deg",
    "body_pitch_deg",
    "body_yaw_deg",
    "max_joint_velocity_deg_s",
    "max_joint_acceleration_deg_s2",
    "smoothing_amount",
    "touchdown_softness",
    "stance_width",
)

# These bounds are narrower than the reachable IK envelope.  Final joint
# commands are still clamped by the canonical URDF limits and the calibrated
# servo limits downstream.
NUMERIC_BOUNDS = {
    "cycle_duration": (0.60, 6.00),
    "stride_length": (0.005, 0.075),
    "lateral_stride_width": (0.0, 0.030),
    "step_height": (0.0, 0.045),
    "duty_factor": (0.55, 0.90),
    "body_height": (0.175, 0.220),
    "body_x": (-0.025, 0.025),
    "body_y": (-0.020, 0.020),
    # Both real-hardware gait engines enforce 0.08 rad. Keep the transaction
    # inside a decimal-degree bound that is never silently clipped later.
    "body_roll_deg": (-4.5, 4.5),
    "body_pitch_deg": (-4.5, 4.5),
    "body_yaw_deg": (-10.0, 10.0),
    # The firmware slews every channel at up to 240 deg/s; the gait engine
    # validates a 190 deg/s swing budget.  A 190 deg/s UI ceiling therefore
    # cannot bypass either downstream authority.
    "max_joint_velocity_deg_s": (60.0, 190.0),
    "max_joint_acceleration_deg_s2": (600.0, 6000.0),
    "smoothing_amount": (0.0, 0.80),
    "touchdown_softness": (0.08, 0.35),
    "stance_width": (0.080, 0.130),
}

TUNABLE_GAITS = (
    "trot",
    "amble",
)


class RealProfileError(ValueError):
    """Raised when a profile could bypass the bounded hardware envelope."""


def default_profile_path():
    """Find the installed/source profile file without assuming a workspace."""
    source = Path(__file__).resolve().parents[1] / "config" / "real_robot_profiles.yaml"
    if source.is_file():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory("volt_description"))
            / "config"
            / "real_robot_profiles.yaml"
        )
    except (ImportError, LookupError):
        return source


def user_profile_path():
    """Return the per-user writable overlay used by GUI Save Profile."""
    config_root = os.environ.get("XDG_CONFIG_HOME", "").strip()
    if not config_root:
        config_root = str(Path.home() / ".config")
    return Path(config_root) / "volt_description" / "real_robot_profiles.yaml"


def validate_tuning(values, allow_simulation=True):
    """Return a complete finite tuning dictionary with strict field names."""
    if not isinstance(values, dict):
        raise RealProfileError("profile must be a mapping")
    unknown = sorted(set(values) - set(TUNABLE_FIELDS))
    if unknown:
        raise RealProfileError("unknown profile fields: %s" % unknown)
    missing = [field for field in TUNABLE_FIELDS if field not in values]
    if missing:
        raise RealProfileError("profile is missing: %s" % missing)

    gait = str(values["gait"]).strip().lower()
    if gait not in TUNABLE_GAITS:
        raise RealProfileError(
            "gait must be one of %s" % (TUNABLE_GAITS,)
        )

    result = {"gait": gait}
    for field, (lower, upper) in NUMERIC_BOUNDS.items():
        if isinstance(values[field], bool):
            raise RealProfileError("%s must be numeric, not boolean" % field)
        try:
            value = float(values[field])
        except (TypeError, ValueError) as exc:
            raise RealProfileError("%s must be numeric" % field) from exc
        if not math.isfinite(value) or not lower <= value <= upper:
            raise RealProfileError(
                "%s must be finite and in [%.4g, %.4g]"
                % (field, lower, upper)
            )
        result[field] = value

    duty = result["duty_factor"]
    if gait == "amble" and not 0.70 - 1e-9 <= duty <= 0.86 + 1e-9:
        raise RealProfileError(
            "amble duty factor must keep at least three legs in stance"
        )
    if gait == "trot" and not 0.52 - 1e-9 <= duty <= 0.68 + 1e-9:
        raise RealProfileError("trot duty factor is outside its safe range")
    # Liftoff uses no more than the touchdown fraction, leaving at least 30%
    # of every swing for horizontal transfer at the upper 0.35 bound.
    return result


def _read_profile_mapping(path):
    try:
        with Path(path).expanduser().open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        raise RealProfileError("cannot load profile file %s: %s" % (path, exc)) from exc
    if not isinstance(raw, dict) or not isinstance(raw.get("profiles"), dict):
        raise RealProfileError("profile file must contain a profiles mapping")
    return raw["profiles"]


def load_profiles(path=None, include_user=True):
    """Load shipped profiles and overlay valid user-saved profiles."""
    profiles = {}
    for name, values in _read_profile_mapping(path or default_profile_path()).items():
        normalized_name = str(name).strip().upper()
        profiles[normalized_name] = validate_tuning(values)

    overlay_path = user_profile_path()
    if include_user and path is None and overlay_path.is_file():
        for name, values in _read_profile_mapping(overlay_path).items():
            normalized_name = str(name).strip().upper()
            if normalized_name in RESERVED_USER_PROFILE_NAMES:
                warnings.warn(
                    "Ignoring reserved user profile %s; the shipped simulator "
                    "profile is read-only." % normalized_name,
                    RuntimeWarning,
                    stacklevel=2,
                )
                continue
            profiles[normalized_name] = validate_tuning(values)

    missing = [name for name in PROFILE_NAMES if name not in profiles]
    if missing:
        raise RealProfileError("required profiles are missing: %s" % missing)
    return profiles


def save_user_profile(name, values, path=None):
    """Persist one validated profile atomically in the user overlay."""
    normalized_name = str(name).strip().upper()
    if not normalized_name or any(
        character not in "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-"
        for character in normalized_name
    ):
        raise RealProfileError("profile name may contain only A-Z, 0-9, _ and -")
    if normalized_name in RESERVED_USER_PROFILE_NAMES:
        raise RealProfileError(
            "%s is reserved and read-only" % normalized_name
        )
    validated = validate_tuning(values)
    target = Path(path or user_profile_path()).expanduser()
    existing = {}
    if target.is_file():
        existing = {
            str(key).strip().upper(): validate_tuning(value)
            for key, value in _read_profile_mapping(target).items()
        }
    existing[normalized_name] = validated
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(
            {"version": 1, "profiles": existing},
            handle,
            sort_keys=False,
        )
    temporary.replace(target)
    return target


def smoothing_alpha(tuning):
    """Convert intuitive smoothing amount to the controller's tracking alpha."""
    return max(0.20, min(1.0, 1.0 - float(tuning["smoothing_amount"])))
