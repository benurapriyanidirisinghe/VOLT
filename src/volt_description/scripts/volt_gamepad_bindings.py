#!/usr/bin/env python3

"""Operator-editable gamepad bindings for the VOLT control GUI.

Kept out of the GUI on purpose.  The binding set decides which physical
button reaches the STOP path on a live robot, so it gets its own validated
schema, its own file, and its own tests rather than living as a dict literal
halfway down a 5000-line Qt module.

The one invariant that is not negotiable: at least one input must be bound to
``stop``.  The arm workflow deliberately honours ``stop`` and nothing else
from the gamepad (volt_control_gui.poll_gamepad), so a binding set without it
would leave the operator holding a controller whose emergency action does not
exist.  validate_bindings() refuses that set.
"""

import os
from pathlib import Path

import yaml


class BindingError(ValueError):
    """Raised for a malformed or unsafe binding set."""


# --------------------------------------------------------------------------
# What can be bound
# --------------------------------------------------------------------------

# (action id, menu caption, group). The action id is what reaches
# volt_control_gui.handle_gamepad_action().
POSE_ACTIONS = (
    ("stop", "STOP", "Motion"),
    ("stand", "Stand Up", "Motion"),
    ("sit", "Sit", "Motion"),
    ("step", "Step In Place", "Motion"),
)

GAIT_ACTIONS = (
    ("prev_gait", "Previous gait", "Gait"),
    ("next_gait", "Next gait", "Gait"),
    ("gait:amble", "Select AMBLE", "Gait"),
    ("gait:trot", "Select TROT", "Gait"),
    ("gait:run", "Select RUN", "Gait"),
)

UI_ACTIONS = (
    ("reset_pose", "Reset body pose", "Console"),
    ("drive_mode", "Toggle drive mode", "Console"),
    ("speed_up", "Speed limit +10%", "Console"),
    ("speed_down", "Speed limit -10%", "Console"),
)

# Deliberately NOT bindable: "zero yaw trim". While a gamepad is enabled,
# poll_gamepad rewrites the yaw slider from the right stick every 30 ms, so
# zeroing it from a button is overwritten before the operator can see it. A
# binding that provably cannot take effect is worse than no binding at all.

# Mirrors DISPLAYED_CARTESIAN_EMOTES in the GUI. Controller-owned finite
# emotes; the controller still refuses any it has not advertised.
EMOTE_ACTIONS = tuple(
    ("emote:%s" % name, caption, "Emote")
    for caption, name in (
        ("Push-ups", "push_ups"),
        ("Body roll", "body_roll"),
        ("Nod / Yes", "nod"),
        ("Wave left", "wave_left"),
        ("Wave right", "wave_right"),
        ("Heart", "heart"),
        ("Bow", "bow"),
        ("Stretch", "stretch"),
        ("Happy dance", "happy_dance"),
        ("Shake / No", "shake_no"),
    )
)

FACE_ACTIONS = tuple(
    ("face:%s" % name, "Face: %s" % name.replace("_", " "), "Face")
    for name in (
        "neutral", "happy", "sad", "angry", "excited", "love", "playful",
        "confused", "thinking", "alert", "scared", "sleeping", "success",
        "error", "idle",
    )
)

UNBOUND = ""

BINDABLE_ACTIONS = (
    (UNBOUND, "— unbound —", "None"),
) + POSE_ACTIONS + GAIT_ACTIONS + UI_ACTIONS + EMOTE_ACTIONS + FACE_ACTIONS

ACTION_IDS = tuple(action for action, _caption, _group in BINDABLE_ACTIONS)
ACTION_CAPTIONS = {
    action: caption for action, caption, _group in BINDABLE_ACTIONS
}

# Inputs the GUI can offer. Buttons are indexed the way pygame reports them,
# which varies between controllers -- the tab shows a live pressed indicator
# so the operator can identify each one rather than guess from the number.
MAX_BUTTONS = 20
HAT_INPUTS = ("hat_left", "hat_right", "hat_up", "hat_down")

# --------------------------------------------------------------------------
# Axes (the sticks)
# --------------------------------------------------------------------------
#
# A stick is continuous, so it cannot be bound to an action the way a button
# is -- it drives a signal. These are the signals the console has:
#
#   drive_forward     the forward/back component of the drive vector
#   drive_horizontal  steer in Normal mode, strafe in Crab mode -- the drive
#                     mode decides which, exactly as the on-screen joystick
#                     already behaves
#   yaw_trim          the additive heading bias
#
# invert exists because the sign convention is a property of the pad, not of
# the robot: most pads report stick-up as NEGATIVE, which is why the
# hard-coded version this replaces read `set_vector(-left_y, left_x)`.
AXIS_FUNCTIONS = (
    ("", "— unused —"),
    ("drive_forward", "Drive forward / back"),
    ("drive_horizontal", "Steer / strafe"),
    ("yaw_trim", "Yaw trim"),
)
AXIS_FUNCTION_IDS = tuple(name for name, _caption in AXIS_FUNCTIONS)
AXIS_FUNCTION_CAPTIONS = dict(AXIS_FUNCTIONS)

# Each signal may be driven by at most one axis; two axes fighting over
# drive_forward is a bug the operator would feel as a stick that half works.
EXCLUSIVE_AXIS_FUNCTIONS = tuple(
    name for name in AXIS_FUNCTION_IDS if name
)

MAX_AXES = 8


def axis_input(index):
    return "axis_%d" % int(index)


def axis_caption(name):
    if str(name).startswith("axis_"):
        return "Axis %s" % str(name).split("_")[1]
    return str(name)


def all_axis_names(axis_count=MAX_AXES):
    count = max(0, min(int(axis_count), MAX_AXES))
    return tuple(axis_input(i) for i in range(count))


# Reproduces the hard-coded behaviour this replaces exactly:
#   left_x -> horizontal, -left_y -> forward, -right_x -> yaw trim
DEFAULT_AXIS_BINDINGS = {
    "axis_0": {"function": "drive_horizontal", "invert": False},
    "axis_1": {"function": "drive_forward", "invert": True},
    "axis_2": {"function": "yaw_trim", "invert": True},
    "axis_3": {"function": "", "invert": False},
    "axis_4": {"function": "", "invert": False},
    "axis_5": {"function": "", "invert": False},
    "axis_6": {"function": "", "invert": False},
    "axis_7": {"function": "", "invert": False},
}


def validate_axis_bindings(raw):
    """Return a clean axis mapping, or raise BindingError."""
    if not isinstance(raw, dict):
        raise BindingError("axis bindings must be a mapping")
    valid = set(all_axis_names())
    result = {}
    for name, entry in raw.items():
        key = str(name).strip()
        if key not in valid:
            raise BindingError("unknown axis '%s'" % key)
        if not isinstance(entry, dict):
            raise BindingError("%s must be a mapping" % key)
        unknown = sorted(set(entry) - {"function", "invert"})
        if unknown:
            raise BindingError("%s has unknown keys: %s" % (key, unknown))
        function = str(entry.get("function", "") or "").strip()
        if function not in AXIS_FUNCTION_IDS:
            raise BindingError(
                "unknown axis function '%s' on %s" % (function, key)
            )
        invert = entry.get("invert", False)
        if not isinstance(invert, bool):
            raise BindingError("%s invert must be true or false" % key)
        result[key] = {"function": function, "invert": invert}
    for function in EXCLUSIVE_AXIS_FUNCTIONS:
        owners = sorted(
            key for key, entry in result.items()
            if entry["function"] == function
        )
        if len(owners) > 1:
            raise BindingError(
                "%s is driven by more than one axis (%s); each signal takes "
                "exactly one" % (function, ", ".join(owners))
            )
    return result


def resolve_axis(axis_bindings, index):
    """Return (function, invert) for a physical axis index."""
    entry = axis_bindings.get(axis_input(index))
    if not isinstance(entry, dict):
        return "", False
    return entry.get("function", ""), bool(entry.get("invert", False))


def button_input(index):
    return "button_%d" % int(index)


def input_caption(name):
    """Human label for an input id."""
    if name in HAT_INPUTS:
        return "D-pad %s" % name.split("_")[1]
    if name.startswith("button_"):
        return "Button %s" % name.split("_")[1]
    return str(name)


def all_input_names(button_count=MAX_BUTTONS):
    count = max(0, min(int(button_count), MAX_BUTTONS))
    return tuple(button_input(i) for i in range(count)) + HAT_INPUTS


# --------------------------------------------------------------------------
# Defaults -- exactly the hard-coded map this file replaces
# --------------------------------------------------------------------------

DEFAULT_BINDINGS = {
    "button_0": "stand",       # A / Cross
    "button_1": "sit",         # B / Circle
    "button_2": "stop",        # X / Square
    "button_3": "step",        # Y / Triangle
    "button_4": "prev_gait",   # LB / L1
    "button_5": "next_gait",   # RB / R1
    "button_6": "reset_pose",  # Back / Select
    "button_7": "drive_mode",  # Start / Options
    "button_8": "stop",        # Left stick press
    "hat_left": "prev_gait",
    "hat_right": "next_gait",
    "hat_up": UNBOUND,
    "hat_down": UNBOUND,
}


def validate_bindings(raw):
    """Return a clean binding mapping, or raise BindingError.

    Unknown inputs and unknown actions are rejected rather than dropped: a
    silently discarded binding is a button the operator believes is mapped.
    """
    if not isinstance(raw, dict):
        raise BindingError("bindings must be a mapping")
    valid_inputs = set(all_input_names())
    result = {}
    for name, action in raw.items():
        key = str(name).strip()
        if key not in valid_inputs:
            raise BindingError("unknown input '%s'" % key)
        value = "" if action is None else str(action).strip()
        if value not in ACTION_IDS:
            raise BindingError(
                "unknown action '%s' bound to %s" % (value, key)
            )
        result[key] = value
    if not any(action == "stop" for action in result.values()):
        raise BindingError(
            "at least one input must be bound to STOP: the arm workflow "
            "accepts only STOP from the gamepad, so a set without it leaves "
            "the operator with no emergency action on the controller"
        )
    return result


def bindings_path():
    """Per-user writable location for the saved binding set."""
    config_root = os.environ.get("XDG_CONFIG_HOME", "").strip()
    if not config_root:
        config_root = str(Path.home() / ".config")
    return Path(config_root) / "volt_description" / "gamepad_bindings.yaml"


def load_bindings(path=None):
    """Load the operator's button bindings, falling back to the defaults.

    A corrupt or unsafe file must not leave the console with no STOP button,
    so this reports the problem and returns the defaults rather than raising
    into GUI construction.
    """
    target = Path(path or bindings_path()).expanduser()
    if not target.is_file():
        return dict(DEFAULT_BINDINGS), ""
    try:
        raw = yaml.safe_load(target.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        return dict(DEFAULT_BINDINGS), "Could not read %s: %s" % (target, exc)
    if not isinstance(raw, dict):
        return dict(DEFAULT_BINDINGS), "%s is not a mapping" % target
    stored = raw.get("bindings", raw)
    if not isinstance(stored, dict):
        # Silently ignoring this would hand back the defaults while the
        # operator believes their file is in force.
        return (
            dict(DEFAULT_BINDINGS),
            "%s has a malformed 'bindings' section (expected a mapping)"
            % target,
        )
    merged = dict(DEFAULT_BINDINGS)
    merged.update(stored)
    try:
        return validate_bindings(merged), ""
    except BindingError as exc:
        return dict(DEFAULT_BINDINGS), "%s rejected: %s" % (target, exc)


def load_axis_bindings(path=None):
    """Load the operator's axis mapping, falling back to the defaults.

    Same contract as load_bindings: a broken file reports and falls back
    rather than raising into GUI construction. A console that will not open
    is worse than one running default sticks.
    """
    target = Path(path or bindings_path()).expanduser()
    if not target.is_file():
        return _copy_axes(DEFAULT_AXIS_BINDINGS), ""
    try:
        raw = yaml.safe_load(target.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        return (
            _copy_axes(DEFAULT_AXIS_BINDINGS),
            "Could not read %s: %s" % (target, exc),
        )
    if not isinstance(raw, dict):
        return _copy_axes(DEFAULT_AXIS_BINDINGS), "%s is not a mapping" % target
    stored = raw.get("axes")
    if stored is None:
        # A file written before axes were bindable is not an error.
        return _copy_axes(DEFAULT_AXIS_BINDINGS), ""
    if not isinstance(stored, dict):
        return (
            _copy_axes(DEFAULT_AXIS_BINDINGS),
            "%s has a malformed 'axes' section (expected a mapping)" % target,
        )
    merged = _copy_axes(DEFAULT_AXIS_BINDINGS)
    for key, entry in stored.items():
        if isinstance(entry, dict):
            merged[str(key)] = dict(entry)
        else:
            merged[str(key)] = entry
    try:
        return validate_axis_bindings(merged), ""
    except BindingError as exc:
        return (
            _copy_axes(DEFAULT_AXIS_BINDINGS),
            "%s axes rejected: %s" % (target, exc),
        )


def _copy_axes(axes):
    return {key: dict(value) for key, value in axes.items()}


def save_bindings(bindings, path=None, axis_bindings=None):
    """Validate and persist atomically. Returns the path written.

    Both sections are written together: they describe one controller, and
    writing only half would leave the file self-inconsistent after an edit.
    """
    validated = validate_bindings(bindings)
    validated_axes = validate_axis_bindings(
        _copy_axes(DEFAULT_AXIS_BINDINGS) if axis_bindings is None
        else axis_bindings
    )
    target = Path(path or bindings_path()).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(
            {
                "version": 1,
                "bindings": validated,
                "axes": validated_axes,
            },
            handle,
            sort_keys=True,
        )
    temporary.replace(target)
    return target


def resolve(bindings, input_name):
    """Action bound to an input, or "" when unbound."""
    return bindings.get(str(input_name), UNBOUND)


def reachable_stop_inputs(bindings, button_count=None):
    """Inputs bound to STOP that the attached pad can actually produce.

    ``button_count`` is what pygame reports for the connected controller, or
    None when nothing is connected (in which case every input is assumed
    reachable, since there is no pad to contradict it).

    This exists because the binding grid offers button_0..button_19 no matter
    what is plugged in.  Binding STOP to Button 14 on an 11-button pad passes
    validate_bindings and leaves the console believing STOP is covered while
    no physical control can send it.
    """
    reachable = []
    for name, action in bindings.items():
        if action != "stop":
            continue
        if button_count is not None and name.startswith("button_"):
            try:
                if int(name.split("_")[1]) >= int(button_count):
                    continue
            except (ValueError, IndexError):
                continue
        reachable.append(name)
    return reachable
