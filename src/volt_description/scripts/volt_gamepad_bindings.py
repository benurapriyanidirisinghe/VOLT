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
    """Load the operator's bindings, falling back to the defaults.

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


def save_bindings(bindings, path=None):
    """Validate and persist atomically. Returns the path written."""
    validated = validate_bindings(bindings)
    target = Path(path or bindings_path()).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(
            {"version": 1, "bindings": validated}, handle, sort_keys=True
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
