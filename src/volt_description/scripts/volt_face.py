#!/usr/bin/env python3

"""Pure face-preset configuration, persistence, and automatic selection.

The module intentionally has no ROS or Qt dependency.  Callers publish only
the returned changes; LED animation remains a nonblocking firmware concern.
"""

from __future__ import annotations

import json
import os
import tempfile
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from types import MappingProxyType
from typing import Mapping, Optional, Tuple

import yaml


SUPPORTED_EFFECTS = (
    "solid",
    "breathe",
    "blink",
    "pulse",
    "rainbow",
    "chase",
    "scanner",
    "sparkle",
    "alternate",
    "loading",
    "off",
)
MIN_SPEED_MS = 10
MAX_SPEED_MS = 60000
SETTINGS_SCHEMA_VERSION = 1


class FaceConfigError(ValueError):
    """Raised when shipped or persisted face settings are invalid."""


def _integer(value, minimum, maximum, label):
    if isinstance(value, bool) or not isinstance(value, int):
        raise FaceConfigError("%s must be an integer" % label)
    if not minimum <= value <= maximum:
        raise FaceConfigError(
            "%s must be in [%d, %d]" % (label, minimum, maximum)
        )
    return value


def _color(value, label):
    if isinstance(value, (str, bytes)):
        raise FaceConfigError("%s must contain three integers" % label)
    try:
        values = tuple(value)
    except TypeError as exc:
        raise FaceConfigError("%s must contain three integers" % label) from exc
    if len(values) != 3:
        raise FaceConfigError("%s must contain exactly three integers" % label)
    return tuple(
        _integer(component, 0, 255, "%s[%d]" % (label, index))
        for index, component in enumerate(values)
    )


def _name(value, label):
    if not isinstance(value, str):
        raise FaceConfigError("%s must be a string" % label)
    name = value.strip().lower()
    if not name or any(
        not (character.islower() or character.isdigit() or character == "_")
        for character in name
    ):
        raise FaceConfigError("%s is not a lower-case identifier" % label)
    return name


@dataclass(frozen=True)
class FacePreset:
    name: str
    color: Tuple[int, int, int]
    effect: str
    brightness: int
    speed_ms: int
    alternate_color: Optional[Tuple[int, int, int]] = None


@dataclass(frozen=True)
class FaceCatalog:
    default_expression: str
    default_brightness: int
    default_speed_ms: int
    presets: Mapping[str, FacePreset]
    emote_mappings: Mapping[str, str]
    state_mappings: Mapping[str, str]
    safety_mappings: Mapping[str, str]


@dataclass(frozen=True)
class FaceSettings:
    enabled: bool = True
    automatic: bool = True
    locked: bool = False
    expression: str = "idle"
    color: Tuple[int, int, int] = (0, 120, 255)
    alternate_color: Tuple[int, int, int] = (0, 120, 255)
    brightness: int = 80
    effect: str = "breathe"
    speed_ms: int = 2200


@dataclass(frozen=True)
class FaceDecision:
    expression: str
    reason: str
    safety_override: bool = False
    restored: bool = False


def default_face_config_path():
    source = Path(__file__).resolve().parents[1] / "config" / "face_expressions.yaml"
    if source.is_file():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory("volt_description"))
            / "config"
            / "face_expressions.yaml"
        )
    except (ImportError, LookupError):
        return source


def _mapping(value, label):
    if not isinstance(value, dict):
        raise FaceConfigError("%s must be a mapping" % label)
    return value


def _exact_keys(value, allowed, required, label):
    unknown = sorted(set(value) - set(allowed))
    missing = sorted(set(required) - set(value))
    if unknown:
        raise FaceConfigError("%s has unknown keys: %s" % (label, unknown))
    if missing:
        raise FaceConfigError("%s is missing keys: %s" % (label, missing))


def load_face_catalog(path=None):
    path = Path(path or default_face_config_path()).expanduser().resolve()
    try:
        with path.open("r", encoding="utf-8") as handle:
            root = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        raise FaceConfigError("could not load face YAML %s: %s" % (path, exc)) from exc
    root = _mapping(root, "face YAML root")
    _exact_keys(
        root,
        {"schema_version", "defaults", "expressions", "automatic_mappings"},
        {"schema_version", "defaults", "expressions", "automatic_mappings"},
        "face YAML root",
    )
    if root["schema_version"] != 1 or isinstance(root["schema_version"], bool):
        raise FaceConfigError("schema_version must be 1")
    defaults = _mapping(root["defaults"], "defaults")
    _exact_keys(
        defaults,
        {"expression", "brightness", "speed_ms"},
        {"expression", "brightness", "speed_ms"},
        "defaults",
    )
    presets = {}
    for raw_name, raw_value in _mapping(root["expressions"], "expressions").items():
        name = _name(raw_name, "expression name")
        value = _mapping(raw_value, "expression %s" % name)
        _exact_keys(
            value,
            {"color", "alternate_color", "effect", "brightness", "speed_ms"},
            {"color", "effect", "brightness", "speed_ms"},
            "expression %s" % name,
        )
        effect = _name(value["effect"], "expression %s effect" % name)
        if effect not in SUPPORTED_EFFECTS:
            raise FaceConfigError("expression %s uses unsupported effect %s" % (name, effect))
        alternate = value.get("alternate_color")
        presets[name] = FacePreset(
            name=name,
            color=_color(value["color"], "expression %s color" % name),
            alternate_color=(
                _color(alternate, "expression %s alternate_color" % name)
                if alternate is not None
                else None
            ),
            effect=effect,
            brightness=_integer(value["brightness"], 0, 255, "expression %s brightness" % name),
            speed_ms=_integer(value["speed_ms"], MIN_SPEED_MS, MAX_SPEED_MS, "expression %s speed_ms" % name),
        )
    if not presets:
        raise FaceConfigError("expressions must not be empty")
    default_expression = _name(defaults["expression"], "defaults.expression")
    if default_expression not in presets:
        raise FaceConfigError("default expression is not defined")

    automatic = _mapping(root["automatic_mappings"], "automatic_mappings")
    _exact_keys(
        automatic,
        {"emotes", "states", "safety"},
        {"emotes", "states", "safety"},
        "automatic_mappings",
    )

    def mappings(section):
        result = {}
        for raw_source, raw_target in _mapping(automatic[section], section).items():
            source = _name(raw_source, "%s source" % section)
            target = _name(raw_target, "%s target" % section)
            if target not in presets:
                raise FaceConfigError("%s maps %s to unknown expression %s" % (section, source, target))
            result[source] = target
        return MappingProxyType(result)

    return FaceCatalog(
        default_expression=default_expression,
        default_brightness=_integer(defaults["brightness"], 0, 255, "defaults.brightness"),
        default_speed_ms=_integer(defaults["speed_ms"], MIN_SPEED_MS, MAX_SPEED_MS, "defaults.speed_ms"),
        presets=MappingProxyType(presets),
        emote_mappings=mappings("emotes"),
        state_mappings=mappings("states"),
        safety_mappings=mappings("safety"),
    )


def settings_for_preset(catalog, expression, base=None):
    expression = _name(expression, "expression")
    try:
        preset = catalog.presets[expression]
    except KeyError as exc:
        raise FaceConfigError("unknown expression %r" % expression) from exc
    current = base or FaceSettings()
    return replace(
        current,
        expression=expression,
        color=preset.color,
        alternate_color=preset.alternate_color or preset.color,
        brightness=preset.brightness,
        effect=preset.effect,
        speed_ms=preset.speed_ms,
    )


def validate_face_settings(value, catalog):
    if isinstance(value, FaceSettings):
        raw = asdict(value)
    elif isinstance(value, dict):
        raw = dict(value)
    else:
        raise FaceConfigError("face settings must be a mapping")
    allowed = {
        "enabled", "automatic", "locked", "expression", "color",
        "alternate_color", "brightness", "effect", "speed_ms",
    }
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise FaceConfigError("face settings have unknown keys: %s" % unknown)
    expression = _name(raw.get("expression", catalog.default_expression), "expression")
    if expression not in catalog.presets:
        raise FaceConfigError("unknown expression %r" % expression)
    preset = catalog.presets[expression]
    effect = _name(raw.get("effect", preset.effect), "effect")
    if effect not in SUPPORTED_EFFECTS:
        raise FaceConfigError("unsupported effect %r" % effect)
    for flag in ("enabled", "automatic", "locked"):
        if flag in raw and not isinstance(raw[flag], bool):
            raise FaceConfigError("%s must be boolean" % flag)
    color = _color(raw.get("color", preset.color), "color")
    alternate_default = preset.alternate_color or color
    return FaceSettings(
        enabled=raw.get("enabled", True),
        automatic=raw.get("automatic", True),
        locked=raw.get("locked", False),
        expression=expression,
        color=color,
        alternate_color=_color(
            raw.get("alternate_color", alternate_default),
            "alternate_color",
        ),
        brightness=_integer(raw.get("brightness", preset.brightness), 0, 255, "brightness"),
        effect=effect,
        speed_ms=_integer(raw.get("speed_ms", preset.speed_ms), MIN_SPEED_MS, MAX_SPEED_MS, "speed_ms"),
    )


def default_face_settings(catalog):
    preset = catalog.presets[catalog.default_expression]
    return FaceSettings(
        expression=catalog.default_expression,
        color=preset.color,
        alternate_color=preset.alternate_color or preset.color,
        brightness=catalog.default_brightness,
        effect=preset.effect,
        speed_ms=catalog.default_speed_ms,
    )


def default_settings_path(environ=None):
    environ = os.environ if environ is None else environ
    config_root = str(environ.get("XDG_CONFIG_HOME", "")).strip()
    if config_root:
        root = Path(config_root).expanduser()
    else:
        home = str(environ.get("HOME", "")).strip()
        root = (Path(home).expanduser() if home else Path.home()) / ".config"
    return root / "volt" / "face_led_settings.json"


def load_face_settings(catalog, path=None):
    path = Path(path or default_settings_path()).expanduser()
    fallback = default_face_settings(catalog)
    if not path.is_file():
        return fallback
    try:
        with path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        if not isinstance(payload, dict) or payload.get("schema_version") != SETTINGS_SCHEMA_VERSION:
            raise FaceConfigError("settings schema_version must be 1")
        settings = dict(payload)
        settings.pop("schema_version")
        return validate_face_settings(settings, catalog)
    except (OSError, ValueError, json.JSONDecodeError, FaceConfigError):
        return fallback


def save_face_settings(settings, catalog, path=None):
    settings = validate_face_settings(settings, catalog)
    path = Path(path or default_settings_path()).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = asdict(settings)
    payload["color"] = list(settings.color)
    payload["alternate_color"] = list(settings.alternate_color)
    payload["schema_version"] = SETTINGS_SCHEMA_VERSION
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=str(path.parent),
            prefix=path.name + ".",
            suffix=".tmp",
            delete=False,
        ) as handle:
            temporary_name = handle.name
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, path)
    except OSError:
        if temporary_name:
            try:
                os.unlink(temporary_name)
            except OSError:
                pass
        raise
    return path


def _truthy(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on", "active")
    return bool(value)


def _fault_active(value):
    if isinstance(value, str):
        return value.strip().lower() not in ("", "0", "false", "no", "off", "none", "-")
    return bool(value)


def safety_condition(status):
    """Return an explicit safety class without treating routine warnings as faults."""
    if not isinstance(status, dict):
        return ""
    for key in ("emergency_stop", "estop", "e_stop"):
        if _truthy(status.get(key, False)):
            return "emergency"
    if _truthy(status.get("low_voltage", False)) or _truthy(status.get("undervoltage", False)):
        return "low_voltage"
    fault = status.get("critical_fault", status.get("fault", False))
    if _fault_active(fault):
        return "fault"
    warning = str(status.get("warning", "")).strip().lower()
    if "emergency stop" in warning or "e-stop" in warning:
        return "emergency"
    if "low voltage" in warning or "undervoltage" in warning:
        return "low_voltage"
    if "critical fault" in warning:
        return "fault"
    return ""


class FaceAutomation:
    """Edge-triggered automatic expression selection with restoration."""

    def __init__(self, catalog, settings):
        self.catalog = catalog
        self.settings = validate_face_settings(settings, catalog)
        self._automatic_context = ""
        self._last_expression = self.settings.expression

    def set_settings(self, settings):
        self.settings = validate_face_settings(settings, self.catalog)
        # A manual edit establishes a new restoration target.  Re-evaluate the
        # current robot context on the next status edge.
        self._automatic_context = ""
        self._last_expression = self.settings.expression

    def _automatic_target(self, status):
        safety = safety_condition(status)
        if safety:
            return self.catalog.safety_mappings.get(safety, "error"), "safety:%s" % safety, True

        if self.settings.locked or not self.settings.automatic:
            return self.settings.expression, "manual", False

        emote_active = _truthy(status.get("emote_active", False)) or _truthy(status.get("emote_pending", False))
        emote_name = str(status.get("emote_name", "")).strip().lower()
        if emote_active and emote_name:
            expression = self.catalog.emote_mappings.get(emote_name)
            if expression:
                return expression, "emote:%s" % emote_name, False

        owner = str(status.get("command_owner", "")).strip().lower()
        if owner == "calibration" or _truthy(status.get("calibration_mode", False)):
            return self.catalog.state_mappings.get("calibration", "thinking"), "state:calibration", False

        state = str(status.get("state", "")).strip().lower()
        if state in self.catalog.state_mappings:
            return self.catalog.state_mappings[state], "state:%s" % state, False

        moving = _truthy(status.get("motion_active", status.get("moving", False)))
        if moving:
            return self.catalog.state_mappings.get("walking", "idle"), "state:walking", False
        return self.settings.expression, "manual", False

    def update(self, status):
        expression, context, safety = self._automatic_target(status or {})
        next_context = context if context != "manual" else ""
        restored = bool(self._automatic_context and not next_context)
        changed = (
            expression != self._last_expression
            or next_context != self._automatic_context
        )
        self._automatic_context = next_context
        self._last_expression = expression
        if not changed:
            return None
        return FaceDecision(
            expression=expression,
            reason=context,
            safety_override=safety,
            restored=restored,
        )
