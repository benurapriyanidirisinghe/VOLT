#!/usr/bin/env python3

"""Tests for the operator-editable gamepad binding set."""

import sys
import unittest
from pathlib import Path

import yaml

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "scripts"),
)

import volt_gamepad_bindings as bindings
from volt_gamepad_bindings import (
    ACTION_IDS,
    BINDABLE_ACTIONS,
    BindingError,
    DEFAULT_BINDINGS,
    HAT_INPUTS,
    all_input_names,
    button_input,
    input_caption,
    load_bindings,
    resolve,
    save_bindings,
    validate_bindings,
)


class CatalogueTests(unittest.TestCase):
    def test_action_ids_are_unique(self):
        self.assertEqual(len(ACTION_IDS), len(set(ACTION_IDS)))

    def test_every_action_has_a_caption_and_group(self):
        for action, caption, group in BINDABLE_ACTIONS:
            self.assertTrue(caption.strip(), action)
            self.assertTrue(group.strip(), action)

    def test_unbound_is_offered(self):
        self.assertIn("", ACTION_IDS)

    def test_emotes_match_the_console_catalogue(self):
        """A bindable emote the GUI cannot start would be a dead binding."""
        source = (
            Path(__file__).resolve().parents[1]
            / "scripts" / "volt_control_gui.py"
        ).read_text()
        start = source.index("DISPLAYED_CARTESIAN_EMOTES = (")
        block = source[start:source.index("\n)", start)]
        displayed = {
            line.split('"')[3]
            for line in block.splitlines()
            if line.count('"') >= 4
        }
        bindable = {
            action.partition(":")[2]
            for action in ACTION_IDS
            if action.startswith("emote:")
        }
        self.assertEqual(displayed, bindable)

    def test_faces_match_the_face_catalogue(self):
        presets = set(
            yaml.safe_load(
                (
                    Path(__file__).resolve().parents[1]
                    / "config" / "face_expressions.yaml"
                ).read_text()
            )["expressions"]
        )
        bindable = {
            action.partition(":")[2]
            for action in ACTION_IDS
            if action.startswith("face:")
        }
        self.assertTrue(
            bindable <= presets,
            "bindable faces not in the catalogue: %s" % (bindable - presets),
        )

    def test_input_names_cover_buttons_and_hat(self):
        names = all_input_names(4)
        self.assertEqual(
            names, ("button_0", "button_1", "button_2", "button_3") + HAT_INPUTS
        )
        self.assertEqual("Button 3", input_caption("button_3"))
        self.assertEqual("D-pad left", input_caption("hat_left"))


class ValidationTests(unittest.TestCase):
    def test_defaults_are_valid(self):
        self.assertEqual(DEFAULT_BINDINGS, validate_bindings(DEFAULT_BINDINGS))

    def test_stop_is_required(self):
        """The arm workflow honours only STOP from the pad."""
        without_stop = {
            key: ("" if value == "stop" else value)
            for key, value in DEFAULT_BINDINGS.items()
        }
        with self.assertRaises(BindingError) as caught:
            validate_bindings(without_stop)
        self.assertIn("STOP", str(caught.exception))

    def test_unknown_input_is_rejected_not_dropped(self):
        broken = dict(DEFAULT_BINDINGS)
        broken["button_99"] = "stop"
        with self.assertRaises(BindingError):
            validate_bindings(broken)

    def test_unknown_action_is_rejected_not_dropped(self):
        broken = dict(DEFAULT_BINDINGS)
        broken["button_1"] = "self_destruct"
        with self.assertRaises(BindingError):
            validate_bindings(broken)

    def test_non_mapping_is_rejected(self):
        with self.assertRaises(BindingError):
            validate_bindings(["button_0", "stop"])

    def test_every_bindable_action_survives_validation(self):
        for action in ACTION_IDS:
            candidate = dict(DEFAULT_BINDINGS)
            candidate["button_1"] = action
            self.assertEqual(action, validate_bindings(candidate)["button_1"])


class PersistenceTests(unittest.TestCase):
    def setUp(self):
        import tempfile

        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / "gamepad_bindings.yaml"

    def tearDown(self):
        self.directory.cleanup()

    def test_round_trip(self):
        custom = dict(DEFAULT_BINDINGS)
        custom["button_3"] = "emote:heart"
        custom["hat_up"] = "face:happy"
        save_bindings(custom, self.path)
        loaded, error = load_bindings(self.path)
        self.assertEqual("", error)
        self.assertEqual("emote:heart", loaded["button_3"])
        self.assertEqual("face:happy", loaded["hat_up"])

    def test_saving_an_unsafe_set_is_refused(self):
        unsafe = {key: "" for key in DEFAULT_BINDINGS}
        with self.assertRaises(BindingError):
            save_bindings(unsafe, self.path)
        self.assertFalse(self.path.exists())

    def test_missing_file_returns_defaults(self):
        loaded, error = load_bindings(self.path)
        self.assertEqual(DEFAULT_BINDINGS, loaded)
        self.assertEqual("", error)

    def test_corrupt_file_falls_back_without_raising(self):
        """A broken file must not leave the console with no STOP."""
        self.path.write_text("bindings: [this is not a mapping]\n")
        loaded, error = load_bindings(self.path)
        self.assertEqual(DEFAULT_BINDINGS, loaded)
        self.assertTrue(error)

    def test_unsafe_file_falls_back_and_reports(self):
        """Unbinding EVERY stop control is refused; defaults come back."""
        unsafe = {
            key: ("" if value == "stop" else value)
            for key, value in DEFAULT_BINDINGS.items()
        }
        self.path.write_text(yaml.safe_dump({"bindings": unsafe}))
        loaded, error = load_bindings(self.path)
        self.assertEqual("stop", loaded["button_2"])
        self.assertIn("STOP", error)

    def test_unbinding_one_stop_is_allowed_while_another_remains(self):
        """Two controls default to STOP; dropping one is a legitimate edit."""
        partial = dict(DEFAULT_BINDINGS)
        partial["button_2"] = "emote:bow"
        self.path.write_text(yaml.safe_dump({"bindings": partial}))
        loaded, error = load_bindings(self.path)
        self.assertEqual("", error)
        self.assertEqual("emote:bow", loaded["button_2"])
        self.assertEqual("stop", loaded["button_8"])

    def test_partial_file_merges_over_defaults(self):
        self.path.write_text(
            yaml.safe_dump({"bindings": {"button_3": "emote:bow"}})
        )
        loaded, error = load_bindings(self.path)
        self.assertEqual("", error)
        self.assertEqual("emote:bow", loaded["button_3"])
        self.assertEqual("stand", loaded["button_0"])


class ResolveTests(unittest.TestCase):
    def test_resolve_known_and_unknown(self):
        self.assertEqual("stand", resolve(DEFAULT_BINDINGS, "button_0"))
        self.assertEqual("", resolve(DEFAULT_BINDINGS, "button_17"))

    def test_button_input_formats_index(self):
        self.assertEqual("button_5", button_input(5))


class ConsoleIntegrationTests(unittest.TestCase):
    """The GUI must dispatch every prefixed action it can bind."""

    def setUp(self):
        self.source = (
            Path(__file__).resolve().parents[1]
            / "scripts" / "volt_control_gui.py"
        ).read_text()

    def test_no_hardcoded_button_map_remains(self):
        self.assertNotIn("BUTTON_ACTIONS", self.source)

    def test_dispatcher_handles_every_prefix(self):
        for prefix in ("emote:", "face:", "gait:"):
            self.assertIn(
                'action.startswith("%s")' % prefix,
                self.source,
                "handle_gamepad_action does not dispatch %s" % prefix,
            )

    def test_dispatcher_handles_every_plain_action(self):
        plain = [
            action for action in ACTION_IDS
            if action and ":" not in action
        ]
        for action in plain:
            self.assertIn(
                '"%s"' % action,
                self.source,
                "action %s is bindable but never referenced by the GUI"
                % action,
            )

    def test_arm_is_blocked_without_a_stop_binding(self):
        """An enabled pad with no STOP must not be able to arm the robot."""
        self.assertIn("ARM blocked: the gamepad is enabled but no control", self.source)
        self.assertIn(
            "reachable_stop_inputs(\n            self.gamepad_bindings, arm_stop_count\n        )",
            self.source,
        )

    def test_tab_label_carries_the_warning(self):
        """The banner lives on one tab; the label is always on screen."""
        self.assertIn('"GAMEPAD  ⚠ NO STOP"', self.source)

    def test_bindings_are_loaded_before_the_tab_is_built(self):
        load = self.source.index("self.gamepad_bindings, self.gamepad_binding_load_error")
        build = self.source.index("self.build_gamepad_tab(")
        self.assertLess(
            load, build,
            "build_gamepad_tab reads the bindings, so they must load first",
        )


class ReachableStopTests(unittest.TestCase):
    """STOP must sit on a control the connected pad can actually send."""

    def test_all_inputs_reachable_when_no_pad_is_connected(self):
        from volt_gamepad_bindings import reachable_stop_inputs

        found = reachable_stop_inputs(DEFAULT_BINDINGS, None)
        self.assertIn("button_2", found)
        self.assertIn("button_8", found)

    def test_button_beyond_the_pad_does_not_count(self):
        from volt_gamepad_bindings import reachable_stop_inputs

        bindings = dict(DEFAULT_BINDINGS)
        bindings["button_2"] = ""
        bindings["button_8"] = ""
        bindings["button_14"] = "stop"
        self.assertEqual([], reachable_stop_inputs(bindings, 11))
        self.assertEqual(["button_14"], reachable_stop_inputs(bindings, 16))

    def test_dpad_stop_counts_regardless_of_button_count(self):
        from volt_gamepad_bindings import reachable_stop_inputs

        bindings = dict(DEFAULT_BINDINGS)
        bindings["button_2"] = ""
        bindings["button_8"] = ""
        bindings["hat_up"] = "stop"
        self.assertEqual(["hat_up"], reachable_stop_inputs(bindings, 11))


class ArmWorkflowDpadTests(unittest.TestCase):
    """A D-pad STOP must work during the guided ARM sequence."""

    def setUp(self):
        self.source = (
            Path(__file__).resolve().parents[1]
            / "scripts" / "volt_control_gui.py"
        ).read_text()

    def test_arm_branch_reads_the_hat(self):
        arm = self.source.index("if self.arm_workflow.active:", self.source.index("def poll_gamepad"))
        branch = self.source[arm:arm + 1600]
        self.assertIn(
            "read_hat_inputs()", branch,
            "the arm-workflow branch must poll the D-pad, or a D-pad STOP is "
            "dead during the one sequence that accepts only STOP",
        )

    def test_arm_branch_records_edge_state_for_every_input(self):
        arm = self.source.index("if self.arm_workflow.active:", self.source.index("def poll_gamepad"))
        branch = self.source[arm:arm + 1600]
        self.assertIn("self.gamepad_buttons[key] = pressed", branch)

    def test_binding_combo_ignores_the_wheel(self):
        """Scrolling the grid must not silently rebind a control."""
        self.assertIn("class _NoWheelComboBox", self.source)
        self.assertIn("def wheelEvent", self.source)
        self.assertIn("self._NoWheelComboBox()", self.source)

    def test_zero_yaw_trim_is_not_bindable(self):
        """The right stick overwrites the slider every poll."""
        self.assertNotIn("yaw_trim_zero", self.source)

    def test_indicators_repaint_on_every_poll_path(self):
        """Four early returns used to freeze the live pressed dots."""
        self.assertIn("def _poll_gamepad_inputs(self):", self.source)
        wrapper = self.source[
            self.source.index("    def poll_gamepad(self):"):
            self.source.index("    def _poll_gamepad_inputs(self):")
        ]
        self.assertIn("finally:", wrapper)
        self.assertIn("self.refresh_gamepad_binding_indicators()", wrapper)
        self.assertIn("self.update_gamepad_status()", wrapper)

    def test_save_reports_filesystem_errors(self):
        """An unwritable config dir must not throw out of the slot."""
        save = self.source[
            self.source.index("    def save_gamepad_bindings(self):"):
        ][:900]
        self.assertIn("except OSError as exc:", save)


class DiagramTests(unittest.TestCase):
    """The clickable controller map, checked without starting Qt."""

    def setUp(self):
        self.source = (
            Path(__file__).resolve().parents[1]
            / "scripts" / "volt_control_gui.py"
        ).read_text()

    def _controls(self):
        """Parse PAD_CONTROLS without importing Qt."""
        import ast

        start = self.source.index("PAD_CONTROLS = (")
        end = self.source.index("\n)", start) + 2
        return ast.literal_eval(self.source[start:end].split("=", 1)[1].strip())

    def test_every_drawn_control_is_a_real_bindable_input(self):
        """A control on the diagram that no binding can reach is a lie."""
        valid = set(all_input_names())
        for name, *_rest in self._controls():
            self.assertIn(name, valid, "%s is drawn but not bindable" % name)

    def test_no_control_is_drawn_twice(self):
        names = [c[0] for c in self._controls()]
        self.assertEqual(len(names), len(set(names)))

    def test_all_four_dpad_directions_are_drawn(self):
        names = {c[0] for c in self._controls()}
        for direction in HAT_INPUTS:
            self.assertIn(direction, names)

    def test_default_stop_bindings_are_reachable_on_the_diagram(self):
        """The operator must be able to see and click their STOP."""
        drawn = {c[0] for c in self._controls()}
        stops = {k for k, v in DEFAULT_BINDINGS.items() if v == "stop"}
        self.assertTrue(
            stops & drawn,
            "no default STOP control appears on the diagram",
        )

    def test_callout_labels_stay_inside_the_coordinate_space(self):
        """Text drawn past the edge is text the widget clips.

        The first cut anchored labels 34 units from the edge and then drew a
        300-unit-wide box outward from there, so the right-hand column ran
        off the widget.
        """
        self.assertIn("PAD_LABEL_X = 290.0", self.source)
        self.assertIn("box_w = (PAD_LABEL_X - 14.0) * factor", self.source)
        controls = self._controls()
        # Every control must sit inside the gutters, not under a label.
        for name, x, _y, radius, *_rest in controls:
            self.assertGreater(
                x - radius, 290.0,
                "%s overlaps the left label gutter" % name,
            )
            self.assertLess(
                x + radius, 1600.0 - 290.0,
                "%s overlaps the right label gutter" % name,
            )

    def test_diagram_drives_selection_and_press_state(self):
        for hook in (
            "class GamepadDiagram",
            "controlClicked = pyqtSignal(str)",
            "def control_at(self, position)",
            "def set_pressed(self, names)",
            "diagram.set_pressed(pressed)",
            "self.gamepad_diagram.controlClicked.connect(self.select_gamepad_control)",
        ):
            self.assertIn(hook, self.source)

    def test_hit_test_and_drawing_share_one_table(self):
        """Drawn in one place and clickable in another is the classic bug."""
        control_at = self.source[
            self.source.index("def control_at(self, position)"):
        ][:600]
        self.assertIn("for name, x, y, radius, _glyph, _side, _row in PAD_CONTROLS", control_at)
        self.assertIn("self._point(x, y)", control_at)
