#!/usr/bin/env python3

"""Pure regression tests for the Cartesian emote engine and shipped catalog."""

import math
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
CONFIG = ROOT / "config" / "cartesian_emotes.yaml"
sys.path.insert(0, str(SCRIPTS))

from volt_emote_engine import (  # noqa: E402
    BUILTIN_EMOTES,
    CartesianEmoteEngine,
    EmoteOptions,
    EmoteStateError,
    EmoteValidationError,
    MAX_REPETITIONS,
    MAX_SCALE,
    MAX_SPEED,
    MIN_REPETITIONS,
    MIN_SCALE,
    MIN_SPEED,
    default_emote_config_path,
    load_builtin_catalog,
    load_emote_catalog,
    preflight_emote,
    sample_definition_pose,
    validate_options,
)
from volt_kinematics import (  # noqa: E402
    JOINT_NAMES,
    LEG_ORDER,
    NOMINAL_FEET,
    WALK_POSE,
)


MINIMAL_CATALOG = """\
schema_version: 1
base_body_height_m: 0.200
emotes:
  test_motion:
    description: A valid minimal motion.
    keyframes:
      - duration_s: 0.0
        easing: smootherstep
        body: {}
        feet: {}
      - duration_s: 0.5
        easing: smootherstep
        body:
          roll_deg: 2.0
        feet: {}
      - duration_s: 0.5
        easing: smootherstep
        body: {}
        feet: {}
"""


def load_text(text, preflight=True, require_builtins=False):
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "emotes.yaml"
        path.write_text(text, encoding="utf-8")
        return load_emote_catalog(
            path,
            preflight=preflight,
            require_builtins=require_builtins,
        )


def assert_frame_safe(testcase, frame):
    testcase.assertEqual(tuple(frame.feet), LEG_ORDER)
    values = (
        frame.body.height,
        frame.body.x,
        frame.body.y,
        frame.body.roll,
        frame.body.pitch,
        frame.body.yaw,
        frame.progress,
    ) + tuple(value for point in frame.feet.values() for value in point)
    testcase.assertTrue(all(math.isfinite(value) for value in values))
    joints, diagnostics = frame.solve_ik()
    testcase.assertEqual(len(joints), len(JOINT_NAMES))
    testcase.assertTrue(all(math.isfinite(value) for value in joints))
    testcase.assertEqual(diagnostics["projected_targets"], [])


class ShippedCatalogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog = load_builtin_catalog(CONFIG)

    def test_default_path_points_at_the_shipped_catalog(self):
        self.assertEqual(default_emote_config_path(), CONFIG.resolve())

    def test_complete_requested_builtin_inventory_is_present(self):
        self.assertEqual(set(self.catalog.emotes), set(BUILTIN_EMOTES))
        self.assertEqual(
            set(BUILTIN_EMOTES),
            {
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
            },
        )

    def test_all_definitions_start_and_end_neutral_with_smootherstep(self):
        for name, definition in self.catalog.emotes.items():
            with self.subTest(emote=name):
                self.assertGreater(definition.total_duration, 0.0)
                for endpoint in (definition.keyframes[0], definition.keyframes[-1]):
                    self.assertEqual(endpoint.pose.body.height, 0.0)
                    self.assertEqual(endpoint.pose.body.x, 0.0)
                    self.assertEqual(endpoint.pose.body.y, 0.0)
                    self.assertEqual(endpoint.pose.body.roll, 0.0)
                    self.assertEqual(endpoint.pose.body.pitch, 0.0)
                    self.assertEqual(endpoint.pose.body.yaw, 0.0)
                    self.assertTrue(all(
                        point == (0.0, 0.0, 0.0)
                        for point in endpoint.pose.foot_offsets
                    ))
                self.assertTrue(all(
                    keyframe.easing == "smootherstep"
                    for keyframe in definition.keyframes
                ))

    def test_every_builtin_preflights_at_default_and_maximum_options(self):
        maximum = validate_options(
            MAX_REPETITIONS,
            MAX_SPEED,
            MAX_SCALE,
            MAX_SCALE,
        )
        for name, definition in self.catalog.emotes.items():
            maximum_for_emote = (
                validate_options(
                    MAX_REPETITIONS,
                    MAX_SPEED,
                    MAX_SCALE,
                    1.25,
                )
                if name == "push_ups"
                else maximum
            )
            for label, options in (
                ("default", EmoteOptions()),
                ("maximum", maximum_for_emote),
            ):
                with self.subTest(emote=name, options=label):
                    report = preflight_emote(
                        definition,
                        self.catalog.base_body_height,
                        options,
                    )
                    self.assertEqual(report.emote, name)
                    self.assertGreater(report.samples, 1)
                    self.assertTrue(math.isfinite(report.maximum_absolute_joint))
                    self.assertGreater(report.maximum_absolute_joint, 0.0)

    def test_every_builtin_has_observable_motion_and_completes_neutral(self):
        """Exercise the full shipped inventory, not only selected examples.

        A syntactically valid catalog entry whose keyframes are all neutral is
        indistinguishable from a broken button to an operator.  Sampling every
        segment endpoint also catches catalog/player regressions that affect
        only one of the less frequently used emotes.
        """
        for name, definition in self.catalog.emotes.items():
            with self.subTest(emote=name):
                engine = CartesianEmoteEngine(self.catalog)
                first = engine.start(name, 20.0)
                assert_frame_safe(self, first)

                elapsed = 0.0
                peak_joint_delta = 0.0
                for keyframe in definition.keyframes[1:]:
                    elapsed += keyframe.duration
                    frame = engine.sample(20.0 + elapsed)
                    assert_frame_safe(self, frame)
                    joints, _diagnostics = frame.solve_ik()
                    peak_joint_delta = max(
                        peak_joint_delta,
                        *(abs(actual - neutral) for actual, neutral in zip(
                            joints,
                            WALK_POSE,
                        )),
                    )

                self.assertGreater(
                    peak_joint_delta,
                    math.radians(1.0),
                    "%s is effectively a no-op" % name,
                )
                self.assertEqual(frame.state, "complete")
                self.assertEqual(frame.body.height, self.catalog.base_body_height)
                self.assertEqual(dict(frame.feet), dict(NOMINAL_FEET))

    def test_yaml_degrees_are_converted_to_runtime_radians(self):
        definition = self.catalog.emotes["nod"]
        pose = sample_definition_pose(definition, 0.55)
        self.assertAlmostEqual(pose.body.pitch, math.radians(5.0), places=12)

    def test_smootherstep_segment_has_expected_quintic_blend(self):
        definition = self.catalog.emotes["push_ups"]
        pose = sample_definition_pose(definition, 0.20)
        blend = 0.25 ** 3 * (0.25 * (0.25 * 6.0 - 15.0) + 10.0)
        self.assertAlmostEqual(pose.body.height, -0.020 * blend, places=12)

    def test_push_up_default_and_gui_maximum_vertical_travel_preflight(self):
        definition = self.catalog.emotes["push_ups"]
        default_pose = sample_definition_pose(definition, 0.8)
        maximum_options = validate_options(depth=1.25)
        maximum_pose = sample_definition_pose(
            definition,
            0.8,
            maximum_options,
        )

        self.assertAlmostEqual(default_pose.body.height, -0.020, places=12)
        self.assertAlmostEqual(maximum_pose.body.height, -0.025, places=12)
        report = preflight_emote(
            definition,
            self.catalog.base_body_height,
            maximum_options,
        )
        self.assertGreater(report.samples, 2)

    def test_wave_targets_are_relative_to_canonical_nominal_feet(self):
        engine = CartesianEmoteEngine(self.catalog)
        engine.start("wave_left", 10.0)
        frame = engine.sample(11.30)
        expected_offset = (0.008, -0.004, 0.038)
        for actual, nominal, offset in zip(
            frame.feet["front_left"],
            NOMINAL_FEET["front_left"],
            expected_offset,
        ):
            self.assertAlmostEqual(actual, nominal + offset, places=12)
        for leg_name in ("front_right", "rear_left", "rear_right"):
            self.assertEqual(frame.feet[leg_name], NOMINAL_FEET[leg_name])
        assert_frame_safe(self, frame)

    def test_heart_traces_mirrored_halves_with_three_leg_support(self):
        definition = self.catalog.emotes["heart"]
        left_index = LEG_ORDER.index("front_left")
        right_index = LEG_ORDER.index("front_right")
        left = definition.keyframes[2].pose.foot_offsets[left_index]
        right = definition.keyframes[7].pose.foot_offsets[right_index]
        self.assertGreater(left[2], 0.0)
        self.assertGreater(right[2], 0.0)
        self.assertAlmostEqual(left[0], right[0], places=12)
        self.assertAlmostEqual(left[1], -right[1], places=12)
        self.assertAlmostEqual(left[2], right[2], places=12)
        for keyframe in definition.keyframes:
            front_lifts = sum(
                keyframe.pose.foot_offsets[index][2] > 1e-12
                for index in (left_index, right_index)
            )
            self.assertLessEqual(front_lifts, 1)

    def test_runtime_scaling_separates_depth_from_amplitude(self):
        push = self.catalog.emotes["push_ups"]
        deep = sample_definition_pose(
            push,
            0.8,
            validate_options(depth=1.25),
        )
        self.assertAlmostEqual(deep.body.height, -0.025, places=12)

        roll = self.catalog.emotes["body_roll"]
        large = sample_definition_pose(
            roll,
            0.55,
            validate_options(amplitude=1.5, depth=0.5),
        )
        self.assertAlmostEqual(large.body.roll, math.radians(6.75), places=12)
        self.assertEqual(large.body.height, 0.0)


class RuntimeEngineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog = load_builtin_catalog(CONFIG)

    def test_start_sample_complete_is_caller_clocked(self):
        engine = CartesianEmoteEngine(self.catalog)
        first = engine.start("push_ups", 100.0)
        self.assertEqual(first.state, "running")
        self.assertEqual(first.progress, 0.0)
        self.assertTrue(engine.active)
        self.assertEqual(first.body.height, self.catalog.base_body_height)

        middle = engine.sample(100.8)
        self.assertAlmostEqual(middle.body.height, 0.180, places=12)
        assert_frame_safe(self, middle)

        final = engine.sample(101.6)
        self.assertEqual(final.state, "complete")
        self.assertFalse(engine.active)
        self.assertEqual(final.progress, 1.0)
        self.assertEqual(final.body.height, self.catalog.base_body_height)
        self.assertEqual(dict(final.feet), dict(NOMINAL_FEET))

    def test_speed_and_repetition_control_wall_duration(self):
        engine = CartesianEmoteEngine(self.catalog)
        engine.start("push_ups", 0.0, repetitions=2, speed=2.0)
        # One 1.6-second definition takes 0.8 wall seconds at 2x. The repetition
        # boundary is neutral but playback is still active.
        boundary = engine.sample(0.8)
        self.assertEqual(boundary.state, "running")
        self.assertAlmostEqual(boundary.progress, 0.5, places=12)
        self.assertEqual(boundary.body.height, self.catalog.base_body_height)
        self.assertEqual(engine.sample(1.6).state, "complete")

    def test_active_request_is_exclusive(self):
        engine = CartesianEmoteEngine(self.catalog)
        engine.start("nod", 0.0)
        with self.assertRaisesRegex(EmoteStateError, "already active"):
            engine.start("bow", 0.1)

    def test_unknown_emote_and_backwards_clock_fail_closed(self):
        engine = CartesianEmoteEngine(self.catalog)
        with self.assertRaises(EmoteValidationError):
            engine.start("missing", 0.0)
        engine.start("nod", 2.0)
        engine.sample(2.5)
        with self.assertRaisesRegex(EmoteStateError, "backwards"):
            engine.sample(2.4)

    def test_cancel_returns_smoothly_from_current_target(self):
        engine = CartesianEmoteEngine(self.catalog, return_duration_s=1.0)
        engine.start("push_ups", 10.0)
        before = engine.sample(10.8)
        self.assertAlmostEqual(before.body.height, 0.180, places=12)
        self.assertTrue(engine.cancel(10.8))
        self.assertEqual(engine.state, "returning")
        self.assertTrue(engine.cancel(10.8))

        start_return = engine.sample(10.8)
        self.assertEqual(start_return.state, "returning")
        self.assertAlmostEqual(start_return.body.height, before.body.height, places=12)
        halfway = engine.sample(11.3)
        self.assertAlmostEqual(halfway.body.height, 0.190, places=12)
        assert_frame_safe(self, halfway)
        complete = engine.sample(11.8)
        self.assertEqual(complete.state, "complete")
        self.assertFalse(engine.active)
        self.assertEqual(complete.body.height, self.catalog.base_body_height)

    def test_cancel_is_false_while_idle_or_complete(self):
        engine = CartesianEmoteEngine(self.catalog)
        self.assertFalse(engine.cancel(0.0))
        engine.start("push_ups", 1.0)
        engine.sample(3.0)
        self.assertFalse(engine.cancel(3.0))

    def test_cancel_paths_for_every_maximum_scaled_builtin_are_safe(self):
        for name, definition in self.catalog.emotes.items():
            with self.subTest(emote=name):
                engine = CartesianEmoteEngine(self.catalog, return_duration_s=0.8)
                engine.start(
                    name,
                    0.0,
                    repetitions=MAX_REPETITIONS,
                    speed=MAX_SPEED,
                    amplitude=MAX_SCALE,
                    depth=(1.25 if name == "push_ups" else MAX_SCALE),
                )
                cancel_time = definition.total_duration * 0.37 / MAX_SPEED
                active = engine.sample(cancel_time)
                assert_frame_safe(self, active)
                self.assertTrue(engine.cancel(cancel_time))
                for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
                    frame = engine.sample(cancel_time + 0.8 * fraction)
                    assert_frame_safe(self, frame)
                self.assertEqual(engine.state, "complete")

    def test_reset_only_operates_when_inactive(self):
        engine = CartesianEmoteEngine(self.catalog)
        engine.start("shake_no", 0.0)
        with self.assertRaisesRegex(EmoteStateError, "cancel"):
            engine.reset()
        engine.sample(10.0)
        engine.reset()
        self.assertEqual(engine.state, "idle")
        self.assertEqual(engine.current_emote, "")
        self.assertFalse(engine.active)

    def test_status_is_serialization_friendly_and_reports_options(self):
        engine = CartesianEmoteEngine(self.catalog)
        engine.start(
            "happy_dance",
            5.0,
            repetitions=3,
            speed=1.5,
            amplitude=1.25,
            depth=0.75,
        )
        self.assertEqual(
            engine.status(),
            {
                "state": "running",
                "active": True,
                "emote": "happy_dance",
                "repetitions": 3,
                "speed": 1.5,
                "amplitude": 1.25,
                "depth": 0.75,
            },
        )

    def test_engine_source_has_no_ros_or_blocking_sleep_dependency(self):
        source = (SCRIPTS / "volt_emote_engine.py").read_text(encoding="utf-8")
        self.assertNotIn("import rclpy", source)
        self.assertNotIn("time.sleep", source)


class RuntimeOptionValidationTests(unittest.TestCase):
    def test_option_boundaries_are_accepted(self):
        low = validate_options(
            MIN_REPETITIONS,
            MIN_SPEED,
            MIN_SCALE,
            MIN_SCALE,
        )
        high = validate_options(
            MAX_REPETITIONS,
            MAX_SPEED,
            MAX_SCALE,
            MAX_SCALE,
        )
        self.assertEqual(low.repetitions, MIN_REPETITIONS)
        self.assertEqual(high.repetitions, MAX_REPETITIONS)

    def test_bad_repetitions_are_rejected(self):
        for value in (True, 1.0, 0, 6, "2"):
            with self.subTest(value=value):
                with self.assertRaises(EmoteValidationError):
                    validate_options(repetitions=value)

    def test_bad_finite_ranges_are_rejected(self):
        cases = (
            {"speed": 0.49},
            {"speed": 2.01},
            {"speed": float("nan")},
            {"amplitude": 0.49},
            {"amplitude": 2.01},
            {"amplitude": float("inf")},
            # depth has its own, wider ceiling (MAX_DEPTH_SCALE) because it
            # only scales vertical travel with all four feet planted.
            {"depth": 0.49},
            {"depth": 3.01},
            {"depth": False},
        )
        for options in cases:
            with self.subTest(options=options):
                with self.assertRaises(EmoteValidationError):
                    validate_options(**options)

    def test_manually_constructed_unsafe_options_are_revalidated(self):
        catalog = load_builtin_catalog(CONFIG)
        unsafe = EmoteOptions(amplitude=99.0)
        with self.assertRaises(EmoteValidationError):
            preflight_emote(catalog.emotes["bow"], catalog.base_body_height, unsafe)


class StrictYamlValidationTests(unittest.TestCase):
    def test_minimal_catalog_loads_and_preflights(self):
        catalog = load_text(MINIMAL_CATALOG)
        self.assertEqual(tuple(catalog.emotes), ("test_motion",))

    def test_duplicate_yaml_key_is_rejected(self):
        duplicate = MINIMAL_CATALOG.replace(
            "    description: A valid minimal motion.\n",
            "    description: first\n    description: second\n",
        )
        with self.assertRaisesRegex(EmoteValidationError, "duplicate"):
            load_text(duplicate)

    def test_unknown_root_emote_keyframe_body_and_leg_keys_are_rejected(self):
        cases = (
            MINIMAL_CATALOG.replace("emotes:\n", "unknown: 1\nemotes:\n"),
            MINIMAL_CATALOG.replace(
                "    description: A valid minimal motion.\n",
                "    description: A valid minimal motion.\n    unknown: true\n",
            ),
            MINIMAL_CATALOG.replace(
                "        easing: smootherstep\n        body:\n          roll_deg: 2.0",
                "        easing: smootherstep\n        unknown: true\n        body:\n          roll_deg: 2.0",
            ),
            MINIMAL_CATALOG.replace("          roll_deg: 2.0", "          bad_angle: 2.0"),
            MINIMAL_CATALOG.replace(
                "          roll_deg: 2.0\n        feet: {}",
                "          roll_deg: 2.0\n        feet:\n          middle_left: [0, 0, 0]",
            ),
        )
        for index, text in enumerate(cases):
            with self.subTest(case=index):
                with self.assertRaisesRegex(EmoteValidationError, "unknown"):
                    load_text(text)

    def test_non_finite_yaml_values_are_rejected(self):
        for token in (".nan", ".inf", "-.inf"):
            text = MINIMAL_CATALOG.replace("roll_deg: 2.0", "roll_deg: %s" % token)
            with self.subTest(token=token):
                with self.assertRaisesRegex(EmoteValidationError, "finite"):
                    load_text(text)

    def test_bad_foot_vector_and_unsafe_body_bound_are_rejected(self):
        bad_foot = MINIMAL_CATALOG.replace(
            "        feet: {}\n      - duration_s: 0.5",
            "        feet:\n          front_left: [0.0, 0.0]\n      - duration_s: 0.5",
            1,
        )
        with self.assertRaisesRegex(EmoteValidationError, "three"):
            load_text(bad_foot)

        unsafe = MINIMAL_CATALOG.replace("roll_deg: 2.0", "x_m: 0.1")
        with self.assertRaisesRegex(EmoteValidationError, "outside"):
            load_text(unsafe)

    def test_schema_duration_easing_and_neutral_endpoint_contracts(self):
        cases = (
            MINIMAL_CATALOG.replace("schema_version: 1", "schema_version: 1.0"),
            MINIMAL_CATALOG.replace("duration_s: 0.0", "duration_s: 0.1", 1),
            MINIMAL_CATALOG.replace("duration_s: 0.5", "duration_s: 0.0", 1),
            MINIMAL_CATALOG.replace("easing: smootherstep", "easing: linear", 1),
            MINIMAL_CATALOG.rsplit("body: {}", 1)[0]
            + "body:\n          yaw_deg: 1.0\n        feet: {}\n",
        )
        for index, text in enumerate(cases):
            with self.subTest(case=index):
                with self.assertRaises(EmoteValidationError):
                    load_text(text)

    def test_preflight_rejects_a_bounded_but_unreachable_target(self):
        text = MINIMAL_CATALOG.replace(
            "        body:\n          roll_deg: 2.0\n        feet: {}",
            "        body:\n          height_offset_m: 0.020\n"
            "        feet:\n"
            "          front_left: [0.0, 0.0, -0.075]\n"
            "          front_right: [0.0, 0.0, -0.075]\n"
            "          rear_left: [0.0, 0.0, -0.075]\n"
            "          rear_right: [0.0, 0.0, -0.075]",
        )
        with self.assertRaisesRegex(EmoteValidationError, "IK projection"):
            load_text(text, preflight=True)

    def test_require_builtins_rejects_an_incomplete_catalog(self):
        with self.assertRaisesRegex(EmoteValidationError, "missing built-ins"):
            load_text(MINIMAL_CATALOG, require_builtins=True)


if __name__ == "__main__":
    unittest.main()
