#!/usr/bin/env python3

"""Behavioural tests for the deliberate SpotMicro reference-video crawl."""

import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from volt_gait_controller import (  # noqa: E402
    GAIT_ALIASES,
    GAITS,
    VoltGaitController,
    canonical_gait_name,
    support_clearance,
)
from volt_kinematics import (  # noqa: E402
    JOINT_NAMES,
    LEG_ORDER,
    NOMINAL_FEET,
    feet_to_joint_positions_diagnostic,
)


GAIT_NAME = "spotmicro_video_walk"
DT = 0.005
EXPECTED_LEG_ORDER = (
    "rear_right",
    "front_right",
    "rear_left",
    "front_left",
)
EXPECTED_PHASES = (
    "shift_front_left",
    "swing_rear_right",
    "shift_back_left",
    "swing_front_right",
    "shift_front_right",
    "swing_rear_left",
    "shift_back_right",
    "swing_front_left",
)
SHIFT_PHASES = EXPECTED_PHASES[0::2]
SWING_PHASES = EXPECTED_PHASES[1::2]


def finite_feet(feet):
    return all(
        math.isfinite(value)
        for leg in LEG_ORDER
        for value in feet[leg]
    )


class VideoWalkHarness:
    def make_controller(self):
        controller = VoltGaitController()
        controller.set_gait(GAIT_NAME, 0.0)
        controller._video_test_time = 0.0
        return controller

    def samples(
        self,
        controller,
        duration,
        velocity=None,
        step_in_place=False,
        body_offset=(0.0, 0.0),
    ):
        config = controller.gaits[GAIT_NAME]
        if velocity is None:
            velocity = (0.35 * config["max_x"], 0.0, 0.0)
        count = int(math.ceil(duration / DT))
        start = controller._video_test_time
        result = []
        for index in range(1, count + 1):
            now = start + index * DT
            feet, body, active = controller.step(
                now,
                DT,
                velocity,
                step_in_place,
                body_offset,
            )
            result.append(
                (now, feet, body, active, controller.debug_snapshot())
            )
        controller._video_test_time = start + count * DT
        return result

    def run_until(
        self,
        controller,
        predicate,
        timeout=None,
        velocity=None,
    ):
        if timeout is None:
            timeout = 1.5 * controller.gaits[GAIT_NAME][
                "full_cycle_duration"
            ]
        for sample in self.samples(controller, timeout, velocity):
            if predicate(sample[4]):
                return sample
        self.fail("Timed out waiting for requested video-walk state")


class VideoWalkConfigurationTests(unittest.TestCase):
    def test_canonical_name_and_explicit_walk_alias(self):
        self.assertIn(GAIT_NAME, GAITS)
        self.assertEqual(GAIT_ALIASES["walk"], GAIT_NAME)
        self.assertEqual(canonical_gait_name(" walk "), GAIT_NAME)
        self.assertEqual(GAITS[GAIT_NAME]["type"], "stable_crawl")
        self.assertEqual(
            tuple(GAITS[GAIT_NAME]["leg_sequence"]),
            EXPECTED_LEG_ORDER,
        )
        self.assertIsNot(GAITS[GAIT_NAME], GAITS["slow_trot"])

    def test_default_timing_and_support_tuning_are_conservative(self):
        config = GAITS[GAIT_NAME]
        self.assertAlmostEqual(config["full_cycle_duration"], 5.20)
        self.assertAlmostEqual(config["shift_duration"], 0.38)
        self.assertAlmostEqual(config["support_verify_duration"], 0.08)
        self.assertAlmostEqual(config["swing_duration"], 0.68)
        self.assertAlmostEqual(config["settle_duration"], 0.16)
        self.assertAlmostEqual(config["forward_body_shift"], 0.020)
        self.assertAlmostEqual(config["rearward_body_shift"], 0.010)
        self.assertAlmostEqual(config["lateral_body_shift"], 0.018)
        self.assertAlmostEqual(config["support_margin"], 0.012)
        self.assertAlmostEqual(config["hardware_speed_scale"], 0.20)
        self.assertGreater(config["hardware_time_scale"], 1.0)
        self.assertLessEqual(config["body_bob_amplitude"], 0.002)

    def test_gui_uses_canonical_name_and_volt_walk_label(self):
        gui_source = (
            ROOT / "scripts" / "volt_control_gui.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"spotmicro_video_walk"', gui_source)
        self.assertIn('"spotmicro_video_walk": "VOLT WALK"', gui_source)
        self.assertNotIn('"SPOTMICRO VIDEO WALK"', gui_source)

    def test_hardware_time_scale_makes_the_cycle_twenty_percent_slower(self):
        controller = VoltGaitController(hardware_mode=True)
        controller.set_gait(GAIT_NAME, 0.0)
        config = controller.config
        self.assertAlmostEqual(controller.gait_time_scale(), 1.20)
        self.assertAlmostEqual(
            config["full_cycle_duration"] * controller.gait_time_scale(),
            6.24,
        )


class VideoWalkPhaseTests(VideoWalkHarness, unittest.TestCase):
    def test_exact_eight_phase_sequence_and_stance_counts(self):
        controller = self.make_controller()
        config = controller.gaits[GAIT_NAME]
        samples = self.samples(
            controller,
            1.10 * config["full_cycle_duration"],
        )
        observed = []
        previous = None
        observed_phase_indices = set()
        for _now, _feet, _body, _active, debug in samples:
            phase_name = debug["phase_name"]
            if phase_name in EXPECTED_PHASES:
                observed_phase_indices.add(debug["phase_index"])
                if phase_name != previous:
                    observed.append(phase_name)
                    previous = phase_name
            if phase_name in SHIFT_PHASES:
                self.assertEqual(debug["swing_legs"], [])
                self.assertEqual(set(debug["stance_legs"]), set(LEG_ORDER))
            if phase_name in SWING_PHASES and debug["swing_legs"]:
                self.assertEqual(len(debug["swing_legs"]), 1)
                self.assertEqual(len(debug["stance_legs"]), 3)
                self.assertTrue(
                    set(debug["swing_legs"]).isdisjoint(
                        debug["stance_legs"]
                    )
                )
        self.assertEqual(observed[:8], list(EXPECTED_PHASES))
        self.assertEqual(observed_phase_indices, set(range(8)))

    def test_support_verification_gates_every_liftoff(self):
        controller = self.make_controller()
        config = controller.gaits[GAIT_NAME]
        samples = self.samples(
            controller,
            config["full_cycle_duration"],
        )
        airborne_samples = 0
        for _now, _feet, _body, _active, debug in samples:
            required = {
                "support_polygon_valid",
                "support_target_x",
                "support_target_y",
                "support_margin",
                "shift_completion",
                "lift_allowed",
            }
            self.assertTrue(required.issubset(debug))
            if debug["swing_legs"]:
                airborne_samples += 1
                self.assertTrue(debug["support_polygon_valid"])
                self.assertTrue(debug["lift_allowed"])
                self.assertGreaterEqual(
                    debug["shift_completion"] + 1e-9,
                    config["shift_completion_threshold"],
                )
            elif debug.get("step_state") in ("SHIFT", "VERIFY_SUPPORT"):
                self.assertEqual(debug["swing_legs"], [])
        self.assertGreater(airborne_samples, 0)

    def test_body_target_is_inside_support_triangle_and_held_during_swing(self):
        controller = self.make_controller()
        config = controller.gaits[GAIT_NAME]
        samples = self.samples(
            controller,
            config["full_cycle_duration"],
        )
        checked = 0
        for _now, feet, _body, _active, debug in samples:
            if not debug["swing_legs"]:
                continue
            triangle = [
                feet[leg][:2]
                for leg in debug["stance_legs"]
            ]
            target = (
                debug["support_target_x"],
                debug["support_target_y"],
            )
            self.assertGreaterEqual(
                support_clearance(target, triangle),
                debug["support_margin"] - 3e-4,
            )
            self.assertAlmostEqual(
                debug["body_shift_x"],
                debug["body_shift_target_x"],
                delta=8e-4,
            )
            self.assertAlmostEqual(
                debug["body_shift_y"],
                debug["body_shift_target_y"],
                delta=8e-4,
            )
            checked += 1
        self.assertGreater(checked, 0)

    def test_degenerate_support_polygon_refuses_lift_and_warns(self):
        controller = self.make_controller()
        degenerate = {
            leg: (0.10, 0.0, NOMINAL_FEET[leg][2])
            for leg in LEG_ORDER
        }
        controller.set_current_feet(degenerate)
        config = controller.gaits[GAIT_NAME]
        samples = self.samples(
            controller,
            config["shift_duration"]
            + config["support_verify_duration"]
            + 0.20,
        )
        self.assertTrue(all(not sample[4]["swing_legs"] for sample in samples))
        last = samples[-1][4]
        self.assertFalse(last["support_polygon_valid"])
        self.assertFalse(last["lift_allowed"])
        self.assertTrue(last["warning"])

    def test_three_stance_feet_remain_world_locked_during_swing(self):
        controller = self.make_controller()
        self.run_until(
            controller,
            lambda debug: (
                debug["swing_legs"] == ["rear_right"]
                and debug["phase_progress"] > 0.15
            ),
        )
        stance_legs = [
            leg for leg in LEG_ORDER if leg != "rear_right"
        ]
        locked = {
            leg: controller.world_feet[leg]
            for leg in stance_legs
        }
        self.samples(
            controller,
            0.25,
            velocity=(0.35 * controller.config["max_x"], 0.0, 0.0),
        )
        for leg in stance_legs:
            for actual, expected in zip(
                controller.world_feet[leg],
                locked[leg],
            ):
                self.assertAlmostEqual(actual, expected, places=12)


class VideoWalkTrajectoryTests(VideoWalkHarness, unittest.TestCase):
    def test_joint_order_remains_canonical(self):
        self.assertEqual(
            tuple(JOINT_NAMES),
            (
                "front_left_shoulder",
                "front_left_leg",
                "front_left_foot",
                "front_right_shoulder",
                "front_right_leg",
                "front_right_foot",
                "rear_left_shoulder",
                "rear_left_leg",
                "rear_left_foot",
                "rear_right_shoulder",
                "rear_right_leg",
                "rear_right_foot",
            ),
        )

    def test_cartesian_arc_has_finite_soft_endpoints_and_exact_height(self):
        controller = self.make_controller()
        config = controller.gaits[GAIT_NAME]
        leg = "rear_right"
        start = (0.11, -0.10, -0.20)
        target = (0.14, -0.095, -0.20)
        controller.swing_origins[leg] = start
        controller.swing_targets[leg] = target
        controller.swing_heights[leg] = config["step_height"]

        points = [
            controller.phase_trot_swing_step(leg, index / 1000.0)
            for index in range(1001)
        ]
        self.assertTrue(all(math.isfinite(v) for point in points for v in point))
        self.assertEqual(points[0], start)
        self.assertEqual(points[-1], target)
        self.assertAlmostEqual(
            max(point[2] for point in points) - start[2],
            config["step_height"],
            delta=1e-9,
        )

        epsilon = 1e-4
        after_start = controller.phase_trot_swing_step(leg, epsilon)
        before_end = controller.phase_trot_swing_step(leg, 1.0 - epsilon)
        start_velocity = [
            (after_start[index] - start[index]) / epsilon
            for index in range(3)
        ]
        end_velocity = [
            (target[index] - before_end[index]) / epsilon
            for index in range(3)
        ]
        self.assertLess(max(abs(value) for value in start_velocity), 1e-4)
        self.assertLess(max(abs(value) for value in end_velocity), 1e-4)

    def test_full_cycle_feet_and_ik_are_finite_canonical_radians(self):
        controller = self.make_controller()
        config = controller.gaits[GAIT_NAME]
        reach_margins = []
        for _now, feet, body, _active, _debug in self.samples(
            controller,
            config["full_cycle_duration"],
        ):
            self.assertEqual(tuple(feet), LEG_ORDER)
            self.assertTrue(finite_feet(feet))
            joints, diagnostics = feet_to_joint_positions_diagnostic(
                feet,
                height=0.200 + body.get("height", 0.0),
                body_x=body.get("body_x_override", 0.0)
                + body.get("x", 0.0),
                body_y=body.get("body_y_override", 0.0)
                + body.get("y", 0.0),
                roll=body.get("roll", 0.0),
                pitch=body.get("pitch", 0.0),
            )
            self.assertEqual(len(joints), len(JOINT_NAMES))
            self.assertEqual(len(joints), 12)
            self.assertTrue(all(math.isfinite(value) for value in joints))
            self.assertIsInstance(diagnostics["projected_targets"], list)
            for diagnostic in diagnostics["legs"].values():
                reach = diagnostic["input_reach"]
                reach_margins.append(min(
                    reach - diagnostic["minimum_reach"],
                    diagnostic["maximum_reach"] - reach,
                ))
            self.assertFalse(diagnostics["non_finite_input"])
            self.assertFalse(diagnostics["non_finite_output"])
            self.assertTrue(
                set(diagnostics["projected_targets"]).issubset(LEG_ORDER)
            )
        self.assertGreaterEqual(min(reach_margins), 0.010)

    def test_gait_source_contains_no_servo_or_pca_conversion(self):
        source = (
            ROOT / "scripts" / "volt_gait_controller.py"
        ).read_text(encoding="utf-8").lower()
        forbidden = (
            "pca9685",
            "servo_calibration",
            "channel_frame",
            "neutral_deg",
            "pulse_min",
            "pulse_max",
        )
        for token in forbidden:
            self.assertNotIn(token, source)


class VideoWalkStopTests(VideoWalkHarness, unittest.TestCase):
    def test_stop_finishes_current_swing_then_centres_with_all_feet_down(self):
        controller = self.make_controller()
        config = controller.gaits[GAIT_NAME]
        sample = self.run_until(
            controller,
            lambda debug: (
                len(debug["swing_legs"]) == 1
                and 0.25 < debug["phase_progress"] < 0.55
            ),
        )
        airborne_leg = sample[4]["swing_legs"][0]
        controller.request_stop()
        stopped_samples = self.samples(
            controller,
            config["swing_duration"]
            + config["settle_duration"]
            + config["shift_duration"]
            + 0.50,
            velocity=(0.0, 0.0, 0.0),
        )

        observed_swing_legs = {
            debug["swing_legs"][0]
            for _now, _feet, _body, _active, debug in stopped_samples
            if debug["swing_legs"]
        }
        self.assertLessEqual(observed_swing_legs, {airborne_leg})
        self.assertFalse(stopped_samples[-1][3])
        last_feet = stopped_samples[-1][1]
        for leg in LEG_ORDER:
            self.assertAlmostEqual(
                last_feet[leg][2],
                NOMINAL_FEET[leg][2],
                places=8,
            )
        last_debug = stopped_samples[-1][4]
        self.assertEqual(last_debug["swing_legs"], [])
        self.assertEqual(set(last_debug["stance_legs"]), set(LEG_ORDER))
        self.assertAlmostEqual(last_debug["body_shift_x"], 0.0, delta=1e-8)
        self.assertAlmostEqual(last_debug["body_shift_y"], 0.0, delta=1e-8)

    def test_zero_command_is_idle_and_step_in_place_is_explicit(self):
        idle_controller = self.make_controller()
        idle = self.samples(
            idle_controller,
            1.5,
            velocity=(0.0, 0.0, 0.0),
        )
        self.assertFalse(any(sample[3] for sample in idle))
        self.assertTrue(all(not sample[4]["swing_legs"] for sample in idle))

        step_controller = self.make_controller()
        stepping = self.samples(
            step_controller,
            1.5,
            velocity=(0.0, 0.0, 0.0),
            step_in_place=True,
        )
        self.assertTrue(any(sample[4]["swing_legs"] for sample in stepping))


if __name__ == "__main__":
    unittest.main()
