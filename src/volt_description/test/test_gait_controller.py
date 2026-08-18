#!/usr/bin/env python3

import math
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from volt_gait_controller import (  # noqa: E402
    GAIT_ALIASES,
    GAITS,
    SPOT_WALK_LEG_ORDER,
    TROT_PHASE_OFFSETS,
    VoltGaitController,
    canonical_gait_name,
    limit_velocity_command,
    load_spot_walk_config,
)
from volt_kinematics import (  # noqa: E402
    JOINT_NAMES,
    JOINT_VELOCITY_LIMITS,
    feet_to_joint_positions,
)
from volt_motion_controller import (  # noqa: E402
    SIMULATION_JOINT_VELOCITY_LIMIT,
    VoltMotionController,
)
from volt_serial_protocol import format_frame_command  # noqa: E402
from volt_servo_calibration import (  # noqa: E402
    ServoCalibrationTable,
    named_positions_from_ordered,
)


TROT_NAMES = ("slow_trot", "normal_trot", "fast_trot")
DIAGONAL_PAIRS = {
    frozenset(("front_left", "rear_right")),
    frozenset(("front_right", "rear_left")),
}


class GaitConfigurationTests(unittest.TestCase):
    def test_legacy_walk_and_slow_trot_are_distinct_safe_modes(self):
        self.assertEqual(GAITS["legacy_walk"]["type"], "legacy")
        self.assertEqual(GAITS["slow_trot"]["type"], "phase_trot")
        self.assertIsNot(GAITS["legacy_walk"], GAITS["slow_trot"])

    def test_walk_alias_resolves_explicitly_to_video_walk(self):
        self.assertEqual(GAIT_ALIASES["walk"], "spotmicro_video_walk")
        self.assertEqual(
            canonical_gait_name("walk"),
            "spotmicro_video_walk",
        )
        controller = VoltGaitController()
        controller.set_gait("walk", 0.0)
        self.assertEqual(controller.gait_name, "spotmicro_video_walk")

    def test_trot_modes_increase_speed_and_cadence(self):
        speeds = [GAITS[name]["max_x"] for name in TROT_NAMES]
        cadences = [GAITS[name]["gait_frequency"] for name in TROT_NAMES]
        self.assertEqual(speeds, sorted(speeds))
        self.assertEqual(cadences, sorted(cadences))
        self.assertEqual(len(set(speeds)), len(speeds))
        self.assertEqual(len(set(cadences)), len(cadences))

        # These floors prevent a future tune from reintroducing the visibly
        # slow 0.87/0.69/0.61 second cycles used by the original profiles.
        self.assertGreaterEqual(speeds[0], 0.090)
        self.assertGreaterEqual(speeds[1], 0.130)
        self.assertGreaterEqual(speeds[2], 0.160)
        self.assertGreaterEqual(cadences[0], 1.45)
        self.assertGreaterEqual(cadences[1], 1.80)
        self.assertGreaterEqual(cadences[2], 2.10)

    def test_trot_cadence_remains_natural_at_partial_command(self):
        for name in TROT_NAMES:
            controller = VoltGaitController()
            controller.set_gait(name, 0.0)
            config = GAITS[name]
            half_forward = (0.5 * config["max_x"], 0.0, 0.0)
            for _ in range(100):
                controller.advance_trot_phase(0.01, half_forward)
            if config.get("type") == "physical_trot":
                # Cartesian fast trot owns a backend-specific period. Command
                # magnitude changes stride; it does not silently speed up the
                # hardware cadence.
                expected = 1.0 / controller.fast_trot_active_cycle_period()
                self.assertAlmostEqual(
                    controller.current_gait_frequency,
                    expected,
                )
                continue
            expected = 0.5 * (
                config["min_gait_frequency"] + config["gait_frequency"]
            )
            self.assertAlmostEqual(controller.current_gait_frequency, expected)
            self.assertLess(1.0 / expected, 0.82, name)

    def test_trot_has_diagonal_pairs_and_four_foot_overlap(self):
        self.assertEqual(TROT_PHASE_OFFSETS["front_left"], 0.0)
        self.assertEqual(TROT_PHASE_OFFSETS["rear_right"], 0.0)
        self.assertEqual(TROT_PHASE_OFFSETS["front_right"], 0.5)
        self.assertEqual(TROT_PHASE_OFFSETS["rear_left"], 0.5)
        for name in TROT_NAMES:
            config = GAITS[name]
            self.assertGreater(config["swing_ratio"], 0.0)
            self.assertLess(config["swing_ratio"], 0.5)
            self.assertAlmostEqual(
                config["swing_ratio"] + config["stance_ratio"],
                1.0,
            )

    def test_combined_velocity_is_ellipsoid_limited(self):
        limits = (0.12, 0.05, 0.75)
        limited = limit_velocity_command(limits, limits)
        demand = math.sqrt(
            sum((value / limit) ** 2 for value, limit in zip(limited, limits))
        )
        self.assertAlmostEqual(demand, 1.0)
        ratios = [value / limit for value, limit in zip(limited, limits)]
        self.assertAlmostEqual(ratios[0], ratios[1])
        self.assertAlmostEqual(ratios[1], ratios[2])

    def test_spot_walk_rejects_unsafe_configuration_ranges(self):
        config_path = ROOT / "config" / "gait_controller.yaml"
        base = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        cases = {
            "negative velocity deadband": {
                "velocity_deadband": -0.001,
            },
            "velocity deadband suppresses every command": {
                "velocity_deadband": 1.0,
            },
            "support margin cannot fit stance triangle": {
                "support_margin": 1.0,
            },
            "touchdown lead beyond stance interval": {
                "touchdown_lead": 1.01,
            },
            "negative roll bound": {
                "maximum_body_roll": -0.01,
            },
        }
        for label, overrides in cases.items():
            with self.subTest(case=label), tempfile.TemporaryDirectory() as directory:
                raw = yaml.safe_load(yaml.safe_dump(base))
                raw["volt_motion_controller"]["ros__parameters"][
                    "spot_walk"
                ].update(overrides)
                path = Path(directory) / "gait.yaml"
                path.write_text(yaml.safe_dump(raw), encoding="utf-8")
                with self.assertRaises(ValueError):
                    load_spot_walk_config(path)

    def test_low_hardware_scaled_spot_command_is_not_lost_to_deadband(self):
        controller = VoltGaitController()
        config = controller.gaits["spot_walk"]
        ten_percent_hardware_x = (
            config["max_x"]
            * config["hardware_speed_scale"]
            * 0.10
        )

        controller.step(
            0.01,
            0.01,
            (ten_percent_hardware_x, 0.0, 0.0),
        )

        self.assertTrue(controller.active)


class SwingTrajectoryTests(unittest.TestCase):
    def controller(self, gait="normal_trot"):
        controller = VoltGaitController()
        controller.set_gait(gait, 0.0)
        return controller

    def test_swing_arc_has_soft_liftoff_and_touchdown(self):
        controller = self.controller()
        leg = "front_left"
        start = (0.10, 0.10, -0.20)
        end = (0.16, 0.08, -0.20)
        controller.swing_origins[leg] = start
        controller.swing_targets[leg] = end
        controller.swing_heights[leg] = 0.014

        self.assertEqual(controller.phase_trot_swing_step(leg, 0.0), start)
        self.assertEqual(controller.phase_trot_swing_step(leg, 1.0), end)
        midpoint = controller.phase_trot_swing_step(leg, 0.5)
        self.assertAlmostEqual(midpoint[2], start[2] + 0.014)

        epsilon = 1e-4
        start_next = controller.phase_trot_swing_step(leg, epsilon)
        end_previous = controller.phase_trot_swing_step(leg, 1.0 - epsilon)
        liftoff_speed = math.dist(start, start_next) / epsilon
        touchdown_speed = math.dist(end_previous, end) / epsilon
        self.assertLess(liftoff_speed, 1e-5)
        self.assertLess(touchdown_speed, 1e-5)

    def test_phase_trot_only_swings_a_diagonal_pair(self):
        controller = self.controller()
        velocity = (GAITS["normal_trot"]["max_x"], 0.0, 0.0)
        observed_pairs = set()
        observed_all_stance = False
        for index in range(1, 401):
            controller.step(index * 0.005, 0.005, velocity)
            swing = frozenset(controller.debug_snapshot()["swing_legs"])
            if swing:
                self.assertIn(swing, DIAGONAL_PAIRS)
                observed_pairs.add(swing)
            else:
                observed_all_stance = True
        self.assertEqual(observed_pairs, DIAGONAL_PAIRS)
        self.assertTrue(observed_all_stance)

    def test_stance_foot_is_locked_in_world(self):
        controller = self.controller()
        velocity = (0.08, 0.0, 0.20)
        controller.step(0.01, 0.01, velocity)

        for index in range(2, 200):
            before_stance = set(controller.debug_snapshot()["stance_legs"])
            before_world = dict(controller.world_feet)
            controller.step(index * 0.01, 0.01, velocity)
            after_stance = set(controller.debug_snapshot()["stance_legs"])
            common_stance = before_stance & after_stance
            if common_stance:
                leg = next(iter(common_stance))
                self.assertEqual(controller.world_feet[leg], before_world[leg])
                return
        self.fail("No consecutive stance samples were observed")

    def test_cadence_and_clearance_scale_with_speed(self):
        controller = self.controller("fast_trot")
        config = GAITS["fast_trot"]
        self.assertAlmostEqual(
            controller.adaptive_step_height((0.0, 0.0, 0.0)),
            0.60 * controller.fast_trot_tuning["step_height"],
        )
        controller.plan_fast_trot_motion(
            0.0,
            (config["max_x"], 0.0, 0.0),
        )
        controller.plan_fast_trot_motion(
            (
                controller.fast_trot_active_cycle_period()
                / config["startup_cycle_speed_fraction"]
                + config["startup_ramp_time"]
            ),
            (config["max_x"], 0.0, 0.0),
        )
        self.assertAlmostEqual(
            controller.adaptive_step_height((config["max_x"], 0.0, 0.0)),
            config["step_height"],
        )

        for _ in range(100):
            controller.advance_trot_phase(
                0.01,
                (config["max_x"], 0.0, 0.0),
            )
        self.assertAlmostEqual(
            controller.current_gait_frequency,
            config["gait_frequency"],
        )


class MotionLimitTests(unittest.TestCase):
    def test_raw_trot_trajectory_stays_within_joint_velocity_envelope(self):
        dt = 0.01
        for gait_name in TROT_NAMES:
            controller = VoltGaitController()
            controller.set_gait(gait_name, 0.0)
            config = GAITS[gait_name]
            previous = None
            previous_velocity = None
            peak_velocity = [0.0] * len(JOINT_NAMES)
            peak_acceleration = 0.0

            for index in range(1, 401):
                feet, body, _active = controller.step(
                    index * dt,
                    dt,
                    (config["max_x"], 0.0, 0.0),
                )
                joints = feet_to_joint_positions(
                    feet,
                    height=0.200 + body["height"],
                    roll=body["roll"],
                    pitch=body["pitch"],
                )
                if previous is not None:
                    velocity = [
                        (current - old) / dt
                        for current, old in zip(joints, previous)
                    ]
                    peak_velocity = [
                        max(peak, abs(value))
                        for peak, value in zip(peak_velocity, velocity)
                    ]
                    if previous_velocity is not None:
                        acceleration = [
                            (current - old) / dt
                            for current, old in zip(velocity, previous_velocity)
                        ]
                        peak_acceleration = max(
                            peak_acceleration,
                            *(abs(value) for value in acceleration),
                        )
                    previous_velocity = velocity
                previous = joints

            if config.get("type") == "physical_trot":
                # The specified 55 mm / 35 mm / 0.42 s Cartesian simulation
                # target is intentionally more demanding than the canonical
                # joint guard. The next test proves the published commands are
                # rate limited; keep the raw demand finite and regression
                # bounded here so that timing pressure remains visible.
                self.assertLess(max(peak_velocity), 11.0, gait_name)
                self.assertLess(peak_acceleration, 380.0, gait_name)
            else:
                self.assertLess(max(peak_velocity), 5.0, gait_name)
                self.assertLess(peak_acceleration, 140.0, gait_name)

    def test_joint_command_filter_enforces_velocity_and_acceleration(self):
        controller = VoltMotionController.__new__(VoltMotionController)
        controller.max_joint_velocity = 4.0
        controller.max_joint_acceleration = 18.0
        controller.hardware_mode = False
        controller.joint_smoothing_factor = 0.12
        controller.commanded_positions = [0.0] * len(JOINT_NAMES)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        dt = 0.01

        previous_velocity = list(controller.commanded_velocities)
        # Keep the target far away so this measures sustained limiting.
        target = [100.0] * len(JOINT_NAMES)
        for _ in range(200):
            controller.commanded_positions = controller.smooth_joint_target(
                target,
                dt,
            )
            for index, velocity in enumerate(controller.commanded_velocities):
                self.assertLessEqual(
                    abs(velocity),
                    min(
                        4.0,
                        JOINT_VELOCITY_LIMITS[index],
                        SIMULATION_JOINT_VELOCITY_LIMIT,
                    ) + 1e-9,
                )
                self.assertLessEqual(
                    abs(velocity - previous_velocity[index]),
                    18.0 * dt + 1e-9,
                )
            previous_velocity = list(controller.commanded_velocities)

    def test_braking_guard_is_not_reported_as_velocity_ceiling_clamp(self):
        controller = VoltMotionController.__new__(VoltMotionController)
        controller.gait_name = "fast_trot"
        controller.gait_configs = {
            name: dict(config)
            for name, config in GAITS.items()
        }
        controller.gait_configs["fast_trot"]["joint_smoothing_alpha"] = 1.0
        controller.gait_configs["fast_trot"]["joint_acceleration_limit"] = 1.0
        controller.hardware_mode = False
        controller.max_joint_velocity = 4.0
        controller.max_joint_acceleration = 1.0
        controller.joint_smoothing_factor = 1.0
        controller.commanded_positions = [0.0] * len(JOINT_NAMES)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)
        controller.joint_velocity_clamp_counts = [0] * len(JOINT_NAMES)
        controller.joint_braking_clamp_counts = [0] * len(JOINT_NAMES)
        controller.joint_acceleration_clamp_counts = [0] * len(JOINT_NAMES)

        controller.smooth_joint_target(
            [0.005] * len(JOINT_NAMES),
            0.005,
        )

        self.assertEqual(sum(controller.joint_velocity_clamp_counts), 0)
        self.assertGreater(sum(controller.joint_braking_clamp_counts), 0)

    def test_spot_walk_joint_smoothing_overrides_global_default(self):
        controller = VoltMotionController.__new__(VoltMotionController)
        controller.max_joint_velocity = 4.0
        controller.max_joint_acceleration = 1000.0
        controller.hardware_mode = False
        controller.joint_smoothing_alpha = 0.12
        controller.joint_smoothing_factor = 0.12
        controller.gait_configs = {
            name: dict(config)
            for name, config in GAITS.items()
        }
        controller.gait_name = "spot_walk"
        controller.commanded_positions = [0.0] * len(JOINT_NAMES)
        controller.commanded_velocities = [0.0] * len(JOINT_NAMES)

        target = [0.001] * len(JOINT_NAMES)
        actual = controller.smooth_joint_target(target, 0.01)

        expected = (
            target[0]
            * controller.gait_configs["spot_walk"]["joint_smoothing_alpha"]
        )
        self.assertAlmostEqual(actual[0], expected, places=12)

    def test_filtered_trots_fit_simulation_command_guard(self):
        for gait_name in TROT_NAMES:
            gait = VoltGaitController()
            gait.set_gait(gait_name, 0.0)
            config = GAITS[gait_name]
            physical_fast_trot = config.get("type") == "physical_trot"
            dt = 0.005 if physical_fast_trot else 0.01

            motion = VoltMotionController.__new__(VoltMotionController)
            motion.gait_name = gait_name
            motion.gait_controller = gait
            motion.gait_configs = {
                name: dict(gait_config)
                for name, gait_config in GAITS.items()
            }
            motion.max_joint_velocity = 4.0
            motion.max_joint_acceleration = (
                60.0 if physical_fast_trot else 18.0
            )
            motion.hardware_mode = False
            motion.joint_smoothing_factor = 0.12
            motion.commanded_positions = None
            motion.commanded_velocities = None

            previous = None
            previous_velocity = None
            peak_tracking_error = 0.0
            command_lag = 0.0
            sample_count = int(round(8.0 / dt))
            for index in range(1, sample_count + 1):
                if physical_fast_trot:
                    gait.set_support_feedback({
                        "command_error": command_lag,
                        "tracking_available": False,
                        "tracking_required": False,
                    })
                feet, body, _active = gait.step(
                    index * dt,
                    dt,
                    (config["max_x"], 0.0, 0.0),
                )
                target = feet_to_joint_positions(
                    feet,
                    height=0.200 + body["height"],
                    roll=body["roll"],
                    pitch=body["pitch"],
                )
                if motion.commanded_positions is None:
                    motion.commanded_positions = list(target)
                    motion.commanded_velocities = [0.0] * len(JOINT_NAMES)
                else:
                    motion.commanded_positions = motion.smooth_joint_target(
                        target,
                        dt,
                    )
                command_lag = max(
                    abs(requested - commanded)
                    for requested, commanded in zip(
                        target,
                        motion.commanded_positions,
                    )
                )

                if index * dt > 1.0:
                    peak_tracking_error = max(
                        peak_tracking_error,
                        *(
                            abs(requested - commanded)
                            for requested, commanded in zip(
                                target,
                                motion.commanded_positions,
                            )
                        ),
                    )

                if previous is not None and index * dt > 1.0:
                    for joint_index, (current, old) in enumerate(zip(
                        motion.commanded_positions,
                        previous,
                    )):
                        self.assertLessEqual(
                            abs(current - old) / dt,
                            motion.joint_velocity_limit(joint_index) + 1e-9,
                            "%s joint %d" % (gait_name, joint_index),
                        )
                    for joint_index, (current, old) in enumerate(zip(
                        motion.commanded_velocities,
                        previous_velocity,
                    )):
                        self.assertLessEqual(
                            abs(current - old),
                            motion.joint_acceleration_limit(joint_index)
                            * dt
                            + 1e-9,
                            "%s joint %d" % (gait_name, joint_index),
                        )
                previous = list(motion.commanded_positions)
                previous_velocity = list(motion.commanded_velocities)

            # A rate limiter can satisfy velocity while falling arbitrarily
            # behind. Physical fast trot therefore feeds command lag back into
            # the common Cartesian/body phase clock.
            tracking_limit_deg = 28.0 if physical_fast_trot else 13.75
            self.assertLessEqual(
                peak_tracking_error,
                math.radians(tracking_limit_deg),
                gait_name,
            )

    def test_downstream_fast_trot_stride_metric_tracks_continuous_stance(self):
        cases = [("simulation", False, None, 0.005)]
        cases.extend(
            (
                preset_name,
                True,
                dict(GAITS["fast_trot"]["presets"][preset_name]),
                0.01,
            )
            for preset_name in ("bench", "floor_test", "wide")
        )
        maximum = dict(
            GAITS["fast_trot"]["presets"]["wide"],
            stride_scale=GAITS["fast_trot"][
                "maximum_safe_stride_scale"
            ],
            hardware_speed_scale=0.75,
        )
        cases.extend((
            ("hardware_max", True, maximum, 0.01),
            ("simulation_max", False, maximum, 0.005),
        ))

        for label, hardware, tuning, dt in cases:
            gait = VoltGaitController(hardware_mode=hardware)
            if tuning is not None:
                gait.set_fast_trot_tuning(tuning)
            gait.set_gait("fast_trot", 0.0)

            motion = VoltMotionController.__new__(VoltMotionController)
            motion.gait_controller = gait
            motion.gait_name = "fast_trot"
            motion.gait_configs = {
                name: dict(config)
                for name, config in GAITS.items()
            }
            motion.hardware_mode = hardware
            motion.max_joint_velocity = 4.0
            motion.max_joint_acceleration = 60.0
            motion.joint_smoothing_factor = 0.12
            motion.commanded_positions = None
            motion.commanded_velocities = None
            motion.reset_fast_trot_cycle_diagnostics()
            velocity = (gait.fast_trot_command_limits()[0], 0.0, 0.0)
            command_lag = 0.0

            for index in range(1, int(round(14.0 / dt)) + 1):
                now = index * dt
                gait.set_support_feedback({
                    "command_error": command_lag,
                    "tracking_available": False,
                    "tracking_required": False,
                })
                feet, body, _active = gait.step(now, dt, velocity)
                motion.last_gait_body_transform = {
                    "height": 0.200 + body["height"],
                    "body_x": 0.0,
                    "body_y": 0.0,
                    "roll": body["roll"],
                    "pitch": body["pitch"],
                    "yaw": 0.0,
                }
                raw = feet_to_joint_positions(
                    feet,
                    **motion.last_gait_body_transform,
                )
                if motion.commanded_positions is None:
                    motion.commanded_positions = list(raw)
                    motion.commanded_velocities = [0.0] * len(JOINT_NAMES)
                else:
                    motion.commanded_positions = (
                        motion.smooth_joint_target(raw, dt)
                    )
                command_lag = max(
                    abs(requested - commanded)
                    for requested, commanded in zip(
                        raw,
                        motion.commanded_positions,
                    )
                )
                motion.update_fast_trot_cycle_diagnostics(
                    raw,
                    motion.commanded_positions,
                )

            requested = motion.fast_trot_completed_requested_stride
            achieved = motion.fast_trot_achieved_stride
            ratio = achieved / requested
            message = (
                "%s requested=%.3f mm achieved=%.3f mm ratio=%.1f%%"
                % (label, requested * 1000.0, achieved * 1000.0, ratio * 100.0)
            )
            self.assertGreaterEqual(
                motion.fast_trot_completed_cycles,
                3,
                message,
            )
            self.assertGreaterEqual(ratio, 0.80, message)
            self.assertLessEqual(ratio, 1.05, message)
            self.assertTrue(motion.fast_trot_stance_grounded, message)
            self.assertLessEqual(
                motion.fast_trot_max_stance_ground_error,
                GAITS["fast_trot"]["stance_ground_tolerance"],
                message,
            )
            self.assertGreaterEqual(
                motion.fast_trot_achieved_step_height,
                0.85 * gait.fast_trot_tuning["step_height"],
                message,
            )
            self.assertGreaterEqual(
                gait.fast_trot_observed_cycle_period,
                gait.fast_trot_active_cycle_period(),
                message,
            )


class HardwareFrameCoverageTests(unittest.TestCase):
    def test_all_gui_gaits_fit_calibration_and_nano_frame(self):
        table = ServoCalibrationTable.from_file(
            ROOT / "config" / "servo_calibration.yaml"
        )
        for gait_name in (
            "spotmicro_video_walk",
            "spot_walk",
            "legacy_walk",
            "amble",
            "slow_trot",
            "normal_trot",
            "fast_trot",
        ):
            config = GAITS[gait_name]
            commands = (
                (config["max_x"], 0.0, 0.0),
                (-config["max_x"], 0.0, 0.0),
                (0.0, config["max_y"], 0.0),
                (0.0, 0.0, config["max_yaw"]),
                (
                    0.55 * config["max_x"],
                    0.45 * config["max_y"],
                    0.45 * config["max_yaw"],
                ),
            )
            for velocity in commands:
                with self.subTest(gait=gait_name, velocity=velocity):
                    gait = VoltGaitController()
                    gait.set_gait(gait_name, 0.0)
                    for index in range(1, 201):
                        feet, body, _active = gait.step(
                            index * 0.01,
                            0.01,
                            velocity,
                        )
                        if isinstance(body, dict):
                            joints = feet_to_joint_positions(
                                feet,
                                height=0.200 + body.get("height", 0.0),
                                roll=body.get("roll", 0.0),
                                pitch=body.get("pitch", 0.0),
                            )
                        else:
                            joints = feet_to_joint_positions(
                                feet,
                                body_x=body[0],
                                body_y=body[1],
                            )
                        frame, details = table.channel_frame_from_positions(
                            named_positions_from_ordered(joints)
                        )
                        self.assertFalse(
                            any(item["clamped"] for item in details)
                        )
                        self.assertLessEqual(
                            len((format_frame_command(frame) + "\n").encode("ascii")),
                            63,
                        )


if __name__ == "__main__":
    unittest.main()
