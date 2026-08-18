#!/usr/bin/env python3

"""Deterministic contract tests for VOLT's physical fast-trot trajectory."""

import copy
import math
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
CONFIG_PATH = ROOT / "config" / "gait_controller.yaml"
PHYSICAL_CONFIG_PATH = ROOT / "config" / "physical_fast_trot.yaml"
CALIBRATION_PATH = ROOT / "config" / "servo_calibration.yaml"
sys.path.insert(0, str(SCRIPTS))

from volt_gait_controller import (  # noqa: E402
    FAST_TROT_PARAMETER_NAMES,
    FAST_TROT_PRESETS,
    FAST_TROT_PRESET_PARAMETER_NAMES,
    FAST_TROT_TUNING_BOUNDS,
    GAITS,
    TROT_PHASE_OFFSETS,
    VoltGaitController,
    load_fast_trot_config,
    load_gait_configs,
    validate_fast_trot_tuning,
)
from volt_kinematics import (  # noqa: E402
    JOINT_NAMES,
    LEG_ORDER,
    NOMINAL_FEET,
    feet_to_joint_positions_diagnostic,
)
from volt_servo_calibration import (  # noqa: E402
    ServoCalibrationTable,
    named_positions_from_ordered,
)


PAIR_A = frozenset(("front_left", "rear_right"))
PAIR_B = frozenset(("front_right", "rear_left"))
DIAGONAL_PAIRS = {PAIR_A, PAIR_B}
DT = 0.002


def _raw_gait_yaml():
    with CONFIG_PATH.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _raw_fast_trot(raw):
    return raw["volt_motion_controller"]["ros__parameters"]["fast_trot"]


def _write_yaml(directory, raw):
    path = Path(directory) / "gait_controller.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return path


def _controller(hardware=False, tuning=None):
    controller = VoltGaitController(hardware_mode=hardware)
    if tuning is not None:
        controller.set_fast_trot_tuning(tuning)
    controller.set_gait("fast_trot", 0.0)
    return controller


def _full_forward_command(controller):
    return (controller.fast_trot_command_limits()[0], 0.0, 0.0)


def _measure_forward_stance_sweep(hardware=False, tuning=None):
    """Measure one steady touchdown-to-liftoff body-frame sweep."""
    controller = _controller(hardware=hardware, tuning=tuning)
    velocity = _full_forward_command(controller)
    steady_after = (
        (
            controller.config["trot_ready_time"]
            if hardware
            else 0.0
        )
        + controller.fast_trot_active_cycle_period()
        / controller.config["startup_cycle_speed_fraction"]
        +
        controller.config["startup_ramp_time"]
        + controller.fast_trot_active_cycle_period()
    )
    previous_swing = set()
    touchdown_x = None
    touchdown_time = None

    for index in range(1, int(math.ceil(8.0 / DT)) + 1):
        now = index * DT
        feet, _body, _active = controller.step(now, DT, velocity)
        swing = set(controller.debug_snapshot()["swing_legs"])
        leg = "front_left"

        if now >= steady_after:
            if leg in previous_swing and leg not in swing:
                touchdown_x = feet[leg][0]
                touchdown_time = now
            elif (
                leg not in previous_swing
                and leg in swing
                and touchdown_x is not None
            ):
                liftoff_x = feet[leg][0]
                nominal_x = NOMINAL_FEET[leg][0]
                return {
                    "controller": controller,
                    "touchdown_lead": touchdown_x - nominal_x,
                    "liftoff_trail": liftoff_x - nominal_x,
                    "stance_sweep": touchdown_x - liftoff_x,
                    "stance_duration": now - touchdown_time,
                    "requested_stride": controller.fast_trot_requested_stride,
                    "achieved_stride": controller.fast_trot_achieved_stride,
                }
        previous_swing = swing

    raise AssertionError("no complete steady front-left stance was observed")


def _format_diagnostic_table(rows):
    columns = (
        ("case", 12),
        ("stride_mm", 11),
        ("joint_deg", 11),
        ("ik_proj", 9),
        ("ik_clamp", 10),
        ("servo_clamp", 13),
    )
    header = " ".join(name.rjust(width) for name, width in columns)
    divider = " ".join(("-" * width) for _name, width in columns)
    lines = [header, divider]
    for row in rows:
        values = (
            row["case"],
            "%.2f" % (1000.0 * row["stride"]),
            "%.2f" % row["joint_excursion_deg"],
            str(row["ik_projections"]),
            str(row["ik_clamps"]),
            str(row["servo_clamps"]),
        )
        lines.append(
            " ".join(
                value.rjust(width)
                for value, (_name, width) in zip(values, columns)
            )
        )
    return "\n" + "\n".join(lines)


class FastTrotYamlContractTests(unittest.TestCase):
    def test_hardware_fast_trot_profile_is_a_separate_overlay(self):
        simulation = load_gait_configs(CONFIG_PATH)
        hardware = load_gait_configs(CONFIG_PATH, PHYSICAL_CONFIG_PATH)

        self.assertNotEqual(
            simulation["fast_trot"]["hardware_cycle_period"],
            hardware["fast_trot"]["hardware_cycle_period"],
        )
        self.assertEqual(
            simulation["normal_trot"],
            hardware["normal_trot"],
        )
        self.assertEqual(hardware["fast_trot"]["duty_factor"], 0.62)
        self.assertEqual(hardware["fast_trot"]["body_height"], 0.188)
        self.assertLessEqual(
            1.0 / hardware["fast_trot"]["presets"]["wide"][
                "hardware_cycle_period"
            ],
            1.5,
        )

    def test_yaml_is_the_owner_of_fast_trot_values_and_presets(self):
        raw = _raw_fast_trot(_raw_gait_yaml())
        loaded = load_fast_trot_config(CONFIG_PATH)

        self.assertEqual(raw["type"], "physical_trot")
        self.assertEqual(loaded["type"], "physical_trot")
        for name in FAST_TROT_PARAMETER_NAMES:
            with self.subTest(parameter=name):
                self.assertEqual(loaded[name], float(raw[name]))
        self.assertEqual(set(raw["presets"]), set(FAST_TROT_PRESETS))
        for preset_name in FAST_TROT_PRESETS:
            for name in FAST_TROT_PRESET_PARAMETER_NAMES:
                with self.subTest(preset=preset_name, parameter=name):
                    self.assertEqual(
                        loaded["presets"][preset_name][name],
                        float(raw["presets"][preset_name][name]),
                    )

        # Prove the loader does not substitute a module constant for the
        # trajectory: a valid alternate YAML owns the resulting values.
        alternate = _raw_gait_yaml()
        alternate_fast = _raw_fast_trot(alternate)
        alternate_fast.update({
            "simulation_cycle_period": 0.44,
            "step_length_x": 0.060,
            "stride_length": 0.060,
            "gait_frequency": 1.0 / 0.44,
            "touchdown_lead_x": 0.032,
            "liftoff_trail_x": -0.028,
        })
        alternate_fast["presets"]["bench"]["hardware_speed_scale"] = 0.30
        with tempfile.TemporaryDirectory() as directory:
            alternate_loaded = load_fast_trot_config(
                _write_yaml(directory, alternate)
            )
        self.assertEqual(alternate_loaded["simulation_cycle_period"], 0.44)
        self.assertEqual(alternate_loaded["step_length_x"], 0.060)
        self.assertEqual(alternate_loaded["touchdown_lead_x"], 0.032)
        self.assertEqual(alternate_loaded["liftoff_trail_x"], -0.028)

    def test_physical_ranges_and_named_presets_are_explicit(self):
        config = load_fast_trot_config(CONFIG_PATH)
        expected_presets = {
            "bench": (0.50, 0.25, 0.75, 0.025),
            "floor_test": (0.65, 0.40, 0.68, 0.030),
            "wide": (0.85, 0.55, 0.62, 0.035),
        }
        self.assertEqual(
            FAST_TROT_TUNING_BOUNDS,
            {
                "stride_scale": (0.50, 1.25),
                "step_height": (0.020, 0.050),
                "hardware_cycle_period": (0.50, 0.90),
                "hardware_speed_scale": (0.20, 0.75),
            },
        )
        self.assertEqual(config["maximum_safe_stride_scale"], 1.25)
        self.assertEqual(config["startup_stride_fraction"], 0.30)
        self.assertEqual(config["touchdown_lead_x"], 0.0275)
        self.assertEqual(config["liftoff_trail_x"], -0.0275)
        self.assertLessEqual(
            config["step_length_x"],
            config["max_step_length_x"],
        )
        self.assertAlmostEqual(
            config["stance_ratio"] + config["swing_ratio"],
            1.0,
        )

        for name, expected in expected_presets.items():
            preset = config["presets"][name]
            actual = tuple(
                preset[field] for field in FAST_TROT_PRESET_PARAMETER_NAMES
            )
            self.assertEqual(actual, expected, name)
            validate_fast_trot_tuning(config, preset)

        hardware = _controller(hardware=True)
        self.assertEqual(
            hardware.fast_trot_tuning,
            config["presets"]["bench"],
            "physical fast trot must default to BENCH, never WIDE",
        )

    def test_invalid_yaml_and_runtime_tuning_are_rejected_not_clamped(self):
        base = _raw_gait_yaml()
        invalid = []

        raw = copy.deepcopy(base)
        del _raw_fast_trot(raw)["step_length_x"]
        invalid.append(("missing required scalar", raw))

        raw = copy.deepcopy(base)
        _raw_fast_trot(raw)["type"] = "phase_trot"
        invalid.append(("wrong gait type", raw))

        raw = copy.deepcopy(base)
        _raw_fast_trot(raw)["step_height"] = "high"
        invalid.append(("non-numeric scalar", raw))

        raw = copy.deepcopy(base)
        _raw_fast_trot(raw)["cycle_period"] = float("nan")
        invalid.append(("non-finite scalar", raw))

        raw = copy.deepcopy(base)
        _raw_fast_trot(raw)["max_step_length_x"] = 0.080
        invalid.append(("workspace maximum exceeded", raw))

        raw = copy.deepcopy(base)
        _raw_fast_trot(raw)["liftoff_trail_x"] = -0.010
        invalid.append(("lead-to-trail mismatch", raw))

        raw = copy.deepcopy(base)
        _raw_fast_trot(raw)["swing_ratio"] = 0.50
        _raw_fast_trot(raw)["stance_ratio"] = 0.50
        invalid.append(("no diagonal stance overlap", raw))

        raw = copy.deepcopy(base)
        _raw_fast_trot(raw)["maximum_safe_stride_scale"] = 1.30
        invalid.append(("unsafe scale ceiling", raw))

        raw = copy.deepcopy(base)
        fast = _raw_fast_trot(raw)
        fast["step_length_x"] = 0.075
        fast["touchdown_lead_x"] = 0.0375
        fast["liftoff_trail_x"] = -0.0375
        invalid.append(("scaled stride bypasses workspace maximum", raw))

        raw = copy.deepcopy(base)
        del _raw_fast_trot(raw)["presets"]["bench"]
        invalid.append(("missing safe preset", raw))

        raw = copy.deepcopy(base)
        _raw_fast_trot(raw)["presets"]["wide"]["stride_scale"] = 1.30
        invalid.append(("preset outside safe bounds", raw))

        raw = copy.deepcopy(base)
        _raw_fast_trot(raw)["presets"]["bench"]["stride_typo"] = 0.50
        invalid.append(("unknown preset field", raw))

        for label, raw in invalid:
            with self.subTest(case=label), tempfile.TemporaryDirectory() as directory:
                with self.assertRaises(ValueError):
                    load_fast_trot_config(_write_yaml(directory, raw))

        config = load_fast_trot_config(CONFIG_PATH)
        bad_runtime_tuning = (
            {"stride_scale": 0.50},
            dict(config["presets"]["bench"], stride_scale=1.26),
            dict(config["presets"]["bench"], extra=1.0),
            dict(config["presets"]["bench"], step_height=float("inf")),
            dict(
                config["presets"]["wide"],
                stride_scale=1.25,
                hardware_speed_scale=0.55,
            ),
        )
        for tuning in bad_runtime_tuning:
            with self.subTest(tuning=tuning), self.assertRaises(ValueError):
                validate_fast_trot_tuning(config, tuning)


class FastTrotTrajectoryTests(unittest.TestCase):
    def test_hardware_swing_lifts_then_transfers_then_lowers(self):
        controller = VoltGaitController(
            load_gait_configs(CONFIG_PATH, PHYSICAL_CONFIG_PATH),
            hardware_mode=True,
        )
        controller.set_gait("fast_trot", 0.0)
        leg = "front_left"
        origin = (0.10, 0.104, -0.20)
        target = (0.15, 0.104, -0.20)
        height = 0.030
        controller.swing_origins[leg] = origin
        controller.swing_targets[leg] = target
        controller.swing_heights[leg] = height
        liftoff = controller.config["liftoff_blend"]
        touchdown = controller.config["touchdown_blend"]

        lifting = controller.phase_trot_swing_step(leg, 0.5 * liftoff)
        self.assertAlmostEqual(lifting[0], origin[0])
        self.assertGreater(lifting[2], origin[2])
        transferred = controller.phase_trot_swing_step(leg, 0.5)
        self.assertGreater(transferred[0], origin[0])
        self.assertLess(transferred[0], target[0])
        self.assertAlmostEqual(transferred[2], origin[2] + height)
        lowering = controller.phase_trot_swing_step(
            leg,
            1.0 - 0.5 * touchdown,
        )
        self.assertAlmostEqual(lowering[0], target[0])
        self.assertGreater(lowering[2], target[2])

    def test_hardware_ready_stage_keeps_all_four_feet_planted(self):
        controller = VoltGaitController(
            load_gait_configs(CONFIG_PATH, PHYSICAL_CONFIG_PATH),
            hardware_mode=True,
        )
        controller.set_gait("fast_trot", 0.0)
        velocity = (controller.fast_trot_command_limits()[0], 0.0, 0.0)
        original_world = dict(controller.world_feet)

        for index in range(1, 90):
            now = index * 0.01
            _feet, _body, active = controller.step(
                now,
                0.01,
                velocity,
            )
            self.assertTrue(active)
            self.assertEqual(
                controller.debug_snapshot()["swing_legs"],
                [],
            )
            self.assertEqual(controller.world_feet, original_world)
            self.assertEqual(
                controller.debug_snapshot()["step_state"],
                "TROT_READY",
            )

    def test_diagonal_pairs_are_exactly_half_a_cycle_apart_and_keep_order(self):
        self.assertEqual(TROT_PHASE_OFFSETS["front_left"], 0.0)
        self.assertEqual(TROT_PHASE_OFFSETS["rear_right"], 0.0)
        self.assertEqual(TROT_PHASE_OFFSETS["front_right"], 0.5)
        self.assertEqual(TROT_PHASE_OFFSETS["rear_left"], 0.5)
        self.assertAlmostEqual(
            (
                TROT_PHASE_OFFSETS["front_right"]
                - TROT_PHASE_OFFSETS["front_left"]
            ) % 1.0,
            0.5,
        )

        config = GAITS["fast_trot"]
        for index in range(200):
            phase = index / 200.0
            swinging = frozenset(
                leg
                for leg in LEG_ORDER
                if (
                    phase + TROT_PHASE_OFFSETS[leg]
                ) % 1.0 < config["swing_ratio"]
            )
            with self.subTest(phase=phase):
                self.assertIn(swinging, DIAGONAL_PAIRS | {frozenset()})

        controller = _controller()
        velocity = _full_forward_command(controller)
        sequence = []
        previous = frozenset()
        for index in range(1, int(math.ceil(1.6 / DT)) + 1):
            controller.step(index * DT, DT, velocity)
            swing = frozenset(controller.debug_snapshot()["swing_legs"])
            if swing and swing != previous:
                sequence.append(swing)
            previous = swing
        self.assertGreaterEqual(len(sequence), 4)
        self.assertEqual(sequence[:4], [PAIR_A, PAIR_B, PAIR_A, PAIR_B])

    def test_forward_touchdown_lead_and_full_world_locked_stance_sweep(self):
        result = _measure_forward_stance_sweep()
        config = result["controller"].config
        self.assertGreater(result["touchdown_lead"], 0.0)
        self.assertLess(result["liftoff_trail"], 0.0)
        self.assertGreaterEqual(
            result["stance_sweep"],
            0.80 * config["step_length_x"],
        )
        self.assertGreaterEqual(
            result["stance_sweep"],
            0.80 * result["requested_stride"],
        )

        controller = _controller()
        velocity = _full_forward_command(controller)
        observed_locked_samples = 0
        for index in range(1, int(math.ceil(2.5 / DT)) + 1):
            before_stance = set(controller.debug_snapshot()["stance_legs"])
            before_world = dict(controller.world_feet)
            before_feet = dict(controller.feet)
            controller.step(index * DT, DT, velocity)
            after_stance = set(controller.debug_snapshot()["stance_legs"])
            if index * DT <= controller.config["startup_ramp_time"]:
                continue
            for leg in before_stance & after_stance:
                self.assertEqual(
                    controller.world_feet[leg],
                    before_world[leg],
                    "stance foothold slipped in world coordinates",
                )
                self.assertLess(
                    controller.feet[leg][0],
                    before_feet[leg][0],
                    "forward body motion did not sweep stance foot backward",
                )
                observed_locked_samples += 1
        self.assertGreater(observed_locked_samples, 100)

    def test_presets_widen_monotonically_and_stay_inside_workspace(self):
        config = GAITS["fast_trot"]
        rows = []
        for name in FAST_TROT_PRESETS:
            tuning = config["presets"][name]
            result = _measure_forward_stance_sweep(
                hardware=True,
                tuning=tuning,
            )
            expected = (
                config["step_length_x"]
                * tuning["stride_scale"]
                * config["hardware_stride_scale"]
            )
            rows.append({
                "name": name,
                "expected": expected,
                "measured": result["stance_sweep"],
                "reported": result["achieved_stride"],
            })

        message = "\n" + "\n".join(
            "%-10s expected=%6.2f mm measured=%6.2f mm reported=%6.2f mm"
            % (
                row["name"],
                1000.0 * row["expected"],
                1000.0 * row["measured"],
                1000.0 * row["reported"],
            )
            for row in rows
        )
        measured = [row["measured"] for row in rows]
        self.assertEqual(measured, sorted(measured), message)
        self.assertEqual(len(set(measured)), len(measured), message)
        for row in rows:
            self.assertGreaterEqual(
                row["measured"],
                0.80 * row["expected"],
                message,
            )
            self.assertLessEqual(
                row["measured"],
                config["max_step_length_x"] + 1e-9,
                message,
            )
            self.assertAlmostEqual(
                row["reported"],
                row["expected"],
                delta=1e-9,
                msg=message,
            )

    def test_increasing_stride_scale_increases_cartesian_displacement(self):
        config = GAITS["fast_trot"]
        low = dict(config["presets"]["wide"], stride_scale=0.50)
        high = dict(
            config["presets"]["wide"],
            stride_scale=1.00,
            hardware_speed_scale=0.60,
        )
        low_result = _measure_forward_stance_sweep(tuning=low)
        high_result = _measure_forward_stance_sweep(tuning=high)

        self.assertGreater(
            high_result["stance_sweep"],
            1.75 * low_result["stance_sweep"],
        )
        self.assertLessEqual(
            high_result["stance_sweep"],
            config["max_step_length_x"] + 1e-9,
        )

    def test_sin_squared_swing_has_grounded_zero_velocity_endpoints(self):
        controller = _controller()
        leg = "front_left"
        origin = (0.080, 0.104, -0.200)
        target = (0.145, 0.104, -0.200)
        height = 0.035
        controller.swing_origins[leg] = origin
        controller.swing_targets[leg] = target
        controller.swing_heights[leg] = height

        self.assertEqual(controller.phase_trot_swing_step(leg, 0.0), origin)
        self.assertEqual(controller.phase_trot_swing_step(leg, 1.0), target)
        midpoint = controller.phase_trot_swing_step(leg, 0.5)
        self.assertAlmostEqual(midpoint[2], origin[2] + height)
        for step in range(1, 10):
            point = controller.phase_trot_swing_step(leg, step / 10.0)
            self.assertGreater(point[2], origin[2])
            self.assertTrue(all(math.isfinite(value) for value in point))

        epsilon = 1e-5
        start_next = controller.phase_trot_swing_step(leg, epsilon)
        end_previous = controller.phase_trot_swing_step(leg, 1.0 - epsilon)
        for axis in (0, 2):
            with self.subTest(endpoint="liftoff", axis=axis):
                self.assertLess(
                    abs(start_next[axis] - origin[axis]) / epsilon,
                    1e-4,
                )
            with self.subTest(endpoint="touchdown", axis=axis):
                self.assertLess(
                    abs(target[axis] - end_previous[axis]) / epsilon,
                    1e-4,
                )


class FastTrotSafetyTests(unittest.TestCase):
    def test_phase_governor_holds_every_contact_boundary_until_ready(self):
        controller = _controller(hardware=True)
        controller.trot_phase = controller.config["swing_ratio"] - 0.001
        controller.fast_trot_cycle_period = 0.75
        controller.set_support_feedback({
            "command_error": math.radians(4.0),
            "tracking_available": False,
            "tracking_required": False,
        })

        first_dt = controller.fast_trot_governed_dt(0.02)
        controller.advance_trot_phase(
            first_dt,
            (0.0, 0.0, 0.0),
        )
        self.assertIsNotNone(controller.fast_trot_transition_hold_phase)
        self.assertLess(
            controller.trot_phase,
            controller.config["swing_ratio"],
        )
        self.assertEqual(controller.fast_trot_governed_dt(0.02), 0.0)

        controller.set_support_feedback({
            "command_error": 0.0,
            "tracking_available": False,
            "tracking_required": True,
        })
        self.assertEqual(controller.fast_trot_governed_dt(0.02), 0.0)

        controller.set_support_feedback({
            "command_error": math.radians(0.5),
            "tracking_available": False,
            "tracking_required": False,
        })
        self.assertEqual(controller.fast_trot_governed_dt(0.02), 0.0)
        released_dt = controller.fast_trot_governed_dt(0.02)
        self.assertGreater(released_dt, 0.0)
        controller.advance_trot_phase(
            released_dt,
            (0.0, 0.0, 0.0),
        )
        self.assertGreaterEqual(
            controller.trot_phase,
            controller.config["swing_ratio"],
        )

    def test_all_presets_and_max_scale_have_finite_unclamped_ik(self):
        config = GAITS["fast_trot"]
        cases = [
            (name, True, dict(config["presets"][name]))
            for name in FAST_TROT_PRESETS
        ]
        maximum = dict(
            config["presets"]["wide"],
            stride_scale=config["maximum_safe_stride_scale"],
            hardware_speed_scale=FAST_TROT_TUNING_BOUNDS[
                "hardware_speed_scale"
            ][1],
        )
        cases.extend((
            ("hardware_1.25", True, maximum),
            ("simulation_1.25", False, maximum),
        ))
        calibration = ServoCalibrationTable.from_file(CALIBRATION_PATH)
        rows = []

        for case_name, hardware, tuning in cases:
            controller = _controller(hardware=hardware, tuning=tuning)
            velocity = _full_forward_command(controller)
            warmup = (
                controller.fast_trot_active_cycle_period()
                / config["startup_cycle_speed_fraction"]
                + config["startup_ramp_time"]
                + 0.02
            )
            end = warmup + controller.fast_trot_active_cycle_period() + 0.02
            joint_min = [float("inf")] * len(JOINT_NAMES)
            joint_max = [float("-inf")] * len(JOINT_NAMES)
            ik_projections = 0
            ik_clamps = 0
            servo_clamps = 0
            sample_count = 0

            for index in range(1, int(math.ceil(end / 0.005)) + 1):
                now = index * 0.005
                feet, body, _active = controller.step(
                    now,
                    0.005,
                    velocity,
                )
                if now < warmup:
                    continue
                positions, diagnostics = feet_to_joint_positions_diagnostic(
                    feet,
                    height=0.200 + body["height"],
                    roll=body["roll"],
                    pitch=body["pitch"],
                )
                self.assertEqual(len(positions), 12, case_name)
                self.assertTrue(
                    all(math.isfinite(value) for value in positions),
                    case_name,
                )
                self.assertTrue(
                    all(
                        math.isfinite(value)
                        for point in feet.values()
                        for value in point
                    ),
                    case_name,
                )
                ik_projections += len(diagnostics["projected_targets"])
                ik_clamps += sum(
                    len(diagnostics["legs"][leg]["clamped_joints"])
                    for leg in LEG_ORDER
                )
                frame, details = calibration.channel_frame_from_positions(
                    named_positions_from_ordered(positions)
                )
                self.assertEqual(len(frame), 12, case_name)
                self.assertTrue(
                    all(math.isfinite(value) for value in frame),
                    case_name,
                )
                servo_clamps += sum(item["clamped"] for item in details)
                for joint_index, value in enumerate(positions):
                    joint_min[joint_index] = min(
                        joint_min[joint_index],
                        value,
                    )
                    joint_max[joint_index] = max(
                        joint_max[joint_index],
                        value,
                    )
                sample_count += 1

            self.assertGreater(sample_count, 20, case_name)
            rows.append({
                "case": case_name,
                "stride": controller.fast_trot_achieved_stride,
                "joint_excursion_deg": max(
                    math.degrees(high - low)
                    for low, high in zip(joint_min, joint_max)
                ),
                "ik_projections": ik_projections,
                "ik_clamps": ik_clamps,
                "servo_clamps": servo_clamps,
            })

        table = _format_diagnostic_table(rows)
        for row in rows:
            self.assertLessEqual(
                row["stride"],
                config["max_step_length_x"],
                table,
            )
            self.assertGreater(row["joint_excursion_deg"], 0.0, table)
            self.assertEqual(row["ik_projections"], 0, table)
            self.assertEqual(row["ik_clamps"], 0, table)
            self.assertEqual(row["servo_clamps"], 0, table)

    def test_sagittal_ik_is_symmetric_and_mount_direction_is_calibration_only(self):
        # This is a symmetric, forward-biased pose inside the configured
        # fast-trot envelope. Mirroring y must not invert leg/knee IK.
        pose = {
            "front_left": (0.133, 0.104, -0.200),
            "front_right": (0.133, -0.104, -0.200),
            "rear_left": (-0.098, 0.104, -0.200),
            "rear_right": (-0.098, -0.104, -0.200),
        }
        positions, diagnostics = feet_to_joint_positions_diagnostic(
            pose,
            height=0.200 + GAITS["fast_trot"]["body_height_offset"],
            pitch=GAITS["fast_trot"]["forward_pitch_bias"],
        )
        named = named_positions_from_ordered(positions)
        self.assertEqual(diagnostics["projected_targets"], [])

        for axle in ("front", "rear"):
            left = "%s_left" % axle
            right = "%s_right" % axle
            self.assertAlmostEqual(
                named["%s_shoulder" % left],
                -named["%s_shoulder" % right],
            )
            self.assertAlmostEqual(
                named["%s_leg" % left],
                named["%s_leg" % right],
            )
            self.assertAlmostEqual(
                named["%s_foot" % left],
                named["%s_foot" % right],
            )

        calibration = ServoCalibrationTable.from_file(CALIBRATION_PATH)
        _frame, details = calibration.channel_frame_from_positions(named)
        self.assertFalse(any(item["clamped"] for item in details))
        for joint_name, radians in named.items():
            servo = calibration.servos[joint_name]
            expected = (
                servo.neutral_deg
                + servo.trim_deg
                + servo.direction * math.degrees(radians)
            )
            self.assertAlmostEqual(
                calibration.ros_radians_to_servo_degrees(
                    joint_name,
                    radians,
                ),
                expected,
                msg=joint_name,
            )

    def test_startup_holds_thirty_percent_stride_for_one_slow_cycle(self):
        config = GAITS["fast_trot"]
        cases = [("simulation", False, None)] + [
            (name, True, config["presets"][name])
            for name in FAST_TROT_PRESETS
        ]
        for name, hardware, tuning in cases:
            with self.subTest(case=name):
                controller = _controller(
                    hardware=hardware,
                    tuning=tuning,
                )
                start = controller.fast_trot_startup_profile(0.0)
                self.assertEqual(start, (0.30, 0.70, 0.60))
                slow_period = (
                    controller.fast_trot_active_cycle_period()
                    / config["startup_cycle_speed_fraction"]
                )
                before_first_cycle = controller.fast_trot_startup_profile(
                    slow_period - 1e-9
                )
                self.assertAlmostEqual(before_first_cycle[0], 0.30)
                self.assertAlmostEqual(before_first_cycle[1], 0.70)

                profile_controller = _controller(
                    hardware=hardware,
                    tuning=tuning,
                )
                total_startup = (
                    slow_period + config["startup_ramp_time"]
                )
                samples = [
                    profile_controller.fast_trot_startup_profile(
                        total_startup * index / 300.0
                    )
                    for index in range(301)
                ]
                first_stride_rise = next(
                    index
                    for index, sample in enumerate(samples)
                    if sample[0] > 0.300001
                )
                first_cadence_rise = next(
                    index
                    for index, sample in enumerate(samples)
                    if sample[1] > 0.700001
                )
                self.assertLess(first_stride_rise, first_cadence_rise)
                self.assertGreaterEqual(
                    first_stride_rise * total_startup / 300.0,
                    slow_period,
                )
                self.assertAlmostEqual(samples[-1][0], 1.0)
                self.assertAlmostEqual(samples[-1][1], 1.0)

    def test_stop_finishes_active_pair_then_holds_loaded_footprint(self):
        controller = _controller(hardware=True)
        velocity = _full_forward_command(controller)
        now = 0.0
        active_pair = None

        for index in range(1, 1000):
            now = index * 0.005
            controller.step(now, 0.005, velocity)
            swing = frozenset(controller.debug_snapshot()["swing_legs"])
            startup_duration = (
                controller.config["trot_ready_time"]
                + controller.fast_trot_active_cycle_period()
                / controller.config["startup_cycle_speed_fraction"]
                + controller.config["startup_ramp_time"]
            )
            if now > startup_duration and swing:
                active_pair = swing
                break
        self.assertIn(active_pair, DIAGONAL_PAIRS)
        controller.request_stop()

        observed_pairs = []
        active_pair_grounded = False
        grounded_footprint = None
        for _index in range(1, 1000):
            now += 0.005
            feet, _body, active = controller.step(
                now,
                0.005,
                (0.0, 0.0, 0.0),
            )
            swing = frozenset(controller.debug_snapshot()["swing_legs"])
            if swing and (not observed_pairs or swing != observed_pairs[-1]):
                observed_pairs.append(swing)
            if not swing and not active_pair_grounded:
                active_pair_grounded = all(
                    abs(feet[leg][2] - NOMINAL_FEET[leg][2]) < 1e-9
                    for leg in active_pair
                )
                grounded_footprint = {
                    leg: tuple(feet[leg]) for leg in LEG_ORDER
                }
            if not active:
                break
        else:
            self.fail("fast trot did not complete its controlled stop")

        self.assertTrue(active_pair_grounded)
        self.assertTrue(all(pair == active_pair for pair in observed_pairs))
        self.assertFalse(controller.active)
        self.assertFalse(controller.settling)
        self.assertEqual(controller.debug_snapshot()["swing_legs"], [])
        self.assertAlmostEqual(controller.fast_trot_achieved_stride, 0.0)
        self.assertAlmostEqual(controller.fast_trot_requested_stride, 0.0)
        self.assertEqual(
            controller.debug_snapshot()["planned_velocity"],
            [0.0, 0.0, 0.0],
        )
        self.assertEqual(
            controller.debug_snapshot()["input_velocity"],
            [0.0, 0.0, 0.0],
        )
        self.assertIsNotNone(grounded_footprint)
        for leg in LEG_ORDER:
            self.assertEqual(
                controller.feet[leg],
                grounded_footprint[leg],
                "physical stop must not recenter a planted foot",
            )


if __name__ == "__main__":
    unittest.main()
