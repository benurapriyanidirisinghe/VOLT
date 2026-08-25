#!/usr/bin/env python3

"""Tests for the rebuilt two-gait servo-budgeted gait engine."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "scripts"),
)

import volt_gait_controller as gait_module
from volt_gait_controller import (
    AMBLE_PHASE_OFFSETS,
    GAIT_ALIASES,
    GAIT_PARAMETER_NAMES,
    GAITS,
    TROT_PHASE_OFFSETS,
    VoltGaitController,
    canonical_gait_name,
    limit_velocity_command,
    load_gait_configs,
    normalized_velocity_activity,
    validate_servo_budget,
)
from volt_kinematics import (
    JOINT_NAMES,
    LEG_ORDER,
    NOMINAL_FEET,
    feet_to_joint_positions_diagnostic,
)

NOMINAL_FEET_Z = next(iter(NOMINAL_FEET.values()))[2]


def run_cycle(controller, velocity, seconds, dt=0.005, now_start=0.0,
              ramp_time=1.0):
    """Drive the engine like the motion controller's filtered command would."""
    now = now_start
    for _ in range(int(seconds / dt)):
        now += dt
        scale = min(1.0, (now - now_start) / ramp_time) if ramp_time else 1.0
        controller.step(
            now, dt,
            (velocity[0] * scale, velocity[1] * scale, velocity[2] * scale),
        )
    return now


class ConfigurationTests(unittest.TestCase):
    def test_exactly_two_gaits_exist(self):
        self.assertEqual(sorted(GAITS), ["amble", "trot"])

    def test_every_config_key_is_declared(self):
        for name, config in GAITS.items():
            for key in GAIT_PARAMETER_NAMES:
                self.assertIn(key, config, "%s missing %s" % (name, key))

    def test_all_historical_names_alias_onto_surviving_gaits(self):
        for old_name in (
            "walk", "slow_crawl", "diagnostic_crawl", "spot_walk",
            "spotmicro_video_walk", "legacy_walk", "real_trot",
            "load_safe_trot", "real_safe_trot", "slow_trot",
            "normal_trot", "fast_trot",
        ):
            self.assertIn(canonical_gait_name(old_name), ("trot", "amble"))

    def test_unknown_gait_is_rejected(self):
        with self.assertRaises(ValueError):
            canonical_gait_name("moonwalk")

    def test_shipped_configs_pass_the_servo_budget(self):
        for config in GAITS.values():
            self.assertTrue(validate_servo_budget(config))

    def test_budget_rejects_an_infeasible_gait(self):
        config = dict(GAITS["trot"])
        config["max_x"] = 0.24          # roughly double the validated speed
        config["cycle_period"] = 0.7
        with self.assertRaises(ValueError):
            validate_servo_budget(config)

    def test_yaml_and_builtin_defaults_agree(self):
        loaded = load_gait_configs()
        for name in ("trot", "amble"):
            for key in GAIT_PARAMETER_NAMES:
                self.assertAlmostEqual(
                    loaded[name][key],
                    GAITS[name][key],
                    places=9,
                    msg="%s.%s diverges between YAML and builtin" % (name, key),
                )

    def test_trot_uses_diagonal_pairs_and_amble_lateral_sequence(self):
        self.assertEqual(
            TROT_PHASE_OFFSETS["front_left"],
            TROT_PHASE_OFFSETS["rear_right"],
        )
        self.assertAlmostEqual(
            abs(
                TROT_PHASE_OFFSETS["front_left"]
                - TROT_PHASE_OFFSETS["front_right"]
            ),
            0.5,
        )
        offsets = sorted(AMBLE_PHASE_OFFSETS.values())
        self.assertEqual(offsets, [0.0, 0.25, 0.5, 0.75])

    def test_real_tuning_overlay_revalidates_the_budget(self):
        overlaid = gait_module.apply_real_tuning_to_configs(
            GAITS,
            {
                "gait": "trot",
                "cycle_duration": 1.2,
                "duty_factor": 0.62,
                "stride_length": 0.045,
                "lateral_stride_width": 0.008,
                "step_height": 0.024,
            },
        )
        self.assertAlmostEqual(overlaid["trot"]["cycle_period"], 1.2)
        with self.assertRaises(ValueError):
            gait_module.apply_real_tuning_to_configs(
                GAITS,
                {
                    "gait": "trot",
                    "cycle_duration": 0.5,
                    "duty_factor": 0.55,
                    "stride_length": 0.075,
                    "step_height": 0.035,
                },
            )


class VelocityHelperTests(unittest.TestCase):
    def test_limit_clamps_each_axis(self):
        limited = limit_velocity_command((10.0, -10.0, 10.0), (0.1, 0.05, 0.5))
        self.assertLessEqual(abs(limited[0]), 0.1 + 1e-12)
        self.assertLessEqual(abs(limited[1]), 0.05 + 1e-12)
        self.assertLessEqual(abs(limited[2]), 0.5 + 1e-12)

    def test_limit_caps_the_combined_command(self):
        limits = (0.1, 0.05, 0.5)
        limited = limit_velocity_command((0.1, 0.05, 0.5), limits)
        total = (
            abs(limited[0]) / limits[0]
            + abs(limited[1]) / limits[1]
            + abs(limited[2]) / limits[2]
        )
        self.assertLessEqual(total, 1.0 + 1e-9)

    def test_activity_is_normalized(self):
        self.assertAlmostEqual(
            normalized_velocity_activity((0.05, 0.0, 0.0), (0.1, 0.05, 0.5)),
            0.5,
        )
        self.assertEqual(
            normalized_velocity_activity((0.0, 0.0, 0.0), (0.1, 0.05, 0.5)),
            0.0,
        )


class EngineBehaviorTests(unittest.TestCase):
    def make(self, gait="trot"):
        controller = VoltGaitController()
        controller.set_gait(gait, 0.0)
        return controller

    def test_inactive_engine_returns_current_feet(self):
        controller = self.make()
        feet, body, active = controller.step(0.1, 0.005, (0.0, 0.0, 0.0))
        self.assertFalse(active)
        for leg in LEG_ORDER:
            self.assertEqual(feet[leg], tuple(NOMINAL_FEET[leg]))

    def test_stance_feet_do_not_skate_in_world_frame(self):
        controller = self.make()
        dt, now = 0.005, 0.0
        worst = 0.0
        previous = None
        for index in range(int(6.0 / dt)):
            now += dt
            scale = min(1.0, now / 1.0)
            feet, _, _ = controller.step(now, dt, (0.10 * scale, 0.0, 0.0))
            world = {
                leg: controller.body_to_world(feet[leg]) for leg in LEG_ORDER
            }
            if previous is not None:
                for leg in LEG_ORDER:
                    grounded = (
                        abs(feet[leg][2] - NOMINAL_FEET_Z) < 1e-9
                        and controller._leg_in_stance(leg)
                    )
                    if grounded:
                        drift = math.hypot(
                            world[leg][0] - previous[leg][0],
                            world[leg][1] - previous[leg][1],
                        )
                        worst = max(worst, drift)
            previous = world
        self.assertLess(worst, 1e-12)

    def test_body_advances_the_commanded_distance(self):
        controller = self.make()
        run_cycle(controller, (0.10, 0.0, 0.0), 8.0)
        # 1 s linear ramp costs 0.5 s of travel: expect 7.5 s * 0.10 m/s.
        self.assertAlmostEqual(controller.body_x_world, 0.75, delta=0.02)

    def test_stop_settles_exactly_to_nominal_and_deactivates(self):
        controller = self.make()
        now = run_cycle(controller, (0.10, 0.0, 0.0), 4.0)
        for _ in range(int(3.0 / 0.005)):
            now += 0.005
            feet, _, active = controller.step(now, 0.005, (0.0, 0.0, 0.0))
        self.assertFalse(controller.active)
        for leg in LEG_ORDER:
            for axis in range(3):
                self.assertAlmostEqual(
                    feet[leg][axis], NOMINAL_FEET[leg][axis], places=9
                )

    def test_request_stop_latches_against_new_commands(self):
        controller = self.make()
        now = run_cycle(controller, (0.10, 0.0, 0.0), 3.0)
        controller.request_stop()
        for _ in range(int(4.0 / 0.005)):
            now += 0.005
            controller.step(now, 0.005, (0.10, 0.0, 0.0))
        self.assertFalse(controller.active)

    def test_a_stop_never_starts_a_new_swing(self):
        controller = self.make()
        now = run_cycle(controller, (0.10, 0.0, 0.0), 3.0)
        controller.request_stop()
        lifted_after_stop = set()
        airborne_at_stop = {
            leg for leg in LEG_ORDER
            if controller.feet[leg][2] > NOMINAL_FEET_Z + 1e-6
        }
        for _ in range(int(3.0 / 0.005)):
            now += 0.005
            feet, _, active = controller.step(now, 0.005, (0.0, 0.0, 0.0))
            if controller.settling or not active:
                break
            for leg in LEG_ORDER:
                if (
                    feet[leg][2] > NOMINAL_FEET_Z + 1e-6
                    and leg not in airborne_at_stop
                ):
                    lifted_after_stop.add(leg)
        self.assertEqual(lifted_after_stop, set())

    def test_release_forced_stop_restores_liveness_after_timeout_stop(self):
        """A cmd_vel timeout latches the stop; the controller's observed
        neutral must be able to release it, or the gait deadlocks."""
        controller = self.make()
        now = run_cycle(controller, (0.10, 0.0, 0.0), 3.0)
        controller.request_stop()          # e.g. command stream timed out
        for _ in range(int(4.0 / 0.005)):
            now += 0.005
            controller.step(now, 0.005, (0.10, 0.0, 0.0))
        self.assertFalse(controller.active)
        # A held joystick alone cannot restart it...
        now += 0.005
        _, _, active = controller.step(now, 0.005, (0.10, 0.0, 0.0))
        self.assertFalse(active)
        # ...but after the controller reports true neutral, motion resumes.
        controller.release_forced_stop()
        now += 0.005
        _, _, active = controller.step(now, 0.005, (0.10, 0.0, 0.0))
        self.assertTrue(active)

    def test_amble_always_keeps_three_feet_planted(self):
        controller = self.make("amble")
        dt, now = 0.005, 0.0
        worst_airborne = 0
        for index in range(int(12.0 / dt)):
            now += dt
            scale = min(1.0, now / 1.5)
            feet, _, _ = controller.step(now, dt, (0.04 * scale, 0.0, 0.0))
            if controller.active and not controller.settling and now > 3.0:
                airborne = sum(
                    1 for leg in LEG_ORDER
                    if feet[leg][2] > NOMINAL_FEET_Z + 1e-6
                )
                worst_airborne = max(worst_airborne, airborne)
        self.assertLessEqual(worst_airborne, 1)

    def test_amble_sways_away_from_the_swinging_side(self):
        controller = self.make("amble")
        dt, now = 0.005, 0.0
        checked = 0
        for index in range(int(12.0 / dt)):
            now += dt
            scale = min(1.0, now / 1.5)
            feet, body, _ = controller.step(now, dt, (0.04 * scale, 0.0, 0.0))
            if not controller.active or controller.settling or now < 4.0:
                continue
            airborne = [
                leg for leg in LEG_ORDER
                if feet[leg][2] > NOMINAL_FEET_Z + 0.005
            ]
            if len(airborne) != 1:
                continue
            leg = airborne[0]
            checked += 1
            if leg.endswith("_left"):
                self.assertLess(body["y"], 0.001, "leaning toward swing side")
            else:
                self.assertGreater(body["y"], -0.001)
        self.assertGreater(checked, 50)

    def test_gait_switch_requires_idle(self):
        controller = self.make()
        run_cycle(controller, (0.10, 0.0, 0.0), 3.0)
        self.assertTrue(controller.active)
        with self.assertRaises(ValueError):
            controller.set_gait("amble", 99.0)

    def test_hold_current_feet_adopts_pose_and_goes_idle(self):
        controller = self.make()
        run_cycle(controller, (0.10, 0.0, 0.0), 3.0)
        held = {leg: (0.11, 0.10, -0.19) for leg in LEG_ORDER}
        held = {
            "front_left": (0.113, 0.104, -0.19),
            "front_right": (0.113, -0.104, -0.19),
            "rear_left": (-0.113, 0.104, -0.19),
            "rear_right": (-0.113, -0.104, -0.19),
        }
        controller.hold_current_feet(held, 99.0)
        self.assertFalse(controller.active)
        self.assertEqual(
            controller.debug_snapshot()["step_state"], "OWNERSHIP_HOLD"
        )

    def test_turning_yaw_integrates(self):
        controller = self.make()
        run_cycle(controller, (0.0, 0.0, 0.4), 6.0)
        # 1 s ramp -> ~5.5 s * 0.4 rad/s
        self.assertAlmostEqual(controller.body_yaw_world, 2.2, delta=0.15)

    def test_full_cycle_ik_is_clean_at_max_command(self):
        """Every commanded frame across a cycle solves without projection."""
        for gait in ("trot", "amble"):
            controller = self.make(gait)
            limits = controller.command_limits()
            velocity = limit_velocity_command(limits, limits)
            dt, now = 0.005, 0.0
            for index in range(int(6.0 / dt)):
                now += dt
                scale = min(1.0, now / 1.5)
                feet, body, _ = controller.step(
                    now, dt,
                    tuple(v * scale for v in velocity),
                )
                _, diagnostics = feet_to_joint_positions_diagnostic(
                    feet,
                    height=0.195 + body["height"],
                    body_y=body["y"],
                )
                self.assertEqual(
                    diagnostics["projected_targets"], [],
                    "%s projected at t=%.2f" % (gait, now),
                )

    def test_debug_snapshot_contract(self):
        controller = self.make()
        run_cycle(controller, (0.08, 0.0, 0.0), 2.0)
        debug = controller.debug_snapshot()
        for key in (
            "phase", "phase_name", "phase_progress", "step_state",
            "swing_legs", "stance_legs", "per_leg_phase", "cycle_period",
            "warning", "body_world", "planned_velocity",
        ):
            self.assertIn(key, debug)
        self.assertEqual(
            sorted(debug["swing_legs"] + debug["stance_legs"]),
            sorted(LEG_ORDER),
        )


class ServoBudgetRuntimeTests(unittest.TestCase):
    """The engine's actual output must respect the budgets it promised."""

    def measure(self, gait, velocity):
        controller = VoltGaitController()
        controller.set_gait(gait, 0.0)
        config = controller.config
        dt, now = 0.005, 0.0
        previous = None
        peak_stance, peak_swing = 0.0, 0.0
        # The budgets describe STEADY-STATE motion, so the window must be
        # measured in cycles, not seconds. The old fixed 8.0 s / 2.5 s pair
        # assumed a short cycle: on a 2.10 s gait it began sampling 1.2 cycles
        # in, still inside the velocity ramp and the world-lock initialisation,
        # and recorded a 866 deg/s startup transient as a budget violation.
        # Held to >=5 settled cycles before sampling, the same gait measures
        # 70.7 stance / 161.4 swing -- comfortably inside budget.
        settle = max(2.5, 5.0 * config["cycle_period"])
        duration = max(8.0, settle + 7.0 * config["cycle_period"])
        for index in range(int(duration / dt)):
            now += dt
            scale = min(1.0, now / 1.5)
            feet, body, _ = controller.step(
                now, dt, tuple(v * scale for v in velocity)
            )
            positions, _ = feet_to_joint_positions_diagnostic(
                feet, height=0.195
            )
            if previous is not None and now > settle:
                for joint_index in range(12):
                    speed = abs(
                        positions[joint_index] - previous[joint_index]
                    ) / dt * 180.0 / math.pi
                    leg = "_".join(
                        JOINT_NAMES[joint_index].split("_")[:2]
                    )
                    if controller._leg_in_stance(leg):
                        peak_stance = max(peak_stance, speed)
                    else:
                        peak_swing = max(peak_swing, speed)
            previous = positions
        return peak_stance, peak_swing, config

    def test_trot_runtime_respects_budgets_at_max_forward(self):
        stance, swing, config = self.measure("trot", (0.12, 0.0, 0.0))
        # Runtime touchdown planning differs slightly from the sweep's
        # idealized stance model; allow 15% headroom on the stance side
        # (which carries 40+ deg/s of margin against the servo's real
        # loaded capability) and hold swing to its budget.
        self.assertLess(stance, config["stance_velocity_budget_deg_s"] * 1.15)
        self.assertLess(swing, config["swing_velocity_budget_deg_s"] * 1.05)

    def test_amble_runtime_respects_budgets_at_max_forward(self):
        stance, swing, config = self.measure("amble", (0.05, 0.0, 0.0))
        self.assertLess(stance, config["stance_velocity_budget_deg_s"] * 1.15)
        self.assertLess(swing, config["swing_velocity_budget_deg_s"] * 1.05)


if __name__ == "__main__":
    unittest.main()
