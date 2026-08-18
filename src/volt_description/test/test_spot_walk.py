#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from volt_gait_controller import (  # noqa: E402
    GAITS,
    SPOT_WALK_LEG_ORDER,
    VoltGaitController,
    support_clearance,
)
from volt_kinematics import (  # noqa: E402
    LEG_ORDER,
    NOMINAL_FEET,
    feet_to_joint_positions_diagnostic,
)


DT = 0.01


class SpotWalkTests(unittest.TestCase):
    def controller(self):
        controller = VoltGaitController()
        controller.set_gait("spot_walk", 0.0)
        return controller

    def step_samples(
        self,
        controller,
        duration,
        velocity,
        step_in_place=False,
        body_offset=(0.0, 0.0),
    ):
        samples = []
        count = int(math.ceil(duration / DT))
        start = getattr(controller, "_test_time", 0.0)
        for index in range(1, count + 1):
            now = start + index * DT
            feet, body, active = controller.step(
                now,
                DT,
                velocity,
                step_in_place,
                body_offset,
            )
            samples.append((
                now,
                feet,
                body,
                active,
                controller.debug_snapshot(),
            ))
        controller._test_time = start + count * DT
        return samples

    def test_zero_command_is_idle_and_step_in_place_is_explicit(self):
        controller = self.controller()
        idle = self.step_samples(controller, 2.0, (0.0, 0.0, 0.0))
        self.assertFalse(any(sample[3] for sample in idle))
        self.assertTrue(all(not sample[4]["swing_legs"] for sample in idle))

        stepping = self.step_samples(
            controller,
            GAITS["spot_walk"]["support_shift_duration"] + 0.25,
            (0.0, 0.0, 0.0),
            step_in_place=True,
        )
        self.assertTrue(any(sample[4]["swing_legs"] for sample in stepping))

    def test_upstream_leg_order_and_single_foot_support(self):
        controller = self.controller()
        velocity = (0.5 * GAITS["spot_walk"]["max_x"], 0.0, 0.0)
        samples = self.step_samples(
            controller,
            2.1 * GAITS["spot_walk"]["cycle_period"],
            velocity,
        )
        observed = []
        previous = ()
        for _now, _feet, _body, _active, debug in samples:
            swing = tuple(debug["swing_legs"])
            self.assertLessEqual(len(swing), 1)
            self.assertGreaterEqual(len(debug["stance_legs"]), 3)
            if swing and swing != previous:
                observed.append(swing[0])
            previous = swing
        self.assertGreaterEqual(len(observed), 8)
        self.assertEqual(
            observed[:8],
            list(SPOT_WALK_LEG_ORDER) * 2,
        )

    def test_complete_support_shift_precedes_every_liftoff(self):
        controller = self.controller()
        velocity = (0.4 * GAITS["spot_walk"]["max_x"], 0.0, 0.0)
        samples = self.step_samples(
            controller,
            GAITS["spot_walk"]["cycle_period"],
            velocity,
        )
        previous = None
        transitions = 0
        for _now, feet, _body, _active, debug in samples:
            if debug["phase_name"].startswith("shift_to_support"):
                self.assertEqual(debug["swing_legs"], [])
                for leg in LEG_ORDER:
                    self.assertAlmostEqual(
                        feet[leg][2],
                        NOMINAL_FEET[leg][2],
                        places=10,
                    )
            if (
                debug["swing_legs"]
                and previous is not None
                and not previous["swing_legs"]
            ):
                self.assertEqual(previous["phase_name"], "shift_to_support")
                self.assertGreaterEqual(previous["phase_progress"], 0.99)
                transitions += 1
            previous = debug
        self.assertEqual(transitions, 4)

    def test_body_projection_has_configured_support_margin_during_swing(self):
        controller = self.controller()
        config = GAITS["spot_walk"]
        velocity = (0.5 * config["max_x"], 0.0, 0.0)
        samples = self.step_samples(controller, config["cycle_period"], velocity)
        checked = 0
        for _now, _feet, _body, _active, debug in samples:
            if not debug["swing_legs"]:
                continue
            triangle = [
                _feet[leg][:2]
                for leg in debug["stance_legs"]
            ]
            shift = (
                debug["body_shift"]["x"],
                debug["body_shift"]["y"],
            )
            self.assertGreaterEqual(
                support_clearance(shift, triangle),
                config["support_margin"] - 2e-4,
            )
            checked += 1
        self.assertGreater(checked, 0)

    def test_swing_height_and_phase_boundaries_are_continuous(self):
        controller = self.controller()
        config = GAITS["spot_walk"]
        velocity = (0.4 * config["max_x"], 0.0, 0.0)
        samples = self.step_samples(controller, config["cycle_period"], velocity)
        maximum_lift = 0.0
        previous_feet = None
        for _now, feet, _body, _active, debug in samples:
            if debug["swing_legs"]:
                leg = debug["swing_legs"][0]
                maximum_lift = max(
                    maximum_lift,
                    feet[leg][2] - NOMINAL_FEET[leg][2],
                )
            if previous_feet is not None:
                largest_step = max(
                    math.dist(previous_feet[leg], feet[leg])
                    for leg in LEG_ORDER
                )
                self.assertLess(largest_step, 0.004)
            previous_feet = feet
        self.assertGreater(maximum_lift, 0.95 * config["step_height"])
        self.assertLessEqual(maximum_lift, config["step_height"] + 1e-9)

    def test_stop_finishes_airborne_foot_then_settles_nominal(self):
        controller = self.controller()
        config = GAITS["spot_walk"]
        velocity = (0.4 * config["max_x"], 0.0, 0.0)
        while True:
            sample = self.step_samples(controller, DT, velocity)[-1]
            debug = sample[4]
            if debug["swing_legs"] and 0.25 < debug["phase_progress"] < 0.55:
                airborne_leg = debug["swing_legs"][0]
                break

        controller.request_stop()
        touchdown_seen = False
        for sample in self.step_samples(
            controller,
            config["swing_duration"] + config["settle_duration"] + 0.5,
            (0.0, 0.0, 0.0),
        ):
            debug = sample[4]
            if debug["swing_legs"]:
                self.assertEqual(debug["swing_legs"], [airborne_leg])
            elif debug["phase_name"] == "settle_to_nominal":
                touchdown_seen = True
                self.assertAlmostEqual(
                    sample[1][airborne_leg][2],
                    NOMINAL_FEET[airborne_leg][2],
                    places=8,
                )
        self.assertTrue(touchdown_seen)
        self.assertFalse(controller.active)
        self.assertFalse(controller.settling)
        for leg in LEG_ORDER:
            self.assertLess(math.dist(controller.feet[leg], NOMINAL_FEET[leg]), 1e-9)

    def test_stop_during_shift_does_not_lift_a_new_foot(self):
        controller = self.controller()
        config = GAITS["spot_walk"]
        velocity = (0.4 * config["max_x"], 0.0, 0.0)
        self.step_samples(
            controller,
            0.35 * config["support_shift_duration"],
            velocity,
        )
        self.assertEqual(controller.debug_snapshot()["swing_legs"], [])
        controller.request_stop()
        samples = self.step_samples(
            controller,
            config["support_shift_duration"] + config["settle_duration"] + 0.2,
            (0.0, 0.0, 0.0),
        )
        self.assertTrue(all(not sample[4]["swing_legs"] for sample in samples))
        self.assertFalse(controller.active)

    def test_operator_body_offsets_are_included_in_support_projection(self):
        config = GAITS["spot_walk"]
        commands = (
            (
                (0.55 * config["max_x"], 0.45 * config["max_y"], 0.45 * config["max_yaw"]),
                (0.025, 0.020),
            ),
            (
                (-0.55 * config["max_x"], -0.45 * config["max_y"], -0.45 * config["max_yaw"]),
                (-0.025, -0.020),
            ),
        )
        for velocity, body_offset in commands:
            with self.subTest(velocity=velocity, body_offset=body_offset):
                controller = self.controller()
                samples = self.step_samples(
                    controller,
                    config["cycle_period"],
                    velocity,
                    body_offset=body_offset,
                )
                checked = 0
                for _now, feet, body, _active, debug in samples:
                    if not debug["swing_legs"]:
                        continue
                    triangle = [
                        feet[leg][:2]
                        for leg in debug["stance_legs"]
                    ]
                    actual_projection = (
                        body["body_x_override"] + body["x"],
                        body["body_y_override"] + body["y"],
                    )
                    self.assertAlmostEqual(
                        actual_projection[0],
                        debug["support_projection"]["x"],
                        places=10,
                    )
                    self.assertAlmostEqual(
                        actual_projection[1],
                        debug["support_projection"]["y"],
                        places=10,
                    )
                    self.assertGreaterEqual(
                        support_clearance(actual_projection, triangle),
                        config["support_margin"] - 1e-8,
                    )
                    checked += 1
                self.assertGreater(checked, 0)

    def test_ownership_hold_preserves_current_pose_and_clears_phase(self):
        controller = self.controller()
        config = GAITS["spot_walk"]
        velocity = (0.5 * config["max_x"], 0.0, 0.0)
        while True:
            sample = self.step_samples(controller, DT, velocity)[-1]
            if sample[4]["swing_legs"]:
                held_feet = sample[1]
                break

        controller.hold_current_feet(held_feet, sample[0])

        self.assertFalse(controller.active)
        self.assertFalse(controller.settling)
        self.assertEqual(controller.debug_snapshot()["phase_name"], "ownership_hold")
        for leg in LEG_ORDER:
            self.assertEqual(controller.feet[leg], held_feet[leg])

    def test_configured_yaw_extremes_stay_inside_ik_workspace(self):
        config = GAITS["spot_walk"]
        for yaw_rate in (-config["max_yaw"], config["max_yaw"]):
            with self.subTest(yaw_rate=yaw_rate):
                controller = self.controller()
                for index in range(
                    1,
                    3 * int(config["cycle_period"] / DT) + 1,
                ):
                    feet, body, _active = controller.step(
                        index * DT,
                        DT,
                        (0.0, 0.0, yaw_rate),
                    )
                    _joints, diagnostics = feet_to_joint_positions_diagnostic(
                        feet,
                        body_x=body["body_x_override"] + body["x"],
                        body_y=body["body_y_override"] + body["y"],
                    )
                    self.assertEqual(diagnostics["projected_targets"], [])


if __name__ == "__main__":
    unittest.main()
