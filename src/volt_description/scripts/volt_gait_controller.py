#!/usr/bin/env python3

"""Stateful stance and swing gait generation for the VOLT quadruped.

Coordinate convention used by the gait planner:
- Body frame x points forward, y points left, z points up.
- Foot targets are stored in the body frame; z is negative because feet are
  below the body.
- Left and right legs are mirrored only through the y coordinate. The sagittal
  leg/foot IK signs stay the same for front and rear legs, so the knees fold in
  the same mechanical direction while the hip origins mirror across the body.
"""

import math

from volt_kinematics import LEG_ORDER, NOMINAL_FEET, clamp, smootherstep


def trot_config(
    step_length,
    step_height,
    gait_frequency,
    duty_factor,
    max_x,
    max_y,
    max_yaw,
    smoothing_factor,
):
    """Create a continuous diagonal-trot configuration.

    During stance, a planted foot travels opposite the commanded body velocity.
    The maximum stance travel is bounded by step_length. Swing then returns the
    foot smoothly to the next touchdown point.
    """
    return {
        "type": "phase_trot",
        "step_length": step_length,
        "lateral_step_length": min(0.050, step_length * 0.45),
        "step_height": step_height,
        "body_height": 0.200,
        "gait_frequency": gait_frequency,
        "period": 1.0 / gait_frequency,
        "duty_factor": duty_factor,
        "swing_time": (1.0 - duty_factor) / gait_frequency,
        "max_x": max_x,
        "max_y": max_y,
        "max_yaw": max_yaw,
        "max_step_x": step_length,
        "max_step_y": min(0.050, step_length * 0.45),
        "body_shift_x": 0.0,
        "body_shift_y": 0.0,
        "shift_time": 0.0,
        "smoothing_factor": smoothing_factor,
        "settle_time": 0.35,
    }


GAITS = {
    "walk": {
        "type": "legacy",
        "period": 1.20,
        "swing_time": 0.29,
        "clearance": 0.020,
        "max_x": 0.058,
        "max_y": 0.028,
        "max_yaw": 0.32,
        "max_step_x": 0.045,
        "max_step_y": 0.026,
        "body_shift_x": 0.004,
        "body_shift_y": 0.009,
        "shift_time": 0.08,
        "smoothing_factor": 0.45,
        # Static sequence: RF -> RL -> LF -> RR.
        "swing_starts": {
            "front_right": 0.04,
            "rear_left": 0.29,
            "front_left": 0.54,
            "rear_right": 0.79,
        },
        "crab_swing_starts": {
            "front_left": 0.03,
            "rear_right": 0.03,
            "front_right": 0.53,
            "rear_left": 0.53,
        },
    },
    "amble": {
        "type": "legacy",
        "period": 0.82,
        "swing_time": 0.30,
        "clearance": 0.022,
        "max_x": 0.090,
        "max_y": 0.035,
        "max_yaw": 0.50,
        "max_step_x": 0.043,
        "max_step_y": 0.026,
        "body_shift_x": 0.003,
        "body_shift_y": 0.006,
        "shift_time": 0.06,
        "smoothing_factor": 0.50,
        # Quasi-static sequence with brief adjacent swing overlap.
        "swing_starts": {
            "front_right": 0.03,
            "rear_left": 0.25,
            "front_left": 0.53,
            "rear_right": 0.75,
        },
        "crab_swing_starts": {
            "front_left": 0.03,
            "rear_right": 0.03,
            "front_right": 0.53,
            "rear_left": 0.53,
        },
    },
    # Smooth diagonal trot modes. Pair 1 is front_left + rear_right.
    # Pair 2 is front_right + rear_left. Pair 2 is always 180 deg out of phase.
    "slow_trot": trot_config(
        step_length=0.060,
        step_height=0.010,
        gait_frequency=1.00,
        duty_factor=0.62,
        max_x=0.058,
        max_y=0.025,
        max_yaw=0.40,
        smoothing_factor=0.55,
    ),
    "normal_trot": trot_config(
        step_length=0.075,
        step_height=0.010,
        gait_frequency=1.30,
        duty_factor=0.58,
        max_x=0.085,
        max_y=0.035,
        max_yaw=0.65,
        smoothing_factor=0.65,
    ),
    "fast_trot": trot_config(
        step_length=0.090,
        step_height=0.010,
        gait_frequency=1.50,
        duty_factor=0.55,
        max_x=0.105,
        max_y=0.045,
        max_yaw=0.75,
        smoothing_factor=0.75,
    ),
}

# Keep the original command name working. It now means the tuned normal trot.
GAITS["trot"] = dict(GAITS["normal_trot"])

TROT_PHASE_OFFSETS = {
    "front_left": 0.0,
    "rear_right": 0.0,
    "front_right": 0.5,
    "rear_left": 0.5,
}


def copy_feet(feet):
    return {leg: tuple(feet[leg]) for leg in LEG_ORDER}


def rotate_z(point, angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    x, y, z = point
    return cosine * x - sine * y, sine * x + cosine * y, z


def periodic_elapsed(phase, start, period):
    return (phase - start) % period


def smooth_bump(value):
    """Unit-height bump with zero velocity and acceleration at both ends."""
    value = clamp(value, 0.0, 1.0)
    return 64.0 * value ** 3 * (1.0 - value) ** 3


def quintic_hermite(start, end, start_velocity, end_velocity, duration, phase):
    """Interpolate position with continuous endpoint velocity/acceleration."""
    phase = clamp(phase, 0.0, 1.0)
    phase2 = phase * phase
    phase3 = phase2 * phase
    phase4 = phase3 * phase
    phase5 = phase4 * phase

    start_position = 1.0 - 10.0 * phase3 + 15.0 * phase4 - 6.0 * phase5
    start_tangent = phase - 6.0 * phase3 + 8.0 * phase4 - 3.0 * phase5
    end_position = 10.0 * phase3 - 15.0 * phase4 + 6.0 * phase5
    end_tangent = -4.0 * phase3 + 7.0 * phase4 - 3.0 * phase5
    return (
        start_position * start
        + start_tangent * duration * start_velocity
        + end_position * end
        + end_tangent * duration * end_velocity
    )


class VoltGaitController:
    """Generate foot targets for crawl/amble and smooth phase-based trots."""

    def __init__(self):
        self.gait_name = "walk"
        self.feet = copy_feet(NOMINAL_FEET)
        self.active = False
        self.settling = False
        self.settled_legs = set()
        self.start_time = None
        self.settle_start_time = None
        self.settle_origins = copy_feet(NOMINAL_FEET)
        self.crab_mode = False
        self.was_swinging = {leg: False for leg in LEG_ORDER}
        self.swing_origins = copy_feet(NOMINAL_FEET)
        self.swing_targets = copy_feet(NOMINAL_FEET)
        self.swing_start_velocities = {
            leg: (0.0, 0.0, 0.0) for leg in LEG_ORDER
        }
        self.swing_end_velocities = {
            leg: (0.0, 0.0, 0.0) for leg in LEG_ORDER
        }
        self.debug_state = {
            "phase": 0.0,
            "swing_legs": [],
            "stance_legs": list(LEG_ORDER),
            "feet": copy_feet(NOMINAL_FEET),
        }

    @property
    def config(self):
        return GAITS[self.gait_name]

    def is_phase_trot(self):
        return self.config.get("type") == "phase_trot"

    def reset(self, now=None):
        self.feet = copy_feet(NOMINAL_FEET)
        self.active = False
        self.settling = False
        self.settled_legs.clear()
        self.start_time = now
        self.settle_start_time = None
        self.settle_origins = copy_feet(NOMINAL_FEET)
        self.crab_mode = False
        self.was_swinging = {leg: False for leg in LEG_ORDER}
        self.swing_origins = copy_feet(NOMINAL_FEET)
        self.swing_targets = copy_feet(NOMINAL_FEET)
        self.swing_start_velocities = {
            leg: (0.0, 0.0, 0.0) for leg in LEG_ORDER
        }
        self.swing_end_velocities = {
            leg: (0.0, 0.0, 0.0) for leg in LEG_ORDER
        }
        self.update_debug(0.0, [], list(LEG_ORDER))

    def set_gait(self, gait_name, now):
        if gait_name not in GAITS:
            raise ValueError("Unknown gait: %s" % gait_name)
        self.gait_name = gait_name
        self.start_time = now
        self.settle_start_time = None
        self.crab_mode = False
        self.was_swinging = {leg: False for leg in LEG_ORDER}
        self.swing_origins = copy_feet(self.feet)
        self.swing_targets = copy_feet(self.feet)
        self.swing_start_velocities = {
            leg: (0.0, 0.0, 0.0) for leg in LEG_ORDER
        }
        self.swing_end_velocities = {
            leg: (0.0, 0.0, 0.0) for leg in LEG_ORDER
        }
        self.settling = False
        self.settled_legs.clear()

    def request_stop(self):
        if self.active:
            self.settling = True
            self.settled_legs.clear()
            self.settle_start_time = None
            self.settle_origins = copy_feet(self.feet)

    def phase(self, now):
        if self.start_time is None:
            self.start_time = now
        return (now - self.start_time) % self.config["period"]

    def cycle_phase(self, now):
        if self.start_time is None:
            self.start_time = now
        frequency = self.config["gait_frequency"]
        return ((now - self.start_time) * frequency) % 1.0

    def phase_starts(self):
        key = "crab_swing_starts" if self.crab_mode else "swing_starts"
        return self.config[key]

    def swing_progress(self, leg_name, phase):
        config = self.config
        start = self.phase_starts()[leg_name] * config["period"]
        elapsed = periodic_elapsed(phase, start, config["period"])
        if elapsed >= config["swing_time"]:
            return None
        return elapsed / config["swing_time"]

    def touchdown_target(self, leg_name, velocity):
        config = self.config
        nominal = NOMINAL_FEET[leg_name]
        vx, vy, yaw_rate = velocity
        stance_time = config["period"] - config["swing_time"]

        predicted = rotate_z(nominal, 0.5 * yaw_rate * stance_time)
        offset_x = predicted[0] - nominal[0] + 0.5 * vx * stance_time
        offset_y = predicted[1] - nominal[1] + 0.5 * vy * stance_time

        workspace_ratio = math.hypot(
            offset_x / config["max_step_x"],
            offset_y / config["max_step_y"],
        )
        if workspace_ratio > 1.0:
            offset_x /= workspace_ratio
            offset_y /= workspace_ratio

        return nominal[0] + offset_x, nominal[1] + offset_y, nominal[2]

    def stance_step(self, location, velocity, dt):
        vx, vy, yaw_rate = velocity
        rotated = rotate_z(location, -yaw_rate * dt)
        return rotated[0] - vx * dt, rotated[1] - vy * dt, NOMINAL_FEET_Z

    def stance_foot_velocity(self, location, velocity):
        """Instantaneous planted-foot velocity in the rotating body frame."""
        vx, vy, yaw_rate = velocity
        return (
            -vx + yaw_rate * location[1],
            -vy - yaw_rate * location[0],
            0.0,
        )

    def swing_step(self, leg_name, progress, velocity):
        origin = self.swing_origins[leg_name]
        target = self.touchdown_target(leg_name, velocity)
        blend = smootherstep(progress)

        x = origin[0] + (target[0] - origin[0]) * blend
        y = origin[1] + (target[1] - origin[1]) * blend
        ground_z = origin[2] + (target[2] - origin[2]) * blend

        lift_shape = 16.0 * progress ** 2 * (1.0 - progress) ** 2
        z = ground_z + self.config["clearance"] * lift_shape
        return x, y, z

    def support_shift(self, phase):
        config = self.config
        transition = config["shift_time"]
        if transition <= 0.0:
            return 0.0, 0.0

        shift_x = 0.0
        shift_y = 0.0
        total_weight = 0.0
        period = config["period"]
        swing_time = config["swing_time"]

        for leg_name in LEG_ORDER:
            start = self.phase_starts()[leg_name] * period
            elapsed = periodic_elapsed(phase, start, period)
            if elapsed < swing_time:
                weight = 1.0
            else:
                before = (start - phase) % period
                after = (phase - (start + swing_time)) % period
                if before < transition:
                    weight = smootherstep(1.0 - before / transition)
                elif after < transition:
                    weight = smootherstep(1.0 - after / transition)
                else:
                    weight = 0.0

            nominal = NOMINAL_FEET[leg_name]
            shift_x -= math.copysign(config["body_shift_x"] * weight, nominal[0])
            shift_y -= math.copysign(config["body_shift_y"] * weight, nominal[1])
            total_weight += weight

        if total_weight > 1.0:
            shift_x /= total_weight
            shift_y /= total_weight
        return shift_x, shift_y

    def begin_trot_swing(self, leg_name, velocity):
        """Freeze a swing target so command changes cannot jerk the foot."""
        origin = self.feet[leg_name]
        target = self.touchdown_target(leg_name, velocity)
        self.swing_origins[leg_name] = origin
        self.swing_targets[leg_name] = target
        self.swing_start_velocities[leg_name] = self.stance_foot_velocity(
            origin,
            velocity,
        )
        self.swing_end_velocities[leg_name] = self.stance_foot_velocity(
            target,
            velocity,
        )

    def phase_trot_swing_step(self, leg_name, swing):
        """Return a C2 swing path that matches stance speed at touchdown."""
        config = self.config
        origin = self.swing_origins[leg_name]
        target = self.swing_targets[leg_name]
        start_velocity = self.swing_start_velocities[leg_name]
        end_velocity = self.swing_end_velocities[leg_name]
        duration = config["swing_time"]

        x = quintic_hermite(
            origin[0],
            target[0],
            start_velocity[0],
            end_velocity[0],
            duration,
            swing,
        )
        y = quintic_hermite(
            origin[1],
            target[1],
            start_velocity[1],
            end_velocity[1],
            duration,
            swing,
        )
        ground_z = quintic_hermite(
            origin[2],
            target[2],
            0.0,
            0.0,
            duration,
            swing,
        )
        z = ground_z + config["step_height"] * smooth_bump(swing)
        return x, y, z

    def settle_feet(self, now):
        config = self.config
        if self.settle_start_time is None:
            self.settle_start_time = now
            self.settle_origins = copy_feet(self.feet)
        progress = (now - self.settle_start_time) / config["settle_time"]
        if progress >= 1.0:
            self.feet = copy_feet(NOMINAL_FEET)
            self.active = False
            self.settling = False
            self.start_time = now
            self.update_debug(0.0, [], list(LEG_ORDER))
            return copy_feet(self.feet), (0.0, 0.0), False

        blend = smootherstep(progress)
        for leg_name in LEG_ORDER:
            origin = self.settle_origins[leg_name]
            target = NOMINAL_FEET[leg_name]
            self.feet[leg_name] = tuple(
                origin[index] + (target[index] - origin[index]) * blend
                for index in range(3)
            )
        self.update_debug(self.cycle_phase(now), [], list(LEG_ORDER))
        return copy_feet(self.feet), (0.0, 0.0), True

    def phase_trot_step(self, now, dt, velocity, step_in_place=False):
        speed = math.hypot(velocity[0], velocity[1])
        commanded_motion = speed > 0.0015 or abs(velocity[2]) > 0.015

        if not self.active and (commanded_motion or step_in_place):
            self.active = True
            self.settling = False
            self.settle_start_time = None
            self.start_time = now

        if not self.active:
            self.update_debug(0.0, [], list(LEG_ORDER))
            return copy_feet(self.feet), (0.0, 0.0), False

        if commanded_motion or step_in_place:
            if self.settling:
                self.start_time = now
            self.settling = False
            self.settle_start_time = None
        elif not self.settling:
            self.settling = True
            self.settle_start_time = now
            self.settle_origins = copy_feet(self.feet)

        if self.settling:
            return self.settle_feet(now)

        cycle_phase = self.cycle_phase(now)
        duty = self.config["duty_factor"]
        swing_legs = []
        stance_legs = []
        for leg_name in LEG_ORDER:
            leg_phase = (
                cycle_phase + TROT_PHASE_OFFSETS[leg_name]
            ) % 1.0
            swinging = leg_phase >= duty

            if swinging:
                if not self.was_swinging[leg_name]:
                    self.begin_trot_swing(leg_name, velocity)
                swing = (leg_phase - duty) / (1.0 - duty)
                self.feet[leg_name] = self.phase_trot_swing_step(
                    leg_name,
                    swing,
                )
                swing_legs.append(leg_name)
            else:
                if self.was_swinging[leg_name]:
                    self.feet[leg_name] = self.swing_targets[leg_name]

                # A planted foot must move backward in the body frame at the
                # requested body velocity. Updating this every control tick
                # creates the forward ground reaction instead of holding a
                # static stance pose while the opposite pair swings.
                stance_velocity = velocity
                if (
                    step_in_place
                    and math.hypot(velocity[0], velocity[1]) < 0.002
                    and abs(velocity[2]) < 0.02
                ):
                    stance_velocity = (0.0, 0.0, 0.0)
                self.feet[leg_name] = self.stance_step(
                    self.feet[leg_name],
                    stance_velocity,
                    dt,
                )
                stance_legs.append(leg_name)
            self.was_swinging[leg_name] = swinging

        self.update_debug(cycle_phase, swing_legs, stance_legs)
        return copy_feet(self.feet), (0.0, 0.0), True

    def legacy_step(self, now, dt, velocity, step_in_place=False):
        speed = math.hypot(velocity[0], velocity[1])
        commanded_motion = speed > 0.0015 or abs(velocity[2]) > 0.015

        if not self.active and (commanded_motion or step_in_place):
            self.active = True
            self.settling = False
            self.settled_legs.clear()
            self.start_time = now
            self.crab_mode = abs(velocity[1]) > 0.002
            self.was_swinging = {leg: False for leg in LEG_ORDER}

        if not self.active:
            self.update_debug(0.0, [], list(LEG_ORDER))
            return copy_feet(self.feet), (0.0, 0.0), False

        if commanded_motion or step_in_place:
            self.settling = False
            self.settled_legs.clear()
        elif not self.settling:
            self.settling = True
            self.settled_legs.clear()

        phase = self.phase(now)
        near_zero = speed < 0.002 and abs(velocity[2]) < 0.02
        swing_legs = []
        stance_legs = []

        for leg_name in LEG_ORDER:
            progress = self.swing_progress(leg_name, phase)
            swinging = progress is not None

            if swinging:
                if not self.was_swinging[leg_name]:
                    self.swing_origins[leg_name] = self.feet[leg_name]
                swing_velocity = (0.0, 0.0, 0.0) if self.settling else velocity
                self.feet[leg_name] = self.swing_step(
                    leg_name,
                    progress,
                    swing_velocity,
                )
                swing_legs.append(leg_name)
            else:
                if self.was_swinging[leg_name]:
                    if self.settling and near_zero:
                        self.feet[leg_name] = NOMINAL_FEET[leg_name]
                        self.settled_legs.add(leg_name)
                    else:
                        self.feet[leg_name] = self.touchdown_target(
                            leg_name,
                            velocity,
                        )
                self.feet[leg_name] = self.stance_step(
                    self.feet[leg_name],
                    velocity,
                    dt,
                )
                stance_legs.append(leg_name)

            self.was_swinging[leg_name] = swinging

        self.update_debug(phase / self.config["period"], swing_legs, stance_legs)

        if self.settling and len(self.settled_legs) == len(LEG_ORDER):
            self.feet = copy_feet(NOMINAL_FEET)
            self.active = False
            self.settling = False
            self.start_time = now
            self.crab_mode = False
            self.update_debug(0.0, [], list(LEG_ORDER))
            return copy_feet(self.feet), (0.0, 0.0), False

        return copy_feet(self.feet), self.support_shift(phase), True

    def step(self, now, dt, velocity, step_in_place=False):
        if self.is_phase_trot():
            return self.phase_trot_step(now, dt, velocity, step_in_place)
        return self.legacy_step(now, dt, velocity, step_in_place)

    def update_debug(self, phase, swing_legs, stance_legs):
        self.debug_state = {
            "phase": phase % 1.0,
            "swing_legs": list(swing_legs),
            "stance_legs": list(stance_legs),
            "feet": copy_feet(self.feet),
        }

    def debug_snapshot(self):
        return dict(self.debug_state)


NOMINAL_FEET_Z = next(iter(NOMINAL_FEET.values()))[2]
