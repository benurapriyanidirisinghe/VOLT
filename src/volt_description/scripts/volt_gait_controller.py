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
    stride_length,
    step_height,
    gait_frequency,
    swing_ratio,
    max_x,
    max_y,
    max_yaw,
    acceleration,
    smoothing_alpha=0.12,
):
    """Create a world-locked diagonal-trot configuration."""
    stance_ratio = 1.0 - swing_ratio
    max_speed = max(max_x, max_y)
    return {
        "type": "phase_trot",
        "strideLength": stride_length,
        "stepHeight": step_height,
        "bodyHeight": 0.200,
        "gaitFrequency": gait_frequency,
        "swingRatio": swing_ratio,
        "stanceRatio": stance_ratio,
        "smoothingAlpha": smoothing_alpha,
        "maxSpeed": max_speed,
        "acceleration": acceleration,
        "stride_length": stride_length,
        "lateral_stride_length": min(0.050, stride_length * 0.45),
        "step_height": step_height,
        "body_height": 0.200,
        "gait_frequency": gait_frequency,
        "period": 1.0 / gait_frequency,
        "swing_ratio": swing_ratio,
        "stance_ratio": stance_ratio,
        "swing_time": swing_ratio / gait_frequency,
        "stance_time": stance_ratio / gait_frequency,
        "max_x": max_x,
        "max_y": max_y,
        "max_yaw": max_yaw,
        "max_step_x": stride_length,
        "max_step_y": min(0.050, stride_length * 0.45),
        "max_speed": max_speed,
        "acceleration": acceleration,
        "body_shift_x": 0.0,
        "body_shift_y": 0.0,
        "shift_time": 0.0,
        "smoothing_factor": smoothing_alpha,
        "body_bob": 0.004,
        "body_roll": 0.018,
        "body_pitch": 0.020,
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
        stride_length=0.060,
        step_height=0.018,
        gait_frequency=1.00,
        swing_ratio=0.35,
        max_x=0.058,
        max_y=0.025,
        max_yaw=0.40,
        acceleration=0.18,
    ),
    "normal_trot": trot_config(
        stride_length=0.075,
        step_height=0.022,
        gait_frequency=1.30,
        swing_ratio=0.35,
        max_x=0.085,
        max_y=0.035,
        max_yaw=0.65,
        acceleration=0.24,
    ),
    "fast_trot": trot_config(
        stride_length=0.090,
        step_height=0.026,
        gait_frequency=1.50,
        swing_ratio=0.35,
        max_x=0.105,
        max_y=0.045,
        max_yaw=0.75,
        acceleration=0.30,
    ),
}

# Keep the common command names on the world-locked natural trot path. The GUI
# still publishes "walk" first, so make that button use the improved trot
# instead of the older body-frame foot animation.
GAITS["walk"] = dict(GAITS["slow_trot"])
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


def rotate_xy(x, y, angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return cosine * x - sine * y, sine * x + cosine * y


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
        self.body_x_world = 0.0
        self.body_y_world = 0.0
        self.body_yaw_world = 0.0
        self.world_feet = copy_feet(NOMINAL_FEET)
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
            "body_motion": {
                "height": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
            },
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
        self.body_x_world = 0.0
        self.body_y_world = 0.0
        self.body_yaw_world = 0.0
        self.world_feet = copy_feet(NOMINAL_FEET)
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

    def body_to_world(self, point, body_x=None, body_y=None, body_yaw=None):
        """Transform a body/local foot point into the gait world frame."""
        if body_x is None:
            body_x = self.body_x_world
        if body_y is None:
            body_y = self.body_y_world
        if body_yaw is None:
            body_yaw = self.body_yaw_world
        x, y, z = point
        world_x, world_y = rotate_xy(x, y, body_yaw)
        return body_x + world_x, body_y + world_y, z

    def world_to_body(self, point):
        """Convert a locked world foothold into the current body/local frame."""
        x, y, z = point
        local_x, local_y = rotate_xy(
            x - self.body_x_world,
            y - self.body_y_world,
            -self.body_yaw_world,
        )
        return local_x, local_y, z

    def sync_world_feet_from_body(self):
        self.world_feet = {
            leg: self.body_to_world(self.feet[leg])
            for leg in LEG_ORDER
        }

    def integrate_body_pose(self, velocity, dt):
        """Move the body through the world while stance feet remain planted."""
        vx, vy, yaw_rate = velocity
        world_vx, world_vy = rotate_xy(vx, vy, self.body_yaw_world)
        self.body_x_world += world_vx * dt
        self.body_y_world += world_vy * dt
        self.body_yaw_world += yaw_rate * dt

    def body_motion(self, phase, velocity):
        """Small natural body bob and attitude compensation for trot support."""
        config = self.config
        speed_scale = clamp(
            math.hypot(velocity[0], velocity[1]) / max(config["max_speed"], 1e-6),
            0.0,
            1.0,
        )
        yaw_scale = clamp(abs(velocity[2]) / max(config["max_yaw"], 1e-6), 0.0, 1.0)
        activity = max(speed_scale, yaw_scale)
        diagonal = math.sin(2.0 * math.pi * phase)
        double_frequency = 0.5 - 0.5 * math.cos(4.0 * math.pi * phase)
        return {
            "height": config["body_bob"] * double_frequency * activity,
            "roll": config["body_roll"] * diagonal * activity,
            "pitch": -config["body_pitch"]
            * math.cos(2.0 * math.pi * phase)
            * speed_scale,
        }

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

    def trot_touchdown_world(self, leg_name, velocity):
        """Pick the next world foothold ahead of the body at swing touchdown."""
        config = self.config
        nominal = NOMINAL_FEET[leg_name]
        vx, vy, yaw_rate = velocity
        swing_time = config["swing_time"]
        stance_time = config["stance_time"]

        lead_x = clamp(
            0.5 * vx * stance_time,
            -0.5 * config["stride_length"],
            0.5 * config["stride_length"],
        )
        lead_y = clamp(
            0.5 * vy * stance_time,
            -0.5 * config["lateral_stride_length"],
            0.5 * config["lateral_stride_length"],
        )
        touchdown_local = (
            nominal[0] + lead_x,
            nominal[1] + lead_y,
            nominal[2],
        )

        future_vx, future_vy = rotate_xy(vx, vy, self.body_yaw_world)
        future_x = self.body_x_world + future_vx * swing_time
        future_y = self.body_y_world + future_vy * swing_time
        future_yaw = (
            self.body_yaw_world
            + yaw_rate * (swing_time + 0.5 * stance_time)
        )
        return self.body_to_world(
            touchdown_local,
            future_x,
            future_y,
            future_yaw,
        )

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
        """Return a smooth swing arc between old and new world footholds."""
        config = self.config
        origin = self.swing_origins[leg_name]
        target = self.swing_targets[leg_name]

        # Swing phase uses a normalized 0..1 blend. X/Y move from the old
        # foothold to the next foothold, while Z follows a sinusoidal lift.
        blend = smootherstep(swing)
        x = origin[0] + (target[0] - origin[0]) * blend
        y = origin[1] + (target[1] - origin[1]) * blend
        ground_z = origin[2] + (target[2] - origin[2]) * blend
        z = ground_z + config["step_height"] * math.sin(math.pi * swing)
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
            self.sync_world_feet_from_body()

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

        self.integrate_body_pose(velocity, dt)
        cycle_phase = self.cycle_phase(now)
        swing_ratio = self.config["swing_ratio"]
        swing_legs = []
        stance_legs = []
        for leg_name in LEG_ORDER:
            # Pair A (front_left + rear_right) uses phase offset 0.0. Pair B
            # (front_right + rear_left) uses 0.5, exactly 180 degrees later.
            leg_phase = (
                cycle_phase + TROT_PHASE_OFFSETS[leg_name]
            ) % 1.0
            swinging = leg_phase < swing_ratio

            if swinging:
                if not self.was_swinging[leg_name]:
                    self.swing_origins[leg_name] = self.world_feet[leg_name]
                    self.swing_targets[leg_name] = self.trot_touchdown_world(
                        leg_name,
                        velocity,
                    )
                swing = leg_phase / swing_ratio
                self.world_feet[leg_name] = self.phase_trot_swing_step(
                    leg_name,
                    swing,
                )
                swing_legs.append(leg_name)
            else:
                if self.was_swinging[leg_name]:
                    self.world_feet[leg_name] = self.swing_targets[leg_name]

                # Stance phase foot locking: do not animate the planted foot in
                # the body frame. Keep its world coordinate fixed; as the body
                # integrates forward, world_to_body() makes the body move over
                # the planted support point without visible ground sliding.
                stance_legs.append(leg_name)
            self.was_swinging[leg_name] = swinging

        self.feet = {
            leg: self.world_to_body(self.world_feet[leg])
            for leg in LEG_ORDER
        }
        body_motion = self.body_motion(cycle_phase, velocity)
        self.update_debug(cycle_phase, swing_legs, stance_legs, body_motion)
        return copy_feet(self.feet), body_motion, True

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

    def update_debug(self, phase, swing_legs, stance_legs, body_motion=None):
        if body_motion is None:
            body_motion = {
                "height": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
            }
        self.debug_state = {
            "phase": phase % 1.0,
            "swing_legs": list(swing_legs),
            "stance_legs": list(stance_legs),
            "feet": copy_feet(self.feet),
            "body_motion": dict(body_motion),
        }

    def debug_snapshot(self):
        return dict(self.debug_state)


NOMINAL_FEET_Z = next(iter(NOMINAL_FEET.values()))[2]
