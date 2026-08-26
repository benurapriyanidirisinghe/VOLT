#!/usr/bin/env python3

"""Find the body_x offset that balances the robot's fore-aft load.

Symptom this exists for: during a trot the front feet clear the ground but the
rear feet do not (or the reverse). Commanded lift is identical on all four
legs, so a pair that fails to clear is carrying more weight than its stiffness
can hold up.

The robot's own compliance makes the direction non-obvious. Vertical foot droop
under load is ``dz = (F/k) * sum(arm_j^2)`` over the leg's three joints, and at
the shipped stance:

    front  sum(arm^2) = 0.009718     (knee moment arm 71.4 mm)
    rear   sum(arm^2) = 0.007009     (knee moment arm 48.8 mm)

so at EQUAL load the front sinks about 39% more than the rear. If the rear is
the pair that fails, it must be carrying materially more load -- which means
the centre of mass sits behind the geometric centre. That offset cannot be read
from the URDF: none of its ``<inertial>`` blocks carry an ``<origin>``, so the
model can only ever report a centred CoM.

``body_x`` moves the body forward over the world-locked feet, shifting static
load off the rear pair and onto the front. This tool ramps it while the robot
walks so the balance point is measured rather than guessed. Watch the feet that
were failing and note the value at which they start to clear.

Run it with the robot on a stand for a first pass, then on the floor. It only
changes body offset; it does not start or stop the gait.
"""

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

# Matches the bound the motion controller clamps to, and the range
# real_robot_profiles.yaml validates body_x against.
BODY_X_LIMIT = 0.025

# body_pose sets ALL SIX pose fields from one Twist, so every field must be
# populated deliberately. A default linear.z of 0.0 clamps to the controller's
# minimum body height of 0.175 m -- a 25 mm crouch, which increases knee torque
# and would make exactly the symptom being investigated worse.
DEFAULT_BODY_HEIGHT = 0.200


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Ramp body_x to find the fore-aft balance point."
    )
    parser.add_argument("--start", type=float, default=0.0,
                        help="First body_x in metres. Default 0.")
    parser.add_argument("--end", type=float, default=0.025,
                        help="Last body_x in metres. Positive moves the body "
                             "forward, unloading the rear. Default 0.025.")
    parser.add_argument("--step", type=float, default=0.005,
                        help="Increment in metres. Default 0.005.")
    parser.add_argument("--dwell", type=float, default=8.0,
                        help="Seconds held at each value. Default 8.")
    parser.add_argument("--body-height", type=float,
                        default=DEFAULT_BODY_HEIGHT,
                        help="Body height held throughout, in metres. "
                             "Default 0.200.")
    return parser.parse_args(argv)


class BalanceRamp(Node):
    def __init__(self, body_height):
        super().__init__("volt_body_balance_test")
        self.body_height = float(body_height)
        self.pose_publisher = self.create_publisher(
            Twist, "/volt/body_pose", 10
        )
        # body_pose is ignored unless MOTION owns the command path, so the
        # ownership is asserted alongside every pose sample.
        self.owner_publisher = self.create_publisher(
            String, "/volt/command_owner", 10
        )

    def hold(self, body_x, seconds):
        message = Twist()
        message.linear.x = float(body_x)
        message.linear.y = 0.0
        message.linear.z = self.body_height
        message.angular.x = 0.0
        message.angular.y = 0.0
        message.angular.z = 0.0
        owner = String()
        owner.data = "MOTION"
        deadline = time.time() + float(seconds)
        while time.time() < deadline:
            self.owner_publisher.publish(owner)
            self.pose_publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)


def ramp_values(start, end, step):
    if step <= 0.0:
        raise ValueError("--step must be positive")
    direction = 1.0 if end >= start else -1.0
    values = []
    value = start
    while (value - end) * direction <= 1e-9:
        values.append(round(value, 6))
        value += step * direction
    return values


def main(argv=None):
    args = parse_args(argv)
    try:
        values = ramp_values(args.start, args.end, args.step)
    except ValueError as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        return 2
    over = [v for v in values if abs(v) > BODY_X_LIMIT + 1e-9]
    if over:
        print("ERROR: %s m exceeds the body_x bound of +-%.3f m"
              % (over[0], BODY_X_LIMIT), file=sys.stderr)
        return 2

    rclpy.init()
    node = BalanceRamp(args.body_height)
    print("\n=== body_x ramp: %+.0f to %+.0f mm in %.0f mm steps, %.0fs each ==="
          % (args.start * 1000, args.end * 1000,
             args.step * 1000, args.dwell))
    print("body height held at %.0f mm." % (args.body_height * 1000))
    print("Positive body_x moves the body forward and unloads the rear pair.")
    print("Watch the feet that were failing to clear; note where they start.\n")
    try:
        for value in values:
            print(">>> body_x = %+.0f mm" % (value * 1000), flush=True)
            node.hold(value, args.dwell)
    except KeyboardInterrupt:
        print("\ninterrupted", flush=True)
    finally:
        print("\nreturning body_x to 0", flush=True)
        node.hold(0.0, 3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
