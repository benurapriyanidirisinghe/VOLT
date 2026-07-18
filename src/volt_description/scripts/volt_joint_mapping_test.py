#!/usr/bin/env python3

import argparse
import math
import sys

import yaml

from volt_kinematics import JOINT_NAMES
from volt_servo_calibration import ServoCalibrationTable


def parse_args():
    parser = argparse.ArgumentParser(description="Dry-run VOLT joint to PCA mapping tests.")
    parser.add_argument("--calibration-file", required=True)
    parser.add_argument("--joint", choices=JOINT_NAMES, default=JOINT_NAMES[0])
    parser.add_argument("--physical", action="store_true", help="Print commands for physical test.")
    return parser.parse_args()


def frame_line(table, pose):
    frame, details = table.channel_frame_from_positions(pose)
    return "FRAME " + " ".join("%.2f" % value for value in frame), details


def main():
    args = parse_args()
    table = ServoCalibrationTable.from_file(args.calibration_file)
    servo = table.servos[args.joint]

    print("Joint: %s" % args.joint)
    print("PCA channel: %d" % servo.pca_channel)
    print("neutral_deg: %.2f" % servo.neutral_deg)
    print("direction: %d" % servo.direction)
    print("dry_run: %s" % (not args.physical))
    pose = {name: 0.0 for name in JOINT_NAMES}

    for label, rad in (("zero", 0.0), ("+0.15 rad", 0.15), ("zero", 0.0), ("-0.15 rad", -0.15), ("return zero", 0.0)):
        pose[args.joint] = rad
        line, details = frame_line(table, pose)
        selected = [item for item in details if item["joint"] == args.joint][0]
        print()
        print("%s: ros=%.3f rad servo=%.2f deg channel=%d%s" % (
            label,
            rad,
            selected["servo_deg"],
            selected["pca_channel"],
            " CLAMPED" if selected["clamped"] else "",
        ))
        print(line)
        if args.physical:
            answer = input("Send this complete pose using the calibration GUI/bridge? Type YES to continue: ")
            if answer != "YES":
                print("Stopped.")
                return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
