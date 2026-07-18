#!/usr/bin/env python3

"""Start VOLT Ignition sim, Arduino bridge, and control GUI together."""

import argparse
import glob
import os
import signal
import subprocess
import sys
import time


def find_serial_port():
    ports = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    if ports:
        return ports[0]
    return "/dev/ttyUSB0"


def start_process(name, command):
    print("\n[%s] %s" % (name, " ".join(command)), flush=True)
    return subprocess.Popen(
        command,
        stdin=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )


def stop_process(name, process, timeout=8.0):
    if process.poll() is not None:
        return

    print("[%s] stopping..." % name, flush=True)
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=timeout)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass

    if process.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=3.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run VOLT Ignition sim, Arduino serial bridge, and control GUI."
    )
    parser.add_argument(
        "--serial-port",
        default=None,
        help="Arduino serial device. Default: first /dev/ttyUSB* or /dev/ttyACM*.",
    )
    parser.add_argument(
        "--baud-rate",
        default="115200",
        help="Arduino firmware baud rate. Default: 115200.",
    )
    parser.add_argument(
        "--gazebo-gui",
        choices=["true", "false"],
        default="true",
        help="Start the Gazebo GUI. Default: true.",
    )
    parser.add_argument(
        "--no-control-gui",
        action="store_true",
        help="Start sim and Arduino bridge without the PyQt control GUI.",
    )
    parser.add_argument(
        "--auto-ready-pose",
        choices=["true", "false"],
        default="false",
        help="Automatically move from loaded zero pose to walk-ready pose. Default: false.",
    )
    parser.add_argument(
        "--arm-hardware",
        choices=["true", "false"],
        default="false",
        help="Send ARM to Arduino after connecting. Default: false.",
    )
    parser.add_argument(
        "--use-hardware",
        choices=["true", "false"],
        default="false",
        help="Allow serial bridge to open Arduino serial port. Default: false.",
    )
    parser.add_argument(
        "--dry-run",
        choices=["true", "false"],
        default="true",
        help="Log outgoing FRAME packets instead of writing serial. Default: true.",
    )
    parser.add_argument(
        "--hardware-delay",
        type=float,
        default=12.0,
        help="Seconds to wait before starting hardware bridge. Default: 12.",
    )
    parser.add_argument(
        "--gui-delay",
        type=float,
        default=2.0,
        help="Seconds to wait after hardware bridge before starting GUI. Default: 2.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    serial_port = args.serial_port or find_serial_port()
    processes = []

    print("VOLT runner starting.", flush=True)
    print("Arduino serial port: %s" % serial_port, flush=True)
    print("Press Ctrl+C to stop everything.", flush=True)

    try:
        processes.append((
            "ignition",
            start_process(
                "ignition",
                [
                    "ros2",
                    "launch",
                    "volt_description",
                    "ignition.launch.py",
                    "gui:=%s" % args.gazebo_gui,
                ],
            ),
        ))

        time.sleep(max(0.0, args.hardware_delay))

        processes.append((
            "hardware",
            start_process(
                "hardware",
                [
                    "ros2",
                    "launch",
                    "volt_description",
                    "hardware_control.launch.py",
                    "serial_port:=%s" % serial_port,
                    "baud_rate:=%s" % args.baud_rate,
                    "auto_ready_pose:=%s" % args.auto_ready_pose,
                    "auto_arm:=%s" % args.arm_hardware,
                    "hardware_enabled:=%s" % args.use_hardware,
                    "dry_run:=%s" % args.dry_run,
                ],
            ),
        ))

        if not args.no_control_gui:
            time.sleep(max(0.0, args.gui_delay))
            processes.append((
                "control_gui",
                start_process(
                    "control_gui",
                    ["ros2", "run", "volt_description", "volt_control_gui.py"],
                ),
            ))

        while True:
            for name, process in processes:
                return_code = process.poll()
                if return_code is not None:
                    raise RuntimeError("%s exited with code %s" % (name, return_code))
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nCtrl+C received.", flush=True)
    except RuntimeError as exc:
        print("\nERROR: %s" % exc, file=sys.stderr, flush=True)
        return 1
    finally:
        for name, process in reversed(processes):
            stop_process(name, process)

    return 0


if __name__ == "__main__":
    sys.exit(main())
