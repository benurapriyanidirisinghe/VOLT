#!/usr/bin/env python3

"""Start VOLT Ignition sim, Arduino bridge, and control GUI together."""

import argparse
import fcntl
import glob
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


CONFLICTING_ROS_NODES = {
    "/clock_bridge",
    "/controller_manager",
    "/gz_ros2_control",
    "/robot_state_publisher",
    "/volt_control_gui",
    "/volt_joint_command_router",
    "/volt_motion_controller",
    "/volt_serial_bridge",
}


class PreflightError(RuntimeError):
    """Raised when the runner cannot safely verify an unused ROS graph."""


def find_serial_port():
    ports = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    if ports:
        return ports[0]
    return "/dev/ttyUSB0"


def start_process(name, command, environment=None):
    print("\n[%s] %s" % (name, " ".join(command)), flush=True)
    return subprocess.Popen(
        command,
        stdin=subprocess.DEVNULL,
        preexec_fn=os.setsid,
        env=environment,
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


def active_ros_nodes(timeout=4.0):
    """Return live ROS graph node names without trusting cached daemon state."""
    try:
        result = subprocess.run(
            [
                "ros2",
                "node",
                "list",
                "--no-daemon",
                "--spin-time",
                "1",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise PreflightError(
            "could not inspect the live ROS graph: %s" % exc
        ) from exc

    if result.returncode != 0:
        detail = result.stderr.strip() or "ros2 node list failed"
        raise PreflightError(
            "could not inspect the live ROS graph: %s" % detail
        )
    return {
        line.strip()
        for line in result.stdout.splitlines()
        if line.strip().startswith("/")
    }


def process_arguments():
    """Read process argv arrays for a ROS-independent duplicate check."""
    argument_lists = []
    for command_path in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            raw = Path(command_path).read_bytes()
        except (OSError, PermissionError):
            continue
        arguments = [
            token.decode("utf-8", errors="replace")
            for token in raw.split(b"\0")
            if token
        ]
        if arguments:
            argument_lists.append(arguments)
    return argument_lists


def conflicting_processes(argument_lists=None):
    if argument_lists is None:
        argument_lists = process_arguments()

    conflicts = set()
    for arguments in argument_lists:
        executable_arguments = arguments[:1]
        if (
            arguments
            and Path(arguments[0]).name.startswith("python")
            and len(arguments) > 1
        ):
            executable_arguments.append(arguments[1])
        executable_basenames = {
            Path(argument).name for argument in executable_arguments
        }
        if "volt_motion_controller.py" in executable_basenames:
            conflicts.add("volt_motion_controller.py")
        if "volt_joint_command_router.py" in executable_basenames:
            conflicts.add("volt_joint_command_router.py")
        if (
            Path(arguments[0]).name == "ign"
            and "gazebo" in arguments
            and "-g" not in arguments
            and ("-r" in arguments or "-s" in arguments)
        ):
            conflicts.add("Ignition Gazebo server")
    return sorted(conflicts)


def existing_stack_conflicts():
    nodes = sorted(active_ros_nodes().intersection(CONFLICTING_ROS_NODES))
    return nodes, conflicting_processes()


def runner_lock_path():
    runtime_root = os.environ.get("XDG_RUNTIME_DIR", "").strip()
    if not runtime_root or not Path(runtime_root).is_dir():
        runtime_root = "/tmp"
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    safe_domain = "".join(
        character if character.isalnum() or character in ("-", "_") else "_"
        for character in domain
    )
    return Path(runtime_root) / (
        "volt-run-all-%d-domain-%s.lock" % (os.getuid(), safe_domain)
    )


def acquire_runner_lock():
    lock_handle = runner_lock_path().open("a+", encoding="utf-8")
    try:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_handle.close()
        raise
    lock_handle.seek(0)
    lock_handle.truncate()
    lock_handle.write("%d\n" % os.getpid())
    lock_handle.flush()
    return lock_handle


def prime_profile():
    try:
        result = subprocess.run(
            ["prime-select", "query"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=2.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return ""
    if result.returncode != 0:
        return ""
    return result.stdout.strip().lower()


def nvidia_module_loaded():
    return Path("/sys/module/nvidia").is_dir()


def intel_drm_available():
    for vendor_path in glob.glob("/sys/class/drm/card*/device/vendor"):
        try:
            vendor = Path(vendor_path).read_text(encoding="utf-8").strip().lower()
        except (OSError, UnicodeError):
            continue
        if vendor == "0x8086":
            return True
    return False


def select_gazebo_renderer(requested):
    if requested != "auto":
        return requested
    if (
        prime_profile() == "nvidia"
        and not nvidia_module_loaded()
        and intel_drm_available()
    ):
        return "intel-mesa"
    return "default"


def gazebo_client_environment(renderer, base_environment=None):
    environment = dict(
        os.environ if base_environment is None else base_environment
    )
    if renderer != "intel-mesa":
        return environment

    for name in (
        "LIBGL_ALWAYS_SOFTWARE",
        "__NV_PRIME_RENDER_OFFLOAD",
        "__VK_LAYER_NV_optimus",
    ):
        environment.pop(name, None)
    environment.update({
        "QT_QPA_PLATFORM": "xcb",
        "QT_XCB_GL_INTEGRATION": "xcb_glx",
        "__GLX_VENDOR_LIBRARY_NAME": "mesa",
        "DRI_PRIME": "0",
        "MESA_LOADER_DRIVER_OVERRIDE": "iris",
    })
    return environment


def gazebo_client_command(renderer):
    command = ["ign", "gazebo", "-g", "-v", "2"]
    if renderer == "intel-mesa":
        command.extend(["--render-engine-gui", "ogre"])
    command.extend(["--force-version", "6"])
    return command


def gazebo_resource_root():
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "--share", "volt_description"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return ""
    if result.returncode != 0:
        return ""
    share_path = result.stdout.strip()
    if not share_path:
        return ""
    return str(Path(share_path).parent)


def add_gazebo_resource_paths(environment, resource_root):
    result = dict(environment)
    if not resource_root:
        return result
    for variable in ("IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH"):
        existing = result.get(variable, "")
        entries = [entry for entry in existing.split(os.pathsep) if entry]
        if resource_root not in entries:
            entries.insert(0, resource_root)
        result[variable] = os.pathsep.join(entries)
    return result


def volt_start_command(args, serial_port):
    # The server is deliberately headless. GUI clients are sibling processes,
    # so a display-driver failure cannot tear down simulation.
    command = [
        "ros2",
        "launch",
        "volt_description",
        "volt_start.launch.py",
        "gazebo_gui:=false",
        "gui:=false",
        "start_serial_bridge:=true",
        "serial_port:=%s" % serial_port,
        "baud_rate:=%s" % args.baud_rate,
        "auto_ready_pose:=%s" % args.auto_ready_pose,
        "auto_arm:=%s" % args.arm_hardware,
        "use_hardware:=%s" % args.use_hardware,
        "dry_run:=%s" % args.dry_run,
        "enable_physical_tests:=%s" % (
            "true" if getattr(args, "physical", False) else "false"
        ),
        # Real gait deadlines must never pause with Ignition.  The physical
        # shadow remains visualization-only and the open-loop hardware
        # controller uses system time.
        "use_sim_time:=%s" % (
            "false" if getattr(args, "physical", False) else "true"
        ),
        "actuator_profile:=%s" % getattr(
            args,
            "actuator_profile",
            "simulation",
        ),
        "joint_rate_diagnostic:=%s" % (
            "true" if getattr(args, "joint_rate_diagnostic", False) else "false"
        ),
    ]
    # ros2 launch rejects "name:=" with an empty value, so only pass the CSV
    # path when one was actually requested.
    rate_output = str(getattr(args, "joint_rate_output", "") or "").strip()
    if rate_output:
        command.append("joint_rate_diagnostic_output:=%s" % rate_output)
    return command


def apply_run_mode(args):
    """Expand the simple physical preset without enabling automatic ARM."""
    if not getattr(args, "physical", False):
        return args
    args.gazebo_gui = "true"
    args.no_control_gui = False
    args.auto_ready_pose = "false"
    args.arm_hardware = "false"
    args.use_hardware = "true"
    args.dry_run = "false"
    return args


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Run VOLT Ignition sim, Arduino serial bridge, and control GUI."
    )
    parser.add_argument(
        "--physical",
        action="store_true",
        help=(
            "One-command physical mode: start Ignition, both GUIs, and the live "
            "Arduino bridge. Automatic ARM remains disabled; use the guided "
            "ARM SYSTEM SAFELY button after supporting the robot."
        ),
    )
    parser.add_argument(
        "--serial-port",
        default=None,
        help="Arduino serial device. Default: first /dev/ttyUSB* or /dev/ttyACM*.",
    )
    parser.add_argument(
        "--baud-rate",
        default="250000",
        help="Arduino firmware baud rate. Default: 57600.",
    )
    parser.add_argument(
        "--gazebo-gui",
        choices=["true", "false"],
        default="true",
        help="Start the Gazebo GUI. Default: true.",
    )
    parser.add_argument(
        "--gazebo-renderer",
        choices=["auto", "default", "intel-mesa"],
        default="auto",
        help=(
            "Gazebo GUI renderer. Auto detects this machine's unavailable "
            "NVIDIA PRIME module and uses Intel Mesa when required."
        ),
    )
    parser.add_argument(
        "--actuator-profile",
        choices=["simulation", "td8130mg"],
        default="simulation",
        help=(
            "Gazebo joint dynamics profile. Default keeps the proven simulation; "
            "td8130mg matches the firmware's 120 deg/s slew more closely."
        ),
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
        help=(
            "Advanced legacy auto-ARM option. Default: false. The --physical "
            "preset always ignores this and requires the guided GUI button."
        ),
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
        "--joint-rate-diagnostic",
        action="store_true",
        help=(
            "Record commanded deg/s per channel to tune gait limits against "
            "the firmware slew ceiling."
        ),
    )
    parser.add_argument(
        "--joint-rate-output",
        default="",
        help="CSV path for --joint-rate-diagnostic.",
    )
    parser.add_argument(
        "--hardware-delay",
        type=float,
        default=0.0,
        help="Deprecated compatibility option; unified launch handles ordering.",
    )
    parser.add_argument(
        "--gui-delay",
        type=float,
        default=0.0,
        help="Deprecated compatibility option; unified launch handles ordering.",
    )
    return parser.parse_args(argv)


def main():
    args = apply_run_mode(parse_args())
    serial_port = args.serial_port or find_serial_port()
    processes = []

    try:
        lock_handle = acquire_runner_lock()
    except BlockingIOError:
        print(
            "ERROR: another volt_run_all.py instance is already running.",
            file=sys.stderr,
            flush=True,
        )
        return 2

    try:
        try:
            nodes, process_conflicts = existing_stack_conflicts()
        except PreflightError as exc:
            print(
                "ERROR: safe startup preflight failed: %s" % exc,
                file=sys.stderr,
                flush=True,
            )
            print(
                "No process was started. Check ROS discovery and retry.",
                file=sys.stderr,
                flush=True,
            )
            return 2
        if nodes or process_conflicts:
            print(
                "ERROR: an existing VOLT / Ignition stack is already running.",
                file=sys.stderr,
                flush=True,
            )
            if nodes:
                print(
                    "ROS nodes: %s" % ", ".join(nodes),
                    file=sys.stderr,
                    flush=True,
                )
            if process_conflicts:
                print(
                    "Processes: %s" % ", ".join(process_conflicts),
                    file=sys.stderr,
                    flush=True,
                )
            print(
                "Use the existing windows, or stop the original runner with "
                "Ctrl+C before starting another one.",
                file=sys.stderr,
                flush=True,
            )
            return 2

        renderer = select_gazebo_renderer(args.gazebo_renderer)
        print("VOLT runner starting.", flush=True)
        if args.physical:
            print(
                "Mode: PHYSICAL ROBOT + IGNITION SHADOW (manual guided ARM)",
                flush=True,
            )
            print(
                "Automatic ARM is OFF. Support the robot, clear every leg, then "
                "press ARM SYSTEM SAFELY in the VOLT GUI.",
                flush=True,
            )
        else:
            print("Mode: SAFE SIMULATION / SERIAL DRY-RUN", flush=True)
        print("Arduino serial port: %s" % serial_port, flush=True)
        if args.gazebo_gui == "true":
            print("Ignition GUI renderer: %s" % renderer, flush=True)
        print("Press Ctrl+C to stop everything.", flush=True)

        try:
            processes.append((
                "volt_start",
                start_process(
                    "volt_start",
                    volt_start_command(args, serial_port),
                ),
                True,
            ))

            if not args.no_control_gui:
                processes.append((
                    "control_gui",
                    start_process(
                        "control_gui",
                        [
                            "ros2",
                            "run",
                            "volt_description",
                            "volt_control_gui.py",
                        ],
                    ),
                    False,
                ))

            if args.gazebo_gui == "true":
                gui_environment = gazebo_client_environment(renderer)
                gui_environment = add_gazebo_resource_paths(
                    gui_environment,
                    gazebo_resource_root(),
                )
                processes.append((
                    "ignition_gui",
                    start_process(
                        "ignition_gui",
                        gazebo_client_command(renderer),
                        environment=gui_environment,
                    ),
                    False,
                ))

            while True:
                for item in list(processes):
                    name, process, required = item
                    return_code = process.poll()
                    if return_code is None:
                        continue
                    processes.remove(item)
                    if required:
                        raise RuntimeError(
                            "%s terminated unexpectedly with code %s; "
                            "inspect the preceding launch error"
                            % (name, return_code)
                        )
                    print(
                        "[%s] exited with code %s; simulation remains active."
                        % (name, return_code),
                        flush=True,
                    )
                time.sleep(1.0)

        except KeyboardInterrupt:
            print("\nCtrl+C received.", flush=True)
        except RuntimeError as exc:
            print("\nERROR: %s" % exc, file=sys.stderr, flush=True)
            return 1
        finally:
            for name, process, _required in reversed(processes):
                stop_process(name, process)
    finally:
        lock_handle.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
