#!/usr/bin/env python3

"""Static launch/configuration safety tests with no ROS runtime dependency."""

import ast
import os
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch"
def source(path):
    return path.read_text(encoding="utf-8")


def call_name(node):
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return ""


def launch_argument_defaults(path):
    """Return literal defaults from DeclareLaunchArgument calls."""
    defaults = {}
    tree = ast.parse(source(path), filename=str(path))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if call_name(node.func) != "DeclareLaunchArgument":
            continue

        name_node = node.args[0] if node.args else None
        for keyword in node.keywords:
            if keyword.arg == "name":
                name_node = keyword.value
        if name_node is None:
            continue
        try:
            name = ast.literal_eval(name_node)
        except (ValueError, TypeError):
            continue

        default_node = None
        for keyword in node.keywords:
            if keyword.arg == "default_value":
                default_node = keyword.value
                break
        if default_node is None:
            defaults[name] = None
            continue
        try:
            defaults[name] = ast.literal_eval(default_node)
        except (ValueError, TypeError):
            defaults[name] = ast.unparse(default_node)
    return defaults


def node_executables(path):
    """Return literal executable values from launch_ros Node calls."""
    executables = []
    tree = ast.parse(source(path), filename=str(path))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or call_name(node.func) != "Node":
            continue
        for keyword in node.keywords:
            if keyword.arg != "executable":
                continue
            try:
                executables.append(ast.literal_eval(keyword.value))
            except (ValueError, TypeError):
                pass
    return executables


class SimulationTimeConfigurationTests(unittest.TestCase):
    def test_shared_gait_yaml_does_not_force_simulation_time(self):
        config = source(ROOT / "config" / "gait_controller.yaml")
        self.assertIsNone(
            re.search(
                r"^\s*use_sim_time\s*:",
                config,
                flags=re.MULTILINE,
            )
        )

    def test_simulation_launches_default_to_simulation_time(self):
        for filename in ("ignition.launch.py", "control.launch.py"):
            path = LAUNCH / filename
            with self.subTest(launch=filename):
                defaults = launch_argument_defaults(path)
                self.assertEqual(
                    str(defaults.get("use_sim_time")).lower(),
                    "true",
                )
                self.assertGreaterEqual(
                    len(
                        re.findall(
                            r"""['"]use_sim_time['"]""",
                            source(path),
                        )
                    ),
                    3,
                    "use_sim_time must be passed to launched ROS nodes",
                )

    def test_hardware_launch_defaults_to_system_time(self):
        path = LAUNCH / "hardware_control.launch.py"
        defaults = launch_argument_defaults(path)
        self.assertEqual(
            str(defaults.get("use_sim_time")).lower(),
            "false",
        )
        self.assertIn(
            '"open_loop_hardware": True',
            source(path),
        )
        self.assertNotIn(
            '"open_loop_hardware": True',
            source(LAUNCH / "control.launch.py"),
        )
        self.assertIn(
            '"open_loop_hardware": ParameterValue(',
            source(LAUNCH / "control.launch.py"),
        )

    def test_combined_hardware_launch_uses_one_open_loop_controller(self):
        text = source(LAUNCH / "volt_start.launch.py")
        self.assertIn(
            '"open_loop_hardware": effective_hardware_enabled',
            text,
        )
        control = source(LAUNCH / "control.launch.py")
        self.assertIn(
            "\"False if '\",",
            control,
        )
        self.assertIn(
            "hardware_mode,",
            control,
        )

    def test_combined_launch_defaults_to_simulation_time(self):
        path = LAUNCH / "volt_start.launch.py"
        defaults = launch_argument_defaults(path)
        self.assertEqual(
            str(defaults.get("use_sim_time")).lower(),
            "true",
        )
        self.assertGreaterEqual(
            len(
                re.findall(
                    r"""['"]use_sim_time['"]""",
                    source(path),
                )
            ),
            4,
            "combined launch must forward use_sim_time to its includes",
        )


class LaunchCompositionTests(unittest.TestCase):
    def test_hardware_defaults_are_non_actuating(self):
        defaults = launch_argument_defaults(
            LAUNCH / "hardware_control.launch.py"
        )
        expected = {
            "gui": "false",
            "auto_ready_pose": "false",
            "auto_arm": "false",
            "dry_run": "true",
            "hardware_enabled": "false",
            "use_sim_time": "false",
        }
        for name, expected_value in expected.items():
            with self.subTest(argument=name):
                self.assertEqual(
                    str(defaults.get(name)).lower(),
                    expected_value,
                )

    def test_combined_launch_serial_defaults_are_safe(self):
        defaults = launch_argument_defaults(
            LAUNCH / "volt_start.launch.py"
        )
        expected = {
            "auto_ready_pose": "false",
            "start_serial_bridge": "false",
            "use_hardware": "false",
            "hardware_enabled": "false",
            "dry_run": "true",
            "auto_arm": "false",
        }
        for name, expected_value in expected.items():
            with self.subTest(argument=name):
                self.assertEqual(
                    str(defaults.get(name)).lower(),
                    expected_value,
                )

    def test_hardware_launch_can_start_the_guided_gui_without_auto_arm(self):
        hardware = source(LAUNCH / "hardware_control.launch.py")
        self.assertIn('executable="volt_control_gui.py"', hardware)
        self.assertIn("condition=IfCondition(gui)", hardware)
        self.assertEqual(
            node_executables(LAUNCH / "hardware_control.launch.py").count(
                "volt_control_gui.py"
            ),
            1,
        )

    def test_combined_launch_includes_each_stack_once(self):
        combined = source(LAUNCH / "volt_start.launch.py")
        control = source(LAUNCH / "control.launch.py")

        self.assertEqual(
            len(
                re.findall(
                    r"""['"]ignition\.launch\.py['"]""",
                    combined,
                )
            ),
            1,
        )
        self.assertEqual(
            len(
                re.findall(
                    r"""['"]control\.launch\.py['"]""",
                    combined,
                )
            ),
            1,
        )
        self.assertNotIn("hardware_control.launch.py", combined)

        # The combined launch receives exactly one router through its single
        # control.launch.py include; it must not create a second local router.
        self.assertEqual(
            node_executables(LAUNCH / "control.launch.py").count(
                "volt_joint_command_router.py"
            ),
            1,
        )
        self.assertNotIn(
            "volt_joint_command_router.py",
            node_executables(LAUNCH / "volt_start.launch.py"),
        )

        combined = source(LAUNCH / "volt_start.launch.py")
        self.assertIn(
            "condition=IfCondition(start_serial_bridge)",
            combined,
        )

    def test_launches_have_no_user_specific_workspace_paths(self):
        forbidden = (
            "/home/ros2",
            "/home/",
            "Documents/volt_ws",
        )
        for path in sorted(LAUNCH.glob("*.launch.py")):
            contents = source(path)
            for token in forbidden:
                with self.subTest(launch=path.name, token=token):
                    self.assertNotIn(token, contents)

    def test_ignition_path_has_no_gazebo_classic_dependencies(self):
        ignition = source(LAUNCH / "ignition.launch.py")
        forbidden = (
            "gazebo_ros",
            "spawn_entity.py",
            "libgazebo_ros2_control.so",
            "GAZEBO_MODEL_PATH",
            "GAZEBO_PLUGIN_PATH",
        )
        for token in forbidden:
            with self.subTest(token=token):
                self.assertNotIn(token, ignition)

    def test_ignition_controller_sequence_requires_successful_exit(self):
        ignition = source(LAUNCH / "ignition.launch.py")
        self.assertIn("def start_after_success", ignition)
        self.assertIn("event.returncode == 0", ignition)
        self.assertEqual(ignition.count("on_exit=start_after_success("), 2)


class ScriptInstallationTests(unittest.TestCase):
    REQUIRED_SCRIPTS = (
        "volt_control_gui.py",
        "volt_motion_controller.py",
        "volt_gait_controller.py",
        "volt_kinematics.py",
        "volt_joint_command_router.py",
        "volt_serial_bridge.py",
        "volt_servo_calibration.py",
        "volt_run_all.py",
        "volt_physical_tests.py",
        "volt_gait_controller.py",
    )

    def test_required_scripts_are_executable(self):
        for filename in self.REQUIRED_SCRIPTS:
            path = ROOT / "scripts" / filename
            with self.subTest(script=filename):
                self.assertTrue(path.is_file())
                self.assertTrue(
                    os.access(path, os.X_OK),
                    "%s is not executable" % path,
                )

    def test_required_scripts_are_installed_as_programs(self):
        cmake = source(ROOT / "CMakeLists.txt")
        program_blocks = re.findall(
            r"install\s*\(\s*PROGRAMS(?P<body>.*?)"
            r"DESTINATION\s+lib/\$\{PROJECT_NAME\}\s*\)",
            cmake,
            flags=re.DOTALL,
        )
        self.assertTrue(program_blocks, "No install(PROGRAMS ...) block found")
        installed_programs = "\n".join(program_blocks)
        for filename in self.REQUIRED_SCRIPTS:
            with self.subTest(script=filename):
                self.assertRegex(
                    installed_programs,
                    r"scripts/" + re.escape(filename) + r"(?:\s|$)",
                )

    def test_guided_arm_helper_is_installed_as_a_private_module(self):
        cmake = source(ROOT / "CMakeLists.txt")
        self.assertRegex(
            cmake,
            r"install\s*\(\s*FILES\s+scripts/volt_arm_workflow\.py"
            r"\s+DESTINATION\s+lib/\$\{PROJECT_NAME\}\s*\)",
        )
        program_blocks = re.findall(
            r"install\s*\(\s*PROGRAMS(?P<body>.*?)"
            r"DESTINATION\s+lib/\$\{PROJECT_NAME\}\s*\)",
            cmake,
            flags=re.DOTALL,
        )
        self.assertNotIn(
            "volt_arm_workflow.py",
            "\n".join(program_blocks),
        )

    def test_gamepad_loss_stops_persistent_step_action(self):
        gui = source(ROOT / "scripts" / "volt_control_gui.py")
        tree = ast.parse(gui)
        methods = {
            node.name: ast.get_source_segment(gui, node) or ""
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef)
        }
        for method_name in ("toggle_gamepad", "handle_gamepad_disconnect"):
            with self.subTest(method=method_name):
                self.assertIn(
                    "self.stop_motion_controls()",
                    methods[method_name],
                )
        self.assertIn('"step_keepalive"', methods["publish_motion"])

    def test_gui_uses_cancellable_guided_arm_workflow(self):
        gui = source(ROOT / "scripts" / "volt_control_gui.py")
        self.assertIn("GuidedArmWorkflow", gui)
        self.assertIn("CANCEL ARM / HOLD", gui)
        self.assertNotIn("QTimer.singleShot(250", gui)


if __name__ == "__main__":
    unittest.main()


class SplitJetsonStackTests(unittest.TestCase):
    """The Jetson/workstation split, checked statically.

    The robot half and the console half are two launch files that must not
    overlap: anything with a deadline belongs next to the Arduino, and
    anything heavy belongs on the workstation. These assert that split, and
    that the single-machine paths were not disturbed to get it.
    """

    JETSON = LAUNCH / "volt_jetson.launch.py"
    OPERATOR = LAUNCH / "volt_operator.launch.py"

    def test_both_launch_files_exist(self):
        self.assertTrue(self.JETSON.is_file())
        self.assertTrue(self.OPERATOR.is_file())

    def test_robot_half_starts_no_renderer_and_no_console(self):
        text = source(self.JETSON)
        self.assertNotIn("ignition.launch.py", text)
        self.assertNotIn("volt_control_gui", text)
        self.assertIn("volt_serial_bridge.py", text)
        self.assertIn("control.launch.py", text)

    def test_console_half_starts_no_hardware(self):
        text = source(self.OPERATOR)
        self.assertNotIn("volt_serial_bridge", text)
        self.assertNotIn("volt_motion_controller", text)
        self.assertIn("volt_control_gui.py", text)
        self.assertIn("ignition.launch.py", text)

    def test_robot_half_defaults_to_dry_run(self):
        """A split stack must not come up talking to servos by accident."""
        defaults = launch_argument_defaults(self.JETSON)
        self.assertEqual("true", defaults["dry_run"])
        self.assertEqual("false", defaults["auto_arm"])
        self.assertEqual("false", defaults["auto_ready_pose"])

    def test_robot_half_never_follows_a_simulation_clock(self):
        """There is no Gazebo on the Jetson, so there is no /clock to follow."""
        text = source(self.JETSON)
        self.assertIn('"use_sim_time": "false"', text)

    def test_console_half_never_follows_a_simulation_clock(self):
        """The authority is the Jetson's wall clock, not the shadow's."""
        self.assertIn('"use_sim_time": "false"', source(self.OPERATOR))

    def test_single_machine_launch_is_unchanged_by_the_split(self):
        """volt_start.launch.py must not learn about the Jetson."""
        text = source(LAUNCH / "volt_start.launch.py")
        for token in ("jetson", "Jetson", "ssh", "volt_operator"):
            self.assertNotIn(token, text)

    def test_runner_script_is_installed(self):
        cmake = source(ROOT / "CMakeLists.txt")
        self.assertIn("scripts/volt_jetson_run.sh", cmake)

    def test_runner_tears_down_the_remote_half_on_every_exit(self):
        """A console that dies must not leave the Jetson streaming frames."""
        runner = source(ROOT / "scripts" / "volt_jetson_run.sh")
        self.assertIn("trap stop_remote EXIT INT TERM", runner)
        # SIGINT before SIGKILL, so the bridge can send HOLD and the
        # firmware disarms cleanly instead of timing out.
        self.assertIn("kill -INT -$pgid", runner)
        self.assertIn("kill -9 -$pgid", runner)

    def test_launcher_keeps_the_original_modes(self):
        launcher = source(ROOT / "scripts" / "volt_desktop_launcher.sh")
        for mode in ("sim)", "gui)", "physical)", "jetson)"):
            self.assertIn(mode, launcher)

    def test_empty_installed_directories_are_kept_in_git(self):
        """install(DIRECTORY ...) fails on a clean clone without these.

        description/ and meshes/ are empty; git does not track empty
        directories, so a fresh clone had neither and the build died in
        ament_cmake_symlink_install_directory before compiling anything.
        """
        for name in ("description", "meshes"):
            self.assertTrue(
                (ROOT / name / ".gitkeep").is_file(),
                "%s/.gitkeep is what makes a clean clone buildable" % name,
            )

    def test_teardown_is_armed_before_any_check_that_can_die(self):
        """A preflight failure must still clear a previous run's stack.

        The port check dies when no Arduino is on the Jetson. If the trap
        were installed after it, that abort would leave an earlier stack
        streaming frames to a robot with no console attached.
        """
        runner = source(ROOT / "scripts" / "volt_jetson_run.sh")
        trap = runner.index("trap stop_remote EXIT INT TERM")
        port_check = runner.index("no /dev/ttyUSB* or /dev/ttyACM* on the Jetson")
        workspace_check = runner.index("is not built on the Jetson")
        self.assertLess(trap, port_check)
        self.assertLess(trap, workspace_check)

    def test_runner_provisions_the_dds_profile_on_the_jetson(self):
        """Shared memory cannot cross a network; UDP-only is not optional."""
        runner = source(ROOT / "scripts" / "volt_jetson_run.sh")
        self.assertIn("useBuiltinTransports>false", runner)
        self.assertIn("FASTRTPS_DEFAULT_PROFILES_FILE", runner)
