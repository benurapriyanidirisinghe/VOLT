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
        "volt_fast_trot_sweep.py",
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
