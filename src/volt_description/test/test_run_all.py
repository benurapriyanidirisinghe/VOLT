#!/usr/bin/env python3

"""Regression tests for the one-command VOLT runner."""

import argparse
import importlib.util
from pathlib import Path
import subprocess
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "volt_run_all.py"
SPEC = importlib.util.spec_from_file_location("volt_run_all", SCRIPT)
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


def arguments(**overrides):
    values = {
        "physical": False,
        "serial_port": None,
        "baud_rate": "57600",
        "gazebo_gui": "true",
        "gazebo_renderer": "auto",
        "no_control_gui": False,
        "auto_ready_pose": "false",
        "arm_hardware": "false",
        "use_hardware": "false",
        "dry_run": "true",
        "hardware_delay": 0.0,
        "gui_delay": 0.0,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


class RunAllTests(unittest.TestCase):
    def test_active_ros_nodes_uses_live_graph_and_parses_names(self):
        completed = subprocess.CompletedProcess(
            args=[],
            returncode=0,
            stdout="/volt_motion_controller\n/controller_manager\nwarning\n",
            stderr="",
        )
        with mock.patch.object(
            RUNNER.subprocess,
            "run",
            return_value=completed,
        ) as run:
            nodes = RUNNER.active_ros_nodes()
        self.assertEqual(
            nodes,
            {"/volt_motion_controller", "/controller_manager"},
        )
        self.assertIn("--no-daemon", run.call_args.args[0])

    def test_conflicting_processes_ignore_separate_gui_client(self):
        argument_lists = [
            ["ign", "gazebo", "-g", "-v", "2", "--render-engine-gui", "ogre"],
            ["ign", "gazebo", "-r", "-s", "-v", "2", "empty_ign.sdf"],
            ["python3", "/installed/volt_motion_controller.py"],
            ["rg", "volt_joint_command_router.py"],
        ]
        self.assertEqual(
            RUNNER.conflicting_processes(argument_lists),
            ["Ignition Gazebo server", "volt_motion_controller.py"],
        )

    def test_ros_discovery_timeout_fails_closed(self):
        with mock.patch.object(
            RUNNER.subprocess,
            "run",
            side_effect=subprocess.TimeoutExpired("ros2", 4.0),
        ):
            with self.assertRaises(RUNNER.PreflightError):
                RUNNER.active_ros_nodes()

    def test_auto_renderer_uses_intel_when_nvidia_prime_has_no_module(self):
        with (
            mock.patch.object(RUNNER, "prime_profile", return_value="nvidia"),
            mock.patch.object(RUNNER, "nvidia_module_loaded", return_value=False),
            mock.patch.object(RUNNER, "intel_drm_available", return_value=True),
        ):
            self.assertEqual(
                RUNNER.select_gazebo_renderer("auto"),
                "intel-mesa",
            )

    def test_intel_client_uses_mesa_ogre_without_software_or_prime_flags(self):
        environment = RUNNER.gazebo_client_environment(
            "intel-mesa",
            {
                "LIBGL_ALWAYS_SOFTWARE": "1",
                "__NV_PRIME_RENDER_OFFLOAD": "1",
                "__VK_LAYER_NV_optimus": "NVIDIA_only",
                "KEEP": "yes",
            },
        )
        self.assertNotIn("LIBGL_ALWAYS_SOFTWARE", environment)
        self.assertNotIn("__NV_PRIME_RENDER_OFFLOAD", environment)
        self.assertNotIn("__VK_LAYER_NV_optimus", environment)
        self.assertEqual(environment["__GLX_VENDOR_LIBRARY_NAME"], "mesa")
        self.assertEqual(environment["MESA_LOADER_DRIVER_OVERRIDE"], "iris")
        self.assertEqual(environment["KEEP"], "yes")
        command = RUNNER.gazebo_client_command("intel-mesa")
        self.assertIn("ogre", command)
        self.assertIn("--render-engine-gui", command)

    def test_combined_launch_is_headless_and_gui_clients_are_separate(self):
        command = RUNNER.volt_start_command(arguments(), "/dev/ttyUSB0")
        self.assertIn("gazebo_gui:=false", command)
        self.assertIn("gui:=false", command)
        self.assertIn("start_serial_bridge:=true", command)
        self.assertIn("use_hardware:=false", command)
        self.assertIn("dry_run:=true", command)

    def test_physical_preset_opens_one_live_bridge_but_never_auto_arms(self):
        parsed = RUNNER.parse_args([
            "--physical",
            "--gazebo-gui",
            "false",
            "--no-control-gui",
            "--auto-ready-pose",
            "true",
            "--arm-hardware",
            "true",
            "--use-hardware",
            "false",
            "--dry-run",
            "true",
        ])
        configured = RUNNER.apply_run_mode(parsed)

        self.assertTrue(configured.physical)
        self.assertEqual(configured.gazebo_gui, "true")
        self.assertFalse(configured.no_control_gui)
        self.assertEqual(configured.auto_ready_pose, "false")
        self.assertEqual(configured.arm_hardware, "false")
        self.assertEqual(configured.use_hardware, "true")
        self.assertEqual(configured.dry_run, "false")

        command = RUNNER.volt_start_command(configured, "/dev/ttyUSB0")
        self.assertIn("gazebo_gui:=false", command)
        self.assertIn("gui:=false", command)
        self.assertIn("auto_arm:=false", command)
        self.assertIn("use_hardware:=true", command)
        self.assertIn("dry_run:=false", command)

    def test_main_refuses_to_create_a_duplicate_stack(self):
        lock_handle = mock.Mock()
        with (
            mock.patch.object(RUNNER, "parse_args", return_value=arguments()),
            mock.patch.object(
                RUNNER,
                "acquire_runner_lock",
                return_value=lock_handle,
            ),
            mock.patch.object(
                RUNNER,
                "existing_stack_conflicts",
                return_value=(["/controller_manager"], []),
            ),
            mock.patch.object(RUNNER, "start_process") as start_process,
        ):
            self.assertEqual(RUNNER.main(), 2)
        start_process.assert_not_called()
        lock_handle.close.assert_called_once_with()

    def test_runner_lock_rejects_a_second_instance(self):
        first = RUNNER.acquire_runner_lock()
        try:
            with self.assertRaises(BlockingIOError):
                RUNNER.acquire_runner_lock()
        finally:
            first.close()


if __name__ == "__main__":
    unittest.main()
