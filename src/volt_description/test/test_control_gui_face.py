#!/usr/bin/env python3

"""Focused adapter tests for typed, bounded face LED ROS publication."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import rclpy
from PyQt5.QtWidgets import QApplication, QScrollArea


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import volt_control_gui as gui  # noqa: E402
from volt_face import FaceSettings  # noqa: E402


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class RecordingLabel:
    def __init__(self):
        self.text = ""
        self.style = ""

    def setText(self, text):
        self.text = str(text)

    def setStyleSheet(self, style):
        self.style = str(style)


class GuiFacePublisherTests(unittest.TestCase):
    def setUp(self):
        self.node = gui.VoltGuiNode.__new__(gui.VoltGuiNode)
        self.node.face_expression_publisher = RecordingPublisher()
        self.node.face_color_publisher = RecordingPublisher()
        self.node.face_alternate_color_publisher = RecordingPublisher()
        self.node.face_brightness_publisher = RecordingPublisher()
        self.node.face_effect_publisher = RecordingPublisher()
        self.node.face_speed_publisher = RecordingPublisher()

    def test_full_snapshot_uses_confirmed_types_and_normalized_color(self):
        settings = FaceSettings(
            expression="love",
            color=(255, 64, 0),
            alternate_color=(4, 128, 255),
            brightness=81,
            effect="pulse",
            speed_ms=475,
        )
        with patch.object(gui.rclpy, "ok", return_value=True):
            self.assertTrue(self.node.publish_face_settings(settings))
        self.assertEqual(self.node.face_expression_publisher.messages[-1].data, "love")
        color = self.node.face_color_publisher.messages[-1]
        self.assertAlmostEqual(color.r, 1.0)
        self.assertAlmostEqual(color.g, 64.0 / 255.0)
        self.assertAlmostEqual(color.b, 0.0)
        self.assertEqual(color.a, 1.0)
        alternate = self.node.face_alternate_color_publisher.messages[-1]
        self.assertAlmostEqual(alternate.r, 4.0 / 255.0)
        self.assertAlmostEqual(alternate.g, 128.0 / 255.0)
        self.assertAlmostEqual(alternate.b, 1.0)
        self.assertEqual(alternate.a, 1.0)
        self.assertEqual(self.node.face_brightness_publisher.messages[-1].data, 81)
        self.assertEqual(self.node.face_effect_publisher.messages[-1].data, "pulse")
        self.assertEqual(self.node.face_speed_publisher.messages[-1].data, 475)

    def test_disabled_snapshot_only_requests_off(self):
        settings = FaceSettings(enabled=False)
        with patch.object(gui.rclpy, "ok", return_value=True):
            self.assertTrue(self.node.publish_face_settings(settings))
        self.assertEqual(self.node.face_effect_publisher.messages[-1].data, "off")
        self.assertEqual(self.node.face_expression_publisher.messages, [])
        self.assertEqual(self.node.face_color_publisher.messages, [])
        self.assertEqual(self.node.face_alternate_color_publisher.messages, [])

    def test_enabled_shutdown_only_requests_firmware_fade(self):
        settings = FaceSettings(
            enabled=True,
            expression="shutdown",
            color=(0, 0, 0),
            brightness=0,
            effect="off",
            speed_ms=900,
        )
        with patch.object(gui.rclpy, "ok", return_value=True):
            self.assertTrue(self.node.publish_face_settings(settings))
        self.assertEqual(
            [message.data for message in self.node.face_expression_publisher.messages],
            ["shutdown"],
        )
        self.assertEqual(self.node.face_color_publisher.messages, [])
        self.assertEqual(self.node.face_alternate_color_publisher.messages, [])
        self.assertEqual(self.node.face_brightness_publisher.messages, [])
        self.assertEqual(self.node.face_effect_publisher.messages, [])
        self.assertEqual(self.node.face_speed_publisher.messages, [])

    def test_adapter_clamps_numeric_messages(self):
        with patch.object(gui.rclpy, "ok", return_value=True):
            self.assertTrue(self.node.publish_face_color((-10, 300, 128)))
            self.assertTrue(
                self.node.publish_face_alternate_color((300, -10, 64))
            )
            self.assertTrue(self.node.publish_face_brightness(999))
            self.assertTrue(self.node.publish_face_speed(1))
        color = self.node.face_color_publisher.messages[-1]
        self.assertEqual((color.r, color.g), (0.0, 1.0))
        self.assertAlmostEqual(color.b, 128.0 / 255.0)
        alternate = self.node.face_alternate_color_publisher.messages[-1]
        self.assertEqual((alternate.r, alternate.g), (1.0, 0.0))
        self.assertAlmostEqual(alternate.b, 64.0 / 255.0)
        self.assertEqual(self.node.face_brightness_publisher.messages[-1].data, 255)
        self.assertEqual(self.node.face_speed_publisher.messages[-1].data, 10)

    def test_serial_reconnect_reapplies_effective_snapshot_once_per_edge(self):
        catalog = gui.load_face_catalog()
        settings = FaceSettings(expression="love")
        calls = []
        window = SimpleNamespace(
            face_catalog=catalog,
            face_status=RecordingLabel(),
            face_connection_active=False,
            face_last_requested_settings=settings,
            ros_node=SimpleNamespace(
                publish_face_settings=lambda value: calls.append(value) or True
            ),
        )
        connected = {
            "connected": "1",
            "face_supported": "1",
            "face_synced": "0",
            "led_error": "-",
        }
        gui.VoltControlWindow.refresh_face_status(window, connected)
        gui.VoltControlWindow.refresh_face_status(window, connected)
        self.assertEqual(calls, [settings])
        gui.VoltControlWindow.refresh_face_status(
            window,
            {"connected": "0", "face_supported": "1"},
        )
        gui.VoltControlWindow.refresh_face_status(window, connected)
        self.assertEqual(calls, [settings, settings])

    def test_gui_derives_configured_or_primary_alternate_color(self):
        catalog = gui.load_face_catalog()

        def control(value, method):
            return SimpleNamespace(**{method: lambda: value})

        def settings_for(expression, color):
            window = SimpleNamespace(
                face_catalog=catalog,
                face_enable=control(True, "isChecked"),
                face_auto=control(True, "isChecked"),
                face_lock=control(False, "isChecked"),
                face_expression_combo=control(expression, "currentText"),
                face_red=control(color[0], "value"),
                face_green=control(color[1], "value"),
                face_blue=control(color[2], "value"),
                face_brightness=control(80, "value"),
                face_effect_combo=control("solid", "currentText"),
                face_speed=control(500, "value"),
            )
            return gui.VoltControlWindow.current_face_settings(window)

        excited = settings_for("excited", (1, 2, 3))
        self.assertEqual(excited.alternate_color, (255, 0, 180))
        happy = settings_for("happy", (4, 5, 6))
        self.assertEqual(happy.alternate_color, (4, 5, 6))

    def test_control_status_exposes_led_loading_and_host_sync_progress(self):
        cases = (
            (
                {"connected": "0", "ready": "0"},
                "OFFLINE — no Arduino link",
            ),
            (
                {"connected": "1", "ready": "0"},
                "LOADING — firmware handshake",
            ),
            (
                {
                    "connected": "1",
                    "ready": "1",
                    "face_loading": "1",
                    "host_sync_state": "waiting_ping",
                },
                "LOADING — waiting for host PING",
            ),
            (
                {
                    "connected": "1",
                    "ready": "1",
                    "face_loading": "1",
                    "host_sync_state": "applying_snapshot",
                },
                "LOADING — applying GUI LED snapshot",
            ),
            (
                {
                    "connected": "1",
                    "ready": "1",
                    "face_loading": "0",
                    "host_sync_state": "synced",
                    "host_synced": "1",
                    "face_synced": "1",
                },
                "READY — HOST SYNCED",
            ),
        )
        for fields, expected in cases:
            with self.subTest(expected=expected):
                text, _color = gui.face_host_sync_view(fields)
                self.assertEqual(text, expected)

    def test_control_status_distinguishes_physical_connection_modes(self):
        cases = (
            (
                {
                    "connected": "0",
                    "ready": "0",
                    "hardware_enabled": "0",
                    "dry_run": "1",
                },
                "DISCONNECTED — hardware disabled / dry-run",
            ),
            (
                {
                    "connected": "1",
                    "ready": "0",
                    "hardware_enabled": "1",
                    "dry_run": "0",
                },
                "CONNECTED — INITIALIZING FIRMWARE",
            ),
            (
                {
                    "connected": "1",
                    "ready": "1",
                    "hardware_enabled": "1",
                    "dry_run": "0",
                },
                "CONNECTED — READY",
            ),
        )
        for fields, expected in cases:
            with self.subTest(expected=expected):
                text, _color = gui.arduino_connection_view(fields)
                self.assertEqual(text, expected)


class GuiLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cls.initialized_rclpy = not rclpy.ok()
        if cls.initialized_rclpy:
            rclpy.init(args=None)

    @classmethod
    def tearDownClass(cls):
        if cls.initialized_rclpy and rclpy.ok():
            rclpy.shutdown()

    def test_operator_controls_are_grouped_in_scrollable_tabs(self):
        window = gui.VoltControlWindow()
        self.addCleanup(window.ros_node.destroy_node)
        self.addCleanup(window.close)

        self.assertEqual(
            [window.main_tabs.tabText(index) for index in range(4)],
            ["CONTROL", "EMOTES + FACE", "TUNING", "DIAGNOSTICS"],
        )
        self.assertEqual(
            [window.main_tabs.widget(index).objectName() for index in range(4)],
            [
                "controlScroll",
                "expressionsScroll",
                "tuningScroll",
                "diagnosticsScroll",
            ],
        )
        for index in range(4):
            tab = window.main_tabs.widget(index)
            self.assertIsInstance(tab, QScrollArea)
            self.assertTrue(tab.widgetResizable())

        def containing_tab(widget):
            parent = widget.parentWidget()
            while parent is not None and not isinstance(parent, QScrollArea):
                parent = parent.parentWidget()
            return parent

        self.assertIs(containing_tab(window.stand_button), window.main_tabs.widget(0))
        self.assertIs(
            containing_tab(window.arm_readiness_state),
            window.main_tabs.widget(0),
        )
        self.assertIs(
            containing_tab(window.hardware_face_sync),
            window.main_tabs.widget(0),
        )
        self.assertIs(containing_tab(window.emote_status), window.main_tabs.widget(1))
        self.assertIs(
            containing_tab(window.pushup_travel_mm),
            window.main_tabs.widget(1),
        )
        self.assertEqual(window.pushup_travel_mm.minimum(), 10.0)
        self.assertEqual(window.pushup_travel_mm.maximum(), 60.0)
        self.assertEqual(window.pushup_travel_mm.value(), 20.0)
        self.assertEqual(window.pushup_travel_mm.singleStep(), 1.0)
        self.assertIs(containing_tab(window.face_status), window.main_tabs.widget(1))
        self.assertIs(
            containing_tab(window.apply_real_profile_button),
            window.main_tabs.widget(2),
        )
        self.assertIs(
            containing_tab(window.commanded_telemetry),
            window.main_tabs.widget(3),
        )


if __name__ == "__main__":
    unittest.main()
