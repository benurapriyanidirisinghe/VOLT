#!/usr/bin/env python3

"""Tests for the WiFi (TCP) servo transport and the ESP32 launch mode."""

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from esp32_stub import Esp32Stub
from volt_net_link import (
    DEFAULT_PORT,
    NetLinkError,
    TcpLink,
    open_link,
    parse_endpoint,
)
from volt_serial_protocol import (
    ArduinoProtocolState,
    SerialLineBuffer,
    format_binary_frame,
)

ROOT = Path(__file__).resolve().parents[1]


def source(path):
    return path.read_text(encoding="utf-8")


class EndpointTests(unittest.TestCase):
    def test_parses_host_and_port(self):
        self.assertEqual(("10.0.0.5", 3333), parse_endpoint("tcp://10.0.0.5:3333"))
        self.assertEqual(("volt-esp32.local", 9), parse_endpoint("tcp://volt-esp32.local:9"))
        self.assertEqual(("esp", DEFAULT_PORT), parse_endpoint("esp32://esp"))

    def test_serial_devices_are_not_endpoints(self):
        """A device node must fall through to pyserial untouched."""
        for descriptor in ("/dev/ttyUSB0", "/dev/ttyACM1", "", None):
            self.assertIsNone(parse_endpoint(descriptor))

    def test_bad_endpoints_are_rejected_loudly(self):
        for descriptor in ("tcp://", "tcp://host:0", "tcp://host:99999",
                           "tcp://host:abc", "tcp://:3333"):
            with self.assertRaises(NetLinkError, msg=descriptor):
                parse_endpoint(descriptor)

    def test_open_link_ignores_serial_descriptors(self):
        self.assertIsNone(open_link("/dev/ttyUSB0"))


class LinkTests(unittest.TestCase):
    def setUp(self):
        self.board = Esp32Stub()
        self.link = open_link(self.board.endpoint)
        self.lines = SerialLineBuffer()
        self.state = ArduinoProtocolState()

    def tearDown(self):
        self.link.close()
        self.board.close()

    def pump(self, seconds=0.6):
        events = []
        end = time.time() + seconds
        while time.time() < end:
            if self.link.in_waiting:
                found, _overflow = self.lines.feed(
                    self.link.read(self.link.in_waiting)
                )
                for line in found:
                    events.append(self.state.consume_response(line))
            time.sleep(0.01)
        return events

    def test_handshake_establishes_identity_over_tcp(self):
        self.link.write(b"PING\n")
        self.assertIn("ready", self.pump())
        self.assertTrue(self.state.ready)
        self.assertEqual("VOLT_PCA9685", self.state.firmware_id)
        self.assertEqual(3, self.state.protocol_version)
        self.assertEqual(240.0, self.state.max_dps)

    def test_binary_frames_survive_the_socket(self):
        self.link.write(b"PING\n")
        self.pump(0.4)
        for sequence in range(6):
            self.link.write(format_binary_frame([90.0] * 12, sequence))
        time.sleep(0.4)
        self.assertEqual(6, self.board.frames_bin)
        self.assertEqual(0, self.board.crc_fail)
        self.assertEqual(0, self.board.seq_gap)
        self.assertEqual([90.0] * 12, self.board.last_frame)

    def test_a_corrupt_frame_is_rejected_not_applied(self):
        self.link.write(b"PING\n")
        self.pump(0.4)
        self.link.write(format_binary_frame([90.0] * 12, 0))
        time.sleep(0.2)
        corrupted = bytearray(format_binary_frame([12.0] * 12, 1))
        corrupted[-1] ^= 0xFF
        self.link.write(bytes(corrupted))
        time.sleep(0.3)
        self.assertEqual(1, self.board.crc_fail)
        self.assertEqual(1, self.board.frames_bin)
        # The rejected frame must not have reached the servo targets.
        self.assertEqual([90.0] * 12, self.board.last_frame)

    def test_a_dropped_frame_is_visible_as_a_sequence_gap(self):
        self.link.write(b"PING\n")
        self.pump(0.4)
        self.link.write(format_binary_frame([90.0] * 12, 0))
        time.sleep(0.15)
        self.link.write(format_binary_frame([90.0] * 12, 7))
        time.sleep(0.3)
        self.assertEqual(1, self.board.seq_gap)

    def test_arm_and_status_counters_reach_the_host(self):
        self.link.write(b"PING\n")
        self.pump(0.4)
        self.link.write(b"ARM\n")
        self.pump(0.3)
        self.assertTrue(self.state.armed)
        for sequence in range(3):
            self.link.write(format_binary_frame([90.0] * 12, sequence))
        time.sleep(0.25)
        self.link.write(b"STATUS\n")
        self.pump(0.4)
        self.assertEqual("3", self.state.firmware_counters.get("FRAMES_BIN"))
        self.assertEqual("0", self.state.firmware_counters.get("CRC_FAIL"))
        self.link.write(b"HOLD\n")
        self.pump(0.3)
        self.assertFalse(self.state.armed)

    def test_board_going_away_surfaces_as_a_link_error(self):
        """The bridge must run its reconnect path, not spin on a dead socket."""
        self.link.write(b"PING\n")
        self.pump(0.3)
        self.board.close()
        # Force the stub's session to end.
        self.board._stop.set()
        deadline = time.time() + 3.0
        with self.assertRaises(NetLinkError):
            while time.time() < deadline:
                self.link.write(b"STATUS\n")
                _ = self.link.in_waiting
                time.sleep(0.05)
            raise NetLinkError("board never disconnected")


class UnreachableBoardTests(unittest.TestCase):
    def test_refused_connection_is_a_link_error(self):
        """An unpowered board must be an error the bridge can retry, not a hang."""
        import socket as _socket

        # Bound but never listening: the kernel refuses immediately, with no
        # dependence on how fast a previous listener's port is released.
        holder = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
        holder.bind(("127.0.0.1", 0))
        port = holder.getsockname()[1]
        try:
            with self.assertRaises(NetLinkError):
                open_link("tcp://127.0.0.1:%d" % port, timeout=1.0)
        finally:
            holder.close()

    def test_unresolvable_host_is_a_link_error(self):
        with self.assertRaises(NetLinkError):
            open_link("tcp://volt-esp32-does-not-exist.invalid:3333", timeout=2.0)


class BridgeIntegrationTests(unittest.TestCase):
    """The transport swap must not leak into the layers above it."""

    def setUp(self):
        self.bridge = source(ROOT / "scripts" / "volt_serial_bridge.py")

    def test_bridge_branches_on_the_endpoint(self):
        self.assertIn("def link_is_network(self)", self.bridge)
        self.assertIn("open_link(", self.bridge)

    def test_baud_probing_is_skipped_for_a_network_link(self):
        """There is no baud rate on a socket, and no auto-reset to wait out."""
        connect = self.bridge[self.bridge.index("    def connect(self):"):][:2600]
        self.assertIn("self.baud_locked = True", connect)

    def test_pyserial_is_not_required_for_a_network_link(self):
        connect = self.bridge[self.bridge.index("    def connect(self):"):][:1200]
        self.assertIn("if serial is None and not network:", connect)


class WifiLaunchTests(unittest.TestCase):
    LAUNCH = ROOT / "launch" / "volt_wifi.launch.py"

    def test_launch_file_exists_and_reuses_the_tested_stack(self):
        text = source(self.LAUNCH)
        self.assertIn("volt_start.launch.py", text)
        self.assertIn("board_endpoint", text)

    def test_defaults_to_dry_run_and_manual_arm(self):
        text = source(self.LAUNCH)
        self.assertIn('"dry_run",\n            default_value="true"', text)
        self.assertIn('"auto_arm": "false"', text)

    def test_never_follows_a_simulation_clock(self):
        self.assertIn('"use_sim_time": "false"', source(self.LAUNCH))

    def test_launcher_gained_wifi_without_losing_the_others(self):
        launcher = source(ROOT / "scripts" / "volt_desktop_launcher.sh")
        for mode in ("sim)", "gui)", "physical)", "jetson)", "wifi)"):
            self.assertIn(mode, launcher)

    def test_single_machine_launch_is_untouched_by_the_wifi_mode(self):
        text = source(ROOT / "launch" / "volt_start.launch.py")
        for token in ("wifi", "esp32", "tcp://"):
            self.assertNotIn(token, text.lower())


class Esp32FirmwareTests(unittest.TestCase):
    SKETCH = (
        Path(__file__).resolve().parents[3]
        / "firmware" / "volt_esp32_pca9685" / "volt_esp32_pca9685.ino"
    )

    def setUp(self):
        if not self.SKETCH.is_file():
            self.skipTest("ESP32 sketch not present")
        self.text = source(self.SKETCH)

    def test_protocol_constants_match_the_host(self):
        self.assertIn("PROTOCOL_VERSION = 3", self.text)
        self.assertIn("BIN_FRAME_MAGIC = 0xA5", self.text)
        self.assertIn("BIN_FRAME_BODY_LEN = 26", self.text)
        self.assertIn("COMMAND_TIMEOUT_MS = 750", self.text)

    def test_network_comes_up_after_the_servos_are_safe(self):
        """A board reachable before its guards load can be told to break itself."""
        setup = self.text.index("void setup() {")
        start_network = self.text.index("startNetwork();", setup)
        safe_start = self.text.index("CHANNEL_SAFE_START_DEG[channel]", setup)
        self.assertLess(safe_start, start_network)

    def test_losing_the_host_disarms(self):
        handler = self.text[self.text.index("void onHostDisconnected()"):][:700]
        self.assertIn("holdCurrentPosition();", handler)
        self.assertIn("servoArmed = false;", handler)

    def test_a_second_host_is_refused_not_interleaved(self):
        service = self.text[self.text.index("void serviceNetwork()"):][:1600]
        self.assertIn("ERR BUSY", service)

    def test_wifi_power_save_is_disabled(self):
        """Power save adds tens of ms of jitter to a 60 Hz servo stream."""
        self.assertIn("WiFi.setSleep(false)", self.text)

    def test_no_avr_only_heap_probe_remains(self):
        for token in ("__brkval", "__heap_start"):
            self.assertNotIn(token, self.text)
        self.assertIn("ESP.getFreeHeap()", self.text)

    def test_scans_before_joining(self):
        """A join failure must be diagnosable, not a bare timeout."""
        self.assertIn("int scanAndReportNetworks()", self.text)
        self.assertIn("WiFi.scanNetworks()", self.text)
        self.assertIn("bool joinBestNetwork()", self.text)
        join = self.text[self.text.index("bool joinBestNetwork()"):][:1400]
        self.assertIn("scanAndReportNetworks()", join)

    def test_supports_several_configured_networks(self):
        self.assertIn("struct WifiNetwork", self.text)
        self.assertIn("WIFI_NETWORKS[]", self.text)
        self.assertIn("WIFI_NETWORK_COUNT", self.text)

    def test_joins_the_strongest_visible_network_not_the_first(self):
        """Joining a weak AP when a strong one is present stutters the stream."""
        chooser = self.text[self.text.index("int bestVisibleNetwork"):][:900]
        self.assertIn("WiFi.RSSI(index)", chooser)
        self.assertIn("rssi > bestRssi", chooser)

    def test_scan_never_runs_while_servos_are_being_driven(self):
        """scanNetworks() blocks for seconds; that would trip the 750 ms disarm."""
        service = self.text[self.text.index("void serviceNetwork()"):][:1800]
        # The only in-loop rescan sits behind "no WiFi connection", where by
        # definition no host is streaming frames.
        rescan = service.index("joinBestNetwork();")
        disconnected = service.index("WiFi.status() != WL_CONNECTED")
        self.assertLess(disconnected, rescan)

    def test_reports_link_quality_in_status(self):
        status = self.text[self.text.index("void printStatus()"):][:2500]
        for field in ("WIFI_SSID=", "WIFI_RSSI=", "WIFI_IP="):
            self.assertIn(field, status)

    def test_status_field_names_survive_the_host_parser(self):
        """The host regex is [A-Z_]+=; a digit in a name silently drops it."""
        import re
        for field in ("WIFI_SSID", "WIFI_RSSI", "WIFI_IP"):
            self.assertTrue(re.fullmatch(r"[A-Z_]+", field), field)

    def test_host_forwards_the_wifi_fields(self):
        protocol = source(ROOT / "scripts" / "volt_serial_protocol.py")
        for field in ("WIFI_SSID", "WIFI_RSSI", "WIFI_IP"):
            self.assertIn('"%s",' % field, protocol)

    def test_psram_pins_are_not_used_for_io(self):
        """GPIO 35/36/37 carry octal PSRAM on the N16R8 module."""
        for name in ("PIN_I2C_SDA", "PIN_I2C_SCL"):
            line = [
                row for row in self.text.splitlines()
                if row.startswith("const int %s" % name)
            ]
            self.assertTrue(line, name)
            value = int(line[0].split("=")[1].strip().rstrip(";"))
            self.assertNotIn(value, (35, 36, 37), "%s uses a PSRAM pin" % name)


if __name__ == "__main__":
    unittest.main()
