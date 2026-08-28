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

    def test_launch_file_reuses_the_tested_control_stack(self):
        """Composed from control.launch.py, not restated node by node."""
        text = source(self.LAUNCH)
        self.assertIn("control.launch.py", text)
        self.assertIn("board_endpoint", text)
        self.assertIn("volt_serial_bridge.py", text)

    def test_no_simulator_by_default(self):
        """The hardware is open-loop, so Ignition renders the COMMANDED pose.

        That is cost without evidence when the real robot is in front of you,
        and it drags gz_ros2_control, robot_state_publisher and the clock
        bridge into a stack with a 750 ms disarm deadline.
        """
        text = source(self.LAUNCH)
        # Search the CODE, not the module docstring -- which mentions
        # volt_start.launch.py precisely to explain why it is not used.
        code = text.split('"""', 2)[-1]
        # It must not go through volt_start.launch.py, which always brings
        # Ignition up regardless of the gui argument.
        self.assertNotIn("volt_start.launch.py", code)
        gazebo_default = text[text.index('"gazebo",'):][:200]
        self.assertIn('default_value="false"', gazebo_default)
        # Still available on request for a demo.
        self.assertIn("ignition.launch.py", text)
        self.assertIn("condition=IfCondition(gazebo)", text)

    def test_defaults_to_dry_run_and_manual_arm(self):
        text = source(self.LAUNCH)
        for name, default in (("dry_run", "true"), ("auto_arm", "false"),
                              ("auto_ready_pose", "false")):
            block = text[text.index('"%s",' % name):][:260]
            self.assertIn('default_value="%s"' % default, block,
                          "%s must default to %s" % (name, default))

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


def function_body(text, signature):
    """Source of one top-level C function, by signature.

    Fixed character windows break the moment a comment is added, which is
    exactly what happened here -- a passing test started failing because the
    function it checks grew a paragraph of explanation.
    """
    start = text.index(signature)
    depth = 0
    for index in range(start, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[start:index + 1]
    raise AssertionError("unbalanced braces after %s" % signature)


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
        handler = function_body(self.text, "void onHostDisconnected()")
        self.assertIn("holdCurrentPosition();", handler)
        self.assertIn("servoArmed = false;", handler)

    def test_a_second_host_is_refused_not_interleaved(self):
        service = function_body(self.text, "void serviceNetwork()")
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
        join = function_body(self.text, "bool joinBestNetwork()")
        self.assertIn("scanAndReportNetworks()", join)

    def test_supports_several_configured_networks(self):
        self.assertIn("struct WifiNetwork", self.text)
        self.assertIn("WIFI_NETWORKS[]", self.text)
        self.assertIn("WIFI_NETWORK_COUNT", self.text)

    def test_joins_the_strongest_visible_network_not_the_first(self):
        """Joining a weak AP when a strong one is present stutters the stream.

        bestVisibleNetwork() has been replaced by an ordered candidate list
        so a refused join can fall through to the next network; strongest is
        still tried first.
        """
        chooser = function_body(self.text, "int visibleNetworksByStrength(")
        self.assertIn("WiFi.RSSI(index)", chooser)
        self.assertIn("rssis[slot - 1] < rssi", chooser)
        self.assertNotIn("int bestVisibleNetwork", self.text)

    def test_scan_never_runs_while_servos_are_being_driven(self):
        """scanNetworks() blocks for seconds; that would trip the 750 ms disarm."""
        service = function_body(self.text, "void serviceNetwork()")
        # The only in-loop rescan sits behind "no WiFi connection", where by
        # definition no host is streaming frames.
        rescan = service.index("joinBestNetwork();")
        disconnected = service.index("WiFi.status() != WL_CONNECTED")
        self.assertLess(disconnected, rescan)

    def test_reports_link_quality_in_status(self):
        status = function_body(self.text, "void printStatus()")
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

    def test_no_credentials_are_committed(self):
        """This repository is public; a committed password is a published one."""
        sketch_dir = self.SKETCH.parent
        real = sketch_dir / "wifi_credentials.h"
        template = sketch_dir / "wifi_credentials_example.h"
        self.assertTrue(
            template.is_file(),
            "the tracked template must exist so a fresh clone can build",
        )
        gitignore = source(
            Path(__file__).resolve().parents[3] / ".gitignore"
        )
        self.assertIn("wifi_credentials.h", gitignore)
        if real.is_file():
            import subprocess

            tracked = subprocess.run(
                ["git", "ls-files", "--error-unmatch", str(real)],
                cwd=str(Path(__file__).resolve().parents[3]),
                capture_output=True,
            )
            self.assertNotEqual(
                0, tracked.returncode,
                "wifi_credentials.h is tracked by git; the password is public",
            )

    def test_sketch_has_no_inline_password(self):
        """The fallback must stay a placeholder, not someone's real key."""
        block = self.text[self.text.index("const WifiNetwork WIFI_NETWORKS[]"):][:400]
        self.assertIn("VOLT_WIFI_NETWORKS", block)
        fallback = self.text[self.text.index("#define VOLT_WIFI_NETWORKS"):][:200]
        self.assertIn("CHANGE_ME", fallback)

    def test_frame_stall_window_suits_a_network_transport(self):
        """20 ms was a UART number and it discarded every split frame.

        TCP does not lose bytes mid-stream, so a gap only means the segment
        boundary fell inside a frame. The measured loop here reached 27-48 ms
        on its own, longer than the old window, so a healthy link reported
        FRAMES_BIN=0 with the host seeing zero drops.
        """
        line = [
            row for row in self.text.splitlines()
            if row.startswith("const uint32_t BIN_FRAME_STALL_US")
        ]
        self.assertTrue(line)
        microseconds = int(line[0].split("=")[1].strip().rstrip("UL;"))
        self.assertGreaterEqual(
            microseconds, 100000,
            "must exceed the worst observed loop time by a wide margin",
        )
        self.assertLess(
            microseconds, 750000,
            "must stay under COMMAND_TIMEOUT_MS, which owns a dead link",
        )

    def test_dead_peer_detection_uses_keepalive_not_silence(self):
        """Silence is not evidence of death for this protocol.

        The bridge streams frames only while ARMED and gates its STATUS poll
        behind protocol.armed too, so a healthy PRE-ARM console sends nothing
        at all. Any silence timeout therefore drops it on a loop and the
        handshake never completes -- observed on hardware as connected=1
        with ready=0 forever. Keepalive probes answer the question without
        needing application traffic.
        """
        self.assertIn("enableClientKeepalive", self.text)
        helper = function_body(self.text, "void enableClientKeepalive")
        for option in ("VOLT_SO_KEEPALIVE", "VOLT_TCP_KEEPIDLE",
                       "VOLT_TCP_KEEPINTVL", "VOLT_TCP_KEEPCNT"):
            self.assertIn(option, helper)
        service = function_body(self.text, "void serviceNetwork()")
        self.assertIn("enableClientKeepalive(voltClient);", service)
        # The mechanism it replaced must be gone, not merely bypassed.
        self.assertNotIn("lastClientByteMs", self.text)
        self.assertNotIn("CLIENT_IDLE_TIMEOUT_MS", self.text)

    def test_keepalive_helper_sits_below_the_sketch_types(self):
        """Arduino inserts auto-prototypes before the FIRST function body.

        Defining this helper above the sketch's enums put every prototype
        ahead of the types they reference, and the build failed in the face
        code with no apparent connection to networking.
        """
        first_enum = self.text.index("enum FaceEffect")
        helper = self.text.index("void enableClientKeepalive")
        self.assertLess(first_enum, helper)

    def test_socket_constants_are_named_locally(self):
        """<lwip/sockets.h> moves the auto-prototype insertion point too."""
        # The comment explaining WHY the header is avoided is expected to
        # mention it; only the include itself must be absent.
        self.assertNotIn("#include <lwip/sockets.h>", self.text)
        self.assertIn("const int VOLT_SO_KEEPALIVE", self.text)

    def test_join_walks_every_visible_network_not_just_the_best(self):
        """One bad candidate must not lock out a good second network.

        Picking only the strongest meant that if it refused the join --
        wrong password, AP full, a hotspot advertising but not accepting --
        every retry rescanned and chose the same loser again, and the other
        configured network was never tried.
        """
        self.assertIn("int visibleNetworksByStrength(", self.text)
        join = function_body(self.text, "bool joinBestNetwork()")
        self.assertIn("visibleNetworksByStrength(", join)
        self.assertIn("for (int attempt = 0; attempt < candidates", join)
        self.assertIn("refused the join", join)

    def test_candidate_list_is_ordered_strongest_first(self):
        chooser = function_body(self.text, "int visibleNetworksByStrength(")
        self.assertIn("rssis[slot - 1] < rssi", chooser)
        # An SSID seen twice (two bands, a repeater) must not burn two join
        # timeouts on the same network.
        self.assertIn("duplicate", chooser)

    def test_reconnect_retries_immediately_then_backs_off(self):
        """Waiting the full backoff before even trying cost ~20 s per outage."""
        service = function_body(self.text, "void serviceNetwork()")
        self.assertIn("!wifiRetriedSinceLoss", service)
        self.assertIn("wifiRetriedSinceLoss = false;", service)
        # File scope, not function-static: a static would give only the first
        # outage after boot a fast retry.
        self.assertIn("bool wifiRetriedSinceLoss = false;", self.text)

    def test_board_answers_mdns(self):
        """setHostname() sets the DHCP name only; the icon needs .local."""
        self.assertIn("MDNS.begin(VOLT_HOSTNAME)", self.text)
        self.assertIn("ESPmDNS.h", self.text)

    def test_all_parser_buffers_reset_between_clients(self):
        """A socket changes peers; a UART never did. Leftovers corrupt the
        next host's first command into ERR UNKNOWN_COMMAND."""
        handler = function_body(self.text, "void onHostDisconnected()")
        for field in ("binActive", "binLength", "lineLength",
                      "discardLineUntilNewline"):
            self.assertIn(field, handler)

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


class StatusLineLengthTests(unittest.TestCase):
    """A real ESP32 STATUS must survive the host's line buffer and regex."""

    def test_a_full_esp32_status_line_is_not_dropped(self):
        """Measured 523 bytes on hardware; at the old 512 limit it vanished.

        An over-length line is DISCARDED, not truncated, so the symptom was
        "STATUS returns nothing" while PING worked -- which points at
        everything except the line limit.
        """
        from volt_serial_protocol import SerialLineBuffer, _STATUS_FIELD

        line = (
            "OK STATUS FW=VOLT_PCA9685 PROTO=3 MAX_DPS=240.0 "
            "FACE_SUPPORTED=1 LED_COUNT=8 HOST_SYNC_REQUIRED=1 HOST_PING=1 "
            "HOST_SNAPSHOT=0 HOST_SYNCED=0 ARMED=0 OUTPUT=0 "
            "LAST_CMD_MS=32184 FRAMES_ASCII=0 FRAMES_BIN=0 CRC_FAIL=0 "
            "SEQ_GAP=0 LOOP_MAX_US=10850 BUS_MAX_US=0 LED_SHOWS=4588 "
            "SRAM_FREE=253688 WIFI_SSID=NextGen_Starlink_2.4GHz "
            "WIFI_RSSI=-72 WIFI_IP=192.168.2.100 LED_ENABLED=1 "
            "LED_COLOR=0,255,255 LED_COLOR_B=0,120,255 LED_BRIGHTNESS=80 "
            "LED_EFFECTIVE_BRIGHTNESS=80 LED_LIMIT=160 LED_EFFECT=breathe "
            "LED_SPEED_MS=3000 FACE=neutral\n"
        )
        self.assertGreater(len(line), 512, "regression guard needs a long line")
        buffer = SerialLineBuffer()
        found, overflow = buffer.feed(line.encode("ascii"))
        self.assertFalse(overflow, "a real STATUS line must not overflow")
        self.assertEqual(1, len(found))
        fields = dict(_STATUS_FIELD.findall(found[0]))
        self.assertEqual("-72", fields.get("WIFI_RSSI"))
        self.assertEqual("192.168.2.100", fields.get("WIFI_IP"))
        self.assertEqual("NextGen_Starlink_2.4GHz", fields.get("WIFI_SSID"))

    def test_a_value_with_a_space_would_be_truncated(self):
        """Why the firmware substitutes underscores rather than sending spaces."""
        from volt_serial_protocol import _STATUS_FIELD

        spaced = dict(_STATUS_FIELD.findall(
            "OK STATUS WIFI_SSID=NextGen Starlink 2.4GHz WIFI_RSSI=-72"
        ))
        self.assertEqual("NextGen", spaced.get("WIFI_SSID"))

    def test_firmware_substitutes_spaces_in_the_ssid(self):
        sketch = (
            Path(__file__).resolve().parents[3]
            / "firmware" / "volt_esp32_pca9685" / "volt_esp32_pca9685.ino"
        )
        if not sketch.is_file():
            self.skipTest("ESP32 sketch not present")
        text = source(sketch)
        self.assertIn("*cursor == ' ' ? '_' : *cursor", text)
