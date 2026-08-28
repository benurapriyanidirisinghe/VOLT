#!/usr/bin/env python3

"""A stand-in ESP32-S2 servo board, speaking PROTO 3 over TCP.

Exists so the host side of the WiFi link can be tested without hardware. It
implements the same command surface the real firmware answers -- PING, ARM,
HOLD, DISARM, DISABLE, STATUS, FRAME -- and the same binary frame format
(0xA5 magic, sequence, twelve uint16 centidegrees, CRC-8 Dallas/Maxim), so a
bridge that talks to this correctly is talking the protocol correctly.

It is deliberately NOT a simulator: no servo motion, no slew, no timing. It
answers the handshake, validates frames the way the firmware does, and
counts what the firmware counts, which is what the host actually observes.
"""

import socket
import threading


CHANNEL_COUNT = 12
BIN_FRAME_MAGIC = 0xA5
BIN_FRAME_BODY_LEN = 26  # sequence + 24 payload + crc


def crc8_maxim(data):
    crc = 0
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) if crc & 1 else (crc >> 1)
    return crc & 0xFF


class Esp32Stub:
    """Minimal PROTO 3 board on a TCP socket."""

    IDENTITY = (
        "FW=VOLT_PCA9685 PROTO=3 MAX_DPS=240.0 FACE_SUPPORTED=1 "
        "LED_COUNT=8 HOST_SYNC_REQUIRED=1"
    )

    def __init__(self, host="127.0.0.1", port=0):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
        self.server.listen(1)
        self.host, self.port = self.server.getsockname()
        self.armed = False
        self.output = False
        self.frames_bin = 0
        self.crc_fail = 0
        self.seq_gap = 0
        self.last_seq = None
        self.last_frame = None
        self.commands = []
        self._buffer = bytearray()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    @property
    def endpoint(self):
        return "tcp://%s:%d" % (self.host, self.port)

    def close(self):
        self._stop.set()
        try:
            self.server.close()
        except OSError:
            pass

    # -- wire ------------------------------------------------------------
    def _serve(self):
        while not self._stop.is_set():
            try:
                client, _address = self.server.accept()
            except OSError:
                return
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._session(client)

    def _session(self, client):
        with client:
            while not self._stop.is_set():
                try:
                    chunk = client.recv(4096)
                except OSError:
                    return
                if not chunk:
                    return
                self._buffer.extend(chunk)
                for reply in self._consume():
                    try:
                        client.sendall((reply + "\n").encode("ascii"))
                    except OSError:
                        return

    def _consume(self):
        """Pull whole frames and whole lines out of the byte stream."""
        replies = []
        while self._buffer:
            if self._buffer[0] == BIN_FRAME_MAGIC:
                if len(self._buffer) < 1 + BIN_FRAME_BODY_LEN:
                    break
                body = bytes(self._buffer[1:1 + BIN_FRAME_BODY_LEN])
                del self._buffer[:1 + BIN_FRAME_BODY_LEN]
                replies.extend(self._binary_frame(body))
                continue
            newline = self._buffer.find(b"\n")
            if newline < 0:
                # Not a frame and not yet a whole line.
                if len(self._buffer) > 512:
                    self._buffer.clear()
                break
            line = bytes(self._buffer[:newline]).decode("ascii", "replace")
            del self._buffer[:newline + 1]
            reply = self._command(line.strip())
            if reply:
                replies.append(reply)
        return replies

    def _binary_frame(self, body):
        if crc8_maxim(body[:-1]) != body[-1]:
            self.crc_fail += 1
            return []
        sequence = body[0]
        if self.last_seq is not None:
            expected = (self.last_seq + 1) & 0xFF
            if sequence != expected:
                self.seq_gap += 1
        self.last_seq = sequence
        values = []
        for index in range(CHANNEL_COUNT):
            low = body[1 + index * 2]
            high = body[2 + index * 2]
            centidegrees = low | (high << 8)
            if centidegrees > 18000:
                # The firmware discards the whole frame rather than clamp.
                return []
            values.append(centidegrees / 100.0)
        self.frames_bin += 1
        self.last_frame = values
        return []

    def _command(self, line):
        if not line:
            return ""
        self.commands.append(line)
        head = line.split()[0].upper()
        if head == "PING":
            return "OK PONG " + self.IDENTITY
        if head == "ARM":
            self.armed = True
            self.output = True
            return "OK ARM ARMED=1 OUTPUT=1"
        if head == "HOLD":
            self.armed = False
            return "OK HOLD ARMED=0 OUTPUT=1"
        if head == "DISARM":
            self.armed = False
            return "OK DISARM ARMED=0 OUTPUT=1"
        if head == "DISABLE":
            self.armed = False
            self.output = False
            return "OK DISABLE ARMED=0 OUTPUT=0"
        if head == "HOST":
            return "OK HOST SYNC"
        if head == "LED":
            return "OK LED STATUS " + self.IDENTITY
        if head == "FACE":
            parts = line.split()
            return "OK FACE %s" % (parts[1] if len(parts) > 1 else "neutral")
        if head == "STATUS":
            return (
                "OK STATUS %s ARMED=%d OUTPUT=%d LAST_CMD_MS=5 "
                "FRAMES_ASCII=0 FRAMES_BIN=%d CRC_FAIL=%d SEQ_GAP=%d "
                "LOOP_MAX_US=900 BUS_MAX_US=0 LED_SHOWS=0 SRAM_FREE=180000 "
                "LED_ENABLED=1 FACE=neutral"
                % (
                    self.IDENTITY,
                    1 if self.armed else 0,
                    1 if self.output else 0,
                    self.frames_bin,
                    self.crc_fail,
                    self.seq_gap,
                )
            )
        return "ERR UNKNOWN"
