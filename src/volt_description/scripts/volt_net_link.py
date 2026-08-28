#!/usr/bin/env python3

"""TCP transport for the VOLT servo link, shaped like a pyserial port.

The ESP32-S2 board runs the same PROTO 3 byte stream the Arduino runs; only
the wire underneath changes, from USB-serial to WiFi. So this presents the
exact surface volt_serial_bridge already uses -- in_waiting, read, write,
flush, close, reset_input_buffer, reset_output_buffer -- and every layer
above it (framing, CRC, ARM handshake, STATUS parsing, the guard mirror)
stays byte-for-byte the same code on both transports.

Two behaviours differ from a cable on purpose:

TCP_NODELAY is set. Nagle would coalesce 27-byte frames and add up to a
tick of latency to a 60 Hz control stream for no benefit.

A frame that cannot be sent immediately is DROPPED, not queued. This is the
important one. TCP will happily buffer and retransmit through a WiFi stall,
and what arrives afterwards is a burst of stale servo targets -- positions
the robot was supposed to be in half a second ago, delivered in order, at
speed. The firmware's 750 ms command timeout is a much safer response to a
bad link than a backlog, so a blocked socket loses the frame and the next
one goes out fresh. dropped_frames counts them so the operator can see a
marginal link rather than guess at it.
"""

import errno
import socket


class NetLinkError(OSError):
    """Raised when the link is unusable; the bridge treats it like a port error."""


DEFAULT_PORT = 3333
CONNECT_TIMEOUT = 4.0
# Enough for several frames of jitter, small enough that a stall is felt as a
# drop rather than absorbed as latency.
SEND_BUFFER_BYTES = 4096


def parse_endpoint(descriptor):
    """Split ``tcp://host:port`` into (host, port), or return None.

    Returns None for anything that is not a TCP descriptor, so the caller can
    fall through to opening it as a serial device.
    """
    text = str(descriptor or "").strip()
    for prefix in ("tcp://", "esp32://"):
        if text.lower().startswith(prefix):
            text = text[len(prefix):]
            break
    else:
        return None
    if not text:
        raise NetLinkError("empty TCP endpoint")
    if text.count(":") == 1:
        host, _sep, port_text = text.partition(":")
        try:
            port = int(port_text)
        except ValueError:
            raise NetLinkError("bad TCP port in '%s'" % descriptor) from None
    else:
        host, port = text, DEFAULT_PORT
    host = host.strip("[]")
    if not host:
        raise NetLinkError("empty host in '%s'" % descriptor)
    if not 1 <= port <= 65535:
        raise NetLinkError("TCP port %d out of range" % port)
    return host, port


class TcpLink:
    """A pyserial-shaped TCP client for the ESP32 servo board."""

    def __init__(self, host, port=DEFAULT_PORT, timeout=CONNECT_TIMEOUT):
        self.host = host
        self.port = int(port)
        self.dropped_frames = 0
        self._buffer = bytearray()
        self._socket = None
        try:
            self._socket = socket.create_connection(
                (host, self.port), timeout=timeout
            )
        except OSError as exc:
            raise NetLinkError(
                "could not connect to %s:%d: %s" % (host, self.port, exc)
            ) from exc
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            self._socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_SNDBUF, SEND_BUFFER_BYTES
            )
        except OSError:
            # Advisory only; a kernel that refuses the hint still works.
            pass
        # Non-blocking from here: the bridge polls at 60 Hz and must never
        # sit in recv() waiting for a board that has gone quiet.
        self._socket.setblocking(False)

    # -- pyserial surface ------------------------------------------------
    @property
    def is_open(self):
        return self._socket is not None

    @property
    def in_waiting(self):
        self._pump()
        return len(self._buffer)

    def read(self, size=1):
        self._pump()
        if size <= 0:
            return b""
        chunk = bytes(self._buffer[:size])
        del self._buffer[:len(chunk)]
        return chunk

    def write(self, payload):
        if self._socket is None:
            raise NetLinkError("link is closed")
        data = bytes(payload)
        try:
            return self._socket.send(data)
        except (BlockingIOError, InterruptedError):
            # See the module docstring: a stalled link loses this frame
            # rather than queueing a stale one behind it.
            self.dropped_frames += 1
            return 0
        except OSError as exc:
            self.close()
            raise NetLinkError("send failed: %s" % exc) from exc

    def flush(self):
        """No-op: TCP_NODELAY means send() has already handed data to the stack."""

    def reset_input_buffer(self):
        self._buffer.clear()
        if self._socket is None:
            return
        # Discard anything the board queued before we were listening.
        while True:
            try:
                if not self._socket.recv(4096):
                    break
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break

    def reset_output_buffer(self):
        """No-op: nothing is queued locally by design."""

    def close(self):
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
        self._socket = None
        self._buffer.clear()

    # -- internals -------------------------------------------------------
    def _pump(self):
        """Drain whatever the board has sent into the local buffer."""
        if self._socket is None:
            return
        while True:
            try:
                chunk = self._socket.recv(4096)
            except (BlockingIOError, InterruptedError):
                return
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    return
                self.close()
                raise NetLinkError("receive failed: %s" % exc) from exc
            if not chunk:
                # Orderly close from the board. Surface it as a link error so
                # the bridge runs its normal reconnect path instead of
                # spinning on an empty socket forever.
                self.close()
                raise NetLinkError("board closed the connection")
            self._buffer.extend(chunk)


def open_link(descriptor, timeout=CONNECT_TIMEOUT):
    """Open ``tcp://host[:port]``; returns None if it is not a TCP descriptor."""
    endpoint = parse_endpoint(descriptor)
    if endpoint is None:
        return None
    host, port = endpoint
    return TcpLink(host, port, timeout=timeout)
