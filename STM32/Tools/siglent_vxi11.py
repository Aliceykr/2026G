"""Minimal VXI-11 client for the Siglent SDG1032X."""

from __future__ import annotations

import argparse
import random
import socket
import struct


PORTMAPPER_PROGRAM = 100000
PORTMAPPER_VERSION = 2
PORTMAPPER_GETPORT = 3
DEVICE_CORE_PROGRAM = 0x0607AF
DEVICE_CORE_VERSION = 1
DEVICE_CREATE_LINK = 10
DEVICE_WRITE = 11
DEVICE_READ = 12
DEVICE_DESTROY_LINK = 23
IPPROTO_TCP = 6
VXI11_FLAG_END = 8


def _u32(value: int) -> bytes:
    return struct.pack(">I", value & 0xFFFFFFFF)


def _xdr_opaque(value: bytes) -> bytes:
    return _u32(len(value)) + value + b"\0" * ((-len(value)) % 4)


def _xdr_string(value: str) -> bytes:
    return _xdr_opaque(value.encode("ascii"))


def _recv_exact(sock: socket.socket, length: int) -> bytes:
    data = bytearray()
    while len(data) < length:
        block = sock.recv(length - len(data))
        if not block:
            raise ConnectionError("VXI-11 socket closed unexpectedly")
        data.extend(block)
    return bytes(data)


def _recv_record(sock: socket.socket) -> bytes:
    data = bytearray()
    while True:
        marker = struct.unpack(">I", _recv_exact(sock, 4))[0]
        data.extend(_recv_exact(sock, marker & 0x7FFFFFFF))
        if marker & 0x80000000:
            return bytes(data)


def _rpc_call(
    host: str,
    port: int,
    program: int,
    version: int,
    procedure: int,
    arguments: bytes,
    timeout: float,
) -> bytes:
    xid = random.randrange(1, 0xFFFFFFFF)
    call = (
        _u32(xid)
        + _u32(0)
        + _u32(2)
        + _u32(program)
        + _u32(version)
        + _u32(procedure)
        + _u32(0)
        + _u32(0)
        + _u32(0)
        + _u32(0)
        + arguments
    )

    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(_u32(0x80000000 | len(call)) + call)
        reply = _recv_record(sock)

    offset = 0

    def get_u32() -> int:
        nonlocal offset
        value = struct.unpack_from(">I", reply, offset)[0]
        offset += 4
        return value

    if get_u32() != xid or get_u32() != 1:
        raise RuntimeError("invalid RPC reply")
    if get_u32() != 0:
        raise RuntimeError("RPC request denied")
    get_u32()  # verifier flavor
    verifier_length = get_u32()
    offset += (verifier_length + 3) & ~3
    if get_u32() != 0:
        raise RuntimeError("RPC procedure rejected")
    return reply[offset:]


class Vxi11Instrument:
    def __init__(self, host: str, timeout: float = 3.0) -> None:
        self.host = host
        self.timeout = timeout
        self.core_port = 0
        self.link_id = 0

    def open(self) -> None:
        mapping = (
            _u32(DEVICE_CORE_PROGRAM)
            + _u32(DEVICE_CORE_VERSION)
            + _u32(IPPROTO_TCP)
            + _u32(0)
        )
        reply = _rpc_call(
            self.host,
            111,
            PORTMAPPER_PROGRAM,
            PORTMAPPER_VERSION,
            PORTMAPPER_GETPORT,
            mapping,
            self.timeout,
        )
        self.core_port = struct.unpack_from(">I", reply, 0)[0]
        if self.core_port == 0:
            raise ConnectionError("VXI-11 core service was not found")

        arguments = (
            _u32(random.randrange(1, 0x7FFFFFFF))
            + _u32(0)
            + _u32(int(self.timeout * 1000))
            + _xdr_string("inst0")
        )
        reply = _rpc_call(
            self.host,
            self.core_port,
            DEVICE_CORE_PROGRAM,
            DEVICE_CORE_VERSION,
            DEVICE_CREATE_LINK,
            arguments,
            self.timeout,
        )
        error, self.link_id = struct.unpack_from(">II", reply, 0)
        if error != 0:
            raise RuntimeError(f"VXI-11 create_link error {error}")

    def close(self) -> None:
        if self.core_port == 0:
            return
        _rpc_call(
            self.host,
            self.core_port,
            DEVICE_CORE_PROGRAM,
            DEVICE_CORE_VERSION,
            DEVICE_DESTROY_LINK,
            _u32(self.link_id),
            self.timeout,
        )
        self.core_port = 0

    def query(self, command: str) -> str:
        if not command.rstrip().endswith("?"):
            raise ValueError("SCPI query must end in '?'")

        self.write(command)

        timeout_ms = int(self.timeout * 1000)
        read_arguments = (
            _u32(self.link_id)
            + _u32(65536)
            + _u32(timeout_ms)
            + _u32(timeout_ms)
            + _u32(0)
            + _u32(0)
        )
        reply = _rpc_call(
            self.host,
            self.core_port,
            DEVICE_CORE_PROGRAM,
            DEVICE_CORE_VERSION,
            DEVICE_READ,
            read_arguments,
            self.timeout,
        )
        error, _reason, length = struct.unpack_from(">III", reply, 0)
        if error != 0:
            raise RuntimeError(f"VXI-11 read error {error}")
        return reply[12 : 12 + length].decode("ascii", errors="replace").strip()

    def write(self, command: str) -> None:
        if not command.strip():
            raise ValueError("SCPI command is empty")

        timeout_ms = int(self.timeout * 1000)
        payload = (command.rstrip() + "\n").encode("ascii")
        write_arguments = (
            _u32(self.link_id)
            + _u32(timeout_ms)
            + _u32(timeout_ms)
            + _u32(VXI11_FLAG_END)
            + _xdr_opaque(payload)
        )
        reply = _rpc_call(
            self.host,
            self.core_port,
            DEVICE_CORE_PROGRAM,
            DEVICE_CORE_VERSION,
            DEVICE_WRITE,
            write_arguments,
            self.timeout,
        )
        error, written = struct.unpack_from(">II", reply, 0)
        if error != 0 or written != len(payload):
            raise RuntimeError(
                f"VXI-11 write error={error}, written={written}"
            )

    def __enter__(self) -> "Vxi11Instrument":
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", help="instrument IPv4 address")
    parser.add_argument("query", nargs="?", default="*IDN?")
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()

    with Vxi11Instrument(args.host, args.timeout) as instrument:
        print(instrument.query(args.query))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
