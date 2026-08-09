#!/usr/bin/env python3
"""Download the most recent completed GEEK session log over USB serial."""
import argparse
import re
import sys
import time
import serial


def read_header(ser, deadline):
    while True:
        if time.monotonic() >= deadline:
            raise TimeoutError("timed out waiting for LOG_BEGIN")
        line = ser.readline()
        if not line:
            continue
        if line.startswith(b"LOG_ERROR"):
            raise RuntimeError(line.decode(errors="replace").strip())
        if line.startswith(b"LOG_BEGIN "):
            return line


def read_exact(ser, size, deadline):
    data = bytearray()
    next_report = 256 * 1024
    while len(data) < size:
        if time.monotonic() >= deadline:
            raise TimeoutError(f"timed out after {len(data)} of {size} bytes")
        chunk = ser.read(size - len(data))
        if not chunk:
            continue
        data.extend(chunk)
        if len(data) >= next_report or len(data) == size:
            print(f"received {len(data)}/{size} bytes ({len(data) * 100 // size}%)",
                  flush=True)
            next_report += 256 * 1024
    return bytes(data)

def read_line(ser, deadline):
    while time.monotonic() < deadline:
        line = ser.readline()
        if line:
            if line.startswith(b"LOG_ERROR"):
                raise RuntimeError(line.decode(errors="replace").strip())
            return line
    raise TimeoutError("timed out waiting for protocol line")

def read_protocol_line(ser, prefix, deadline):
    while time.monotonic() < deadline:
        line = read_line(ser, deadline)
        if line.startswith(prefix):
            return line
    raise TimeoutError("timed out waiting for " + prefix.decode())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--output", default="downloaded-fusion-log.bin")
    ap.add_argument("--timeout", type=float, default=300.0,
                    help="overall transfer timeout in seconds")
    args = ap.parse_args()

    with serial.Serial(args.port, 115200, timeout=2) as ser:
        deadline = time.monotonic() + args.timeout
        ser.write(b"DUMP\n")
        line = read_protocol_line(ser, b"LOG_CHUNK_BEGIN ", deadline)
        match = re.match(rb"LOG_CHUNK_BEGIN (\d+) (\d+)\n", line)
        if not match:
            raise RuntimeError("bad response: " + line.decode(errors="replace"))
        size, chunk_size = int(match.group(1)), int(match.group(2))
        payload = bytearray(); seq = 0
        crc = __import__('zlib').crc32
        while len(payload) < size:
            ready = read_line(ser, deadline)
            if not ready.startswith(b"LOG_CHUNK_READY "):
                raise RuntimeError("bad chunk ready: " + ready.decode(errors="replace"))
            ser.write(f"GET {seq}\n".encode())
            header = read_line(ser, deadline)
            cm = re.match(rb"LOG_CHUNK (\d+) (\d+) ([0-9A-Fa-f]+)\n", header)
            if not cm: raise RuntimeError("bad chunk header: " + header.decode(errors="replace"))
            got_seq, length, expected = int(cm.group(1)), int(cm.group(2)), int(cm.group(3), 16)
            if got_seq != seq: raise RuntimeError("unexpected chunk sequence")
            data = ser.read(length)
            if len(data) != length: raise TimeoutError("timed out receiving chunk")
            if (crc(data) & 0xffffffff) != expected:
                ser.write(f"NACK {seq}\n".encode()); raise RuntimeError(f"CRC mismatch on chunk {seq}")
            payload.extend(data); ser.write(f"ACK {seq}\n".encode()); ser.flush(); seq += 1
            print(f"received {len(payload)}/{size} bytes ({len(payload) * 100 // size}%)", flush=True)
        end = read_line(ser, deadline)
        if not end.startswith(b"LOG_CHUNK_END "):
            raise RuntimeError("missing LOG_CHUNK_END")
        with open(args.output, "wb") as out:
            out.write(payload)
        print(f"saved {len(payload)} bytes to {args.output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, TimeoutError, RuntimeError) as exc:
        print(f"dump failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
