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
        line = read_header(ser, deadline)
        match = re.match(rb"LOG_BEGIN (\S+) (\d+)\n", line)
        if not match:
            raise RuntimeError("bad response: " + line.decode(errors="replace"))
        name, size = match.group(1).decode(), int(match.group(2))
        payload = read_exact(ser, size, deadline)
        end = ser.readline()
        if end == b"\n":
            end = ser.readline()
        if not end.startswith(b"LOG_END "):
            raise RuntimeError("missing LOG_END")
        with open(args.output, "wb") as out:
            out.write(payload)
        print(f"saved {len(payload)} bytes from {name} to {args.output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, TimeoutError, RuntimeError) as exc:
        print(f"dump failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
