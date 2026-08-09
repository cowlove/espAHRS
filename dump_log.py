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
        if size > 4096 and (len(data) >= next_report or len(data) == size):
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
    ap.add_argument("--trace", help="write protocol trace lines to this file")
    args = ap.parse_args()

    with serial.Serial(args.port, 115200, timeout=2) as ser:
        trace = open(args.trace, "w") if args.trace else None
        def log(direction, data):
            if trace:
                trace.write(f"{direction} {data!r}\n"); trace.flush()
        deadline = time.monotonic() + args.timeout
        ser.write(b"DUMP\n")
        log("TX", b"DUMP\\n")
        line = read_protocol_line(ser, b"LOG_CHUNK_BEGIN ", deadline)
        match = re.match(rb"LOG_CHUNK_BEGIN (\d+) (\d+)\n", line)
        if not match:
            raise RuntimeError("bad response: " + line.decode(errors="replace"))
        size, chunk_size = int(match.group(1)), int(match.group(2))
        payload = bytearray(); seq = 0
        crc = __import__('zlib').crc32
        while len(payload) < size:
            header = None
            for attempt in range(4):
                ser.write(f"GET {seq}\n".encode()); ser.flush(); time.sleep(0.10)
                log("TX", f"GET {seq}\\n".encode())
                marker = read_protocol_line(ser, b"LOG_GET_OK ", deadline)
                if not marker.startswith(f"LOG_GET_OK {seq}".encode()):
                    continue
                try:
                    header = read_line(ser, deadline)
                    break
                except TimeoutError:
                    if attempt == 3: raise
            if header is None:
                raise TimeoutError(f"no response to GET {seq}")
            cm = re.match(rb"LOG_CHUNK (\d+) (\d+) ([0-9A-Fa-f]+)\n", header)
            if not cm: raise RuntimeError("bad chunk header: " + header.decode(errors="replace"))
            got_seq, length, expected = int(cm.group(1)), int(cm.group(2)), int(cm.group(3), 16)
            if got_seq != seq: raise RuntimeError("unexpected chunk sequence")
            data = read_exact(ser, length, deadline)
            if (crc(data) & 0xffffffff) != expected:
                raise RuntimeError(f"CRC mismatch on chunk {seq}")
            payload.extend(data); seq += 1
            if len(payload) % (256 * 1024) < length or len(payload) == size:
                print(f"received {len(payload)}/{size} bytes ({len(payload) * 100 // size}%)", flush=True)
        end = read_line(ser, deadline)
        if not end.startswith(b"LOG_CHUNK_END "):
            raise RuntimeError("missing LOG_CHUNK_END")
        with open(args.output, "wb") as out:
            out.write(payload)
        if trace: trace.close()
        print(f"saved {len(payload)} bytes to {args.output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, TimeoutError, RuntimeError) as exc:
        print(f"dump failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
