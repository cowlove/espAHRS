#!/usr/bin/env python3
"""Download the most recent completed GEEK session log over USB serial."""
import argparse
import os
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
    ap.add_argument("--resume", action="store_true", help="resume an existing .part file")
    ap.add_argument("--attempts", type=int, default=8)
    ap.add_argument("--request-delay", type=float, default=0.005,
                    help="settling delay after each GET (seconds)")
    ap.add_argument("--transaction-timeout", type=float, default=5.0,
                    help="timeout for one DUMP/GET response")
    args = ap.parse_args()
    part = args.output + ".part"
    if not args.resume and os.path.exists(part): os.remove(part)
    # Must match the firmware for calculating a resume sequence before the
    # device sends LOG_CHUNK_BEGIN. A .part from an older block size should
    # be removed before starting a new transfer.
    started = time.monotonic(); size = None; chunk_size = 4096
    for attempt in range(args.attempts):
        if time.monotonic() - started >= args.timeout: break
        existing = os.path.getsize(part) if os.path.exists(part) else 0
        if existing % chunk_size: raise RuntimeError("partial file is not chunk aligned")
        seq = existing // chunk_size
        try:
            with serial.Serial(args.port, 115200, timeout=2) as ser:
                overall_deadline = started + args.timeout
                # A reconnect can leave status or a previous transfer's terminal
                # error queued in USB CDC. Only responses to this session count.
                ser.reset_input_buffer()
                trace = open(args.trace, "a") if args.trace else None
                def log(direction, data):
                    if trace: trace.write(f"{direction} {data!r}\n"); trace.flush()
                command = f"DUMP {seq}\n".encode() if seq else b"DUMP\n"
                ser.write(command); ser.flush(); log("TX", command)
                line = read_protocol_line(ser, b"LOG_CHUNK_BEGIN ",
                                          min(overall_deadline, time.monotonic() + 10.0))
                match = re.match(rb"LOG_CHUNK_BEGIN (\d+) (\d+)\n", line)
                if not match: raise RuntimeError("bad response: " + line.decode(errors="replace"))
                size, chunk_size = int(match.group(1)), int(match.group(2))
                if existing != seq * chunk_size: raise RuntimeError("resume chunk size changed")
                with open(part, "ab") as out:
                    crc = __import__('zlib').crc32
                    while existing < size:
                        request = f"GET {seq}\n".encode()
                        ser.write(request); ser.flush()
                        if args.request_delay: time.sleep(args.request_delay)
                        log("TX", request)
                        transaction_deadline = min(overall_deadline,
                                                   time.monotonic() + args.transaction_timeout)
                        read_protocol_line(ser, b"LOG_GET_OK ", transaction_deadline)
                        header = read_line(ser, transaction_deadline)
                        cm = re.match(rb"LOG_CHUNK (\d+) (\d+) ([0-9A-Fa-f]+)\n", header)
                        if not cm: raise RuntimeError("bad chunk header")
                        got, length, expected = int(cm.group(1)), int(cm.group(2)), int(cm.group(3), 16)
                        if got != seq: raise RuntimeError("unexpected chunk sequence")
                        data = read_exact(ser, length, transaction_deadline)
                        if (crc(data) & 0xffffffff) != expected: raise RuntimeError(f"CRC mismatch chunk {seq}")
                        out.write(data); out.flush(); existing += length; seq += 1
                        if existing % (256 * 1024) < length or existing == size: print(f"received {existing}/{size} bytes ({existing * 100 // size}%)", flush=True)
                end = read_line(ser, min(overall_deadline,
                                         time.monotonic() + args.transaction_timeout))
                if not end.startswith(b"LOG_CHUNK_END "): raise RuntimeError("missing LOG_CHUNK_END")
                if trace: trace.close()
                os.replace(part, args.output); print(f"saved {existing} bytes to {args.output}"); return 0
        except (OSError, TimeoutError, RuntimeError) as exc:
            print(f"attempt {attempt + 1}/{args.attempts} paused at chunk {seq}: {exc}", file=sys.stderr)
            time.sleep(1)
    raise TimeoutError("resume attempts exhausted; partial file retained as " + part)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, TimeoutError, RuntimeError) as exc:
        print(f"dump failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
