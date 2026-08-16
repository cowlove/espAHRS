#!/usr/bin/env python3
"""Capture a timed ESP32 session log, download it, and summarize its streams.

Requires pyserial.  The board must be running the espAHRS firmware with the
START_LOG/STOP_LOG/LIST/DUMP serial protocol.
"""
import argparse
import os
import re
import struct
import subprocess
import sys
import time
from collections import defaultdict

import serial

ROOT = os.path.dirname(os.path.abspath(__file__))
TYPE_NAMES = {1: "EVENT", 2: "G5_RAW", 3: "G5_PACKET", 4: "IMU0", 5: "GPS",
              6: "BARO", 7: "COMPASS0", 8: "COMPASS1", 9: "IMU1",
              10: "IMU2", 11: "IMU3", 12: "COMPASS2", 13: "COMPASS3",
              14: "METADATA"}
# Native C++ layout: four bytes of padding before timestamp and trailing
# padding after payloadLength (the firmware asserts sizeof == 32).
HEADER = struct.Struct("<I4xQI B3x I4x")


def command(ser, text, timeout=5):
    ser.reset_input_buffer()
    ser.write((text + "\n").encode()); ser.flush()
    end = time.monotonic() + timeout
    lines = []
    while time.monotonic() < end:
        line = ser.readline()
        if not line:
            continue
        line = line.decode(errors="replace").strip()
        lines.append(line)
        if (line.startswith("LOG_ERROR") or line.startswith("SESSION_LOG ") or
                line.startswith("LOG_LIST_END")):
            return lines
    raise TimeoutError("timed out waiting for " + text)


def list_logs(ser):
    lines = command(ser, "LIST")
    files = []
    for line in lines:
        m = re.match(r"LOG_LIST_FILE name=(\S+) size=(\d+)", line)
        if m:
            files.append((m.group(1), int(m.group(2))))
    # An empty card is a valid starting state; FORMAT can intentionally leave
    # it this way before the first capture.
    return dict(files)


def select_captured_log(before, after):
    changed = [name for name, size in after.items()
               if name not in before or before[name] != size]
    if len(changed) == 1:
        return changed[0]
    if not changed:
        raise RuntimeError("capture produced no new or changed log file")
    raise RuntimeError("capture produced multiple changed log files: " + ", ".join(changed))


def analyze(path):
    samples = defaultdict(list)
    counts = defaultdict(int)
    with open(path, "rb") as f:
        while True:
            raw = f.read(HEADER.size)
            if not raw:
                break
            if len(raw) != HEADER.size:
                raise ValueError("truncated record header")
            magic, timestamp, sequence, typ, length = HEADER.unpack(raw)
            if magic != 0x31474F4C:
                raise ValueError("bad record magic at byte %d" % (f.tell() - HEADER.size))
            payload = f.read(length)
            if len(payload) != length:
                raise ValueError("truncated payload")
            name = TYPE_NAMES.get(typ, "TYPE_%d" % typ)
            counts[name] += 1
            # G5 payloads are variable-length, so their record-header time is
            # the stream timestamp. Other sensor payloads carry their own
            # timestamp (GPS is milliseconds; the rest are microseconds).
            if typ in (2, 3):
                samples[name].append(timestamp / 1_000_000.0)
            elif typ in (4, 5, 6, 7, 8, 9, 10, 11, 12, 13) and len(payload) >= 4:
                ts = struct.unpack_from("<I" if typ == 5 else "<Q", payload)[0]
                samples[name].append(ts / (1000.0 if typ == 5 else 1_000_000.0))
    print("Log: %s" % path)
    print("Sensors/streams present: " + (", ".join(sorted(counts)) or "none"))
    print("Records: %d" % sum(counts.values()))
    for name in sorted(counts):
        if name not in samples or len(samples[name]) < 2:
            print("  %-10s %6d records (insufficient timestamps)" % (name, counts[name]))
            continue
        values = samples[name]
        gaps = [b - a for a, b in zip(values, values[1:]) if b >= a]
        duration = values[-1] - values[0]
        rate = (len(values) - 1) / duration if duration > 0 else 0
        nominal = sorted(gaps)[len(gaps) // 2]
        threshold = max(0.1, nominal * 2.5)
        large = [g for g in gaps if g > threshold]
        print("  %-10s %6d records, %7.2f Hz, span %7.2fs, gaps %d (max %.3fs)" %
              (name, len(values), rate, duration, len(large), max(gaps)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--output", default=None, help="download path (default: capture-<time>.bin)")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()
    if args.duration <= 0:
        ap.error("--duration must be positive")
    output = args.output or "capture-%s.bin" % time.strftime("%Y%m%d-%H%M%S")
    try:
        with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
            before = list_logs(ser)
            started = command(ser, "START_LOG")
            if not any(x == "SESSION_LOG STARTED" for x in started):
                raise RuntimeError("start failed: " + " | ".join(started))
            print("recording for %.1f seconds..." % args.duration, flush=True)
            time.sleep(args.duration)
            stopped = command(ser, "STOP_LOG", timeout=10)
            if not any(x.startswith("SESSION_LOG STOPPED") for x in stopped):
                raise RuntimeError("stop failed: " + " | ".join(stopped))
            filename = select_captured_log(before, list_logs(ser))
        print("downloading %s..." % filename, flush=True)
        subprocess.run([sys.executable, os.path.join(ROOT, "dump_log.py"),
                        "--port", args.port, "--file", filename, "--output", output],
                       check=True)
        analyze(output)
    except (OSError, serial.SerialException, TimeoutError, RuntimeError, ValueError,
            subprocess.CalledProcessError) as exc:
        print("capture failed: %s" % exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
