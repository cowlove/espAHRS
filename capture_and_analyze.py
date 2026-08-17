#!/usr/bin/env python3
"""Capture a timed ESP32 session log, download it, and summarize its streams.

Requires pyserial.  The board must be running the espAHRS firmware with the
START_LOG/STOP_LOG/LIST/DUMP serial protocol.
"""
import argparse
import os
import re
import subprocess
import sys
import time

import serial

ROOT = os.path.dirname(os.path.abspath(__file__))


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
        subprocess.run([sys.executable, os.path.join(ROOT, "analyze_log.py"), output],
                       check=True)
    except (OSError, serial.SerialException, TimeoutError, RuntimeError, ValueError,
            subprocess.CalledProcessError) as exc:
        print("capture failed: %s" % exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
