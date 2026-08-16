#!/usr/bin/env python3
"""Download selected or all .bin logs from an espAHRS device."""
import argparse
import fnmatch
import os
import re
import subprocess
import sys

import serial

ROOT = os.path.dirname(os.path.abspath(__file__))


def list_logs(port, baud):
    with serial.Serial(port, baud, timeout=0.5) as ser:
        ser.reset_input_buffer()
        ser.write(b"LIST\n")
        ser.flush()
        files = []
        while True:
            line = ser.readline()
            if not line:
                continue
            text = line.decode(errors="replace").strip()
            match = re.match(r"LOG_LIST_FILE name=(\S+) size=(\d+)", text)
            if match:
                files.append((match.group(1), int(match.group(2))))
            if text == "LOG_LIST_END":
                return files
            if text.startswith("LOG_ERROR"):
                raise RuntimeError(text)


def selected(files, patterns):
    if not patterns:
        return files
    result = [(name, size) for name, size in files
              if any(fnmatch.fnmatchcase(name, pattern) for pattern in patterns)]
    requested = set(name for name, _ in result)
    unmatched = [pattern for pattern in patterns
                 if not any(fnmatch.fnmatchcase(name, pattern) for name, _ in files)]
    if unmatched:
        raise RuntimeError("no device logs matched: " + ", ".join(unmatched))
    return [(name, size) for name, size in result if name in requested]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("patterns", nargs="*", metavar="GLOB",
                    help="device filename or glob (repeatable; default: all logs)")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--output-dir", default=".",
                    help="local destination directory (default: current directory)")
    ap.add_argument("--timeout", type=float, default=300.0,
                    help="per-file transfer timeout in seconds")
    args = ap.parse_args()
    try:
        os.makedirs(args.output_dir, exist_ok=True)
        files = selected(list_logs(args.port, args.baud), args.patterns)
        if not files:
            print("device contains no .bin logs")
            return 0
        for name, size in files:
            output = os.path.join(args.output_dir, os.path.basename(name))
            print("downloading %s (%d bytes) -> %s" % (name, size, output), flush=True)
            subprocess.run([sys.executable, os.path.join(ROOT, "dump_log.py"),
                            "--port", args.port,
                            "--file", name, "--output", output,
                            "--timeout", str(args.timeout)], check=True)
        return 0
    except (OSError, serial.SerialException, RuntimeError,
            subprocess.CalledProcessError) as exc:
        print("download failed: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
