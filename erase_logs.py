#!/usr/bin/env python3
"""Erase all firmware .bin log files from the ESP32 SD card."""
import argparse
import sys
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--yes", action="store_true",
                        help="confirm the destructive erase")
    args = parser.parse_args()
    if not args.yes:
        parser.error("refusing to erase without --yes")

    try:
        with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
            ser.reset_input_buffer()
            ser.write(b"FORMAT\n")
            ser.flush()
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                line = ser.readline()
                if not line:
                    continue
                text = line.decode(errors="replace").strip()
                print(text)
                if text == "SD_FORMAT OK (logs cleared)":
                    return 0
                if text.startswith("SD_FORMAT FAIL") or text.startswith("LOG_ERROR"):
                    return 1
            print("timed out waiting for SD_FORMAT response", file=sys.stderr)
            return 1
    except (OSError, serial.SerialException) as exc:
        print("erase failed: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
