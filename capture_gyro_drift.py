#!/usr/bin/env python3
"""Capture Geek-board ten-second gyro means from every available IMU."""
import argparse, csv, os, re, sys, time
from datetime import datetime
from pathlib import Path
import serial

ROW = re.compile(r"^GYRO_DRIFT (?P<elapsed>[0-9.]+),(?P<imu>[^,]+),(?P<samples>[0-9]+),(?P<x>[-+0-9.eE]+|nan),(?P<y>[-+0-9.eE]+|nan),(?P<z>[-+0-9.eE]+|nan),(?P<t>[-+0-9.eE]+|nan)$")

def duration(value):
    m = re.fullmatch(r"\s*([0-9]+(?:\.[0-9]+)?)\s*([smh]?)\s*", value.lower())
    if not m or float(m[1]) <= 0: raise argparse.ArgumentTypeError("use seconds or a suffix such as 20m")
    return float(m[1]) * {"": 1, "s": 1, "m": 60, "h": 3600}[m[2]]

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("-d", "--duration", type=duration, default=1200.0)
    p.add_argument("-o", "--output", type=Path, default=None)
    p.add_argument("-p", "--port", default="/dev/ttyACM1")
    p.add_argument("--baud", type=int, default=115200)
    a = p.parse_args(); out = a.output or Path("gyrodrift.geek." + datetime.now().astimezone().strftime("%Y%m%d-%H%M%S") + ".csv")
    started = False
    rows = 0
    interrupted = False
    with serial.Serial(a.port, a.baud, timeout=1) as port, out.open("x", newline="", encoding="utf-8") as f:
        w = csv.writer(f); w.writerow(["elapsed_s", "imu", "samples", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "temperature_c"]); f.flush(); os.fsync(f.fileno())
        # Native USB can reconnect before setup() finishes after a reset/flash.
        # Retry the idempotent start handshake rather than silently capturing nothing.
        time.sleep(1); port.reset_input_buffer(); handshake_deadline = time.monotonic() + 8
        next_start = 0.0
        while not started and time.monotonic() < handshake_deadline:
            now = time.monotonic()
            if now >= next_start:
                port.write(b"GYRO_DRIFT_START\n"); port.flush(); next_start = now + 1
            line = port.readline().decode(errors="replace").strip()
            if line.startswith("GYRO_DRIFT STARTED") or line.startswith("GYRO_DRIFT ALREADY_STARTED"):
                started = True; print(line)
        if not started:
            raise RuntimeError("device did not acknowledge GYRO_DRIFT_START within 8 seconds")
        deadline = time.monotonic() + a.duration
        try:
            while time.monotonic() < deadline:
                line = port.readline().decode(errors="replace").strip()
                if line.startswith("GYRO_DRIFT STARTED"): started = True; print(line); continue
                m = ROW.fullmatch(line)
                if m:
                    w.writerow([m[x] for x in ("elapsed", "imu", "samples", "x", "y", "z", "t")]); f.flush(); os.fsync(f.fileno()); rows += 1; print(line)
        except KeyboardInterrupt:
            interrupted = True; print("Interrupted; retaining partial capture.", file=sys.stderr)
        finally:
            port.write(b"GYRO_DRIFT_STOP\n"); port.flush()
            end = time.monotonic() + 2
            while time.monotonic() < end:
                if port.readline().decode(errors="replace").strip().startswith("GYRO_DRIFT STOPPED"): break
    print(f"Saved {rows} averaging windows to {out}")
    return 130 if interrupted else 0

if __name__ == "__main__":
    try: raise SystemExit(main())
    except (OSError, RuntimeError, serial.SerialException) as e: print(f"capture failed: {e}", file=sys.stderr); raise SystemExit(1)
