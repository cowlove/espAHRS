#!/usr/bin/env python3
"""Analyze a downloaded espAHRS binary log."""
import argparse
import struct
from collections import defaultdict

TYPE_NAMES = {1: "EVENT", 2: "G5_RAW", 3: "G5_PACKET", 4: "IMU0", 5: "GPS",
              6: "BARO", 7: "COMPASS0", 8: "COMPASS1", 9: "IMU1",
              10: "IMU2", 11: "IMU3", 12: "COMPASS2", 13: "COMPASS3",
              14: "METADATA"}
HEADER = struct.Struct("<I4xQI B3x I4x")


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
    ap.add_argument("path", help="downloaded .bin log to analyze")
    args = ap.parse_args()
    try:
        analyze(args.path)
    except (OSError, ValueError) as exc:
        ap.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
