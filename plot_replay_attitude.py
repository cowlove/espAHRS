#!/usr/bin/env python3
"""Run replay and plot G5 versus AHRS pitch and roll.

Example:
  ./plot_replay_attitude.py flight.bin --param roll_correction_sec==6 --param gps_heading_weight=2
"""

import argparse
import csv
import os
import subprocess
import sys
import tempfile


def read_column(path, name):
    with open(path, newline="") as stream:
        rows = csv.DictReader(stream)
        if not rows.fieldnames or name not in rows.fieldnames:
            raise RuntimeError(f"{path} has no column {name!r}")
        values = []
        for row in rows:
            try:
                values.append(float(row[name]))
            except (TypeError, ValueError):
                values.append(float("nan"))
        return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bin_file", help="binary FusionSession log")
    parser.add_argument("--param", action="append", default=[],
                        metavar="NAME==VALUE",
                        help="replay parameter; repeat for multiple parameters")
    parser.add_argument("--hal", default="geek", choices=("geek", "tbeam"))
    parser.add_argument("--device-mac",
                        help="required for legacy logs without identity metadata")
    parser.add_argument("--replay", default=None,
                        help="path to replay executable (default: ./replay)")
    parser.add_argument("--output", help="save PNG instead of only displaying it")
    args = parser.parse_args()

    replay = args.replay or os.path.join(os.path.dirname(__file__), "replay")
    if not os.path.isfile(replay):
        parser.error(f"replay executable not found: {replay}; run `make replay` first")

    # Accept the requested NAME==VALUE spelling as well as replay's NAME=VALUE.
    params = []
    for item in args.param:
        if "==" in item:
            name, value = item.split("==", 1)
        elif "=" in item:
            name, value = item.split("=", 1)
        else:
            parser.error(f"invalid --param {item!r}; expected NAME==VALUE")
        if not name or not value:
            parser.error(f"invalid --param {item!r}; expected NAME==VALUE")
        params += ["--param", f"{name}={value}"]

    import matplotlib.pyplot as plt

    with tempfile.TemporaryDirectory(prefix="replay-attitude-") as temp:
        pitch_csv = os.path.join(temp, "pitch.csv")
        roll_csv = os.path.join(temp, "roll.csv")
        command = [replay, args.bin_file, "--hal", args.hal,
                   "--pitch-csv", pitch_csv, "--roll-csv", roll_csv] + params
        if args.device_mac:
            command += ["--device-mac", args.device_mac]
        result = subprocess.run(command, text=True, capture_output=True)
        if result.returncode:
            sys.stderr.write(result.stdout)
            sys.stderr.write(result.stderr)
            return result.returncode

        time_pitch = read_column(pitch_csv, "time_s")
        g5_pitch = read_column(pitch_csv, "g5_pitch")
        ahrs_pitch = read_column(pitch_csv, "ahrs_pitch")
        time_roll = read_column(roll_csv, "time_s")
        g5_roll = read_column(roll_csv, "g5_roll")
        ahrs_roll = read_column(roll_csv, "ahrs_roll")

    fig, (pitch_ax, roll_ax) = plt.subplots(2, 1, sharex=False, figsize=(12, 8),
                                             constrained_layout=True)
    pitch_ax.plot(time_pitch, g5_pitch, label="G5 pitch", linewidth=1.0)
    pitch_ax.plot(time_pitch, ahrs_pitch, label="AHRS pitch", linewidth=1.0)
    pitch_ax.set_title("G5 pitch and AHRS pitch")
    pitch_ax.set_ylabel("degrees")
    pitch_ax.grid(True, alpha=0.3)
    pitch_ax.legend()

    roll_ax.plot(time_roll, g5_roll, label="G5 roll", linewidth=1.0)
    roll_ax.plot(time_roll, ahrs_roll, label="AHRS roll", linewidth=1.0)
    roll_ax.set_title("G5 roll and AHRS roll")
    roll_ax.set_xlabel("log time (s)")
    roll_ax.set_ylabel("degrees")
    roll_ax.grid(True, alpha=0.3)
    roll_ax.legend()

    fig.suptitle(os.path.basename(args.bin_file))
    if args.output:
        fig.savefig(args.output, dpi=150)
        print(f"saved {args.output}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
