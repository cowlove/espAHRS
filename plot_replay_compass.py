#!/usr/bin/env python3
"""Plot GPS ground track and both replayed compass headings.

Example:
  ./plot_replay_compass.py flight-data-latest/20260818/G247C021.bin \
      --output analysis-plots/G247C021-gps-compass-heading.png

The default magnitude tolerance is deliberately wide so rejected compass
samples remain visible while investigating continuity.  This is a replay
diagnostic and does not change firmware configuration.
"""

import argparse
import csv
import os
import re
import subprocess
import sys
import tempfile


def read_heading_csv(path):
    gps_t, gps = [], []
    compass_t, compass = [], []
    with open(path, newline="") as stream:
        for row in csv.DictReader(stream):
            t = float(row["time_s"])
            if int(row["gps_valid"]):
                gps_t.append(t)
                gps.append(float(row["gps_track_deg"]))
            if int(row["compass_valid"]):
                compass_t.append(t)
                compass.append(float(row["compass_heading_deg"]))
    return gps_t, gps, compass_t, compass


def unwrap_with_gaps(times, values, gap_sec=0.20, max_step_deg=20.0):
    """Unwrap heading without letting impossible bursts slip a 360-deg branch.

    Compass samples arrive at roughly 50 Hz, so a change larger than 20 degrees
    between adjacent samples cannot be aircraft motion.  Break the plotted line
    and retain the last stable unwrap branch until the raw heading returns to a
    physically plausible neighborhood.
    """
    if not values:
        return [], []
    out_t = [times[0]]
    out_y = [0.0]
    stable_raw = values[0]
    stable_unwrapped = 0.0
    previous_time = times[0]
    line_broken = False
    recovery_raw = None
    recovery_count = 0
    for time, value in zip(times[1:], values[1:]):
        delta = (value - stable_raw + 180.0) % 360.0 - 180.0
        implausible = abs(delta) > max_step_deg
        gap = time - previous_time > gap_sec
        if gap or implausible:
            if not line_broken:
                out_t.append(float("nan"))
                out_y.append(float("nan"))
                line_broken = True
            recovery_delta = (value - recovery_raw + 180.0) % 360.0 - 180.0 \
                if recovery_raw is not None else 0.0
            recovery_count = recovery_count + 1 \
                if recovery_raw is not None and abs(recovery_delta) <= max_step_deg \
                else 1
            recovery_raw = value
            if not gap and recovery_count >= 5:
                # The signal has settled again.  Resume on the equivalent
                # branch nearest the last trustworthy point, without folding
                # the rejected burst into the cumulative heading change.
                recovery_from_stable = \
                    (value - stable_raw + 180.0) % 360.0 - 180.0
                stable_unwrapped += recovery_from_stable
                stable_raw = value
                out_t.extend((float("nan"), time))
                out_y.extend((float("nan"), stable_unwrapped))
                line_broken = False
                recovery_raw = None
                recovery_count = 0
            previous_time = time
            continue

        stable_unwrapped += delta
        stable_raw = value
        recovery_raw = None
        recovery_count = 0
        if line_broken:
            out_t.append(float("nan"))
            out_y.append(float("nan"))
            line_broken = False
        out_t.append(time)
        out_y.append(stable_unwrapped)
        previous_time = time
    return out_t, out_y


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bin_file", help="binary FusionSession log")
    parser.add_argument("--output", help="save PNG instead of displaying it")
    parser.add_argument("--replay", help="replay executable (default: ./replay)")
    parser.add_argument("--device-mac", help="MAC or suffix for legacy logs")
    parser.add_argument("--hal", default="geek", choices=("geek", "tbeam"))
    parser.add_argument(
        "--magnitude-tolerance", type=float, default=-1.0,
        help="accepted absolute field-magnitude deviation from 1.0; "
             "negative disables the gate (default: disabled; use 0.20 "
             "to inspect the former production gate)")
    args = parser.parse_args()

    if not os.path.isfile(args.bin_file):
        parser.error(f"binary log file not found: {args.bin_file}")
    replay = args.replay or os.path.join(os.path.dirname(__file__), "replay")
    if not os.path.isfile(replay):
        parser.error(f"replay executable not found: {replay}; run `make replay` first")

    device_mac = args.device_mac
    if not device_mac:
        match = re.fullmatch(r"[GT]([0-9A-Fa-f]{4})[0-9]+\.bin",
                             os.path.basename(args.bin_file))
        if match:
            device_mac = match.group(1)

    import matplotlib.pyplot as plt

    with tempfile.TemporaryDirectory(prefix="replay-compass-") as temp:
        traces = {}
        for source in (0, 1):
            csv_path = os.path.join(temp, f"compass{source}.csv")
            command = [replay, args.bin_file, "--hal", args.hal,
                       "--param", f"compass_source={source}",
                       "--param",
                       f"magnetic_field_magnitude_tolerance={args.magnitude_tolerance}",
                       "--heading-csv", csv_path]
            if device_mac:
                command += ["--device-mac", device_mac]
            result = subprocess.run(command, text=True, capture_output=True)
            if result.returncode:
                sys.stderr.write(result.stdout)
                sys.stderr.write(result.stderr)
                return result.returncode
            traces[source] = read_heading_csv(csv_path)

    gps_t, gps, _, _ = traces[0]
    fig, ax = plt.subplots(figsize=(13, 5.5))
    if gps:
        gps_plot_t, gps_plot_y = unwrap_with_gaps(gps_t, gps)
        ax.plot(gps_plot_t, gps_plot_y, label="GPS ground-track change",
                color="tab:blue", linewidth=1.2)
    for source, color in ((0, "tab:orange"), (1, "tab:green")):
        _, _, compass_t, compass = traces[source]
        if compass:
            compass_plot_t, compass_plot_y = unwrap_with_gaps(compass_t, compass)
            ax.plot(compass_plot_t, compass_plot_y,
                    label=f"Compass {source}", color=color, linewidth=0.8)
    ax.axhline(0.0, color="black", linewidth=0.4)
    gate_label = ("(magnitude gate disabled)" if args.magnitude_tolerance < 0 else
                  f"(magnitude gate ±{args.magnitude_tolerance:g}00%)")
    ax.set_title(
        f"{os.path.basename(args.bin_file)}: GPS track and compass headings "
        f"{gate_label}")
    ax.set_xlabel("log time (s)")
    ax.set_ylabel("heading change (degrees; arbitrary initial offset)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    if args.output:
        fig.savefig(args.output, dpi=150)
        print(f"saved {args.output}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
