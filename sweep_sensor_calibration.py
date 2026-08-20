#!/usr/bin/env python3
"""Estimate sensor offsets and initial gyro biases from one level-flight log.

Despite the historical filename, this performs no grid sweep.  Each parameter
is solved from a signed mean error and one small sensitivity probe.  Adaptive
gyro-bias learning is disabled so it cannot hide the fixed calibration error.
"""

import argparse
import csv
import json
import math
import os
import subprocess
import tempfile


def replay_rows(replay, log, device_mac, params, csv_kind, replay_options,
                start_time=None, end_time=None):
    with tempfile.TemporaryDirectory(prefix="sensor-calibration-") as directory:
        output = os.path.join(directory, csv_kind + ".csv")
        command = [replay, log, "--hal", "geek", "--device-mac", device_mac,
                   "--" + csv_kind + "-csv", output] + replay_options
        for name, value in params.items():
            command += ["--param", f"{name}={value:.9f}"]
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
        with open(output, newline="") as stream:
            rows = list(csv.DictReader(stream))
        if start_time is None and end_time is None:
            return rows
        selected = []
        for row in rows:
            try:
                time_s = float(row["time_s"])
            except (KeyError, TypeError, ValueError):
                continue
            if start_time is not None and time_s < start_time:
                continue
            if end_time is not None and time_s > end_time:
                continue
            selected.append(row)
        return selected


def stable_yaw_intervals(replay, log, mac, params, replay_options,
                         maximum_gps_bank_deg, minimum_duration_sec):
    """Find sustained straight segments using GPS-derived bank, not gyro Z."""
    rows = replay_rows(replay, log, mac, params, "roll", replay_options)
    qualifying_times = []
    for row in rows:
        try:
            time_s = float(row["time_s"])
            gps_bank = float(row["gps_turn_rate_bank_deg"])
            gps_bank_valid = int(row["gps_turn_rate_bank_valid"])
        except (KeyError, TypeError, ValueError):
            continue
        if gps_bank_valid and math.isfinite(time_s) and math.isfinite(gps_bank) and abs(
                gps_bank) <= maximum_gps_bank_deg:
            qualifying_times.append(time_s)

    # The roll trace contains an entry at each IMU update.  Split at gaps or
    # rejected samples, then retain only sustained straight portions.
    intervals = []
    start = previous = None
    for time_s in qualifying_times:
        if previous is None or time_s - previous > 0.1:
            if (start is not None and
                    previous - start >= minimum_duration_sec):
                intervals.append((start, previous))
            start = time_s
        previous = time_s
    if (start is not None and previous - start >= minimum_duration_sec):
        intervals.append((start, previous))
    if not intervals:
        raise RuntimeError(
            "no stable yaw intervals; relax --yaw-max-gps-bank-deg or "
            "--yaw-min-stable-sec")
    return intervals


def rows_in_intervals(rows, intervals):
    selected = []
    interval_index = 0
    for row in rows:
        try:
            time_s = float(row["time_s"])
        except (KeyError, TypeError, ValueError):
            continue
        while (interval_index < len(intervals) and
               time_s > intervals[interval_index][1]):
            interval_index += 1
        if (interval_index < len(intervals) and
                intervals[interval_index][0] <= time_s):
            selected.append(row)
    return selected


def signed_mean(rows, left, right, required=None):
    values = []
    for row in rows:
        try:
            if required is not None and not math.isfinite(float(row[required])):
                continue
            value = float(row[left]) - float(row[right])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    if not values:
        raise RuntimeError(f"no finite {left}/{right} samples")
    return sum(values) / len(values), len(values)


def solve_parameter(replay, log, mac, params, name, csv_kind, left, right,
                    probe, replay_options, required=None,
                    maximum_magnitude=None):
    base_rows = replay_rows(replay, log, mac, params, csv_kind, replay_options)
    base_error, count = signed_mean(base_rows, left, right, required)
    trial = dict(params)
    trial[name] += probe
    probe_rows = replay_rows(replay, log, mac, trial, csv_kind, replay_options)
    probe_error, _ = signed_mean(probe_rows, left, right, required)
    sensitivity = (probe_error - base_error) / probe
    if not math.isfinite(sensitivity) or abs(sensitivity) < 1.0e-4:
        raise RuntimeError(f"{name} has unusable sensitivity {sensitivity}")
    estimate = params[name] - base_error / sensitivity
    if maximum_magnitude is not None and abs(estimate) > maximum_magnitude:
        raise RuntimeError(
            f"{name} estimate {estimate:.3f} exceeds physical guard "
            f"+/-{maximum_magnitude:.3f}; cadence/model mismatch likely")
    params[name] = estimate
    check_rows = replay_rows(replay, log, mac, params, csv_kind, replay_options)
    residual, _ = signed_mean(check_rows, left, right, required)
    print(f"{name}: estimate={estimate:.6f} base_error={base_error:.6f} "
          f"sensitivity={sensitivity:.6f} residual={residual:.6f} samples={count}")


def solve_linear_3x3(matrix, vector):
    """Solve a small dense system with pivoted Gaussian elimination."""
    augmented = [list(row) + [value]
                 for row, value in zip(matrix, vector)]
    for column in range(3):
        pivot = max(range(column, 3),
                    key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1.0e-5:
            raise RuntimeError("gyro-bias sensitivity matrix is singular")
        augmented[column], augmented[pivot] = (
            augmented[pivot], augmented[column])
        divisor = augmented[column][column]
        augmented[column] = [value / divisor
                             for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [left - factor * right for left, right in zip(
                augmented[row], augmented[column])]
    return [augmented[row][3] for row in range(3)]


def solve_gyro_biases(replay, log, mac, params, probe, replay_options,
                      stable_intervals):
    columns = ("raw_gyro_x_deg_sec", "raw_gyro_y_deg_sec",
               "raw_gyro_z_deg_sec")

    def means(rows):
        values = [[], [], []]
        for row in rows:
            for axis, column in enumerate(columns):
                try:
                    value = float(row[column])
                except (KeyError, TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    values[axis].append(value)
        if any(not axis_values for axis_values in values):
            raise RuntimeError("no finite three-axis gyro samples")
        return [sum(v) / len(v) for v in values], min(map(len, values))

    full_base, full_count = means(replay_rows(
        replay, log, mac, params, "imu", replay_options))
    window_base, window_count = means(rows_in_intervals(replay_rows(
        replay, log, mac, params, "imu", replay_options), stable_intervals))
    # Keep the well-observed pitch/roll estimates based on the whole level-
    # flight run.  Only yaw uses the optional, more strictly straight window.
    base = [full_base[0], full_base[1], window_base[2]]
    sensitivity = [[0.0] * 3 for _ in range(3)]
    names = ("gyro_bias_x_deg_sec", "gyro_bias_y_deg_sec",
             "gyro_bias_z_deg_sec")
    for raw_axis, name in enumerate(names):
        trial = dict(params)
        trial[name] += probe
        full_shifted, _ = means(replay_rows(
            replay, log, mac, trial, "imu", replay_options))
        window_shifted, _ = means(rows_in_intervals(replay_rows(
            replay, log, mac, trial, "imu", replay_options),
            stable_intervals))
        shifted = [full_shifted[0], full_shifted[1], window_shifted[2]]
        for body_axis in range(3):
            sensitivity[body_axis][raw_axis] = (
                shifted[body_axis] - base[body_axis]) / probe

    deltas = solve_linear_3x3(sensitivity, [-value for value in base])
    for name, delta in zip(names, deltas):
        params[name] += delta
    full_residual, _ = means(replay_rows(
        replay, log, mac, params, "imu", replay_options))
    window_residual, _ = means(rows_in_intervals(replay_rows(
        replay, log, mac, params, "imu", replay_options), stable_intervals))
    residual = [full_residual[0], full_residual[1], window_residual[2]]
    print("gyro_bias_xyz_deg_sec: "
          f"estimate=[{','.join(f'{params[name]:.6f}' for name in names)}] "
          f"body_rate_mean=[{','.join(f'{value:.6f}' for value in base)}] "
          f"residual=[{','.join(f'{value:.6f}' for value in residual)}] "
          f"samples_xy={full_count} samples_z={window_count}")


def imu_cadence(replay, log, mac, params, replay_options):
    rows = replay_rows(replay, log, mac, params, "imu", replay_options)
    values = []
    for row in rows:
        try:
            value = float(row["dt_s"])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value) and value > 0:
            values.append(value)
    values.sort()
    if not values:
        return None
    return sum(values) / len(values), values[len(values) // 2], len(values)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument("--replay", default="./replay")
    parser.add_argument("--device-mac", required=True,
                        help="four-digit suffix, e.g. 247C")
    parser.add_argument("--offset-probe-deg", type=float, default=1.0)
    parser.add_argument("--bias-probe-deg-sec", type=float, default=0.5)
    parser.add_argument("--yaw-max-gps-bank-deg", type=float, default=1.0,
                        help="maximum absolute GPS turn-rate bank for stable "
                             "yaw samples (default: 1.0)")
    parser.add_argument("--yaw-min-stable-sec", type=float, default=5.0,
                        help="minimum continuous stable yaw segment "
                             "duration (default: 5.0)")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--axis-remap", nargs=9, type=float)
    parser.add_argument("--gyro-axis-remap", nargs=9, type=float)
    args = parser.parse_args()
    if args.yaw_max_gps_bank_deg < 0:
        parser.error("--yaw-max-gps-bank-deg must be non-negative")
    if args.yaw_min_stable_sec <= 0:
        parser.error("--yaw-min-stable-sec must be positive")
    if not os.path.isfile(args.log):
        parser.error("log file not found")
    if not os.path.isfile(args.replay):
        parser.error("replay executable not found; run make replay")

    params = {
        "sensor_pitch_offset_deg": 0.0,
        "sensor_roll_offset_deg": 0.0,
        "gyro_bias_x_deg_sec": 0.0,
        "gyro_bias_y_deg_sec": 0.0,
        "gyro_bias_z_deg_sec": 0.0,
        "adaptive_gyro_bias_enabled": 0.0,
    }
    replay_options = []
    if args.axis_remap:
        replay_options += ["--axis-remap", *map(str, args.axis_remap)]
    if args.gyro_axis_remap:
        replay_options += ["--gyro-axis-remap", *map(str, args.gyro_axis_remap)]

    cadence = imu_cadence(args.replay, args.log, args.device_mac, params,
                          replay_options)
    if cadence:
        mean_dt, median_dt, count = cadence
        print(f"imu_cadence: mean_dt={mean_dt:.6f}s median_dt={median_dt:.6f}s "
              f"mean_rate={1.0/mean_dt:.2f}Hz samples={count}")
        if abs(mean_dt - 0.020) > 0.002:
            print("WARNING: recorded IMU cadence differs from the replay's "
                  "fixed 0.020 s gyro integration interval")

    # Installed-sensor offsets align independent correction targets to G5.
    solve_parameter(args.replay, args.log, args.device_mac, params,
                    "sensor_pitch_offset_deg", "pitch",
                    "pitch_correction_target_deg", "g5_pitch",
                    args.offset_probe_deg, replay_options)
    solve_parameter(args.replay, args.log, args.device_mac, params,
                    "sensor_roll_offset_deg", "roll",
                    "roll_correction_target_deg", "g5_roll",
                    args.offset_probe_deg, replay_options)
    # Roll mounting rotation has a small second-order effect on the pitch
    # target.  Revisit pitch once after roll so the final pair, rather than
    # only the intermediate pitch estimate, has zero signed reference error.
    solve_parameter(args.replay, args.log, args.device_mac, params,
                    "sensor_pitch_offset_deg", "pitch",
                    "pitch_correction_target_deg", "g5_pitch",
                    args.offset_probe_deg, replay_options)

    # A level-flight run has approximately zero mean pitch/roll body rate.  The
    # yaw estimate is flight-derived (not a stationary cold-boot calibration),
    # so its zero-mean constraint may use a more strictly straight window and
    # should be validated on independent maneuver logs.
    # Solve the fixed raw-axis biases from that physical constraint.  This is
    # independent of AHRS correction-loop equilibria and integration cadence;
    # the probe captures raw-axis polarity/remapping.
    yaw_intervals = stable_yaw_intervals(
        args.replay, args.log, args.device_mac, params, replay_options,
        args.yaw_max_gps_bank_deg, args.yaw_min_stable_sec)
    stable_duration = sum(end - start for start, end in yaw_intervals)
    print(f"yaw_stable_segments: count={len(yaw_intervals)} "
          f"duration={stable_duration:.1f}s "
          f"max_gps_bank={args.yaw_max_gps_bank_deg:.2f}deg")
    solve_gyro_biases(args.replay, args.log, args.device_mac, params,
                      args.bias_probe_deg_sec, replay_options, yaw_intervals)

    result = {name: value for name, value in params.items()
              if name != "adaptive_gyro_bias_enabled"}
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print("best-fit calibration:")
        for name, value in result.items():
            print(f"  {name} = {value:.6f}")


if __name__ == "__main__":
    main()
