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


def replay_rows(replay, log, device_mac, params, csv_kind, replay_options):
    with tempfile.TemporaryDirectory(prefix="sensor-calibration-") as directory:
        output = os.path.join(directory, csv_kind + ".csv")
        command = [replay, log, "--hal", "geek", "--device-mac", device_mac,
                   "--" + csv_kind + "-csv", output] + replay_options
        for name, value in params.items():
            command += ["--param", f"{name}={value:.9f}"]
        subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
        with open(output, newline="") as stream:
            return list(csv.DictReader(stream))


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


def solve_gyro_biases(replay, log, mac, params, probe, replay_options):
    columns = ("raw_gyro_x_deg_sec", "raw_gyro_y_deg_sec")

    def means(rows):
        values = [[], []]
        for row in rows:
            for axis, column in enumerate(columns):
                try:
                    value = float(row[column])
                except (KeyError, TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    values[axis].append(value)
        if not values[0] or not values[1]:
            raise RuntimeError("no finite horizontal gyro samples")
        return [sum(v) / len(v) for v in values], min(map(len, values))

    base, count = means(replay_rows(
        replay, log, mac, params, "imu", replay_options))
    sensitivity = [[0.0, 0.0], [0.0, 0.0]]
    names = ("gyro_bias_x_deg_sec", "gyro_bias_y_deg_sec")
    for raw_axis, name in enumerate(names):
        trial = dict(params)
        trial[name] += probe
        shifted, _ = means(replay_rows(
            replay, log, mac, trial, "imu", replay_options))
        for body_axis in range(2):
            sensitivity[body_axis][raw_axis] = (
                shifted[body_axis] - base[body_axis]) / probe

    a, b = sensitivity[0]
    c, d = sensitivity[1]
    determinant = a * d - b * c
    if abs(determinant) < 1.0e-5:
        raise RuntimeError("horizontal gyro-bias sensitivity is singular")
    delta_x = (-base[0] * d + b * base[1]) / determinant
    delta_y = (-a * base[1] + base[0] * c) / determinant
    params[names[0]] += delta_x
    params[names[1]] += delta_y
    residual, _ = means(replay_rows(
        replay, log, mac, params, "imu", replay_options))
    print("gyro_bias_xy_deg_sec: "
          f"estimate=[{params[names[0]]:.6f},{params[names[1]]:.6f}] "
          f"body_rate_mean=[{base[0]:.6f},{base[1]:.6f}] "
          f"residual=[{residual[0]:.6f},{residual[1]:.6f}] samples={count}")


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
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--axis-remap", nargs=9, type=float)
    parser.add_argument("--gyro-axis-remap", nargs=9, type=float)
    args = parser.parse_args()
    if not os.path.isfile(args.log):
        parser.error("log file not found")
    if not os.path.isfile(args.replay):
        parser.error("replay executable not found; run make replay")

    params = {
        "sensor_pitch_offset_deg": 0.0,
        "sensor_roll_offset_deg": 0.0,
        "gyro_bias_x_deg_sec": 0.0,
        "gyro_bias_y_deg_sec": 0.0,
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

    # A level-flight run has approximately zero mean pitch/roll body rate.
    # Solve the fixed raw-axis biases from that physical constraint.  This is
    # independent of AHRS correction-loop equilibria and integration cadence;
    # the probe captures raw-axis polarity/remapping.
    solve_gyro_biases(args.replay, args.log, args.device_mac, params,
                      args.bias_probe_deg_sec, replay_options)

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
