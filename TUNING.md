# AHRS replay tuning guide

This page documents the central replay parameter surface in `ReplayConfig.h`.
The replay harness lets us change AHRS behavior without editing the replay loop
or recompiling between every trial:

```sh
./replay session.bin --list-params
./replay session.bin --param roll_correction_sec=6.0
./replay session.bin \
  --param accelerometer_roll_weight=0.8 \
  --param turn_bank_weight=0.2 \
  --roll-csv roll-trial.csv
```

Parameters may be repeated in one invocation. Each later value replaces the
earlier value. Unknown names are rejected. The command-line values affect the
host replay only; they do not change the firmware or the HAL defaults.

## How to tune safely

Start with a structurally good log. Before interpreting an AHRS score, check
the replay summary for:

```text
SEQUENCE gaps=0 duplicates=0
```

Also check the embedded stop summary for logger drops and write errors. A
parameter sweep cannot repair missing or corrupt input data.

Tune in this order:

1. Confirm the sensor frame, gyro signs, fixed trims, and G5 reference
   convention.
2. Set the G5 time and heading offsets.
3. Tune accelerometer filtering and the roll/pitch correction time constants.
4. Tune the roll-target weights and coordinated-turn parameters.
5. Tune magnetic roll aiding conservatively, using a flight log to decide
   whether it helps during periods where accelerometer or GPS-turn aiding is
   weak.
6. Tune GPS and barometer dynamics only when the log contains enough motion
   to make those parameters observable.

Use a single maneuver-rich log for comparisons whenever possible. A stationary
log is good for frame conventions, sensor calibration, and low-speed magnetic
behavior, but it cannot meaningfully identify turn-rate or GPS-speed aiding.

## Roll correction model

When the accelerometer has good specific-force quality, the AHRS forms the
roll correction target as:

```text
roll_target = accelerometer_roll_weight * accelerometer_roll
            + turn_bank_weight * bank_target
            + gps_bank_weight * gps_bank
```

`accelerometer_roll` is the gravity-derived roll after the current roll is
used to compensate the accelerometer geometry. `bank_target` is derived from
the filtered rate of change of the fused magnetic/GPS heading and the assumed
ground speed:

```text
bank_target = atan(speed * fused_turn_rate) / g
```

`gps_bank` uses the filtered derivative of GPS ground track only:

```text
gps_bank = atan(speed * gps_turn_rate) / g
```

The result is limited by `maximum_bank_target_deg`.

The two weights are independent direct gains. They are **not normalized** and
are not percentages. For example:

```text
accelerometer_roll_weight = 1.0
turn_bank_weight          = 0.0
```

uses only the accelerometer target, while:

```text
accelerometer_roll_weight = 0.0
turn_bank_weight          = 1.0
```

uses only the turn-derived bank target. Values of `0.5` and `0.5` produce a
conventional additive blend. The committed GEEK-S3 tuning uses
`gps_bank_weight=1.05`, with `turn_bank_weight=0` and
`magnetic_roll_weight=0` because the flight replay showed discontinuities and
poor magnetic roll observations.

`accel_correction_sec` controls how quickly the AHRS follows this combined
target. If accelerometer quality is poor, the implementation falls back to
the turn-bank target and uses `roll_correction_sec` instead. Magnetic roll
aid is a separate, subsequent correction with its own weight and time
constant; it is not included in the equation above.

Useful first sweeps include:

```sh
# Compare pure accelerometer, equal-gain, and pure turn-bank behavior.
./replay flight.bin --param accelerometer_roll_weight=1 --param turn_bank_weight=0
./replay flight.bin --param accelerometer_roll_weight=0.5 --param turn_bank_weight=0.5
./replay flight.bin --param accelerometer_roll_weight=0 --param turn_bank_weight=1

# Slow down or speed up the response after choosing the target composition.
./replay flight.bin --param accel_correction_sec=8
./replay flight.bin --param accel_correction_sec=16
```

## Parameter reference

### Reference alignment and sensor frame

| Parameter | Units | Default | Meaning |
|---|---:|---:|---|
| `g5_heading_offset_deg` | deg | 0 | Constant heading adjustment applied when comparing replay AHRS output with the G5 reference. |
| `g5_time_offset_ms` | ms | 20 | Effective G5 latency used when selecting the AHRS state for a G5 comparison. |
| `sensor_pitch_offset_deg` | deg | HAL | Sensor-to-aircraft pitch mounting offset. The same rigid rotation is applied to gyro, accelerometer, and compass vectors. |
| `sensor_roll_offset_deg` | deg | HAL | Sensor-to-aircraft roll mounting offset. |
| `accel_input_scale` | — | 1 | Replay-only scale applied to logged accelerometer input; useful for diagnosing unit-conversion problems. |

The current GEEK-S3 HAL defaults are the calibrated values, rather than the
original exploratory values. Inspect `HardwareAbstraction.h` when recording
the exact calibration used by a replay.

### Gyro and accelerometer trims

| Parameter | Units | Default | Meaning |
|---|---:|---:|---|
| `gyro_bias_x_deg_sec` | deg/s | HAL | Bias subtracted from the aircraft-frame X gyro channel before integration. |
| `gyro_bias_y_deg_sec` | deg/s | HAL | Bias subtracted from Y. |
| `gyro_bias_z_deg_sec` | deg/s | HAL | Bias subtracted from Z. |
| `gyro_axis_sign_x` | sign | HAL | Polarity multiplier for X, normally `1` or `-1`. |
| `gyro_axis_sign_y` | sign | HAL | Polarity multiplier for Y. |
| `gyro_axis_sign_z` | sign | HAL | Polarity multiplier for Z. |
| `accel_bias_x_mps2` | m/s² | 0 or HAL | Bias subtracted from X acceleration. |
| `accel_bias_y_mps2` | m/s² | 0 or HAL | Bias subtracted from Y acceleration. |
| `accel_bias_z_mps2` | m/s² | 0 or HAL | Bias subtracted from Z acceleration. |

These are applied before the AHRS equations. A gyro bias error accumulates
with time, so it often appears as a steadily growing attitude error. A sign
error usually produces an error that grows in the wrong direction during a
deliberate motion and then gets pulled back by the correction target.

### Basic attitude response

Time constants are first-order response constants: smaller values respond
faster and trust the observation more; larger values respond more slowly and
trust gyro integration more.

| Parameter | Units | Default | Meaning |
|---|---:|---:|---|
| `yaw_correction_sec` | s | 2.0 | Heading response to the fused heading reference. |
| `roll_correction_sec` | s | 4.0 | Fallback roll response when accelerometer aiding is unavailable. |
| `pitch_correction_sec` | s | 8.0 | Response to the GPS flight-path/angle-of-attack pitch reference. |
| `accel_correction_sec` | s | 5.0 | Response to the combined accelerometer/GPS-bank roll target. |
| `pitch_kinematic_correction_sec` | s | 20.0 | Response to kinematic pitch correction when gravity-only pitch aiding is unavailable. |
| `pitch_gravity_correction_sec` | s | 12.0 | Response to the gravity-derived pitch target when vertical motion is stable. |
| `gps_derivative_sec` | s | 1.5 | Smoothing of GPS track rate used for turn-rate and bank estimation. |

Do not compensate for a bad axis sign by making a correction time constant
very small. Resolve frame and polarity errors first; otherwise the correction
loop can hide the real defect in one maneuver and amplify it in another.

### Accelerometer quality and gravity aiding

| Parameter | Units | Default | Meaning |
|---|---:|---:|---|
| `accel_filter_sec` | s | 0.25 | Low-pass time constant for the accepted accelerometer vector. |
| `accel_tolerance_mps2` | m/s² | 1.5 | Allowed difference between filtered specific-force magnitude and 1 g. Outside this gate, gravity-derived roll/pitch aiding is disabled. |
| `vertical_rate_filter_sec` | s | 1.5 | Smoothing of GPS altitude-rate and derived vertical acceleration. |
| `vertical_accel_tolerance_mps2` | m/s² | 0.35 | Maximum filtered vertical acceleration considered compatible with gravity-only pitch aiding. |
| `vertical_smoothness_window_sec` | s | 3.0 | Time the vertical-motion gate must remain good before gravity pitch aiding is enabled. |
| `angle_of_attack_deg` | deg | 0 | Added to the GPS flight-path angle when forming the kinematic pitch reference. |

The vertical-motion gate is deliberately conservative. It prevents the
fore-aft accelerometer component from being interpreted as pitch during a
climb, descent, or changing vertical rate.

### GPS heading and turn-bank aiding

The fused heading uses calibrated magnetic heading at low speed. GPS track is
added as speed increases, with its weight rising smoothly between the minimum
accepted ground speed and the configured threshold.

| Parameter | Units | Default | Meaning |
|---|---:|---:|---|
| `min_ground_speed_mps` | m/s | 5.0 | Minimum speed for kinematic GPS history and heading-rate aiding. |
| `gps_timeout_sec` | s | 1.0 | Maximum GPS age before GPS aiding is considered stale. |
| `fused_heading_filter_sec` | s | 0.35 | Low-pass time constant for the circular fused heading. |
| `gps_heading_speed_threshold_mps` | m/s | 20.576 | Speed at which GPS track reaches its configured maximum blend; 20.576 m/s is 40 kt. |
| `gps_heading_weight` | — | 3.0 | Maximum GPS-track weight relative to the combined compass heading weight. |
| `maximum_bank_target_deg` | deg | 60.0 | Hard limit on the turn-rate-derived bank target. |

These parameters embody the current deliberate simplification that ground
speed approximates airspeed and track curvature approximates coordinated-turn
bank. Wind and sideslip can violate that model, so tune it against flight
data rather than assuming a perfect physical solution.

### Magnetic roll aiding

The 3-D magnetic solver uses the local field declination and inclination,
holds the current pitch estimate, chooses the attitude branch closest to the
continuous AHRS state, and produces a slow roll observation. It is gated and
should remain a weak long-period correction until flight data proves otherwise.

| Parameter | Units | Default | Meaning |
|---|---:|---:|---|
| `magnetic_declination_deg` | deg | 14.89224 | East-positive local magnetic declination. |
| `magnetic_inclination_deg` | deg | 68.75569 | Down-positive magnetic dip/inclination. |
| `magnetic_roll_correction_sec` | s | 40.0 | Time constant for applying the magnetic roll innovation. |
| `magnetic_roll_weight` | — | 0.0 | Direct gain on the magnetic roll correction; disabled in the committed flight tuning. |
| `magnetic_field_magnitude_tolerance` | fraction | 0.20 | Allowed deviation of calibrated field magnitude from its normalized value of 1. |
| `magnetic_roll_max_disagreement_deg` | deg | 15.0 | Maximum disagreement between the two compass roll observations before magnetic roll aiding is rejected. |
| `magnetic_roll_min_geometry` | normalized | 0.25 | Minimum geometric sensitivity required to solve roll from the field vector. |

The magnetic roll weight is separate from `accelerometer_roll_weight` and
`turn_bank_weight`. Increasing it does not change the roll target equation; it
strengthens a later magnetic innovation update. A useful sweep keeps the
magnetic time constant long while trying weights such as `0`, `0.1`, `0.25`,
and `0.5`.

### Barometer and altitude fusion

These parameters are dormant or less observable until a valid barometer
stream is present, but they remain part of the shared replay surface.

| Parameter | Units | Default | Meaning |
|---|---:|---:|---|
| `baro_alt_filter_sec` | s | 0.5 | Barometric altitude low-pass time constant. |
| `baro_rate_filter_sec` | s | 0.75 | Barometric climb-rate low-pass time constant. |
| `baro_gps_bias_sec` | s | 30.0 | Time constant for slowly reconciling barometric altitude to GPS altitude. |
| `baro_timeout_sec` | s | 2.0 | Maximum barometer age for selecting barometric vertical rate. |

## Reading sweep results

The replay prints signed bias, MAE, RMSE, maximum absolute error, and p95
absolute error for roll, pitch, and heading. RMSE is useful for penalizing
large excursions; p95 is useful for judging the typical worst-case behavior;
signed bias identifies a constant offset or polarity/reference problem.

For detailed time-series inspection:

```sh
./replay flight.bin --roll-csv roll-trial.csv --pitch-csv pitch-trial.csv
```

The roll CSV includes the accelerometer roll, combined correction target,
turn-rate bank target, magnetic roll observation, and magnetic innovation.
The pitch CSV includes the accelerometer pitch, AHRS pitch, and G5 pitch.
Plot those intermediate signals before optimizing a final RMSE: a low score
obtained by excessive smoothing is not necessarily a good real-time AHRS.

The three canonical flight logs are kept in `flight-data-primary/`. Before
committing an AHRS change, regenerate their replay artifacts with:

```sh
make flight-results
```

This writes roll and pitch CSV results to `flight-data-primary/results/`.
Those CSVs are versioned with the AHRS commit so before/after behavior can be
inspected directly from the commit history.

## Central parameter ownership

`ReplayConfig.h` is intentionally the single command-line mapping layer. The
replay loop does not know where a parameter is used internally; it receives a
fully configured `AircraftAHRS::Config`. This keeps sweeps reproducible and
prevents parameter parsing from being scattered across sensor, GPS, and AHRS
code. Hardware-specific calibration belongs in `HardwareAbstraction.h`; the
replay command line is for experiment-specific overrides.
