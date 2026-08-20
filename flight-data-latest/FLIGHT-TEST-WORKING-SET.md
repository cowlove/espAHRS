# Flight-test working set

This directory contains the current paired flight-test data set used for
English-language discussion and replay analysis. The paired files were
recorded during the same four test sequences using two hardware instances:

- `G247C020.bin` through `G247C023.bin`: GEEK unit ending in MAC `247C`
- `GD5BC006.bin` through `GD5BC009.bin`: GEEK unit ending in MAC `D5BC`

The matching number identifies the test sequence on the two devices. Use the
MAC-specific device profile when replaying each file; the logs contain raw
sensor records, GPS data, and G5 reference data where available.

## Test sequences

### 1. Straight and level

Approximately ten minutes of relatively straight-and-level flight. This is
the primary data set for checking steady-state pitch/roll offsets, gyro bias,
heading behavior, and slow drift.

| Hardware | File |
|---|---|
| 247C | `G247C020.bin` |
| D5BC | `GD5BC006.bin` |

### 2. Steep sustained turns

Sustained steep turns in both directions. This sequence exercises turn-rate
bank estimation, roll tracking, coordinated-turn assumptions, and behavior
under sustained lateral acceleration.

| Hardware | File |
|---|---|
| 247C | `G247C021.bin` |
| D5BC | `GD5BC007.bin` |

### 3. Normal-envelope gentle turns

Gentle, brief turns mixed with level flight, gentle climbs, and gentle
descents. This is intended to approximate the operating envelope of a normal
autopilot flight and is useful for evaluating ordinary coupled pitch/roll
behavior rather than isolated maneuvers.

| Hardware | File |
|---|---|
| 247C | `G247C022.bin` |
| D5BC | `GD5BC008.bin` |

### 4. Steep pitch excursions

Large pitch excursions using two styles of speed management: attempts to hold
constant speed, and pitch changes allowed to produce acceleration or
deceleration. This sequence exercises pitch gyro/accelerometer kinematics,
pitch correction, and longitudinal-acceleration compensation.

| Hardware | File |
|---|---|
| 247C | `G247C023.bin` |
| D5BC | `GD5BC009.bin` |

## Current analysis notes

- The two hardware instances recorded the same maneuver sequence, making
  paired comparisons useful for separating algorithm behavior from hardware
  and sensor differences.
- Compass heading results are currently considered unvalidated. In
  particular, recent plots show evidence of gross calibration errors and
  possible axis-mapping or frame-mismatch issues.
- Compass 0 and compass 1 heading weights are currently set to zero in the
  AHRS defaults while the compass response is investigated.
- The compass calibration-motion file `GD5BC010.bin` is separate from this
  four-sequence paired flight-test working set; it contains a magnetometer
  calibration motion sequence for the D5BC device.

## Compass installation-calibration limitations and plan

The out-of-aircraft calibration runs may provide a good estimate of each
magnetometer's intrinsic offset and scale response, but the installed aircraft
environment can add hard-iron and soft-iron effects from nearby wires,
batteries, fasteners, and airframe structure. The recent in-aircraft plots
also suggest that axis/frame mapping errors may be present in addition to
ordinary calibration error.

The aircraft's practical test envelope is only about 50 degrees of bank and
30 degrees of pitch. That is not sufficient to identify an unconstrained
full 3-D ellipsoid reliably in the aircraft, so future in-aircraft data should
not be used for an unrestricted nine-parameter refit.

When the compass work resumes, the planned approach is to:

1. preserve the out-of-aircraft calibration as the intrinsic baseline;
2. verify sensor-to-aircraft axis mapping and frame rotation independently;
3. estimate only constrained aircraft-installation corrections, starting with
   hard-iron offsets while holding the intrinsic soft-iron matrix fixed;
4. validate any correction on separate flight sequences rather than the data
   used to fit it.

Until that work is completed, both compass heading weights remain zero and
the AHRS uses GPS ground track as its available heading reference. GPS track
alone is considered sufficient for the current pitch/roll flight-test and
normal-autopilot operating-envelope work.
