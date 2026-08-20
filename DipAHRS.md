# DipAHRS design notes

## Mission

DipAHRS is intended to be a small, portable, allocation-free attitude
estimator for one commodity 9-DOF MARG sensor: a 3-axis gyroscope, 3-axis
accelerometer, and 3-axis magnetometer. Its goal is a practical three-axis
orientation estimate without GPS, air data, a stationary startup period, or
other runtime aiding.

The package should be useful on resource-constrained systems and in
GNSS/external-aiding-denied operation. It is not magnetically denied: the
distinctive roll observation depends on a calibrated, locally plausible Earth
magnetic-field vector.

The minimum runtime inputs are:

- monotonically increasing sample time;
- calibrated gyro, accelerometer, and magnetometer vectors;
- sensor-to-body frame transform;
- an expected local magnetic-field direction, expressed as declination and
  inclination/dip or an equivalent normalized vector.

The core should have no GPS, barometer, SD card, display, radio, RTOS, heap
allocation, or device drivers. The GEEK-S3 project remains the experimental
test bench and validation recorder; DipAHRS should be a reusable result rather
than a cleaned-up copy of the test application.

## The magnetic-dip bank observation

Most simple AHRS implementations reduce a 3-D magnetometer sample to a heading
or deliberately remove its vertical component so magnetic disturbances cannot
affect roll and pitch. DipAHRS instead retains geomagnetic inclination as useful
attitude information.

A single calibrated magnetic vector constrains attitude to a one-dimensional
family: rotation about that vector is still free. If pitch is independently
trusted, the remaining family normally reduces to two discrete roll/heading
solutions. The observer:

1. normalizes the calibrated body-frame magnetic vector;
2. constructs the expected local Earth-field direction from declination and
   inclination;
3. holds pitch at the current trusted estimate;
4. solves the two roll/heading candidates consistent with the measured vector;
5. rejects weak roll geometry; and
6. chooses the branch nearest the previous continuous attitude.

The result is an explicit magnetic bank observation. It is not applied as an
instantaneous attitude replacement. The gyro remains the short-period source,
while magnetic bank supplies a weak, gated, long-period roll-drift correction.
The current implementation of only this geometry is in [DipAHRS.h](DipAHRS.h).

## Boundary with espAHRS

DipAHRS is an experimental, standalone estimator concept, not the production
magnetic-heading implementation inside espAHRS. The integration boundary is
deliberately narrow:

- `AircraftAHRS` owns magnetometer calibration, aircraft-frame mapping,
  conventional roll/pitch tilt compensation, horizontal heading calculation,
  dual-compass fusion, GPS fusion, and magnetic turn-rate derivation.
- `DipAHRS::observe()` may supply only its full-vector magnetic-roll
  observation and geometry score to the optional DipAHRS roll input of the
  roll gyro correction target.
- `AircraftAHRS` must not consume the heading candidate returned by DipAHRS,
  and DipAHRS validity must not gate production magnetic-heading validity.
- Compass yaw weights and DipAHRS roll-source weights are independent. This
  preserves the experiment even while production compass yaw is disabled.

This separation is intentional so DipAHRS can later move into its own project
and paper without making espAHRS yaw behavior depend on the experimental
two-branch magnetic-dip solver.

## Why this decomposition is interesting

Magnetometer-aided attitude estimation is established prior art, including
TRIAD/Wahba matching, complementary observers, EKFs, and production 3-D
magnetic fusion. The narrower engineering contribution here is the explicit
pitch-constrained observer:

- it trusts one scalar tilt constraint rather than pretending the complete
  accelerometer vector is always gravity;
- it exposes the two-solution ambiguity and resolves it by continuity;
- it emits a separately inspectable bank observation and geometry score;
- it uses magnetic dip specifically for long-term roll correction;
- it can operate while stationary and does not inherit wind errors from GPS
  ground track or groundspeed; and
- it is computationally cheap enough for small microcontrollers.

See [MAGNETIC_ROLL_PRIOR_ART.md](MAGNETIC_ROLL_PRIOR_ART.md) for the literature
and public-code survey.

## The trusted-pitch question

The principal dependency is a useful pitch estimate. The original GEEK
prototype used a conservative external-aiding rule: correct pitch from the
roll-compensated fore/aft accelerometer component only when the recent altitude
history suggested little vertical acceleration. That gate is too restrictive
for the eventual DipAHRS package and requires data the package should not need.

An accelerometer measures specific force:

```text
f_body = R^T (a_navigation - g_navigation)
```

Pure vertical acceleration changes the magnitude of the vertical specific
force but not its direction. After normalization it can retain the same tilt
information, except near free fall. Lateral acceleration can largely be
separated after roll is known. The fundamental pitch ambiguity is sustained
longitudinal acceleration: forward acceleration and nose-up tilt can create
the same fore/aft accelerometer component.

The useful, honest assumption is therefore weaker than “no vertical
acceleration”:

> Over the pitch-correction time horizon, sustained longitudinal acceleration
> is small or has approximately zero mean.

This permits steady climbs/descents, vertical gust acceleration, load-factor
changes, coordinated turns, and many pull-up/push-over conditions. It cannot
guarantee pitch during arbitrary unknown sustained translation; no MARG-only
estimator can.

A future sensor-only pitch observer should:

1. propagate attitude with gyro integration;
2. de-roll the accelerometer using the current roll estimate;
3. form pitch from the de-rolled fore/aft and vertical components;
4. treat force parallel to predicted vertical as load-factor/vertical-motion
   magnitude rather than automatic invalidation;
5. reduce confidence during high jerk, near-free-fall, saturation, implausible
   innovations, or likely longitudinal transients; and
6. apply a slow robust correction so bounded acceleration disturbances average
   away while gyro bias does not.

A more ambitious formulation could jointly select the attitude on the
magnetic-vector solution family that minimizes modeled longitudinal specific
force. That could reduce the hard dependency on an already trusted pitch, but
it should be evaluated as a later extension rather than hidden in the first
implementation.

## Proposed estimator scope

The standalone estimator should eventually contain only:

1. quaternion gyro propagation;
2. accelerometer pitch/tilt observation with specific-force confidence;
3. conventional magnetic heading observation;
4. pitch-constrained full-vector magnetic bank observation;
5. slow complementary corrections and optional online gyro-bias adaptation;
6. freshness, innovation, field-magnitude, and geometry rejection; and
7. explicit observation validity and confidence diagnostics.

It should exclude GPS turn-bank inference, pressure/altitude logic, dual-
compass fusion, G5 parsing, binary logging, display code, and board HALs.
The production API should require one magnetometer. The two magnetometers in
the GEEK test hardware are an accidental but useful validation and disturbance-
detection arrangement, not a core algorithm requirement.

## Failure modes and required diagnostics

Magnetic bank must be rejected or heavily downweighted when:

- calibrated field magnitude is implausible;
- local hard/soft-iron interference changes the vector;
- pitch confidence is poor;
- the field geometry has little sensitivity to roll;
- the innovation is discontinuous or implausibly large; or
- branch continuity is lost during initialization or extreme motion.

Useful outputs include the selected roll and heading, selected branch,
roll-geometry score, field magnitude residual, innovation, correction
availability, and overall attitude confidence. A second compass may optionally
cross-check these diagnostics, but the core must work with one.

## Validation plan

The clean package should have three small examples:

- synthetic motion with injected bias/noise and branch transitions;
- desktop CSV replay using exactly the embedded estimator; and
- a minimal Arduino/ESP32 example with the sensor driver kept separate.

Tests should cover exact known rotations, coordinate-frame signs, angle wrap,
both magnetic branches, geomagnetic dip from equator to high latitude,
hard/soft-iron regression fixtures, magnetic disturbances, coordinated turns,
bounded longitudinal acceleration, reboot in motion, convergence after gyro
bias, and CPU/RAM/flash cost. Recorded GEEK/G5 sessions can be exported as
small anonymized MARG-plus-reference fixtures; DipAHRS itself should not parse
the GEEK binary format.

The initial standalone geometry checks are in
[dipahrs_test.cpp](dipahrs_test.cpp) and run with `make dipahrs-test` under
C++11.

Comparisons should include gyro-only, heading-only magnetic aiding, common
Madgwick/Mahony implementations, and DipAHRS with magnetic bank disabled and
enabled. Report bias, MAE, RMSE, p95/max error, convergence time, aiding
availability, rejection counts, and compute/memory cost.

## Publication direction

The technical algorithm note and the software paper are complementary:

- a short technical paper can derive the pitch-constrained two-branch magnetic
  bank observer and report ground/flight results; and
- a later JOSS paper can describe the mature, tested, documented, reusable
  software and its research need.

For JOSS, the standalone project will need an OSI license, stable public API,
tests and CI, API/algorithm/calibration/frame documentation, examples,
reproducible fixtures, tagged releases, public development history, and some
evidence of external use. Restrained C++11 is the practical first language:
no heap, exceptions, RTTI, STL containers, OS calls, or hardware dependencies.
