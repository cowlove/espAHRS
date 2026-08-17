# Flight-Condition Gyro-Bias Observer Design

## Purpose

Develop a slow gyro-bias estimator that can learn during ordinary, gentle
cruise flight. The estimator must not require a motionless, perfectly
straight-and-level interval and must not confuse aircraft maneuvering with
sensor bias.

The four representative cruise logs `G247C015` through `G247C018` are the
initial offline test envelope. They contain realistic small attitude changes,
descent/turn activity, and imperfectly steady flight.

## Hard boundary: G5 is offline-only

G5 data is a training and validation reference only. It must never be read,
passed into, or used to update the live AHRS or live bias observer.

In particular, production code must not use G5 for:

- attitude correction, gyro-bias correction, or state propagation;
- bias-observer qualification, gating, weighting, or reset decisions;
- fallback behavior, health decisions, or validity flags;
- parameter selection or adaptive time constants.

G5 may be consumed by replay/offline analysis to compare an estimate against a
reference, generate plots, score an experiment, or tune a proposed observer.
The production observer must be executable with logs that contain no G5 data,
and its tests must include a no-G5 path.

## Proposed estimator

Use a slow error-state observer whose bias state is expressed in the raw gyro
axes:

```text
b_raw = [bias_raw_x, bias_raw_y, bias_raw_z]
```

The live calibration chain is treated as part of the model:

```text
omega_body = R_orientation * R_axis * (omega_raw - b_raw)
```

`R_axis` includes the configured sensor-axis remapping and signs. `R_orientation`
includes the fine sensor-to-aircraft orientation calibration. The attitude
propagator then maps body angular rate into the quaternion/error-state
dynamics. Bias corrections are therefore applied to the correct raw axes
without assuming that Euler pitch maps to one gyro channel or roll maps to
another.

Maintain either:

1. a 3x3 sensitivity matrix `S = d(attitude_error) / d(b_raw)`, propagated
   through the same quaternion and calibration equations; or
2. a conventional quaternion-plus-gyro-bias error-state covariance and its
   measurement Jacobian.

The second form is preferred for the eventual live implementation. A first
replay prototype may use the explicit sensitivity matrix because it is easier
to inspect and validate.

## Live information sources

The observer may use only live sensors already available to the AHRS:

- gyro angular rates;
- accelerometer specific force, with a dynamic-acceleration consistency test;
- GPS velocity, track, and freshness/quality indicators;
- compass and barometer only where their existing AHRS validity models support
  them.

The observer should use residuals from these live aiding models, not raw gyro
averages. Candidate residuals include gravity-vector error, GPS velocity/track
innovation, and existing pitch/roll correction-target innovation. The exact
measurement set must be chosen per axis and accompanied by a confidence value.

G5 is deliberately absent from this list.

## Bias update

For an available residual `r` with sensitivity `H` to the bias state, use a
slow, bounded update such as:

```text
K = P H^T (H P H^T + R)^-1
b_raw <- b_raw + K r
P <- (I - K H) P + Q
```

`Q` should model very slow bias drift. Apply long time constants, innovation
limits, hysteresis, and per-axis confidence. Do not update a weakly observable
direction merely because another direction has a strong residual.

An equivalent normalized-gradient implementation is acceptable for the first
prototype, but it must use the full sensitivity/Jacobian rather than separate
pitch-to-pitch and roll-to-roll corrections.

## Observability and safety

Two pitch/roll residuals cannot uniquely determine three gyro-bias components
at one instant. Attitude changes and GPS/heading information provide temporal
observability, but not every flight segment observes every axis equally.

The observer must therefore expose, per raw axis:

- accumulated information or covariance;
- whether the axis is currently observable;
- candidate bias and applied bias;
- innovation magnitude and rejection count;
- update age and confidence.

When information is insufficient, hold the bias rather than inventing a
correction. Bound the applied bias and rate of change. A rejected or stale GPS,
accelerometer, compass, or barometer measurement must not silently become bias
evidence.

## Offline replay and validation plan

1. Add an offline observer mode that consumes only IMU, GPS, compass, barometer,
   and existing AHRS correction residuals.
2. Run it on `G247C015` through `G247C018` with G5 disabled or removed from the
   input stream.
3. Use G5 only after computation, to plot reference-vs-estimate errors and
   compare bias estimates between experiments. It must not affect observer
   state, gates, or parameters.
4. Inject known constant and slowly drifting biases into replay and verify that
   the observer converges to the correct *raw-axis* bias after the calibration
   and orientation transforms.
5. Perturb axis remaps and fine orientation offsets and verify that the
   estimated correction follows the transformed sensitivity rather than a
   fixed Euler-axis assignment.
6. Run ablations with GPS, compass, and accelerometer aiding individually
   disabled to identify which bias directions remain observable.
7. Compare against the current stationary/quiet estimator and require that the
   new method improves useful convergence on the four cruise logs without
   introducing unstable attitude drift.

## Acceptance criteria

- Live AHRS behavior is bit-for-bit unaffected when the observer is disabled.
- No live source file references G5 data for state, bias, gating, or health.
- A no-G5 replay produces the same observer result as the corresponding log
  with G5 records removed.
- Known injected raw-axis biases converge to the correct axes within defined
  tolerances.
- Bias updates remain bounded and stop when observability or sensor quality is
  inadequate.
- The observer does not learn maneuver acceleration as gyro bias on the four
  representative cruise logs.
- Offline G5 comparison is clearly separated from production computation in
  code, tests, and documentation.
