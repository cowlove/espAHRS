# Magnetic-vector roll aiding: prior art and project distinction

This note summarizes a literature and public-code review performed on
August 9, 2026. It is an engineering survey, not a patentability or exhaustive
novelty search.

## The method used in this project

The project uses the calibrated three-dimensional magnetic-field vector as a
slow bank-angle observation rather than reducing it to heading alone:

1. Use a trusted pitch estimate as an independent attitude constraint.
2. Use the modeled local geomagnetic vector, including declination and
   inclination/dip.
3. Solve for the two roll/yaw attitudes consistent with the measured magnetic
   vector and trusted pitch.
4. Select the branch nearest the continuous AHRS attitude.
5. Apply the resulting roll innovation as a weak, gated, long-period correction
   to gyro-integrated roll.
6. Reject or reduce the correction using field magnitude, geometric
   sensitivity, innovation limits, freshness, and agreement between two
   independently calibrated magnetometers.

This provides a GPS-independent constraint on roll drift while preserving the
gyro as the short-period attitude source.

## Bottom line

The broad principle is established prior art: vector magnetometers have long
been used for attitude determination, vector matching, gyro drift correction,
and full three-axis magnetic fusion.

The project's exact decomposition does not appear to be a standard named
algorithm or a common public-code implementation. In particular, the review
did not find another implementation combining trusted pitch, a two-branch
roll/yaw solve using magnetic dip, temporal branch selection, a deliberately
weak roll-only observation, and dual-magnetometer agreement gating.

The best description is therefore a lightweight, aircraft-specific constrained
attitude observer built from established magnetic-vector geometry.

## Closest literature

### Aircraft attitude measurement using a vector magnetometer (NASA, 1977)

This report investigates aircraft attitude determination from all three
components of the Earth's magnetic field. It develops small-angle relationships
between magnetic-vector changes and pitch, roll, and yaw, and discusses
combining a strapped-down vector magnetometer with gyros so magnetic
measurements provide long-term attitude stability.

This is the clearest historical conceptual ancestor, although it uses
small-angle relationships and a different system architecture.

- [NASA Technical Reports Server: Aircraft attitude measurement using a vector magnetometer](https://ntrs.nasa.gov/citations/19780009107)

### MAV attitude determination by vector matching (2008)

Gebre-Egziabher and Elkaim determine attitude by matching measured Earth
gravity and magnetic-field vectors to their navigation-frame references. This
belongs to the TRIAD/Wahba/vector-matching family and demonstrates the general
power of two nonparallel reference vectors for complete attitude determination.

Unlike this project, it relies on a complete gravity reference rather than
using only trusted pitch to resolve the magnetic-vector ambiguity.

- [DOI: 10.1109/TAES.2008.4655360](https://doi.org/10.1109/TAES.2008.4655360)

### UAV attitude measurement using magnetometer and satellite positioning (2022)

This paper is geometrically very close. Given pitch and yaw inferred from GNSS
velocity, it derives roll directly from the measured body magnetic vector and
the navigation-frame geomagnetic vector. Its Equation 4 is an explicit
magnetic-vector roll solution.

The important difference is that it assumes both pitch and yaw are already
known. This project supplies only trusted pitch, solves the remaining roll/yaw
ambiguity as two candidates, and uses temporal continuity to choose a branch.

- [UAV Attitude Angle Measurement Method Based on Magnetometer-Satellite Positioning System](https://www.mdpi.com/2076-3417/12/12/5947)

### Nonlinear complementary observers and EKFs

Mahony-style observers, Madgwick filters, and attitude EKFs use vector
innovations to correct gyro-integrated attitude. Some also estimate gyro bias.
These are more general mathematical frameworks that can produce the same broad
effect, but they do not normally expose this project's pitch-constrained
magnetic bank observation as a separate, tunable correction source.

- [Mahony, Hamel, and Pflimlin: Nonlinear Complementary Filters on the Special Orthogonal Group](https://doi.org/10.1109/TAC.2008.923738)
- [On the Observability of Attitude with Single Direction Measurements](https://arxiv.org/abs/2008.13067)

## Public-code comparison

### PX4 EKF2: closest production analogue

PX4 supports true three-axis magnetometer fusion. Its EKF predicts the body
measurement from an Earth magnetic-field state, can update tilt as well as
heading, and can update X/Y gyro-bias states when full 3-D magnetic fusion is
enabled. It also uses magnetic-field checks and a world magnetic model.

This is the closest public production implementation in functional effect. It
is a full covariance-based EKF, however, rather than a closed-form
pitch-constrained roll observer.

- [PX4 magnetometer fusion control](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp)
- [PX4 three-axis magnetometer measurement update](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_fusion.cpp)

### Madgwick AHRS

The Madgwick MARG update uses the complete measured magnetic vector and an
estimated Earth-field direction in a quaternion gradient-descent correction.
Magnetic and gravity observations are optimized jointly rather than producing
a separately inspectable roll observation.

- [Arduino MadgwickAHRS implementation](https://github.com/arduino-libraries/MadgwickAHRS/blob/master/src/MadgwickAHRS.cpp)

### Common heading-only implementations

Many practical AHRS implementations deliberately prevent magnetometer data
from changing roll and pitch because local magnetic interference can corrupt
tilt estimates:

- INAV rotates the measured magnetic vector into the Earth frame, discards its
  vertical component, and explicitly states that ignoring inclination keeps
  magnetic feedback from affecting roll or pitch.
- xioTechnologies Fusion forms a horizontal west reference from gravity crossed
  with the magnetometer, likewise making magnetic feedback principally a
  heading correction.

This widespread heading-only design makes this project's deliberate use of
magnetic inclination for bank estimation unusual. It also explains why strong
calibration, disturbance rejection, slow correction, and dual-sensor agreement
are important.

- [INAV IMU implementation](https://github.com/iNavFlight/inav/blob/master/src/main/flight/imu.c)
- [xioTechnologies Fusion AHRS](https://github.com/xioTechnologies/Fusion/blob/main/Fusion/FusionAhrs.c)

## What appears distinctive here

The individual mathematical ingredients are known. The distinctive engineering
combination is:

- trusted pitch used as the only independent scalar attitude constraint;
- full local magnetic declination and inclination retained;
- explicit two-candidate roll/yaw solution;
- continuous-attitude proximity used for branch selection;
- magnetic result applied only as weak, long-period roll aiding;
- two independently calibrated magnetometers used for agreement gating; and
- coexistence with accelerometer roll and GPS/turn-rate-derived bank targets as
  separately tunable correction sources.

Useful search terms for further work include **vector matching**, **Wahba
problem with Euler-angle constraint**, **single-vector attitude observability**,
**magnetic-vector attitude aiding**, and **3-D magnetometer gyro-bias fusion**.
