#include "AircraftAHRS.h"
#include "HardwareAbstraction.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

namespace {
constexpr double kDegToRad = 0.01745329251994329577;

void multiply(const double a[3][3], const double b[3][3], double out[3][3]) {
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        out[i][j] = 0;
        for (int k = 0; k < 3; ++k) out[i][j] += a[i][k] * b[k][j];
    }
}

void attitudeMatrix(double phi, double theta, double psi, double out[3][3]) {
    const double cph = cos(phi), sph = sin(phi);
    const double ct = cos(theta), st = sin(theta);
    const double cp = cos(psi), sp = sin(psi);
    const double value[3][3] = {
        {cp*ct, cp*st*sph-sp*cph, cp*st*cph+sp*sph},
        {sp*ct, sp*st*sph+cp*cph, sp*st*cph-cp*sph},
        {-st, ct*sph, ct*cph}
    };
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
        out[i][j] = value[i][j];
}

void bodyIncrement(double p, double q, double r, double dt, double out[3][3]) {
    double x = p*dt, y = q*dt, z = r*dt;
    const double angle = sqrt(x*x + y*y + z*z);
    x /= angle; y /= angle; z /= angle;
    const double c = cos(angle), s = sin(angle), oneMinusC = 1-c;
    const double value[3][3] = {
        {c+x*x*oneMinusC, x*y*oneMinusC-z*s, x*z*oneMinusC+y*s},
        {y*x*oneMinusC+z*s, c+y*y*oneMinusC, y*z*oneMinusC-x*s},
        {z*x*oneMinusC-y*s, z*y*oneMinusC+x*s, c+z*z*oneMinusC}
    };
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
        out[i][j] = value[i][j];
}

void checkAgainstIndependentMatrix(double rollDeg, double pitchDeg,
                                   double pDeg, double qDeg, double rDeg) {
    float rollRate, pitchRate, headingRate;
    AircraftAHRS::bodyRatesToEulerRates(
        rollDeg, pitchDeg, pDeg, qDeg, rDeg,
        rollRate, pitchRate, headingRate);
    const double phi = rollDeg*kDegToRad, theta = pitchDeg*kDegToRad;
    const double p = pDeg*kDegToRad, q = qDeg*kDegToRad, r = rDeg*kDegToRad;
    const double dt = 1e-6;
    double attitude[3][3], increment[3][3], next[3][3];
    attitudeMatrix(phi, theta, 0, attitude);
    bodyIncrement(p, q, r, dt, increment);
    multiply(attitude, increment, next);
    const double nextRoll = atan2(next[2][1], next[2][2]);
    const double nextPitch = asin(-next[2][0]);
    const double nextHeading = atan2(next[1][0], next[0][0]);
    assert(fabs(rollRate - (nextRoll-phi)/dt/kDegToRad) < 0.002);
    assert(fabs(pitchRate - (nextPitch-theta)/dt/kDegToRad) < 0.002);
    assert(fabs(headingRate - nextHeading/dt/kDegToRad) < 0.002);
}
}

int main() {
    checkAgainstIndependentMatrix(0, 0, 10, 0, 0);
    checkAgainstIndependentMatrix(30, 0, 0, 0, 10);
    checkAgainstIndependentMatrix(-42, 17, 12, -23, 19);
    checkAgainstIndependentMatrix(65, -28, -31, 17, -13);

    // Prove raw gyro polarity is applied before the mounting rotation.
    AircraftAHRS::Config config;
    config.gyroAxisSignX = 1;
    config.gyroAxisSignY = -1;
    config.gyroAxisSignZ = -1;
    config.gyroRateLimitDegSec = 0;
    AircraftAHRS ahrs(config);
    float rotation[3][3];
    halMakeSensorFrameRotation(11.0f, 7.0f, 3.0f, rotation);
    ahrs.setSensorFrameRotation(rotation);
    float x = 4, y = -8, z = -12;
    halRotateVector(rotation, x, y, z); // signs make raw vector (4,-8,-12)
    ahrs.updateImu(4, 8, 12, 1000);
    ahrs.updateImu(4, 8, 12, 21000);
    const auto &state = ahrs.state(21);
    assert(fabsf(state.lastPitchBodyRateDegSec - y) < 1e-5f);
    assert(fabsf(state.lastYawBodyRateDegSec - z) < 1e-5f);

    puts("Aircraft AHRS kinematics tests passed");
    return 0;
}
