#include "AircraftAHRS.h"
#include "DeviceConfiguration.h"

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

float determinant(const float m[3][3]) {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}
}

int main() {
    const uint8_t knownMac[6] = {0x28,0x37,0x2F,0xF8,0x24,0x7C};
    const uint8_t unknownMac[6] = {1,2,3,4,5,6};
    assert(findDeviceConfiguration(knownMac) == &DEVICE_CONFIGURATIONS[0]);
    assert(findDeviceConfiguration(unknownMac) == nullptr);
    assert(DEVICE_CONFIGURATIONS[0].boardKind == HalBoardKind::GeekS3);
    const DeviceConfiguration *resolved = nullptr;
    assert(resolveDeviceConfiguration("28:37:2F:F8:24:7C", resolved) == DeviceConfigurationLookup::Found);
    assert(resolved == &DEVICE_CONFIGURATIONS[0]);
    assert(resolveDeviceConfiguration("28372ff8247c", resolved) == DeviceConfigurationLookup::Found);
    assert(resolveDeviceConfiguration("247C", resolved) == DeviceConfigurationLookup::Found);
    assert(resolveDeviceConfiguration("24:7c", resolved) == DeviceConfigurationLookup::Found);
    assert(resolveDeviceConfiguration("7C", resolved) == DeviceConfigurationLookup::Invalid);
    assert(resolveDeviceConfiguration("ZZZZ", resolved) == DeviceConfigurationLookup::Invalid);
    checkAgainstIndependentMatrix(0, 0, 10, 0, 0);
    checkAgainstIndependentMatrix(30, 0, 0, 0, 10);
    checkAgainstIndependentMatrix(-42, 17, 12, -23, 19);
    checkAgainstIndependentMatrix(65, -28, -31, 17, -13);

    // Body-rate remapping must remain a proper right-handed rotation.  A
    // single-axis convention flip turns this into a reflection and breaks
    // coordinated-turn q/r cancellation.
    // The two currently installed GEEK IMUs have different raw polarity but
    // their per-source maps must produce the same body-frame vectors.
    constexpr DeviceConfiguration geek = DEVICE_CONFIGURATIONS[0];
    float pGx = 10.0f, pGy = -20.0f, pGz = -30.0f;
    applyRawGyroCalibration(geek.calibration.imu[0], pGx, pGy, pGz);
    applyAxisRemap(geek.calibration.imu[0].gyroAxisRemap,
                            pGx, pGy, pGz);
    float sGx = 10.0f, sGy = 20.0f, sGz = 30.0f;
    applyRawGyroCalibration(geek.calibration.imu[1], sGx, sGy, sGz);
    applyAxisRemap(geek.calibration.imu[1].gyroAxisRemap,
                            sGx, sGy, sGz);
    // Remove IMU0's characterized bias for this polarity-only comparison.
    assert(fabsf((pGx + geek.calibration.imu[0].gyroBiasDegSec[0]) - sGx) < 1e-6f);
    assert(fabsf((pGy - geek.calibration.imu[0].gyroBiasDegSec[1]) - sGy) < 1e-6f);
    assert(fabsf((pGz - geek.calibration.imu[0].gyroBiasDegSec[2]) - sGz) < 1e-6f);
    float pAx = 1.0f, pAy = 2.0f, pAz = 3.0f;
    float sAx = 1.0f, sAy = -2.0f, sAz = -3.0f;
    applyAxisRemap(geek.calibration.imu[0].sensorAxisRemap,
                            pAx, pAy, pAz);
    applyAxisRemap(geek.calibration.imu[1].sensorAxisRemap,
                            sAx, sAy, sAz);
    assert(fabsf(pAx - sAx) < 1e-6f);
    assert(fabsf(pAy - sAy) < 1e-6f);
    assert(fabsf(pAz - sAz) < 1e-6f);

    // In aircraft X-forward/Z-down coordinates, a nose-up gravity vector has
    // negative X.  The public/internal pitch observation must be positive.
    AircraftAHRS pitchConvention;
    pitchConvention.updateImu(0, 0, 0, 1000, -1.7029f, 0, 9.6577f, true);
    pitchConvention.updateImu(0, 0, 0, 21000, -1.7029f, 0, 9.6577f, true);
    assert(pitchConvention.state(21).accelerometerPitchDeg > 9.5f);

    // Raw sensor bias/polarity must be applied before the installed-sensor
    // axis remap.  This guards the HAL/replay calibration ordering.
    DeviceImuCalibration rawCalibration{};
    rawCalibration.gyroBiasDegSec[0] = 1.0f;
    rawCalibration.gyroBiasDegSec[1] = 2.0f;
    rawCalibration.gyroBiasDegSec[2] = 3.0f;
    rawCalibration.gyroAxisSign[0] = 1.0f;
    rawCalibration.gyroAxisSign[1] = -1.0f;
    rawCalibration.gyroAxisSign[2] = 1.0f;
    float rawX = 5.0f, rawY = 8.0f, rawZ = 12.0f;
    applyRawGyroCalibration(rawCalibration, rawX, rawY, rawZ);
    assert(fabsf(rawX - 4.0f) < 1e-6f);
    assert(fabsf(rawY + 6.0f) < 1e-6f);
    assert(fabsf(rawZ - 9.0f) < 1e-6f);
    const float remap[3][3] = {
        {1, 0, 0}, {0, 0, -1}, {0, -1, 0}
    };
    applyAxisRemap(remap, rawX, rawY, rawZ);
    assert(fabsf(rawX - 4.0f) < 1e-6f);
    assert(fabsf(rawY + 9.0f) < 1e-6f);
    assert(fabsf(rawZ - 6.0f) < 1e-6f);

    // Prove raw gyro polarity is applied before the mounting rotation.
    AircraftAHRS::Config config;
    config.gyroAxisSignX = 1;
    config.gyroAxisSignY = -1;
    config.gyroAxisSignZ = -1;
    config.gyroRateLimitDegSec = 0;
    AircraftAHRS ahrs(config);
    float rotation[3][3];
    makeSensorFrameRotation(11.0f, 7.0f, 3.0f, rotation);
    ahrs.setSensorFrameRotation(rotation);
    float x = 4, y = -8, z = -12;
    applyAxisRemap(rotation, x, y, z); // signs make raw vector (4,-8,-12)
    ahrs.updateImu(4, 8, 12, 1000);
    ahrs.updateImu(4, 8, 12, 21000);
    const auto &state = ahrs.state(21);
    assert(fabsf(state.lastPitchBodyRateDegSec - y) < 1e-5f);
    assert(fabsf(state.lastYawBodyRateDegSec - z) < 1e-5f);

    // The adaptive observer starts from zero every boot, accumulates
    // per-axis information from production correction targets, and converges
    // toward injected input-axis bias while ordinary moving-flight GPS and
    // accelerometer aiding remain active.
    AircraftAHRS::Config adaptiveConfig;
    adaptiveConfig.gyroIntegrationDtSec = 0.02f;
    adaptiveConfig.adaptiveGyroBiasQualificationTimeSec = 0.2f;
    adaptiveConfig.adaptiveGyroBiasLearningTimeSec = 0.5f;
    adaptiveConfig.adaptiveGyroBiasMaximumSlewDegSec2 = 2.0f;
    adaptiveConfig.adaptiveGyroBiasMaximumDegSec = 1.0f;
    AircraftAHRS adaptive(adaptiveConfig);
    float observerRotation[3][3];
    makeSensorFrameRotation(11.0f, 7.0f, 3.0f, observerRotation);
    adaptive.setSensorFrameRotation(observerRotation);
    // Input-frame acceleration that becomes (0,0,+g) after rotation.
    const float accelInputX = observerRotation[2][0] * 9.80665f;
    const float accelInputY = observerRotation[2][1] * 9.80665f;
    const float accelInputZ = observerRotation[2][2] * 9.80665f;
    for (uint32_t ms = 0; ms <= 10000; ms += 20) {
        if (ms % 100 == 0) adaptive.updateGps(90, 30, 100, true, ms + 1);
        adaptive.updateImu(0.4f, -0.2f, 0.1f, (ms + 1) * 1000,
                           accelInputX, accelInputY, accelInputZ, true);
    }
    const auto &adapted = adaptive.state(10001);
    assert(adapted.adaptiveGyroBiasQualified);
    assert(fabsf(adapted.adaptiveGyroBiasXDegSec - 0.4f) < 0.02f);
    assert(fabsf(adapted.adaptiveGyroBiasYDegSec + 0.2f) < 0.02f);
    assert(fabsf(adapted.adaptiveGyroBiasZDegSec - 0.1f) < 0.02f);
    assert(adapted.adaptiveGyroBiasConfidenceX == 1.0f);
    assert(adapted.adaptiveGyroBiasConfidenceY == 1.0f);
    assert(adapted.adaptiveGyroBiasConfidenceZ == 1.0f);
    adaptive.reset();
    assert(adaptive.state(0).adaptiveGyroBiasXDegSec == 0.0f);

    AircraftAHRS::Config disabledConfig = adaptiveConfig;
    disabledConfig.adaptiveGyroBiasEnabled = false;
    AircraftAHRS disabled(disabledConfig);
    for (uint32_t ms = 0; ms <= 1000; ms += 20) {
        if (ms % 100 == 0) disabled.updateGps(90, 30, 100, true, ms + 1);
        disabled.updateImu(0.4f, -0.2f, 0.1f, (ms + 1) * 1000,
                           0, 0, 9.80665f, true);
    }
    assert(disabled.state(1001).adaptiveGyroBiasXDegSec == 0.0f);
    assert(disabled.state(1001).adaptiveGyroBiasYDegSec == 0.0f);
    assert(disabled.state(1001).adaptiveGyroBiasZDegSec == 0.0f);

    // Leaving the 1-g magnitude envelope must fade the accelerometer roll
    // contribution rather than removing it in one sample.
    AircraftAHRS::Config fadeConfig;
    fadeConfig.adaptiveGyroBiasEnabled = false;
    fadeConfig.accelerometerRollConfidenceTimeSec = 0.5f;
    AircraftAHRS fading(fadeConfig);
    for (uint32_t ms = 0; ms <= 2000; ms += 20) {
        if (ms % 100 == 0) fading.updateGps(90, 30, 100, true, ms + 1);
        fading.updateImu(0, 0, 0, (ms + 1) * 1000,
                         0, 1.0f, 9.7555f, true);
    }
    const float confidenceBefore = fading.state(2001).accelerometerRollConfidence;
    const float targetBefore = fading.state(2001).rollCorrectionTargetDeg;
    fading.updateImu(0, 0, 0, 2021000, 0, 0, 20.0f, true);
    const auto &faded = fading.state(2021);
    assert(confidenceBefore > 0.95f);
    assert(faded.accelerometerRollConfidence > 0.90f);
    assert(fabsf(faded.rollCorrectionTargetDeg - targetBefore) < 0.2f);

    puts("Aircraft AHRS kinematics tests passed");
    return 0;
}
