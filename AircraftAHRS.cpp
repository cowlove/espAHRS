#include "AircraftAHRS.h"

#include <math.h>
#include <string.h>

namespace {
constexpr float DEG_TO_RAD_F = 0.01745329251994329577f;
constexpr float RAD_TO_DEG_F = 57.29577951308232088f;
constexpr float GRAVITY_MPS2 = 9.80665f;
}

AircraftAHRS::AircraftAHRS() : config_(Config()) {}
AircraftAHRS::AircraftAHRS(const Config &config) : config_(config) {}

void AircraftAHRS::setCompassCalibration(uint8_t source, const float offset[3],
                                          const float matrix[3][3]) {
    if (source >= 2) return;
    auto &cfg = config_.compass[source];
    cfg.offsetXM = offset[0]; cfg.offsetYM = offset[1]; cfg.offsetZM = offset[2];
    memcpy(cfg.calibrationMatrix, matrix, sizeof(cfg.calibrationMatrix));
}

void AircraftAHRS::reset() {
    state_ = State();
    lastImuUs_ = lastGpsMs_ = 0;
    lastBaroMs_ = 0;
    lastCompassMs_[0] = lastCompassMs_[1] = 0;
    compassHave_[0] = compassHave_[1] = false;
    filteredTurnRateRadSec_ = filteredClimbRateMps_ = 0;
    haveGpsHistory_ = false;
    filteredAccelX_ = filteredAccelY_ = 0;
    filteredAccelZ_ = -GRAVITY_MPS2;
    haveAccel_ = false;
    filteredBaroAltitudeM_ = filteredBaroRateMps_ = baroBiasM_ = 0;
    haveBaro_ = false;
}

void AircraftAHRS::updateCompass(uint8_t source, float x, float y, float z,
                                 bool valid, uint32_t nowMs) {
    if (source >= 2) return;
    auto &cfg = config_.compass[source];
    if (!valid) {
        compassHave_[source] = false;
        state_.compassValid[source] = false;
        applyCompassAiding(nowMs);
        return;
    }
    float rawX = (x - cfg.offsetXM) * cfg.scaleX;
    float rawY = (y - cfg.offsetYM) * cfg.scaleY;
    float rawZ = (z - cfg.offsetZM) * cfg.scaleZ;
    x = cfg.calibrationMatrix[0][0] * rawX + cfg.calibrationMatrix[0][1] * rawY + cfg.calibrationMatrix[0][2] * rawZ;
    y = cfg.calibrationMatrix[1][0] * rawX + cfg.calibrationMatrix[1][1] * rawY + cfg.calibrationMatrix[1][2] * rawZ;
    z = cfg.calibrationMatrix[2][0] * rawX + cfg.calibrationMatrix[2][1] * rawY + cfg.calibrationMatrix[2][2] * rawZ;
    (void)z; // Heading-only aiding; tilt compensation belongs at the call site.
    if (fabsf(x) < 1.0e-6f && fabsf(y) < 1.0e-6f) return;
    compassHeading_[source] = wrap360(atan2f(y, x) * RAD_TO_DEG_F +
                                      cfg.headingOffsetDeg + cfg.declinationDeg);
    compassHave_[source] = true;
    lastCompassMs_[source] = nowMs;
    state_.compassHeadingDeg[source] = compassHeading_[source];
    state_.compassValid[source] = true;
    applyCompassAiding(nowMs);
}

void AircraftAHRS::applyCompassAiding(uint32_t nowMs) {
    float sumX = 0.0f, sumY = 0.0f, totalWeight = 0.0f;
    for (int i = 0; i < 2; ++i) {
        const auto &cfg = config_.compass[i];
        bool fresh = compassHave_[i] && lastCompassMs_[i] &&
            (uint32_t)(nowMs - lastCompassMs_[i]) <= cfg.timeoutSec * 1000.0f;
        state_.compassValid[i] = fresh;
        if (!fresh || cfg.weight <= 0.0f) continue;
        float r = compassHeading_[i] * DEG_TO_RAD_F;
        sumX += cfg.weight * cosf(r);
        sumY += cfg.weight * sinf(r);
        totalWeight += cfg.weight;
    }
    if (totalWeight <= 0.0f) {
        state_.compassAidingValid = false;
        return;
    }
    float target = wrap360(atan2f(sumY, sumX) * RAD_TO_DEG_F);
    state_.fusedCompassHeadingDeg = target;
    float dt = lastImuUs_ ? 0.01f : 0.0f;
    float tc = 0.0f;
    for (int i = 0; i < 2; ++i)
        if (state_.compassValid[i]) tc = tc > 0 ? fminf(tc, config_.compass[i].correctionTimeSec) : config_.compass[i].correctionTimeSec;
    float blend = correctionFraction(dt, tc);
    state_.headingDeg = wrap360(state_.headingDeg + blend * wrap180(target - state_.headingDeg));
    state_.compassAidingValid = true;
}

float AircraftAHRS::wrap180(float degrees) {
    while (degrees > 180) degrees -= 360;
    while (degrees <= -180) degrees += 360;
    return degrees;
}

float AircraftAHRS::wrap360(float degrees) {
    while (degrees >= 360) degrees -= 360;
    while (degrees < 0) degrees += 360;
    return degrees;
}

float AircraftAHRS::correctionFraction(float dt, float timeConstant) {
    return timeConstant <= 0 ? 1 : 1 - expf(-dt / timeConstant);
}

void AircraftAHRS::updateImu(float pDegSec, float qDegSec, float rDegSec,
                             uint32_t nowUs, float accelX, float accelY,
                             float accelZ, bool accelerometerValid) {
    if (!lastImuUs_) {
        lastImuUs_ = nowUs;
        return;
    }
    float dt = (uint32_t)(nowUs - lastImuUs_) * 1.0e-6f;
    lastImuUs_ = nowUs;
    if (dt <= 0 || dt > 0.1f) return;

    // Standard body-rate to Euler-rate conversion (aircraft x-forward,
    // y-right, z-down). Mounting-axis remapping belongs at the call site.
    float phi = state_.rollDeg * DEG_TO_RAD_F;
    float theta = state_.pitchDeg * DEG_TO_RAD_F;
    float cosTheta = cosf(theta);
    if (fabsf(cosTheta) < 0.1f) cosTheta = copysignf(0.1f, cosTheta);
    float tanTheta = sinf(theta) / cosTheta;
    float p = pDegSec * DEG_TO_RAD_F;
    float q = qDegSec * DEG_TO_RAD_F;
    float r = rDegSec * DEG_TO_RAD_F;
    float phiDot = p + tanTheta * (q * sinf(phi) + r * cosf(phi));
    float thetaDot = q * cosf(phi) - r * sinf(phi);
    float psiDot = (q * sinf(phi) + r * cosf(phi)) / cosTheta;
    state_.rollDeg = wrap180(state_.rollDeg + phiDot * dt * RAD_TO_DEG_F);
    state_.pitchDeg = wrap180(state_.pitchDeg + thetaDot * dt * RAD_TO_DEG_F);
    state_.headingDeg = wrap360(state_.headingDeg + psiDot * dt * RAD_TO_DEG_F);

    // Accelerometers measure specific force, not gravity.  They are therefore
    // only used as a slow model-consistency aid when GPS has supplied a valid
    // coordinated-turn attitude target and the measured force is close to 1g.
    // The call site uses aircraft axes: X forward, Y right, Z down.  In this
    // convention a stationary level aircraft reports approximately (0, 0, -g).
    if (accelerometerValid) {
        float magnitude = sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);
        if (fabsf(magnitude - GRAVITY_MPS2) <= config_.accelMagnitudeToleranceMps2) {
            float alpha = correctionFraction(dt, config_.accelFilterTimeSec);
            if (!haveAccel_) {
                filteredAccelX_ = accelX; filteredAccelY_ = accelY;
                filteredAccelZ_ = accelZ; haveAccel_ = true;
            } else {
                filteredAccelX_ += alpha * (accelX - filteredAccelX_);
                filteredAccelY_ += alpha * (accelY - filteredAccelY_);
                filteredAccelZ_ += alpha * (accelZ - filteredAccelZ_);
            }
        }
    }

    state_.accelerometerAidingValid = false;
    if (haveAccel_ && state_.kinematicAidingValid) {
        float magnitude = sqrtf(filteredAccelX_ * filteredAccelX_ +
                                 filteredAccelY_ * filteredAccelY_ +
                                 filteredAccelZ_ * filteredAccelZ_);
        if (fabsf(magnitude - GRAVITY_MPS2) <= config_.accelMagnitudeToleranceMps2) {
            float accelPitch = atan2f(filteredAccelX_, -filteredAccelZ_) * RAD_TO_DEG_F;
            float horizontal = sqrtf(filteredAccelX_ * filteredAccelX_ +
                                     filteredAccelZ_ * filteredAccelZ_);
            float accelRoll = atan2f(-filteredAccelY_, horizontal) * RAD_TO_DEG_F;
            float pitchTarget = state_.gpsFlightPathDeg + config_.angleOfAttackDeg;
            float blend = correctionFraction(dt, config_.accelCorrectionTimeSec);
            state_.pitchDeg += blend * wrap180(accelPitch - pitchTarget);
            state_.rollDeg += blend * wrap180(accelRoll - state_.gpsBankDeg);
            state_.accelerometerAidingValid = true;
        }
    }
}

void AircraftAHRS::updateGps(float trackDeg, float speed, float altitudeM,
                             bool fixValid, uint32_t nowMs) {
    state_.gpsValid = fixValid;
    if (!fixValid) {
        state_.kinematicAidingValid = false;
        return;
    }

    trackDeg = wrap360(trackDeg);
    state_.gpsTrackDeg = trackDeg;
    state_.groundSpeedMps = speed;
    state_.gpsAltitudeM = altitudeM;
    float trackRad = trackDeg * DEG_TO_RAD_F;
    state_.velocityNorthMps = speed * cosf(trackRad);
    state_.velocityEastMps = speed * sinf(trackRad);

    bool moving = speed >= config_.minimumGroundSpeedMps;
    if (!moving) state_.kinematicAidingValid = false;
    float dt = lastGpsMs_ ? (uint32_t)(nowMs - lastGpsMs_) * 0.001f : 0;
    if (moving && haveGpsHistory_ && dt > 0.02f && dt < 2.0f) {
        float derivativeAlpha = correctionFraction(dt, config_.gpsDerivativeTimeSec);
        float turnRate = wrap180(trackDeg - lastTrackDeg_) * DEG_TO_RAD_F / dt;
        float gpsClimbRate = (altitudeM - lastAltitudeM_) / dt;
        filteredTurnRateRadSec_ += derivativeAlpha * (turnRate - filteredTurnRateRadSec_);
        filteredClimbRateMps_ += derivativeAlpha * (gpsClimbRate - filteredClimbRateMps_);
        state_.gpsBankDeg = atanf(speed * filteredTurnRateRadSec_ / GRAVITY_MPS2) * RAD_TO_DEG_F;
        float climbRate = selectedClimbRate(nowMs);
        if (haveBaro_ && lastBaroMs_ &&
            (uint32_t)(nowMs - lastBaroMs_) <= config_.baroTimeoutSec * 1000.0f) {
            float biasTarget = filteredBaroAltitudeM_ - altitudeM;
            float biasAlpha = correctionFraction(dt, config_.baroGpsBiasTimeSec);
            baroBiasM_ += biasAlpha * (biasTarget - baroBiasM_);
        }
        state_.gpsFlightPathDeg = atan2f(climbRate, speed) * RAD_TO_DEG_F;
        state_.fusedClimbRateMps = climbRate;
        state_.fusedAltitudeM = haveBaro_ ? filteredBaroAltitudeM_ - baroBiasM_ : altitudeM;
        state_.verticalAidingValid = true; // GPS vertical rate is available here.

        state_.headingDeg = wrap360(state_.headingDeg +
            correctionFraction(dt, config_.yawCorrectionTimeSec) *
            wrap180(trackDeg - state_.headingDeg));
        state_.rollDeg += correctionFraction(dt, config_.rollCorrectionTimeSec) *
                          wrap180(state_.gpsBankDeg - state_.rollDeg);
        float pitchReference = state_.gpsFlightPathDeg + config_.angleOfAttackDeg;
        state_.pitchDeg += correctionFraction(dt, config_.pitchCorrectionTimeSec) *
                           wrap180(pitchReference - state_.pitchDeg);
        state_.kinematicAidingValid = true;
        state_.headingValid = true;
    } else if (moving && !state_.headingValid) {
        state_.headingDeg = trackDeg;
        state_.headingValid = true;
    }

    lastTrackDeg_ = trackDeg;
    lastAltitudeM_ = altitudeM;
    lastGpsMs_ = nowMs;
    haveGpsHistory_ = moving;
}

void AircraftAHRS::updateBaro(float pressureAltitudeM, bool valid, uint32_t nowMs) {
    if (!valid) {
        state_.barometerValid = false;
        state_.verticalAidingValid = false;
        return;
    }

    if (!lastBaroMs_) {
        lastBaroMs_ = nowMs;
        filteredBaroAltitudeM_ = pressureAltitudeM;
        filteredBaroRateMps_ = 0;
        haveBaro_ = true;
    } else {
        float dt = (uint32_t)(nowMs - lastBaroMs_) * 0.001f;
        lastBaroMs_ = nowMs;
        if (dt <= 0 || dt > 5.0f) return;
        float previous = filteredBaroAltitudeM_;
        float alphaAltitude = correctionFraction(dt, config_.baroAltitudeFilterTimeSec);
        filteredBaroAltitudeM_ += alphaAltitude * (pressureAltitudeM - filteredBaroAltitudeM_);
        float measuredRate = (filteredBaroAltitudeM_ - previous) / dt;
        float alphaRate = correctionFraction(dt, config_.baroRateFilterTimeSec);
        filteredBaroRateMps_ += alphaRate * (measuredRate - filteredBaroRateMps_);
    }

    state_.baroAltitudeM = filteredBaroAltitudeM_;
    state_.baroClimbRateMps = filteredBaroRateMps_;
    state_.barometerValid = true;
    state_.verticalAidingValid = true;
    state_.baroAgeMs = 0;
    state_.fusedAltitudeM = filteredBaroAltitudeM_ - baroBiasM_;
    state_.fusedClimbRateMps = filteredBaroRateMps_;
}

float AircraftAHRS::selectedClimbRate(uint32_t nowMs) const {
    if (haveBaro_ && lastBaroMs_ &&
        (uint32_t)(nowMs - lastBaroMs_) <= config_.baroTimeoutSec * 1000.0f)
        return filteredBaroRateMps_;
    return filteredClimbRateMps_;
}

const AircraftAHRS::State &AircraftAHRS::state(uint32_t nowMs) {
    applyCompassAiding(nowMs);
    state_.gpsAgeMs = lastGpsMs_ ? (uint32_t)(nowMs - lastGpsMs_) : UINT32_MAX;
    if (!lastGpsMs_ || state_.gpsAgeMs > config_.gpsTimeoutSec * 1000) {
        state_.gpsValid = false;
        state_.kinematicAidingValid = false;
    }
    state_.baroAgeMs = lastBaroMs_ ? (uint32_t)(nowMs - lastBaroMs_) : UINT32_MAX;
    state_.barometerValid = haveBaro_ && state_.baroAgeMs <= config_.baroTimeoutSec * 1000.0f;
    if (state_.barometerValid) {
        state_.baroAltitudeM = filteredBaroAltitudeM_;
        state_.baroClimbRateMps = filteredBaroRateMps_;
        state_.fusedAltitudeM = filteredBaroAltitudeM_ - baroBiasM_;
        state_.fusedClimbRateMps = selectedClimbRate(nowMs);
    }
    state_.verticalAidingValid = state_.barometerValid || state_.kinematicAidingValid;
    return state_;
}

