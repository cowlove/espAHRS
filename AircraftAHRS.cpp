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

void AircraftAHRS::setCompassFrameRotation(const float matrix[3][3]) {
    for (auto &cfg : config_.compass) memcpy(cfg.frameRotation, matrix,
                                             sizeof(cfg.frameRotation));
}

void AircraftAHRS::reset() {
    state_ = State();
    lastImuUs_ = lastGpsMs_ = 0;
    lastBaroMs_ = 0;
    lastCompassMs_[0] = lastCompassMs_[1] = 0;
    lastHeadingAidingMs_ = 0;
    lastFusedHeadingMs_ = 0;
    compassHave_[0] = compassHave_[1] = false;
    filteredTurnRateRadSec_ = filteredClimbRateMps_ = 0;
    filteredFusedTurnRateRadSec_ = 0;
    filteredFusedHeadingDeg_ = 0;
    previousFusedHeadingDeg_ = 0;
    haveFusedHeading_ = false;
    verticalAccelerationMps2_ = 0;
    verticalSmoothSinceMs_ = 0;
    verticalMotionStable_ = false;
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
        applyHeadingAiding(nowMs);
        return;
    }
    float rawX = (x - cfg.offsetXM) * cfg.scaleX;
    float rawY = (y - cfg.offsetYM) * cfg.scaleY;
    float rawZ = (z - cfg.offsetZM) * cfg.scaleZ;
    x = cfg.calibrationMatrix[0][0] * rawX + cfg.calibrationMatrix[0][1] * rawY + cfg.calibrationMatrix[0][2] * rawZ;
    y = cfg.calibrationMatrix[1][0] * rawX + cfg.calibrationMatrix[1][1] * rawY + cfg.calibrationMatrix[1][2] * rawZ;
    z = cfg.calibrationMatrix[2][0] * rawX + cfg.calibrationMatrix[2][1] * rawY + cfg.calibrationMatrix[2][2] * rawZ;
    float frameX = cfg.frameRotation[0][0] * x + cfg.frameRotation[0][1] * y + cfg.frameRotation[0][2] * z;
    float frameY = cfg.frameRotation[1][0] * x + cfg.frameRotation[1][1] * y + cfg.frameRotation[1][2] * z;
    float frameZ = cfg.frameRotation[2][0] * x + cfg.frameRotation[2][1] * y + cfg.frameRotation[2][2] * z;
    x = frameX; y = frameY; z = frameZ;
    (void)z; // Heading-only aiding; tilt compensation belongs at the call site.
    if (fabsf(x) < 1.0e-6f && fabsf(y) < 1.0e-6f) return;
    compassHeading_[source] = wrap360(atan2f(y, x) * RAD_TO_DEG_F +
                                      cfg.headingOffsetDeg + cfg.declinationDeg);
    compassHave_[source] = true;
    lastCompassMs_[source] = nowMs;
    state_.compassHeadingDeg[source] = compassHeading_[source];
    state_.compassValid[source] = true;
    applyHeadingAiding(nowMs);
}

void AircraftAHRS::applyHeadingAiding(uint32_t nowMs) {
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
    bool compassAvailable = totalWeight > 0.0f;
    float magneticHeading = compassAvailable ?
        wrap360(atan2f(sumY, sumX) * RAD_TO_DEG_F) : 0.0f;
    if (compassAvailable) state_.fusedCompassHeadingDeg = magneticHeading;

    // GPS track is deliberately treated as a heading source only after the
    // accepted low-speed threshold.  Below that threshold the magnetometers
    // provide the yaw reference; above it GPS track progressively dominates.
    bool gpsFresh = state_.gpsValid && lastGpsMs_ &&
        (uint32_t)(nowMs - lastGpsMs_) <= config_.gpsTimeoutSec * 1000.0f;
    float gpsWeight = 0.0f;
    if (gpsFresh && config_.gpsHeadingSpeedThresholdMps > config_.minimumGroundSpeedMps) {
        float speed = state_.groundSpeedMps;
        float speedBlend = (speed - config_.minimumGroundSpeedMps) /
            (config_.gpsHeadingSpeedThresholdMps - config_.minimumGroundSpeedMps);
        speedBlend = fmaxf(0.0f, fminf(1.0f, speedBlend));
        gpsWeight = config_.gpsHeadingWeight * speedBlend;
    }

    float fusedTargetX = 0.0f, fusedTargetY = 0.0f, fusedWeight = 0.0f;
    if (compassAvailable) {
        float r = magneticHeading * DEG_TO_RAD_F;
        fusedTargetX += totalWeight * cosf(r);
        fusedTargetY += totalWeight * sinf(r);
        fusedWeight += totalWeight;
    }
    if (gpsWeight > 0.0f) {
        float r = state_.gpsTrackDeg * DEG_TO_RAD_F;
        fusedTargetX += gpsWeight * cosf(r);
        fusedTargetY += gpsWeight * sinf(r);
        fusedWeight += gpsWeight;
    }
    if (fusedWeight <= 0.0f) {
        state_.compassAidingValid = false;
        state_.headingAidingValid = false;
        state_.bankTargetDeg = 0.0f;
        state_.fusedTurnRateDegSec = 0.0f;
        return;
    }

    float target = wrap360(atan2f(fusedTargetY, fusedTargetX) * RAD_TO_DEG_F);
    if (!haveFusedHeading_) {
        filteredFusedHeadingDeg_ = target;
        haveFusedHeading_ = true;
    } else if (lastFusedHeadingMs_ && nowMs != lastFusedHeadingMs_) {
        float dt = (uint32_t)(nowMs - lastFusedHeadingMs_) * 0.001f;
        if (dt > 0.0f && dt <= 2.0f) {
            float blend = correctionFraction(dt, config_.fusedHeadingFilterTimeSec);
            filteredFusedHeadingDeg_ = wrap360(filteredFusedHeadingDeg_ +
                blend * wrap180(target - filteredFusedHeadingDeg_));
        }
    }
    lastFusedHeadingMs_ = nowMs;
    state_.fusedHeadingDeg = filteredFusedHeadingDeg_;

    const uint32_t previousHeadingMs = lastHeadingAidingMs_;
    // Derive turn rate from the filtered fused heading.  This uses the last
    // filtered value rather than raw GPS/compass jumps, so changing source
    // weights cannot create an artificial bank command.
    float headingDt = previousHeadingMs ?
        (uint32_t)(nowMs - previousHeadingMs) * 0.001f : 0.0f;
    if (headingDt > 0.0f && headingDt <= 2.0f && haveFusedHeading_) {
        float rawTurnRate = wrap180(filteredFusedHeadingDeg_ - previousFusedHeadingDeg_) /
            headingDt * DEG_TO_RAD_F;
        float blend = correctionFraction(headingDt, config_.gpsDerivativeTimeSec);
        filteredFusedTurnRateRadSec_ += blend * (rawTurnRate - filteredFusedTurnRateRadSec_);
    }
    previousFusedHeadingDeg_ = filteredFusedHeadingDeg_;
    lastHeadingAidingMs_ = nowMs;

    state_.fusedTurnRateDegSec = filteredFusedTurnRateRadSec_ * RAD_TO_DEG_F;
    state_.gpsBankDeg = atanf(state_.groundSpeedMps * filteredTurnRateRadSec_ /
                              GRAVITY_MPS2) * RAD_TO_DEG_F;
    float bank = atanf(state_.groundSpeedMps * filteredFusedTurnRateRadSec_ /
                       GRAVITY_MPS2) * RAD_TO_DEG_F;
    state_.bankTargetDeg = fmaxf(-config_.maximumBankTargetDeg,
                                 fminf(config_.maximumBankTargetDeg, bank));

    if (!state_.headingAidingValid) {
        state_.headingDeg = filteredFusedHeadingDeg_;
    } else if (headingDt > 0.0f && headingDt <= 2.0f) {
            state_.headingDeg = wrap360(state_.headingDeg +
                correctionFraction(headingDt, config_.yawCorrectionTimeSec) *
                wrap180(filteredFusedHeadingDeg_ - state_.headingDeg));
    }
    state_.compassAidingValid = compassAvailable;
    state_.headingAidingValid = true;
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
    pDegSec -= config_.gyroBiasXDegSec;
    qDegSec -= config_.gyroBiasYDegSec;
    rDegSec -= config_.gyroBiasZDegSec;
    pDegSec *= config_.gyroAxisSignX;
    qDegSec *= config_.gyroAxisSignY;
    rDegSec *= config_.gyroAxisSignZ;
    accelX -= config_.accelBiasXMps2;
    accelY -= config_.accelBiasYMps2;
    accelZ -= config_.accelBiasZMps2;
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

    // Accelerometers measure specific force, not gravity.  When the magnitude
    // is close to 1g they provide a useful roll observation.  The coordinated
    // turn bank target is added to that observation: this lets the AHRS retain
    // a measured ground tilt while also asking for the bank implied by a turn.
    // The call site uses aircraft axes: X forward, Y right, Z down.  The
    // installed sensor convention currently reports approximately (0, 0, +g)
    // for a stationary level aircraft.
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
    state_.pitchGravityAidingValid = false;
    bool accelerationQualityGood = false;
    if (haveAccel_) {
        float magnitude = sqrtf(filteredAccelX_ * filteredAccelX_ +
                                 filteredAccelY_ * filteredAccelY_ +
                                 filteredAccelZ_ * filteredAccelZ_);
        accelerationQualityGood = fabsf(magnitude - GRAVITY_MPS2) <=
                                  config_.accelMagnitudeToleranceMps2;
        if (accelerationQualityGood) {
            // Undo the current roll before extracting the fore/aft tilt.  The
            // installed GEEK/ICM convention used by the current logs has
            // gravity approximately (0, 0, +g) when level and positive G5
            // roll corresponds to positive Y.
            float currentRoll = state_.rollDeg * DEG_TO_RAD_F;
            float correctedZ = -sinf(currentRoll) * filteredAccelY_ +
                               cosf(currentRoll) * filteredAccelZ_;
            float accelPitch = atan2f(filteredAccelX_, correctedZ) * RAD_TO_DEG_F;
            float horizontal = sqrtf(filteredAccelX_ * filteredAccelX_ +
                                     filteredAccelZ_ * filteredAccelZ_);
            float accelRoll = atan2f(filteredAccelY_, horizontal) * RAD_TO_DEG_F;
            state_.accelerometerRollDeg = accelRoll;
            state_.accelerometerPitchDeg = accelPitch;
            state_.rollCorrectionTargetDeg =
                config_.accelerometerRollWeight * accelRoll +
                config_.turnBankWeight * state_.bankTargetDeg;
            float rollBlend = correctionFraction(dt, config_.accelCorrectionTimeSec);
            state_.rollDeg += rollBlend * wrap180(state_.rollCorrectionTargetDeg - state_.rollDeg);

            if (verticalMotionStable_) {
                state_.pitchCorrectionTargetDeg = accelPitch;
                float pitchBlend = correctionFraction(
                    dt, config_.pitchGravityCorrectionTimeSec);
                state_.pitchDeg += pitchBlend *
                    wrap180(state_.pitchCorrectionTargetDeg - state_.pitchDeg);
                state_.pitchGravityAidingValid = true;
            } else if (state_.kinematicAidingValid) {
                float pitchTarget = state_.gpsFlightPathDeg + config_.angleOfAttackDeg;
                state_.pitchCorrectionTargetDeg = pitchTarget;
                state_.pitchDeg += rollBlend * wrap180(accelPitch - pitchTarget);
            }
            state_.accelerometerAidingValid = true;
        }
    }
    if (!accelerationQualityGood && state_.headingAidingValid) {
        float rollBlend = correctionFraction(dt, config_.rollCorrectionTimeSec);
        state_.rollCorrectionTargetDeg = state_.bankTargetDeg;
        state_.rollDeg += rollBlend * wrap180(state_.bankTargetDeg - state_.rollDeg);
    }
}

void AircraftAHRS::updateGps(float trackDeg, float speed, float altitudeM,
                             bool fixValid, uint32_t nowMs) {
    state_.gpsValid = fixValid;
    if (!fixValid) {
        state_.kinematicAidingValid = false;
        verticalMotionStable_ = false;
        verticalSmoothSinceMs_ = 0;
        state_.verticalMotionStable = false;
        applyHeadingAiding(nowMs);
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

    // Use the GPS altitude stream as a conservative vertical-motion gate for
    // gravity-based pitch aiding.  A slowly changing altitude is acceptable;
    // a changing climb rate is treated as vertical acceleration and disables
    // the pitch target until the stream is smooth again.
    if (dt > 0.02f && dt < 2.0f && lastGpsMs_) {
        float altitudeRate = (altitudeM - lastAltitudeM_) / dt;
        float rateBlend = correctionFraction(dt, config_.verticalRateFilterTimeSec);
        float previousRate = filteredClimbRateMps_;
        filteredClimbRateMps_ += rateBlend * (altitudeRate - filteredClimbRateMps_);
        float measuredVerticalAcceleration =
            (filteredClimbRateMps_ - previousRate) / dt;
        float accelBlend = correctionFraction(dt, config_.verticalRateFilterTimeSec);
        verticalAccelerationMps2_ += accelBlend *
            (measuredVerticalAcceleration - verticalAccelerationMps2_);
        state_.verticalAccelerationMps2 = verticalAccelerationMps2_;
        if (fabsf(verticalAccelerationMps2_) <=
            config_.verticalAccelerationToleranceMps2) {
            if (!verticalSmoothSinceMs_) verticalSmoothSinceMs_ = nowMs;
            verticalMotionStable_ =
                (uint32_t)(nowMs - verticalSmoothSinceMs_) >=
                config_.verticalSmoothnessWindowSec * 1000.0f;
        } else {
            verticalSmoothSinceMs_ = 0;
            verticalMotionStable_ = false;
        }
    }
    state_.verticalMotionStable = verticalMotionStable_;

    if (moving && haveGpsHistory_ && dt > 0.02f && dt < 2.0f) {
        float derivativeAlpha = correctionFraction(dt, config_.gpsDerivativeTimeSec);
        float turnRate = wrap180(trackDeg - lastTrackDeg_) * DEG_TO_RAD_F / dt;
        filteredTurnRateRadSec_ += derivativeAlpha * (turnRate - filteredTurnRateRadSec_);
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
    applyHeadingAiding(nowMs);
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
    applyHeadingAiding(nowMs);
    state_.gpsAgeMs = lastGpsMs_ ? (uint32_t)(nowMs - lastGpsMs_) : UINT32_MAX;
    if (!lastGpsMs_ || state_.gpsAgeMs > config_.gpsTimeoutSec * 1000) {
        state_.gpsValid = false;
        state_.kinematicAidingValid = false;
        verticalMotionStable_ = false;
        verticalSmoothSinceMs_ = 0;
    }
    state_.verticalMotionStable = verticalMotionStable_;
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
