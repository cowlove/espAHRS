#include "AircraftAHRS.h"

#include <math.h>
#include <string.h>

namespace {
constexpr float DEG_TO_RAD_F = 0.01745329251994329577f;
constexpr float RAD_TO_DEG_F = 57.29577951308232088f;
constexpr float GRAVITY_MPS2 = 9.80665f;
void rotateVector(const float matrix[3][3], float &x, float &y, float &z) {
    const float a = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z;
    const float b = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z;
    const float c = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z;
    x = a; y = b; z = c;
}
}

AircraftAHRS::AircraftAHRS() : config_(Config()) {}
AircraftAHRS::AircraftAHRS(const Config &config) : config_(config) {}

void AircraftAHRS::setSensorFrameRotation(const float matrix[3][3]) {
    memcpy(sensorFrameRotation_, matrix, sizeof(sensorFrameRotation_));
}

void AircraftAHRS::bodyRatesToEulerRates(
        float rollDeg, float pitchDeg, float pDegSec, float qDegSec,
        float rDegSec, float &rollRateDegSec, float &pitchRateDegSec,
        float &headingRateDegSec) {
    const float phi = rollDeg * DEG_TO_RAD_F;
    const float theta = pitchDeg * DEG_TO_RAD_F;
    float cosTheta = cosf(theta);
    if (fabsf(cosTheta) < 0.1f) cosTheta = copysignf(0.1f, cosTheta);
    const float tanTheta = sinf(theta) / cosTheta;
    rollRateDegSec = pDegSec + tanTheta *
        (qDegSec * sinf(phi) + rDegSec * cosf(phi));
    pitchRateDegSec = qDegSec * cosf(phi) - rDegSec * sinf(phi);
    headingRateDegSec =
        (qDegSec * sinf(phi) + rDegSec * cosf(phi)) / cosTheta;
}

void AircraftAHRS::setCompassCalibration(uint8_t source, const float offset[3],
                                          const float matrix[3][3]) {
    if (source >= 2) return;
    auto &cfg = config_.compass[source];
    cfg.offsetXM = offset[0]; cfg.offsetYM = offset[1]; cfg.offsetZM = offset[2];
    memcpy(cfg.calibrationMatrix, matrix, sizeof(cfg.calibrationMatrix));
}

void AircraftAHRS::setCompassFrameRotation(uint8_t source,
                                            const float matrix[3][3]) {
    if (source >= 2) return;
    memcpy(config_.compass[source].frameRotation, matrix,
           sizeof(config_.compass[source].frameRotation));
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
    compassRoll_[0] = compassRoll_[1] = 0.0f;
    compassMagnitude_[0] = compassMagnitude_[1] = 0.0f;
    compassRollGeometry_[0] = compassRollGeometry_[1] = 0.0f;
    filteredGpsTurnRateRadSec_ = filteredClimbRateMps_ = 0;
    lastVelocityNorthMps_ = lastVelocityEastMps_ = 0;
    filteredGpsLongitudinalAccelerationMps2_ = 0;
    filteredMagTurnRateRadSec_ = 0;
    filteredYawGyroTurnRateRadSec_ = 0;
    acceptedGyroXDegSec_ = acceptedGyroYDegSec_ = acceptedGyroZDegSec_ = 0;
    haveAcceptedGyro_ = false;
    filteredFusedHeadingDeg_ = 0;
    previousMagHeadingDeg_ = 0;
    lastMagHeadingMs_ = 0;
    haveFusedHeading_ = false;
    haveGpsHistory_ = false;
    filteredAccelX_ = filteredAccelY_ = 0;
    filteredAccelZ_ = -GRAVITY_MPS2;
    haveAccel_ = false;
    lastAcceptedAccelUs_ = lastAccelAidingUs_ = 0;
    filteredBaroAltitudeM_ = filteredBaroRateMps_ = baroBiasM_ = 0;
    haveBaro_ = false;
    memset(adaptiveGyroBias_, 0, sizeof(adaptiveGyroBias_));
    memset(adaptiveGyroBiasInformation_, 0, sizeof(adaptiveGyroBiasInformation_));
    adaptiveGyroBiasRejectedInnovations_ = 0;
}

void AircraftAHRS::updateAdaptiveGyroBias(float dt) {
    float transform[3][3];
    const float gain[3] = {config_.gyroGainX, config_.gyroGainY,
                           config_.gyroGainZ};
    for (int row = 0; row < 3; ++row)
        for (int axis = 0; axis < 3; ++axis)
            transform[row][axis] = gain[row] * sensorFrameRotation_[row][axis];

    const float phi = state_.rollDeg * DEG_TO_RAD_F;
    const float theta = state_.pitchDeg * DEG_TO_RAD_F;
    float cosTheta = cosf(theta);
    if (fabsf(cosTheta) < 0.1f) cosTheta = copysignf(0.1f, cosTheta);
    const float euler[3][3] = {
        {1.0f, sinf(phi) * sinf(theta) / cosTheta,
               cosf(phi) * sinf(theta) / cosTheta},
        {0.0f, cosf(phi), -sinf(phi)},
        {0.0f, sinf(phi) / cosTheta, cosf(phi) / cosTheta}
    };
    float sensitivity[3][3]{};
    for (int measurement = 0; measurement < 3; ++measurement)
        for (int axis = 0; axis < 3; ++axis)
            for (int body = 0; body < 3; ++body)
                sensitivity[measurement][axis] +=
                    euler[measurement][body] * transform[body][axis];

    const bool valid[3] = {
        state_.accelerometerAidingValid || state_.gpsTurnRateBankValid ||
            state_.magneticRollAidingValid,
        state_.pitchGravityAidingValid,
        state_.headingAidingValid
    };
    float innovation[3] = {
        wrap180(state_.rollDeg - state_.rollCorrectionTargetDeg),
        wrap180(state_.pitchDeg - state_.pitchCorrectionTargetDeg),
        wrap180(state_.headingDeg - state_.fusedHeadingDeg)
    };
    state_.adaptiveGyroBiasRollInnovationDeg = valid[0] ? innovation[0] : 0.0f;
    state_.adaptiveGyroBiasPitchInnovationDeg = valid[1] ? innovation[1] : 0.0f;
    state_.adaptiveGyroBiasHeadingInnovationDeg = valid[2] ? innovation[2] : 0.0f;

    float numerator[3]{};
    float information[3]{};
    for (int measurement = 0; measurement < 3; ++measurement) {
        if (!valid[measurement]) continue;
        if (!isfinite(innovation[measurement]) ||
            fabsf(innovation[measurement]) >
                config_.adaptiveGyroBiasInnovationLimitDeg) {
            ++adaptiveGyroBiasRejectedInnovations_;
            continue;
        }
        for (int axis = 0; axis < 3; ++axis) {
            numerator[axis] += sensitivity[measurement][axis] *
                               innovation[measurement];
            information[axis] += sensitivity[measurement][axis] *
                                 sensitivity[measurement][axis];
        }
    }

    const float threshold = fmaxf(0.001f,
        config_.adaptiveGyroBiasQualificationTimeSec);
    for (int axis = 0; axis < 3; ++axis) {
        adaptiveGyroBiasInformation_[axis] += dt * information[axis];
        const float confidence = fminf(1.0f,
            adaptiveGyroBiasInformation_[axis] / threshold);
        if (config_.adaptiveGyroBiasEnabled && confidence >= 1.0f &&
            information[axis] > 1.0e-5f) {
            float rate = numerator[axis] /
                (information[axis] *
                 fmaxf(0.1f, config_.adaptiveGyroBiasLearningTimeSec));
            const float maxSlew = config_.adaptiveGyroBiasMaximumSlewDegSec2;
            if (maxSlew > 0.0f)
                rate = fmaxf(-maxSlew, fminf(maxSlew, rate));
            adaptiveGyroBias_[axis] += dt * rate;
            adaptiveGyroBias_[axis] = fmaxf(-config_.adaptiveGyroBiasMaximumDegSec,
                fminf(config_.adaptiveGyroBiasMaximumDegSec,
                      adaptiveGyroBias_[axis]));
        }
    }

    state_.adaptiveGyroBiasInformationXSec = adaptiveGyroBiasInformation_[0];
    state_.adaptiveGyroBiasInformationYSec = adaptiveGyroBiasInformation_[1];
    state_.adaptiveGyroBiasInformationZSec = adaptiveGyroBiasInformation_[2];
    state_.adaptiveGyroBiasConfidenceX = fminf(1.0f,
        adaptiveGyroBiasInformation_[0] / threshold);
    state_.adaptiveGyroBiasConfidenceY = fminf(1.0f,
        adaptiveGyroBiasInformation_[1] / threshold);
    state_.adaptiveGyroBiasConfidenceZ = fminf(1.0f,
        adaptiveGyroBiasInformation_[2] / threshold);
    state_.adaptiveGyroBiasQualified =
        state_.adaptiveGyroBiasConfidenceX >= 1.0f ||
        state_.adaptiveGyroBiasConfidenceY >= 1.0f ||
        state_.adaptiveGyroBiasConfidenceZ >= 1.0f;
    state_.adaptiveGyroBiasQualifyingTimeSec = fminf(
        adaptiveGyroBiasInformation_[0], fminf(adaptiveGyroBiasInformation_[1],
                                                adaptiveGyroBiasInformation_[2]));
    state_.adaptiveGyroBiasXDegSec = adaptiveGyroBias_[0];
    state_.adaptiveGyroBiasYDegSec = adaptiveGyroBias_[1];
    state_.adaptiveGyroBiasZDegSec = adaptiveGyroBias_[2];
    // Retained as zero-valued legacy fields for existing telemetry readers.
    state_.adaptiveGyroBiasCandidateXDegSec = 0.0f;
    state_.adaptiveGyroBiasCandidateYDegSec = 0.0f;
    state_.adaptiveGyroBiasCandidateZDegSec = 0.0f;
    state_.adaptiveGyroBiasStdDevXDegSec = 0.0f;
    state_.adaptiveGyroBiasStdDevYDegSec = 0.0f;
    state_.adaptiveGyroBiasStdDevZDegSec = 0.0f;
    state_.adaptiveGyroBiasRejectedInnovations =
        adaptiveGyroBiasRejectedInnovations_;
}

void AircraftAHRS::updateCompass(uint8_t source, float x, float y, float z,
                                 bool valid, uint32_t nowMs) {
    if (source >= 2) return;
    auto &cfg = config_.compass[source];
    if (!valid) {
        compassHave_[source] = false;
        state_.compassValid[source] = false;
        applyHeadingAiding(nowMs, true);
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
    compassMagnitude_[source] = sqrtf(x * x + y * y + z * z);
    if (fabsf(compassMagnitude_[source] - 1.0f) >
        config_.magneticFieldMagnitudeTolerance) {
        compassHave_[source] = false;
        state_.compassValid[source] = false;
        applyHeadingAiding(nowMs, true);
        return;
    }
    DipAHRS::Config dipConfig;
    dipConfig.magneticDeclinationDeg = config_.magneticDeclinationDeg;
    dipConfig.magneticInclinationDeg = config_.magneticInclinationDeg;
    dipConfig.minimumRollGeometry = config_.magneticRollMinimumGeometry;
    DipAHRS::Observation dipObservation;
    if (!DipAHRS::observe(x, y, z, state_.pitchDeg, state_.rollDeg,
                          state_.headingDeg, dipConfig, dipObservation)) {
        compassHave_[source] = false;
        state_.compassValid[source] = false;
        applyHeadingAiding(nowMs, true);
        return;
    }
    compassRoll_[source] = dipObservation.rollDeg;
    compassRollGeometry_[source] = dipObservation.rollGeometry;
    compassHeading_[source] =
        wrap360(dipObservation.headingDeg + cfg.headingOffsetDeg);
    compassHave_[source] = true;
    lastCompassMs_[source] = nowMs;
    state_.compassHeadingDeg[source] = compassHeading_[source];
    state_.compassValid[source] = true;
    applyHeadingAiding(nowMs, true);
}

void AircraftAHRS::applyHeadingAiding(uint32_t nowMs,
                                      bool magneticHeadingUpdated) {
    float sumX = 0.0f, sumY = 0.0f, totalWeight = 0.0f;
    float rollSumX = 0.0f, rollSumY = 0.0f, rollTotalWeight = 0.0f;
    float sourceRoll[2] = {0.0f, 0.0f};
    int rollSourceCount = 0;
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
        const float rollRadians = compassRoll_[i] * DEG_TO_RAD_F;
        const float rollWeight = cfg.weight * compassRollGeometry_[i];
        rollSumX += rollWeight * cosf(rollRadians);
        rollSumY += rollWeight * sinf(rollRadians);
        rollTotalWeight += rollWeight;
        sourceRoll[rollSourceCount++] = compassRoll_[i];
    }
    bool compassAvailable = totalWeight > 0.0f;
    float magneticHeading = compassAvailable ?
        wrap360(atan2f(sumY, sumX) * RAD_TO_DEG_F) : 0.0f;
    if (compassAvailable) state_.fusedCompassHeadingDeg = magneticHeading;

    // This derivative is magnetic-only. GPS is deliberately excluded so its
    // independent track derivative cannot be counted twice in roll fusion.
    if (compassAvailable && magneticHeadingUpdated) {
        float dt = lastMagHeadingMs_ ?
            (uint32_t)(nowMs - lastMagHeadingMs_) * 0.001f : 0.0f;
        if (dt > 0.0f && dt <= 2.0f) {
            float rawRate = wrap180(magneticHeading - previousMagHeadingDeg_) *
                            DEG_TO_RAD_F / dt;
            float blend = correctionFraction(dt, config_.magneticDerivativeTimeSec);
            filteredMagTurnRateRadSec_ +=
                blend * (rawRate - filteredMagTurnRateRadSec_);
            state_.magTurnRateBankDeg = atanf(
                state_.groundSpeedMps * filteredMagTurnRateRadSec_ / GRAVITY_MPS2) *
                RAD_TO_DEG_F;
            state_.magTurnRateBankValid =
                state_.groundSpeedMps >= config_.minimumGroundSpeedMps;
        }
        previousMagHeadingDeg_ = magneticHeading;
        lastMagHeadingMs_ = nowMs;
    } else if (!compassAvailable) {
        state_.magTurnRateBankValid = false;
    }

    state_.magneticRollAidingValid = false;
    state_.magneticRollSourceCount = static_cast<uint8_t>(rollSourceCount);
    state_.magneticRollSourceDisagreementDeg = 0.0f;
    if (rollSourceCount == 2) {
        state_.magneticRollSourceDisagreementDeg =
            fabsf(wrap180(sourceRoll[0] - sourceRoll[1]));
    }
    if (rollTotalWeight > 0.0f &&
        (rollSourceCount < 2 || state_.magneticRollSourceDisagreementDeg <=
                                config_.magneticRollMaximumDisagreementDeg)) {
        state_.magneticRollDeg = wrap180(atan2f(rollSumY, rollSumX) * RAD_TO_DEG_F);
        state_.magneticRollInnovationDeg =
            wrap180(state_.magneticRollDeg - state_.rollDeg);
        state_.magneticRollAidingValid = true;
    }

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
    float headingDt = previousHeadingMs ?
        (uint32_t)(nowMs - previousHeadingMs) * 0.001f : 0.0f;
    lastHeadingAidingMs_ = nowMs;

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

void AircraftAHRS::updateRollCorrectionTarget(bool accelerometerResidualValid) {
    float weightedBank = 0.0f;
    float turnWeight = 0.0f;
    auto addTurnBank = [&](bool valid, float bank, float weight) {
        if (!valid || weight <= 0.0f) return;
        weightedBank += weight * bank;
        turnWeight += weight;
    };
    addTurnBank(state_.gpsTurnRateBankValid, state_.gpsTurnRateBankDeg,
                config_.gpsTurnRateBankWeight);
    addTurnBank(state_.magTurnRateBankValid, state_.magTurnRateBankDeg,
                config_.magTurnRateBankWeight);
    addTurnBank(state_.yawGyroTurnRateBankValid, state_.yawGyroTurnRateBankDeg,
                config_.yawGyroTurnRateBankWeight);

    float turnRateBank = turnWeight > 0.0f ? weightedBank / turnWeight : 0.0f;
    if (accelerometerResidualValid) {
        turnRateBank += config_.accelerometerRollWeight *
                        state_.accelerometerRollDeg;
    }
    state_.fusedTurnRateBankDeg = fmaxf(-config_.maximumBankTargetDeg,
        fminf(config_.maximumBankTargetDeg, turnRateBank));

    float finalSum = 0.0f;
    float finalWeight = 0.0f;
    if ((turnWeight > 0.0f || accelerometerResidualValid) &&
        config_.fusedTurnRateBankWeight > 0.0f) {
        finalSum += config_.fusedTurnRateBankWeight *
                    state_.fusedTurnRateBankDeg;
        finalWeight += config_.fusedTurnRateBankWeight;
    }
    if (state_.magneticRollAidingValid && config_.dipAhrsRollWeight > 0.0f) {
        finalSum += config_.dipAhrsRollWeight * state_.magneticRollDeg;
        finalWeight += config_.dipAhrsRollWeight;
    }
    if (finalWeight > 0.0f) {
        state_.rollCorrectionTargetDeg = finalSum / finalWeight;
    }
}

void AircraftAHRS::updateImu(float pDegSec, float qDegSec, float rDegSec,
                             uint32_t nowUs, float accelX, float accelY,
                             float accelZ, bool accelerometerValid) {
    state_.lastPitchAccelCorrectionDeltaDeg = 0;
    state_.lastPitchGpsCorrectionDeltaDeg = 0;
    state_.lastGyroSampleAccepted = true;
    pDegSec -= config_.gyroBiasXDegSec;
    qDegSec -= config_.gyroBiasYDegSec;
    rDegSec -= config_.gyroBiasZDegSec;
    pDegSec *= config_.gyroAxisSignX;
    qDegSec *= config_.gyroAxisSignY;
    rDegSec *= config_.gyroAxisSignZ;
    // The learned state is expressed in the calibrated AHRS input axes.  It
    // is removed before the fine installed-sensor rotation so the observer's
    // Jacobian can distribute Euler innovations across all three channels.
    pDegSec -= adaptiveGyroBias_[0];
    qDegSec -= adaptiveGyroBias_[1];
    rDegSec -= adaptiveGyroBias_[2];
    rotateVector(sensorFrameRotation_, pDegSec, qDegSec, rDegSec);
    pDegSec *= config_.gyroGainX;
    qDegSec *= config_.gyroGainY;
    rDegSec *= config_.gyroGainZ;
    rotateVector(sensorFrameRotation_, accelX, accelY, accelZ);
    accelX -= config_.accelBiasXMps2;
    accelY -= config_.accelBiasYMps2;
    accelZ -= config_.accelBiasZMps2;
    if (!lastImuUs_) {
        lastImuUs_ = nowUs;
        return;
    }
    float dt = (uint32_t)(nowUs - lastImuUs_) * 1.0e-6f;
    state_.lastImuDtSec = dt;
    lastImuUs_ = nowUs;
    if (dt <= 0 || dt > 0.1f) return;

    const float gyroLimit = config_.gyroRateLimitDegSec;
    if (gyroLimit > 0.0f &&
        (fabsf(pDegSec) > gyroLimit || fabsf(qDegSec) > gyroLimit ||
         fabsf(rDegSec) > gyroLimit)) {
        state_.lastGyroSampleAccepted = false;
        if (haveAcceptedGyro_) {
            pDegSec = acceptedGyroXDegSec_;
            qDegSec = acceptedGyroYDegSec_;
            rDegSec = acceptedGyroZDegSec_;
        } else {
            pDegSec = qDegSec = rDegSec = 0.0f;
        }
    } else {
        acceptedGyroXDegSec_ = pDegSec;
        acceptedGyroYDegSec_ = qDegSec;
        acceptedGyroZDegSec_ = rDegSec;
        haveAcceptedGyro_ = true;
    }

    // Standard body-rate to Euler-rate conversion (aircraft x-forward,
    // y-right, z-down). Raw-axis correction and mounting rotation above make
    // p/q/r aircraft-frame body rates before they reach this conversion.
    float phi = state_.rollDeg * DEG_TO_RAD_F;
    float phiDotDegSec, thetaDotDegSec, psiDotDegSec;
    bodyRatesToEulerRates(state_.rollDeg, state_.pitchDeg,
                          pDegSec, qDegSec, rDegSec,
                          phiDotDegSec, thetaDotDegSec, psiDotDegSec);
    state_.lastPitchBodyRateDegSec = qDegSec;
    state_.lastYawBodyRateDegSec = rDegSec;
    state_.lastPitchQContributionDegSec = qDegSec * cosf(phi);
    state_.lastPitchYawCouplingDegSec = -rDegSec * sinf(phi);
    float integrationDt = config_.gyroIntegrationDtSec > 0.0f
        ? config_.gyroIntegrationDtSec : dt;
    state_.lastPitchGyroDeltaDeg = thetaDotDegSec * integrationDt;
    state_.pitchDeg = wrap180(state_.pitchDeg + state_.lastPitchGyroDeltaDeg);
    state_.rollDeg = wrap180(state_.rollDeg + phiDotDegSec * integrationDt);
    state_.headingDeg = wrap360(state_.headingDeg + psiDotDegSec * integrationDt);

    // Gyro-only turn-rate bank estimate. Ground speed supplies the radius-to-
    // bank conversion but no GPS heading or track derivative enters here.
    float yawBlend = correctionFraction(dt, config_.yawGyroDerivativeTimeSec);
    filteredYawGyroTurnRateRadSec_ +=
        yawBlend * (rDegSec * DEG_TO_RAD_F - filteredYawGyroTurnRateRadSec_);
    state_.yawGyroTurnRateBankValid =
        state_.gpsValid &&
        state_.groundSpeedMps >= config_.minimumGroundSpeedMps;
    if (state_.yawGyroTurnRateBankValid) {
        state_.yawGyroTurnRateBankDeg = atanf(
            state_.groundSpeedMps * filteredYawGyroTurnRateRadSec_ / GRAVITY_MPS2) *
            RAD_TO_DEG_F;
    }

    // Accelerometers measure specific force, not gravity.  When the magnitude
    // is close to 1g they provide a useful roll observation.  The coordinated
    // turn bank target is added to that observation: this lets the AHRS retain
    // a measured ground tilt while also asking for the bank implied by a turn.
    // The call site uses aircraft axes: X forward, Y right, Z down.  The
    // installed sensor convention currently reports approximately (0, 0, +g)
    // for a stationary level aircraft.
    state_.accelerometerSampleAccepted = false;
    if (accelerometerValid) {
        float magnitude = sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);
        state_.accelerometerMagnitudeMps2 = magnitude;
        state_.accelerometerSampleAccepted =
            fabsf(magnitude - GRAVITY_MPS2) <=
            config_.accelMagnitudeToleranceMps2;
        if (state_.accelerometerSampleAccepted) {
            lastAcceptedAccelUs_ = nowUs;
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
    state_.gpsLongitudinalCompensationValid = false;
    bool accelerationQualityGood = false;
    if (state_.accelerometerSampleAccepted && haveAccel_) {
        float magnitude = sqrtf(filteredAccelX_ * filteredAccelX_ +
                                 filteredAccelY_ * filteredAccelY_ +
                                 filteredAccelZ_ * filteredAccelZ_);
        accelerationQualityGood = fabsf(magnitude - GRAVITY_MPS2) <=
                                  config_.accelMagnitudeToleranceMps2;
        if (accelerationQualityGood) {
            float accelAidingDt = lastAccelAidingUs_
                ? (uint32_t)(nowUs - lastAccelAidingUs_) * 1.0e-6f
                : dt;
            if (accelAidingDt > 1.0f) accelAidingDt = 1.0f;
            lastAccelAidingUs_ = nowUs;
            // Undo the current roll before extracting the fore/aft tilt.  The
            // The installed sensor convention used by the current logs has
            // gravity approximately (0, 0, +g) when level and positive
            // aircraft roll corresponds to positive Y.
            float currentRoll = state_.rollDeg * DEG_TO_RAD_F;
            float correctedZ = -sinf(currentRoll) * filteredAccelY_ +
                               cosf(currentRoll) * filteredAccelZ_;
            // Aerospace pitch is positive nose-up.  With aircraft X forward
            // and Z down, gravity/specific-force tilt produces negative X
            // during a nose-up attitude, hence the explicit X negation.
            float rawAccelPitch =
                atan2f(-filteredAccelX_, correctedZ) * RAD_TO_DEG_F;
            bool turnAllowsLongitudinalCompensation =
                !state_.gpsTurnRateBankValid ||
                fabsf(state_.gpsTurnRateBankDeg) <=
                    config_.gpsLongitudinalAccelerationMaximumBankDeg;
            state_.gpsLongitudinalCompensationValid =
                state_.gpsLongitudinalAccelerationValid &&
                turnAllowsLongitudinalCompensation;
            float longitudinalCompensation =
                state_.gpsLongitudinalCompensationValid
                    ? config_.gpsLongitudinalAccelerationCompensationGain *
                          state_.gpsLongitudinalAccelerationMps2
                    : 0.0f;
            float compensatedAccelX = filteredAccelX_ - longitudinalCompensation;
            float accelPitch = atan2f(-compensatedAccelX, correctedZ) * RAD_TO_DEG_F;
            float horizontal = sqrtf(filteredAccelX_ * filteredAccelX_ +
                                     filteredAccelZ_ * filteredAccelZ_);
            float accelRoll = atan2f(filteredAccelY_, horizontal) * RAD_TO_DEG_F;
            state_.accelerometerRollDeg = accelRoll;
            state_.rawAccelerometerPitchDeg = rawAccelPitch;
            state_.accelerometerPitchDeg = accelPitch;
            updateRollCorrectionTarget(true);
            float rollBlend = correctionFraction(
                accelAidingDt, config_.accelCorrectionTimeSec);
            state_.rollDeg += rollBlend * wrap180(state_.rollCorrectionTargetDeg - state_.rollDeg);

            state_.pitchCorrectionTargetDeg = accelPitch;
            float pitchBlend = correctionFraction(
                accelAidingDt, config_.pitchGravityCorrectionTimeSec);
            state_.lastPitchAccelCorrectionDeltaDeg = pitchBlend *
                wrap180(state_.pitchCorrectionTargetDeg - state_.pitchDeg);
            state_.pitchDeg += state_.lastPitchAccelCorrectionDeltaDeg;
            state_.pitchGravityAidingValid = true;
            state_.accelerometerAidingValid = true;
        }
    }
    if (!accelerationQualityGood &&
        (state_.gpsTurnRateBankValid || state_.magTurnRateBankValid ||
         state_.yawGyroTurnRateBankValid || state_.magneticRollAidingValid)) {
        float rollBlend = correctionFraction(dt, config_.rollCorrectionTimeSec);
        updateRollCorrectionTarget(false);
        state_.rollDeg += rollBlend *
            wrap180(state_.rollCorrectionTargetDeg - state_.rollDeg);
    }
    // Update after all production correction targets and validity flags have
    // been evaluated.  The new bias is applied beginning with the next IMU
    // sample, avoiding an algebraic loop through the current correction.
    updateAdaptiveGyroBias(dt);
}

void AircraftAHRS::updateGps(float trackDeg, float speed, float altitudeM,
                             bool fixValid, uint32_t nowMs) {
    state_.gpsValid = fixValid;
    if (!fixValid) {
        state_.kinematicAidingValid = false;
        state_.gpsTurnRateBankValid = false;
        state_.gpsLongitudinalAccelerationMps2 = 0;
        state_.gpsLongitudinalAccelerationValid = false;
        filteredGpsLongitudinalAccelerationMps2_ = 0;
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

    if (dt > 0.02f && dt < 2.0f && lastGpsMs_) {
        float accelerationNorthMps2 =
            (state_.velocityNorthMps - lastVelocityNorthMps_) / dt;
        float accelerationEastMps2 =
            (state_.velocityEastMps - lastVelocityEastMps_) / dt;
        float forwardHeadingDeg = state_.headingAidingValid
            ? state_.fusedHeadingDeg : trackDeg;
        float forwardHeadingRad = forwardHeadingDeg * DEG_TO_RAD_F;
        float measuredLongitudinalAcceleration =
            accelerationNorthMps2 * cosf(forwardHeadingRad) +
            accelerationEastMps2 * sinf(forwardHeadingRad);
        float longitudinalAccelerationBlend = correctionFraction(
            dt, config_.gpsLongitudinalAccelerationFilterTimeSec);
        filteredGpsLongitudinalAccelerationMps2_ +=
            longitudinalAccelerationBlend *
            (measuredLongitudinalAcceleration -
             filteredGpsLongitudinalAccelerationMps2_);
        state_.gpsLongitudinalAccelerationMps2 =
            filteredGpsLongitudinalAccelerationMps2_;
        state_.gpsLongitudinalAccelerationValid = true;

        float altitudeRate = (altitudeM - lastAltitudeM_) / dt;
        float rateBlend = correctionFraction(dt, config_.verticalRateFilterTimeSec);
        filteredClimbRateMps_ += rateBlend * (altitudeRate - filteredClimbRateMps_);
    }

    if (moving && haveGpsHistory_ && dt > 0.02f && dt < 2.0f) {
        float derivativeAlpha = correctionFraction(dt, config_.gpsDerivativeTimeSec);
        float turnRate = wrap180(trackDeg - lastTrackDeg_) * DEG_TO_RAD_F / dt;
        filteredGpsTurnRateRadSec_ += derivativeAlpha *
            (turnRate - filteredGpsTurnRateRadSec_);
        state_.gpsTurnRateBankDeg = atanf(
            speed * filteredGpsTurnRateRadSec_ / GRAVITY_MPS2) * RAD_TO_DEG_F;
        state_.gpsTurnRateBankValid = true;
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
        state_.lastPitchGpsCorrectionDeltaDeg =
            correctionFraction(dt, config_.pitchCorrectionTimeSec) *
            wrap180(pitchReference - state_.pitchDeg);
        state_.pitchDeg += state_.lastPitchGpsCorrectionDeltaDeg;
        state_.kinematicAidingValid = true;
        state_.headingValid = true;
    } else if (moving && !state_.headingValid) {
        state_.headingDeg = trackDeg;
        state_.headingValid = true;
    } else if (!moving) {
        state_.gpsTurnRateBankValid = false;
    }

    lastTrackDeg_ = trackDeg;
    lastAltitudeM_ = altitudeM;
    lastVelocityNorthMps_ = state_.velocityNorthMps;
    lastVelocityEastMps_ = state_.velocityEastMps;
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
    state_.gpsAgeMs = lastGpsMs_ ? (uint32_t)(nowMs - lastGpsMs_) : UINT32_MAX;
    if (!lastGpsMs_ || state_.gpsAgeMs > config_.gpsTimeoutSec * 1000) {
        state_.gpsValid = false;
        state_.kinematicAidingValid = false;
        state_.gpsTurnRateBankValid = false;
        state_.yawGyroTurnRateBankValid = false;
        state_.gpsLongitudinalAccelerationMps2 = 0;
        state_.gpsLongitudinalAccelerationValid = false;
        filteredGpsLongitudinalAccelerationMps2_ = 0;
    }
    applyHeadingAiding(nowMs);
    if (lastAcceptedAccelUs_) {
        int32_t ageUs = (int32_t)(nowMs * 1000U - lastAcceptedAccelUs_);
        state_.accelerometerSampleAgeMs = ageUs > 0
            ? (uint32_t)ageUs / 1000U : 0U;
    } else {
        state_.accelerometerSampleAgeMs = UINT32_MAX;
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
