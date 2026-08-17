#pragma once

#include <stdint.h>

#include "DipAHRS.h"

// Deliberately narrow GPS-aided AHRS for coordinated, low-wind airplane flight.
// Gyros provide short-term attitude. Independent GPS-track, magnetic-heading,
// and yaw-gyro turn rates provide coordinated-turn bank observations.
// Accelerometers add a gated lateral-specific-force residual, and DipAHRS
// supplies a separate absolute roll observation.
class AircraftAHRS {
public:
    struct Config {
        float minimumGroundSpeedMps = 5.0f;
        float yawCorrectionTimeSec = 2.0f;
        float rollCorrectionTimeSec = 2.0f;
        float pitchCorrectionTimeSec = 8.0f;
        float gpsDerivativeTimeSec = 1.5f;
        float gpsLongitudinalAccelerationFilterTimeSec = 0.0f;
        float gpsLongitudinalAccelerationCompensationGain = 1.0f;
        float gpsLongitudinalAccelerationMaximumBankDeg = 10.0f;
        float fusedHeadingFilterTimeSec = 0.35f;
        float magneticDerivativeTimeSec = 1.5f;
        float yawGyroDerivativeTimeSec = 1.5f;
        float gpsHeadingSpeedThresholdMps = 20.576f; // 40 kt
        float gpsHeadingWeight = 3.0f;
        float gyroBiasXDegSec = 0.0f;
        float gyroBiasYDegSec = 0.0f;
        float gyroBiasZDegSec = 0.0f;
        float gyroAxisSignX = 1.0f;
        float gyroAxisSignY = 1.0f;
        float gyroAxisSignZ = 1.0f;
        // Per-axis scale factors applied after bias/sign correction.
        float gyroGainX = 1.0f;
        float gyroGainY = 1.0f;
        float gyroGainZ = 1.0f;
        // Per-sample aircraft-frame gyro outlier gate.  A rejected sample
        // holds the previously accepted body-rate vector.
        float gyroRateLimitDegSec = 60.0f;
        // Fixed per-sample gyro integration weight. Zero uses measured dt.
        float gyroIntegrationDtSec = 0.020f;
        // Boot-local slow integral observer. Attitude correction-target
        // innovations are projected through the Euler-rate and installed-
        // sensor rotation Jacobians back into the AHRS input gyro axes.
        bool adaptiveGyroBiasEnabled = true;
        float adaptiveGyroBiasQualificationTimeSec = 30.0f;
        float adaptiveGyroBiasLearningTimeSec = 1200.0f;
        float adaptiveGyroBiasMeanTimeSec = 10.0f;
        float adaptiveGyroBiasMaximumDegSec = 0.25f;
        float adaptiveGyroBiasMaximumBodyRateDegSec = 1.5f;
        float adaptiveGyroBiasMaximumGpsTrackRateDegSec = 0.5f;
        float adaptiveGyroBiasMaximumStdDevDegSec = 0.15f;
        float adaptiveGyroBiasAccelToleranceMps2 = 0.35f;
        float adaptiveGyroBiasStationarySpeedMps = 2.0f;
        float adaptiveGyroBiasInnovationLimitDeg = 1.0f;
        float adaptiveGyroBiasMaximumSlewDegSec2 = 0.01f;
        float accelBiasXMps2 = 0.0f;
        float accelBiasYMps2 = 0.0f;
        float accelBiasZMps2 = 0.0f;
        float verticalRateFilterTimeSec = 1.5f;
        float angleOfAttackDeg = 0.0f;
        float gpsTimeoutSec = 1.0f;
        float accelCorrectionTimeSec = 2.5f;
        float pitchGravityCorrectionTimeSec = 12.0f;
        float accelFilterTimeSec = 0.25f;
        float accelMagnitudeToleranceMps2 = 1.5f;
        // The three independent turn-rate bank estimates are normalized by
        // the sum of the weights for the currently valid sources.
        float gpsTurnRateBankWeight = 1.0f;
        // Magnetic turn-rate bank is currently disabled pending investigation
        // of noise and occasional discontinuity glitches.
        float magTurnRateBankWeight = 0.0f;
        float yawGyroTurnRateBankWeight = 0.0f;
        // Lateral specific force is an additive uncoordinated-flight residual.
        float accelerometerRollWeight = 0.4f;
        // Final normalized fusion of turn-rate/accelerometer roll and DipAHRS.
        // DipAHRS roll is retained as a diagnostic input, but is disabled in
        // the roll correction target by default because the flight data shows
        // it to be unreliable.
        float fusedTurnRateBankWeight = 20.0f;
        float dipAhrsRollWeight = 0.0f;
        float maximumBankTargetDeg = 60.0f;
        // NOAA WMM-2025 field at 47.6062 N, 122.3321 W, 0 km on 2026-08-09.
        // Declination is east-positive; inclination is down-positive in NED.
        float magneticDeclinationDeg = 14.89224f;
        float magneticInclinationDeg = 68.75569f;
        float magneticFieldMagnitudeTolerance = 0.20f;
        float magneticRollMaximumDisagreementDeg = 15.0f;
        float magneticRollMinimumGeometry = 0.25f;
        float baroAltitudeFilterTimeSec = 0.5f;
        float baroRateFilterTimeSec = 0.75f;
        float baroGpsBiasTimeSec = 30.0f;
        float baroTimeoutSec = 2.0f;
        struct CompassConfig {
            float offsetXM = 0.0f, offsetYM = 0.0f, offsetZM = 0.0f;
            float scaleX = 1.0f, scaleY = 1.0f, scaleZ = 1.0f;
            // Full raw-vector calibration: calibrated = matrix * ((raw-offset) * scale).
            // Row-major; identity preserves legacy diagonal calibration behavior.
            float calibrationMatrix[3][3] = {
                {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}
            };
            float frameRotation[3][3] = {
                {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}
            };
            float headingOffsetDeg = 0.0f;
            float declinationDeg = 0.0f;
            float correctionTimeSec = 8.0f;
            float weight = 1.0f;
            float timeoutSec = 1.0f;
        } compass[2];
    };

    struct State {
        float rollDeg = 0;
        float pitchDeg = 0;
        float headingDeg = 0;
        float groundSpeedMps = 0;
        float velocityNorthMps = 0;
        float velocityEastMps = 0;
        float gpsTrackDeg = 0;
        float gpsAltitudeM = 0;
        float gpsTurnRateBankDeg = 0;
        float magTurnRateBankDeg = 0;
        float yawGyroTurnRateBankDeg = 0;
        float fusedTurnRateBankDeg = 0;
        float fusedHeadingDeg = 0;
        float accelerometerRollDeg = 0;
        float rollCorrectionTargetDeg = 0;
        float magneticRollDeg = 0;
        float magneticRollInnovationDeg = 0;
        float magneticRollSourceDisagreementDeg = 0;
        uint8_t magneticRollSourceCount = 0;
        float accelerometerPitchDeg = 0;
        float rawAccelerometerPitchDeg = 0;
        float pitchCorrectionTargetDeg = 0;
        float gpsFlightPathDeg = 0;
        float gpsLongitudinalAccelerationMps2 = 0;
        float accelerometerMagnitudeMps2 = 0;
        bool gpsValid = false;
        bool kinematicAidingValid = false;
        bool headingValid = false;
        bool compassValid[2] = {false, false};
        float compassHeadingDeg[2] = {0.0f, 0.0f};
        float fusedCompassHeadingDeg = 0.0f;
        bool compassAidingValid = false;
        bool accelerometerAidingValid = false;
        bool accelerometerSampleAccepted = false;
        bool gpsLongitudinalAccelerationValid = false;
        bool gpsLongitudinalCompensationValid = false;
        bool magneticRollAidingValid = false;
        bool gpsTurnRateBankValid = false;
        bool magTurnRateBankValid = false;
        bool yawGyroTurnRateBankValid = false;
        bool pitchGravityAidingValid = false;
        bool headingAidingValid = false;
        bool barometerValid = false;
        bool verticalAidingValid = false;
        float baroAltitudeM = 0;
        float baroClimbRateMps = 0;
        float fusedAltitudeM = 0;
        float fusedClimbRateMps = 0;
        uint32_t gpsAgeMs = UINT32_MAX;
        uint32_t baroAgeMs = UINT32_MAX;
        uint32_t accelerometerSampleAgeMs = UINT32_MAX;
        float lastImuDtSec = 0;
        float lastPitchGyroDeltaDeg = 0;
        float lastPitchAccelCorrectionDeltaDeg = 0;
        float lastPitchGpsCorrectionDeltaDeg = 0;
        float lastPitchBodyRateDegSec = 0;
        float lastYawBodyRateDegSec = 0;
        float lastPitchQContributionDegSec = 0;
        float lastPitchYawCouplingDegSec = 0;
        bool lastGyroSampleAccepted = false;
        bool adaptiveGyroBiasQualified = false;
        float adaptiveGyroBiasQualifyingTimeSec = 0;
        float adaptiveGyroBiasXDegSec = 0;
        float adaptiveGyroBiasYDegSec = 0;
        float adaptiveGyroBiasZDegSec = 0;
        float adaptiveGyroBiasCandidateXDegSec = 0;
        float adaptiveGyroBiasCandidateYDegSec = 0;
        float adaptiveGyroBiasCandidateZDegSec = 0;
        float adaptiveGyroBiasStdDevXDegSec = 0;
        float adaptiveGyroBiasStdDevYDegSec = 0;
        float adaptiveGyroBiasStdDevZDegSec = 0;
        float adaptiveGyroBiasInformationXSec = 0;
        float adaptiveGyroBiasInformationYSec = 0;
        float adaptiveGyroBiasInformationZSec = 0;
        float adaptiveGyroBiasConfidenceX = 0;
        float adaptiveGyroBiasConfidenceY = 0;
        float adaptiveGyroBiasConfidenceZ = 0;
        float adaptiveGyroBiasRollInnovationDeg = 0;
        float adaptiveGyroBiasPitchInnovationDeg = 0;
        float adaptiveGyroBiasHeadingInnovationDeg = 0;
        uint32_t adaptiveGyroBiasRejectedInnovations = 0;
    };

    AircraftAHRS();
    explicit AircraftAHRS(const Config &config);
    void setGyroIntegrationDt(float seconds) { config_.gyroIntegrationDtSec = seconds; }
    void reset();
    void updateImu(float gyroXDegSec, float gyroYDegSec, float gyroZDegSec,
                   uint32_t nowUs, float accelXMps2 = 0.0f,
                   float accelYMps2 = 0.0f, float accelZMps2 = 0.0f,
                   bool accelerometerValid = false);
    // Rotation from the raw installed sensor frame into aircraft X-forward,
    // Y-right, Z-down. Raw gyro bias/polarity correction occurs before this
    // rotation; aircraft-frame gains and Euler conversion occur after it.
    void setSensorFrameRotation(const float matrix[3][3]);
    static void bodyRatesToEulerRates(float rollDeg, float pitchDeg,
                                      float pDegSec, float qDegSec,
                                      float rDegSec, float &rollRateDegSec,
                                      float &pitchRateDegSec,
                                      float &headingRateDegSec);
    void updateGps(float trackDeg, float groundSpeedMps, float altitudeM,
                   bool fixValid, uint32_t nowMs);
    // Supply calibrated body-frame magnetic vectors. Calibration is kept
    // independently for each source in Config::compass[source].
    void updateCompass(uint8_t source, float x, float y, float z,
                       bool valid, uint32_t nowMs);
    void setCompassCalibration(uint8_t source, const float offset[3],
                               const float matrix[3][3]);
    void setCompassFrameRotation(uint8_t source, const float matrix[3][3]);
    void setCompassFrameRotation(const float matrix[3][3]);
    // pressureAltitudeM should use the same sign convention as GPS altitude.
    // Baro is the responsive vertical signal; GPS slowly removes baro bias.
    void updateBaro(float pressureAltitudeM, bool valid, uint32_t nowMs);
    const State &state(uint32_t nowMs);

private:
    Config config_;
    State state_;
    uint32_t lastImuUs_ = 0;
    uint32_t lastGpsMs_ = 0;
    uint32_t lastBaroMs_ = 0;
    uint32_t lastCompassMs_[2] = {0, 0};
    uint32_t lastHeadingAidingMs_ = 0;
    uint32_t lastFusedHeadingMs_ = 0;
    float compassHeading_[2] = {0.0f, 0.0f};
    float compassRoll_[2] = {0.0f, 0.0f};
    float compassMagnitude_[2] = {0.0f, 0.0f};
    float compassRollGeometry_[2] = {0.0f, 0.0f};
    bool compassHave_[2] = {false, false};
    float lastTrackDeg_ = 0;
    float lastAltitudeM_ = 0;
    float lastVelocityNorthMps_ = 0;
    float lastVelocityEastMps_ = 0;
    float filteredGpsTurnRateRadSec_ = 0;
    float filteredGpsLongitudinalAccelerationMps2_ = 0;
    float filteredMagTurnRateRadSec_ = 0;
    float filteredYawGyroTurnRateRadSec_ = 0;
    float acceptedGyroXDegSec_ = 0;
    float acceptedGyroYDegSec_ = 0;
    float acceptedGyroZDegSec_ = 0;
    bool haveAcceptedGyro_ = false;
    float sensorFrameRotation_[3][3] = {
        {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}
    };
    float filteredFusedHeadingDeg_ = 0;
    float previousMagHeadingDeg_ = 0;
    uint32_t lastMagHeadingMs_ = 0;
    bool haveFusedHeading_ = false;
    float filteredClimbRateMps_ = 0;
    bool haveGpsHistory_ = false;
    float filteredAccelX_ = 0;
    float filteredAccelY_ = 0;
    float filteredAccelZ_ = -9.80665f;
    bool haveAccel_ = false;
    uint32_t lastAcceptedAccelUs_ = 0;
    uint32_t lastAccelAidingUs_ = 0;
    float filteredBaroAltitudeM_ = 0;
    float filteredBaroRateMps_ = 0;
    float baroBiasM_ = 0;
    bool haveBaro_ = false;
    float adaptiveGyroBias_[3] = {0, 0, 0};
    float adaptiveGyroBiasInformation_[3] = {0, 0, 0};
    uint32_t adaptiveGyroBiasRejectedInnovations_ = 0;

    static float wrap180(float degrees);
    static float wrap360(float degrees);
    static float correctionFraction(float dt, float timeConstant);
    float selectedClimbRate(uint32_t nowMs) const;
    void applyHeadingAiding(uint32_t nowMs, bool magneticHeadingUpdated = false);
    void updateRollCorrectionTarget(bool accelerometerResidualValid);
    void updateAdaptiveGyroBias(float dt);
};
