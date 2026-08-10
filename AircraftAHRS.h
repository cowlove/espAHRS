#pragma once

#include <stdint.h>

// Deliberately narrow GPS-aided AHRS for coordinated, low-wind airplane flight.
// Gyros provide short-term attitude; GPS track, turn rate and climb angle provide
// the long-term yaw, roll and pitch references. Accelerometers contribute a
// gated roll observation when their magnitude is consistent with gravity;
// coordinated-turn bank is added as a separate target term.
class AircraftAHRS {
public:
    struct Config {
        float minimumGroundSpeedMps = 5.0f;
        float yawCorrectionTimeSec = 2.0f;
        float rollCorrectionTimeSec = 4.0f;
        float pitchCorrectionTimeSec = 8.0f;
        float gpsDerivativeTimeSec = 1.5f;
        float fusedHeadingFilterTimeSec = 0.35f;
        float gpsHeadingSpeedThresholdMps = 20.576f; // 40 kt
        float gpsHeadingWeight = 3.0f;
        float angleOfAttackDeg = 0.0f;
        float gpsTimeoutSec = 1.0f;
        float accelCorrectionTimeSec = 12.0f;
        float accelFilterTimeSec = 0.25f;
        float accelMagnitudeToleranceMps2 = 1.5f;
        float accelerometerRollWeight = 1.0f;
        float turnBankWeight = 1.0f;
        float maximumBankTargetDeg = 60.0f;
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
        float gpsBankDeg = 0;
        float fusedHeadingDeg = 0;
        float fusedTurnRateDegSec = 0;
        float bankTargetDeg = 0;
        float accelerometerRollDeg = 0;
        float rollCorrectionTargetDeg = 0;
        float gpsFlightPathDeg = 0;
        bool gpsValid = false;
        bool kinematicAidingValid = false;
        bool headingValid = false;
        bool compassValid[2] = {false, false};
        float compassHeadingDeg[2] = {0.0f, 0.0f};
        float fusedCompassHeadingDeg = 0.0f;
        bool compassAidingValid = false;
        bool accelerometerAidingValid = false;
        bool headingAidingValid = false;
        bool barometerValid = false;
        bool verticalAidingValid = false;
        float baroAltitudeM = 0;
        float baroClimbRateMps = 0;
        float fusedAltitudeM = 0;
        float fusedClimbRateMps = 0;
        uint32_t gpsAgeMs = UINT32_MAX;
        uint32_t baroAgeMs = UINT32_MAX;
    };

    AircraftAHRS();
    explicit AircraftAHRS(const Config &config);
    void reset();
    void updateImu(float gyroXDegSec, float gyroYDegSec, float gyroZDegSec,
                   uint32_t nowUs, float accelXMps2 = 0.0f,
                   float accelYMps2 = 0.0f, float accelZMps2 = 0.0f,
                   bool accelerometerValid = false);
    void updateGps(float trackDeg, float groundSpeedMps, float altitudeM,
                   bool fixValid, uint32_t nowMs);
    // Supply calibrated body-frame magnetic vectors. Calibration is kept
    // independently for each source in Config::compass[source].
    void updateCompass(uint8_t source, float x, float y, float z,
                       bool valid, uint32_t nowMs);
    void setCompassCalibration(uint8_t source, const float offset[3],
                               const float matrix[3][3]);
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
    bool compassHave_[2] = {false, false};
    float lastTrackDeg_ = 0;
    float lastAltitudeM_ = 0;
    float filteredTurnRateRadSec_ = 0;
    float filteredFusedTurnRateRadSec_ = 0;
    float filteredFusedHeadingDeg_ = 0;
    float previousFusedHeadingDeg_ = 0;
    bool haveFusedHeading_ = false;
    float filteredClimbRateMps_ = 0;
    bool haveGpsHistory_ = false;
    float filteredAccelX_ = 0;
    float filteredAccelY_ = 0;
    float filteredAccelZ_ = -9.80665f;
    bool haveAccel_ = false;
    float filteredBaroAltitudeM_ = 0;
    float filteredBaroRateMps_ = 0;
    float baroBiasM_ = 0;
    bool haveBaro_ = false;

    static float wrap180(float degrees);
    static float wrap360(float degrees);
    static float correctionFraction(float dt, float timeConstant);
    float selectedClimbRate(uint32_t nowMs) const;
    void applyHeadingAiding(uint32_t nowMs);
};
