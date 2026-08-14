#include "AircraftAHRS.h"
#include "FusionLogFormat.h"
#include "ReplayConfig.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

static constexpr uint32_t LogMagic = 0x31474F4CUL;

struct ReplayHeader {
    uint32_t magic;
    uint32_t padding;
    uint64_t timestampUs;
    uint32_t sequence;
    uint8_t type;
    uint8_t reserved[3];
    uint32_t payloadLength;
    uint32_t trailingPadding;
};
static_assert(sizeof(ReplayHeader) == 32, "log header ABI changed");

static uint32_t msFromUs(uint64_t us) {
    return static_cast<uint32_t>(us / 1000ULL);
}

struct ErrorMetric {
    std::vector<double> absErrors;
    double sum = 0, sumSq = 0, signedSum = 0, maximum = 0;
    void add(double error) {
        double a = std::fabs(error); absErrors.push_back(a);
        sum += a; sumSq += error * error; signedSum += error;
        maximum = std::max(maximum, a);
    }
    void print(const char *name) const {
        if (absErrors.empty()) { std::printf("ERROR %s count=0\n", name); return; }
        auto sorted = absErrors; std::sort(sorted.begin(), sorted.end());
        double p95 = sorted[(sorted.size() - 1) * 95 / 100];
        std::printf("ERROR %s count=%zu bias=%.4f mae=%.4f rmse=%.4f max=%.4f p95=%.4f\n",
                    name, absErrors.size(), signedSum / absErrors.size(),
                    sum / absErrors.size(), std::sqrt(sumSq / absErrors.size()),
                    maximum, p95);
    }
};

struct TimedState {
    uint64_t timestampUs;
    AircraftAHRS::State state;
};

struct TimedError {
    uint64_t timestampUs;
    bool haveRoll = false, havePitch = false, haveHeading = false;
    float roll = 0, pitch = 0, heading = 0;
};

static bool g5Field(const uint8_t *payload, uint32_t length, const char *key, float &value) {
    std::string text(reinterpret_cast<const char *>(payload), length);
    std::string needle = std::string(key) + "=";
    size_t begin = 0;
    while ((begin = text.find(needle, begin)) != std::string::npos) {
        if (begin && text[begin - 1] != ' ' && text[begin - 1] != '\n' && text[begin - 1] != '\r') { begin += needle.size(); continue; }
        char *end = nullptr;
        value = std::strtof(text.c_str() + begin + needle.size(), &end);
        if (end != text.c_str() + begin + needle.size()) return std::isfinite(value);
        begin += needle.size();
    }
    return false;
}

static float angleError(float estimate, float reference) {
    float e = std::fmod(estimate - reference + 540.0f, 360.0f) - 180.0f;
    return e;
}

static void rotateVector(const float m[3][3], float &x, float &y, float &z) {
    float a = m[0][0]*x + m[0][1]*y + m[0][2]*z;
    float b = m[1][0]*x + m[1][1]*y + m[1][2]*z;
    float c = m[2][0]*x + m[2][1]*y + m[2][2]*z;
    x = a; y = b; z = c;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s session.bin [--hal geek|tbeam] [--axis-remap 9 values] [--gyro-axis-remap 9 values] [--param name=value] [--roll-csv FILE] [--pitch-csv FILE] [--imu-csv FILE] [--list-params]\n", argv[0]);
        return 2;
    }
    enum class ReplayHal { Geek, TBeam } replayHal = ReplayHal::Geek;
    // Select the base profile before constructing ReplayConfig.  Parameter
    // overrides are parsed afterward, so they still take precedence.
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--hal") == 0 && i + 1 < argc) {
            const char *name = argv[++i];
            if (std::strcmp(name, "geek") == 0) replayHal = ReplayHal::Geek;
            else if (std::strcmp(name, "tbeam") == 0) replayHal = ReplayHal::TBeam;
            else { std::fprintf(stderr, "unknown HAL: %s\n", name); return 2; }
        }
    }
    const HalHardwareProfile hardware = replayHal == ReplayHal::TBeam
        ? makeTBeamSupremeProfile() : makeGeekS3Profile();
    ReplayConfig replayConfig(hardware);
    bool axisRemapOverride = false;
    float axisRemap[3][3]{};
    bool gyroAxisRemapOverride = false;
    float gyroAxisRemap[3][3]{};
    std::FILE *rollCsv = nullptr;
    std::FILE *pitchCsv = nullptr;
    std::FILE *imuCsv = nullptr;
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--list-params") == 0) { ReplayConfig::list(); return 0; }
        if (std::strcmp(argv[i], "--hal") == 0 && i + 1 < argc) {
            const char *name = argv[++i];
            if (std::strcmp(name, "geek") == 0) replayHal = ReplayHal::Geek;
            else if (std::strcmp(name, "tbeam") == 0) replayHal = ReplayHal::TBeam;
            else { std::fprintf(stderr, "unknown HAL: %s\n", name); return 2; }
            continue;
        }
        if (std::strcmp(argv[i], "--axis-remap") == 0 && i + 9 < argc) {
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    axisRemap[row][column] = std::strtof(argv[++i], nullptr);
            axisRemapOverride = true;
            continue;
        }
        if (std::strcmp(argv[i], "--gyro-axis-remap") == 0 && i + 9 < argc) {
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    gyroAxisRemap[row][column] = std::strtof(argv[++i], nullptr);
            gyroAxisRemapOverride = true;
            continue;
        }
        if (std::strcmp(argv[i], "--roll-csv") == 0 && i + 1 < argc) {
            rollCsv = std::fopen(argv[++i], "w");
            if (!rollCsv) { std::perror("--roll-csv"); return 1; }
            std::fprintf(rollCsv,
                         "time_s,g5_roll,ahrs_roll,gps_turn_rate_bank_deg,g5_slip_raw,"
                         "mag_turn_rate_bank_deg,yaw_gyro_turn_rate_bank_deg,"
                         "fused_turn_rate_bank_deg,accel_roll_deg,roll_correction_target_deg,"
                         "magnetic_roll_deg,magnetic_roll_innovation_deg,"
                         "magnetic_roll_disagreement_deg,magnetic_roll_valid,error\n");
            continue;
        }
        if (std::strcmp(argv[i], "--pitch-csv") == 0 && i + 1 < argc) {
            pitchCsv = std::fopen(argv[++i], "w");
            if (!pitchCsv) { std::perror("--pitch-csv"); return 1; }
            std::fprintf(pitchCsv,
                         "time_s,g5_pitch,ahrs_pitch,accel_pitch,g5_slip_raw,"
                         "raw_accel_pitch,raw_pitch_gyro_deg_sec,gps_longitudinal_accel_mps2,"
                         "accel_magnitude_mps2,accel_sample_accepted,"
                         "accel_sample_age_ms,gps_longitudinal_compensation_valid,"
                         "pitch_correction_target_deg,error\n");
            continue;
        }
        if (std::strcmp(argv[i], "--imu-csv") == 0 && i + 1 < argc) {
            imuCsv = std::fopen(argv[++i], "w");
            if (!imuCsv) { std::perror("--imu-csv"); return 1; }
                std::fprintf(imuCsv, "time_s,dt_s,raw_gyro_x_deg_sec,raw_gyro_y_deg_sec,raw_gyro_z_deg_sec,body_pitch_rate_deg_sec,yaw_rate_deg_sec,pitch_q_contribution_deg_sec,pitch_yaw_coupling_deg_sec,gyro_pitch_delta_deg,accel_pitch_correction_delta_deg,gps_pitch_correction_delta_deg,ahrs_roll,ahrs_pitch,gyro_sample_accepted,adaptive_bias_qualified,adaptive_bias_qualifying_sec,adaptive_bias_x_deg_sec,adaptive_bias_y_deg_sec,adaptive_bias_z_deg_sec,adaptive_candidate_x_deg_sec,adaptive_candidate_y_deg_sec,adaptive_candidate_z_deg_sec,adaptive_stddev_x_deg_sec,adaptive_stddev_y_deg_sec,adaptive_stddev_z_deg_sec\n");
            continue;
        }
        if (std::strcmp(argv[i], "--param") == 0 && i + 1 < argc) ++i;
        else if (std::strncmp(argv[i], "--param=", 8) == 0) argv[i] += 8;
        else { std::fprintf(stderr, "unknown option: %s\n", argv[i]); return 2; }
        char *equals = std::strchr(argv[i], '=');
        if (!equals) { std::fprintf(stderr, "parameter must be name=value\n"); return 2; }
        *equals = '\0'; float value = std::strtof(equals + 1, nullptr);
        if (!replayConfig.set(argv[i], value)) { std::fprintf(stderr, "unknown parameter: %s\n", argv[i]); return 2; }
    }
    std::ifstream in(argv[1], std::ios::binary);
    if (!in) { std::perror(argv[1]); return 1; }

    AircraftAHRS ahrs(replayConfig.ahrs);
    HalHardwareProfile selectedHardware = hardware;
    if (axisRemapOverride)
        std::memcpy(selectedHardware.calibration.imu[replayConfig.selectedImuSource].sensorAxisRemap,
                    axisRemap, sizeof(axisRemap));
    if (gyroAxisRemapOverride)
        std::memcpy(selectedHardware.calibration.imu[replayConfig.selectedImuSource].gyroAxisRemap,
                    gyroAxisRemap, sizeof(gyroAxisRemap));
    ahrs.setCompassCalibration(0, hardware.calibration.compass[0].offset,
                               hardware.calibration.compass[0].matrix);
    ahrs.setCompassCalibration(1, hardware.calibration.compass[1].offset,
                               hardware.calibration.compass[1].matrix);
    float sensorFrameRotation[3][3];
    halMakeSensorFrameRotation(replayConfig.sensorPitchOffsetDeg,
                               replayConfig.sensorRollOffsetDeg,
                               replayConfig.sensorYawOffsetDeg,
                               sensorFrameRotation);
    ahrs.setSensorFrameRotation(sensorFrameRotation);
    float compassFrameRotation[3][3];
    halMultiplyMatrix(sensorFrameRotation,
                      hardware.calibration.compass[0].frameRotation,
                      compassFrameRotation);
    ahrs.setCompassFrameRotation(0, compassFrameRotation);
    halMultiplyMatrix(sensorFrameRotation,
                      hardware.calibration.compass[1].frameRotation,
                      compassFrameRotation);
    ahrs.setCompassFrameRotation(1, compassFrameRotation);
    uint64_t records = 0, bytes = 0;
    uint32_t lastMs = 0;
    uint32_t expectedSequence = 0, sequenceGaps = 0, sequenceDuplicates = 0;
    uint32_t firstSequenceAnomaly = UINT32_MAX;
    bool legacyAccelUnitsDetected = false;
    float lastRawPitchGyroDegSec = NAN;
    ErrorMetric rollError, pitchError, headingError, magneticRollError;
    uint32_t magneticRollReferences = 0;
    std::vector<TimedError> timedErrors;
    uint32_t g5Parsed = 0;
    std::vector<TimedState> stateHistory;
    auto stateAt = [&](uint64_t timestampUs) -> const AircraftAHRS::State & {
        static AircraftAHRS::State fallback;
        if (stateHistory.empty()) return fallback;
        auto it = std::lower_bound(stateHistory.begin(), stateHistory.end(), timestampUs,
                                   [](const TimedState &sample, uint64_t t) {
                                       return sample.timestampUs < t;
                                   });
        if (it == stateHistory.begin()) return it->state;
        if (it == stateHistory.end()) return stateHistory.back().state;
        auto previous = it - 1;
        return (timestampUs - previous->timestampUs <= it->timestampUs - timestampUs) ?
               previous->state : it->state;
    };
    uint32_t counts[9] = {};
    ReplayHeader h{};
    while (in.read(reinterpret_cast<char *>(&h), sizeof(h))) {
        if (h.magic != LogMagic || h.payloadLength > 512) {
            std::fprintf(stderr, "invalid record at byte %llu\n",
                         static_cast<unsigned long long>(bytes));
            return 1;
        }
        uint8_t payload[512]{};
        if (!in.read(reinterpret_cast<char *>(payload), h.payloadLength)) {
            std::fprintf(stderr, "truncated payload at byte %llu\n",
                         static_cast<unsigned long long>(bytes));
            return 1;
        }
        bytes += sizeof(h) + h.payloadLength;
        ++records;
        if (h.sequence != expectedSequence) {
            if (h.sequence < expectedSequence) ++sequenceDuplicates;
            else sequenceGaps += h.sequence - expectedSequence;
            if (firstSequenceAnomaly == UINT32_MAX) firstSequenceAnomaly = h.sequence;
        }
        expectedSequence = h.sequence + 1;
        if (h.type < 14) ++counts[h.type];
        const uint32_t nowMs = msFromUs(h.timestampUs);
        if (nowMs > lastMs) lastMs = nowMs;
        switch (h.type) {
        case FUSION_LOG_IMU0:
        case FUSION_LOG_IMU1:
        case FUSION_LOG_IMU2:
        case FUSION_LOG_IMU3: {
            if (fusionLogIsImu(static_cast<FusionLogType>(h.type))) {
            if (fusionLogSource(static_cast<FusionLogType>(h.type)) != replayConfig.selectedImuSource) break;
            if (h.payloadLength != sizeof(FusionImuRecord)) return 1;
            FusionImuRecord r; std::memcpy(&r, payload, sizeof(r));
            // Early GEEK captures contain ICM-20948 milli-g values that were
            // accidentally tagged as m/s^2.  Keep those logs replayable while
            // allowing an explicit scale override for other hardware.
            float accelMagnitude = std::sqrt(r.accelX * r.accelX +
                                             r.accelY * r.accelY +
                                             r.accelZ * r.accelZ);
            float accelScale = replayConfig.accelInputScale;
            if (accelScale == 1.0f && accelMagnitude > 100.0f) {
                accelScale = 0.001f;
                legacyAccelUnitsDetected = true;
            }
            r.accelX *= accelScale; r.accelY *= accelScale; r.accelZ *= accelScale;
            r.gyroX = (r.gyroX - replayConfig.rawGyroBiasDegSec[0]) *
                      replayConfig.rawGyroAxisSign[0];
            r.gyroY = (r.gyroY - replayConfig.rawGyroBiasDegSec[1]) *
                      replayConfig.rawGyroAxisSign[1];
            r.gyroZ = (r.gyroZ - replayConfig.rawGyroBiasDegSec[2]) *
                      replayConfig.rawGyroAxisSign[2];
            const HalImuCalibration &imuCalibration =
                selectedHardware.calibration.imu[replayConfig.selectedImuSource];
            halApplySensorAxisRemap(imuCalibration.gyroAxisRemap,
                                    r.gyroX, r.gyroY, r.gyroZ);
            halApplySensorAxisRemap(imuCalibration.sensorAxisRemap,
                                    r.accelX, r.accelY, r.accelZ);
            lastRawPitchGyroDegSec = r.gyroY;
            ahrs.updateImu(r.gyroX, r.gyroY, r.gyroZ,
                            static_cast<uint32_t>(h.timestampUs),
                            r.accelX, r.accelY, r.accelZ, r.valid != 0);
            if (imuCsv) {
                const auto &s = ahrs.state(nowMs);
                std::fprintf(imuCsv, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                             h.timestampUs * 1.0e-6, s.lastImuDtSec,
                             r.gyroX, r.gyroY, r.gyroZ,
                             s.lastPitchBodyRateDegSec, s.lastYawBodyRateDegSec,
                             s.lastPitchQContributionDegSec,
                             s.lastPitchYawCouplingDegSec,
                             s.lastPitchGyroDeltaDeg,
                             s.lastPitchAccelCorrectionDeltaDeg,
                             s.lastPitchGpsCorrectionDeltaDeg,
                             s.rollDeg, s.pitchDeg,
                             s.lastGyroSampleAccepted ? 1 : 0,
                             s.adaptiveGyroBiasQualified ? 1 : 0,
                             s.adaptiveGyroBiasQualifyingTimeSec,
                             s.adaptiveGyroBiasXDegSec,
                             s.adaptiveGyroBiasYDegSec,
                             s.adaptiveGyroBiasZDegSec,
                             s.adaptiveGyroBiasCandidateXDegSec,
                             s.adaptiveGyroBiasCandidateYDegSec,
                             s.adaptiveGyroBiasCandidateZDegSec,
                             s.adaptiveGyroBiasStdDevXDegSec,
                             s.adaptiveGyroBiasStdDevYDegSec,
                             s.adaptiveGyroBiasStdDevZDegSec);
            }
            break;
            }
            break;
        }
        case FUSION_LOG_COMPASS0:
        case FUSION_LOG_COMPASS1:
        case FUSION_LOG_COMPASS2:
        case FUSION_LOG_COMPASS3: {
            if (fusionLogSource(static_cast<FusionLogType>(h.type)) != replayConfig.selectedCompassSource) break;
            if (h.payloadLength != sizeof(FusionCompassRecord)) return 1;
            FusionCompassRecord r; std::memcpy(&r, payload, sizeof(r));
            ahrs.updateCompass(0,
                               r.x, r.y, r.z, r.valid != 0, nowMs);
            break;
        }
        case FUSION_LOG_BARO: {
            if (h.payloadLength != sizeof(FusionBaroRecord)) return 1;
            FusionBaroRecord r; std::memcpy(&r, payload, sizeof(r));
            ahrs.updateBaro(r.altitudeM, r.valid != 0, nowMs);
            break;
        }
        case FUSION_LOG_GPS: {
            if (h.payloadLength != sizeof(FusionGpsRecord)) return 1;
            FusionGpsRecord r; std::memcpy(&r, payload, sizeof(r));
            ahrs.updateGps(r.headingE5 * 1.0e-5f, r.groundSpeedMmps * 1.0e-3f,
                           r.altitudeMm * 1.0e-3f, r.fixValid != 0, nowMs);
            break;
        }
        default:
            if (h.type == FUSION_LOG_G5_PACKET) {
                float g5Roll, g5Pitch, g5Heading, g5Slip = NAN;
                bool haveRoll = g5Field(payload, h.payloadLength, "R", g5Roll);
                bool havePitch = g5Field(payload, h.payloadLength, "P", g5Pitch);
                bool haveHeading = g5Field(payload, h.payloadLength, "HDG", g5Heading);
                bool haveSlip = g5Field(payload, h.payloadLength, "SL", g5Slip);
                // Administrative packets can contain unrelated fields named
                // R= or P=.  Only a complete attitude tuple is a G5 reference.
                if (haveRoll && havePitch && haveHeading) {
                    g5Pitch += replayConfig.g5PitchBiasDeg;
                    uint64_t targetUs = h.timestampUs -
                        static_cast<int64_t>(replayConfig.g5TimeOffsetMs * 1000.0f);
                    const auto &state = stateAt(targetUs);
                    if (haveRoll) rollError.add(state.rollDeg - g5Roll);
                    if (haveRoll && state.magneticRollAidingValid) {
                        magneticRollError.add(angleError(state.magneticRollDeg,
                                                         g5Roll));
                        ++magneticRollReferences;
                    }
                    if (havePitch) pitchError.add(state.pitchDeg - g5Pitch);
                    if (haveHeading) headingError.add(angleError(state.headingDeg,
                                                                  g5Heading + replayConfig.g5HeadingOffsetDeg));
                    TimedError timed{h.timestampUs, haveRoll, havePitch, haveHeading,
                                     haveRoll ? state.rollDeg - g5Roll : 0.0f,
                                     havePitch ? state.pitchDeg - g5Pitch : 0.0f,
                                     haveHeading ? angleError(state.headingDeg,
                                                               g5Heading + replayConfig.g5HeadingOffsetDeg) : 0.0f};
                    timedErrors.push_back(timed);
                    if (rollCsv) {
                        std::fprintf(rollCsv, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.6f\n",
                                     h.timestampUs * 1.0e-6,
                                     g5Roll, state.rollDeg,
                                     state.gpsTurnRateBankDeg,
                                     g5Slip,
                                     state.magTurnRateBankDeg,
                                     state.yawGyroTurnRateBankDeg,
                                     state.fusedTurnRateBankDeg,
                                     state.accelerometerRollDeg,
                                     state.rollCorrectionTargetDeg,
                                     state.magneticRollDeg,
                                     state.magneticRollInnovationDeg,
                                     state.magneticRollSourceDisagreementDeg,
                                     state.magneticRollAidingValid ? 1 : 0,
                                     state.rollDeg - g5Roll);
                    }
                    if (pitchCsv) {
                        std::fprintf(pitchCsv,
                                     "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                                     "%d,%u,%d,%.6f,%.6f\n",
                                     h.timestampUs * 1.0e-6,
                                     g5Pitch, state.pitchDeg,
                                     state.accelerometerPitchDeg,
                                     g5Slip,
                                     state.rawAccelerometerPitchDeg,
                                     lastRawPitchGyroDegSec,
                                     state.gpsLongitudinalAccelerationMps2,
                                     state.accelerometerMagnitudeMps2,
                                     state.accelerometerSampleAccepted ? 1 : 0,
                                     state.accelerometerSampleAgeMs,
                                     state.gpsLongitudinalCompensationValid ? 1 : 0,
                                     state.pitchCorrectionTargetDeg,
                                     state.pitchDeg - g5Pitch);
                    }
                    ++g5Parsed;
                }
            }
            break;
        }
        if (h.type != FUSION_LOG_G5_PACKET && h.type != FUSION_LOG_G5_RAW_ESPNOW)
            stateHistory.push_back({h.timestampUs, ahrs.state(nowMs)});
    }
    const auto &s = ahrs.state(lastMs);
    if (rollCsv) std::fclose(rollCsv);
    if (pitchCsv) std::fclose(pitchCsv);
    if (imuCsv) std::fclose(imuCsv);
    std::printf("REPLAY records=%llu bytes=%llu imu=%u compass0=%u compass1=%u baro=%u g5raw=%u g5=%u\n",
                static_cast<unsigned long long>(records),
                static_cast<unsigned long long>(bytes), counts[FUSION_LOG_IMU],
                counts[FUSION_LOG_COMPASS0], counts[FUSION_LOG_COMPASS1],
                counts[FUSION_LOG_BARO], counts[FUSION_LOG_G5_RAW_ESPNOW],
                counts[FUSION_LOG_G5_PACKET]);
    std::printf("SEQUENCE gaps=%u duplicates=%u first_anomaly=%s\n",
                sequenceGaps, sequenceDuplicates,
                firstSequenceAnomaly == UINT32_MAX ? "none" : std::to_string(firstSequenceAnomaly).c_str());
    std::printf("G5_REFERENCE parsed=%u\n", g5Parsed);
    std::printf("ACCEL_INPUT scale=%.6f legacy_mg_detected=%d\n",
                legacyAccelUnitsDetected && replayConfig.accelInputScale == 1.0f ? 0.001f : replayConfig.accelInputScale,
                legacyAccelUnitsDetected ? 1 : 0);
    rollError.print("roll_deg"); pitchError.print("pitch_deg"); headingError.print("heading_deg");
    std::printf("MAGNETIC_ROLL_REFERENCE valid=%u/%u\n",
                magneticRollReferences, g5Parsed);
    magneticRollError.print("magnetic_roll_deg");
    if (!timedErrors.empty()) {
        uint64_t first = timedErrors.front().timestampUs;
        uint64_t last = timedErrors.back().timestampUs;
        std::printf("ERROR_BY_TIME quartiles:\n");
        for (int q = 0; q < 4; ++q) {
            ErrorMetric r, p, hd;
            for (const auto &e : timedErrors) {
                uint64_t span = last > first ? last - first : 1;
                int bucket = std::min(3, static_cast<int>(((e.timestampUs - first) * 4) / span));
                if (bucket != q) continue;
                if (e.haveRoll) r.add(e.roll);
                if (e.havePitch) p.add(e.pitch);
                if (e.haveHeading) hd.add(e.heading);
            }
            std::printf("  Q%d ", q + 1);
            r.print("roll_deg");
            std::printf("    "); p.print("pitch_deg");
            std::printf("    "); hd.print("heading_deg");
        }
    }
    std::printf("STATE roll=%.3f pitch=%.3f heading=%.3f fused_heading=%.3f "
                "gps_bank=%.3f mag_bank=%.3f yaw_gyro_bank=%.3f fused_bank=%.3f "
                "accel_roll=%.3f roll_target=%.3f "
                "mag_roll=%.3f mag_innov=%.3f mag_disagree=%.3f mag_valid=%d "
                "accel_pitch=%.3f pitch_target=%.3f gps_long_accel=%.3f "
                "pitch_aiding=%d compass=%.3f valid=%d\n",
                s.rollDeg, s.pitchDeg, s.headingDeg, s.fusedHeadingDeg,
                s.gpsTurnRateBankDeg, s.magTurnRateBankDeg,
                s.yawGyroTurnRateBankDeg, s.fusedTurnRateBankDeg,
                s.accelerometerRollDeg,
                s.rollCorrectionTargetDeg, s.magneticRollDeg,
                s.magneticRollInnovationDeg, s.magneticRollSourceDisagreementDeg,
                s.magneticRollAidingValid ? 1 : 0, s.accelerometerPitchDeg,
                s.pitchCorrectionTargetDeg, s.gpsLongitudinalAccelerationMps2,
                s.pitchGravityAidingValid ? 1 : 0,
                s.fusedCompassHeadingDeg,
                s.compassAidingValid ? 1 : 0);
    return in.eof() ? 0 : 1;
}
