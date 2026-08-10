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

static void makeFrameRotation(float pitchDeg, float rollDeg, float m[3][3]) {
    const float p = pitchDeg * 0.01745329252f, r = rollDeg * 0.01745329252f;
    const float cp = std::cos(p), sp = std::sin(p), cr = std::cos(r), sr = std::sin(r);
    // R = Rx(roll) * Ry(pitch), applied to sensor vectors.
    m[0][0] = cp; m[0][1] = 0; m[0][2] = sp;
    m[1][0] = sr * sp; m[1][1] = cr; m[1][2] = -sr * cp;
    m[2][0] = -cr * sp; m[2][1] = sr; m[2][2] = cr * cp;
}

static void rotateVector(const float m[3][3], float &x, float &y, float &z) {
    float a = m[0][0]*x + m[0][1]*y + m[0][2]*z;
    float b = m[1][0]*x + m[1][1]*y + m[1][2]*z;
    float c = m[2][0]*x + m[2][1]*y + m[2][2]*z;
    x = a; y = b; z = c;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s session.bin [--param name=value] [--list-params]\n", argv[0]);
        return 2;
    }
    ReplayConfig replayConfig;
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--list-params") == 0) { ReplayConfig::list(); return 0; }
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
    float sensorFrameRotation[3][3];
    makeFrameRotation(replayConfig.sensorPitchOffsetDeg,
                      replayConfig.sensorRollOffsetDeg, sensorFrameRotation);
    ahrs.setCompassFrameRotation(sensorFrameRotation);
    uint64_t records = 0, bytes = 0;
    uint32_t lastMs = 0;
    uint32_t expectedSequence = 0, sequenceGaps = 0, sequenceDuplicates = 0;
    uint32_t firstSequenceAnomaly = UINT32_MAX;
    ErrorMetric rollError, pitchError, headingError;
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
        if (h.type < 9) ++counts[h.type];
        const uint32_t nowMs = msFromUs(h.timestampUs);
        if (nowMs > lastMs) lastMs = nowMs;
        switch (h.type) {
        case FUSION_LOG_IMU: {
            if (h.payloadLength != sizeof(FusionImuRecord)) return 1;
            FusionImuRecord r; std::memcpy(&r, payload, sizeof(r));
            rotateVector(sensorFrameRotation, r.gyroX, r.gyroY, r.gyroZ);
            rotateVector(sensorFrameRotation, r.accelX, r.accelY, r.accelZ);
            ahrs.updateImu(r.gyroX, r.gyroY, r.gyroZ,
                            static_cast<uint32_t>(h.timestampUs),
                            r.accelX, r.accelY, r.accelZ, r.valid != 0);
            break;
        }
        case FUSION_LOG_COMPASS0:
        case FUSION_LOG_COMPASS1: {
            if (h.payloadLength != sizeof(FusionCompassRecord)) return 1;
            FusionCompassRecord r; std::memcpy(&r, payload, sizeof(r));
            ahrs.updateCompass(h.type == FUSION_LOG_COMPASS1 ? 1 : 0,
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
                float g5Roll, g5Pitch, g5Heading;
                bool haveRoll = g5Field(payload, h.payloadLength, "R", g5Roll);
                bool havePitch = g5Field(payload, h.payloadLength, "P", g5Pitch);
                bool haveHeading = g5Field(payload, h.payloadLength, "HDG", g5Heading);
                if (haveRoll || havePitch || haveHeading) {
                    uint64_t targetUs = h.timestampUs -
                        static_cast<int64_t>(replayConfig.g5TimeOffsetMs * 1000.0f);
                    const auto &state = stateAt(targetUs);
                    if (haveRoll) rollError.add(state.rollDeg - g5Roll);
                    if (havePitch) pitchError.add(state.pitchDeg - g5Pitch);
                    if (haveHeading) headingError.add(angleError(state.headingDeg,
                                                                  g5Heading + replayConfig.g5HeadingOffsetDeg));
                    ++g5Parsed;
                }
            }
            break;
        }
        if (h.type != FUSION_LOG_G5_PACKET && h.type != FUSION_LOG_G5_RAW_ESPNOW)
            stateHistory.push_back({h.timestampUs, ahrs.state(nowMs)});
    }
    const auto &s = ahrs.state(lastMs);
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
    rollError.print("roll_deg"); pitchError.print("pitch_deg"); headingError.print("heading_deg");
    std::printf("STATE roll=%.3f pitch=%.3f heading=%.3f fused_heading=%.3f "
                "turn_rate=%.3f bank_target=%.3f accel_roll=%.3f roll_target=%.3f "
                "compass=%.3f valid=%d\n",
                s.rollDeg, s.pitchDeg, s.headingDeg, s.fusedHeadingDeg,
                s.fusedTurnRateDegSec, s.bankTargetDeg, s.accelerometerRollDeg,
                s.rollCorrectionTargetDeg, s.fusedCompassHeadingDeg,
                s.compassAidingValid ? 1 : 0);
    return in.eof() ? 0 : 1;
}
