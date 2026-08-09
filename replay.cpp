#include "AircraftAHRS.h"
#include "FusionLogFormat.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

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

int main(int argc, char **argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s session.bin\n", argv[0]);
        return 2;
    }
    std::ifstream in(argv[1], std::ios::binary);
    if (!in) { std::perror(argv[1]); return 1; }

    AircraftAHRS ahrs;
    uint64_t records = 0, bytes = 0;
    uint32_t lastMs = 0;
    uint32_t expectedSequence = 0, sequenceGaps = 0, sequenceDuplicates = 0;
    uint32_t firstSequenceAnomaly = UINT32_MAX;
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
        default:
            // Events, GPS, and G5 reference records are retained in the log;
            // only normalized sensor records drive this first replay pass.
            break;
        }
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
    std::printf("STATE roll=%.3f pitch=%.3f heading=%.3f compass=%.3f valid=%d\n",
                s.rollDeg, s.pitchDeg, s.headingDeg, s.fusedCompassHeadingDeg,
                s.compassAidingValid ? 1 : 0);
    return in.eof() ? 0 : 1;
}
