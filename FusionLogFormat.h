#pragma once

#include <stdint.h>

// Stream IDs are part of the on-disk ABI.  The original single-IMU and two
// compass values remain valid aliases for stream zero/one, so old logs replay
// unchanged while new logs can carry parallel devices.
enum FusionLogType : uint8_t { FUSION_LOG_EVENT=1, FUSION_LOG_G5_RAW_ESPNOW=2,
    FUSION_LOG_G5_PACKET=3, FUSION_LOG_IMU0=4, FUSION_LOG_GPS=5,
    FUSION_LOG_BARO=6, FUSION_LOG_COMPASS0=7, FUSION_LOG_COMPASS1=8,
    FUSION_LOG_IMU1=9, FUSION_LOG_IMU2=10, FUSION_LOG_IMU3=11,
    FUSION_LOG_COMPASS2=12, FUSION_LOG_COMPASS3=13,
    FUSION_LOG_IMU=FUSION_LOG_IMU0 };

inline FusionLogType fusionImuLogType(uint8_t source) {
    switch (source) {
    case 1: return FUSION_LOG_IMU1;
    case 2: return FUSION_LOG_IMU2;
    case 3: return FUSION_LOG_IMU3;
    default: return FUSION_LOG_IMU0;
    }
}
inline FusionLogType fusionCompassLogType(uint8_t source) {
    return static_cast<FusionLogType>(FUSION_LOG_COMPASS0 + source);
}
inline bool fusionLogIsImu(FusionLogType type) {
    return type == FUSION_LOG_IMU0 || type == FUSION_LOG_IMU1 ||
           type == FUSION_LOG_IMU2 || type == FUSION_LOG_IMU3;
}
inline bool fusionLogIsCompass(FusionLogType type) {
    return type >= FUSION_LOG_COMPASS0 && type <= FUSION_LOG_COMPASS3;
}
inline uint8_t fusionLogSource(FusionLogType type) {
    switch (type) {
    case FUSION_LOG_IMU1: case FUSION_LOG_COMPASS1: return 1;
    case FUSION_LOG_IMU2: case FUSION_LOG_COMPASS2: return 2;
    case FUSION_LOG_IMU3: case FUSION_LOG_COMPASS3: return 3;
    default: return 0;
    }
}
struct FusionImuRecord { uint64_t timestampUs; float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ; uint8_t valid; };
struct FusionGpsRecord { uint32_t timestampMs; int32_t latitudeE7, longitudeE7;
    int32_t altitudeMm; uint32_t groundSpeedMmps; int32_t headingE5; uint8_t fixValid; };
struct FusionBaroRecord { uint64_t timestampUs; float pressurePa, altitudeM; uint8_t valid; };
struct FusionCompassRecord { uint64_t timestampUs; float x, y, z; uint8_t valid; };
// The trailing padding is part of the on-disk ABI: sizeof(...) is 32 bytes.
struct FusionLogRecordHeader { uint32_t magic=0x31474F4CUL; uint64_t timestampUs=0;
    uint32_t sequence=0; uint8_t type=0; uint8_t reserved[3]={}; uint32_t payloadLength=0; };
static_assert(sizeof(FusionLogRecordHeader) == 32, "unexpected log header ABI");
