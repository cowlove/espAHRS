#pragma once

#include <stdint.h>

enum FusionLogType : uint8_t { FUSION_LOG_EVENT=1, FUSION_LOG_G5_RAW_ESPNOW=2,
    FUSION_LOG_G5_PACKET=3, FUSION_LOG_IMU=4, FUSION_LOG_GPS=5, FUSION_LOG_BARO=6,
    FUSION_LOG_COMPASS0=7, FUSION_LOG_COMPASS1=8 };
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

