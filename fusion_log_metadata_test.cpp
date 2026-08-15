#include "DeviceConfiguration.h"
#include "FusionLogFormat.h"
#include <cstdio>
#include <cstring>

static bool writeLog(const char *path, uint32_t hashDelta) {
    std::FILE *out = std::fopen(path, "wb");
    if (!out) return false;
    FusionLogMetadataRecord metadata{};
    metadata.formatVersion = FUSION_LOG_FORMAT_VERSION;
    metadata.halKind = static_cast<uint8_t>(DEVICE_CONFIGURATIONS[0].boardKind);
    std::memcpy(metadata.mac, DEVICE_CONFIGURATIONS[0].mac, sizeof(metadata.mac));
    std::strncpy(metadata.profileName, DEVICE_CONFIGURATIONS[0].name, sizeof(metadata.profileName)-1);
    std::strncpy(metadata.configurationRevision, DEVICE_CONFIGURATIONS[0].revision,
                 sizeof(metadata.configurationRevision)-1);
    metadata.configurationHash = deviceConfigurationHash(DEVICE_CONFIGURATIONS[0]) + hashDelta;
    FusionLogRecordHeader header{};
    header.type = FUSION_LOG_METADATA;
    header.payloadLength = sizeof(metadata);
    bool ok = std::fwrite(&header, sizeof(header), 1, out) == 1 &&
              std::fwrite(&metadata, sizeof(metadata), 1, out) == 1;
    std::fclose(out);
    return ok;
}

int main() {
    return writeLog("/tmp/espahrs-metadata-v2.bin", 0) &&
           writeLog("/tmp/espahrs-metadata-v2-stale.bin", 1) ? 0 : 1;
}
