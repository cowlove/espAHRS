#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <esp_heap_caps.h>
#include "FusionLogFormat.h"

// TEMPORARY LOGGING MODE:
// Records are buffered in PSRAM and written synchronously when logging stops.
// This is a compatibility workaround for the T-Beam's shared QMI/SD SPI
// wiring. Replace it with a concurrent SD writer after shared-bus ownership
// is redesigned; PSRAM is not intended to be the permanent log transport or
// file-size limit.
class FusionSessionLog {
    static constexpr uint32_t MaxPayload = 512;
    static constexpr uint32_t InternalRecords = 64;
    FS *storage_ = &SD;
    uint8_t *buffer_ = nullptr;
    size_t capacityBytes_ = 0, bufferedBytes_ = 0;
    uint32_t buffered_ = 0, sequence_ = 0;
    volatile uint32_t dropped_ = 0, written_ = 0, writeErrors_ = 0;
    volatile bool active_ = false, bufferFull_ = false;
    bool psram_ = false;
    char fileName_[32] = {};
    portMUX_TYPE producerMux_ = portMUX_INITIALIZER_UNLOCKED;

    bool allocateBuffer() {
        const size_t freePsram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        // Some ESP32 builds report psramFound() even when PSRAM startup
        // failed.  Require actual free SPIRAM before selecting the PSRAM
        // capacity.  Use all but a small reserve, rather than requiring the
        // full T-Beam-sized maximum on boards with smaller PSRAM.
        const size_t reserve = 512 * 1024;
        if (freePsram > reserve) {
            capacityBytes_ = freePsram - reserve;
            buffer_ = static_cast<uint8_t *>(heap_caps_malloc(
                capacityBytes_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (buffer_) psram_ = true;
        }
        if (!buffer_) {
            capacityBytes_ = InternalRecords * (sizeof(FusionLogRecordHeader) + MaxPayload);
            buffer_ = static_cast<uint8_t *>(malloc(capacityBytes_));
            if (buffer_) psram_ = false;
        }
        buffered_ = 0; bufferedBytes_ = 0; bufferFull_ = false;
        return buffer_ != nullptr;
    }
    void releaseBuffer() {
        if (buffer_) {
            if (psram_) heap_caps_free(buffer_); else free(buffer_);
        }
        buffer_ = nullptr; capacityBytes_ = bufferedBytes_ = buffered_ = 0; psram_ = false;
    }
    bool writeBuffered() {
        if (!buffer_ || !buffered_) return false;
        pinMode(34, OUTPUT); digitalWrite(34, HIGH);
        pinMode(47, OUTPUT); digitalWrite(47, HIGH);
        File file = storage_->open(fileName_, FILE_WRITE);
        if (!file) { writeErrors_++; return false; }
        size_t offset = 0;
        while (offset + sizeof(FusionLogRecordHeader) <= bufferedBytes_) {
            FusionLogRecordHeader header;
            memcpy(&header, buffer_ + offset, sizeof(header));
            size_t recordBytes = sizeof(header) + header.payloadLength;
            if (header.payloadLength > MaxPayload || offset + recordBytes > bufferedBytes_) { ++writeErrors_; file.close(); return false; }
            bool ok = file.write(buffer_ + offset, recordBytes) == recordBytes;
            if (ok) ++written_; else { ++writeErrors_; file.close(); return false; }
            offset += recordBytes;
        }
        file.flush(); file.close();
        return true;
    }
    void event(const char *s) { append(FUSION_LOG_EVENT, micros(), s, strlen(s)); }

public:
    bool recoverLatest(FS &storage) {
        storage_ = &storage; unsigned latest = 0; bool found = false; char path[32] = {};
        File root = storage.open("/"); if (!root) return false; File entry;
        while ((entry = root.openNextFile())) {
            if (!entry.isDirectory()) { unsigned n = 0; char suffix = 0; const char *name = entry.name();
                if ((sscanf(name, "/fusion-%u.bin%c", &n, &suffix) == 1 || sscanf(name, "fusion-%u.bin%c", &n, &suffix) == 1) && (!found || n > latest)) {
                    if (name[0] == '/') strncpy(path, name, sizeof(path)-1);
                    else snprintf(path, sizeof(path), "/%s", name);
                    latest = n; found = true;
                }
            }
            entry.close();
        }
        root.close(); if (found) strncpy(fileName_, path, sizeof(fileName_)-1); return found;
    }
    bool begin(uint8_t cs, const char *prefix="/fusion-") {
        storage_ = &SD; if (!SD.begin(cs)) return false; return beginMounted(prefix);
    }
    bool begin(FS &storage, const char *prefix="/fusion-") {
        storage_ = &storage; return beginMounted(prefix);
    }
private:
    bool beginMounted(const char *prefix) {
        snprintf(fileName_, sizeof(fileName_), "%s%04lu.bin", prefix, (unsigned long)(millis() % 10000UL));
        sequence_ = 0; if (!allocateBuffer()) return false;
        Serial.printf("SESSION_LOG BUFFER bytes=%lu storage=%s\n",
                      (unsigned long)capacityBytes_, psram_ ? "PSRAM" : "INTERNAL");
        active_ = true; event("START"); return true;
    }
public:
    void stop() {
        if (!active_ && !buffer_) return;
        if (active_) event(bufferFull_ ? "STOP_BUFFER_FULL" : "STOP");
        active_ = false; writeBuffered(); releaseBuffer();
    }
    bool active() const { return active_; }
    bool pendingFlush() const { return buffer_ != nullptr && !active_; }
    uint32_t dropped() const { return dropped_; }
    uint32_t written() const { return written_; }
    uint32_t writeErrors() const { return writeErrors_; }
    uint32_t buffered() const { return buffered_; }
    uint32_t capacity() const { return (uint32_t)capacityBytes_; }
    bool usingPsram() const { return psram_; }
    uint32_t freeLogSeconds(float recordsPerSecond = 50.0f) const {
        if (!capacityBytes_ || recordsPerSecond <= 0.0f) return 0;
        // 128 bytes/record is a conservative estimate for this log mix.
        const float remainingRecords = (float)(capacityBytes_ - (bufferedBytes_ < capacityBytes_ ? bufferedBytes_ : capacityBytes_)) / 128.0f;
        return (uint32_t)(remainingRecords / recordsPerSecond);
    }
    const char *fileName() const { return fileName_; }
    bool selectFile(const char *requested) {
        if (!requested || !*requested) return false;
        const char *base = requested[0] == '/' ? requested + 1 : requested;
        if (strncmp(base, "fusion-", 7) != 0 || !strstr(base, ".bin") || strchr(base, '/') || strchr(base, '\\')) return false;
        char path[32]; snprintf(path, sizeof(path), "/%s", base);
        File f = storage_->open(path, FILE_READ);
        if (!f) return false;
        f.close(); strncpy(fileName_, path, sizeof(fileName_) - 1); fileName_[sizeof(fileName_) - 1] = 0;
        return true;
    }
    size_t fileSize() const { File f = storage_->open(fileName_, FILE_READ); if (!f) return 0; size_t n=f.size(); f.close(); return n; }
    File openRead() const { return storage_->open(fileName_, FILE_READ); }
    bool append(FusionLogType type, uint64_t t, const void *p, uint32_t n) {
        if (!active_ || n > MaxPayload) { if (active_) ++dropped_; return false; }
        portENTER_CRITICAL(&producerMux_);
        const size_t recordBytes = sizeof(FusionLogRecordHeader) + n;
        if (bufferedBytes_ + recordBytes > capacityBytes_) { bufferFull_ = true; active_ = false; ++dropped_; portEXIT_CRITICAL(&producerMux_); return false; }
        FusionLogRecordHeader header;
        header.magic=0x31474F4CUL; header.timestampUs=t; header.sequence=sequence_++; header.type=type; header.payloadLength=n;
        memcpy(buffer_ + bufferedBytes_, &header, sizeof(header));
        if (n) memcpy(buffer_ + bufferedBytes_ + sizeof(header), p, n);
        bufferedBytes_ += recordBytes; ++buffered_;
        portEXIT_CRITICAL(&producerMux_); return true;
    }
    bool appendImu(uint8_t source, uint64_t t,float gx,float gy,float gz,float ax,float ay,float az,bool valid=true) { FusionImuRecord r{t,gx,gy,gz,ax,ay,az,(uint8_t)(valid?1:0)}; return append(fusionImuLogType(source),t,&r,sizeof(r)); }
    bool appendImu(uint64_t t,float gx,float gy,float gz,float ax,float ay,float az,bool valid=true) { return appendImu(0, t, gx, gy, gz, ax, ay, az, valid); }
    bool appendGps(uint32_t t,int32_t lat,int32_t lon,int32_t alt,uint32_t speed,int32_t heading,bool valid) { FusionGpsRecord r{t,lat,lon,alt,speed,heading,(uint8_t)(valid?1:0)}; return append(FUSION_LOG_GPS,(uint64_t)t*1000ULL,&r,sizeof(r)); }
    bool appendBaro(uint64_t t,float pressure,float altitude,bool valid=true) { FusionBaroRecord r{t,pressure,altitude,(uint8_t)(valid?1:0)}; return append(FUSION_LOG_BARO,t,&r,sizeof(r)); }
    bool appendCompass(uint8_t source,uint64_t t,float x,float y,float z,bool valid=true) { if(source>3)return false; FusionCompassRecord r{t,x,y,z,(uint8_t)(valid?1:0)}; return append(fusionCompassLogType(source),t,&r,sizeof(r)); }
    void flush() {}
};
