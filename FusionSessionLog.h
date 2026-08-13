#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "FusionLogFormat.h"

// Records are queued from the real-time producers and written incrementally
// by a low-priority SD writer task.  This keeps SD latency out of the sensor
// loop without imposing a PSRAM-sized maximum log duration.
class FusionSessionLog {
    static constexpr uint32_t QueueDepth = 128;
    static constexpr uint32_t MaxPayload = 512;
    struct QueueItem { FusionLogRecordHeader header; uint8_t payload[MaxPayload]; };
    File file_;
    QueueHandle_t queue_ = nullptr;
    TaskHandle_t task_ = nullptr;
    FS *storage_ = &SD;
    volatile uint32_t sequence_ = 0, dropped_ = 0, written_ = 0, writeErrors_ = 0;
    volatile bool active_ = false;
    uint32_t startedMs_ = 0;
    portMUX_TYPE producerMux_ = portMUX_INITIALIZER_UNLOCKED;
    char fileName_[32] = {};
    char identityPrefix_[6] = {};
    char counterKey_[14] = {};

    static bool isHex(char c) {
        return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') ||
               (c >= 'a' && c <= 'f');
    }
    bool parseCurrentSequence(const char *name, uint32_t &number) const {
        const char *base = name && name[0] == '/' ? name + 1 : name;
        if (!base || !identityPrefix_[0] || strncmp(base, identityPrefix_, 5) != 0)
            return false;
        const char *digits = base + 5;
        if (!isdigit(*digits)) return false;
        char *end = nullptr;
        unsigned long parsed = strtoul(digits, &end, 10);
        if (!end || strcmp(end, ".bin") != 0 || parsed > UINT32_MAX) return false;
        number = static_cast<uint32_t>(parsed);
        return true;
    }

    static void taskEntry(void *arg) { static_cast<FusionSessionLog *>(arg)->writerLoop(); }
    void writerLoop() {
        QueueItem item;
        uint32_t sinceFlush = 0;
        for (;;) {
            if (xQueueReceive(queue_, &item, pdMS_TO_TICKS(100)) == pdTRUE) {
                bool ok = file_.write(reinterpret_cast<const uint8_t *>(&item.header), sizeof(item.header)) == sizeof(item.header);
                if (ok && item.header.payloadLength)
                    ok = file_.write(item.payload, item.header.payloadLength) == item.header.payloadLength;
                if (ok) ++written_; else ++writeErrors_;
                if (++sinceFlush >= 16) { file_.flush(); sinceFlush = 0; }
            } else {
                if (!active_ && uxQueueMessagesWaiting(queue_) == 0) break;
                if (sinceFlush) { file_.flush(); sinceFlush = 0; }
            }
        }
        file_.flush();
        task_ = nullptr;
        vTaskDelete(nullptr);
    }
    void event(const char *s) { append(FUSION_LOG_EVENT, micros(), s, strlen(s)); }

public:
    void configureIdentity(char hal, const uint8_t mac[6]) {
        snprintf(identityPrefix_, sizeof(identityPrefix_), "%c%02X%02X",
                 hal, mac[4], mac[5]);
        snprintf(counterKey_, sizeof(counterKey_), "n%02X%02X%02X%02X%02X%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    static bool isLogFileName(const char *name) {
        const char *base = name && name[0] == '/' ? name + 1 : name;
        if (!base || strchr(base, '/') || strchr(base, '\\')) return false;
        unsigned legacy = 0; char suffix = 0;
        if (sscanf(base, "fusion-%u.bin%c", &legacy, &suffix) == 1) return true;
        size_t length = strlen(base);
        if (length < 10 || (base[0] != 'G' && base[0] != 'T') ||
            strcmp(base + length - 4, ".bin") != 0) return false;
        for (size_t i = 1; i < 5; ++i) if (!isHex(base[i])) return false;
        for (size_t i = 5; i < length - 4; ++i) if (!isdigit(base[i])) return false;
        return length > 9;
    }
    bool recoverLatest(FS &storage) {
        storage_ = &storage; uint32_t latest = 0; unsigned legacyLatest = 0;
        bool found = false, legacyFound = false; char path[32] = {}, legacyPath[32] = {};
        File root = storage.open("/"); if (!root) return false; File entry;
        while ((entry = root.openNextFile())) {
            if (!entry.isDirectory()) {
                unsigned n = 0; char suffix = 0; const char *name = entry.name();
                uint32_t current = 0;
                if (parseCurrentSequence(name, current) && (!found || current > latest)) {
                    if (name[0] == '/') strncpy(path, name, sizeof(path) - 1);
                    else snprintf(path, sizeof(path), "/%s", name);
                    latest = current; found = true;
                } else if ((sscanf(name, "/fusion-%u.bin%c", &n, &suffix) == 1 ||
                            sscanf(name, "fusion-%u.bin%c", &n, &suffix) == 1) &&
                           (!legacyFound || n > legacyLatest)) {
                    if (name[0] == '/') strncpy(legacyPath, name, sizeof(legacyPath) - 1);
                    else snprintf(legacyPath, sizeof(legacyPath), "/%s", name);
                    legacyLatest = n; legacyFound = true;
                }
            }
            entry.close();
        }
        root.close();
        if (found) strncpy(fileName_, path, sizeof(fileName_) - 1);
        else if (legacyFound) strncpy(fileName_, legacyPath, sizeof(fileName_) - 1);
        return found || legacyFound;
    }
    bool begin(uint8_t cs, const char *prefix = "/fusion-") {
        storage_ = &SD; if (!SD.begin(cs)) return false; return beginMounted(prefix);
    }
    bool begin(FS &storage, const char *prefix = "/fusion-") {
        storage_ = &storage; return beginMounted(prefix);
    }

private:
    bool beginMounted(const char *prefix) {
        if (!identityPrefix_[0] || !counterKey_[0]) return false;
        (void)prefix;
        Preferences preferences;
        if (!preferences.begin("espahrs-log", false)) return false;
        uint32_t next = preferences.getUInt(counterKey_, 1);
        do {
            snprintf(fileName_, sizeof(fileName_), "/%s%03lu.bin",
                     identityPrefix_, (unsigned long)next);
            if (!storage_->exists(fileName_)) break;
            if (next == UINT32_MAX) { preferences.end(); return false; }
            ++next;
        } while (true);
        // Reserve the number before file creation. A failed creation may
        // leave a gap, but a number can never be reused after a reset.
        if (next == UINT32_MAX || preferences.putUInt(counterKey_, next + 1) != sizeof(uint32_t)) {
            preferences.end(); return false;
        }
        preferences.end();
        file_ = storage_->open(fileName_, FILE_WRITE);
        if (!file_) return false;
        sequence_ = dropped_ = written_ = writeErrors_ = 0;
        startedMs_ = millis();
        queue_ = xQueueCreate(QueueDepth, sizeof(QueueItem));
        if (!queue_) { file_.close(); return false; }
        active_ = true;
        if (xTaskCreatePinnedToCore(taskEntry, "fusion-log", 8192, this, 1, &task_, 1) != pdPASS) {
            active_ = false; vQueueDelete(queue_); queue_ = nullptr; file_.close(); return false;
        }
        event("START");
        return true;
    }

public:
    void stop() {
        if (!active_) return;
        event("STOP");
        active_ = false;
        uint32_t deadline = millis() + 10000;
        while (task_ && (int32_t)(deadline - millis()) > 0) delay(5);
        if (task_) { vTaskDelete(task_); task_ = nullptr; }
        if (file_) { file_.flush(); file_.close(); }
        if (queue_) { vQueueDelete(queue_); queue_ = nullptr; }
    }
    bool active() const { return active_; }
    uint32_t dropped() const { return dropped_; }
    uint32_t written() const { return written_; }
    uint32_t writeErrors() const { return writeErrors_; }
    uint32_t elapsedSeconds() const {
        return active_ ? (millis() - startedMs_) / 1000U : 0U;
    }
    const char *fileName() const { return fileName_; }
    bool selectFile(const char *requested) {
        if (!requested || !*requested) return false;
        const char *base = requested[0] == '/' ? requested + 1 : requested;
        if (!isLogFileName(base)) return false;
        char path[32]; snprintf(path, sizeof(path), "/%s", base);
        File f = storage_->open(path, FILE_READ); if (!f) return false;
        f.close(); strncpy(fileName_, path, sizeof(fileName_) - 1); fileName_[sizeof(fileName_) - 1] = 0; return true;
    }
    size_t fileSize() const { File f = storage_->open(fileName_, FILE_READ); if (!f) return 0; size_t n = f.size(); f.close(); return n; }
    File openRead() const { return storage_->open(fileName_, FILE_READ); }
    bool append(FusionLogType type, uint64_t t, const void *p, uint32_t n) {
        if (!active_ || n > MaxPayload) { if (active_) ++dropped_; return false; }
        QueueItem item; item.header.magic = 0x31474F4CUL; item.header.timestampUs = t;
        item.header.type = type; item.header.payloadLength = n;
        if (n) memcpy(item.payload, p, n);
        portENTER_CRITICAL(&producerMux_);
        item.header.sequence = sequence_++;
        bool queued = xQueueSend(queue_, &item, 0) == pdTRUE;
        portEXIT_CRITICAL(&producerMux_);
        if (!queued) { ++dropped_; return false; }
        return true;
    }
    bool appendImu(uint8_t source, uint64_t t, float gx, float gy, float gz, float ax, float ay, float az, bool valid = true) {
        FusionImuRecord r{t, gx, gy, gz, ax, ay, az, (uint8_t)(valid ? 1 : 0)};
        return append(fusionImuLogType(source), t, &r, sizeof(r));
    }
    bool appendImu(uint64_t t, float gx, float gy, float gz, float ax, float ay, float az, bool valid = true) {
        return appendImu(0, t, gx, gy, gz, ax, ay, az, valid);
    }
    bool appendGps(uint32_t t, int32_t lat, int32_t lon, int32_t alt, uint32_t speed, int32_t heading, bool valid) {
        FusionGpsRecord r{t, lat, lon, alt, speed, heading, (uint8_t)(valid ? 1 : 0)};
        return append(FUSION_LOG_GPS, (uint64_t)t * 1000ULL, &r, sizeof(r));
    }
    bool appendBaro(uint64_t t, float pressure, float altitude, bool valid = true) {
        FusionBaroRecord r{t, pressure, altitude, (uint8_t)(valid ? 1 : 0)};
        return append(FUSION_LOG_BARO, t, &r, sizeof(r));
    }
    bool appendCompass(uint8_t source, uint64_t t, float x, float y, float z, bool valid = true) {
        if (source > 3) return false;
        FusionCompassRecord r{t, x, y, z, (uint8_t)(valid ? 1 : 0)};
        return append(fusionCompassLogType(source), t, &r, sizeof(r));
    }
    void flush() { if (file_) file_.flush(); }
};
