#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SD.h>
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
    portMUX_TYPE producerMux_ = portMUX_INITIALIZER_UNLOCKED;
    char fileName_[32] = {};

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
    bool recoverLatest(FS &storage) {
        storage_ = &storage; unsigned latest = 0; bool found = false; char path[32] = {};
        File root = storage.open("/"); if (!root) return false; File entry;
        while ((entry = root.openNextFile())) {
            if (!entry.isDirectory()) {
                unsigned n = 0; char suffix = 0; const char *name = entry.name();
                if ((sscanf(name, "/fusion-%u.bin%c", &n, &suffix) == 1 ||
                     sscanf(name, "fusion-%u.bin%c", &n, &suffix) == 1) && (!found || n > latest)) {
                    if (name[0] == '/') strncpy(path, name, sizeof(path) - 1);
                    else snprintf(path, sizeof(path), "/%s", name);
                    latest = n; found = true;
                }
            }
            entry.close();
        }
        root.close();
        if (found) strncpy(fileName_, path, sizeof(fileName_) - 1);
        return found;
    }
    bool begin(uint8_t cs, const char *prefix = "/fusion-") {
        storage_ = &SD; if (!SD.begin(cs)) return false; return beginMounted(prefix);
    }
    bool begin(FS &storage, const char *prefix = "/fusion-") {
        storage_ = &storage; return beginMounted(prefix);
    }

private:
    bool beginMounted(const char *prefix) {
        unsigned next = 0;
        File root = storage_->open("/");
        if (root) {
            File entry;
            while ((entry = root.openNextFile())) {
                if (!entry.isDirectory()) {
                    unsigned number = 0; char suffix = 0; const char *name = entry.name();
                    if ((sscanf(name, "/fusion-%u.bin%c", &number, &suffix) == 1 ||
                         sscanf(name, "fusion-%u.bin%c", &number, &suffix) == 1) &&
                        number < 10000 && number >= next) next = number + 1;
                }
                entry.close();
            }
            root.close();
        }
        if (next >= 10000) return false;
        snprintf(fileName_, sizeof(fileName_), "%s%04u.bin", prefix, next);
        if (storage_->exists(fileName_)) return false;
        file_ = storage_->open(fileName_, FILE_WRITE);
        if (!file_) return false;
        sequence_ = 0;
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
        uint32_t deadline = millis() + 2000;
        while (task_ && (int32_t)(deadline - millis()) > 0) delay(5);
        if (task_) { vTaskDelete(task_); task_ = nullptr; }
        if (file_) { file_.flush(); file_.close(); }
        if (queue_) { vQueueDelete(queue_); queue_ = nullptr; }
    }
    bool active() const { return active_; }
    uint32_t dropped() const { return dropped_; }
    uint32_t written() const { return written_; }
    uint32_t writeErrors() const { return writeErrors_; }
    const char *fileName() const { return fileName_; }
    bool selectFile(const char *requested) {
        if (!requested || !*requested) return false;
        const char *base = requested[0] == '/' ? requested + 1 : requested;
        if (strncmp(base, "fusion-", 7) != 0 || !strstr(base, ".bin") || strchr(base, '/') || strchr(base, '\\')) return false;
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
