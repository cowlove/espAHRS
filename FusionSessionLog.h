#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "FusionLogFormat.h"

class FusionSessionLog {
    // A slow card can pause writes long enough to absorb several sensor/G5
    // bursts. Keep producer-side queueing separate from SD write latency.
    static constexpr uint32_t QueueDepth = 128;
    static constexpr uint32_t MaxPayload = 512;
    struct QueueItem { FusionLogRecordHeader header; uint8_t payload[MaxPayload]; };
    File file_; QueueHandle_t queue_=nullptr; TaskHandle_t task_=nullptr;
    FS *storage_=&SD;
    volatile uint32_t sequence_=0, dropped_=0, written_=0, writeErrors_=0;
    volatile bool active_=false;
    // G5 callbacks and the main sensor loop can both produce records.  The
    // sequence assignment and queue insertion must be one critical section;
    // otherwise two producers can race and emit duplicate sequence numbers.
    portMUX_TYPE producerMux_ = portMUX_INITIALIZER_UNLOCKED;
    char fileName_[32] = {};
    static void taskEntry(void *arg) { static_cast<FusionSessionLog *>(arg)->writerLoop(); }
    void writerLoop() {
        QueueItem item;
        uint32_t sinceFlush = 0;
        for (;;) {
            if (xQueueReceive(queue_, &item, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (file_.write((const uint8_t *)&item.header, sizeof(item.header)) != sizeof(item.header) ||
                    (item.header.payloadLength && file_.write(item.payload, item.header.payloadLength) != item.header.payloadLength))
                    writeErrors_++;
                else written_++;
                if (++sinceFlush >= 16) {
                    file_.flush();
                    sinceFlush = 0;
                }
            } else {
                if (!active_ && uxQueueMessagesWaiting(queue_) == 0) break;
                // Avoid a flush for every momentary queue lull. The timed
                // receive bounds flush latency when traffic is sparse.
                if (sinceFlush != 0) {
                    file_.flush();
                    sinceFlush = 0;
                }
            }
        }
        file_.flush();
        task_=nullptr;
        vTaskDelete(nullptr);
    }
    void event(const char *s) { append(FUSION_LOG_EVENT, micros(), s, strlen(s)); }
public:
    bool recoverLatest(FS &storage) {
        storage_ = &storage; unsigned latest = 0; bool found = false; char path[32] = {};
        File root = storage.open("/"); if (!root) return false; File entry;
        while ((entry = root.openNextFile())) {
            if (!entry.isDirectory()) { unsigned n=0; char suffix=0; const char *name=entry.name();
                if ((sscanf(name,"/fusion-%u.bin%c",&n,&suffix)==1 || sscanf(name,"fusion-%u.bin%c",&n,&suffix)==1) && (!found || n>latest)) {
                    char candidate[32] = {};
                    if (name[0] == '/') strncpy(candidate, name, sizeof(candidate)-1);
                    else snprintf(candidate, sizeof(candidate), "/%s", name);
                    File check = storage.open(candidate, FILE_READ);
                    if (check) {
                        check.close(); latest=n; found=true;
                        strncpy(path,candidate,sizeof(path)-1);
                    }
                } }
            entry.close();
        }
        root.close(); if (found) strncpy(fileName_,path,sizeof(fileName_)-1); return found;
    }
    bool begin(uint8_t cs, const char *prefix="/fusion-") {
        storage_ = &SD;
        if (!SD.begin(cs)) return false;
        return beginMounted(prefix);
    }
    bool begin(FS &storage, const char *prefix="/fusion-") {
        storage_ = &storage;
        return beginMounted(prefix);
    }
private:
    bool beginMounted(const char *prefix) {
        // Enumerate the card rather than relying on a retained counter.  The
        // files themselves are the durable sequence state across reboots.
        unsigned next = 0;
        File root = storage_->open("/");
        if (root) {
            File entry;
            while ((entry = root.openNextFile())) {
                if (!entry.isDirectory()) {
                    const char *name = entry.name();
                    unsigned number = 0;
                    char suffix = 0;
                    if (sscanf(name, "/fusion-%u.bin%c", &number, &suffix) == 1 ||
                        sscanf(name, "fusion-%u.bin%c", &number, &suffix) == 1) {
                        if (number < 10000 && number >= next) next = number + 1;
                    }
                }
                entry.close();
            }
            root.close();
        }
        if (next >= 10000) return false;
        char n[32]; snprintf(n,sizeof(n),"%s%04u.bin",prefix,next);
        if (storage_->exists(n)) return false;
                strncpy(fileName_, n, sizeof(fileName_) - 1);
        fileName_[sizeof(fileName_) - 1] = '\0';
        { file_=storage_->open(n,FILE_WRITE); if (!file_) return false;
                sequence_=0;
                queue_=xQueueCreate(QueueDepth,sizeof(QueueItem));
                if (!queue_) { file_.close(); return false; }
                active_=true;
                if (xTaskCreatePinnedToCore(taskEntry,"fusion-log",8192,this,1,&task_,1)!=pdPASS) {
                    active_=false; vQueueDelete(queue_); queue_=nullptr; file_.close(); return false;
                }
                event("START"); return true; }
    }
public:
    void stop() {
        if (!active_) return; event("STOP"); active_=false;
        uint32_t deadline=millis()+2000;
        while (task_ && (int32_t)(deadline-millis())>0) delay(5);
        if (task_) { vTaskDelete(task_); task_=nullptr; }
        if (file_) { file_.flush(); file_.close(); }
        if (queue_) { vQueueDelete(queue_); queue_=nullptr; }
    }
    bool active() const { return active_; }
    uint32_t dropped() const { return dropped_; }
    uint32_t written() const { return written_; }
    uint32_t writeErrors() const { return writeErrors_; }
    const char *fileName() const { return fileName_; }
    size_t fileSize() const {
        if (!storage_ || !fileName_[0]) return 0;
        File f = storage_->open(fileName_, FILE_READ);
        if (!f) return 0;
        size_t n = f.size();
        f.close();
        return n;
    }
    File openRead() const {
        if (!storage_ || !fileName_[0]) return File();
        return storage_->open(fileName_, FILE_READ);
    }
    size_t dumpTo(Stream &out) const {
        if (!storage_ || !fileName_[0]) return 0;
        File f = storage_->open(fileName_, FILE_READ);
        if (!f) return 0;
        uint8_t buf[512];
        size_t total = 0;
        while (f.available()) {
            size_t n = f.read(buf, sizeof(buf));
            if (!n) break;
            size_t sent = 0;
            while (sent < n) {
                size_t written = out.write(buf + sent, n - sent);
                if (written) sent += written;
                else { delay(1); yield(); }
            }
            total += n;
        }
        f.close();
        return total;
    }
    bool append(FusionLogType type,uint64_t t,const void *p,uint32_t n) {
        if (!active_ || n>MaxPayload) { if (active_) dropped_++; return false; }
        QueueItem item; item.header.timestampUs=t;
        item.header.type=type; item.header.payloadLength=n;
        if (n) memcpy(item.payload,p,n);
        portENTER_CRITICAL(&producerMux_);
        item.header.sequence=sequence_++;
        const bool queued = xQueueSend(queue_,&item,0)==pdTRUE;
        portEXIT_CRITICAL(&producerMux_);
        if (!queued) { dropped_++; return false; }
        return true;
    }
    bool appendImu(uint64_t t, float gx, float gy, float gz,
                   float ax, float ay, float az, bool valid=true) {
        FusionImuRecord r{t,gx,gy,gz,ax,ay,az,(uint8_t)(valid ? 1 : 0)};
        return append(FUSION_LOG_IMU,t,&r,sizeof(r));
    }
    bool appendGps(uint32_t t, int32_t latE7, int32_t lonE7,
                   int32_t altitudeMm, uint32_t speedMmps, int32_t headingE5,
                   bool valid) {
        FusionGpsRecord r{t,latE7,lonE7,altitudeMm,speedMmps,headingE5,
                          (uint8_t)(valid ? 1 : 0)};
        return append(FUSION_LOG_GPS,(uint64_t)t*1000ULL,&r,sizeof(r));
    }
    bool appendBaro(uint64_t t, float pressurePa, float altitudeM, bool valid=true) {
        FusionBaroRecord r{t,pressurePa,altitudeM,(uint8_t)(valid ? 1 : 0)};
        return append(FUSION_LOG_BARO,t,&r,sizeof(r));
    }
    bool appendCompass(uint8_t source, uint64_t t, float x, float y, float z,
                       bool valid=true) {
        if (source > 1) return false;
        FusionCompassRecord r{t,x,y,z,(uint8_t)(valid ? 1 : 0)};
        return append(source == 0 ? FUSION_LOG_COMPASS0 : FUSION_LOG_COMPASS1,
                      t,&r,sizeof(r));
    }
    void flush() { /* writer task owns SD I/O; it flushes when the queue drains */ }
};
