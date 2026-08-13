#include <Arduino.h>
#include <esp_heap_caps.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <esp_mac.h>
#include <math.h>
#include <Arduino_CRC32.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <SD.h>
#include <U8g2lib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <jimlib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BME280.h>
#include <ICM_20948.h>
#include <SensorQMI8658.hpp>
#include <SensorQMC6310.hpp>
#include <XPowersLib.h>
#include <espNowMux.h>
#include <reliableStream.h>
#include "AircraftAHRS.h"
#include "FusionSessionLog.h"
#include "SharedUbloxGPS.h"
#include "HardwareAbstraction.h"

HalHardwareProfile HARDWARE = makeGeekS3Profile();
constexpr HalImuRequest REQUESTED_IMU = {50.0f, 20.0f};
HalImuConfiguration actualImu = {0, 0, 0.02f, 0, false};
struct IcmRateProfile {
  uint16_t gyroDivider;
  uint16_t accelDivider;
  float gyroOdrHz;
  float accelOdrHz;
  float ahrsIntegrationDtSec;
  ICM_20948_GYRO_CONFIG_1_DLPCFG_e gyroDlpf;
  ICM_20948_ACCEL_CONFIG_DLPCFG_e accelDlpf;
};

constexpr IcmRateProfile makeIcmRateProfile(uint16_t gyroDivider,
                                            uint16_t accelDivider,
                                            ICM_20948_GYRO_CONFIG_1_DLPCFG_e gyroDlpf,
                                            ICM_20948_ACCEL_CONFIG_DLPCFG_e accelDlpf) {
  return {gyroDivider, accelDivider,
          1100.0f / (1.0f + gyroDivider),
          1125.0f / (1.0f + accelDivider),
          (1.0f + gyroDivider) / 1100.0f,
          gyroDlpf, accelDlpf};
}

// Change this one profile when testing another ICM-20948 hardware rate.  The
// AHRS integration interval follows the gyro ODR so a slower sensor stream is
// not accidentally integrated with the old 50 Hz weight. The DLPF settings
// travel with the rate profile so the filter remains appropriate for its
// output Nyquist frequency.
constexpr IcmRateProfile ICM_RATE_PROFILE = makeIcmRateProfile(
    21, 21, gyr_d23bw9_n35bw9, acc_d23bw9_n34bw4);
constexpr uint32_t COMPASS1_OUTPUT_PERIOD_US =
    static_cast<uint32_t>(1.0e6f / 50.0f);

// This is the LCD configuration from the last known-good pre-status-page
// firmware.  Keep it unchanged until the panel is stable again.
Adafruit_ST7789 display(HARDWARE.lcdCs, HARDWARE.lcdDc, HARDWARE.lcdRst);
SPIClass sdSpi(HSPI);
ReliableStreamESPNow espnow("GEEK", true /* alwaysBroadcast */);
HardwareSerial gpsSerial(1);
Adafruit_LSM9DS1 imu;
ICM_20948_I2C icm20948;
SensorQMI8658 qmi8658;
SensorQMI8658 qmi8658Secondary;
SensorQMC6310 qmc6310;
XPowersAXP2101 pmu;
IMUdata qmiAccel, qmiGyro;
U8G2_SH1106_128X64_NONAME_1_HW_I2C tbeamDisplay(U8G2_R0, U8X8_PIN_NONE);
uint8_t tbeamDisplayAddress = 0;
Adafruit_BMP280 baro;
Adafruit_BMP3XX bmp3;
Adafruit_BME280 bme;
bool bmeOk = false;
enum class BaroKind { None, BMP280, BMP388, BMP390, BME280 };
BaroKind baroKind = BaroKind::None;
float latestBaroPressurePa = NAN, latestBaroAltitudeM = NAN;
uint32_t baroReadyAfterMs = 0;
enum class ImuKind { None, LSM9DS1, ICM20948, QMI8658 };
ImuKind imuKind = ImuKind::None;
bool secondaryImuOk = false;
constexpr uint8_t SECONDARY_LSM6DSL = 0x6A;
constexpr uint8_t BERRY_BARO_ADDRESS = 0x77;
constexpr uint32_t BARO_OUTPUT_PERIOD_US = 40000; // 25 Hz

static bool i2cReadRegister(uint8_t address, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(address); Wire.write(reg);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(address, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

static const char *baroKindName() {
  switch (baroKind) {
  case BaroKind::BMP280: return "BMP280";
  case BaroKind::BMP388: return "BMP388";
  case BaroKind::BMP390: return "BMP390";
  case BaroKind::BME280: return "BME280";
  default: return "ABSENT";
  }
}
static bool secondaryLsm6Read(uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(SECONDARY_LSM6DSL); Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(SECONDARY_LSM6DSL, (uint8_t)n) != n) return false;
  for (size_t i=0; i<n; ++i) buf[i] = Wire.read();
  return true;
}
static bool secondaryLsm6Write(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(SECONDARY_LSM6DSL); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
static bool readSecondaryLsm6(float &gx, float &gy, float &gz,
                              float &ax, float &ay, float &az, float &temp) {
  uint8_t b[12], t[2];
  if (!secondaryLsm6Read(0x22, b, sizeof(b))) return false;
  auto s16 = [](uint8_t lo, uint8_t hi) { return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8)); };
  gx = s16(b[0],b[1]) * 0.00875f; gy = s16(b[2],b[3]) * 0.00875f; gz = s16(b[4],b[5]) * 0.00875f;
  constexpr float accelMps2PerLsb = 0.000061f * 9.80665f;
  ax = s16(b[6],b[7]) * accelMps2PerLsb; ay = s16(b[8],b[9]) * accelMps2PerLsb;
  az = s16(b[10],b[11]) * accelMps2PerLsb;
  temp = secondaryLsm6Read(0x20, t, 2) ? 25.0f + s16(t[0],t[1]) / 256.0f : NAN;
  return true;
}

struct GyroDriftLogger {
  bool active = false;
  uint32_t startedMs = 0;
  uint32_t windowStartedMs = 0;
  uint32_t samples[2] = {0, 0};
  double sum[2][3] = {{0, 0, 0}, {0, 0, 0}};

  void resetWindow(uint32_t now) {
    windowStartedMs = now; samples[0] = samples[1] = 0;
    memset(sum, 0, sizeof(sum));
  }
  void start(uint32_t now) {
    active = true; startedMs = now; resetWindow(now);
    Serial.println("GYRO_DRIFT STARTED interval_s=10 units=dps");
    Serial.println("GYRO_DRIFT_COLUMNS elapsed_s,imu,samples,gyro_x_dps,gyro_y_dps,gyro_z_dps,temperature_c");
  }
  void stop(uint32_t now) {
    active = false;
    Serial.printf("GYRO_DRIFT STOPPED elapsed_s=%.3f\n", (now-startedMs)*0.001f);
  }
  void add(uint8_t which, float x, float y, float z) {
    if (!active || which > 1) return;
    sum[which][0] += x; sum[which][1] += y; sum[which][2] += z; ++samples[which];
  }
  void update(uint32_t now, float primaryTemp, float secondaryTemp) {
    if (!active || (uint32_t)(now-windowStartedMs) < 10000) return;
    for (uint8_t i = 0; i < 2; ++i) {
      float temp = i == 0 ? primaryTemp : secondaryTemp;
      if (samples[i]) Serial.printf("GYRO_DRIFT %.3f,%s,%lu,%.6f,%.6f,%.6f,%.3f\n",
        (now-startedMs)*0.001f, i == 0 ? "primary" : "secondary",
        (unsigned long)samples[i], sum[i][0]/samples[i], sum[i][1]/samples[i],
        sum[i][2]/samples[i], temp);
      else Serial.printf("GYRO_DRIFT %.3f,%s,0,nan,nan,nan,%.3f\n",
        (now-startedMs)*0.001f, i == 0 ? "primary" : "secondary", temp);
    }
    resetWindow(now);
  }
};
GyroDriftLogger gyroDrift;
bool displayOk = false, sdOk = false, imuOk = false, baroOk = false, qmcOk = false;
bool qmcPOk = false;
bool gpsOk = false;
uint8_t gpsFixQuality = 0;
uint8_t gpsSatellites = 0;
float gpsPdop = 99.0f;
uint32_t lastG5PacketMs = 0;
uint32_t discardedG5NmeaPackets = 0;
SemaphoreHandle_t displayStatusMutex = nullptr;
TaskHandle_t displayTaskHandle = nullptr;
HalDisplayStatus latestDisplayStatus{};
SharedUbloxGPS sharedGps;
static AircraftAHRS::Config makeAhrsConfigFromHal() {
  AircraftAHRS::Config config;
  const HalImuCalibration &calibration = HARDWARE.calibration.imu[0];
  // Raw gyro bias and polarity belong to the sensor frame and are applied
  // immediately before gyroAxisRemap at the sample call site.  AircraftAHRS
  // therefore receives already-remapped body rates with neutral raw-axis
  // calibration here.
  if (calibration.applyAccelBias) {
    config.accelBiasXMps2 = calibration.accelBiasMps2[0];
    config.accelBiasYMps2 = calibration.accelBiasMps2[1];
    config.accelBiasZMps2 = calibration.accelBiasMps2[2];
  }
  config.gyroIntegrationDtSec = ICM_RATE_PROFILE.ahrsIntegrationDtSec;
  return config;
}
AircraftAHRS ahrs(makeAhrsConfigFromHal());
float sensorFrameRotation[3][3];
ReliableStreamESPNow g5("G5", true /* incoming benchmark traffic */);
FusionSessionLog sessionLog;

static bool isG5NmeaPacket(const uint8_t *data, size_t length) {
  static const uint8_t marker[] = {'N','M','E','A','='};
  if (!data || length < sizeof(marker)) return false;
  for (size_t i = 0; i + sizeof(marker) <= length; ++i)
    if (memcmp(data + i, marker, sizeof(marker)) == 0) return true;
  return false;
}

static void stopSessionWithSummary() {
  char summary[192];
  snprintf(summary, sizeof(summary),
           "STOP_SUMMARY written=%lu dropped=%lu errors=%lu g5_nmea_discarded=%lu",
           (unsigned long)sessionLog.written(),
           (unsigned long)sessionLog.dropped(),
           (unsigned long)sessionLog.writeErrors(),
           (unsigned long)discardedG5NmeaPackets);
  // This event is queued before stop() drains and closes the file.  It remains
  // available in battery/flight logs when serial output is unavailable.
  sessionLog.append(FUSION_LOG_EVENT, micros(), summary, strlen(summary));
  sessionLog.stop();
  if (displayStatusMutex) {
    if (xSemaphoreTake(displayStatusMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      latestDisplayStatus.logging = false;
      xSemaphoreGive(displayStatusMutex);
    }
  }
}
bool lastLogButton = true;
uint32_t logButtonChangedMs = 0;
bool bootLogSession = false;
uint32_t bootLogDeadlineMs = 0;
bool startSessionLog();

HalBoardKind detectBoard() {
  // The AXP2101 at 0x34 on the dedicated GPIO42/41 bus is a strong,
  // non-destructive T-Beam Supreme signature. GEEK has no device there.
  Wire1.begin(42, 41);
  Wire1.setTimeOut(20);
  Wire1.beginTransmission(AXP2101_SLAVE_ADDRESS);
  if (Wire1.endTransmission() == 0) return HalBoardKind::TBeamSupreme;
  return HalBoardKind::GeekS3;
}

void setupDisplay() {
  if (HARDWARE.display == HalDisplayKind::None) return;
  if (HARDWARE.display == HalDisplayKind::TBeamSupreme_SH1106) {
  Wire.begin(HARDWARE.i2cSda, HARDWARE.i2cScl);
  Wire.setTimeOut(20);
  // Both addresses ACK on this board. Test 0x3D first: it is the alternate
  // SH1106 address and selecting 0x3C produced no visible pixels.
  for (uint8_t address : {uint8_t(0x3D), uint8_t(0x3C)}) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) { tbeamDisplayAddress = address; break; }
  }
  Serial.printf("SH1106 I2C SDA=17 SCL=18 address=%s0x%02X\n",
                tbeamDisplayAddress ? "" : "not-found ", tbeamDisplayAddress);
  if (!tbeamDisplayAddress) { displayOk = false; return; }
  // U8g2 takes the 8-bit form of the I2C address.
  tbeamDisplay.setI2CAddress(tbeamDisplayAddress << 1);
  tbeamDisplay.setBusClock(100000);
  displayOk = tbeamDisplay.begin() == 1;
  if (displayOk) {
    tbeamDisplay.setFont(u8g2_font_6x10_tf);
    tbeamDisplay.clearBuffer();
    tbeamDisplay.drawBox(0, 0, 128, 12);
    tbeamDisplay.setDrawColor(0); tbeamDisplay.drawStr(2, 10, "T-BEAM OLED OK");
    tbeamDisplay.setDrawColor(1); tbeamDisplay.drawStr(0, 28, "SH1106 128x64");
    tbeamDisplay.drawStr(0, 40, "boot diagnostic");
    tbeamDisplay.drawStr(0, 52, "I2C 17/18");
    tbeamDisplay.sendBuffer();
  } else {
    Serial.println("SH1106 U8g2 begin failed");
  }
  return;
  }
  pinMode(HARDWARE.lcdBacklight, OUTPUT); digitalWrite(HARDWARE.lcdBacklight, HIGH);
  display.init(135, 240); display.setRotation(1); display.fillScreen(ST77XX_BLACK);
  display.setTextColor(ST77XX_WHITE, ST77XX_BLACK); display.setTextSize(2);
  display.setCursor(4, 4); display.println("ESP32-S3 Geek");
  display.println("hardware test"); displayOk = true;
}

// HAL display entry point. The application owns the task and publishes a
// board-neutral status snapshot; this function owns only board-specific
// rendering and is called by the low-priority display task.
void halUpdateDisplay(const HalDisplayStatus &status) {
  if (!displayOk) return;
  uint32_t nowMs = status.uptimeSeconds * 1000UL;
  if (HARDWARE.display == HalDisplayKind::TBeamSupreme_SH1106) {
  char line[32];
  bool more;
  tbeamDisplay.firstPage();
  do {
    tbeamDisplay.setFont(u8g2_font_6x10_tf);
    auto health = [&](int x, int y, const char *label, bool good) {
      if (good) { tbeamDisplay.drawBox(x, y - 9, 38, 11); tbeamDisplay.setDrawColor(0); }
      tbeamDisplay.drawStr(x + 2, y, label); tbeamDisplay.setDrawColor(1);
      taskYIELD();
    };
    health(0, 10, "GPS", status.gps); health(44, 10, "IMU", status.imu);
    health(88, 10, "SD", status.sd);
    health(0, 22, "BARO", status.baro); health(44, 22, "MAG", status.compass);
    health(88, 22, "LOG", status.logging);
    snprintf(line, sizeof(line), "R%5.1f P%5.1f", status.rollDeg, status.pitchDeg);
    tbeamDisplay.drawStr(0, 34, line); taskYIELD();
    snprintf(line, sizeof(line), "H%5.1f V%4.1f", status.headingDeg, status.groundSpeedMps);
    tbeamDisplay.drawStr(0, 46, line); taskYIELD();
    snprintf(line, sizeof(line), "LOG %lus", (unsigned long)status.loggingElapsedSeconds);
    tbeamDisplay.drawStr(0, 58, line);
    more = tbeamDisplay.nextPage();
    // Do not dispatch the next page immediately.  Leaving a real bus-idle
    // window prevents a series of page writes from monopolizing Wire even
    // though this task has low FreeRTOS priority.
    if (more) vTaskDelay(pdMS_TO_TICKS(25));
  } while (more);
  return;
  }

  // The panel is 240x135 in rotation 1.  Redraw fixed-height rows instead of
  // clearing the whole panel; this avoids the visible 1 Hz flash and also
  // prevents variable-width status text from leaving stale pixels behind.
  constexpr int16_t left = 3;
  constexpr int16_t rowHeight = 27;
  auto beginRow = [&](int16_t row, uint8_t size) {
    display.fillRect(0, row * rowHeight, 240, rowHeight, ST77XX_BLACK);
    display.setTextSize(size);
    display.setCursor(left, row * rowHeight + 3);
  };

  beginRow(0, 3);
  display.setTextColor(status.gps ? ST77XX_GREEN : ST77XX_RED, ST77XX_BLACK);
  display.printf("G%u S%u", status.gpsFixQuality, status.gpsSatellites);

  beginRow(1, 2);
  display.setTextColor(status.gpsPdop < 2.0f ? ST77XX_GREEN : status.gpsPdop < 4.0f ? ST77XX_YELLOW : ST77XX_RED, ST77XX_BLACK);
  display.printf("PD%.1f ", status.gpsPdop);
  display.setTextColor(status.compass ? ST77XX_GREEN : ST77XX_RED); display.print("CP ");
  display.setTextColor(status.imu ? ST77XX_GREEN : ST77XX_RED); display.print("IM ");
  display.setTextColor(status.sd ? ST77XX_GREEN : ST77XX_RED); display.print("SD ");
  display.setTextColor(status.g5 ? ST77XX_GREEN : ST77XX_RED); display.print("G5");

  beginRow(2, 3);
  display.setTextColor(status.logging ? ST77XX_YELLOW : ST77XX_WHITE, ST77XX_BLACK);
  if (status.logging)
    display.printf("LOG %lus", (unsigned long)status.loggingElapsedSeconds);
  else
    display.print("---");
  display.setTextColor(ST77XX_WHITE, ST77XX_BLACK); display.printf(" D%lu", (unsigned long)status.droppedLogRecords);

  beginRow(3, 3);
  display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.printf("%lus", (unsigned long)status.uptimeSeconds);

  beginRow(4, 3);
  display.printf("P %.1f", status.pressureHpa);
}

void setupStorage() {
  if (!HARDWARE.hasSd) return;
  uint8_t mac[6]{};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  sessionLog.configureIdentity('G', mac);
  sdSpi.begin(HARDWARE.sdSck, HARDWARE.sdMiso, HARDWARE.sdMosi);
  sdOk = SD.begin(HARDWARE.sdCs, sdSpi, 20000000);
  if (sdOk) sessionLog.recoverLatest(SD);
  Serial.printf("microSD SPI SCK=%d MISO=%d MOSI=%d CS=%d: %s\n",
                HARDWARE.sdSck, HARDWARE.sdMiso, HARDWARE.sdMosi, HARDWARE.sdCs, sdOk ? "OK" : "not detected");
}

bool clearFusionLogs() {
  File root = SD.open("/");
  if (!root) return false;
  bool ok = true; File entry;
  while ((entry = root.openNextFile())) {
    if (!entry.isDirectory()) {
      String name = entry.name();
      if (FusionSessionLog::isLogFileName(name.c_str())) {
        String path = name.startsWith("/") ? name : "/" + name;
        if (!SD.remove(path)) ok = false;
      }
    }
    entry.close();
  }
  root.close(); return ok;
}

void setupG5Logging() {
  std::string ignored;
  g5.read(ignored); // Register the ReliableStream receive callback.
  defaultEspNowMux.registerReadCallback("G5",
    [](const uint8_t *mac, const uint8_t *data, int len) {
      if (sessionLog.active() && len >= 0 && len <= 506) {
        uint8_t raw[512]; memcpy(raw, mac, 6);
        if (len) memcpy(raw + 6, data, len);
        if (isG5NmeaPacket(raw, len + 6)) { ++discardedG5NmeaPackets; return; }
        sessionLog.append(FUSION_LOG_G5_RAW_ESPNOW, micros(), raw, len + 6);
      }
    });
}

void updateLoggingButton() {
  if (!HARDWARE.hasLogButton) return;
  bool pressed = digitalRead(HARDWARE.logButton) == LOW;
  if (pressed != lastLogButton && millis() - logButtonChangedMs > 40) {
    logButtonChangedMs = millis(); lastLogButton = pressed;
    if (pressed) {
      if (sessionLog.active()) {
        stopSessionWithSummary();
        bootLogSession = false;
        Serial.printf("SESSION_LOG STOPPED written=%lu dropped=%lu errors=%lu g5_nmea_discarded=%lu\n",
                      (unsigned long)sessionLog.written(),
                      (unsigned long)sessionLog.dropped(),
                      (unsigned long)sessionLog.writeErrors(),
                      (unsigned long)discardedG5NmeaPackets);
      } else if ((discardedG5NmeaPackets = 0, startSessionLog())) {
        bootLogSession = false;
        Serial.println("SESSION_LOG STARTED");
      } else {
        Serial.println("SESSION_LOG START FAILED");
      }
    }
  }
}

void displayUpdateTask(void *) {
  HalDisplayStatus status{};
  for (;;) {
    if (displayStatusMutex &&
        xSemaphoreTake(displayStatusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      status = latestDisplayStatus;
      xSemaphoreGive(displayStatusMutex);
      halUpdateDisplay(status);
    }
    // Priority 0 keeps display transfers below the sensor/AHRS loop.  The
    // short delay also prevents a failed/unplugged panel from being retried
    // continuously at the expense of the rest of the system.
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void startDisplayUpdateTask() {
  if (!displayOk || HARDWARE.display == HalDisplayKind::None) return;
  displayStatusMutex = xSemaphoreCreateMutex();
  if (!displayStatusMutex) return;
  xTaskCreatePinnedToCore(displayUpdateTask, "display", 4096, nullptr, 0,
                          &displayTaskHandle, 1);
}

void publishDisplayStatus(const HalDisplayStatus &status) {
  if (!displayStatusMutex) return;
  if (xSemaphoreTake(displayStatusMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    latestDisplayStatus = status;
    xSemaphoreGive(displayStatusMutex);
  }
}

void updateBootLogging() {}

bool startSessionLog() {
  if (!sdOk || !sessionLog.begin(SD)) return false;
  // Publish the logging state; the low-priority display task continues to
  // update the panel while the sensor and SD writer tasks run.
  if (displayStatusMutex) {
    if (xSemaphoreTake(displayStatusMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      latestDisplayStatus.logging = true;
      xSemaphoreGive(displayStatusMutex);
    }
  }
  return true;
}

bool readSerialLine(String &line, uint32_t timeoutMs) {
  line = ""; uint32_t deadline = millis() + timeoutMs;
  while ((int32_t)(deadline - millis()) > 0) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n' || c == '\r') { line.trim(); return line.length() != 0; }
      if (c >= 32 && c <= 126 && line.length() < 40) line += c;
    }
    delay(1);
  }
  return false;
}

void listFusionLogs() {
  if (!sdOk) { Serial.println("LOG_ERROR SD_ABSENT"); return; }
  File root = SD.open("/");
  if (!root) { Serial.println("LOG_ERROR LIST_OPEN"); return; }
  uint32_t count = 0;
  Serial.println("LOG_LIST_BEGIN");
  File entry;
  while ((entry = root.openNextFile())) {
    const char *name = entry.name();
    if (!entry.isDirectory() && name && FusionSessionLog::isLogFileName(name)) {
      Serial.printf("LOG_LIST_FILE name=%s size=%lu\n", name, (unsigned long)entry.size());
      ++count;
    }
    entry.close();
  }
  root.close();
  Serial.printf("LOG_LIST_END count=%lu\n", (unsigned long)count);
}

void dumpChunked(uint32_t startSeq = 0, const char *requestedFile = nullptr) {
  // Host-driven half-duplex plus CRC lets us use the SD/USB-friendly 4 KiB
  // payload size without returning to the unreliable unacknowledged stream.
  const uint32_t chunkSize = 4096;
  if (requestedFile && !sessionLog.selectFile(requestedFile)) {
    Serial.printf("LOG_ERROR FILE name=%s\n", requestedFile); return;
  }
  File f = sessionLog.openRead();
  if (!f) {
    Serial.printf("LOG_ERROR OPEN name=%s size_probe=%lu\n", sessionLog.fileName(),
                  (unsigned long)sessionLog.fileSize());
    return;
  }
  size_t totalSize = f.size();
  Serial.printf("LOG_FILE name=%s size=%lu start=%lu\n", sessionLog.fileName(),
                (unsigned long)totalSize, (unsigned long)startSeq);
  if (!f.seek((size_t)startSeq * chunkSize)) { f.close(); Serial.println("LOG_ERROR SEEK"); return; }
  Arduino_CRC32 crc;
  Serial.printf("LOG_CHUNK_BEGIN %lu %lu\n", (unsigned long)totalSize, (unsigned long)chunkSize);
  Serial.flush();
  uint8_t buf[chunkSize]; uint32_t seq = startSeq;
  while (f.available()) {
    size_t len = f.read(buf, chunkSize);
    if (!len) { size_t pos = f.position(); f.close(); Serial.printf("LOG_ERROR SD_READ seq=%lu pos=%lu\n", (unsigned long)seq, (unsigned long)pos); return; }
    String command;
    uint32_t deadline = millis() + 30000;
    Serial.printf("LOG_WAIT %lu\n", (unsigned long)seq); Serial.flush();
    while ((int32_t)(deadline - millis()) > 0) {
      if (!readSerialLine(command, 1000)) continue;
      Serial.printf("LOG_RX %s\n", command.c_str()); Serial.flush();
      if (command == (String("GET ") + seq)) break;
    }
    if (command != (String("GET ") + seq)) {
      f.close(); Serial.printf("LOG_ERROR GET_TIMEOUT seq=%lu\n", (unsigned long)seq); return;
    }
    Serial.printf("LOG_GET_OK %lu\n", (unsigned long)seq); Serial.flush();
    uint32_t sum = crc.calc(buf, len);
    Serial.printf("LOG_CHUNK %lu %u %08lX\n", (unsigned long)seq, (unsigned)len, (unsigned long)sum);
    size_t sent = 0;
    while (sent < len) { size_t n = Serial.write(buf + sent, len - sent); if (n) sent += n; else { delay(1); yield(); } }
    Serial.flush();
    seq++;
  }
  f.close(); Serial.printf("LOG_CHUNK_END %lu\n", (unsigned long)seq); Serial.flush();
}

void handleSerialCommands() {
  auto scanI2c = []() {
    Serial.printf("I2C_SCAN_BEGIN SDA=%d SCL=%d\n", HARDWARE.i2cSda, HARDWARE.i2cScl);
    uint8_t found = 0;
    for (uint8_t address = 1; address < 0x78; ++address) {
      Wire.beginTransmission(address);
      uint8_t error = Wire.endTransmission();
      if (error == 0) {
        Serial.printf("I2C_DEVICE address=0x%02X%s\n", address,
                      address == 0x0D ? " QMC5883L_EXPECTED" : "");
        ++found;
      }
      delay(1);
    }
    Serial.printf("I2C_SCAN_END count=%u QMC=%s\n", found,
                  qmcOk ? "OK" : "ABSENT");
  };
  static String command;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      command.trim();
      if (command.equalsIgnoreCase("START_LOG")) {
        if (sessionLog.active()) Serial.println("LOG_ERROR ACTIVE");
        else if ((discardedG5NmeaPackets = 0, startSessionLog())) Serial.println("SESSION_LOG STARTED");
        else Serial.println("SESSION_LOG START FAILED");
      } else if (command.equalsIgnoreCase("STOP_LOG")) {
        if (!sessionLog.active()) Serial.println("LOG_ERROR INACTIVE");
        else { stopSessionWithSummary(); Serial.printf("SESSION_LOG STOPPED written=%lu dropped=%lu errors=%lu g5_nmea_discarded=%lu\n", (unsigned long)sessionLog.written(), (unsigned long)sessionLog.dropped(), (unsigned long)sessionLog.writeErrors(), (unsigned long)discardedG5NmeaPackets); }
      } else if (command.equalsIgnoreCase("FORMAT")) {
        if (sessionLog.active()) Serial.println("LOG_ERROR ACTIVE");
        else if (sdOk && clearFusionLogs()) Serial.println("SD_FORMAT OK (logs cleared)");
        else Serial.println("SD_FORMAT FAIL");
      } else if (command.equalsIgnoreCase("LIST")) {
        listFusionLogs();
      } else if (command.equalsIgnoreCase("DUMP") || command.startsWith("DUMP ")) {
        if (sessionLog.active()) {
          Serial.println("LOG_ERROR ACTIVE");
        } else {
          uint32_t startSeq = 0;
          String requestedFile;
          if (command.startsWith("DUMP ")) {
            String args = command.substring(5); args.trim();
            int space = args.indexOf(' ');
            String first = space < 0 ? args : args.substring(0, space);
            bool numeric = first.length() > 0;
            for (size_t i = 0; i < first.length(); ++i) numeric = numeric && isdigit(first[i]);
            if (numeric) startSeq = (uint32_t)first.toInt();
            else {
              requestedFile = first;
              if (space >= 0) startSeq = (uint32_t)args.substring(space + 1).toInt();
            }
          }
          dumpChunked(startSeq, requestedFile.length() ? requestedFile.c_str() : nullptr);
        }
      } else if (command.equalsIgnoreCase("SCAN")) {
        scanI2c();
      } else if (command.equalsIgnoreCase("GYRO_DRIFT_START")) {
        if (sessionLog.active()) Serial.println("GYRO_DRIFT ERROR session_log_active");
        else if (gyroDrift.active) Serial.println("GYRO_DRIFT ALREADY_STARTED");
        else gyroDrift.start(millis());
      } else if (command.equalsIgnoreCase("GYRO_DRIFT_STOP")) {
        if (gyroDrift.active) gyroDrift.stop(millis());
        else Serial.println("GYRO_DRIFT ALREADY_STOPPED");
      } else if (command.equalsIgnoreCase("GYRO_DRIFT_STATUS")) {
        Serial.printf("GYRO_DRIFT STATUS active=%s secondary=%s elapsed_s=%.3f\n",
                      gyroDrift.active ? "yes" : "no", secondaryImuOk ? "yes" : "no",
                      gyroDrift.active ? (millis()-gyroDrift.startedMs)*0.001f : 0.0f);
      } else if (command.length()) {
        Serial.println("LOG_ERROR UNKNOWN_COMMAND");
      }
      command = "";
    } else if (c >= 32 && c <= 126 && command.length() < 32) {
      command += c;
    }
  }
}

void setupBerryIMU() {
  // BerryIMUv3 normally uses I2C: LSM9DS1 XG=0x6B, magnetometer=0x1C,
  // BMP280=0x76 (some modules strap the barometer to 0x77).
  // Waveshare's Geek I2C example uses GPIO16/17. Explicit pins are required:
  // the ESP32-S3 defaults can overlap the LCD's DC/RESET pins (8/9).
  Wire.begin(HARDWARE.i2cSda, HARDWARE.i2cScl);
  Wire.setTimeOut(20); // Missing Qwiic hardware must never stall the test.
  uint8_t secondaryWhoAmI = 0;
  // The externally powered BerryIMU can become ready slightly after the ESP32
  // I2C controller. Do not permanently lose IMU1 because the first transaction
  // after boot happened during that interval.
  for (uint8_t attempt = 0; attempt < 20 && !secondaryImuOk; ++attempt) {
    secondaryImuOk = secondaryLsm6Read(0x0F, &secondaryWhoAmI, 1) &&
                     secondaryWhoAmI == 0x6A;
    if (!secondaryImuOk) delay(25);
  }
  if (secondaryImuOk) {
    secondaryLsm6Write(0x12, 0x44); // BDU and register-address auto-increment
    secondaryLsm6Write(0x11, 0x50); // gyro: 208 Hz, 245 dps
    secondaryLsm6Write(0x10, 0x50); // accel: 208 Hz, 2 g
  }
  Serial.printf("Secondary LSM6DSL address=0x6A: %s (WHO_AM_I=0x%02X)\n", secondaryImuOk ? "OK" : "not detected", secondaryWhoAmI);
  if (HARDWARE.kind == HalBoardKind::TBeamSupreme) {
  Serial.println("T-Beam I2C scan SDA=17 SCL=18");
  // Include 0x7C: the LilyGO QMC63xx example defines QMC6309 there.
  for (uint8_t address = 0x08; address < 0x80; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) Serial.printf("I2C device address=0x%02X\n", address);
  }
  Serial.println("I2C probe candidates: BME280=0x76,0x77; QMC6310U=0x1C QMC6310N=0x3C QMC6309=0x7C");
  bool qmcN = qmc6310.begin(Wire, QMC6310N_SLAVE_ADDRESS,
                            HARDWARE.i2cSda, HARDWARE.i2cScl);
  qmcOk = qmcN;
  Serial.printf("QMC6310N probe address=0x3C: %s\n", qmcN ? "FOUND" : "not detected");
  if (qmcN) {
    qmc6310.configMagnetometer(OperationMode::CONTINUOUS_MEASUREMENT,
                               MagFullScaleRange::FS_8G, 50.0f,
                               MagOverSampleRatio::OSR_4,
                               MagDownSampleRatio::DSR_1);
  }
  bool pmuOk = pmu.begin(Wire1, AXP2101_SLAVE_ADDRESS, 42, 41);
  Wire1.setTimeOut(20);
  Serial.println("T-Beam PMU I2C scan SDA=42 SCL=41");
  for (uint8_t address = 0x08; address < 0x80; ++address) {
    Wire1.beginTransmission(address);
    if (Wire1.endTransmission() == 0)
      Serial.printf("PMU-bus I2C device address=0x%02X%s\n", address,
                    address == AXP2101_SLAVE_ADDRESS ? " (AXP2101)" : "");
  }
  if (pmuOk) {
    pmu.setALDO1Voltage(3300); pmu.enableALDO1();
    pmu.setALDO2Voltage(3300); pmu.enableALDO2();
    // T-Beam Supreme power table: GNSS is ALDO4 and TF/SD is BLDO1.
    pmu.setALDO4Voltage(3300); pmu.enableALDO4();
    pmu.setBLDO1Voltage(3300); pmu.enableBLDO1();
    Serial.printf("T-Beam power rails: ALDO1=%s %umV ALDO2=%s %umV ALDO4/GNSS=%s %umV BLDO1/SD=%s %umV\n",
                  pmu.isEnableALDO1() ? "ON" : "OFF", pmu.getALDO1Voltage(),
                  pmu.isEnableALDO2() ? "ON" : "OFF", pmu.getALDO2Voltage(),
                  pmu.isEnableALDO4() ? "ON" : "OFF", pmu.getALDO4Voltage(),
                  pmu.isEnableBLDO1() ? "ON" : "OFF", pmu.getBLDO1Voltage());
  }
  // Give the GNSS rail and receiver time to complete cold startup before
  // probing UART. PPS is optional timing output and is not needed for UBX.
  delay(500);
#if defined(TBEAM_NO_QMI)
  imuOk = false;
  Serial.println("T-Beam QMI8658: DISABLED FOR SD TEST");
#else
  // The T-Beam routes QMI8658 and microSD over the same physical SPI wires.
  // Use the already-initialized SD controller rather than mapping a second
  // ESP32 SPI peripheral onto those pins.  The devices have separate CS
  // lines (QMI=34, SD=47); SensorLib performs the QMI transactions while
  // the logger owns SD transactions.
  imuOk = qmi8658.begin(sdSpi, 34, 35, 37, 36);
  if (imuOk) {
    // QMI8658 has no exact 50 Hz pair. Choose the closest supported ODRs;
    // LPF_MODE_3 is the hardware low-pass setting selected by this HAL.
    qmi8658.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,
                                SensorQMI8658::ACC_ODR_62_5Hz,
                                SensorQMI8658::LPF_MODE_3);
    qmi8658.configGyroscope(SensorQMI8658::GYR_RANGE_128DPS,
                            SensorQMI8658::GYR_ODR_56_05Hz,
                            SensorQMI8658::LPF_MODE_3);
    qmi8658.enableAccelerometer(); qmi8658.enableGyroscope();
    actualImu = {62.5f, 56.05f, 1.0f / 56.05f, REQUESTED_IMU.lowPassCutoffHz, true};
    ahrs.setGyroIntegrationDt(actualImu.integrationDtSec);
  }
#endif
  bmeOk = bme.begin(0x77, &Wire);
  baroOk = bmeOk;
  baroKind = bmeOk ? BaroKind::BME280 : BaroKind::None;
  imuKind = imuOk ? ImuKind::QMI8658 : ImuKind::None;
  Serial.printf("T-Beam PMU=%s QMI8658=%s BME280=%s\n", pmuOk ? "OK" : "ABSENT",
                imuOk ? "OK" : "ABSENT", bmeOk ? "0x77" : "ABSENT");
  Serial.printf("HAL IMU requested=%.1fHz actual gyro=%.2fHz accel=%.2fHz dt=%.6fs DLPF=%s\n",
                REQUESTED_IMU.sampleRateHz, actualImu.gyroRateHz,
                actualImu.accelRateHz, actualImu.integrationDtSec,
                actualImu.hardwareLowPassEnabled ? "ON" : "OFF");
  return;
  }
  Wire.beginTransmission(0x0D); // QMC5883L compass on the SEQURE GPS module.
  qmcOk = Wire.endTransmission() == 0;
  Serial.printf("QMC5883L I2C address=0x0D: %s\n", qmcOk ? "OK" : "not detected");
  // Prefer the newer SparkFun ICM-20948, then fall back to the BerryIMUv3
  // LSM9DS1. ADR selects 0x68/0x69 on the ICM board; the library's ad0val
  // argument maps directly to that address bit.
  if (icm20948.begin(Wire, false) == ICM_20948_Stat_Ok) {
    imuKind = ImuKind::ICM20948;
    imuOk = true;
  } else if (icm20948.begin(Wire, true) == ICM_20948_Stat_Ok) {
    imuKind = ImuKind::ICM20948;
    imuOk = true;
  } else {
    imuOk = imu.begin();
    if (imuOk) imuKind = ImuKind::LSM9DS1;
  }
  if (imuKind == ImuKind::ICM20948) {
    // ICM-20948 ODR = 1.1 kHz/(1+gyro_div), 1.125 kHz/(1+accel_div).
    // Keep the hardware rates and the AHRS per-sample integration interval in
    // the single ICM_RATE_PROFILE above.
    ICM_20948_smplrt_t sampleRate{};
    sampleRate.g = ICM_RATE_PROFILE.gyroDivider;
    sampleRate.a = ICM_RATE_PROFILE.accelDivider;
    ICM_20948_Status_e status = icm20948.setSampleRate(
        ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, sampleRate);
    ICM_20948_dlpcfg_t dlpf{};
    dlpf.g = ICM_RATE_PROFILE.gyroDlpf;
    dlpf.a = ICM_RATE_PROFILE.accelDlpf;
    ICM_20948_Status_e dlpfConfigStatus = icm20948.setDLPFcfg(
        ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlpf);
    ICM_20948_Status_e dlpfEnableStatus = icm20948.enableDLPF(
        ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, true);
    Serial.printf("ICM ODR configuration: %s (gyro=%.2fHz, accel=%.2fHz, integration=%.3fs, DLPF=%s)\n",
                  status == ICM_20948_Stat_Ok ? "OK" : "FAILED",
                  ICM_RATE_PROFILE.gyroOdrHz, ICM_RATE_PROFILE.accelOdrHz,
                  ICM_RATE_PROFILE.ahrsIntegrationDtSec,
                  (dlpfConfigStatus == ICM_20948_Stat_Ok &&
                   dlpfEnableStatus == ICM_20948_Stat_Ok) ? "ON" : "FAILED");
    actualImu = {ICM_RATE_PROFILE.accelOdrHz, ICM_RATE_PROFILE.gyroOdrHz,
                 ICM_RATE_PROFILE.ahrsIntegrationDtSec,
                 REQUESTED_IMU.lowPassCutoffHz, true};
    ahrs.setGyroIntegrationDt(actualImu.integrationDtSec);
  }
  baroOk = baro.begin(0x76);
  if (!baroOk) baroOk = baro.begin(0x77);
  if (baroOk) {
    baroKind = BaroKind::BMP280;
  } else {
    uint8_t chipId = 0;
    bool chipIdOk = i2cReadRegister(BERRY_BARO_ADDRESS, 0x00, chipId);
    Serial.printf("Berry barometer address=0x%02X chip_id=%s0x%02X\n",
                  BERRY_BARO_ADDRESS, chipIdOk ? "" : "unreadable/", chipId);
    if (chipIdOk && (chipId == 0x50 || chipId == 0x60) &&
        bmp3.begin_I2C(BERRY_BARO_ADDRESS, &Wire)) {
      bmp3.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
      bmp3.setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp3.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp3.setOutputDataRate(BMP3_ODR_25_HZ);
      baroKind = chipId == 0x50 ? BaroKind::BMP388 : BaroKind::BMP390;
      baroOk = true;
      // The BMP388 produces compensated but implausible pressure briefly
      // after reset. Preserve those raw samples as invalid log records, and
      // keep them out of altitude/climb state until the compensation settles.
      baroReadyAfterMs = millis() + 2500;
    }
  }
  if (imuKind == ImuKind::LSM9DS1) {
    imu.setupAccel(imu.LSM9DS1_ACCELRANGE_4G, imu.LSM9DS1_ACCELDATARATE_119HZ);
    imu.setupMag(imu.LSM9DS1_MAGGAIN_4GAUSS);
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);
  }
  Serial.printf("Qwiic GPS=%s QMC=%s IMU=%s BARO=%s\n", gpsOk ? "OK" : "ABSENT",
                qmcOk ? "OK" : "ABSENT",
                imuKind == ImuKind::ICM20948 ? "ICM20948" :
                imuKind == ImuKind::LSM9DS1 ? "LSM9DS1" :
                imuKind == ImuKind::QMI8658 ? "QMI8658" : "ABSENT",
                baroKindName());
}

bool qmcWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(0x2C); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool qmcRead(uint8_t reg, uint8_t *data, size_t length) {
  Wire.beginTransmission(0x2C); Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(0x2C, length) != length) return false;
  for (size_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

bool setupQmc5883p() {
  uint8_t id = 0;
  if (!qmcRead(0x0D, &id, 1)) return false;
  Serial.printf("QMC5883P address=0x2C ID=0x%02X\n", id);
  // QMC5883P: OSR=512, 8 gauss, 50 Hz, continuous mode.
  return qmcWrite(0x0B, 0x00) && qmcWrite(0x0C, 0x01) && qmcWrite(0x0A, 0xCD);
}

bool readQmc5883p(float &x, float &y, float &z) {
  uint8_t b[6];
  if (!qmcRead(0x01, b, sizeof(b))) return false;
  int16_t rx = (int16_t)((uint16_t)b[1] << 8 | b[0]);
  int16_t ry = (int16_t)((uint16_t)b[3] << 8 | b[2]);
  int16_t rz = (int16_t)((uint16_t)b[5] << 8 | b[4]);
  x = rx; y = ry; z = rz;
  return isfinite(x) && isfinite(y) && isfinite(z) && (rx || ry || rz);
}

void setupGPS() {
  if (HARDWARE.kind == HalBoardKind::TBeamSupreme) {
  pinMode(HARDWARE.gpsEnable, OUTPUT);
  digitalWrite(HARDWARE.gpsEnable, HIGH);
  pinMode(HARDWARE.gpsPps, INPUT);
  Serial.printf("GNSS control: EN=GPIO%d HIGH PPS=GPIO%d input\n",
                HARDWARE.gpsEnable, HARDWARE.gpsPps);
  delay(250);
  }
  static const uint32_t candidates[] = {38400, 9600, 115200};
  if (!sharedGps.begin(gpsSerial, HARDWARE.gpsRx, HARDWARE.gpsTx, candidates,
                       sizeof(candidates) / sizeof(candidates[0]))) {
    Serial.printf("u-blox GNSS not detected on GPIO%d/%d; UART activity probe failed\n",
                  HARDWARE.gpsRx, HARDWARE.gpsTx);
    return;
  }
  gpsOk = sharedGps.configure(38400, 10);
  Serial.printf("GPS config: UBX/NAV10/PVT=%s baud=%lu\n",
                gpsOk ? "OK" : "FAIL", (unsigned long)sharedGps.baud);
}

void updateGPS(uint32_t nowMs) {
  if (!gpsOk || !sharedGps.check()) return;
  gpsFixQuality = sharedGps.gnss.getFixType(0);
  gpsSatellites = sharedGps.gnss.getSIV(0);
  gpsPdop = sharedGps.gnss.getPDOP(0) * 0.01f;
  bool valid = gpsFixQuality >= 3;
  if (sessionLog.active()) {
    sessionLog.appendGps(nowMs,
      sharedGps.gnss.getLatitude(0), sharedGps.gnss.getLongitude(0),
      sharedGps.gnss.getAltitudeMSL(0), sharedGps.gnss.getGroundSpeed(0),
      sharedGps.gnss.getHeading(0), valid);
  }
  ahrs.updateGps(sharedGps.gnss.getHeading(0) * 1.0e-5f,
                 sharedGps.gnss.getGroundSpeed(0) * 1.0e-3f,
                 sharedGps.gnss.getAltitudeMSL(0) * 1.0e-3f,
                 valid, nowMs);
}

void setup() {
  Serial.begin(115200); delay(500);
  HARDWARE = detectBoard() == HalBoardKind::TBeamSupreme
      ? makeTBeamSupremeProfile() : makeGeekS3Profile();
  Serial.printf("HAL board auto-detect: %s\n", HARDWARE.name);
  // The global AHRS is constructed before runtime board detection, when the
  // provisional profile is GEEK. Reconstruct it from the detected profile so
  // per-board bias, polarity, and acceleration calibration cannot leak across
  // hardware variants.
  ahrs = AircraftAHRS(makeAhrsConfigFromHal());
  if (HARDWARE.hasLogButton) pinMode(HARDWARE.logButton, INPUT_PULLUP);
  halMakeSensorFrameRotation(HARDWARE.calibration.imu[0].sensorPitchOffsetDeg,
                             HARDWARE.calibration.imu[0].sensorRollOffsetDeg,
                             HARDWARE.calibration.imu[0].sensorYawOffsetDeg,
                             sensorFrameRotation);
  ahrs.setSensorFrameRotation(sensorFrameRotation);
  ahrs.setCompassCalibration(0, HARDWARE.calibration.compass[0].offset,
                             HARDWARE.calibration.compass[0].matrix);
  ahrs.setCompassCalibration(1, HARDWARE.calibration.compass[1].offset,
                             HARDWARE.calibration.compass[1].matrix);
  float compassFrameRotation[3][3];
  halMultiplyMatrix(sensorFrameRotation,
      HARDWARE.calibration.compass[0].frameRotation, compassFrameRotation);
  ahrs.setCompassFrameRotation(0, compassFrameRotation);
  halMultiplyMatrix(sensorFrameRotation,
      HARDWARE.calibration.compass[1].frameRotation, compassFrameRotation);
  ahrs.setCompassFrameRotation(1, compassFrameRotation);
#if defined(TBEAM_QMI_RELEASE_AFTER_INIT)
  setupDisplay(); setupStorage(); setupBerryIMU();
  if (HARDWARE.kind == HalBoardKind::TBeamSupreme && imuOk) {
    pinMode(34, OUTPUT); digitalWrite(34, HIGH);
    imuOk = false;
    imuKind = ImuKind::None;
    Serial.println("T-Beam QMI8658 initialized, then FSPI released for SD test");
  }
  setupGPS(); setupG5Logging();
#else
  setupDisplay(); startDisplayUpdateTask(); setupStorage(); setupBerryIMU(); setupGPS(); setupG5Logging();
#endif
  qmcPOk = setupQmc5883p();
  Serial.printf("LCD=%s SD=%s GPS=%s QMC=%s IMU=%s BARO=%s flash=%uMB PSRAM=%s freeRAM=%uKB freePSRAM=%uKB totalPSRAM=%uKB\n", displayOk ? "OK" : "FAIL",
                sdOk ? "OK" : "ABSENT", gpsOk ? "OK" : "ABSENT", qmcPOk ? "QMC5883P" : qmcOk ? "QMC?" : "ABSENT",
                imuOk ? "OK" : "ABSENT", baroOk ? "OK" : "ABSENT",
                ESP.getFlashChipSize() / 1048576,
                psramFound() ? "YES" : "NO",
                (unsigned)(ESP.getFreeHeap() / 1024),
                (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
                (unsigned)(ESP.getPsramSize() / 1024));
  Serial.println("SESSION_LOG READY (use START_LOG / STOP_LOG)");
}

void loop() {
  static uint32_t last = 0;
  static uint32_t nextCompass1SampleUs = 0;
  static uint32_t nextBaroSampleUs = 0;
  handleSerialCommands();
  updateLoggingButton();
  updateBootLogging();
  updateGPS(millis());
  if (qmcPOk) {
    float qx, qy, qz;
    bool valid = readQmc5883p(qx, qy, qz);
    uint32_t sampleUs = micros();
    if ((int32_t)(sampleUs - nextCompass1SampleUs) >= 0) {
      nextCompass1SampleUs = sampleUs + COMPASS1_OUTPUT_PERIOD_US;
      uint64_t nowUs = sampleUs;
      if (sessionLog.active()) sessionLog.appendCompass(1, nowUs, qx, qy, qz, valid);
      ahrs.updateCompass(1, qx, qy, qz, valid, millis());
    }
  }
  if (HARDWARE.kind == HalBoardKind::TBeamSupreme && qmcOk) {
    static uint32_t nextQmcSampleUs = 0;
    uint32_t sampleUs = micros();
    if ((int32_t)(sampleUs - nextQmcSampleUs) >= 0) {
      nextQmcSampleUs = sampleUs + COMPASS1_OUTPUT_PERIOD_US;
      MagnetometerData data{};
      bool valid = qmc6310.readData(data);
      float x = data.magnetic_field.x, y = data.magnetic_field.y,
            z = data.magnetic_field.z;
      valid = valid && isfinite(x) && isfinite(y) && isfinite(z);
      if (sessionLog.active()) sessionLog.appendCompass(1, sampleUs, x, y, z, valid);
      ahrs.updateCompass(1, x, y, z, valid, millis());
    }
  }
  std::string g5Packet;
  while (g5.read(g5Packet) > 0) {
    lastG5PacketMs = millis();
    if (isG5NmeaPacket(reinterpret_cast<const uint8_t *>(g5Packet.data()), g5Packet.size())) {
      ++discardedG5NmeaPackets;
      continue;
    }
    if (sessionLog.active()) sessionLog.append(FUSION_LOG_G5_PACKET, micros(),
                                                g5Packet.data(), g5Packet.size());
    Serial.printf("G5_PACKET len=%u\n", (unsigned)g5Packet.size());
  }
  if (imuOk) {
    sensors_event_t accel, gyro, mag;
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    if (imuKind == ImuKind::QMI8658) {
      if (!qmi8658.getDataReady()) { delay(1); return; }
      qmi8658.getAccelerometer(qmiAccel.x, qmiAccel.y, qmiAccel.z);
      qmi8658.getGyroscope(qmiGyro.x, qmiGyro.y, qmiGyro.z);
      ax = qmiAccel.x * 9.80665f; ay = qmiAccel.y * 9.80665f; az = qmiAccel.z * 9.80665f;
      gx = qmiGyro.x; gy = qmiGyro.y; gz = qmiGyro.z;
      mx = my = mz = NAN;
    } else if (imuKind == ImuKind::ICM20948) {
      if (!icm20948.dataReady()) { delay(1); return; }
      icm20948.getAGMT();
      // SparkFun's accX/Y/Z accessors return milli-g, not g or m/s^2.
      // Convert to the SI units expected by AircraftAHRS and the log format.
      ax = icm20948.accX() * 0.00980665f; ay = icm20948.accY() * 0.00980665f; az = icm20948.accZ() * 0.00980665f;
      gx = icm20948.gyrX(); gy = icm20948.gyrY(); gz = icm20948.gyrZ();
      mx = icm20948.magX(); my = icm20948.magY(); mz = icm20948.magZ();
    } else {
      imu.getEvent(&accel, &mag, &gyro, nullptr);
      ax = accel.acceleration.x; ay = accel.acceleration.y; az = accel.acceleration.z;
      gx = gyro.gyro.x * 57.2957795f; gy = gyro.gyro.y * 57.2957795f; gz = gyro.gyro.z * 57.2957795f;
      mx = mag.magnetic.x; my = mag.magnetic.y; mz = mag.magnetic.z;
    }
    uint64_t nowUs = micros();
    bool imuSampleValid = isfinite(ax) && isfinite(ay) && isfinite(az) && isfinite(gx) && isfinite(gy) && isfinite(gz);
    bool compass0Valid = isfinite(mx) && isfinite(my) && isfinite(mz);
    gyroDrift.add(0, gx, gy, gz);
    if (sessionLog.active()) sessionLog.appendImu(nowUs,
      gx, gy, gz, ax, ay, az, imuSampleValid);
    if (sessionLog.active()) sessionLog.appendCompass(0, nowUs,
      mx, my, mz, compass0Valid);
    // Preserve raw values in the log. Apply the HAL's installed-sensor to
    // aircraft-axis remap only to the values entering the AHRS.
    float bodyGx = gx, bodyGy = gy, bodyGz = gz;
    float bodyAx = ax, bodyAy = ay, bodyAz = az;
    const HalImuCalibration &imuCalibration = HARDWARE.calibration.imu[0];
    halApplyRawGyroCalibration(imuCalibration,
                               bodyGx, bodyGy, bodyGz);
    halApplySensorAxisRemap(imuCalibration.gyroAxisRemap,
                            bodyGx, bodyGy, bodyGz);
    halApplySensorAxisRemap(imuCalibration.sensorAxisRemap,
                            bodyAx, bodyAy, bodyAz);
    ahrs.updateImu(bodyGx, bodyGy, bodyGz, nowUs,
                   bodyAx, bodyAy, bodyAz, imuSampleValid);
    ahrs.updateCompass(0, mx, my, mz, compass0Valid, millis());
  }
  float secondaryTemp = NAN;
  if (secondaryImuOk) {
    float sgx, sgy, sgz, sax, say, saz;
    uint64_t secondaryNowUs = micros();
    if (readSecondaryLsm6(sgx, sgy, sgz, sax, say, saz, secondaryTemp)) {
      gyroDrift.add(1, sgx, sgy, sgz);
      if (sessionLog.active()) sessionLog.appendImu(1, secondaryNowUs,
        sgx, sgy, sgz, sax, say, saz,
        isfinite(sgx) && isfinite(sgy) && isfinite(sgz) &&
        isfinite(sax) && isfinite(say) && isfinite(saz));
    }
  }
  if (gyroDrift.active) {
    float primaryTemp = imuKind == ImuKind::QMI8658 ? qmi8658.getTemperature_C() : NAN;
    gyroDrift.update(millis(), primaryTemp, secondaryTemp);
  }
  uint32_t baroSampleUs = micros();
  if (baroOk && (int32_t)(baroSampleUs - nextBaroSampleUs) >= 0) {
    nextBaroSampleUs = baroSampleUs + BARO_OUTPUT_PERIOD_US;
    bool readOk = true;
    float pressurePa;
    if (baroKind == BaroKind::BME280) pressurePa = bme.readPressure();
    else if (baroKind == BaroKind::BMP280) pressurePa = baro.readPressure();
    else {
      readOk = bmp3.performReading();
      pressurePa = readOk ? (float)bmp3.pressure : NAN;
    }
    float altitudeM = 44330.0f *
        (1.0f - powf(pressurePa / 101325.0f, 0.19029496f));
    bool settled = baroReadyAfterMs == 0 ||
                   (int32_t)(millis() - baroReadyAfterMs) >= 0;
    bool valid = settled && readOk && isfinite(pressurePa) &&
                 isfinite(altitudeM) && pressurePa > 0.0f;
    if (valid) {
      latestBaroPressurePa = pressurePa;
      latestBaroAltitudeM = altitudeM;
    }
    if (sessionLog.active()) sessionLog.appendBaro(
        baroSampleUs, pressurePa, altitudeM, valid);
    ahrs.updateBaro(altitudeM, valid, millis());
  }
  if (millis() - last >= 1000) {
    last = millis();
    char packet[128];
    float pressure = 0.0f;
    if (baroOk) {
      pressure = latestBaroPressurePa / 100.0f;
    }
    snprintf(packet, sizeof(packet), "%s TEST=1 MILLIS=%lu LCD=%s SD=%s GPS=%s QMC=%s IMU=%s BARO=%s P=%.1f\n",
             HARDWARE.name, (unsigned long)last, displayOk ? "OK" : "FAIL", sdOk ? "OK" : "ABSENT",
             gpsOk ? "OK" : "ABSENT", qmcPOk ? "QMC5883P" : qmcOk ? "QMC6310N" : "ABSENT", imuOk ? "OK" : "ABSENT",
             baroOk ? "OK" : "ABSENT", pressure);
    espnow.write(packet, true);
    Serial.print(packet);
    const AircraftAHRS::State &fused = ahrs.state(millis());
    Serial.printf("AHRS roll=%.1f pitch=%.1f heading=%.1f baroAlt=%.1f climb=%.2f\n",
                  fused.rollDeg, fused.pitchDeg, fused.headingDeg,
                  fused.fusedAltitudeM, fused.fusedClimbRateMps);
    const AircraftAHRS::State &displayState = ahrs.state(last);
    HalDisplayStatus status{
      gpsOk, imuOk, baroOk, qmcOk || qmcPOk, sdOk, sessionLog.active(),
      lastG5PacketMs != 0 && (last - lastG5PacketMs) < 2000,
      gpsFixQuality, gpsSatellites, gpsPdop, pressure,
      displayState.rollDeg, displayState.pitchDeg, displayState.headingDeg,
      displayState.groundSpeedMps, last / 1000,
      static_cast<uint32_t>(sessionLog.dropped()),
      static_cast<uint32_t>(ESP.getFreeHeap() / 1024),
      static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
      sessionLog.elapsedSeconds()
    };
    publishDisplayStatus(status);
  }
  delay(1);
}
