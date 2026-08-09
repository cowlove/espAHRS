#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <math.h>
#include <Arduino_CRC32.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <jimlib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_BMP280.h>
#include <ICM_20948.h>
#include <espNowMux.h>
#include <reliableStream.h>
#include "AircraftAHRS.h"
#include "FusionSessionLog.h"
#include "SharedUbloxGPS.h"

// Waveshare-style ESP32-S3 Geek pinout. Verify against the board revision.
// Pin map verified against the vendor ESP32-S3-GEEK demo sources.
constexpr int LCD_SCLK = 12, LCD_MOSI = 11, LCD_CS = 10, LCD_DC = 8;
constexpr int LCD_RST = 9, LCD_BL = 7;
constexpr int I2C_SDA = 16, I2C_SCL = 17;
// Exposed UART connector, cross-connected to the GPS module:
// GEEK TX43 -> GPS RX, GEEK RX44 <- GPS TX.
constexpr int GPS_TX = 43, GPS_RX = 44;
constexpr int SD_CS = 34, SD_SCK = 36, SD_MISO = 37, SD_MOSI = 35;
constexpr int LOG_BUTTON = 0;
constexpr uint32_t IMU_OUTPUT_PERIOD_US = 20000; // 50 Hz application stream

// This is the LCD configuration from the last known-good pre-status-page
// firmware.  Keep it unchanged until the panel is stable again.
Adafruit_ST7789 display(LCD_CS, LCD_DC, LCD_RST);
SPIClass sdSpi(HSPI);
ReliableStreamESPNow espnow("GEEK", true /* alwaysBroadcast */);
HardwareSerial gpsSerial(1);
Adafruit_LSM9DS1 imu;
ICM_20948_I2C icm20948;
Adafruit_BMP280 baro;
enum class ImuKind { None, LSM9DS1, ICM20948 };
ImuKind imuKind = ImuKind::None;
bool displayOk = false, sdOk = false, imuOk = false, baroOk = false, qmcOk = false;
bool qmcPOk = false;
bool gpsOk = false;
uint8_t gpsFixQuality = 0;
SharedUbloxGPS sharedGps;
AircraftAHRS ahrs;
ReliableStreamESPNow g5("G5", true /* incoming benchmark traffic */);
FusionSessionLog sessionLog;
bool lastLogButton = true;
uint32_t logButtonChangedMs = 0;
bool bootLogSession = false;
uint32_t bootLogDeadlineMs = 0;

void setupDisplay() {
  pinMode(LCD_BL, OUTPUT); digitalWrite(LCD_BL, HIGH);
  display.init(135, 240); display.setRotation(1); display.fillScreen(ST77XX_BLACK);
  display.setTextColor(ST77XX_WHITE, ST77XX_BLACK); display.setTextSize(2);
  display.setCursor(4, 4); display.println("ESP32-S3 Geek");
  display.println("hardware test"); displayOk = true;
}

void updateDisplay(uint32_t nowMs, float pressure) {
  if (!displayOk) return;

  display.fillScreen(ST77XX_BLACK);
  display.setTextSize(2); display.setCursor(2, 4);
  display.setTextColor(gpsOk ? ST77XX_GREEN : ST77XX_RED, ST77XX_BLACK);
  display.printf("G%u ", gpsFixQuality);
  display.setTextColor(qmcPOk ? ST77XX_GREEN : ST77XX_RED); display.print("QP ");
  display.setTextColor(imuOk ? ST77XX_GREEN : ST77XX_RED); display.print("IM ");
  display.setTextColor(sdOk ? ST77XX_GREEN : ST77XX_RED); display.print("SD\n");
  display.setCursor(2, 28); display.setTextColor(sessionLog.active() ? ST77XX_YELLOW : ST77XX_WHITE);
  display.print(sessionLog.active() ? "LOG" : "---");
  display.setTextColor(ST77XX_WHITE); display.printf(" D%lu", (unsigned long)sessionLog.dropped());
  display.setCursor(2, 52); display.printf("%lus", (unsigned long)(nowMs / 1000));
  display.setCursor(2, 76); display.printf("P%.0f", pressure);
}

void setupStorage() {
  sdSpi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdOk = SD.begin(SD_CS, sdSpi, 20000000);
  if (sdOk) sessionLog.recoverLatest(SD);
  Serial.printf("microSD SPI SCK=%d MISO=%d MOSI=%d CS=%d: %s\n",
                SD_SCK, SD_MISO, SD_MOSI, SD_CS, sdOk ? "OK" : "not detected");
}

bool clearFusionLogs() {
  File root = SD.open("/");
  if (!root) return false;
  bool ok = true; File entry;
  while ((entry = root.openNextFile())) {
    if (!entry.isDirectory()) {
      String name = entry.name();
      if (name.endsWith(".bin") && name.indexOf("fusion-") >= 0) {
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
        sessionLog.append(FUSION_LOG_G5_RAW_ESPNOW, micros(), raw, len + 6);
      }
    });
}

void updateLoggingButton() {
  bool pressed = digitalRead(LOG_BUTTON) == LOW;
  if (pressed != lastLogButton && millis() - logButtonChangedMs > 40) {
    logButtonChangedMs = millis(); lastLogButton = pressed;
    if (pressed) {
      if (sessionLog.active()) {
        sessionLog.stop();
        bootLogSession = false;
        Serial.printf("SESSION_LOG STOPPED written=%lu dropped=%lu errors=%lu\n",
                      (unsigned long)sessionLog.written(),
                      (unsigned long)sessionLog.dropped(),
                      (unsigned long)sessionLog.writeErrors());
      } else if (sdOk && sessionLog.begin(SD)) {
        bootLogSession = false;
        Serial.println("SESSION_LOG STARTED");
      } else {
        Serial.println("SESSION_LOG START FAILED");
      }
    }
  }
}

void updateBootLogging() {}

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

void dumpChunked(uint32_t startSeq = 0) {
  // Host-driven half-duplex plus CRC makes a 256-byte payload a good balance:
  // fewer transactions than the diagnostic 64-byte mode, while remaining
  // comfortably below the USB CDC buffering failure seen with raw streaming.
  const uint32_t chunkSize = 256;
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
    Serial.println("I2C_SCAN_BEGIN SDA=16 SCL=17");
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
        else if (sdOk && sessionLog.begin(SD)) Serial.println("SESSION_LOG STARTED");
        else Serial.println("SESSION_LOG START FAILED");
      } else if (command.equalsIgnoreCase("STOP_LOG")) {
        if (!sessionLog.active()) Serial.println("LOG_ERROR INACTIVE");
        else { sessionLog.stop(); Serial.printf("SESSION_LOG STOPPED written=%lu dropped=%lu errors=%lu\n", (unsigned long)sessionLog.written(), (unsigned long)sessionLog.dropped(), (unsigned long)sessionLog.writeErrors()); }
      } else if (command.equalsIgnoreCase("FORMAT")) {
        if (sessionLog.active()) Serial.println("LOG_ERROR ACTIVE");
        else if (sdOk && clearFusionLogs()) Serial.println("SD_FORMAT OK (logs cleared)");
        else Serial.println("SD_FORMAT FAIL");
      } else if (command.equalsIgnoreCase("DUMP") || command.startsWith("DUMP ")) {
        if (sessionLog.active()) {
          Serial.println("LOG_ERROR ACTIVE");
        } else {
          uint32_t startSeq = 0;
          if (command.startsWith("DUMP ")) startSeq = (uint32_t)command.substring(5).toInt();
          dumpChunked(startSeq);
        }
      } else if (command.equalsIgnoreCase("SCAN")) {
        scanI2c();
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
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(20); // Missing Qwiic hardware must never stall the test.
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
  baroOk = baro.begin(0x76);
  if (!baroOk) baroOk = baro.begin(0x77);
  if (imuKind == ImuKind::LSM9DS1) {
    imu.setupAccel(imu.LSM9DS1_ACCELRANGE_4G, imu.LSM9DS1_ACCELDATARATE_119HZ);
    imu.setupMag(imu.LSM9DS1_MAGGAIN_4GAUSS);
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);
  }
  Serial.printf("Qwiic GPS=%s QMC=%s IMU=%s BMP280=%s\n", gpsOk ? "OK" : "ABSENT",
                qmcOk ? "OK" : "ABSENT",
                imuKind == ImuKind::ICM20948 ? "ICM20948" :
                imuKind == ImuKind::LSM9DS1 ? "LSM9DS1" : "ABSENT",
                baroOk ? "OK" : "ABSENT");
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
  static const uint32_t candidates[] = {38400, 9600, 115200};
  if (!sharedGps.begin(gpsSerial, GPS_RX, GPS_TX, candidates,
                       sizeof(candidates) / sizeof(candidates[0]))) {
    Serial.println("u-blox GNSS not detected on GPIO43/44");
    return;
  }
  gpsOk = sharedGps.configure(38400, 10);
  Serial.printf("GPS config: UBX/NAV10/PVT=%s baud=%lu\n",
                gpsOk ? "OK" : "FAIL", (unsigned long)sharedGps.baud);
}

void updateGPS(uint32_t nowMs) {
  if (!gpsOk || !sharedGps.check()) return;
  gpsFixQuality = sharedGps.gnss.getFixType(0);
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
  Serial.begin(115200); delay(500); Serial.println("ESP32-S3 Geek device test");
  Serial.println("Built in: LCD, microSD, WiFi/BLE, USB, UART, GPIO, I2C");
  pinMode(LOG_BUTTON, INPUT_PULLUP);
  setupDisplay(); setupStorage(); setupBerryIMU(); setupGPS(); setupG5Logging();
  qmcPOk = setupQmc5883p();
  Serial.printf("LCD=%s SD=%s GPS=%s QMC=%s IMU=%s BARO=%s flash=%uMB PSRAM=%s\n", displayOk ? "OK" : "FAIL",
                sdOk ? "OK" : "ABSENT", gpsOk ? "OK" : "ABSENT", qmcPOk ? "QMC5883P" : qmcOk ? "QMC?" : "ABSENT",
                imuOk ? "OK" : "ABSENT", baroOk ? "OK" : "ABSENT",
                ESP.getFlashChipSize() / 1048576,
                psramFound() ? "YES" : "NO");
  Serial.println("SESSION_LOG READY (use START_LOG / STOP_LOG)");
}

void loop() {
  static uint32_t last = 0;
  static uint32_t nextImuSampleUs = 0;
  handleSerialCommands();
  updateLoggingButton();
  updateBootLogging();
  updateGPS(millis());
  if (qmcPOk) {
    float qx, qy, qz;
    bool valid = readQmc5883p(qx, qy, qz);
    uint64_t nowUs = micros();
    if (sessionLog.active()) sessionLog.appendCompass(1, nowUs, qx, qy, qz, valid);
    ahrs.updateCompass(1, qx, qy, qz, valid, millis());
  }
  std::string g5Packet;
  while (g5.read(g5Packet) > 0) {
    if (sessionLog.active()) sessionLog.append(FUSION_LOG_G5_PACKET, micros(),
                                                g5Packet.data(), g5Packet.size());
    Serial.printf("G5_PACKET len=%u\n", (unsigned)g5Packet.size());
  }
  if (imuOk) {
    sensors_event_t accel, gyro, mag;
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    if (imuKind == ImuKind::ICM20948) {
      if (!icm20948.dataReady()) { delay(1); return; }
      uint32_t readyUs = micros();
      if ((int32_t)(readyUs - nextImuSampleUs) < 0) { delay(1); return; }
      nextImuSampleUs = readyUs + IMU_OUTPUT_PERIOD_US;
      icm20948.getAGMT();
      ax = icm20948.accX() * 9.80665f; ay = icm20948.accY() * 9.80665f; az = icm20948.accZ() * 9.80665f;
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
    if (sessionLog.active()) sessionLog.appendImu(nowUs,
      gx, gy, gz, ax, ay, az, imuSampleValid);
    if (sessionLog.active()) sessionLog.appendCompass(0, nowUs,
      mx, my, mz, compass0Valid);
    ahrs.updateImu(gx, gy, gz, nowUs, ax, ay, az, imuSampleValid);
    ahrs.updateCompass(0, mx, my, mz, compass0Valid, millis());
  }
  if (baroOk) {
    float pressurePa = baro.readPressure();
    float altitudeM = baro.readAltitude(1013.25f);
    uint64_t nowUs = micros();
    bool baroSampleValid = isfinite(pressurePa) && isfinite(altitudeM) && pressurePa > 0.0f;
    if (sessionLog.active()) sessionLog.appendBaro(nowUs, pressurePa, altitudeM, baroSampleValid);
    ahrs.updateBaro(altitudeM, baroSampleValid, millis());
  }
  if (millis() - last >= 1000) {
    last = millis();
    char packet[128];
    float pressure = baroOk ? baro.readPressure() / 100.0f : 0.0f;
    snprintf(packet, sizeof(packet), "GEEK TEST=1 MILLIS=%lu LCD=%s SD=%s GPS=%s QMC=%s IMU=%s BARO=%s P=%.1f\n",
             (unsigned long)last, displayOk ? "OK" : "FAIL", sdOk ? "OK" : "ABSENT",
             gpsOk ? "OK" : "ABSENT", qmcPOk ? "QMC5883P" : qmcOk ? "QMC?" : "ABSENT", imuOk ? "OK" : "ABSENT",
             baroOk ? "OK" : "ABSENT", pressure);
    espnow.write(packet, true);
    Serial.print(packet);
    const AircraftAHRS::State &fused = ahrs.state(millis());
    Serial.printf("AHRS roll=%.1f pitch=%.1f heading=%.1f baroAlt=%.1f climb=%.2f\n",
                  fused.rollDeg, fused.pitchDeg, fused.headingDeg,
                  fused.fusedAltitudeM, fused.fusedClimbRateMps);
    updateDisplay(last, pressure);
  }
  delay(1);
}
