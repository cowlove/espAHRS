#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <math.h>
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
constexpr uint32_t BOOT_LOG_DURATION_MS = 120000;
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
bool displayOk = false, sdOk = false, imuOk = false, baroOk = false;
bool gpsOk = false;
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
  display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display.setTextSize(1);
  display.setCursor(4, 4);
  display.print("GEEK LIVE "); display.print((unsigned long)(nowMs / 1000)); display.println("s");
  display.drawFastHLine(0, 14, display.width(), ST77XX_BLUE);
  int y = 18;
  display.setCursor(4, y); display.print("GPS   "); display.println(gpsOk ? "OK" : "ABSENT"); y += 10;
  display.setCursor(4, y); display.print("IMU   ");
  display.println(imuKind == ImuKind::ICM20948 ? "ICM20948" :
                  imuKind == ImuKind::LSM9DS1 ? "LSM9DS1" : "ABSENT"); y += 10;
  display.setCursor(4, y); display.print("BARO  "); display.println(baroOk ? "OK" : "ABSENT"); y += 10;
  display.setCursor(4, y); display.print("SD    "); display.println(sdOk ? "OK" : "ABSENT"); y += 10;
  display.setCursor(4, y); display.print("LOG   "); display.println(sessionLog.active() ? "ACTIVE" : "INACTIVE"); y += 10;
  display.setCursor(4, y); display.print("DROP  "); display.println((unsigned long)sessionLog.dropped()); y += 10;
  display.setCursor(4, y); display.print("PRES  "); display.print(pressure, 1); display.println(" hPa");
}

void setupStorage() {
  sdSpi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdOk = SD.begin(SD_CS, sdSpi, 20000000);
  Serial.printf("microSD SPI SCK=%d MISO=%d MOSI=%d CS=%d: %s\n",
                SD_SCK, SD_MISO, SD_MOSI, SD_CS, sdOk ? "OK" : "not detected");
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

void updateBootLogging() {
  if (bootLogSession && sessionLog.active() &&
      (int32_t)(millis() - bootLogDeadlineMs) >= 0) {
    sessionLog.stop();
    bootLogSession = false;
    Serial.printf("SESSION_LOG AUTO-STOPPED written=%lu dropped=%lu errors=%lu\n",
                  (unsigned long)sessionLog.written(),
                  (unsigned long)sessionLog.dropped(),
                  (unsigned long)sessionLog.writeErrors());
  }
}

void handleSerialCommands() {
  static String command;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      command.trim();
      if (command.equalsIgnoreCase("DUMP")) {
        if (sessionLog.active()) {
          Serial.println("LOG_ERROR ACTIVE");
        } else {
          size_t size = sessionLog.fileSize();
          Serial.printf("LOG_BEGIN %s %lu\n", sessionLog.fileName(),
                        (unsigned long)size);
          Serial.flush();
          size_t sent = sessionLog.dumpTo(Serial);
          Serial.flush();
          Serial.printf("\nLOG_END %lu\n", (unsigned long)sent);
        }
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
  Serial.printf("Qwiic GPS=%s IMU=%s BMP280=%s\n", gpsOk ? "OK" : "ABSENT",
                imuKind == ImuKind::ICM20948 ? "ICM20948" :
                imuKind == ImuKind::LSM9DS1 ? "LSM9DS1" : "ABSENT",
                baroOk ? "OK" : "ABSENT");
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
  bool valid = sharedGps.gnss.getFixType(0) >= 3;
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
  Serial.printf("LCD=%s SD=%s GPS=%s IMU=%s BARO=%s flash=%uMB PSRAM=%s\n", displayOk ? "OK" : "FAIL",
                sdOk ? "OK" : "ABSENT", gpsOk ? "OK" : "ABSENT", imuOk ? "OK" : "ABSENT", baroOk ? "OK" : "ABSENT",
                ESP.getFlashChipSize() / 1048576,
                psramFound() ? "YES" : "NO");
  if (sdOk && sessionLog.begin(SD)) {
    bootLogSession = true;
    bootLogDeadlineMs = millis() + BOOT_LOG_DURATION_MS;
    Serial.printf("SESSION_LOG AUTO-STARTED duration=%lums\n",
                  (unsigned long)BOOT_LOG_DURATION_MS);
  } else {
    Serial.println("SESSION_LOG AUTO-START FAILED");
  }
}

void loop() {
  static uint32_t last = 0;
  static uint32_t nextImuSampleUs = 0;
  handleSerialCommands();
  updateLoggingButton();
  updateBootLogging();
  updateGPS(millis());
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
    snprintf(packet, sizeof(packet), "GEEK TEST=1 MILLIS=%lu LCD=%s SD=%s GPS=%s IMU=%s BARO=%s P=%.1f\n",
             (unsigned long)last, displayOk ? "OK" : "FAIL", sdOk ? "OK" : "ABSENT",
             gpsOk ? "OK" : "ABSENT", imuOk ? "OK" : "ABSENT", baroOk ? "OK" : "ABSENT", pressure);
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
