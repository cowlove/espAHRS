#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <jimlib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_BMP280.h>
#include <espNowMux.h>
#include <reliableStream.h>
#include "AircraftAHRS.h"
#include "FusionSessionLog.h"

// Waveshare-style ESP32-S3 Geek pinout. Verify against the board revision.
// Pin map verified against the vendor ESP32-S3-GEEK demo sources.
constexpr int LCD_SCLK = 12, LCD_MOSI = 11, LCD_CS = 10, LCD_DC = 8;
constexpr int LCD_RST = 9, LCD_BL = 7;
constexpr int SD_CS = 34, SD_SCK = 36, SD_MISO = 37, SD_MOSI = 35;
constexpr int LOG_BUTTON = 0;

Adafruit_ST7789 display(LCD_CS, LCD_DC, LCD_RST);
SPIClass sdSpi(HSPI);
ReliableStreamESPNow espnow("GEEK", true /* alwaysBroadcast */);
Adafruit_LSM9DS1 imu;
Adafruit_BMP280 baro;
bool displayOk = false, sdOk = false, imuOk = false, baroOk = false;
bool gpsOk = false; // Reserved for the future Qwiic GPS module.
AircraftAHRS ahrs;
ReliableStreamESPNow g5("G5", true /* incoming benchmark traffic */);
FusionSessionLog sessionLog;
bool lastLogButton = true;
uint32_t logButtonChangedMs = 0;

void setupDisplay() {
  pinMode(LCD_BL, OUTPUT); digitalWrite(LCD_BL, HIGH);
  display.init(135, 240); display.setRotation(1); display.fillScreen(ST77XX_BLACK);
  display.setTextColor(ST77XX_WHITE, ST77XX_BLACK); display.setTextSize(2);
  display.setCursor(4, 4); display.println("ESP32-S3 Geek");
  display.println("hardware test"); displayOk = true;
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
        Serial.printf("SESSION_LOG STOPPED written=%lu dropped=%lu errors=%lu\n",
                      (unsigned long)sessionLog.written(),
                      (unsigned long)sessionLog.dropped(),
                      (unsigned long)sessionLog.writeErrors());
      } else if (sdOk && sessionLog.begin(SD)) {
        Serial.println("SESSION_LOG STARTED");
      } else {
        Serial.println("SESSION_LOG START FAILED");
      }
    }
  }
}

void setupBerryIMU() {
  // BerryIMUv3 normally uses I2C: LSM9DS1 XG=0x6B, magnetometer=0x1C,
  // BMP280=0x76 (some modules strap the barometer to 0x77).
  Wire.begin();
  Wire.setTimeOut(20); // Missing Qwiic hardware must never stall the test.
  imuOk = imu.begin();
  baroOk = baro.begin(0x76);
  if (!baroOk) baroOk = baro.begin(0x77);
  if (imuOk) {
    imu.setupAccel(imu.LSM9DS1_ACCELRANGE_4G, imu.LSM9DS1_ACCELDATARATE_119HZ);
    imu.setupMag(imu.LSM9DS1_MAGGAIN_4GAUSS);
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);
  }
  Serial.printf("Qwiic GPS=%s BerryIMUv3 LSM9DS1=%s BMP280=%s\n",
                gpsOk ? "OK" : "ABSENT", imuOk ? "OK" : "ABSENT", baroOk ? "OK" : "ABSENT");
}

void setup() {
  Serial.begin(115200); delay(500); Serial.println("ESP32-S3 Geek device test");
  Serial.println("Built in: LCD, microSD, WiFi/BLE, USB, UART, GPIO, I2C");
  pinMode(LOG_BUTTON, INPUT_PULLUP);
  setupDisplay(); setupStorage(); setupBerryIMU(); setupG5Logging();
  Serial.printf("LCD=%s SD=%s GPS=%s IMU=%s BARO=%s flash=%uMB PSRAM=%s\n", displayOk ? "OK" : "FAIL",
                sdOk ? "OK" : "ABSENT", gpsOk ? "OK" : "ABSENT", imuOk ? "OK" : "ABSENT", baroOk ? "OK" : "ABSENT",
                ESP.getFlashChipSize() / 1048576,
                psramFound() ? "YES" : "NO");
}

void loop() {
  static uint32_t last = 0;
  updateLoggingButton();
  std::string g5Packet;
  while (g5.read(g5Packet) > 0) {
    if (sessionLog.active()) sessionLog.append(FUSION_LOG_G5_PACKET, micros(),
                                                g5Packet.data(), g5Packet.size());
    Serial.printf("G5_PACKET len=%u\n", (unsigned)g5Packet.size());
  }
  if (imuOk) {
    sensors_event_t accel, gyro, mag;
    imu.getEvent(&accel, &mag, &gyro, nullptr);
    uint64_t nowUs = micros();
    if (sessionLog.active()) sessionLog.appendImu(nowUs,
      gyro.gyro.x * 57.2957795f, gyro.gyro.y * 57.2957795f,
      gyro.gyro.z * 57.2957795f, accel.acceleration.x,
      accel.acceleration.y, accel.acceleration.z, true);
    ahrs.updateImu(gyro.gyro.x * 57.2957795f, gyro.gyro.y * 57.2957795f,
                   gyro.gyro.z * 57.2957795f, nowUs,
                   accel.acceleration.x, accel.acceleration.y,
                   accel.acceleration.z, true);
  }
  if (baroOk) {
    float pressurePa = baro.readPressure();
    float altitudeM = baro.readAltitude(1013.25f);
    uint64_t nowUs = micros();
    if (sessionLog.active()) sessionLog.appendBaro(nowUs, pressurePa, altitudeM, true);
    ahrs.updateBaro(altitudeM, true, millis());
  }
  if (millis() - last >= 1000) {
    last = millis();
    char packet[128];
    sensors_event_t accel, gyro, mag;
    if (imuOk) imu.getEvent(&accel, &mag, &gyro, nullptr);
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
    if (displayOk) {
      display.fillRect(0, 42, 240, 70, ST77XX_BLACK); display.setCursor(4, 42);
      display.printf("1Hz %lus GPS %s\nIMU %s BARO %s\nP %.1f hPa SD %s",
                     last / 1000, gpsOk ? "OK" : "--", imuOk ? "OK" : "--",
                     baroOk ? "OK" : "--", pressure, sdOk ? "OK" : "--");
    }
  }
  delay(1);
}
