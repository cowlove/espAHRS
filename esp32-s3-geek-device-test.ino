#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <SD.h>
#include <LovyanGFX.hpp>
#include <jimlib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_BMP280.h>
#include <espNowMux.h>
#include <reliableStream.h>

// Waveshare-style ESP32-S3 Geek pinout. Verify against the board revision.
constexpr int LCD_SCLK = 12, LCD_MOSI = 11, LCD_CS = 10, LCD_DC = 13;
constexpr int LCD_RST = 14, LCD_BL = 15;
constexpr int SD_CS = 4;

class GeekDisplay : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI _bus;
public:
  GeekDisplay() {
    auto b = _bus.config(); b.spi_host = SPI2_HOST; b.spi_mode = 0;
    b.freq_write = 40000000; b.pin_sclk = LCD_SCLK; b.pin_mosi = LCD_MOSI;
    b.pin_miso = -1; b.pin_dc = LCD_DC; _bus.config(b); _panel.setBus(&_bus);
    auto p = _panel.config(); p.pin_cs = LCD_CS; p.pin_rst = LCD_RST;
    p.panel_width = 135; p.panel_height = 240; p.offset_x = 0; p.offset_y = 0;
    p.invert = true; p.rgb_order = false; _panel.config(p); setPanel(&_panel);
  }
};

GeekDisplay display;
ReliableStreamESPNow espnow("GEEK", true /* alwaysBroadcast */);
Adafruit_LSM9DS1 imu;
Adafruit_BMP280 baro;
bool displayOk = false, sdOk = false, imuOk = false, baroOk = false;
bool gpsOk = false; // Reserved for the future Qwiic GPS module.

void setupDisplay() {
  pinMode(LCD_BL, OUTPUT); digitalWrite(LCD_BL, HIGH);
  display.init(); display.setRotation(1); display.fillScreen(TFT_BLACK);
  display.setTextColor(TFT_WHITE, TFT_BLACK); display.setTextSize(2);
  display.setCursor(4, 4); display.println("ESP32-S3 Geek");
  display.println("hardware test"); displayOk = true;
}

void setupStorage() {
  sdOk = SD.begin(SD_CS, SPI, 20000000);
  Serial.printf("microSD CS=%d: %s\n", SD_CS, sdOk ? "OK" : "not detected");
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
  setupDisplay(); setupStorage(); setupBerryIMU();
  Serial.printf("LCD=%s SD=%s GPS=%s IMU=%s BARO=%s flash=%uMB PSRAM=%s\n", displayOk ? "OK" : "FAIL",
                sdOk ? "OK" : "ABSENT", gpsOk ? "OK" : "ABSENT", imuOk ? "OK" : "ABSENT", baroOk ? "OK" : "ABSENT",
                ESP.getFlashChipSize() / 1048576,
                psramFound() ? "YES" : "NO");
}

void loop() {
  static uint32_t last = 0;
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
    if (displayOk) {
      display.fillRect(0, 42, 240, 70, TFT_BLACK); display.setCursor(4, 42);
      display.printf("1Hz %lus GPS %s\nIMU %s BARO %s\nP %.1f hPa SD %s",
                     last / 1000, gpsOk ? "OK" : "--", imuOk ? "OK" : "--",
                     baroOk ? "OK" : "--", pressure, sdOk ? "OK" : "--");
    }
  }
  delay(1);
}
