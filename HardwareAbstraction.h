#pragma once

#include <stdint.h>

// Board-specific wiring and capabilities. Sensor identities intentionally do
// not belong here: I2C devices are discovered at runtime.
enum class HalDisplayKind : uint8_t {
  None,
  GeekS3_ST7789
};

struct HalHardwareProfile {
  const char *name;
  int i2cSda, i2cScl;
  int gpsTx, gpsRx;
  int logButton;
  int lcdSclk, lcdMosi, lcdCs, lcdDc, lcdRst, lcdBacklight;
  int sdCs, sdSck, sdMiso, sdMosi;
  uint8_t gpsUartIndex;
  uint32_t gpsDefaultBaud;
  HalDisplayKind display;
  bool hasSd;
  bool hasLogButton;
};

// The only active hardware target today. A future board adds another profile
// here without changing sensor probing or application logic.
constexpr HalHardwareProfile makeGeekS3Profile() {
  return {
    "GEEK-S3",
    16, 17,                 // I2C SDA/SCL
    43, 44,                 // GPS TX/RX
    0,                      // start/stop button
    12, 11, 10, 8, 9, 7,    // ST7789
    34, 36, 37, 35,         // SD SPI
    1, 38400,
    HalDisplayKind::GeekS3_ST7789,
    true, true
  };
}

constexpr HalHardwareProfile makeHeadlessProfile() {
  auto p = makeGeekS3Profile();
  p.name = "headless";
  p.display = HalDisplayKind::None;
  return p;
}

