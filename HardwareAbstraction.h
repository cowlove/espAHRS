#pragma once

#include <stdint.h>

// Board-specific wiring and capabilities. Sensor identities intentionally do
// not belong here: I2C devices are discovered at runtime.
enum class HalDisplayKind : uint8_t {
  None,
  GeekS3_ST7789,
  TBeamSupreme_SH1106
};

enum class HalBoardKind : uint8_t { GeekS3, TBeamSupreme, Headless };

struct HalImuRequest {
  float sampleRateHz;
  float lowPassCutoffHz;
};

struct HalImuConfiguration {
  float accelRateHz;
  float gyroRateHz;
  float integrationDtSec;
  float lowPassCutoffHz;
  bool hardwareLowPassEnabled;
};

struct HalDisplayStatus {
  bool gps, imu1, baro, compass, sd, logging, g5;
  uint8_t gpsFixQuality, gpsSatellites;
  float gpsPdop, pressureHpa, rollDeg, pitchDeg, headingDeg, groundSpeedMps;
  uint32_t uptimeSeconds, droppedLogRecords;
  uint32_t freeRamKb, freePsramKb;
  uint32_t loggingElapsedSeconds;
};

struct HalHardwareProfile {
  HalBoardKind kind;
  const char *name;
  int i2cSda, i2cScl;
  int gpsTx, gpsRx, gpsEnable, gpsPps;
  int logButton;
  int lcdSclk, lcdMosi, lcdCs, lcdDc, lcdRst, lcdBacklight;
  int sdCs, sdSck, sdMiso, sdMosi;
  uint8_t gpsUartIndex;
  uint32_t gpsDefaultBaud;
  HalDisplayKind display;
  bool hasSd;
  bool hasLogButton;
};

// The GEEK profile retains the calibrated values from the flight corpus.
constexpr HalHardwareProfile makeGeekS3Profile() {
  return {
    HalBoardKind::GeekS3,
    "GEEK-S3",
    16, 17,                 // I2C SDA/SCL
    43, 44, 255, 255,       // GPS TX/RX, enable/PPS unused on GEEK
    0,                      // start/stop button
    12, 11, 10, 8, 9, 7,    // ST7789
    34, 36, 37, 35,         // SD SPI
    1, 38400,
    HalDisplayKind::GeekS3_ST7789,
    true, true
  };
}

// LilyGO T-Beam Supreme V3.0.  The board uses a QMI8658 over SPI, an SH1106
// over the peripheral I2C bus, an AXP2101 on Wire1, and a u-blox receiver on
// UART1.  The generic application currently treats the display as optional;
// the pin map is nevertheless kept here so board bring-up and later drivers
// use one authoritative definition.
constexpr HalHardwareProfile makeTBeamSupremeProfile() {
  auto p = makeGeekS3Profile();
  p.name = "T-BEAM-SUPREME";
  p.kind = HalBoardKind::TBeamSupreme;
  p.i2cSda = 17; p.i2cScl = 18;
  // LilyGO's vendor GPS example is authoritative for the onboard receiver:
  // RX=9, TX=8, enable=7, PPS=6. GPIO43/44 are expansion UART pins.
  p.gpsTx = 8; p.gpsRx = 9; p.gpsEnable = 7; p.gpsPps = 6;
  p.logButton = 0;
  p.lcdSclk = 36; p.lcdMosi = 35; p.lcdCs = -1; p.lcdDc = -1;
  p.lcdRst = -1; p.lcdBacklight = -1;
  p.sdCs = 47; p.sdSck = 36; p.sdMiso = 37; p.sdMosi = 35;
  p.gpsUartIndex = 1;
  p.display = HalDisplayKind::TBeamSupreme_SH1106;
  p.hasSd = true; p.hasLogButton = true;
  return p;
}

constexpr HalHardwareProfile makeHeadlessProfile() {
  auto p = makeGeekS3Profile();
  p.name = "headless";
  p.kind = HalBoardKind::Headless;
  p.display = HalDisplayKind::None;
  return p;
}
