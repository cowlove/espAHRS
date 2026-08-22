#pragma once

#include <stdint.h>

// Board-specific wiring and capabilities. Sensor identities intentionally do
// not belong here: I2C devices are discovered at runtime.
enum class HalDisplayKind : uint8_t {
  None,
  GeekS3_ST7789,
  TBeamSupreme_SH1106,
  TDisplayS3_ST7789_Parallel
};

enum class HalBoardKind : uint8_t {
  GeekS3,
  TBeamSupreme,
  TDisplayS3,
  Headless
};

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
  int lcdWr, lcdRd;
  int lcdData[8];
  int touchSda, touchScl, touchIrq, touchRst;
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
    -1, -1, {-1,-1,-1,-1,-1,-1,-1,-1},
    -1, -1, -1, -1,
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
  p.lcdWr = -1; p.lcdRd = -1;
  for (int &pin : p.lcdData) pin = -1;
  p.touchSda = p.touchScl = p.touchIrq = p.touchRst = -1;
  p.sdCs = 47; p.sdSck = 36; p.sdMiso = 37; p.sdMosi = 35;
  p.gpsUartIndex = 1;
  p.display = HalDisplayKind::TBeamSupreme_SH1106;
  p.hasSd = true; p.hasLogButton = true;
  return p;
}

// LilyGO T-Display-S3 touch-screen variant. These values come from the
// verified lilygo-t-display-s3 working sketch, not generic board examples.
// The LCD is an 8-bit parallel ST7789; the dedicated Qwiic bus and the CST816
// touch bus are separate, and GNSS uses the adjacent GPIO1/2 UART pair.
inline HalHardwareProfile makeTDisplayS3Profile() {
  auto p = makeGeekS3Profile();
  p.kind = HalBoardKind::TDisplayS3;
  p.name = "T-DISPLAY-S3";
  p.i2cSda = 43; p.i2cScl = 44;
  p.gpsTx = 1; p.gpsRx = 2; p.gpsEnable = -1; p.gpsPps = -1;
  p.logButton = -1;
  p.lcdSclk = -1; p.lcdMosi = -1; p.lcdCs = 6; p.lcdDc = 7;
  p.lcdRst = 5; p.lcdBacklight = 38;
  p.lcdWr = 8; p.lcdRd = 9;
  const int data[8] = {39,40,41,42,45,46,47,48};
  for (int i = 0; i < 8; ++i) p.lcdData[i] = data[i];
  p.touchSda = 18; p.touchScl = 17; p.touchIrq = 16; p.touchRst = 21;
  p.gpsUartIndex = 1; p.gpsDefaultBaud = 38400;
  p.display = HalDisplayKind::TDisplayS3_ST7789_Parallel;
  p.hasSd = false; p.hasLogButton = false;
  return p;
}

constexpr HalHardwareProfile makeHeadlessProfile() {
  auto p = makeGeekS3Profile();
  p.name = "headless";
  p.kind = HalBoardKind::Headless;
  p.display = HalDisplayKind::None;
  return p;
}
