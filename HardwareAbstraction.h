#pragma once

#include <math.h>
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
  bool gps, imu, baro, compass, sd, logging, g5;
  uint8_t gpsFixQuality, gpsSatellites;
  float gpsPdop, pressureHpa, rollDeg, pitchDeg, headingDeg, groundSpeedMps;
  uint32_t uptimeSeconds, droppedLogRecords;
  uint32_t freeRamKb, freePsramKb;
  uint32_t loggingElapsedSeconds;
};

// Numeric sensor calibration belongs to the board profile just like pin
// wiring does.  The sensor devices themselves remain runtime-discovered.
struct HalCompassCalibration {
  float offset[3];
  float matrix[3][3];
  // Rotation from this compass's calibrated axes into the installed sensor
  // frame.  The common sensor-to-aircraft mounting rotation is applied after
  // this matrix, so replay offset sweeps still rotate every sensor together.
  float frameRotation[3][3];
};

struct HalImuCalibration {
  // Rotation from the installed sensor frame into the aircraft frame.
  float sensorPitchOffsetDeg;
  float sensorRollOffsetDeg;
  float sensorYawOffsetDeg;
  // Bias and polarity are in raw installed-sensor axes and are applied before
  // the sensor-to-aircraft mounting rotation.
  float gyroBiasDegSec[3];
  float gyroAxisSign[3];

  // Candidate estimate from the stable alignment portions.  It remains
  // disabled until a flight corpus confirms that it improves the solution.
  float accelBiasMps2[3];
  bool applyAccelBias;

  // Exact installed-sensor-axis -> aircraft-axis remap.  This handles the
  // large, discrete mounting orientation; fine Euler alignment is composed
  // after it at runtime.  Zero-initialized legacy profiles are normalized to
  // identity by setup().
  float sensorAxisRemap[3][3];
  // Some sensor packages expose gyro axes with a different sign convention
  // from their accelerometer axes. Keep this separate rather than forcing a
  // physically identical matrix when the device ABI requires otherwise.
  float gyroAxisRemap[3][3];

};

struct HalSensorCalibration {
  // Index matches the stable IMU source ID carried in the binary log.
  HalImuCalibration imu[4];
  // Index 0/1 matches AHRS compass source 0/1, not an IMU source.
  HalCompassCalibration compass[2];
};

constexpr HalImuCalibration halIdentityImuCalibration() {
  return {
    0.0f, 0.0f, 0.0f,
    {0.0f, 0.0f, 0.0f},
    {1.0f, 1.0f, 1.0f},
    {0.0f, 0.0f, 0.0f},
    false,
    {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
    {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}
  };
}

inline void halMakeSensorFrameRotation(float pitchDeg, float rollDeg, float yawDeg,
                                       float matrix[3][3]) {
  const float pitch = pitchDeg * 0.01745329252f;
  const float roll = rollDeg * 0.01745329252f;
  const float yaw = yawDeg * 0.01745329252f;
  const float cp = cosf(pitch), sp = sinf(pitch);
  const float cr = cosf(roll), sr = sinf(roll);
  const float cy = cosf(yaw), sy = sinf(yaw);
  matrix[0][0] = cy * cp; matrix[0][1] = -sy * cp; matrix[0][2] = sp;
  matrix[1][0] = cy * sr * sp + sy * cr;
  matrix[1][1] = -sy * sr * sp + cy * cr;
  matrix[1][2] = -sr * cp;
  matrix[2][0] = -cy * cr * sp + sy * sr;
  matrix[2][1] = sy * cr * sp + cy * sr;
  matrix[2][2] = cr * cp;
}

inline void halApplySensorAxisRemap(const float matrix[3][3],
                                    float &x, float &y, float &z) {
  const float inX = x, inY = y, inZ = z;
  x = matrix[0][0] * inX + matrix[0][1] * inY + matrix[0][2] * inZ;
  y = matrix[1][0] * inX + matrix[1][1] * inY + matrix[1][2] * inZ;
  z = matrix[2][0] * inX + matrix[2][1] * inY + matrix[2][2] * inZ;
}

inline void halApplyRawGyroCalibration(const HalImuCalibration &calibration,
                                       float &x, float &y, float &z) {
  x = (x - calibration.gyroBiasDegSec[0]) * calibration.gyroAxisSign[0];
  y = (y - calibration.gyroBiasDegSec[1]) * calibration.gyroAxisSign[1];
  z = (z - calibration.gyroBiasDegSec[2]) * calibration.gyroAxisSign[2];
}

inline void halRotateVector(const float matrix[3][3], float &x, float &y,
                            float &z) {
  const float a = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z;
  const float b = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z;
  const float c = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z;
  x = a; y = b; z = c;
}

inline void halMultiplyMatrix(const float left[3][3], const float right[3][3],
                              float result[3][3]) {
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result[row][column] = 0.0f;
      for (int k = 0; k < 3; ++k)
        result[row][column] += left[row][k] * right[k][column];
    }
  }
}

inline void halSetIdentityIfUnset(float matrix[3][3]) {
  bool unset = true;
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      if (matrix[row][column] != 0.0f) unset = false;
  if (unset)
    for (int row = 0; row < 3; ++row)
      for (int column = 0; column < 3; ++column)
        matrix[row][column] = row == column ? 1.0f : 0.0f;
}

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
  HalSensorCalibration calibration;
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
    true, true,
    {
      {
        {
          10.3f, 7.5f, 0.0f,
          {-0.48f, 0.05f, 0.16f},
          {1.0f, -1.0f, -1.0f},
          {-0.002f, -0.019f, 0.222f},
          false,
          {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
          {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}
        },
        // Current BerryIMUv3 installation.  The motion calibration log shows
        // that its gyro is already in body polarity, while its accelerometer
        // Y/Z axes need inversion to match IMU0.  Keep the same fine board
        // alignment until a per-device fit replaces it.
        {
          10.3f, 7.5f, 0.0f,
          {0.0f, 0.0f, 0.0f},
          {1.0f, 1.0f, 1.0f},
          {0.0f, 0.0f, 0.0f},
          false,
          {{1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, -1.0f}},
          {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}
        },
        halIdentityImuCalibration(), halIdentityImuCalibration()
      },
      {
        // Per-compass axis alignment fitted from the motion-alignment session
        // after ellipsoid correction.  The common mounting pitch/roll is
        // deliberately excluded and applied later to every sensor stream.
        {
          {-37.4412f, -2.7410f, -9.2134f},
          {
            {0.02139609f, 0.00013715f, 0.00008128f},
            {0.00013715f, 0.02186670f, -0.00068774f},
            {0.00008128f, -0.00068774f, 0.02277121f}
          },
          {
            {0.77213273f, -0.09675043f, -0.62805286f},
            {0.13093247f, 0.99135694f, 0.00825255f},
            {0.62182614f, -0.08860457f, 0.77812691f}
          }
        },
        {
          {149.5027f, 88.4811f, -73.3429f},
          {
            {0.00200813f, -0.00000361f, -0.00005624f},
            {0.00000361f, -0.00209991f, 0.00003358f},
            {0.00005624f, 0.00003358f, -0.00219467f}
          },
          {
            {0.84730183f, -0.04935318f, -0.52881365f},
            {0.05163944f, 0.99861103f, -0.01045820f},
            {0.52859529f, -0.01844639f, 0.84867352f}
          }
        }
      }
    }
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
  p.calibration.imu[1] = halIdentityImuCalibration();
  p.calibration.imu[2] = halIdentityImuCalibration();
  p.calibration.imu[3] = halIdentityImuCalibration();
  // Calibration for this mounted physical sample.  Keep it independent of
  // the GEEK profile and apply raw gyro biases before the axis remap.
  p.calibration.imu[0].sensorPitchOffsetDeg = 3.70f;
  p.calibration.imu[0].sensorRollOffsetDeg = -3.52f;
  p.calibration.imu[0].sensorYawOffsetDeg = 0.0f;
  // Raw QMI8658 sensor-axis zero-rate offsets, applied before the installed
  // sensor-to-aircraft remap.
  p.calibration.imu[0].gyroBiasDegSec[0] = 3.92f;
  p.calibration.imu[0].gyroBiasDegSec[1] = 0.12f;
  p.calibration.imu[0].gyroBiasDegSec[2] = -0.58f;
  p.calibration.imu[0].gyroAxisSign[0] = 1.0f;
  p.calibration.imu[0].gyroAxisSign[1] = 1.0f;
  p.calibration.imu[0].gyroAxisSign[2] = 1.0f;
  p.calibration.imu[0].applyAccelBias = false;
  // In the aircraft installation, the QMI sweep identified the installed
  // sensor axes as: aircraft nose=+QMI X, right=+QMI Z, down=+QMI Y.
  p.calibration.imu[0].sensorAxisRemap[0][0] = -1.0f;
  p.calibration.imu[0].sensorAxisRemap[0][1] = 0.0f;
  p.calibration.imu[0].sensorAxisRemap[0][2] = 0.0f;
  p.calibration.imu[0].sensorAxisRemap[1][0] = 0.0f;
  p.calibration.imu[0].sensorAxisRemap[1][1] = 0.0f;
  p.calibration.imu[0].sensorAxisRemap[1][2] = -1.0f;
  p.calibration.imu[0].sensorAxisRemap[2][0] = 0.0f;
  p.calibration.imu[0].sensorAxisRemap[2][1] = 1.0f;
  p.calibration.imu[0].sensorAxisRemap[2][2] = 0.0f;
  p.calibration.imu[0].gyroAxisRemap[0][0] = 1.0f;
  p.calibration.imu[0].gyroAxisRemap[0][1] = 0.0f;
  p.calibration.imu[0].gyroAxisRemap[0][2] = 0.0f;
  p.calibration.imu[0].gyroAxisRemap[1][0] = 0.0f;
  p.calibration.imu[0].gyroAxisRemap[1][1] = 0.0f;
  p.calibration.imu[0].gyroAxisRemap[1][2] = 1.0f;
  p.calibration.imu[0].gyroAxisRemap[2][0] = 0.0f;
  p.calibration.imu[0].gyroAxisRemap[2][1] = -1.0f;
  p.calibration.imu[0].gyroAxisRemap[2][2] = 0.0f;
  for (int compass = 0; compass < 2; ++compass) {
    for (int axis = 0; axis < 3; ++axis) {
      p.calibration.compass[compass].offset[axis] = 0.0f;
      for (int column = 0; column < 3; ++column) {
        p.calibration.compass[compass].matrix[axis][column] =
            axis == column ? 1.0f : 0.0f;
        p.calibration.compass[compass].frameRotation[axis][column] =
            axis == column ? 1.0f : 0.0f;
      }
    }
  }
  // T-Beam QMC6310N ellipsoid correction from /fusion-4397.bin.
  // This is source 1; source 0 remains the GEEK/QMC5883P calibration slot.
  p.calibration.compass[1].offset[0] = -1.382413f;
  p.calibration.compass[1].offset[1] = -0.255437f;
  p.calibration.compass[1].offset[2] = -0.504605f;
  const float tbeamMag[3][3] = {
    {0.368011f, 0.083384f, -0.046895f},
    {0.083384f, 0.784554f, 0.050228f},
    {-0.046895f, 0.050228f, 0.595937f}
  };
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      p.calibration.compass[1].matrix[row][column] = tbeamMag[row][column];
  // Seattle heading/dip plus the controlled yaw/pitch/roll sweep identify the
  // QMC axes as aircraft nose=+QMC Y, right=+QMC Z, down=+QMC X.  This is
  // relative to the QMI remap above because setup composes
  // sensorFrameRotation * compass.frameRotation.
  const float tbeamCompassFrame[3][3] = {
    {0.0f, 1.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
  };
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      p.calibration.compass[1].frameRotation[row][column] = tbeamCompassFrame[row][column];
  return p;
}

constexpr HalHardwareProfile makeHeadlessProfile() {
  auto p = makeGeekS3Profile();
  p.name = "headless";
  p.kind = HalBoardKind::Headless;
  p.display = HalDisplayKind::None;
  return p;
}
