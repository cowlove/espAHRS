#pragma once

#include <math.h>
#include <stdint.h>

// Board-specific wiring and capabilities. Sensor identities intentionally do
// not belong here: I2C devices are discovered at runtime.
enum class HalDisplayKind : uint8_t {
  None,
  GeekS3_ST7789
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

struct HalSensorCalibration {
  // Rotation from the installed sensor frame into the aircraft frame.
  float sensorPitchOffsetDeg;
  float sensorRollOffsetDeg;
  float gyroBiasDegSec[3];
  float gyroAxisSign[3];

  // Candidate estimate from the stable alignment portions.  It remains
  // disabled until a flight corpus confirms that it improves the solution.
  float accelBiasMps2[3];
  bool applyAccelBias;

  // Index 0/1 matches AHRS compass source 0/1, not a sensor identity.
  HalCompassCalibration compass[2];
};

inline void halMakeSensorFrameRotation(float pitchDeg, float rollDeg,
                                       float matrix[3][3]) {
  const float pitch = pitchDeg * 0.01745329252f;
  const float roll = rollDeg * 0.01745329252f;
  const float cp = cosf(pitch), sp = sinf(pitch);
  const float cr = cosf(roll), sr = sinf(roll);
  matrix[0][0] = cp;       matrix[0][1] = 0.0f; matrix[0][2] = sp;
  matrix[1][0] = sr * sp;   matrix[1][1] = cr;   matrix[1][2] = -sr * cp;
  matrix[2][0] = -cr * sp;  matrix[2][1] = sr;   matrix[2][2] = cr * cp;
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
  HalSensorCalibration calibration;
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
    true, true,
    {
      10.3f, 7.5f,
      {-0.48f, 0.05f, 0.16f},
      {1.0f, -1.0f, 1.0f},
      {-0.002f, -0.019f, 0.222f},
      false,
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

constexpr HalHardwareProfile makeHeadlessProfile() {
  auto p = makeGeekS3Profile();
  p.name = "headless";
  p.display = HalDisplayKind::None;
  return p;
}
