#pragma once

#include "HardwareAbstraction.h"
#include <math.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

struct DeviceCompassCalibration {
  float offset[3];
  float matrix[3][3];
  float frameRotation[3][3];
};

struct DeviceImuCalibration {
  float sensorPitchOffsetDeg, sensorRollOffsetDeg, sensorYawOffsetDeg;
  float gyroBiasDegSec[3];
  float gyroAxisSign[3];
  float accelBiasMps2[3];
  bool applyAccelBias;
  float sensorAxisRemap[3][3];
  float gyroAxisRemap[3][3];
};

struct DeviceSensorCalibration {
  DeviceImuCalibration imu[4];
  DeviceCompassCalibration compass[2];
};

struct DeviceConfiguration {
  uint8_t mac[6];
  const char *name;
  const char *revision;
  HalBoardKind boardKind;
  DeviceSensorCalibration calibration;
};

constexpr DeviceImuCalibration identityImuCalibration() {
  return {0, 0, 0, {0, 0, 0}, {1, 1, 1}, {0, 0, 0}, false,
          {{1,0,0},{0,1,0},{0,0,1}}, {{1,0,0},{0,1,0},{0,0,1}}};
}

// Physical GEEK unit 28:37:2F:F8:24:7C. Numeric values are intentionally
// source-controlled so firmware, replay, simulation, and sweeps share them.
constexpr DeviceConfiguration DEVICE_CONFIGURATIONS[] = {{
  {0x28, 0x37, 0x2F, 0xF8, 0x24, 0x7C}, "geek-247c", "geek-247c-r5",
  HalBoardKind::GeekS3,
  {{
    {1.04055f, -0.67885f, 0.0f, {-0.45114f,0.01748f,0.10932f}, {1,-1,-1},
     {-0.002f,-0.019f,0.222f}, false,
     {{1,0,0},{0,1,0},{0,0,1}}, {{1,0,0},{0,1,0},{0,0,1}}},
    {0.0f, 0.0f, 0.0f, {0,0,0}, {1,1,1}, {0,0,0}, false,
     {{1,0,0},{0,-1,0},{0,0,-1}}, {{1,0,0},{0,1,0},{0,0,1}}},
    identityImuCalibration(), identityImuCalibration()
  }, {
    {{-37.4412f,-2.7410f,-9.2134f},
     {{0.02139609f,0.00013715f,0.00008128f},{0.00013715f,0.02186670f,-0.00068774f},{0.00008128f,-0.00068774f,0.02277121f}},
     // Retain the 21:22 diagnostic replay mapping: Compass 0 horizontal
     // axes aligned; calibrated aircraft-frame Z polarity inverted.
     {{1,0,0},{0,1,0},{0,0,-1}}},
    {{149.5027f,88.4811f,-73.3429f},
     // Soft-iron calibration is deliberately symmetric positive-definite;
     // sensor orientation belongs exclusively in frameRotation below.
     {{0.00200813f,-0.00000361f,-0.00005624f},{-0.00000361f,0.00209991f,-0.00003358f},{-0.00005624f,-0.00003358f,0.00219467f}},
     // Retain the 21:22 diagnostic replay mapping shared by both GNSS units.
     {{1,0,0},{0,-1,0},{0,0,-1}}}
  }}}, {
  {0x90, 0x70, 0x69, 0x85, 0xD5, 0xBC}, "geek-d5bc", "geek-d5bc-r5",
  HalBoardKind::GeekS3,
  {{
    {2.07970f, 7.53569f, 0.0f, {-0.23702f,0.21373f,-0.00810f}, {1,-1,-1},
     {0,0,0}, false,
     {{0,1,0},{-1,0,0},{0,0,1}}, {{0,-1,0},{1,0,0},{0,0,1}}},
    identityImuCalibration(), identityImuCalibration(), identityImuCalibration()
  }, {
    {{-16.3147f,-14.5599f,1.0301f},
     {{0.0212943f,-0.00027135f,-0.0000611f},{-0.00027135f,0.0209339f,-0.00004285f},{-0.0000611f,-0.00004285f,0.0207962f}},
     // GD5BC012 bench sequence: compass-0 sensor X/Y are exchanged and
     // the calibrated aircraft-frame polarity is X+, Y-, Z-.
     {{0,1,0},{-1,0,0},{0,0,-1}}},
    {{-11.3685f,119.5854f,-39.9449f},
     {{0.00196225f,0.0000370f,-0.0000828f},{0.0000370f,0.00205668f,0.0000274f},{-0.0000828f,0.0000274f,0.00199705f}},
     // Shared SEQURE GNSS compass mapping; see the 247C profile above.
     {{1,0,0},{0,-1,0},{0,0,-1}}}
  }}
}};

constexpr DeviceConfiguration UNCALIBRATED_DEVICE_CONFIGURATION = {
  {0,0,0,0,0,0}, "unknown-uncalibrated", "uncalibrated-r1",
  HalBoardKind::Headless,
  {{identityImuCalibration(), identityImuCalibration(), identityImuCalibration(), identityImuCalibration()},
   {{{0,0,0},{{1,0,0},{0,1,0},{0,0,1}},{{1,0,0},{0,1,0},{0,0,1}}},
    {{0,0,0},{{1,0,0},{0,1,0},{0,0,1}},{{1,0,0},{0,1,0},{0,0,1}}}}}
};

inline const DeviceConfiguration *findDeviceConfiguration(const uint8_t mac[6]) {
  for (const auto &configuration : DEVICE_CONFIGURATIONS)
    if (memcmp(configuration.mac, mac, 6) == 0) return &configuration;
  return nullptr;
}

enum class DeviceConfigurationLookup { Found, Invalid, Unknown, Ambiguous };

inline DeviceConfigurationLookup resolveDeviceConfiguration(
    const char *text, const DeviceConfiguration *&result) {
  result = nullptr;
  if (!text) return DeviceConfigurationLookup::Invalid;
  char suffix[13]{};
  size_t length = 0;
  for (const char *p=text; *p; ++p) {
    if (*p == ':') continue;
    if (!isxdigit(static_cast<unsigned char>(*p)) || length >= 12)
      return DeviceConfigurationLookup::Invalid;
    suffix[length++] = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
  }
  // Four hexadecimal digits is the shortest accepted git-style abbreviation.
  if (length < 4) return DeviceConfigurationLookup::Invalid;
  for (const auto &configuration : DEVICE_CONFIGURATIONS) {
    char full[13];
    snprintf(full, sizeof(full), "%02X%02X%02X%02X%02X%02X",
             configuration.mac[0],configuration.mac[1],configuration.mac[2],
             configuration.mac[3],configuration.mac[4],configuration.mac[5]);
    if (memcmp(full + 12 - length, suffix, length) != 0) continue;
    if (result) return DeviceConfigurationLookup::Ambiguous;
    result = &configuration;
  }
  return result ? DeviceConfigurationLookup::Found : DeviceConfigurationLookup::Unknown;
}

inline uint32_t deviceConfigurationHash(const DeviceConfiguration &configuration) {
  uint32_t hash = 2166136261u;
  auto add = [&hash](const void *value, size_t size) {
    const uint8_t *bytes = static_cast<const uint8_t *>(value);
    for (size_t i=0; i<size; ++i) hash=(hash^bytes[i])*16777619u;
  };
  for (const auto &imu : configuration.calibration.imu) {
    add(&imu.sensorPitchOffsetDeg, sizeof(float));
    add(&imu.sensorRollOffsetDeg, sizeof(float));
    add(&imu.sensorYawOffsetDeg, sizeof(float));
    add(imu.gyroBiasDegSec, sizeof(imu.gyroBiasDegSec));
    add(imu.gyroAxisSign, sizeof(imu.gyroAxisSign));
    add(imu.accelBiasMps2, sizeof(imu.accelBiasMps2));
    const uint8_t applyAccelBias = imu.applyAccelBias ? 1 : 0;
    add(&applyAccelBias, sizeof(applyAccelBias));
    add(imu.sensorAxisRemap, sizeof(imu.sensorAxisRemap));
    add(imu.gyroAxisRemap, sizeof(imu.gyroAxisRemap));
  }
  for (const auto &compass : configuration.calibration.compass) {
    add(compass.offset, sizeof(compass.offset));
    add(compass.matrix, sizeof(compass.matrix));
    add(compass.frameRotation, sizeof(compass.frameRotation));
  }
  return hash;
}

inline void makeSensorFrameRotation(float pitchDeg, float rollDeg, float yawDeg,
                                    float matrix[3][3]) {
  const float p=pitchDeg*0.01745329252f, r=rollDeg*0.01745329252f, y=yawDeg*0.01745329252f;
  const float cp=cosf(p),sp=sinf(p),cr=cosf(r),sr=sinf(r),cy=cosf(y),sy=sinf(y);
  matrix[0][0]=cy*cp; matrix[0][1]=-sy*cp; matrix[0][2]=sp;
  matrix[1][0]=cy*sr*sp+sy*cr; matrix[1][1]=-sy*sr*sp+cy*cr; matrix[1][2]=-sr*cp;
  matrix[2][0]=-cy*cr*sp+sy*sr; matrix[2][1]=sy*cr*sp+cy*sr; matrix[2][2]=cr*cp;
}
inline void applyAxisRemap(const float m[3][3], float &x,float &y,float &z) {
  const float a=x,b=y,c=z; x=m[0][0]*a+m[0][1]*b+m[0][2]*c; y=m[1][0]*a+m[1][1]*b+m[1][2]*c; z=m[2][0]*a+m[2][1]*b+m[2][2]*c;
}
inline void applyRawGyroCalibration(const DeviceImuCalibration &c,float &x,float &y,float &z) {
  x=(x-c.gyroBiasDegSec[0])*c.gyroAxisSign[0]; y=(y-c.gyroBiasDegSec[1])*c.gyroAxisSign[1]; z=(z-c.gyroBiasDegSec[2])*c.gyroAxisSign[2];
}
inline void multiplyMatrix(const float a[3][3],const float b[3][3],float out[3][3]) {
  for(int r=0;r<3;++r) for(int c=0;c<3;++c) { out[r][c]=0; for(int k=0;k<3;++k) out[r][c]+=a[r][k]*b[k][c]; }
}
