# ESP32-S3 Geek hardware test

For the detailed reusable bring-up lessons, see [BUILD_NOTES.md](BUILD_NOTES.md).

Bring-up project for the ESP32-S3 Geek board (Amazon ASIN B0CR6FV3QC).
The test initializes the 1.14-inch LCD, probes the TF/microSD slot, reports
ESP32 flash/PSRAM, and sends a `ReliableStreamESPNow("GEEK")` diagnostic once
per second.

The BerryIMUv3, SparkFun Qwiic 9DoF (ICM-20948), and SEQURE M10-25Q GPS are optional external Qwiic/I2C
peripherals. They are not required for the firmware to run. Missing sensors
are reported as `ABSENT`, with a 20 ms I2C timeout.

## Arduino CLI dependencies

Install the downloaded libraries with:

```sh
arduino-cli lib install \
  'Adafruit GFX Library' \
  'Adafruit ST7735 and ST7789 Library' \
  'Adafruit LSM9DS1 Library' \
  'Adafruit BMP280 Library' \
  'Adafruit Unified Sensor' \
  'Adafruit BusIO' \
  'Adafruit LIS3MDL'
```

Versions used during bring-up:

| Library | Version | Used for |
|---|---:|---|
| Adafruit GFX Library | 1.12.6 | Display drawing primitives |
| Adafruit ST7735 and ST7789 Library | 1.11.0 | ST7789-class 240×135 LCD |
| Adafruit LSM9DS1 Library | 2.2.1 | BerryIMUv3 accelerometer, gyro, magnetometer |
| SparkFun ICM-20948 Arduino Library | current | SparkFun Qwiic 9DoF ICM-20948 accelerometer, gyro, magnetometer |
| Adafruit BMP280 Library | 3.0.0 | BerryIMUv3 pressure/altitude |
| Adafruit LIS3MDL | 1.2.5 | LSM9DS1 library dependency |
| Adafruit BusIO | 1.17.4 | Adafruit sensor-library dependency |
| Adafruit Unified Sensor | 1.1.15 | Adafruit sensor interface |

These are local/shared project dependencies and do not need downloading:

- `esp32jimlib` — `jimlib.h`, `reliableStream.h`, ESP-NOW support
- `Arduino_CRC32` — ReliableStream framing CRC
- ESP32 Arduino core 3.2.0 — Wi-Fi, SPI, SD, Wire, USB support

## Build and monitor

```sh
make
make upload PORT=/dev/ttyACM0
make cat PORT=/dev/ttyACM0
```

The current Makefile has explicit exclusions for unrelated global libraries.
Arduino CLI compilation is also supported when the Makefile scanner encounters
platform-library conflicts:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3 .
```

## External sensor addresses

## LCD pin map

Verified against the vendor ESP32-S3-GEEK demo sources: SCLK `12`, MOSI
`11`, CS `10`, DC `8`, reset `9`, and backlight `7`.

- LSM9DS1 accelerometer/gyro: `0x6B`
- LSM9DS1 magnetometer: `0x1C`
- ICM-20948: `0x68` or `0x69`, selected by the board's ADR link
- BMP280: `0x76`, with fallback to `0x77`
- Future GPS: not yet implemented; currently reported as `GPS=ABSENT`

## Shared AircraftAHRS and session logging

The Geek test imports the shared fusion implementation from the sibling
`tbeam-supreme-device-test` checkout by default:

```sh
make FUSION_SHARED_DIR=../tbeam-supreme-device-test
```

At startup the GEEK firmware probes the ICM-20948 at both possible addresses,
then falls back to the LSM9DS1. The selected device's accelerometer, gyro, and
magnetometer are normalized into the existing IMU and compass-0 log records;
the device name is shown on the status display and serial startup line.

This supplies `AircraftAHRS.h/.cpp` and `FusionSessionLog.h`. The same
non-blocking logger records raw G5 ESP-NOW frames, decoded G5 packets, IMU
samples, and barometer samples. GPIO0 toggles logging and the Geek board's
microSD uses the documented HSPI wiring: SCK GPIO36, MISO GPIO37, MOSI GPIO35,
and CS GPIO34. GPS records remain absent until a GPS source is added.

For a different checkout or packaged shared library, override
`FUSION_SHARED_DIR` with its path. Keep the shared source and both projects on
compatible commits when collecting replay data.
