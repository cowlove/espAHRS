# ESP32-S3 Geek hardware test

For the detailed reusable bring-up lessons, see [BUILD_NOTES.md](BUILD_NOTES.md).
For the replay/AHRS parameter reference and tuning workflow, see
[TUNING.md](TUNING.md).
For the proposed standalone DipAHRS package scope and algorithm design, see
[DipAHRS.md](DipAHRS.md).
For the literature and public-code background of the magnetic-vector roll
observer, see [MAGNETIC_ROLL_PRIOR_ART.md](MAGNETIC_ROLL_PRIOR_ART.md).

The planned board-identity, per-device calibration, and persistent log naming
scheme is documented in [LOGGING_IDENTITY_PLAN.md](LOGGING_IDENTITY_PLAN.md).

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
| Adafruit LSM9DS1 Library | 2.2.1 | Legacy BerryIMU support |
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

## AircraftAHRS, replay, and session logging

The current buffer-first logfile implementation is temporary. It captures
records in PSRAM and pauses the runtime while flushing them to SD when logging
stops. This accommodates the T-Beam's shared QMI/SD SPI wiring; the intended
long-term design is a concurrent SD write stream. The ESP32-S3 PSRAM build
configuration is permanent and is separate from this temporary logging mode.

The GEEK project contains its own copies of the fusion, logging, GPS, and
replay sources. It no longer depends on a sibling T-Beam checkout. The
replay/AHRS tuning workflow is documented in [TUNING.md](TUNING.md).

### Board-specific build settings

Both boards compile the same application and HAL sources. Only the ESP32-S3
memory/boot mode is selected per device by the Makefile:

```sh
make DEVICE=tbeam                 # qio_qspi, octal PSRAM
make DEVICE=geek PORT=<geek-port> # qio_qspi, GEEK memory layout
```

At startup the GEEK firmware probes the ICM-20948 at both possible addresses,
then falls back to the LSM9DS1. The selected device's accelerometer, gyro, and
magnetometer are normalized into the existing IMU and compass-0 log records;
the device name is shown on the status display and serial startup line.

The non-blocking logger records raw G5 ESP-NOW frames, decoded G5 packets, IMU
samples, compass samples, GPS PVT records, and barometer samples when those
sources are available. GPIO0 toggles logging and the Geek board's microSD uses
the documented HSPI wiring: SCK GPIO36, MISO GPIO37, MOSI GPIO35, and CS
GPIO34.

### T-Beam display logging workaround

The T-Beam SH1106 display is rendered by a low-priority application-owned
FreeRTOS task through the board-specific HAL entry point. It uses U8g2 page
transfers with a delay between pages. When logging starts, the task renders the
`LOG` indication once and then freezes all display/I2C updates until logging
stops. This is a deliberate temporary real-time safeguard: the display remains
static during capture so its shared I2C bus cannot interfere with the sensor
loop. Display updates resume after the log is closed. The longer-term design
can refine the page scheduler and bus arbitration; two occasional mid-log
timing outliers remain under investigation.
