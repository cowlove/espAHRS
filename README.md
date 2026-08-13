# espAHRS

For the detailed reusable bring-up lessons, see [BUILD_NOTES.md](BUILD_NOTES.md).
For the replay/AHRS parameter reference and tuning workflow, see
[TUNING.md](TUNING.md).
For the proposed standalone DipAHRS package scope and algorithm design, see
[DipAHRS.md](DipAHRS.md).
For the literature and public-code background of the magnetic-vector roll
observer, see [MAGNETIC_ROLL_PRIOR_ART.md](MAGNETIC_ROLL_PRIOR_ART.md).

The planned board-identity, per-device calibration, and persistent log naming
scheme is documented in [LOGGING_IDENTITY_PLAN.md](LOGGING_IDENTITY_PLAN.md).

The espAHRS project is the flight-oriented AHRS and sensor-logging firmware
for the ESP32-S3 Geek board (Amazon ASIN B0CR6FV3QC). Hardware bring-up is
complete; the board name below identifies the supported target hardware.
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

The logger queues records from the sensor/G5 producers and writes them
incrementally to the SD card from a dedicated low-priority writer task. This
keeps card-write latency out of the real-time loop without making PSRAM the
log transport or imposing a PSRAM-sized maximum capture duration.

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

The external BerryIMUv3 barometer is a BMP388 at I2C address `0x77`. It uses
the maintained Adafruit BMP3XX driver with 2x temperature oversampling, 4x
pressure oversampling, coefficient-3 IIR filtering, and a 25 Hz output/log
schedule. Its brief reset transient is logged with `valid=0` and excluded from
the live altitude/climb solution. The verified full-system capture delivered
18.3 Hz; blocking compensated conversions and the other sensor work account
for the difference from the configured sensor ODR.

IMU calibration is indexed by the stable log source ID (`IMU0` through
`IMU3`). Each source has independent accelerometer and gyro axis-remap
matrices, gyro bias/polarity, accelerometer bias, and fine pitch/roll/yaw
alignment. Binary logs retain raw sensor values; live fusion applies IMU0's
calibration and replay applies the selected source's calibration. The current
GEEK `IMU1` mapping describes the temporary BerryIMUv3 mounting measured by
`fusion-5484.bin`.

### T-Beam display updates

The T-Beam SH1106 display is rendered by a low-priority application-owned
FreeRTOS task through the board-specific HAL entry point. It uses U8g2 page
transfers with a delay between pages and continues updating during logging.
The sensor loop and SD writer remain higher priority; occasional display bus
contention is therefore visible as display latency rather than a frozen panel.
