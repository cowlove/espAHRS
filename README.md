# espAHRS

For the detailed reusable bring-up lessons, see [BUILD_NOTES.md](BUILD_NOTES.md).
For the replay/AHRS parameter reference and tuning workflow, see
[TUNING.md](TUNING.md).
For the proposed standalone DipAHRS package scope and algorithm design, see
[DipAHRS.md](DipAHRS.md).
For the literature and public-code background of the magnetic-vector roll
observer, see [MAGNETIC_ROLL_PRIOR_ART.md](MAGNETIC_ROLL_PRIOR_ART.md).

The board-identity, per-device calibration, and persistent log naming
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

### Build settings

The ESP32-S3 Geek is the active hardware target. Build and flash it with:

```sh
make
make upload
```

The former T-Beam Supreme target is abandoned. Its historical source remains
for reference, but it is no longer built, flashed, tested, or considered when
changing the application.

At startup the GEEK firmware probes the ICM-20948 at both possible addresses,
then falls back to the LSM9DS1. The selected device's accelerometer, gyro, and
magnetometer are normalized into the existing IMU and compass-0 log records;
the device name is shown on the status display and serial startup line.

The non-blocking logger records raw G5 ESP-NOW frames, decoded G5 packets, IMU
samples, compass samples, GPS PVT records, and barometer samples when those
sources are available. GPIO0 toggles logging and the Geek board's microSD uses
the documented HSPI wiring: SCK GPIO36, MISO GPIO37, MOSI GPIO35, and CS
GPIO34.

New files use `G<MAC4><sequence>.bin`, for example `G247C001.bin`. The
zero-padded sequence is reserved in ESP32 NVS before file creation and keeps
increasing across reboots, firmware updates, SD formatting, and card swaps.
Legacy `fusion-*.bin` files remain listable and downloadable.

The external BerryIMUv3 barometer is a BMP388 at I2C address `0x77`. It uses
the maintained Adafruit BMP3XX driver with 2x temperature oversampling, 4x
pressure oversampling, coefficient-3 IIR filtering, and a 25 Hz output/log
schedule. Its brief reset transient is logged with `valid=0` and excluded from
the live altitude/climb solution. The verified full-system capture delivered
18.3 Hz; blocking compensated conversions and the other sensor work account
for the difference from the configured sensor ODR.

The BerryIMUv3 LSM6DSL accelerometer and gyro run at 104 Hz for the nominal
50 Hz application stream. Accelerometer LPF1 is set to ODR/4 (about 26 Hz);
the gyro's fixed LPF2 is about 33 Hz at this ODR.

Physical-device calibration is source-controlled in `DeviceConfiguration.h`
and selected by the ESP32's full base MAC address. The HAL contains board
wiring/display capabilities only. Replay uses the same table and accepts
`--device-mac XX:XX:XX:XX:XX:XX`; unknown firmware MACs are reported and run
with explicit identity calibration rather than borrowing another device's map.
Version-2 logs record the HAL kind, full MAC, profile name, configuration
revision, and deterministic calibration hash. Replay selects this profile
automatically and rejects source/configuration drift. Older logs remain
readable when their identity is supplied with `--device-mac`. That option
accepts a colon-separated MAC, a 12-digit MAC, or a unique trailing suffix of
at least four hexadecimal digits (for example `247C`).

IMU calibration is indexed by the stable log source ID (`IMU0` through
`IMU3`). Each source has independent accelerometer and gyro axis-remap
matrices, gyro bias/polarity, accelerometer bias, and fine pitch/roll/yaw
alignment. Binary logs retain raw sensor values; live fusion applies IMU0's
calibration and replay applies the selected source's calibration. The current
GEEK `IMU1` mapping describes the temporary BerryIMUv3 mounting measured by
`fusion-5484.bin`.

During recording, the Geek display shows `LOG <seconds>s` next to the active
logging indicator. The elapsed value comes from the current session's start
time and resets for every new file.

### Timed capture and stream-health summary

Use `capture_and_analyze.py` to start a session, record for 20 seconds, stop
and download the newest log, then print per-stream rates and timestamp gaps:

```sh
./capture_and_analyze.py --port /dev/ttyACM0 --duration 20
```

The output file can be selected with `--output`; the port and duration are
optional and default to `/dev/ttyACM0` and 20 seconds. The script requires
`pyserial` and uses the firmware's CRC-checked `DUMP` protocol.

To erase all `.bin` logs from the SD card, use the separate destructive
utility with its required confirmation flag:

```sh
./erase_logs.py --port /dev/ttyACM0 --yes
```
