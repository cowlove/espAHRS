# Build and configuration notes

This project is intended to be a reference for bringing up unfamiliar ESP32
development boards. The final working workflow is the Makefile build, not an
Arduino CLI compile. Arduino CLI was useful for library installation and an
early comparison build, but it scans the whole user library tree differently
from `makeEspArduino` and exposed unrelated global-library problems.

## Board identification

The Amazon listing was not sufficient to establish the complete pin map. The
board was identified as the Waveshare/Spotpear ESP32-S3-GEEK: ESP32-S3R2,
16 MB flash, 2 MB PSRAM, ST7789P3 240x135 LCD, TF slot, USB-A, UART, GPIO,
and I2C headers. The vendor demo archive was downloaded and inspected before
finalizing the LCD pins.

Verified LCD map:

| Signal | GPIO |
|---|---:|
| SPI SCLK | 12 |
| SPI MOSI | 11 |
| LCD CS | 10 |
| LCD DC | 8 |
| LCD reset | 9 |
| Backlight | 7 |

## TF-card pinout mismatch and resolution

The storage failure was caused by trusting a plausible-looking pinout from
the wrong board family. The board is the Waveshare ESP32-S3-GEEK, not the
LILYGO T-Dongle-S3. Those boards look similar in search results but do not
share the same TF-card wiring.

The first storage implementation assumed an ordinary SPI card with CS on
GPIO4 and called:

```cpp
SD.begin(4, SPI, 20000000);
```

That assumption came from the initial project configuration and from a
generic ST7789/SD example. It produced `SD=ABSENT` even after a card was
reformatted. Reformatting was a worthwhile check because the original card
was mounting read-only on the laptop, but it was not the remaining cause.

A web search also surfaced the LILYGO T-Dongle-S3 documentation, which lists
TF pins GPIO5, GPIO3, GPIO7, and GPIO4 for SD/MMC 1-bit mode. Trying those
values and `SD_MMC.begin()` was incorrect for this hardware and produced
`sdmmc_card_init failed` errors. That was a board-family mismatch, not an
SD-card-format problem.

The authoritative source for this board was the Waveshare ESP32-S3-GEEK
`07_SD_Test` example. It uses an explicit HSPI object and the following map:

| TF signal | GPIO |
|---|---:|
| CS | 34 |
| SCK | 36 |
| MISO | 37 |
| MOSI | 35 |

The working initialization is:

```cpp
SPIClass sdSpi(HSPI);
sdSpi.begin(36, 37, 35, 34);
SD.begin(34, sdSpi, 20000000);
```

After applying that exact mapping and protocol, the board reported:

```text
LCD=OK SD=OK GPS=ABSENT IMU=ABSENT BARO=ABSENT
```

while simultaneously receiving repeated `G5_PACKET` records. This is the
pin/protocol combination to use for future ESP32-S3-GEEK projects. Do not
copy the T-Dongle-S3 SD/MMC map into this board template, and do not assume
GPIO4 is the GEEK TF chip-select.

Reference sources checked during diagnosis:

- [Waveshare ESP32-S3-GEEK wiki](https://www.waveshare.com/wiki/ESP32-S3-GEEK)
- Waveshare `07_SD_Test` example, which uses `SPIClass hspi(HSPI)` and
  `hspi.begin(36, 37, 35, 34)`
- [LILYGO T-Dongle-S3 documentation](https://wiki.lilygo.cc/products/t-dongle-series/t-dongle-s3/),
  retained here specifically as the *different* board whose GPIO5/3/7/4
  mapping must not be applied to the GEEK

The shared `FusionSessionLog` now accepts an already-mounted `FS` object, so
the T-Beam can continue using its documented SPI `SD` instance while the
GEEK uses this explicit HSPI `SD` instance without duplicating logger code.

The first implementation used guessed DC, reset, and backlight pins. The
LCD driver could initialize and report `LCD=OK` while the physical screen
remained dark. A vendor example is stronger evidence than an inferred pin map.

## Makefile lessons

### Start from a known-good ESP32-S3 project

The important platform settings are:

```make
BOARD ?= esp32s3
CHIP ?= esp32
CDC_ON_BOOT = 1
BUILD_MEMORY_TYPE = qio_qspi
```

`CDC_ON_BOOT=1` is what makes `Serial` appear through the board's native USB
CDC interface. This matches the working `autotrim`, `espSensorModule`, and
T-Beam Makefiles. Without it, flashing can succeed while application serial
output is absent from `/dev/ttyACM0`.

### Use `LIBS` for library discovery

The initial Makefile manually added library include paths through
`BUILD_EXTRA_FLAGS`. That made the dependency graph opaque and caused a
particularly bad interaction with LovyanGFX. Its `src/internal/limits.h`
could shadow the C++ standard `limits.h`, producing a large cascade of
“template with C linkage” errors while compiling unrelated ESP32 sources.

The final project excludes unrelated global libraries and lists the required
libraries through `LIBS`:

- `esp32jimlib`
- `Arduino_CRC32`
- Adafruit GFX and ST7789
- Adafruit LSM9DS1, BMP280, LIS3MDL, BusIO, and Unified Sensor

LovyanGFX and TFT_eSPI are explicitly excluded. LovyanGFX is not updated or
modified globally; this project uses the smaller Adafruit ST7789 stack instead.

### Avoid adding an entire multi-platform graphics library

Adding all LovyanGFX sources recursively caused duplicate platform objects
and missing-header errors. Adding its library directory caused the header
shadowing problem above. If a future board genuinely requires LovyanGFX,
select only the target platform sources and isolate its include ordering.

### Upload-port precedence

The host environment had `UPLOAD_PORT=/dev/ttyUSB0`, which silently overrode
`make upload PORT=/dev/ttyACM0`. The result was a misleading upload failure:
the tool found an ESP32 device on the wrong port and said it was not an
ESP32-S3. The project now binds:

```make
UPLOAD_PORT := $(PORT)
MONITOR_PORT := $(PORT)
```

That makes the command-line `PORT` selection authoritative for this project.

## Serial debugging

The board enumerates as an Espressif USB JTAG/serial device at
`/dev/ttyACM0`. A reliable capture sequence after upload is:

```sh
python - <<'PY'
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=.2)
s.dtr = False
time.sleep(.2)
s.dtr = True
end = time.time() + 5
while time.time() < end:
    data = s.read(4096)
    if data:
        print(data.decode(errors='replace'), end='')
PY
```

Expected output includes the boot banner, hardware status, and one
`GEEK TEST` packet per second. The test deliberately reports missing external
hardware as `ABSENT`; no GPS or IMU is required for serial or display bring-up.

## Flash and PSRAM caveat

The board identifies at runtime as having 16 MB flash and 2 MB PSRAM. The
generic `esp32s3` board definition used by makeEspArduino still reports a
4 MB flash configuration in its build summary. Do not rely on the complete
flash capacity until the board definition/partition configuration is made
explicit.

The board's PSRAM mode also needs verification. An OPI setting produced a
PSRAM initialization error. The final Makefile uses `qio_qspi`, which matches
the safer ESP32-S3 configuration, but the current runtime still reports
`PSRAM=NO`. This is a configuration issue to resolve separately from the
successful LCD and serial bring-up.

## External Qwiic hardware

The BerryIMUv3 and future GPS are optional. I2C has a 20 ms timeout, and the
firmware continues operating when they are disconnected:

- LSM9DS1: 0x6B accelerometer/gyro, 0x1C magnetometer
- BMP280: 0x76 or 0x77
- Future GPS: currently reported as `GPS=ABSENT`

This separation is important: a hardware-vetting sketch should test the board
itself even when add-on sensors have not arrived.

## Working commands

```sh
arduino-cli lib install \
  'Adafruit GFX Library' \
  'Adafruit ST7735 and ST7789 Library' \
  'Adafruit LSM9DS1 Library' \
  'Adafruit BMP280 Library' \
  'Adafruit Unified Sensor' \
  'Adafruit BusIO' \
  'Adafruit LIS3MDL'

make
make upload PORT=/dev/ttyACM0
make cat PORT=/dev/ttyACM0
```

The project was successfully built and flashed with Make, with verified
serial output and a visually working LCD after the vendor pin map was applied.
