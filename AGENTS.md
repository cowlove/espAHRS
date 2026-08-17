# espAHRS

Flight AHRS and sensor-logging project targeting the Amazon ESP32-S3 Geek
board (ASIN B0CR6FV3QC). The project was renamed from
`esp32-s3-geek-device-test` to `espAHRS` after hardware investigation was
completed.
It tests the onboard 1.14-inch 240x135 IPS LCD, microSD/TF slot, ESP32
resources, and the `ReliableStreamESPNow("GEEK")` broadcast channel at 1 Hz.

An external BerryIMUv3 can be connected to the I2C header. The firmware leaves
its secondary IMU unprobed; only the primary onboard IMU participates in live
AHRS and health reporting. The external board remains available for separate
bench experiments, but is not an onboard peripheral.

All external sensors are optional. I2C has a 20 ms transaction timeout and
missing devices are reported as `ABSENT`; the display and 1 Hz diagnostics
continue normally. The future Qwiic GPS path currently reports `GPS=ABSENT`
until its driver is added.

The listing documents ESP32-S3R2, 2 MB PSRAM, 16 MB flash, USB-A, UART,
GPIO, I2C, and Wi-Fi/Bluetooth. It does not document onboard GNSS, IMU,
LoRa, battery charging, or a PMU. The LCD pin map is verified in
`BUILD_NOTES.md`; the SD pin remains a hardware-test assumption.

Build with `make`; monitor with `make cat`. Both default to the Geek board's
stable USB device ID.
Arduino CLI dependencies and exact install commands are documented in
`README.md`. The detailed build/configuration investigation is in
`BUILD_NOTES.md`. In summary, download Adafruit GFX Library, Adafruit ST7789,
Adafruit BMP280 Library, Adafruit LIS3MDL, Adafruit BusIO, and Adafruit
Unified Sensor. The local `esp32jimlib` and `Arduino_CRC32` libraries are
shared dependencies and should not be downloaded again.

## Active hardware target

The ESP32-S3 Geek is the only active hardware target. The T-Beam Supreme
target is abandoned: do not build, flash, test, or make compatibility changes
for it unless Jim explicitly reverses this decision. `make` and `make upload`
default to the Geek board and its stable USB device ID.

## IMU drift logging

`GYRO_DRIFT_START` / `GYRO_DRIFT_STOP` are the supported serial commands for
long stationary drift captures. The command reports ten-second raw gyro means
for every available IMU; use `capture_gyro_drift.py` to save them as CSV.
