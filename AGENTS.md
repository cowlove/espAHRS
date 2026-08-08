# ESP32-S3 Geek device test

Bring-up sketch for the Amazon ESP32-S3 Geek board (ASIN B0CR6FV3QC).
It tests the onboard 1.14-inch 240x135 IPS LCD, microSD/TF slot, ESP32
resources, and the `ReliableStreamESPNow("GEEK")` broadcast channel at 1 Hz.

An external BerryIMUv3 can be connected to the I2C header. The sketch probes
its LSM9DS1 accelerometer/gyro/magnetometer at 0x6B/0x1C and BMP280 at 0x76
or 0x77, then includes sensor health and pressure in the 1 Hz report. The
board is external; it is not an onboard peripheral.

All external sensors are optional. I2C has a 20 ms transaction timeout and
missing devices are reported as `ABSENT`; the display and 1 Hz diagnostics
continue normally. The future Qwiic GPS path currently reports `GPS=ABSENT`
until its driver is added.

The listing documents ESP32-S3R2, 2 MB PSRAM, 16 MB flash, USB-A, UART,
GPIO, I2C, and Wi-Fi/Bluetooth. It does not document onboard GNSS, IMU,
LoRa, battery charging, or a PMU. LCD/SD pin constants are provisional and
must be checked against the board revision before upload.

Build with `make`; monitor with `make cat PORT=/dev/ttyACM0`.
Arduino CLI dependencies and exact install commands are documented in
`README.md`. In summary, download LovyanGFX, Adafruit LSM9DS1 Library,
Adafruit BMP280 Library, Adafruit LIS3MDL, Adafruit BusIO, and Adafruit
Unified Sensor. The local `esp32jimlib` and `Arduino_CRC32` libraries are
shared dependencies and should not be downloaded again.
