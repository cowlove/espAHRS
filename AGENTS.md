# ESP32-S3 Geek device test

Bring-up sketch for the Amazon ESP32-S3 Geek board (ASIN B0CR6FV3QC).
It tests the onboard 1.14-inch 240x135 IPS LCD, microSD/TF slot, ESP32
resources, and the `ReliableStreamESPNow("GEEK")` broadcast channel at 1 Hz.

The listing documents ESP32-S3R2, 2 MB PSRAM, 16 MB flash, USB-A, UART,
GPIO, I2C, and Wi-Fi/Bluetooth. It does not document onboard GNSS, IMU,
LoRa, battery charging, or a PMU. LCD/SD pin constants are provisional and
must be checked against the board revision before upload.

Build with `make`; monitor with `make cat PORT=/dev/ttyACM0`.
Arduino CLI dependencies: LovyanGFX and the existing local esp32jimlib and
Arduino_CRC32 libraries.
