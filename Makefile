BOARD ?= esp32s3
PORT ?= /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_28:37:2F:F8:24:7C-if00
UPLOAD_PORT := $(PORT)
MONITOR_PORT := $(PORT)
CHIP ?= esp32
DEVICE ?= geek
ALIBS = ${HOME}/Arduino/libraries
GIT_VERSION := "$(shell git describe --abbrev=8 --dirty --always --tags 2>/dev/null || echo local)"

CDC_ON_BOOT = 1
# The ESP32-S3 Geek is the active and default hardware target. The former
# T-Beam target is abandoned and retained only as historical source code.
ifeq ($(DEVICE),geek)
BUILD_MEMORY_TYPE = qio_qspi
BUILD_EXTRA_FLAGS += -DBOARD_HAS_PSRAM
else
BUILD_MEMORY_TYPE = qio_qspi
BUILD_EXTRA_FLAGS += -DBOARD_HAS_PSRAM
endif
BUILD_EXTRA_FLAGS += -DGIT_VERSION=\"$(GIT_VERSION)\"
EXCLUDE_DIRS = ${ALIBS}/lvgl|${ALIBS}/TFT_eSPI|${ALIBS}/LovyanGFX|${ALIBS}/jimlib|${ALIBS}/esp32csim
LIBS += ${ALIBS}/esp32jimlib ${ALIBS}/Arduino_CRC32
LIBS += ${ALIBS}/Adafruit_GFX_Library ${ALIBS}/Adafruit_ST7735_and_ST7789_Library
LIBS += ${ALIBS}/Adafruit_LSM9DS1_Library
LIBS += ${ALIBS}/SparkFun_ICM-20948_ArduinoLibrary/src/ICM_20948.cpp
LIBS += ${ALIBS}/SensorLib
LIBS += ${ALIBS}/XPowersLib
LIBS += ${ALIBS}/U8g2
BUILD_EXTRA_FLAGS += -I${ALIBS}/U8g2/src
BUILD_EXTRA_FLAGS += -I${ALIBS}/SparkFun_ICM-20948_ArduinoLibrary/src
LIBS += ${ALIBS}/Adafruit_BMP280_Library ${ALIBS}/Adafruit_LIS3MDL
LIBS += ${ALIBS}/Adafruit_BMP3XX_Library
LIBS += ${ALIBS}/Adafruit_BME280_Library
LIBS += ${ALIBS}/SparkFun_u-blox_GNSS_Arduino_Library/src/SparkFun_u-blox_GNSS_Arduino_Library.cpp
BUILD_EXTRA_FLAGS += -I${ALIBS}/SparkFun_u-blox_GNSS_Arduino_Library/src
LIBS += ${ALIBS}/Adafruit_BusIO ${ALIBS}/Adafruit_Unified_Sensor
LIBS += ${ALIBS}/esp32jimlib/src/espNowMux.cpp
LIBS += ${ALIBS}/esp32jimlib/src/jimlib.cpp
LIBS += ${ALIBS}/esp32jimlib/src/simulatedFailures.cpp
LIBS += ${ALIBS}/Arduino_CRC32/src/Arduino_CRC32.cpp ${ALIBS}/Arduino_CRC32/src/crc.cpp

include ${ALIBS}/makeEspArduino/makeEspArduino.mk

.PHONY: cat
cat:
	while sleep .01; do if [ -c ${PORT} ]; then stty -F ${PORT} -echo raw 115200 && cat ${PORT}; fi; done | tee ./cat.`basename ${PORT}`.out

.PHONY: replay flight-results dipahrs-test ahrs-kinematics-test
replay:
	$(CXX) -std=c++17 -O2 -I. replay.cpp AircraftAHRS.cpp -o replay

flight-results: replay
	@mkdir -p flight-data-primary/results
	@for log in flight-data-primary/flight-data-*.bin; do \
		base=$$(basename $$log .bin); \
		./replay $$log --roll-csv flight-data-primary/results/$${base}-roll.csv \
			--pitch-csv flight-data-primary/results/$${base}-pitch.csv >/dev/null; \
	done

dipahrs-test:
	$(CXX) -std=c++11 -O2 -I. dipahrs_test.cpp -o /tmp/esp32-s3-geek-dipahrs-test
	/tmp/esp32-s3-geek-dipahrs-test

ahrs-kinematics-test:
	$(CXX) -std=c++17 -O2 -I. aircraft_ahrs_kinematics_test.cpp AircraftAHRS.cpp -o /tmp/esp32-s3-geek-ahrs-kinematics-test
	/tmp/esp32-s3-geek-ahrs-kinematics-test
