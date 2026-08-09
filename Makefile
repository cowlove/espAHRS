BOARD ?= esp32s3
PORT ?= /dev/ttyACM0
UPLOAD_PORT := $(PORT)
MONITOR_PORT := $(PORT)
CHIP ?= esp32
ALIBS = ${HOME}/Arduino/libraries
FUSION_SHARED_DIR ?= ../tbeam-supreme-device-test
GIT_VERSION := "$(shell git describe --abbrev=8 --dirty --always --tags 2>/dev/null || echo local)"

CDC_ON_BOOT = 1
# The board has QSPI PSRAM; OPI mode caused PSRAM initialization failure.
BUILD_MEMORY_TYPE = qio_qspi
BUILD_EXTRA_FLAGS += -DGIT_VERSION=\"$(GIT_VERSION)\"
EXCLUDE_DIRS = ${ALIBS}/lvgl|${ALIBS}/TFT_eSPI|${ALIBS}/LovyanGFX|${ALIBS}/jimlib|${ALIBS}/esp32csim|${FUSION_SHARED_DIR}
LIBS += ${ALIBS}/esp32jimlib ${ALIBS}/Arduino_CRC32
LIBS += ${ALIBS}/Adafruit_GFX_Library ${ALIBS}/Adafruit_ST7735_and_ST7789_Library
LIBS += ${ALIBS}/Adafruit_LSM9DS1_Library
LIBS += ${ALIBS}/Adafruit_BMP280_Library ${ALIBS}/Adafruit_LIS3MDL
LIBS += ${ALIBS}/SparkFun_u-blox_GNSS_Arduino_Library
LIBS += ${ALIBS}/Adafruit_BusIO ${ALIBS}/Adafruit_Unified_Sensor
LIBS += ${ALIBS}/esp32jimlib/src/espNowMux.cpp
LIBS += ${ALIBS}/esp32jimlib/src/jimlib.cpp
LIBS += ${ALIBS}/esp32jimlib/src/simulatedFailures.cpp
LIBS += ${ALIBS}/Arduino_CRC32/src/Arduino_CRC32.cpp ${ALIBS}/Arduino_CRC32/src/crc.cpp
LIBS += ${FUSION_SHARED_DIR}/AircraftAHRS.cpp
BUILD_EXTRA_FLAGS += -I${FUSION_SHARED_DIR}

include ${ALIBS}/makeEspArduino/makeEspArduino.mk

.PHONY: cat
cat:
	while sleep .01; do if [ -c ${PORT} ]; then stty -F ${PORT} -echo raw 115200 && cat ${PORT}; fi; done | tee ./cat.`basename ${PORT}`.out
