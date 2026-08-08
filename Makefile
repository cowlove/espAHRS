BOARD ?= esp32s3
PORT ?= /dev/ttyACM0
ALIBS = ${HOME}/Arduino/libraries
GIT_VERSION := "$(shell git describe --abbrev=8 --dirty --always --tags 2>/dev/null || echo local)"

BUILD_EXTRA_FLAGS += -DGIT_VERSION=\"$(GIT_VERSION)\"
BUILD_EXTRA_FLAGS += -I${ALIBS}/esp32jimlib/src -I${ALIBS}/Arduino_CRC32/src
BUILD_EXTRA_FLAGS += -I${ESP_ROOT}/libraries/ArduinoOTA/src -I${ESP_ROOT}/libraries/WiFi/src
EXCLUDE_DIRS = ${ALIBS}/lvgl|${ALIBS}/LovyanGFX|${ALIBS}/jimlib|${ALIBS}/esp32csim
LIBS += ${ALIBS}/esp32jimlib/src/espNowMux.cpp
LIBS += ${ALIBS}/esp32jimlib/src/jimlib.cpp
LIBS += ${ALIBS}/esp32jimlib/src/simulatedFailures.cpp
LIBS += ${ALIBS}/Arduino_CRC32/src/Arduino_CRC32.cpp ${ALIBS}/Arduino_CRC32/src/crc.cpp

include ${ALIBS}/makeEspArduino/makeEspArduino.mk

.PHONY: cat
cat:
	while sleep .01; do if [ -c ${PORT} ]; then stty -F ${PORT} -echo raw 115200 && cat ${PORT}; fi; done | tee ./cat.`basename ${PORT}`.out
