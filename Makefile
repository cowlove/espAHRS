DEVICE ?= tdisplay-s3
.DEFAULT_GOAL := all
PORT ?= /dev/ttyACM0
V := 1
VERBOSE=1
UPLOAD_PORT := $(PORT)
ARDUINO_CLI ?= $(HOME)/bin/arduino-cli
PARTITION_SCHEME ?= huge_app
FQBN ?= esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PartitionScheme=$(PARTITION_SCHEME)
BUILD_DIR ?= build/$(DEVICE)-$(PARTITION_SCHEME)
ALIBS := $(HOME)/Arduino/libraries
GIT_VERSION := $(shell git describe --abbrev=8 --dirty --always --tags 2>/dev/null || echo local)

# Keep the library roots explicit.  LovyanGFX is only needed by the T-Display
# target; excluding it from the default build avoids its header-name collision
# with the C++ standard library.
LIBRARY_DIRS := \
	$(ALIBS)/esp32jimlib \
	$(ALIBS)/Arduino_CRC32 \
	$(ALIBS)/Adafruit_GFX_Library \
	$(ALIBS)/Adafruit_ST7735_and_ST7789_Library \
	$(ALIBS)/Adafruit_LSM9DS1_Library \
	$(ALIBS)/SensorLib \
	$(ALIBS)/XPowersLib \
	$(ALIBS)/U8g2 \
	$(ALIBS)/Adafruit_BMP280_Library \
	$(ALIBS)/Adafruit_LIS3MDL \
	$(ALIBS)/Adafruit_BMP3XX_Library \
	$(ALIBS)/Adafruit_BME280_Library \
	$(ALIBS)/Adafruit_BusIO \
	$(ALIBS)/Adafruit_Unified_Sensor \
	$(ALIBS)/SparkFun_Qwiic_6DoF_LSM6DSO_Arduino_Library \
	$(ALIBS)/SparkFun_MicroPressure_Library

ifeq ($(DEVICE),tdisplay-s3)
BUILD_FLAGS := -DESPAHRS_TDISPLAY_S3
LIBRARY_DIRS += $(ALIBS)/LovyanGFX
else
BUILD_FLAGS :=
endif

BUILD_FLAGS += -DESP_PLATFORM -DCONFIG_IDF_TARGET_ESP32S3 -DBOARD_HAS_PSRAM -DGIT_VERSION=\"$(GIT_VERSION)\"
LIBRARY_ARGS := $(foreach dir,$(LIBRARY_DIRS),--libraries $(dir))

.PHONY: cli-compile cli-upload
cli-compile:
	time $(ARDUINO_CLI) compile --fqbn $(FQBN) $(LIBRARY_ARGS) \
		--build-path $(BUILD_DIR) -v \
		--build-property "compiler.cpp.extra_flags=$(BUILD_FLAGS)" .

cli-upload: cli-compile
	$(ARDUINO_CLI) upload --fqbn $(FQBN) --input-dir $(BUILD_DIR) --port $(UPLOAD_PORT)

.PHONY: cat legacy-all
legacy-all: all

cat:
	while sleep .01; do if [ -c ${PORT} ]; then stty -F ${PORT} -echo raw 115200 && cat ${PORT}; fi; done | tee ./cat.`basename ${PORT}`.out

.PHONY: replay flight-results dipahrs-test ahrs-kinematics-test log-metadata-test
replay:
	$(CXX) -std=c++17 -O2 -I. host-tests/replay.cpp AircraftAHRS.cpp -o replay

flight-results: replay
	@mkdir -p flight-data-primary/results
	@for log in flight-data-primary/flight-data-*.bin; do \
		base=$$(basename $$log .bin); \
		./replay $$log --device-mac 247C \
			--roll-csv flight-data-primary/results/$${base}-roll.csv \
			--pitch-csv flight-data-primary/results/$${base}-pitch.csv >/dev/null; \
	done

dipahrs-test:
	$(CXX) -std=c++11 -O2 -I. host-tests/dipahrs_test.cpp -o /tmp/esp32-s3-geek-dipahrs-test
	/tmp/esp32-s3-geek-dipahrs-test

ahrs-kinematics-test:
	$(CXX) -std=c++17 -O2 -I. host-tests/aircraft_ahrs_kinematics_test.cpp AircraftAHRS.cpp -o /tmp/esp32-s3-geek-ahrs-kinematics-test
	/tmp/esp32-s3-geek-ahrs-kinematics-test

log-metadata-test: replay
	$(CXX) -std=c++17 -O2 -I. host-tests/fusion_log_metadata_test.cpp -o /tmp/espahrs-log-metadata-test
	/tmp/espahrs-log-metadata-test
	./replay /tmp/espahrs-metadata-v2.bin >/dev/null
	! ./replay /tmp/espahrs-metadata-v2-stale.bin >/dev/null 2>&1

# Default firmware builds use makeEspArduino.  The CLI workflow remains
# available explicitly as `make cli-compile` / `make cli-upload`.
include Makefile.makeEspArduino
