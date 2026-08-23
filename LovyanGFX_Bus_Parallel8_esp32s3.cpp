// Unique source basename for makeEspArduino.  LovyanGFX also contains an
// ESP32 implementation named Bus_Parallel8.cpp; makeEspArduino maps source
// basenames directly to object names, so compiling the S3 implementation via
// this wrapper avoids that collision.
#include "/home/jim/Arduino/libraries/LovyanGFX/src/lgfx/v1/platforms/esp32s3/Bus_Parallel8.cpp"
