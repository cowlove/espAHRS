#pragma once

#ifndef CSIM
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// Board-independent u-blox-compatible UART/UBX initialization.
// The caller owns the serial port pins and consumes gnss for its application.
class SharedUbloxGPS {
public:
    SFE_UBLOX_GNSS gnss;
    HardwareSerial *serial = nullptr;
    int rxPin = -1;
    int txPin = -1;
    uint32_t baud = 0;
    bool gpsGood = false;

    bool tryBaud(uint32_t candidate) {
        if (!serial) return false;
        serial->begin(candidate, SERIAL_8N1, rxPin, txPin);
        while (serial->available()) serial->read();
        delay(100);
        return gnss.begin(*serial);
    }

    bool begin(HardwareSerial &port, int receiverRx, int receiverTx,
               const uint32_t *candidates, size_t candidateCount) {
        serial = &port;
        rxPin = receiverRx;
        txPin = receiverTx;
        gpsGood = false;
        baud = 0;
        for (size_t i = 0; i < candidateCount; ++i) {
            if (tryBaud(candidates[i])) {
                baud = candidates[i];
                gpsGood = true;
                break;
            }
        }
        return gpsGood;
    }

    bool configure(uint32_t targetBaud, uint8_t navFrequencyHz,
                   uint16_t timeoutMs = 100) {
        if (!gpsGood || !serial) return false;
        bool ubxOk = gnss.setUART1Output(COM_TYPE_UBX, timeoutMs);
        bool navOk = gnss.setNavigationFrequency(navFrequencyHz, timeoutMs);
        bool pvtOk = gnss.setAutoPVT(true, true, timeoutMs);
        if (!ubxOk || !navOk || !pvtOk) return false;

        if (baud != targetBaud) {
            gnss.setSerialRate(targetBaud, COM_PORT_UART1, timeoutMs);
            serial->begin(targetBaud, SERIAL_8N1, rxPin, txPin);
            baud = targetBaud;
        }
        return gnss.saveConfiguration();
    }

    bool check(uint32_t timeoutMs = 10) {
        return gpsGood && gnss.getPVT(timeoutMs);
    }
};
#endif

