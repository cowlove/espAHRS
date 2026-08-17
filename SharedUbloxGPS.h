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
        size_t rawBytes = 0;
        uint32_t sampleUntil = millis() + 250;
        while ((int32_t)(millis() - sampleUntil) < 0) {
            while (serial->available()) { serial->read(); ++rawBytes; }
            delay(2);
        }
        gnss.disableDebugging();
        static const uint8_t cfgPrtPoll[] = {0xB5, 0x62, 0x06, 0x00,
                                               0x01, 0x00, 0x01, 0x08, 0x21};
        serial->write(cfgPrtPoll, sizeof(cfgPrtPoll));
        serial->flush();
        uint8_t response[96]; size_t responseLength = 0;
        uint32_t responseUntil = millis() + 500;
        while ((int32_t)(millis() - responseUntil) < 0) {
            while (serial->available() && responseLength < sizeof(response))
                response[responseLength++] = (uint8_t)serial->read();
            delay(2);
        }
        Serial.printf("GNSS raw CFG-PRT poll response bytes=%u:",
                      (unsigned)responseLength);
        for (size_t i = 0; i < responseLength; ++i)
            Serial.printf(" %02X", response[i]);
        Serial.println();
        bool ok = gnss.begin(*serial);
        Serial.printf("GNSS probe baud=%lu raw_bytes=%u ubx=%s\n",
                      (unsigned long)candidate, (unsigned)rawBytes,
                      ok ? "OK" : "FAIL");
        return ok;
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
                   uint16_t timeoutMs = 1000) {
        if (!gpsGood || !serial) return false;
        Serial.printf("GNSS config begin baud=%lu target_baud=%lu nav_hz=%u timeout_ms=%u rx=%d tx=%d\n",
                      (unsigned long)baud, (unsigned long)targetBaud,
                      (unsigned)navFrequencyHz, (unsigned)timeoutMs, rxPin, txPin);
        bool ubxOk = gnss.setUART1Output(COM_TYPE_UBX, timeoutMs);
        Serial.printf("GNSS config setUART1Output=%s\n", ubxOk ? "OK" : "FAIL");
        bool navOk = gnss.setNavigationFrequency(navFrequencyHz, timeoutMs);
        Serial.printf("GNSS config setNavigationFrequency=%s\n", navOk ? "OK" : "FAIL");
        bool pvtOk = gnss.setAutoPVT(true, true, timeoutMs);
        Serial.printf("GNSS config setAutoPVT=%s\n", pvtOk ? "OK" : "FAIL");
        if (!ubxOk || !navOk || !pvtOk) {
            Serial.println("GNSS config failed before baud/save step");
            return false;
        }

        if (baud != targetBaud) {
            gnss.setSerialRate(targetBaud, COM_PORT_UART1, timeoutMs);
            Serial.println("GNSS config setSerialRate=ISSUED");
            serial->begin(targetBaud, SERIAL_8N1, rxPin, txPin);
            baud = targetBaud;
        }
        bool saveOk = gnss.saveConfiguration();
        Serial.printf("GNSS config saveConfiguration=%s final_baud=%lu\n",
                      saveOk ? "OK" : "FAIL", (unsigned long)baud);
        return saveOk;
    }

    bool check(uint32_t timeoutMs = 10) {
        return gpsGood && gnss.getPVT(timeoutMs);
    }
};
#endif
