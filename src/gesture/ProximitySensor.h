#ifndef PROXIMITY_SENSOR_H
#define PROXIMITY_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <M5Unified.h>

class ProximitySensor {
public:
    ProximitySensor();
    ~ProximitySensor();

    bool initialize();

    uint16_t readProximity();

    bool isObjectDetected(uint16_t threshold = 300);

    uint16_t getLastProximityValue() const { return m_lastProximityValue; }

    void setDebug(bool debug) { m_debug = debug; }

private:

    bool writeInternalI2CReg(uint8_t reg, uint8_t value);
    bool readInternalI2CReg(uint8_t reg, uint8_t *value);

    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
    uint16_t readReg16(uint8_t reg);
    uint16_t readReg16Safe(uint8_t reg, bool &success);

    static const uint8_t LTR553_ADDR = 0x23;

    static const uint8_t LTR553_ALS_CONTR = 0x80;
    static const uint8_t LTR553_PS_CONTR = 0x81;
    static const uint8_t LTR553_PS_LED = 0x82;
    static const uint8_t LTR553_PS_N_PULSES = 0x83;
    static const uint8_t LTR553_PS_MEAS_RATE = 0x84;
    static const uint8_t LTR553_PS_DATA_0 = 0x8D;
    static const uint8_t LTR553_PS_DATA_1 = 0x8E;

    bool m_initialized;
    bool m_debug;
    uint16_t m_lastProximityValue;
    int m_failureCount;
};

#endif
