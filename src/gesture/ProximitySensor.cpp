#include "ProximitySensor.h"
#include <M5Unified.h>

#define LTR553_ADDR 0x23

#define LTR553_ALS_CONTR      0x80
#define LTR553_PS_CONTR       0x81
#define LTR553_PS_LED         0x82
#define LTR553_PS_N_PULSES    0x83
#define LTR553_PS_MEAS_RATE   0x84
#define LTR553_ALS_MEAS_RATE  0x85
#define LTR553_PART_ID        0x86
#define LTR553_PS_DATA_0      0x8D
#define LTR553_PS_DATA_1      0x8E

#define I2C_FREQ_HZ           100000

ProximitySensor::ProximitySensor() :
    m_initialized(false),
    m_debug(false),
    m_lastProximityValue(0),
    m_failureCount(0)
{
}

ProximitySensor::~ProximitySensor() {

}

bool ProximitySensor::initialize() {
    if (m_initialized) {
        return true;
    }

    for (int attempt = 0; attempt < 3; attempt++) {

        M5.In_I2C.release();
        delay(50);

        uint8_t deviceId = M5.In_I2C.readRegister8(LTR553_ADDR, LTR553_PART_ID, I2C_FREQ_HZ);

        if (deviceId != 0x92) {
            if (m_debug) {
                Serial.printf("尝试 %d: 设备ID不匹配或读取失败, 读取值: 0x%02X (期望: 0x92)\n", attempt + 1, deviceId);
            }
            delay(100);
            continue;
        }

        if (m_debug) {
            Serial.printf("尝试 %d: 设备ID确认: 0x%02X\n", attempt + 1, deviceId);
        }

        bool configSuccess = true;

        configSuccess &= M5.In_I2C.writeRegister8(LTR553_ADDR, LTR553_ALS_CONTR, 0x00, I2C_FREQ_HZ);

        configSuccess &= M5.In_I2C.writeRegister8(LTR553_ADDR, LTR553_PS_CONTR, 0x03, I2C_FREQ_HZ);

        configSuccess &= M5.In_I2C.writeRegister8(LTR553_ADDR, LTR553_PS_LED, 0x7F, I2C_FREQ_HZ);

        configSuccess &= M5.In_I2C.writeRegister8(LTR553_ADDR, LTR553_PS_N_PULSES, 0x08, I2C_FREQ_HZ);

        configSuccess &= M5.In_I2C.writeRegister8(LTR553_ADDR, LTR553_PS_MEAS_RATE, 0x00, I2C_FREQ_HZ);

        if (!configSuccess) {
            if (m_debug) {
                Serial.printf("尝试 %d: 配置传感器寄存器失败\n", attempt + 1);
            }
            delay(100);
            continue;
        }

        uint8_t psContr = M5.In_I2C.readRegister8(LTR553_ADDR, LTR553_PS_CONTR, I2C_FREQ_HZ);
        if (psContr != 0x03) {
            if (m_debug) {
                Serial.printf("尝试 %d: 配置验证失败, PS_CONTR=0x%02X\n", attempt + 1, psContr);
            }
            delay(100);
            continue;
        }

        if (m_debug) {
            Serial.printf("尝试 %d: LTR-553接近传感器初始化成功\n", attempt + 1);
        }

        m_initialized = true;
        m_failureCount = 0;
        return true;
    }

    if (m_debug) {
        Serial.println("LTR-553接近传感器初始化失败，将使用默认安全值");
    }
    return false;
}

bool ProximitySensor::writeInternalI2CReg(uint8_t reg, uint8_t value) {
    return M5.In_I2C.writeRegister8(LTR553_ADDR, reg, value, I2C_FREQ_HZ);
}

bool ProximitySensor::readInternalI2CReg(uint8_t reg, uint8_t *value) {
    if (value == nullptr) return false;

    *value = M5.In_I2C.readRegister8(LTR553_ADDR, reg, I2C_FREQ_HZ);
    return true;
}

uint16_t ProximitySensor::readProximity() {
    if (!m_initialized) {
        if (initialize()) {
            delay(50);
        } else {
            return 0;
        }
    }

    uint16_t value = 0;
    bool readSuccess = false;

    for (int attempt = 0; attempt < 3; attempt++) {
        uint8_t low = 0, high = 0;

        low = M5.In_I2C.readRegister8(LTR553_ADDR, LTR553_PS_DATA_0, I2C_FREQ_HZ);
        high = M5.In_I2C.readRegister8(LTR553_ADDR, LTR553_PS_DATA_1, I2C_FREQ_HZ);

        if (low != 0 || high != 0) {
            value = (high << 8) | low;
            readSuccess = true;
            break;
        } else {
            if (m_debug) {
                Serial.printf("尝试 %d: 读取接近传感器数据失败或数值为0\n", attempt + 1);
            }
            delay(10);
        }
    }

    if (readSuccess) {
        m_lastProximityValue = value;
    } else {

        if (m_debug) {
            Serial.println("接近传感器读取失败，使用上次的安全值");
        }
        return m_lastProximityValue;
    }

    return m_lastProximityValue;
}

bool ProximitySensor::isObjectDetected(uint16_t threshold) {
    uint16_t proximityValue = readProximity();

    bool detected = proximityValue > threshold;

    if (detected && m_debug) {
        Serial.printf("检测到物体! 距离值: %d (阈值: %d)\n", proximityValue, threshold);
    }

    return detected;
}

void ProximitySensor::writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(LTR553_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t ProximitySensor::readReg(uint8_t reg) {
    Wire.beginTransmission(LTR553_ADDR);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)LTR553_ADDR, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

uint16_t ProximitySensor::readReg16(uint8_t reg) {
    uint16_t value = 0;
    uint8_t low = readReg(reg);
    uint8_t high = readReg(reg + 1);

    value = (high << 8) | low;
    return value;
}

uint16_t ProximitySensor::readReg16Safe(uint8_t reg, bool &success) {
    success = false;

    Wire.beginTransmission(LTR553_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) {
        if (m_debug) {
            Serial.println("I2C通信错误: 无法写入寄存器地址");
        }
        return 0;
    }

    if (Wire.requestFrom((uint8_t)LTR553_ADDR, (uint8_t)2) != 2) {
        if (m_debug) {
            Serial.println("I2C通信错误: 请求数据失败");
        }
        return 0;
    }

    uint8_t low = Wire.read();
    uint8_t high = Wire.read();
    success = true;

    return (high << 8) | low;
}
