#ifndef TOF050F_SENSOR_H
#define TOF050F_SENSOR_H

#include <Arduino.h>
#include <HardwareSerial.h>

#define TOF050F_RX_PIN 1
#define TOF050F_TX_PIN 2
#define TOF050F_BAUD_RATE 115200

#define REG_RD_ADDR 0x0010
#define REG_WR_ADDR 0x0005

typedef signed char     int8;
typedef unsigned char   uint8;
typedef signed short    int16;
typedef unsigned short  uint16;
typedef signed long     int32;
typedef unsigned long   uint32;

class TOF050FSensor {
private:
    HardwareSerial* tofSerial;
    uint8_t rxBuffer[80];
    uint8_t txBuffer[80];
    uint8_t rxRdIndex = 0;
    uint8_t rxWrIndex = 0;
    uint32_t lastRxTimestamp = 0;
    uint8_t frameTimeInvt = 4;
    uint32_t timCount = 0;

    static const uint8_t crc16mTableHi[];
    static const uint8_t crc16mTableLo[];

    uint16_t crc16_calculate_modbus(uint8_t *p_data, uint16_t len);
    uint16_t uart_read_isr(uint8_t channel, uint8_t *p_buff);
    uint8_t uart_tpm_rx_isr(uint8_t channel, uint8_t data);
    void uart_tpm_main(void);
    uint8_t uart_tpm_tx_data(uint8_t channel, uint8_t *p_data, uint16_t len);
    void modbus_recv_data(uint8_t *p_data, uint16_t len);
    uint32_t tim_get_count(void);
    uint8_t tim_check_timeout(uint32_t start, uint32_t now, uint32_t invt);

public:
    TOF050FSensor();
    ~TOF050FSensor();

    bool begin();

    int16_t readDistance();

    void setContinuousOutput(uint16_t interval_ms);

    void stopContinuousOutput();

    void update();

    bool hasNewData();

    int16_t getLastDistance();

    bool isValid();

private:
    int16_t lastDistance = -1;
    bool sensorValid = false;
    bool newDataAvailable = false;
};

#endif
