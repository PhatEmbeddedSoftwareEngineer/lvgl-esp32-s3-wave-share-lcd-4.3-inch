#ifndef DATA_DEVICE_H
#define DATA_DEVICE_H

#define DATA_PORT_BAUDRATE     115200
#define DATA_PORT_TIMEOUT       100
#include "../Utils/crc.hpp"
#include "../Utils/variable.hpp"

class DataDevice
{
private:
    HardwareSerial *port;
    CRCClass *crcFunc = CRCClass::getInstance();
    void clearWriteError();
    void send(uint8_t *arr, uint8_t length);
    void receive(volatile uint8_t *ouput, uint8_t length, bool *receive);
    uint8_t *receiveFr;

    struct SEND_FRAME
    {
        uint8_t main[DISPLAY_FRAME]; // id command length d1 d2 d3 d4 crcH crcL
    }sendFr;
    bool    lv_receive = true;
    uint8_t lv_count_send = 0;
    uint8_t lv_step_send = 0;
    unsigned long lv_time_check = 0;
public:
    DataDevice(HardwareSerial &port);
    ~DataDevice();

    bool init();
    uint8_t bms_volume = 0;
    void update(        BMS_FRAME *_bmsFr,
                        DCDC_FRAME *_dcdcFr,
                        BATTERY_FRAME *_batteyrFr,
                        FAN_FRAME *_fanFr,
                        ACTUATOR_FRAME *_actuatorFr,
                        HardwareSerial &debugPort
                );
};

#endif // DATA_DEVICE_H