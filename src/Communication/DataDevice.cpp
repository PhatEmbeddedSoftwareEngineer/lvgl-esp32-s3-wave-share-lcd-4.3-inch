#include "DataDevice.hpp"

DataDevice::DataDevice(HardwareSerial &port)
{
    this->port = &port;
}

DataDevice::~DataDevice(){}

bool DataDevice::init()
{
    // Null check the serial interface
    if (this->port == nullptr) return false;

    // Initialize the serial link to 9600 baud with 8 data bits and no parity bits, per the Daly BMS spec
    this->port->begin(DATA_PORT_BAUDRATE);
    this->port->setTimeout(DATA_PORT_TIMEOUT);
    sendFr.main[0] = DEVICE_ID::DISPLAY_ID;
    lv_time_check = millis();
    return true;
}

void DataDevice::clearWriteError()
{
    while(this->port->available() > 0) char t = this->port->read();
    this->port->clearWriteError();
}

void DataDevice::update(BMS_FRAME *_bmsFr,
                        DCDC_FRAME *_dcdcFr,
                        BATTERY_FRAME *_batteryFr,
                        FAN_FRAME *_fanFr,
                        ACTUATOR_FRAME *_actuatorFr,
                        HardwareSerial &debugPort)
{
    if (lv_receive)
    {
        switch (lv_step_send)
        {
        case 0: // BMS_VOLUME
            sendFr.main[1] = COMMAND_ID::BMS;
            sendFr.main[2] = DATA_LENGTH::BMS_VOLUME_FRAME;
            (*_bmsFr).instance = BMS_ADDRESS::BMS_FIRST;
            if (bms_volume > 0) lv_step_send++;
            break;
        case 1: // BMS_MAIN_FR
            sendFr.main[1] = COMMAND_ID::BMS;
            sendFr.main[2] = DATA_LENGTH::BMS_MAIN_FRAME;
            sendFr.main[3] = (*_bmsFr).instance;
            ((*_bmsFr).instance) ++;
            if ((*_bmsFr).instance == (BMS_ADDRESS::BMS_FIRST + bms_volume)) 
            {
                (*_bmsFr).instance = BMS_ADDRESS::BMS_FIRST;
                lv_step_send++;
            }
            break;
        case 2: // BMS_CELL_FR
            sendFr.main[1] = COMMAND_ID::BMS;
            sendFr.main[2] = DATA_LENGTH::BMS_CELL_FRAME;
            sendFr.main[3] = (*_bmsFr).instance;
            ((*_bmsFr).instance) ++;
            if ((*_bmsFr).instance == (BMS_ADDRESS::BMS_FIRST + bms_volume)) 
            {
                (*_bmsFr).instance = BMS_ADDRESS::BMS_FIRST;
                lv_step_send++;
            }
            break;
        case 3: // BMS_ADD_FR
            sendFr.main[1] = COMMAND_ID::BMS;
            sendFr.main[2] = DATA_LENGTH::BMS_ADDITION_FRAME;
            sendFr.main[3] = (*_bmsFr).instance;
            ((*_bmsFr).instance) ++;
            if ((*_bmsFr).instance == (BMS_ADDRESS::BMS_FIRST + bms_volume)) 
            {
                (*_bmsFr).instance = BMS_ADDRESS::BMS_FIRST;
                lv_step_send++;
            }
            break;
        case 4: // DCDC_MAIN_FRAME
            sendFr.main[1] = COMMAND_ID::DCDC;
            sendFr.main[2] = DATA_LENGTH::DCDC_MAIN_FRAME;
            lv_step_send++;
            break;
        case 5: // BATTERY_MAIN_FRAME
            sendFr.main[1] = COMMAND_ID::BATTERY;
            sendFr.main[2] = DATA_LENGTH::BATTERY_MAIN_FRAME;
            lv_step_send++;
        case 6: // BATTERY_CELL_FRAME
            sendFr.main[1] = COMMAND_ID::BATTERY;
            sendFr.main[2] = DATA_LENGTH::BATTERY_CELL_FRAME;
            lv_step_send++;
            break;
        case 7: // BATTERY_ADDITION_FRAME
            sendFr.main[1] = COMMAND_ID::BATTERY;
            sendFr.main[2] = DATA_LENGTH::BATTERY_ADDITION_FRAME;
            lv_step_send++;
            break;
        case 8: // FAN_MAIN_FRAME
            sendFr.main[1] = COMMAND_ID::FAN;
            sendFr.main[2] = DATA_LENGTH::FAN_MAIN_FRAME;
            lv_step_send++;
            break;
        case 9: // ACTUATOR_MAIN_FRAME
            sendFr.main[1] = COMMAND_ID::ACTUATOR;
            sendFr.main[2] = DATA_LENGTH::ACTUATOR_MAIN_FRAME;
            lv_step_send=0;
            break;
        default:
            break;
        }
        send(sendFr.main, DATA_FRAME);
        lv_receive = false;
        // lv_count_send = 0;
    }

    if ((unsigned long)(millis() - lv_time_check) > 300)
    {
        lv_step_send = 0;
        lv_receive = true;
        lv_time_check = millis();
    }
    

    if (this->port->available())
    {
        if (this->port->peek() == DEVICE_ID::DATA_ID)
        {
            switch (sendFr.main[2])
            {
            case DATA_LENGTH::BMS_VOLUME_FRAME:
                receive((*_bmsFr).volume,   DATA_LENGTH::BMS_VOLUME_FRAME,  &lv_receive);
                bms_volume = (*_bmsFr).volume[3];
                break;
            case DATA_LENGTH::BMS_MAIN_FRAME:
                receive((*_bmsFr).ins[sendFr.main[3] - BMS_ADDRESS::BMS_FIRST].main,     DATA_LENGTH::BMS_MAIN_FRAME,    &lv_receive);
                break;
            case DATA_LENGTH::BMS_CELL_FRAME:
                receive((*_bmsFr).ins[sendFr.main[3] - BMS_ADDRESS::BMS_FIRST].cell,     DATA_LENGTH::BMS_CELL_FRAME,    &lv_receive);
                break;
            case DATA_LENGTH::BMS_ADDITION_FRAME:
                receive((*_bmsFr).ins[sendFr.main[3] - BMS_ADDRESS::BMS_FIRST].add,      DATA_LENGTH::BMS_ADDITION_FRAME,&lv_receive);
                break;
            case DATA_LENGTH::DCDC_MAIN_FRAME:
                receive((*_dcdcFr).main,    DATA_LENGTH::DCDC_MAIN_FRAME,   &lv_receive);
                break;
            case DATA_LENGTH::BATTERY_MAIN_FRAME:
                receive((*_batteryFr).main, DATA_LENGTH::BATTERY_MAIN_FRAME,&lv_receive);
                break;
            case DATA_LENGTH::BATTERY_CELL_FRAME:
                receive((*_batteryFr).cell, DATA_LENGTH::BATTERY_CELL_FRAME,&lv_receive);
                break;
            case DATA_LENGTH::BATTERY_ADDITION_FRAME:
                receive((*_batteryFr).add,  DATA_LENGTH::BATTERY_ADDITION_FRAME,&lv_receive);
                break;
            case DATA_LENGTH::FAN_MAIN_FRAME:
                receive((*_fanFr).main,     DATA_LENGTH::FAN_MAIN_FRAME,    &lv_receive);
                break;
            case DATA_LENGTH::ACTUATOR_MAIN_FRAME:
                receive((*_actuatorFr).main,DATA_LENGTH::ACTUATOR_MAIN_FRAME,&lv_receive);
                break;
            default:
                break;
            }
        }
        else  while (this->port->available() > 0) this->port->read(); 
    }
    lv_count_send++;

    // if (lv_count_send == 125)
    // {
    //     for (uint8_t i = 0; i < DATA_LENGTH::BMS_CELL_FRAME; i++)
    //     {
    //         debugPort.print((*_bmsFr).ins[1].cell[i], HEX);
    //         debugPort.print(" ");
    //     }
    //     debugPort.println();
    //     lv_count_send = 0;
    // }
    
}

void DataDevice::send(uint8_t *arr, uint8_t length)
{
    uint16_t lv_crc = crcFunc->calculateCRC16(arr, length - 2);
    arr[length - 2] = (uint8_t)((lv_crc >> 8) & 0xFF);   
    arr[length - 1] = (uint8_t)(lv_crc & 0xFF); 
    for (uint8_t i = 0; i < length; i++) this->port->write(arr[i]);
    this->port->flush();
}

void DataDevice::receive(volatile uint8_t *output, uint8_t length, bool *receive)
{
    this->receiveFr = new uint8_t[length];
    this->port->readBytes(this->receiveFr, length);
    if ((   (this->receiveFr[length - 2] << 8) | this->receiveFr[length - 1]) == 
            crcFunc->calculateCRC16(this->receiveFr, length - 2))
    {
        for (uint8_t i = 1; i < length; i++) output[i] = this->receiveFr[i];
        *receive = true;
        lv_time_check = millis();
    }
    else  while (this->port->available() > 0) this->port->read(); 
    delete[] this->receiveFr;
}