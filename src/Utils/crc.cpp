#include "crc.hpp"

// Khởi tạo con trỏ tĩnh instance
CRCClass* CRCClass::instance = nullptr;

CRCClass::CRCClass() {}

CRCClass* CRCClass::getInstance() {
    // Kiểm tra xem instance đã được tạo chưa, nếu chưa thì tạo mới
    if (instance == nullptr) {
        instance = new CRCClass();
    }
    return instance;
}

uint16_t CRCClass::updateCRC16(uint16_t _crc, uint8_t Data)
{
    uint8_t   crc_index;
    uint16_t  crc_lookup;

    crc_index   = _crc ^ Data;
    crc_lookup  = crc_tab16[crc_index];
  
    return ((_crc >> 8) ^ crc_lookup);
}

uint16_t CRCClass::calculateCRC16(uint8_t volatile * ptr, size_t length)
{
    uint16_t  crc16 = CRC16_INITIAL_VALUE;
            
    while(length--)  crc16 = updateCRC16(crc16, *(ptr++)); 
    
    return  crc16;
}