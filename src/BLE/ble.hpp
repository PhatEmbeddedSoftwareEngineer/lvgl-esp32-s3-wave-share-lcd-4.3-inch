/*
   MIT License
   Copyright (c) 2021 Felix Biego
   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:
   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

#include <Update.h>
#include "FS.h"
#include "FFat.h"
#include "LittleFS.h"
// #include "SPIFFS.h"
#include <NimBLEDevice.h>
#include "Display/lcd.hpp"
//#include <SD.h>             // For OTA with SD Card

#define BUILTINLED 2
#define FORMAT_SPIFFS_IF_FAILED true
#define FORMAT_FFAT_IF_FAILED true

#define USE_SPIFFS  //comment to use FFat

#ifdef USE_SPIFFS
//#define FLASH SPIFFS
#define FLASH LittleFS
#define FASTMODE false    //SPIFFS write is slow
#else
#define FLASH FFat
#define FASTMODE true    //FFat is faster
#endif

#define NORMAL_MODE   0   // normal
#define UPDATE_MODE   1   // receiving firmware
#define OTA_MODE      2   // installing firmware

#define SERVICE_UUID              "fb1e4001-54ae-4a28-9f74-dfccb248601d"
#define CHARACTERISTIC_UUID_RX    "fb1e4002-54ae-4a28-9f74-dfccb248601d"
#define CHARACTERISTIC_UUID_TX    "fb1e4003-54ae-4a28-9f74-dfccb248601d"

class Bluetooth
{
public:
    Bluetooth(void);
    ~Bluetooth(void);
    void setupBluetooth();
    void loopBluetooth();
};

class MyServerCallbacks: public BLEServerCallbacks {
public:
    void onConnect(BLEServer* pServer);
    void onDisconnect(BLEServer* pServer);

};
class MyCallbacks: public BLECharacteristicCallbacks
{
public:
    void onRead(BLECharacteristic* pCharacteristic);
    void onNotify(BLECharacteristic *pCharacteristic);
    void onWrite(BLECharacteristic *pCharacteristic);
};