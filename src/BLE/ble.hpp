#ifndef BLE_HPP
#define BLE_HPP

#include <Update.h>
#include "FS.h"
#include "FFat.h"
#include "SPIFFS.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_system.h>

extern int MODE;

// define FORMAT_SPIFFS_IF_FAILED mặc định là true 
#define FORMAT_SPIFFS_IF_FAILED true
// define FORMAT_FFAT_IF_FAILED mặc định là true 
#define FORMAT_FFAT_IF_FAILED true

#define USE_SPIFFS  //comment to use FFat

/* 
	nếu define USE_SPIFFS thì define FLASH thành SPIFFS
	define FASTMODE là false 
*/
#ifdef USE_SPIFFS
#define FLASH SPIFFS
#define FASTMODE false    //SPIFFS write is slow
#else
#define FLASH FFat
#define FASTMODE true    //FFat is faster
#endif

/*
	define 3 mode 
*/
#define NORMAL_MODE   0   // normal
#define UPDATE_MODE   1   // receiving firmware
#define OTA_MODE      2   // installing firmware

#define SERVICE_UUID              "fb1e4001-54ae-4a28-9f74-dfccb248601d"
#define CHARACTERISTIC_UUID_RX    "fb1e4002-54ae-4a28-9f74-dfccb248601d"
#define CHARACTERISTIC_UUID_TX    "fb1e4003-54ae-4a28-9f74-dfccb248601d"


#define RTC_DATA_ADDR 0 // Address in RTC RAM to store the restart status

#define BLE_TASKNAME            "Bluetooth update firmware"
#define BLE_STACKSIZE            1024*3
#define BLE_PRIORITY             7
#define BLE_COREID               0

class Bluetooth
{
public:
    Bluetooth();
    ~Bluetooth();
    void setupBluetooth();
    //void loopBluetooth();
    //void initTaskBluetooth();
    void mainBluetooth();
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer);
    void onDisconnect(BLEServer* pServer);
};

class MyCallbacks: public BLECharacteristicCallbacks {
	/*
		create function onNotify nhận tham số con trỏ BLECharacteristic 
		tạo con trỏ pData;
		
	*/
    void onNotify(BLECharacteristic *pCharacteristic);
    void onWrite(BLECharacteristic *pCharacteristic);
};

#endif