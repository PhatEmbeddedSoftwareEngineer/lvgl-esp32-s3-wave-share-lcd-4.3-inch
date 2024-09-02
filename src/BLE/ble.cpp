#include "ble.hpp"

Bluetooth::Bluetooth(void)
{
    Serial.begin(115200);
}

Bluetooth::~Bluetooth(void)
{

}

uint8_t updater[16384*2];
uint8_t updater2[16384*2];

static BLECharacteristic* pCharacteristicTX;
static BLECharacteristic* pCharacteristicRX;

static bool deviceConnected = false, sendMode = false, sendSize = true;
static bool writeFile = false, request = false;
static int writeLen = 0, writeLen2 = 0;
static bool current = true;
static int parts = 0, next = 0, cur = 0, MTU = 0;
static int MODE = NORMAL_MODE;
unsigned long rParts, tParts;

static void rebootEspWithReason(String reason) {
  Serial.println(reason);
  delay(1000);
  ESP.restart();
}

void MyServerCallbacks::onConnect(BLEServer* pServer) {
    Serial.println("Connected");
    deviceConnected = true;

}
void MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    Serial.println("disconnected");
    deviceConnected = false;
}

void MyCallbacks::onRead(BLECharacteristic* pCharacteristic) {
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.print(": onRead(), value: ");
    Serial.println(pCharacteristic->getValue().c_str());
};

void MyCallbacks::onNotify(BLECharacteristic *pCharacteristic) {
    std::string pData = pCharacteristic->getValue();
    int len = pData.length();
    Serial.print("TX  ");
    for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", pData[i]);
    }
    Serial.println();
}

void MyCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    std::string pData = pCharacteristic->getValue();
    int len = pData.length();
    if (pData[0] == 0xFB) {
        int pos = pData[1];
        for (int x = 0; x < len - 2; x++) {
          if (current) {
            updater[(pos * MTU) + x] = pData[x + 2];
          } else {
            updater2[(pos * MTU) + x] = pData[x + 2];
          }
        }

    } 
    else if  (pData[0] == 0xFC) {
        if (current) {
          writeLen = (pData[1] * 256) + pData[2];
        } else {
          writeLen2 = (pData[1] * 256) + pData[2];
        }
        current = !current;
        cur = (pData[3] * 256) + pData[4];
        writeFile = true;
        if (cur < parts - 1) {
          request = !FASTMODE;
        }
    } 
    else if (pData[0] == 0xFD) {
        sendMode = true;
        if (FLASH.exists("/update.bin")) {
          FLASH.remove("/update.bin");
        }
    } 
    else if (pData[0] == 0xFE) {
        rParts = 0;
        tParts = (pData[1] * 256 * 256 * 256) + (pData[2] * 256 * 256) + (pData[3] * 256) + pData[4];

        Serial.print("Available space: ");
        Serial.println(FLASH.totalBytes() - FLASH.usedBytes());
        Serial.print("File Size: ");
        Serial.println(tParts);

    } 
    else if  (pData[0] == 0xFF) {
        parts = (pData[1] * 256) + pData[2];
        MTU = (pData[3] * 256) + pData[4];
        MODE = UPDATE_MODE;

    } 
    else if (pData[0] == 0xEF) {
        FLASH.format();
        sendSize = true;
    }
}

static void writeBinary(fs::FS &fs, const char * path, uint8_t *dat, int len)
{
    File file = fs.open(path, FILE_APPEND);

    if (!file) {
        Serial.println("- failed to open file for writing");
        return;
    }
    file.write(dat, len);
    file.close();
    writeFile = false;
    rParts += len;
}

static void sendOtaResult(String result) {
    byte arr[result.length()];
    result.getBytes(arr, result.length());
    pCharacteristicTX->setValue(arr, result.length());
    pCharacteristicTX->notify();
    delay(200);
}

static void performUpdate(Stream &updateSource, size_t updateSize) {
    char s1 = 0x0F;
    String result = String(s1);
    if (Update.begin(updateSize)) {
        size_t written = Update.writeStream(updateSource);
        if (written == updateSize) {
            Serial.println("Written : " + String(written) + " successfully");
        }
        else {
            Serial.println("Written only : " + String(written) + "/" + String(updateSize) + ". Retry?");
        }
        result += "Written : " + String(written) + "/" + String(updateSize) + " [" + String((written / updateSize) * 100) + "%] \n";
        if (Update.end()) {
            Serial.println("OTA done!");
            result += "OTA Done: ";
            if (Update.isFinished()) {
                Serial.println("Update successfully completed. Rebooting...");
                result += "Success!\n";
            }
            else {
                Serial.println("Update not finished? Something went wrong!");
                result += "Failed!\n";
            }
        }
        else {
            Serial.println("Error Occurred. Error #: " + String(Update.getError()));
            result += "Error #: " + String(Update.getError());
        }
    }
    else
    {
        Serial.println("Not enough space to begin OTA");
        result += "Not enough space for OTA";
    }
    if (deviceConnected) {
        sendOtaResult(result);
        delay(500);
    }
}


static void updateFromFS(fs::FS &fs) {
    File updateBin = fs.open("/update.bin");
    if (updateBin) {
        if (updateBin.isDirectory()) {
            Serial.println("Error, update.bin is not a file");
            updateBin.close();
            return;
        }

        size_t updateSize = updateBin.size();

        if (updateSize > 0) {
            Serial.println("Trying to start update");
            performUpdate(updateBin, updateSize);
        }
        else {
            Serial.println("Error, file is empty");
        }

        updateBin.close();

        // when finished remove the binary from spiffs to indicate end of the process
        Serial.println("Removing update file");
        fs.remove("/update.bin");

        rebootEspWithReason("Rebooting to complete OTA update");
    }
    else {
        Serial.println("Could not load update.bin from spiffs root");
    }
}

static void initBLE() {
    BLEDevice::init("NimBLE OTA");
    BLEDevice::setMTU(517);
    BLEDevice::setPower(ESP_PWR_LVL_P21);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristicTX = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ );
    pCharacteristicRX = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR);
    pCharacteristicRX->setCallbacks(new MyCallbacks());
    pCharacteristicTX->setCallbacks(new MyCallbacks());
    pService->start();
    pServer->start();

    // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
    NimBLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->start();
    if (FLASH.exists("/update.bin")) {
        Serial.printf("have update.bin\n");
        FLASH.remove("/update.bin");
        Serial.printf("da xoa update.bin\n");
    }

    //BLEDevice::startAdvertising();
    Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void Bluetooth::setupBluetooth()
{
    Serial.println("Starting BLE OTA sketch");
    //SPI.begin(18, 22, 23, 5);       // For OTA with SD Card
    //SD.begin(5);                    // For OTA with SD Card
#ifdef USE_SPIFFS
    if (!FLASH.begin(FORMAT_SPIFFS_IF_FAILED)) {
        Serial.println("SPIFFS Mount Failed");
        return;
    }
#else
    if (!FFat.begin()) {
        Serial.println("FFat Mount Failed");
        if (FORMAT_FFAT_IF_FAILED) FFat.format();
        return;
    }
    
#endif
    Serial.print("Available space: ");
    Serial.println(FLASH.totalBytes() - FLASH.usedBytes());
    // 2281841
    if(FLASH.totalBytes() - FLASH.usedBytes() < 2000000)
    {
        if(FLASH.format())
        {
        Serial.println("format success");
        }
        else
        Serial.println("format failed");
    }
    Serial.print("Available space: ");
    Serial.println(FLASH.totalBytes() - FLASH.usedBytes());

    initBLE();
}
static bool onlyOne=true;
void Bluetooth::loopBluetooth()
{
    switch (MODE) {
    case NORMAL_MODE:
        if (deviceConnected) {
            //digitalWrite(BUILTINLED, HIGH);
            if (sendMode) {
                uint8_t fMode[] = {0xAA, FASTMODE};
                pCharacteristicTX->setValue(fMode, 2);
                pCharacteristicTX->notify();
                delay(30);
                sendMode = false;
            }
            if (sendSize) {
                unsigned long x = FLASH.totalBytes();
                unsigned long y = FLASH.usedBytes();
                uint8_t fSize[] = {0xEF, (uint8_t) (x >> 16), (uint8_t) (x >> 8), (uint8_t) x, (uint8_t) (y >> 16), (uint8_t) (y >> 8), (uint8_t) y};
                pCharacteristicTX->setValue(fSize, 7);
                pCharacteristicTX->notify();
                delay(30);
                sendSize = false;
            }

            // your loop code here
        } else {
            //digitalWrite(BUILTINLED, LOW);
        }
        // or here
        break;
    case UPDATE_MODE:
        if(onlyOne)
        {
          additionFr.set=5;
          onlyOne=false;
        }
        if (request) {
            uint8_t rq[] = {0xF1, (uint8_t)((cur + 1) / 256), (uint8_t)((cur + 1) % 256)};
            pCharacteristicTX->setValue(rq, 3);
            pCharacteristicTX->notify();
            delay(30);
            request = false;
        }

        if (writeFile) {
            if (!current) {
                writeBinary(FLASH, "/update.bin", updater, writeLen);
            } else {
                writeBinary(FLASH, "/update.bin", updater2, writeLen2);
            }
            writeFile = false;
        }

        if (cur + 1 == parts) { // received complete file
            uint8_t com[] = {0xF2, (uint8_t)((cur + 1) / 256), (uint8_t)((cur + 1) % 256)};
            pCharacteristicTX->setValue(com, 3);
            pCharacteristicTX->notify();
            delay(30);
            MODE = OTA_MODE;
        }
        break;
    case OTA_MODE:
        if (writeFile) {
            if (!current) {
            writeBinary(FLASH, "/update.bin", updater, writeLen);
            } else {
            writeBinary(FLASH, "/update.bin", updater2, writeLen2);
            }
        }
        if (rParts == tParts) {
            Serial.println("Complete");
            delay(500);
            updateFromFS(FLASH);
        } else {
            writeFile = true;
            Serial.println("Incomplete");
            Serial.print("Expected: ");
            Serial.print(tParts);
            Serial.print("Received: ");
            Serial.println(rParts);
            if(rParts > tParts)
                esp_restart();
            delay(1);
        }
        break;
  }

}