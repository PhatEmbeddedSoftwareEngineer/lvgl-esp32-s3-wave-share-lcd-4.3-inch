#include "ble.hpp"
#include "define.hpp"
#include "Display/lcd.hpp"

Bluetooth::Bluetooth()
{

}

Bluetooth::~Bluetooth()
{

}

/*
	khởi tạo 2 mảng updater 16384 bytes và updater2
*/
static uint8_t updater[16384];
static uint8_t updater2[16384];

/*
	cấp phát tĩnh con trỏ pCharacteristicTX , pCharacteristicRX
*/

static BLECharacteristic* pCharacteristicTX;
static BLECharacteristic* pCharacteristicRX;

/*
	khởi tạo deviceConnected = false, sendMode = false, sendSize = true;
	khởi tạo writeFile = false, request = false
	khởi tạo biến tĩnh int writeLen =0, writeLen2 =0;
	khởi tạo biến tĩnh current = true;
	khởi tạo biến int parts =0, next=0,cur=0,MTU=0;
	khởi tạo mặc định là MODE = NORMAL_MODE;
	khởi tạo rParts, và tParts là hai biến unsigned long 
*/
static bool deviceConnected = false, sendMode = false, sendSize = true;
static bool writeFile = false, request = false;
static int writeLen = 0, writeLen2 = 0;
static bool current = true;
static int parts = 0, next = 0, cur = 0, MTU = 0;
int MODE = NORMAL_MODE;
unsigned long rParts, tParts;


/*
	hàm reset in ra reason để reset 
*/

static void rebootEspWithReason(String reason) {
  Serial.println(reason);
  delay(100);
  ESP.restart();
}

/*
	khởi tạo lớp cha là MyServerCallbacks thừa hưởng toàn bộ những phần public của lớp con là 
	BLEServerCallbacks 
	hàm onConnect set biến tĩnh deviceConnected = true;
	hàm onDisconnect set biến tĩnh deviceConnected = false;
	
*/

void MyServerCallbacks::onConnect(BLEServer* pServer)
{
  deviceConnected = true;
}

void MyServerCallbacks::onDisconnect(BLEServer* pServer)
{
  deviceConnected = false;
}

void MyCallbacks::onNotify(BLECharacteristic *pCharacteristic)
{
  uint8_t* pData;
	// tạo chuỗi value chứa giá trị của getValue();
  std::string value = pCharacteristic->getValue();
	// tạo biến len lấy chiều dài của chuỗi 
  int len = value.length();
	  
  pData = pCharacteristic->getData();
  if (pData != NULL) {
    Serial.print("TX  ");
    for (int i = 0; i < len; i++) {
      Serial.printf("%02X ", pData[i]);
    }
    Serial.println();
  }
}

void MyCallbacks::onWrite(BLECharacteristic *pCharacteristic)
{
  uint8_t* pData;
  std::string value = pCharacteristic->getValue();
  int len = value.length();
  pData = pCharacteristic->getData();
  if (pData != NULL) {
	/*
	  nếu pData[0] == 0xFB
	*/
  if (pData[0] == 0xFB) {
	// pos = pData[1];
    int pos = pData[1];
    for (int x = 0; x < len - 2; x++) {
		// current mặc định ban đầu là true, MTU =0 
      if (current) {
        updater[(pos * MTU) + x] = pData[x + 2];
      } else {
        updater2[(pos * MTU) + x] = pData[x + 2];
      }
    }

  } else if  (pData[0] == 0xFC) {
	// writeLen =0, writeLen2=0
    if (current) {
      writeLen = (pData[1] * 256) + pData[2];
    } else {
      writeLen2 = (pData[1] * 256) + pData[2];
    }
		// cập nhật lại biến current là false;
    current = !current;
		// cur =0;
    cur = (pData[3] * 256) + pData[4];
		// writeFile mặc định ban đầu là false; cập nhật lại writeFile = true;
    writeFile = true;
		// parts =0; request=false;FASTMODE = false 
		// nếu điều kiện này thỏa update lại request = true;
    if (cur < parts - 1) {
      request = !FASTMODE;
    }
    } else if (pData[0] == 0xFD) {
		// sendMode=false; update lại sendMode = true;
      sendMode = true;
		  // #define FLASH SPIFFS 
		  // nếu trong SPIFFS tồn tại file update.bin xóa nó đi 
      if (FLASH.exists("/update.bin")) {
        FLASH.remove("/update.bin");
      }
    } else if (pData[0] == 0xFE) {
		  // unsigned long rParts, tParts;
      rParts = 0;
      tParts = (pData[1] * 256 * 256 * 256) + (pData[2] * 256 * 256) + (pData[3] * 256) + pData[4];

      Serial.print("Available space: ");
      Serial.println(FLASH.totalBytes() - FLASH.usedBytes());
      Serial.print("File Size: ");
      Serial.println(tParts);

    } else if  (pData[0] == 0xFF) {
      parts = (pData[1] * 256) + pData[2];
      MTU = (pData[3] * 256) + pData[4];
		  // nếu trong case này thì là MODE = UPDATE_MODE;
      MODE = UPDATE_MODE;

    } else if (pData[0] == 0xEF) {
      FLASH.format();
      sendSize = true;
    }


  }
}


// hàm này nhận 4 tham số địa chỉ của fs, đường dẫn path, con trỏ 
static void writeBinary(fs::FS &fs, const char * path, uint8_t *dat, int len) {
  // mở đường dẫn lấy file chế độ FILE_APPEND
  File file = fs.open(path, FILE_APPEND);
  /*
	nếu mở file không được in ra lỗi 
  */
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  // hàm này sẽ ghi dat vào file theo chiều dài của file 
  file.write(dat, len);
  // ghi xong đóng file lại 
  file.close();
  // update writeFile = false , update rParts += len 
  writeFile = false;
  rParts += len;
}

/*
	hàm gửi kết quả ota 
	chuyển đổi chuỗi String result thành char * để gửi 
*/
static void sendOtaResult(String result) {
  pCharacteristicTX->setValue(result.c_str());
  pCharacteristicTX->notify();
  delay(200);
}


/*
	hàm performUpdate nhận hai tham số địa chỉ của Stream và một tham số size_t
	khởi tạo biến char s1= 0x0F;
	result = convert char s1 thành String -> String(s1);
	
*/

static void performUpdate(Stream &updateSource, size_t updateSize) {
  char s1 = 0x0F;
  String result = String(s1);
  // Update.begin (updateSize) kích thước size 
  if (Update.begin(updateSize)) {
	// ghi dữ liệu từ một luồng Stream vào thẳng trực tiếp bộ nhớ flash của esp32
    size_t written = Update.writeStream(updateSource);
	
    if (written == updateSize) {
      Serial.println("Written : " + String(written) + " successfully");
    }
    else {
      Serial.println("Written only : " + String(written) + "/" + String(updateSize) + ". Retry?");
    }
	// result sẽ là một chuỗi của Written + String(written) + "/" ...
    result += "Written : " + String(written) + "/" + String(updateSize) + " [" + String((written / updateSize) * 100) + "%] \n";
    /*
		nếu Update.end() 
		thì ota đã cập nhật xong 
		đưa chuỗi "OTA Done: " vào result 
		
	*/
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
  /*
	nếu deviceConnected = true 
  */
  if (deviceConnected) {
    sendOtaResult(result);
    delay(500);
  }
}

static void updateFromFS(fs::FS &fs) {
  // mở file update.bin 
  File updateBin = fs.open("/update.bin");
  // nếu mở được file updateBin
  if (updateBin) {
	// nếu bên trong file này không có thư mục nghĩa là không có file pin đóng file và dừng chương trình tại đây 
    if (updateBin.isDirectory()) {
      Serial.println("Error, update.bin is not a file");
      updateBin.close();
      return;
    }
	// size_t updateSize = kích thước của file bin 
    size_t updateSize = updateBin.size();
	/*
		nếu updateSize > 0 thì truyền dữ liệu vào chương trình performUpdate(updateBin,size )
		void performUpdate(Stream &updateSource, size_t updateSize)
		
	*/
    if (updateSize > 0) {
      Serial.println("Trying to start update");
      performUpdate(updateBin, updateSize);
    }
    else {
      Serial.println("Error, file is empty");
    }
	// chạy xong chức năng cần chạy rồi thì đóng file lại kết thúc chương trình truyền hình đến đây là hết
    updateBin.close();
	
    // when finished remove the binary from spiffs to indicate end of the process
    Serial.println("Removing update file");
	// remove file update.bin 
    fs.remove("/update.bin");
	/*
		hàm này sẽ in ra một dòng chữ sau đó reset lại esp32 
	*/
    rebootEspWithReason("Rebooting to complete OTA update");
  }
  else {
	// in không thể load update.bin from spiffs 
    Serial.println("Could not load update.bin from spiffs root");
  }
}



/*
	khởi tạo BLE có tên là ECOPOWER
	
*/


static void initBLE() {
  
  BLEDevice::init("ECOPOWER");
  
  BLEServer *pServer = BLEDevice::createServer();
  // cấp phát động cho lớp cha MyServerCallbacks();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // tạo SERVICE_UUID
  // 
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // tạo character uuid tx 
  pCharacteristicTX = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY );
  // tạo character uuid rx 
  pCharacteristicRX = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  // cấp phát động cho class bố MyCallbacks 
  pCharacteristicRX->setCallbacks(new MyCallbacks());
  // cấp phát động cho class bố MyCallbacks
  pCharacteristicTX->setCallbacks(new MyCallbacks());
  // cấp phát động cho BLE2902 
  pCharacteristicTX->addDescriptor(new BLE2902());
  pCharacteristicTX->setNotifyProperty(true);
  pService->start();

 
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	// addServiceUUID 
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  // chức năng giúp giải quyết vấn đề kết nối của iphone 
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");

}


static void fcnResetLimitedTimes(uint8_t maxResets)
{
  nvs_flash_init();
  nvs_handle_t nvs_handle;
  nvs_open("storage", NVS_READWRITE, &nvs_handle);

  uint8_t currentResets = 0;
  nvs_get_u8(nvs_handle, "resetCount", &currentResets);

  if (currentResets < maxResets) {
    Serial.printf("khoi dong lai lan thu %d/%d\n", currentResets + 1, maxResets);
    currentResets++;
    nvs_set_u8(nvs_handle, "resetCount", currentResets);
    nvs_commit(nvs_handle);
    vTaskDelay(pdMS_TO_TICKS(1));
    esp_restart();
  } else {
    Serial.println("da khoi dong xong.");
    nvs_set_u8(nvs_handle, "resetCount", 0);
    nvs_commit(nvs_handle);
  }

  nvs_close(nvs_handle);
}


void Bluetooth::setupBluetooth()
{
  Serial.begin(115200);
  //delay(5000);
  
#if 1
  
  Serial.println("Starting BLE OTA sketch");

#ifdef USE_SPIFFS
  // nếu kết nối SPIFFS không được thì dừng tại đây lun 
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
  // 1075786 2281841
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
  //initBLE();
#endif 
  //fcnResetLimitedTimes(5);
  
}
static bool one=true;
static bool onlyOne=true;
void Bluetooth::mainBluetooth()
{
  #if 1
  if(one)
  {
    Serial.printf("MODE := %s\n",(MODE == 0)? "NORMAL_MODE":(MODE == 1) ? "UPDATE_MODE" : (MODE == 2) ? "OTA_MODE":"cai eo gi vay ");
    one=false;
  }
  
  switch (MODE) {

    case NORMAL_MODE:
      if (deviceConnected) {
		// sendMode == true 
      
        if (sendMode) {
          uint8_t fMode[] = {0xAA, FASTMODE};
          pCharacteristicTX->setValue(fMode, 2);
          pCharacteristicTX->notify();
          delay(50);
          sendMode = false;
        }
		// sendSize == true 
        if (sendSize) {
          unsigned long x = FLASH.totalBytes();
          unsigned long y = FLASH.usedBytes();
          uint8_t fSize[] = {0xEF, (uint8_t) (x >> 16), (uint8_t) (x >> 8), (uint8_t) x, (uint8_t) (y >> 16), (uint8_t) (y >> 8), (uint8_t) y};
          pCharacteristicTX->setValue(fSize, 7);
          pCharacteristicTX->notify();
          delay(50);
          sendSize = false;
        }
        // your loop code here
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
        delay(50);
        request = false;
      }

      if (cur + 1 == parts) { // received complete file
        uint8_t com[] = {0xF2, (uint8_t)((cur + 1) / 256), (uint8_t)((cur + 1) % 256)};
        pCharacteristicTX->setValue(com, 3);
        pCharacteristicTX->notify();
        delay(50);
        MODE = OTA_MODE;
      }

      if (writeFile) {
        if (!current) {
          writeBinary(FLASH, "/update.bin", updater, writeLen);
        } else {
          writeBinary(FLASH, "/update.bin", updater2, writeLen2);
        }
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
        {
          esp_restart();
        }
        
        delay(10);
      }
      // vTaskResume(TaskDataDevice_Handler);
      // vTaskResume(TaskMain_Handler);
      // vTaskResume(TaskDisplay_Handler);
      // vTaskResume(TaskLVGLMAIN_Handler);
      // vTaskResume(TaskGetData_Handler);
      // vTaskResume(TaskUpdataScreen_Handler);
      // vTaskResume(TaskUpdateLCD_Handler);
      break;

  }
#endif
}
// static void loopBluetooth(void *arg)
// {
    
//   /*
// 	#define NORMAL_MODE   0   // normal
// 	#define UPDATE_MODE   1   // receiving firmware
// 	#define OTA_MODE      2   // installing firmware
//   */
//  while(1)
//  {
// #if 1
//   if(one)
//   {
//     Serial.printf("MODE := %s\n",(MODE == 0)? "NORMAL_MODE":(MODE == 1) ? "UPDATE_MODE" : (MODE == 2) ? "OTA_MODE":"cai eo gi vay ");
//     one=false;
//   }
  
//   switch (MODE) {

//     case NORMAL_MODE:
//       if (deviceConnected) {
//         //vTaskSuspend(TaskDataDevice_Handler);
// 		// sendMode == true 
//         if (sendMode) {
//           uint8_t fMode[] = {0xAA, FASTMODE};
//           pCharacteristicTX->setValue(fMode, 2);
//           pCharacteristicTX->notify();
//           delay(50);
//           sendMode = false;
//         }
// 		// sendSize == true 
//         if (sendSize) {
//           unsigned long x = FLASH.totalBytes();
//           unsigned long y = FLASH.usedBytes();
//           uint8_t fSize[] = {0xEF, (uint8_t) (x >> 16), (uint8_t) (x >> 8), (uint8_t) x, (uint8_t) (y >> 16), (uint8_t) (y >> 8), (uint8_t) y};
//           pCharacteristicTX->setValue(fSize, 7);
//           pCharacteristicTX->notify();
//           delay(50);
//           sendSize = false;
//         }

//         // your loop code here
//       } 

//       // or here

//       break;

//     case UPDATE_MODE:

//       if (request) {
//         uint8_t rq[] = {0xF1, (cur + 1) / 256, (cur + 1) % 256};
//         pCharacteristicTX->setValue(rq, 3);
//         pCharacteristicTX->notify();
//         delay(50);
//         request = false;
//       }

//       if (cur + 1 == parts) { // received complete file
//         uint8_t com[] = {0xF2, (cur + 1) / 256, (cur + 1) % 256};
//         pCharacteristicTX->setValue(com, 3);
//         pCharacteristicTX->notify();
//         delay(50);
//         MODE = OTA_MODE;
//       }

//       if (writeFile) {
//         if (!current) {
//           writeBinary(FLASH, "/update.bin", updater, writeLen);
//         } else {
//           writeBinary(FLASH, "/update.bin", updater2, writeLen2);
//         }
//       }

//       break;

//     case OTA_MODE:

//       if (writeFile) {
//         if (!current) {
//           writeBinary(FLASH, "/update.bin", updater, writeLen);
//         } else {
//           writeBinary(FLASH, "/update.bin", updater2, writeLen2);
//         }
//       }


//       if (rParts == tParts) {
//         Serial.println("Complete");
//         delay(500);
//         updateFromFS(FLASH);
//       } else {
//         writeFile = true;
//         Serial.println("Incomplete");
//         Serial.print("Expected: ");
//         Serial.print(tParts);
//         Serial.print("Received: ");
//         Serial.println(rParts);
//         if(rParts > tParts)
//         {
//           esp_restart();
//         }
//         delay(10);
//       }
//       break;

//   }
// #endif
//   vTaskDelay(pdMS_TO_TICKS(10));
//  }
// }

// void Bluetooth::initTaskBluetooth()
// {
//   xTaskCreatePinnedToCore(loopBluetooth,BLE_TASKNAME,BLE_STACKSIZE,NULL,BLE_PRIORITY,NULL,BLE_COREID);

// }

