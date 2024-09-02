#include "./Display/lcd.hpp"
#include <iostream>
#include <string>
#include <hal/lv_hal_tick.h>
#include "define.hpp"
#include "BLE/ble.hpp"

DataDevice dataDevice(DATA_PORT);
// define task handles

TaskHandle_t TaskDataDevice_Handler;
TaskHandle_t TaskMain_Handler;
TaskHandle_t TaskDisplay_Handler;
TaskHandle_t TaskLVGLMAIN_Handler;
TaskHandle_t TaskGetData_Handler;
TaskHandle_t TaskUpdataScreen_Handler;
TaskHandle_t TaskUpdateLCD_Handler;

// define task
static void TaskDataDevice( void *pvParameters );
static void TaskMain( void *pvParameters );
static void TaskDisplay( void *pvParameters );
Bluetooth bluetooth;
LCD_4_3 obj;
/* tạo hàng đợi*/
QueueHandle_t QueueHandle;
const int QueueElementSize = 10;


/*tạo hàng đợi SemaphoreHandler_t tên là gui_mutex*/
SemaphoreHandle_t gui_mutex;

BATTERY_TYPE battery;
void setup()
{
    dataDevice.init();
    QueueHandle = xQueueCreate(QueueElementSize, sizeof(message_t));
    if (QueueHandle == NULL) {
        Serial.println("Queue could not be created. Halt.");
        while (1) delay(1000);  // Halt at this point as is not possible to continue
    }
    gui_mutex = xSemaphoreCreateRecursiveMutex();
    if (gui_mutex == NULL) {
        // Handle semaphore creation failure
        Serial.println("semaphore creation failure");
        return;
    }
    bluetooth.setupBluetooth();
    obj.stateMachine(START);
    obj.stateMachine(SCREEN0);
    additionFr.set=2;
    obj.initMuititask();

    // create task
    BaseType_t xDataDeviceCreate   = xTaskCreatePinnedToCore(   TaskDataDevice,
                                                                DATADEVICE_TASKNAME, 
                                                                DATADEVICE_STACKSIZE, 
                                                                NULL, 
                                                                DATADEVICE_PRIORITY, 
                                                                &TaskDataDevice_Handler,
                                                                DATADEVICE_COREID);
    if(xDataDeviceCreate != pdPASS) Serial.println("Create xDataDeviceCreate fail!");   

    BaseType_t xMainCreate   = xTaskCreatePinnedToCore(         TaskMain,
                                                                MAIN_TASKNAME, 
                                                                MAIN_STACKSIZE, 
                                                                NULL, 
                                                                MAIN_PRIORITY, 
                                                                &TaskMain_Handler,
                                                                MAIN_COREID); 
    if(xMainCreate != pdPASS) Serial.println("Create xMainCreate fail!");   

    BaseType_t xDisplayCreate   = xTaskCreatePinnedToCore(      TaskMain,
                                                                DISPLAY_TASKNAME, 
                                                                DISPLAY_STACKSIZE, 
                                                                NULL, 
                                                                DISPLAY_PRIORITY, 
                                                                &TaskDisplay_Handler,
                                                                DISPLAY_COREID); 
    if(xDisplayCreate != pdPASS) Serial.println("Create xDisplayCreate fail!"); 
}
unsigned long waitSeconds=0;
bool updateState=true;
static void uploadCode(void);

void loop()
{
    bluetooth.loopBluetooth();
    uploadCode(); 
    //Serial.printf("hello world\n"); delay(500);
    // loop not use
}
static void uploadCode(void)
{
#if 1
    if(!updateState)
    {
        vTaskSuspend(TaskDataDevice_Handler);
        vTaskSuspend(TaskMain_Handler);
        vTaskSuspend(TaskDisplay_Handler);
        vTaskSuspend(TaskLVGLMAIN_Handler);
        vTaskSuspend(TaskGetData_Handler);
        vTaskSuspend(TaskUpdataScreen_Handler);
        vTaskSuspend(TaskUpdateLCD_Handler);
    }
    if(additionFr.set==5)
    {
        for(int i=0;i<2;i++)
        {
            _father.stateMachine(5);
            delay(100);
        }
        updateState=false;
    }
#endif
}
static void TaskMain( void *pvParameters )
{
/*
    Explain Task:
    Task for handle main function such as 
    - convert data to display
*/
    (void) pvParameters;
    for (;;) 
    {
        // if (xSemaphoreTakeRecursive(gui_mutex, portMAX_DELAY) == pdTRUE)
        // {
        //     if(additionFr.set==3 || additionFr.set==4)
        //     {
                // ------------------------------------- BMS -----------------------------------
                for (uint8_t i = 0; i < dataDevice.bms_volume; i++)
                {
                    // voltage
                    bmsFr.get[i].packVoltage =  (float)(bmsFr.ins[i].main[4] * 100 + bmsFr.ins[i].main[5]) / 100;
                    // current
                    bmsFr.get[i].packCurrent =  (float)((int8_t)bmsFr.ins[i].main[6] * 100 + bmsFr.ins[i].main[7]) / 100;
                    // SOC, percent
                    bmsFr.get[i].packSOC     =  (float)(bmsFr.ins[i].main[8] * 100 + bmsFr.ins[i].main[9]) / 100;
                    // temperature
                    bmsFr.get[i].tempMin     =  bmsFr.ins[i].main[10];
                    bmsFr.get[i].tempMax     =  bmsFr.ins[i].main[11];
                    bmsFr.get[i].tempAverage =  (bmsFr.get[i].tempMin + bmsFr.get[i].tempMax) / 2;
                    // error
                    bmsFr.get[i].error       =  bmsFr.ins[i].main[12];
                    // cell
                    for (size_t j = 0; j < 16; j++) bmsFr.get[i].cellVmV[j] = (bmsFr.ins[i].cell[4 + 2*j] << 8) | bmsFr.ins[i].cell[5 + 2*j];
                    // Maximum cell voltage (mV)
                    bmsFr.get[i].maxCellmV = (bmsFr.ins[i].add[4] << 8)| bmsFr.ins[i].add[5];
                    // Minimum cell voltage (mV)
                    bmsFr.get[i].minCellmV = (bmsFr.ins[i].add[6] << 8)| bmsFr.ins[i].add[7];
                    // charge/discharge status (0 stationary, 1 charge, 2 discharge)
                    bmsFr.get[i].chargeDischargeStatus = bmsFr.ins[i].add[8];
                    // charging MOSFET status
                    bmsFr.get[i].chargeFetState = bmsFr.ins[i].add[9];
                    // discharge MOSFET state
                    bmsFr.get[i].disChargeFetState = bmsFr.ins[i].add[10];
                    // BMS life (0~255 cycles)?
                    bmsFr.get[i].bmsHeartBeat = bmsFr.ins[i].add[11];
                    // residual capacity mAH
                    bmsFr.get[i].resCapacitymAh =   (bmsFr.ins[i].add[12] << 24) | (bmsFr.ins[i].add[13] << 16)
                                        |           (bmsFr.ins[i].add[14] << 8)  | (bmsFr.ins[i].add[15]);
                    // Cell count
                    bmsFr.get[i].numberOfCells = bmsFr.ins[i].add[16];
                    // Temp sensor count
                    bmsFr.get[i].numOfTempSensors = bmsFr.ins[i].add[17];
                    // charger status 0 = disconnected 1 = connected
                    bmsFr.get[i].chargeState = bmsFr.ins[i].add[18];
                    // Load Status 0=disconnected 1=connected
                    bmsFr.get[i].loadState = bmsFr.ins[i].add[19];
                    // charge / discharge cycles
                    bmsFr.get[i].bmsCycles = (bmsFr.ins[i].add[21] << 8) | bmsFr.ins[i].add[22];
                }
            // }   

            // if(additionFr.set==2)
            // {  
                    // ------------------------------ DJI Battery -------------------------
                    // voltage
                    batteryFr.get.voltage = (batteryFr.main[3] << 8) | batteryFr.main[4];
                    // current
                    batteryFr.get.current = (batteryFr.main[5] << 24) | (batteryFr.main[6] << 16)
                                            |(batteryFr.main[7] << 8) | (batteryFr.main[8]);
                    // temperature
                    batteryFr.get.temperature = (batteryFr.main[9] << 8) | batteryFr.main[10];
                    // percent
                    batteryFr.get.percent = batteryFr.main[11];
                    // error
                    batteryFr.get.countError = batteryFr.main[12];
                    // cell
                    for (uint8_t i = 0; i < 14; i++)  batteryFr.get.cell[i] = (batteryFr.cell[3+2*i] << 8) | batteryFr.cell[4+2*i];
                    // numberCharge
                    batteryFr.get.numberCharge = (batteryFr.add[3] << 8) | batteryFr.add[4];
                    // capacity
                    batteryFr.get.capacity = (batteryFr.add[5] << 8) | batteryFr.add[6];
                    // version
                    for (uint8_t i = 0; i < 4; i++) batteryFr.get.version[i] = batteryFr.add[7 + i];
                    // serinumber
                    for (uint8_t i = 0; i < 14; i++) batteryFr.get.seriNumber[i] = batteryFr.add[11 + i];
                    
                    // ------------------------------- Fan --------------------------------
                    for (uint8_t i = 0; i < 6; i++) fanFr.fan_rpm[i] = fanFr.main[6 + i]; 
                    
                    // ------------------------------- Led --------------------------------
                    actuatorFr.led.bms = actuatorFr.main[3];
                    actuatorFr.led.dcdc = actuatorFr.main[4];
                    actuatorFr.led.battery = actuatorFr.main[5];

                    // ------------------------------- ADDITION_FRAME --------------------------------
                    if(additionFr.main[3] % 3 == 0) additionFr.set = 2;
                    if(additionFr.main[3] % 3 == 1) additionFr.set = 3;
                    if(additionFr.main[3] % 3 == 2) additionFr.set = 4;
                    //additionFr.set = additionFr.main[3];
                    // Serial.print(additionFr.main[3] % 3);
                    // Serial.print(" ");
                    // Serial.println(additionFr.set);
        //     }
        //     xSemaphoreGiveRecursive(gui_mutex);
        // }
        vTaskDelay((100L * configTICK_RATE_HZ) / 1000L);    




        // Debug test data
        // Serial.print(bmsFr.get[0].packVoltage); Serial.print(" ");
        // Serial.print(bmsFr.get[0].cellVmV[7]); Serial.print(" ");
        // Serial.print(bmsFr.get[1].cellVmV[5]); Serial.print(" ");
        // Serial.print(batteryFr.get.percent); Serial.print(" ");
        // Serial.print(batteryFr.get.cell[1]); Serial.print(" ");
        // Serial.println(batteryFr.get.capacity);
        // Serial.println(additionFr.set);

        // vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L);
    }
}

static void TaskDataDevice( void *pvParameters )
{
/*
    Explain Task:
    Task for update frame struct for get data to display
*/
    (void) pvParameters;
    for (;;) 
    {
        // if (xSemaphoreTakeRecursive(gui_mutex, portMAX_DELAY) == pdTRUE) 
        // {
            dataDevice.update(&bmsFr, &dcdcFr, &batteryFr, &fanFr, &actuatorFr, &additionFr, Serial);
        //     xSemaphoreGiveRecursive(gui_mutex);
        // }
        
        vTaskDelay((1L * configTICK_RATE_HZ) / 1000L);
    }
}

static void TaskDisplay( void *pvParameters )
{
/*
    Explain Task:
    Task for display screen
*/
    (void) pvParameters;
    for (;;) 
    {
        // if (xSemaphoreTakeRecursive(gui_mutex, portMAX_DELAY) == pdTRUE) 
        // {
        //     xSemaphoreGiveRecursive(gui_mutex);
        // }
        vTaskDelay((1L * configTICK_RATE_HZ) / 1000L);
    } 
}