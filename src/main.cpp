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

unsigned long now=0;

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

#if 1
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
    //bluetooth.setupBluetooth();
    obj.stateMachine(START);
    obj.stateMachine(SCREEN0);
    
    additionFr.set=2;
    obj.initMuititask();
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
    BaseType_t xDisplayCreate   = xTaskCreatePinnedToCore(      TaskDisplay,
                                                                DISPLAY_TASKNAME, 
                                                                DISPLAY_STACKSIZE, 
                                                                NULL, 
                                                                DISPLAY_PRIORITY, 
                                                                &TaskDisplay_Handler,
                                                                DISPLAY_COREID); 
    if(xDisplayCreate != pdPASS) Serial.println("Create xDisplayCreate fail!"); 
#endif
    
}

static void uploadCode(void);
unsigned long waitSeconds=0;
bool updateState=true;
static uint16_t count=0;
void loop() 
{
    if(millis() - now >= 5000)
    {
        now = millis();
        count++;
        Serial.printf("count:= %d\n",count);
        vTaskSuspend(TaskDataDevice_Handler);
        vTaskSuspend(TaskMain_Handler);
        vTaskSuspend(TaskDisplay_Handler);
        vTaskSuspend(TaskLVGLMAIN_Handler);
        vTaskSuspend(TaskGetData_Handler);
        vTaskSuspend(TaskUpdataScreen_Handler);
        //vTaskDelete(TaskUpdataScreen_Handler);
        additionFr.set++;
        if(additionFr.set>4)
            additionFr.set=2;
    }
#if 0
   bluetooth.mainBluetooth();
   uploadCode(); 
   
#endif
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
        //Serial.printf("\n[TaskMain] running on core: %d, Free stack space: %d\n", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));

        // BMS
        //for (uint8_t i = 0; i < dataDevice.bms_volume; i++) // dataDevice.bms_volume=2;
        for (uint8_t i = 0; i < 2; i++)
        {
            // voltage
            //bmsFr.get[i].packVoltage =  (float)(bmsFr.ins[i].main[4] * 100 + bmsFr.ins[i].main[5]) / 100;
            bmsFr.get[i].packVoltage =  (float)random(40000,58000)/1000;
            // current
            //bmsFr.get[i].packCurrent =  (float)(bmsFr.ins[i].main[6] * 100 + bmsFr.ins[i].main[7]) / 100;
            bmsFr.get[i].packCurrent =  random(0,2000)/10;
            // SOC, percent
            //bmsFr.get[i].packSOC     =  (float)(bmsFr.ins[i].main[8] * 100 + bmsFr.ins[i].main[9]) / 100;
            bmsFr.get[i].packSOC     =  random(1,100);
            // temperature
            bmsFr.get[i].tempMin     =  bmsFr.ins[i].main[10];
            bmsFr.get[i].tempMax     =  bmsFr.ins[i].main[11];
            //bmsFr.get[i].tempAverage =  (bmsFr.get[i].tempMin + bmsFr.get[i].tempMax) / 2;
            bmsFr.get[i].tempAverage =  random(250,1000)/10;
            // error
            bmsFr.get[i].error       =  bmsFr.ins[i].main[12];
            // cell
            //for (size_t j = 0; j < 16; j++) bmsFr.get[i].cellVmV[j] = (bmsFr.ins[i].cell[4 + 2*j] << 8) | bmsFr.ins[i].cell[5 + 2*j];
            for (size_t j = 0; j < 16; j++) bmsFr.get[i].cellVmV[j] = (float)random(325,425)/100;
            // Maximum cell voltage (mV)
            bmsFr.get[i].maxCellmV = (bmsFr.ins[i].add[4] << 8)| bmsFr.ins[i].add[5];
            // Minimum cell voltage (mV)
            bmsFr.get[i].minCellmV = (bmsFr.ins[i].add[6] << 8)| bmsFr.ins[i].add[7];
            // charge/discharge status (0 stationary, 1 charge, 2 discharge)
            bmsFr.get[i].chargeDischargeStatus = bmsFr.ins[i].add[8];
            // charging MOSFET status
            // discharge MOSFET state
            // BMS life (0~255 cycles)?
            // residual capacity mAH
            // Cell count
            // Temp sensor count
            // charger status 0 = disconnected 1 = connected
            // Load Status 0=disconnected 1=connected
            // charge / discharge cycles
            // SO LAN SAC
            // bmsFr.get[i].bmsCycles = (bmsFr.ins[i].add[21] << 8) | bmsFr.ins[i].add[22];
            bmsFr.get[i].bmsCycles = random(0,2500);
        }

        // DJI Battery
        // voltage
        //batteryFr.get.voltage = (batteryFr.main[3] << 8) | batteryFr.main[4];
        batteryFr.get.voltage = random(40000,58000);
        // current
        // batteryFr.get.current = (batteryFr.main[5] << 24) | (batteryFr.main[6] << 16)
        //                         |(batteryFr.main[7] << 8) | (batteryFr.main[8]);

        batteryFr.get.current = random(0,200);

        // temperature
        //batteryFr.get.temperature = (batteryFr.main[9] << 8) | batteryFr.main[10];
        batteryFr.get.temperature = random(250,1000);
        // percent
        //batteryFr.get.percent = batteryFr.main[11];
        batteryFr.get.percent = random(1,100);
        // error
        batteryFr.get.countError = batteryFr.main[12];
        // cell
        //for (uint8_t i = 0; i < 14; i++)  batteryFr.get.cell[i] = (batteryFr.cell[3+2*i] << 8) | batteryFr.cell[4+2*i];
        for (uint8_t i = 0; i < 14; i++)  batteryFr.get.cell[i] = random(325,425);
        // numberCharge // so lan sac 
        //batteryFr.get.numberCharge = (batteryFr.add[3] << 8) | batteryFr.add[4];
        batteryFr.get.numberCharge = random(0,2500);
        // capacity 
        // ten pin 
        batteryFr.get.capacity = BATTERY_T30;
        // version
        //for (uint8_t i = 0; i < 4; i++) batteryFr.get.version[i] = batteryFr.add[7+i];
        for (uint8_t i = 0; i < 4; i++) batteryFr.get.version[i] = random(0,5);
        // serinumber
        //for (uint8_t i = 0; i < 14; i++) batteryFr.get.seriNumber[i] = batteryFr.add[11+i];
        for (uint8_t i = 0; i < 14; i++) batteryFr.get.seriNumber[i] = random(65,91);
        
        // // Debug test data
        // Serial.print(bmsFr.get[0].packVoltage); Serial.print(" ");
        // Serial.print(bmsFr.get[0].cellVmV[7]); Serial.print(" ");
        // Serial.print(bmsFr.get[1].cellVmV[5]); Serial.print(" ");
        // Serial.print(batteryFr.get.percent); Serial.print(" ");
        // Serial.print(batteryFr.get.cell[1]); Serial.print(" ");
        // Serial.println(batteryFr.get.capacity);

        vTaskDelay((100L * configTICK_RATE_HZ) / 1000L);
    }
    vTaskDelete(NULL);
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
        dataDevice.update(&bmsFr, &dcdcFr, &batteryFr, &fanFr, &actuatorFr, Serial);
        vTaskDelay((1L * configTICK_RATE_HZ) / 1000L);
    }
    vTaskDelete(NULL);
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
        //lv_task_handler();
        vTaskDelay((1L * configTICK_RATE_HZ) / 1000L);
    } 
    vTaskDelete(NULL);
}