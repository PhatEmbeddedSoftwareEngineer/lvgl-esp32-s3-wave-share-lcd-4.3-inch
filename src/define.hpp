#ifndef DEFINE_H
#define DEFINE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern TaskHandle_t TaskDataDevice_Handler;
extern TaskHandle_t TaskMain_Handler;
extern TaskHandle_t TaskDisplay_Handler;
extern TaskHandle_t TaskLVGLMAIN_Handler;
extern TaskHandle_t TaskGetData_Handler;
extern TaskHandle_t TaskUpdataScreen_Handler;
extern TaskHandle_t TaskUpdateLCD_Handler;


// config hardware 
#define DATA_PORT   Serial1

// Communication
// DataDevice
#include "Communication/DataDevice.hpp"


// define marco for task in application
#define DATADEVICE_TASKNAME      "DataDeviceTask"
#define DATADEVICE_STACKSIZE     1024*3
#define DATADEVICE_PRIORITY      6
#define DATADEVICE_COREID        1

#define MAIN_TASKNAME            "MainTask"
#define MAIN_STACKSIZE           1024*3
#define MAIN_PRIORITY            4
#define MAIN_COREID              1

#define DISPLAY_TASKNAME         "DisplayTask"
#define DISPLAY_STACKSIZE         1024
#define DISPLAY_PRIORITY          1
#define DISPLAY_COREID            0

#define LVGL_TASKNAME             "lvgl"
#define LVGL_TASK_STACK_SIZE      4 * 1024
#define LVGL_TASK_PRIORITY        10
#define LVGL_COREID               0

#define GETDATA_TASKNAME          "Task get data"
#define GETDATA_STACKSIZE         1024*3
#define GETDATA_PRIORITY          2
#define GETDATA_COREID            1


#define UPDATE_TASKNAME          "task screen update"
#define UPDATE_STACKSIZE         1024*3
#define UPDATE_PRIORITY          7
#define UPDATE_COREID            0

#define CHANGE_SCREEN            "change screen"
#define CHANGE_SCREEN_STACKSIZE  1024*3
#define CHANGE_SCREEN_PRIORITY   8
#define CHANGE_SCREEN_COREID     0



#endif // DEFINE_H