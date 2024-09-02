#ifndef _LCD_HPP
#define _LCD_HPP

#include <Arduino.h>
#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include <ui.h>
#include <lv_conf.h>
#include <functional>
#include "define.hpp"

// Frame struct for get data to display
// Frame struct for get data to display
extern BMS_FRAME       bmsFr;
extern DCDC_FRAME      dcdcFr;
extern BATTERY_FRAME   batteryFr;
extern FAN_FRAME       fanFr;
extern ACTUATOR_FRAME  actuatorFr;
extern ADDITION_FRAME  additionFr;





enum showCell
{
    TEN_CELL=10,
    FOURTEEN_CELL=14,
    SIXTEEN_CELL=16
};



extern QueueHandle_t QueueHandle;
extern SemaphoreHandle_t gui_mutex;

typedef struct 
{
    /*for DJI*/
    char buf[512]; // 20 bytes 
    uint16_t solanSac; // 2 bytes 
    uint16_t count; // 2byte 
    double dienAp; // 8 byte 
    double currentAmpere; // 8 bytes 
    double Temperature; // 8 bytes 
    uint8_t version[4]; // 4 bytes
    char seri[14];  // 14 byte
    float cellDJI[14]; 

    /* for BMS1*/
    double dienApBms1;
    double currentAmpereBms1;
    double TemperatureBms1;
    uint16_t solanSacBms1;
    float cellBMS1[16];

    /* for BMS2*/
    double dienApBms2;
    double currentAmpereBms2;
    double TemperatureBms2;
    uint16_t solanSacBms2;
    float cellBMS2[16];

}message_t;


typedef enum
{
    DJI,
    BMS1,
    BMS2
}PageType_t;


typedef enum 
{
    BATTERY_UNKNOWN = 0,
    BATTERY_T30 = 29000,
    BATTERY_T20P = 13000,
    BATTERY_T40 = 30000,
    BATTERY_T25 = 16818,
    BATTERY_T50 = 33022
}BATTERY_TYPE;

extern BATTERY_TYPE battery;

typedef enum 
{
    START, //0
    SCREEN0, //1
    SCREEN1,//2
    SCREEN2,//3
    SCREEN3, //4
    SCREEN4, // 5
    SCREEN5, // 6
}state_t;


// #define SIZE_FAN 230
// #define SIZE_LED 200
// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5

// I2C Pin define
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9

#include <demos/lv_demos.h>
#include <examples/lv_examples.h>

/* LVGL porting configurations */
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (500)
#define LVGL_TASK_MIN_DELAY_MS  (1)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 20) // 800*20 



extern int DataRandom;
extern int PercentPowerBMS1;


/*thay đổi màu nền của màn hình hoạt động 
    Đỏ (Red): #FF0000
    Xanh lá cây (Green): #00FF00
    Xanh dương (Blue): #0000FF
    Vàng (Yellow): #FFFF00
    Tím (Purple): #800080
    Cyan (Cyan): #00FFFF
    Trắng (White): #FFFFFF
    Đen (Black): #000000
    Xám (Gray): #808080
    Cam (Orange): #FFA500*/

typedef struct
{
    uint32_t RED=0xFF0000;
    uint32_t GREEN = 0x00FF00;
    uint32_t BLUE = 0x0000FF;
    uint32_t YELLOW = 0xFFFF00;
    uint32_t PURPLE = 0x800080;
    uint32_t CYAN = 0x00FFFF;
    uint32_t WHITE = 0xFFFFFF;
    uint32_t BLACK = 0x000000;
    uint32_t GRAY = 0x808080;
    uint32_t ORANGE = 0xFFA500;
}Color;

typedef enum chooseBar
{
    BARDJI,
    BARBMS1,
    BARBMS2,
}Bar;

enum CHOOSE
{
    CHOOSE_DJI=2,
    CHOOSE_BMS1=3,
    CHOOSE_BMS2=4
};

LV_IMG_DECLARE(logosmall);
LV_FONT_DECLARE(arial_40);
LV_FONT_DECLARE(font_1);

class LCD_4_3
{
private:
    
public:
    LCD_4_3();
    void stateMachine(int state);
    void initMuititask(void);
};


class father : public LCD_4_3
{
public:
};

extern father _father;
#endif
