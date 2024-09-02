#ifndef VARIABLE_H
#define VARIABLE_H

#ifdef  BMS_NUMBER
#define BMS_MAX_INSTANCES   BMS_NUMBER
#define BMS_PORT_TIMEOUT    BMS_TIMEOUT
#else
#define BMS_MAX_INSTANCES   2 
#define BMS_PORT_TIMEOUT    100   
#endif

enum DEVICE_ID
{
    DCDC_ID =   0x54,
    MAIN_ID =   0x52,
    C4G_ID =    0x55, 
    DATA_ID =   0x59, 
    DISPLAY_ID =0x45, 
    BLUETOOTH = 0x4E, 
};

enum COMMAND_ID
{
    BMS     = 0x44,
    DCDC    = 0x49,
    BATTERY = 0x4E,
    FAN     = 0x48,
    ACTUATOR= 0x52,
    ADDITION= 0x73,
};

enum DATA_LENGTH
{
    BMS_VOLUME_FRAME        = 7,
    BMS_MAIN_FRAME          = 22,
    BMS_CELL_FRAME          = 38,
    BMS_ADDITION_FRAME      = 25,
    DCDC_MAIN_FRAME         = 21,
    BATTERY_MAIN_FRAME      = 15,
    BATTERY_CELL_FRAME      = 33,
    BATTERY_ADDITION_FRAME  = 27,
    FAN_MAIN_FRAME          = 14,
    ACTUATOR_MAIN_FRAME     = 10,
    C4G_FRAME               = 9,
    ADDITION_MAIN_FRAME     = 8,
};

enum BMS_ADDRESS
{
    BMS_FIRST = 0x40,
    BMS_SECOND = 0x41,
    BMS_THIRD = 0x42,
    BMS_FOURTH = 0x43,
};

struct BMS_FRAME
{
    struct bms_ins
    {
        volatile uint8_t main[DATA_LENGTH::BMS_MAIN_FRAME];
        volatile uint8_t cell[DATA_LENGTH::BMS_CELL_FRAME];
        volatile uint8_t add[DATA_LENGTH::BMS_ADDITION_FRAME];
    }ins[BMS_MAX_INSTANCES];
    volatile uint8_t volume[DATA_LENGTH::BMS_VOLUME_FRAME];
    volatile uint8_t instance;

    struct Data
    {
        // data from 0x90
        volatile float packVoltage; // Total pack voltage (0.1 V)
        volatile float packCurrent; // Current in (+) or out (-) of pack (0.1 A)
        volatile float packSOC;     // State Of Charge

        // data from 0x91
        volatile float maxCellmV; // Maximum cell voltage (mV)
        volatile int maxCellVNum; // Number of cell with highest voltage
        volatile float minCellmV; // Minimum cell voltage (mV)
        volatile int minCellVNum; // Number of cell with lowest voltage
        volatile float cellDiff;  // Difference between min and max cell voltages

        // data from 0x92
        volatile int tempMax;       // Maximum temperature sensor reading (°C)
        volatile int tempMin;       // Minimum temperature sensor reading (°C)
        volatile float tempAverage; // Average of temp sensors

        // data from 0x93
        volatile uint8_t chargeDischargeStatus; // charge/discharge status (0 stationary, 1 charge, 2 discharge)
        volatile bool chargeFetState;          // charging MOSFET status
        volatile bool disChargeFetState;       // discharge MOSFET state
        volatile int bmsHeartBeat;             // BMS life (0~255 cycles)?
        volatile int resCapacitymAh;           // residual capacity mAH

        // data from 0x94
        volatile int numberOfCells;    // Cell count
        volatile int numOfTempSensors; // Temp sensor count
        volatile bool chargeState;     // charger status 0 = disconnected 1 = connected
        volatile bool loadState;       // Load Status 0=disconnected 1=connected
        volatile bool dIO[8];          // No information about this
        volatile int bmsCycles;        // charge / discharge cycles

        // data from 0x95
        volatile float cellVmV[48]; // Store Cell Voltages (mV)

        // data from 0x96
        volatile int cellTemperature[16]; // array of cell Temperature sensors

        // data from 0x97
        volatile bool cellBalanceState[48]; // bool array of cell balance states
        volatile bool cellBalanceActive;    // bool is cell balance active

        // error
        volatile uint8_t error;
    };

    Data get[BMS_MAX_INSTANCES];

    volatile uint8_t led = 0;

    /**
     * @brief alarm struct holds booleans corresponding to all the possible alarms
     * (aka errors/warnings) the BMS can report
     */

    struct Alarm
    {
        // data from 0x98
        /* 0x00 */
        volatile bool levelOneCellVoltageTooHigh;
        volatile bool levelTwoCellVoltageTooHigh;
        volatile bool levelOneCellVoltageTooLow;
        volatile bool levelTwoCellVoltageTooLow;
        volatile bool levelOnePackVoltageTooHigh;
        volatile bool levelTwoPackVoltageTooHigh;
        volatile bool levelOnePackVoltageTooLow;
        volatile bool levelTwoPackVoltageTooLow;
        volatile uint8_t byte0;
        /* 0x01 */
        volatile bool levelOneChargeTempTooHigh;
        volatile bool levelTwoChargeTempTooHigh;
        volatile bool levelOneChargeTempTooLow;
        volatile bool levelTwoChargeTempTooLow;
        volatile bool levelOneDischargeTempTooHigh;
        volatile bool levelTwoDischargeTempTooHigh;
        volatile bool levelOneDischargeTempTooLow;
        volatile bool levelTwoDischargeTempTooLow;
        volatile uint8_t byte1;
        /* 0x02 */
        volatile bool levelOneChargeCurrentTooHigh;
        volatile bool levelTwoChargeCurrentTooHigh;
        volatile bool levelOneDischargeCurrentTooHigh;
        volatile bool levelTwoDischargeCurrentTooHigh;
        volatile bool levelOneStateOfChargeTooHigh;
        volatile bool levelTwoStateOfChargeTooHigh;
        volatile bool levelOneStateOfChargeTooLow;
        volatile bool levelTwoStateOfChargeTooLow;
        volatile uint8_t byte2;
        /* 0x03 */
        volatile bool levelOneCellVoltageDifferenceTooHigh;
        volatile bool levelTwoCellVoltageDifferenceTooHigh;
        volatile bool levelOneTempSensorDifferenceTooHigh;
        volatile bool levelTwoTempSensorDifferenceTooHigh;
        volatile uint8_t byte3;
        /* 0x04 */
        volatile bool chargeFETTemperatureTooHigh;
        volatile bool dischargeFETTemperatureTooHigh;
        volatile bool failureOfChargeFETTemperatureSensor;
        volatile bool failureOfDischargeFETTemperatureSensor;
        volatile bool failureOfChargeFETAdhesion;
        volatile bool failureOfDischargeFETAdhesion;
        volatile bool failureOfChargeFETTBreaker;
        volatile bool failureOfDischargeFETBreaker;
        volatile uint8_t byte4;
        /* 0x05 */
        volatile bool failureOfAFEAcquisitionModule;
        volatile bool failureOfVoltageSensorModule;
        volatile bool failureOfTemperatureSensorModule;
        volatile bool failureOfEEPROMStorageModule;
        volatile bool failureOfRealtimeClockModule;
        volatile bool failureOfPrechargeModule;
        volatile bool failureOfVehicleCommunicationModule;
        volatile bool failureOfIntranetCommunicationModule;
        volatile uint8_t byte5;
        /* 0x06 */
        volatile bool failureOfCurrentSensorModule;
        volatile bool failureOfMainVoltageSensorModule;
        volatile bool failureOfShortCircuitProtection;
        volatile bool failureOfLowVoltageNoCharging;
        volatile uint8_t byte6;
    };

    Alarm alarm[BMS_MAX_INSTANCES];
};

struct DCDC_FRAME
{
    volatile uint8_t main[DATA_LENGTH::DCDC_MAIN_FRAME];
    struct dcdc_struct_set
    {
        volatile double temperature[4];
        volatile double Iout[4];
    }get;
};

struct BATTERY_FRAME
{
    volatile uint8_t main[DATA_LENGTH::BATTERY_MAIN_FRAME];
    volatile uint8_t cell[DATA_LENGTH::BATTERY_CELL_FRAME];
    volatile uint8_t add [DATA_LENGTH::BATTERY_ADDITION_FRAME];
    struct batteryStruct
    {
        volatile bool active = false;
        volatile uint8_t seriNumber[14] = {};
        volatile uint8_t version[4] = {};
        volatile uint16_t voltage;
        volatile int32_t current;
        volatile uint8_t percent;
        volatile uint16_t cell[14] = {};
        volatile int16_t numberCharge = -1;
        volatile uint16_t temperature;
        volatile uint16_t capacity;
        volatile uint8_t countError = 0;
        volatile uint8_t led = 0;
    }get;
};

struct FAN_FRAME
{
    volatile uint8_t main[DATA_LENGTH::FAN_MAIN_FRAME];
    volatile uint8_t fan_rpm[6];
};

struct ACTUATOR_FRAME
{
    volatile uint8_t main[DATA_LENGTH::ACTUATOR_MAIN_FRAME];
    struct LED
    {
        volatile uint8_t bms;
        volatile uint8_t dcdc;
        volatile uint8_t battery;
    }led;
    
};

struct ADDITION_FRAME
{
    volatile uint8_t main[DATA_LENGTH::ADDITION_MAIN_FRAME];
    volatile uint8_t set;
};
#define DATA_FRAME                  DATA_LENGTH::C4G_FRAME
#define DISPLAY_FRAME               DATA_LENGTH::C4G_FRAME
#endif // VARIABLE_H