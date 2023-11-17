/**************************************************
 * @file    main.c
 * @brief   SPI_MasterSlave Demo
 * @details Shows Master loopback demo for QSPI0
 *          Read the printf() for instructions
**************************************************/

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "gpio.h"
#include "led.h"
#include "tmr.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "pb.h"
#include "spi.h"
#include "spimss.h"
#include "uart.h"
#include "i2c_regs.h"
#include "i2c.h"
#include "board.h"

// Driver Includes
#include "driver_w25qxx.h"
#include "driver_w25qxx_basic.h"
#include "driver_w25qxx_advance.h"
#include "Wire.h"
#include "millis.h"
#include "MXC4005XC_I2C.h"


/***** Definitions *****/
#define I2C_MASTER       MXC_I2C1
#define I2C_FREQ         100000
#define I2C_FILLER       0xAA
#define DATA_LEN         32

// eLED being the external transmitting LED
#define MXC_GPIO_PORT_OUT     MXC_GPIO0
#define eLED_PIN              MXC_GPIO_PIN_4
mxc_gpio_cfg_t eLED;

// iLED being the LED on the MCU maki roll
#define iLED_PIN              MXC_GPIO_PIN_5
mxc_gpio_cfg_t iLED;

/** Defining Hex Address for Fiber Peripherals **/
#define ACCEL_SENSOR            0x15
#define BLE_SENSOR              0x30
#define LIGHT_SENSOR            0x39     // TSL2584TSV
#define TEMP_SENSOR_1           0x48
#define TEMP_SENSOR_2           0x4F
#define MCU_READOUT_SENSOR      0x50
#define ARDUINO                 0x55     // Slave address of Arduino


/***** Globals *****/
uint8_t config[DATA_LEN];      // Placeholder for BLE messaging
uint8_t rxdata[DATA_LEN];      // To get response from BLE
uint8_t txdata[DATA_LEN];      // To send to SPI
volatile uint32_t isr_cnt;
volatile uint32_t isr_flags;
int error;
bool startMsgRx = false;       // To break loops in app()
uint32_t StartTime;
int ADDRESS;

/** BLE Command Hex Values for DA14531 **/
enum BLE_COMMAND {
    GET_DATA_FROM_NODE_NUM =      0x29,

    UPDATE_TEMP_CHAR_DATA =       0x40,
    UPDATE_LIGHT_CHAR_DATA =      0x41,
    UPDATE_ACCEL_CHAR_DATA =      0x42,
    UPDATE_PPG_CHAR_DATA =        0x43,

    ON_OFF_LED_STATE =            0x14,

    SYNCH_START_MSG =             0xBB,
    RESTART_LOOP =                0xDD,
    DOWNLOAD_MSG =                0xCC,
};

// Helper Function Declarations
void delay(int milliseconds);
void writeLEDLow();
void writeLEDHigh();
void writeI_LEDLow();
void writeI_LEDHigh();

void Setup(void);
void testing(void);
void collect(void);
void Chip_Erase(void);
void Readout(void);
float Read_Temp_Sensor(int which);
void Data_Collect(void);
short Float32toFloat16(float input);



/******************************************************************************************************************************************/
/******************************************************************************************************************************************/
/******************************************************************************************************************************************/
/******************************************************************************************************************************************/

int main(void)
{
    Setup();

    // Chip_Erase();
    // collect();
    // Data_Collect();

    // Readout();

    // testing();
}


void Setup(void)
{
    printf("\n\n------\nBeginning System Setup...\n");

    // Initializing timer and interrupts
    MXC_NVIC_SetVector(TMR2_IRQn, ContinuousTimerHandler);
    NVIC_EnableIRQ(TMR2_IRQn);
    ContinuousTimer();
    printf(" - Initialized timer and interrupts.\n");

    /***********************************/
    /* Setup external LED output pin. */
    eLED.port = MXC_GPIO_PORT_OUT;
    eLED.mask = eLED_PIN;
    eLED.pad  = MXC_GPIO_PAD_NONE;
    eLED.func = MXC_GPIO_FUNC_OUT;
    MXC_GPIO_Config(&eLED);
    writeLEDLow();

    /* Setup internal LED output pin. */
    iLED.port = MXC_GPIO_PORT_OUT;
    iLED.mask = iLED_PIN;
    iLED.pad  = MXC_GPIO_PAD_NONE;
    iLED.func = MXC_GPIO_FUNC_OUT;
    MXC_GPIO_Config(&iLED);
    writeI_LEDLow();

    // writeI_LEDHigh();
    // writeLEDHigh();
    printf(" - Initialized LEDs.\n");
    /***********************************/
    

    /************ SPI SETUP ************/
    uint8_t retVal = w25qxx_basic_init(0x1F16, W25QXX_INTERFACE_SPI, W25QXX_BOOL_FALSE);
    if (retVal==0)
        printf(" - Initialized SPI bus & AT25SL64 flash.\n");
    else
        printf(" - Failed to initialize SPI bus!!!");
    /***********************************/


    /************ I2C SETUP ************/
    error = Wire_begin();
    if (error != E_NO_ERROR)
        while (1)
        {
            printf("Failed to initiate I2C controller port.\n");
            writeI_LEDLow();
            delay(1000);
            NVIC_SystemReset(); // Reset MCU
        }
    else printf(" - Initialized I2C controller port.\n");

    // Check Accelerometer
    Wire_beginTransmission(ACCEL_SENSOR);
    error = Wire_endTransmission();
    if (error != 0) { printf(" - Cannot find accel sensor!!!\n"); } 
    else            { printf(" - Initialized accel sensor.\n"); }
    MXC4005XC_config(2); // Set accel sensor to 2G range

    // Check for Temp Sensor #1
    Wire_beginTransmission(TEMP_SENSOR_1);
    error = Wire_endTransmission();
    if (error != 0) { printf(" - Cannot find temp sensor 1!!!\n"); } 
    else            { printf(" - Initialized temp sensor 1.\n"); }

    // Check for Temp Sensor #2
    Wire_beginTransmission(TEMP_SENSOR_2);
    error = Wire_endTransmission();
    if (error != 0) { printf(" - Cannot find temp sensor 2!!!\n"); } 
    else            { printf(" - Initialized temp sensor 2.\n"); }
    /***********************************/
    printf("System Setup Complete!\n\n");


    writeI_LEDHigh();
    writeLEDHigh();
    delay(1000);
}


void Data_Collect(void)
{
    printf("\n\n------\nBeginning Data Collection...\n");
    unsigned short x1_, y1_, z1_;
    uint32_t accel_addr = 0x00;
    uint32_t temp_addr = 0x80000;

    for (int MINUTES = 0; MINUTES < 10; MINUTES++) // Number of minutes we want to record data for
    {
        // Initialize data buffers and indices
        uint8_t* accel_data = (uint8_t*) malloc(18000 * sizeof(uint8_t));  // 60sec * 50hz = 3000 readings * 6 B = 18 kB
        uint8_t* temp_data  = (uint8_t*) malloc(240 * sizeof(uint8_t));    // 60sec * 1hz = 60 readings * 4 B = 240 B
        int accel_index = 0;
        int temp_index = 0;

        int SECONDS = 0;
        while (SECONDS < 60)   // Number of seconds in a minute, since we need to perform certain actions at 1 Hz
        {
            uint32_t SecondStart = millis();
            int SAMPLES = 0;
            while (SAMPLES < 50)
            {
                uint32_t SampleStart = millis();

                x1_ = Float32toFloat16( MXC4005XC_readX_Axis() );
                y1_ = Float32toFloat16( MXC4005XC_readY_Axis() );
                z1_ = Float32toFloat16( MXC4005XC_readZ_Axis() );

                accel_data[accel_index]   = (uint8_t)(x1_ >> 8);
                accel_data[accel_index+1] = (uint8_t)(x1_ & 0xFF);
                accel_data[accel_index+2] = (uint8_t)(y1_ >> 8);
                accel_data[accel_index+3] = (uint8_t)(y1_ & 0xFF);
                accel_data[accel_index+4] = (uint8_t)(z1_ >> 8);
                accel_data[accel_index+5] = (uint8_t)(z1_ & 0xFF);
                accel_index += 6;

                // printf("%d:%d --- %.2f C, %.2f C, (%.2f, %.2f, %.2f)\n", MINUTES, SECONDS, val1, val2, _x, _y, _z);

                SAMPLES += 1;
                while (millis() - SampleStart < 20); // Wait 20 ms to get 50 Hz sampling rate
            }

            // Get temp values, put in buffer, and increment index
            short val1_16 = Float32toFloat16(Read_Temp_Sensor(1));
            short val2_16 = Float32toFloat16(Read_Temp_Sensor(2));
            temp_data[temp_index]   = (uint8_t)(val1_16 >> 8);
            temp_data[temp_index+1] = (uint8_t)(val1_16 & 0xFF);
            temp_data[temp_index+2] = (uint8_t)(val2_16 >> 8);
            temp_data[temp_index+3] = (uint8_t)(val2_16 & 0xFF);
            temp_index += 4;

            SECONDS += 1;
            while (millis() - SecondStart < 1000);  // Wait 1 second to get 1 Hz sampling rate
        }

        // Save accel data buffer to flash
        w25qxx_basic_write(accel_addr, accel_data, 18000);
        free(accel_data);
        accel_addr += 18000;

        // Write temperature data buffer to flash
        w25qxx_basic_write(temp_addr, temp_data, 240);
        free(temp_data);
        temp_addr += 240;

        printf("MINUTE %d !!!!!!!!!!!!!!\n", MINUTES);
    }

    printf("\nData Collection Complete!\n\n");
}


void Readout(void)
{
    // Print out Accelerometer Data
    uint32_t read_addr = 0x00;
    for (int i=0; i<10; i++)
    {
        /***** Try reading the page you just wrote *****/
        uint32_t read_size = 18000; // Pick a number of bytes to read out.
        uint8_t* data_boi = (uint8_t*) malloc(read_size*sizeof(uint8_t));

        if (w25qxx_basic_read(read_addr, data_boi, read_size) > 0)
        {
            printf("Agh, basic read threw and error :(\n");
        }
        else
        {
            for (int i = 0; i < read_size; i++)
            {
                if (i % 16 == 0)     { printf("\n0x%06X |   ", read_addr + i); } 
                else if (i % 8 == 0) { printf("  "); }
                printf("%02X ", data_boi[i]);
            }
        }
        read_addr += 18000;
        free(data_boi);
    }


    // Print out Temperature data
    read_addr = 0x80000;
    printf("\n\nDELIMITER\n\n");

    for (int i=0; i<10; i++)
    {
        /***** Try reading the page you just wrote *****/
        uint32_t read_size = 240; // Pick a number of bytes to read out.
        uint8_t* data_boi = (uint8_t*) malloc(read_size*sizeof(uint8_t));

        if (w25qxx_basic_read(read_addr, data_boi, read_size) > 0)
        {
            printf("Agh, basic read threw and error :(\n");
        }
        else
        {
            for (int i = 0; i < read_size; i++)
            {
                if (i % 16 == 0)     { printf("\n0x%06X |   ", read_addr + i); } 
                else if (i % 8 == 0) { printf("  "); }
                printf("%02X ", data_boi[i]);
            }
        }
        read_addr += 240;
        free(data_boi);
    }  
}


void collect(void)
{
    printf("\n\n------\nBeginning Data Collection...\n");
    unsigned short x1_, y1_, z1_;
    uint32_t addr = 0x00;

    int index = 0;

    uint32_t SIZE = 18000;

    uint8_t* to_write = (uint8_t*) malloc(SIZE*sizeof(uint8_t));

    for (int j=0; j<SIZE/6; j++)
    {
        StartTime = millis();

        x1_ = Float32toFloat16( (float)MXC4005XC_readX_Axis() );
        y1_ = Float32toFloat16( (float)MXC4005XC_readY_Axis() );
        z1_ = Float32toFloat16( (float)MXC4005XC_readZ_Axis() );

        to_write[index]   = (uint8_t)(x1_ >> 8);
        to_write[index+1] = (uint8_t)(x1_ & 0xFF);
        to_write[index+2] = (uint8_t)(y1_ >> 8);
        to_write[index+3] = (uint8_t)(y1_ & 0xFF);
        to_write[index+4] = (uint8_t)(z1_ >> 8);
        to_write[index+5] = (uint8_t)(z1_ & 0xFF);

        index += 6;
        // w25qxx_basic_write(addr, to_write, 6);
        // while(millis() - StartTime < 200);
    }
    printf("time 0: %d\n", millis());
    w25qxx_basic_write(addr, to_write, 6000);
    printf("time 1: %d\n", millis());
    printf("\nData Collection Complete!\n\n"); 


    /***** Try reading the page you just wrote *****/
    uint32_t read_size = SIZE; // Pick a number of bytes to read out.
    uint32_t read_addr = 0x0;
    uint8_t* data_boi = (uint8_t*) malloc(read_size*sizeof(uint8_t));   // Malloc some space to hold the read data.
    printf("\n\n--\nReading %d bytes from address 0x%06X.\n", read_size, read_addr);
    // Do the read.
    if (w25qxx_basic_read(read_addr, data_boi, read_size) > 0)
    {
        printf("Agh, basic read threw and error :(\n");
    }
    else
    {
        printf("Read %d bytes! They are as follows (in hex):", read_size);
        for (int i = 0; i < read_size; i++)
        {
            if (i % 256 == 0)    { printf("\n\nPage %d:", (read_addr + i)/256); }
            if (i % 16 == 0)     { printf("\n0x%06X |   ", read_addr + i); } 
            else if (i % 8 == 0) { printf("  "); }
            printf("%02X ", data_boi[i]);
        }
        printf("\n");
    }
}


/******************************************************************
 * Erase the entire chip, providing a status update along the way.
******************************************************************/
void Chip_Erase(void)
{
    printf("\n-------\nConducting a chip erase. This may take a few seconds...\n");
    if (w25qxx_basic_chip_erase() > 0)
    {
        printf("Agh, basic chip erase threw and error :(\n");
    }
    else
    {
        printf("Chip erase finished without an error.\n");
    }
}


/*************************************************************
 * Method to test the original Winbond code on Renesas chip.
*************************************************************/
void testing(void)
{
    printf("\n\n\n******************** Renesas Chip SPI w/ Winbond SDK Attempt ********************\n");

    // uint8_t retVal = w25qxx_basic_init(0x1F16, W25QXX_INTERFACE_SPI, W25QXX_BOOL_FALSE);
    // printf("\nInitialization completed with value %d.\n", retVal);


    /***** Try reading the manufacturer and device ID information *****/
    uint8_t man_id = 0;
    uint8_t dev_id = 0;
    printf("\n--\nGetting manufacturer and device ID for Renesas chip...\n");
    w25qxx_basic_get_id(&man_id, &dev_id);
    if (man_id != 0x1F)
    {
        printf("Expected 0x1F for manufacterer ID, but got %X instead!\n", man_id);
    }
    else
    {
        printf("Yay~~ Correct manufacturer ID of 0x%02X!\n", man_id);
    }
    if (dev_id != 0x16)
    {
        printf("Expected 0x16 for device ID, but got %X instead!\n", dev_id);
    }
    else
    {
        printf("Yay~~ Correct device ID of 0x%02X!\n", dev_id);
    }
    

    /***** Try erasing the chip *****/
    printf("\n--\nConducting a chip erase. This may take a few seconds...\n");
    if (w25qxx_basic_chip_erase() > 0)
    {
        printf("Agh, basic chip erase threw and error :(\n");
    }
    else
    {
        printf("Chip erase finished without an error!\n");
    }
    

    /***** Try reading an empty block *****/
    uint32_t read_size = 256; // Pick a number of bytes to read out.
    uint32_t read_addr = 0x0;
    printf("\n--\nReading %d bytes from address 0x%06X.\n", read_size, read_addr);
    uint8_t* data_boi = (uint8_t*) malloc(read_size*sizeof(uint8_t));   // Malloc some space to hold the read data.
    if (data_boi == NULL)
    {
        printf("Failed to allocate memory for data_boi.\n"); // Check if the malloc worked.
        exit(0);
    }
    // Do the read.
    if (w25qxx_basic_read(read_addr, data_boi, read_size) > 0) {
        printf("Agh, basic read threw and error :(\n");
    } 
    else {
        printf("Read %d bytes! They are as follows (in hex):", read_size);
        for (int i = 0; i < read_size; i++) {
            if (i % 256 == 0)    { printf("\n\nPage %d:", (read_addr + i)/256); }
            if (i % 16 == 0)     { printf("\n0x%06X |   ", read_addr + i); }
            else if (i % 8 == 0) { printf("  "); } 
            printf("%02X ", data_boi[i]);
        }
        printf("\n");
    }


    /***** Try writing to a page *****/
    uint32_t write_size = 200;
    printf("\n--\nWriting %d bytes to address 0x%06X.\n", write_size, read_addr);
    // Fill our data_boi buffer with some recognizable values.
    for (int i = 0; i < write_size; i++) {
        switch(i % 4) {
            case 0:
                data_boi[i] = 0xDE;
                break;
            case 1:
                data_boi[i] = 0xAD;
                break;
            case 2:
                data_boi[i] = 0xBE;
                break;
            case 3:
                data_boi[i] = 0xEF;
                break;
        }
    }

    // // Do the write. 
    if (w25qxx_basic_write(read_addr, data_boi, write_size) > 0) {
        printf("Agh, basic write threw an error :(\n");
    }
    else {
        printf("Write finished without an error!\n");
    }


    /***** Try reading the page you just wrote *****/
    printf("\n--\nReading %d bytes from address 0x%06X.\n", read_size, read_addr);
    // Do the read.
    if (w25qxx_basic_read(read_addr, data_boi, read_size) > 0) {
        printf("Agh, basic read threw and error :(\n");
    } 
    else
    {
        printf("Read %d bytes! They are as follows (in hex):", read_size);
        for (int i = 0; i < read_size; i++)
        {
            if (i % 256 == 0)    { printf("\n\nPage %d:", (read_addr + i)/256); }
            if (i % 16 == 0)     { printf("\n0x%06X |   ", read_addr + i); } 
            else if (i % 8 == 0) { printf("  "); }
            printf("%02X ", data_boi[i]);
        }
        printf("\n");
    }

    return;
}


/******************************************************************
 * Read from 0x00 to get two bytes of temperature from the sensor
******************************************************************/
float Read_Temp_Sensor(int which)
{
    uint8_t txdata[] = { 0x00 }; // get temperature read back

    int sensor_addr;
    if (which==1) { sensor_addr = TEMP_SENSOR_1; }
    else          { sensor_addr = TEMP_SENSOR_2; }

    Wire_beginTransmission( sensor_addr );
    Wire_write(txdata, 1);
    Wire_endTransmission();

    uint8_t rxdata[] = {};
    Wire_requestFromAndRead(rxdata, sensor_addr, 2);

    float temp = (float)rxdata[0];      // Convert integer part to float

    int8_t placeholder = rxdata[1]>>4;  // Add decimal part in most significant four bits
    temp += (float)placeholder / 16;    // LSB is 0.0625 (1/16) degrees C

    return temp;
}


/**********************************************************************
 * Converts a float (32 bits) into memory space of short type (int16)
**********************************************************************/
short Float32toFloat16(float input)
{
    // Copy Float bits into Int32
    uint32_t input_bits;
    memcpy(&input_bits, &input, sizeof(input));
    
    // StackOverflow magic to convert Float32 to Float16 using fast operations
    unsigned short fltInt16;
    fltInt16 = (input_bits >> 31) << 5;
    unsigned short tmp = (input_bits >> 23) & 0xFF;
    tmp = (tmp - 0x70) & ((unsigned int)((int)(0x70 - tmp) >> 4) >> 27);
    fltInt16 = (fltInt16 | tmp) << 10;
    fltInt16 |= (input_bits >> 13) & 0x3FF;

    return fltInt16;
}

void delay(int milliseconds)
{
    MXC_Delay(milliseconds * 1000);
    return;
}

void writeLEDLow()
{
    MXC_GPIO_OutClr(eLED.port, eLED.mask);
}

void writeLEDHigh()
{
    MXC_GPIO_OutSet(eLED.port, eLED.mask);
}

void writeI_LEDLow()
{
    MXC_GPIO_OutClr(iLED.port, iLED.mask);
}

void writeI_LEDHigh()
{
    MXC_GPIO_OutSet(iLED.port, iLED.mask);
}
