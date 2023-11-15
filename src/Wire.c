/******************************
 * Wire.c
 *     Created on: Dec 6, 2021
 *         Author: Henry
 *         Editor: Trevor
******************************/

/*---------- Dependent Includes ----------*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_errors.h"
#include "nvic_table.h"
#include "i2c_regs.h"
#include "i2c.h"
#include "spi.h"


/*---------- Global Functions ----------*/
#define I2C_MASTER      MXC_I2C1
#define I2C_FREQ        100000

mxc_i2c_req_t reqMaster;

static const mxc_i2c_req_t emptyI2Creq;

typedef enum{
  FAILED,
  PASSED
}test_t;


/*************************
 * Begin I2C connection
*************************/
int Wire_begin()
{
  int error;
  error = MXC_I2C_Init(I2C_MASTER, 1, 0);
  if (error != E_NO_ERROR)
  {
    printf("Wire_begin() error: %d\n", error);
    return FAILED;
  }

  MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);
  reqMaster.i2c = I2C_MASTER;
  reqMaster.restart = 0;

  return 0;
}


/*************************************************
 * Begin Transmission to a specific Wire Address
*************************************************/
void Wire_beginTransmission(int address)
{
	reqMaster.addr = address;
	return;
}


/**************************************************************************************************************************
 * takes an unsized array and passes the pointer to tx_buf variable on the i2c struct along with the length of the array
**************************************************************************************************************************/
void Wire_write(uint8_t transmitData[], int length)
{
  reqMaster.tx_buf = transmitData;
  reqMaster.tx_len = length;
  reqMaster.rx_len = 0;
}


/******************************************
 * Function to end I2C Wire Transmission
******************************************/
int Wire_endTransmission()
{
	int error = MXC_I2C_MasterTransaction(&reqMaster);
	reqMaster = emptyI2Creq;
	reqMaster.i2c = I2C_MASTER;
	return error;
}


/****************************************
 * Request from a specific Wire Address
****************************************/
int Wire_requestFromAndRead(uint8_t *buffer, int address, int quantity)
{
	reqMaster.addr = address;
  reqMaster.rx_buf = buffer;
  reqMaster.rx_len = quantity;
  reqMaster.tx_len = 0;
  reqMaster.restart = 0;

	int error = MXC_I2C_MasterTransaction(&reqMaster);

	reqMaster = emptyI2Creq;
	reqMaster.i2c = I2C_MASTER;
	return error;
}


int Wire_shutdown()
{
  int error = MXC_I2C_Shutdown(I2C_MASTER);
  if (error != E_NO_ERROR)
  {
    printf("Wire_shutdown() Error: %d\n", error);
    return FAILED;
  }
  return error;
}


/****************************************************************/
/********************	SPI CONNECTION SECTION ********************/

#define SPI_MASTER      MXC_SPI0
#define SPI_FREQ        100000
#define SPI_MODE        1         // 0 for Slave, 1 for Master
#define NUM_SLAVES      1

volatile int SPI_FLAG;


int SPI_Begin() 
{
  int error = MXC_SPI_Init(SPI_MASTER, SPI_MODE, 0, NUM_SLAVES, 0, SPI_FREQ);
  if (error != E_NO_ERROR)
  {
    printf("Failed to initialize SPI. (Wire.c)");
    return error;
  }
  else
    return 0;
}


void SPI_Callback(mxc_spi_req_t *req, int error)
{
  SPI_FLAG = error;
}

/****************************************************************/
