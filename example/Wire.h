#ifndef WIRE_H_
#define WIRE_H_

#include "Wire.h"

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

int Wire_begin();

void Wire_beginTransmission(int address);

int Wire_endTransmission();

void Wire_write(uint8_t transmitData[], int length);

int Wire_requestFromAndRead(uint8_t *buffer, int address, int quantity);

int Wire_shutdown();

int SPI_Begin();

void SPI_Callback(mxc_spi_req_t *req, int error);


#endif /* WIRE_H_ */
