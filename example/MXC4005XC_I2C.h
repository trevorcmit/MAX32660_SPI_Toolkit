#ifndef MXC4005XC_I2C_H_
#define MXC4005XC_I2C_H_

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


/******************************************/
#define MXC4005XC_ADDRESS       0x15
#define MXC4005XC_INT_SRC0      0X00
#define MXC4005XC_INT_CLR0      0X00
#define MXC4005XC_INT_SRC1      0X01
#define MXC4005XC_INT_CLR1      0X01
#define MXC4005XC_STATUS        0X02
#define MXC4005XC_XOUT_UPPER    0X03
#define MXC4005XC_XOUT_LOWER    0X04
#define MXC4005XC_YOUT_UPPER    0X05
#define MXC4005XC_YOUT_LOWER    0X06
#define MXC4005XC_ZOUT_UPPER    0X07
#define MXC4005XC_ZOUT_LOWER    0X08
#define MXC4005XC_TOUT          0X09
#define MXC4005XC_INT_MASK0     0X0A
#define MXC4005XC_INT_MASK1     0X0B
#define MXC4005XC_DETECTION     0X0C
#define MXC4005XC_CONTROL       0X0D
#define MXC4005XC_DEVICE_ID     0X0E    // should always be value of 0x02
#define MXC4005XC_WHO_AM_I      0X0F    // 8-bit version code programmed in factory, unique
/******************************************/

#define MXC4005XC_FSR_2G        (0)
#define MXC4005XC_FSR_4G        (1 << 5)
#define MXC4005XC_FSR_8G        (1 << 6)

#define MXC4005XC_POWEROFF      (1)
// MXC4005XC accelerometer powers up when CONTROL register has bit 0 set to 0, so just set the FSR range again

/******************************************/


/***************************************************************************
 * Sets up stuff, including what the full-scale acceleration possible is...
***************************************************************************/
void MXC4005XC_config(uint8_t FSR_range);


/***************************************************************************
 * Performs 2 register reads and combines/converts to g's
 * Returns a float that represents acceleration in terms of gravity (g's)
***************************************************************************/
float MXC4005XC_readX_Axis(void);


/***************************************************************************
 * Performs 2 register reads and combines/converts to g's
 * Returns a float that represents acceleration in terms of gravity (g's)
***************************************************************************/
float MXC4005XC_readY_Axis(void);


/***************************************************************************
 * Performs 2 register reads and combines/converts to g's
 * Returns a float that represents acceleration in terms of gravity (g's)
***************************************************************************/
float MXC4005XC_readZ_Axis(void);


#endif /* WIRE_H_ */
