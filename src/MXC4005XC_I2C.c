#include "MXC4005XC_I2C.h"
#include "Wire.h"
#include <stdint.h>


/*****************************************************************************
 * Sets up stuff, including what the full-scale acceleration possible is...
*****************************************************************************/
void MXC4005XC_config(uint8_t FSR_range)
{
  uint8_t set_range;
  if (FSR_range == 4)       set_range = MXC4005XC_FSR_4G;
  else if (FSR_range == 8)  set_range = MXC4005XC_FSR_8G;
  else                      set_range = MXC4005XC_FSR_2G;

  uint8_t config[] = { MXC4005XC_CONTROL, set_range };

  Wire_beginTransmission(MXC4005XC_ADDRESS);
  Wire_write(config, 2);
  Wire_endTransmission();
}


/***************************************************************************
 * Performs 2 register reads and combines/converts to g's
 * Returns a float that represents acceleration in terms of gravity (g's)
***************************************************************************/
float MXC4005XC_readX_Axis(void)
{
  uint8_t txdata[] = { MXC4005XC_XOUT_UPPER };
  Wire_beginTransmission(MXC4005XC_ADDRESS);
  Wire_write(txdata, 1);
  Wire_endTransmission();

  uint8_t rxdata[] = { 0 };
  Wire_requestFromAndRead(rxdata, MXC4005XC_ADDRESS, 1);

  int16_t xaxis = rxdata[0];
  xaxis <<= 8;

  txdata[0] = MXC4005XC_XOUT_LOWER;
  Wire_beginTransmission(MXC4005XC_ADDRESS);
  Wire_write(txdata, 1);
  Wire_endTransmission();

  rxdata[0] = 0;
  Wire_requestFromAndRead(rxdata, MXC4005XC_ADDRESS, 1);

  xaxis |= rxdata[0];
  xaxis >>= 4;

  return xaxis/512.0;
}


/**************************************************************************
 * Performs 2 register reads and combines/converts to g's
 * Returns a float that represents acceleration in terms of gravity (g's)
**************************************************************************/
float MXC4005XC_readY_Axis(void)
{
  uint8_t txdata[] = { MXC4005XC_YOUT_UPPER };
  Wire_beginTransmission(MXC4005XC_ADDRESS);
  Wire_write(txdata, 1);
  Wire_endTransmission();

  uint8_t rxdata[] = { 0 };
  Wire_requestFromAndRead(rxdata, MXC4005XC_ADDRESS, 1);

  int16_t yaxis = rxdata[0];
  yaxis <<= 8;

  txdata[0] = MXC4005XC_YOUT_LOWER;
  Wire_beginTransmission(MXC4005XC_ADDRESS);
  Wire_write(txdata, 1);
  Wire_endTransmission();

  rxdata[0] = 0;
  Wire_requestFromAndRead(rxdata, MXC4005XC_ADDRESS, 1);

  yaxis |= rxdata[0];
  yaxis >>= 4;

  return yaxis/512.0;
}


/**************************************************************************
 * Performs 2 register reads and combines/converts to g's
 * Returns a float that represents acceleration in terms of gravity (g's)
**************************************************************************/
float MXC4005XC_readZ_Axis(void)
{
  uint8_t txdata[] = { MXC4005XC_ZOUT_UPPER };
  Wire_beginTransmission(MXC4005XC_ADDRESS);
  Wire_write(txdata, 1);
  Wire_endTransmission();

  uint8_t rxdata[] = {0};
  Wire_requestFromAndRead(rxdata, MXC4005XC_ADDRESS, 1);

  int16_t zaxis = rxdata[0];
  zaxis <<= 8;

  txdata[0] = MXC4005XC_ZOUT_LOWER;
  Wire_beginTransmission(MXC4005XC_ADDRESS);
  Wire_write(txdata, 1);
  Wire_endTransmission();

  rxdata[0] = 0;
  Wire_requestFromAndRead(rxdata, MXC4005XC_ADDRESS, 1);

  zaxis |= rxdata[0];
  zaxis >>= 4;

  return zaxis/512.0;
}
