//********************************************
//* (c) 2015 Cristian Copcea                 *
//* Please retain here the credits           *
//* just add yours if you make modifications *
//* to this file                             *
//********************************************
#include <util/delay.h>
#include "ds1621.h"
#include "i2cmaster/i2cmaster.h"


uint8_t get_ds1621_temperature(void) 
	{
  
  unsigned char msb,lsb;
  // Set sensor config
  i2c_start_wait(DS1621_W);
  i2c_write(ACCESS_CONFIG);
  i2c_write(0x00);	//continuous temperature conversion
  // Start temp conversion
  i2c_rep_start(DS1621_W);
  i2c_write(START_CONVERT);
  i2c_stop();
  // Read the last measured value
  i2c_start_wait(DS1621_W);
  i2c_write(READ_TEMP);
  i2c_rep_start(DS1621_R);
  msb = i2c_readAck();
  lsb = i2c_readNak();
  i2c_stop();
  return msb;
}
