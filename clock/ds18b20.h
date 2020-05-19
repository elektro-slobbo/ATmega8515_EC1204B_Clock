/*
Cristian Copcea 2015

Original work from Davide Gironi 2012:

Released under GPLv3.
Please refer to LICENSE file for licensing information.

References:
  + Using DS18B20 digital temperature sensor on AVR microcontrollers
    by Gerard Marull Paretas, 2007
    http://teslabs.com/openplayer/docs/docs/other/ds18b20_pre1.pdf


Reworked using info from Martin Thomas
Other information from MAXIM's ibutton datasheet

*/


#ifndef DS18B20_H_
#define DS18B20_H_

#include <avr/io.h>

//setup connection
#define DS18B20_PORT PORTB
#define DS18B20_DDR DDRB
#define DS18B20_PIN PINB
#define DS18B20_DQ PB0

//commands
#define DS18B20_CMD_CONVERTTEMP 	0x44
#define DS18B20_CMD_RSCRATCHPAD 	0xBE
#define DS18B20_CMD_WSCRATCHPAD 	0x4E
#define DS18B20_CMD_CPYSCRATCHPAD 	0x48
#define DS18B20_CMD_RECEEPROM 		0xB8
#define DS18B20_CMD_RPWRSUPPLY 		0xB4
#define DS18B20_CMD_SEARCHROM 		0xF0
#define DS18B20_CMD_READROM 		0x33
#define DS18B20_CMD_MATCHROM		0x55
#define DS18B20_CMD_SKIPROM			0xCC
#define DS18B20_CMD_ALARMSEARCH		0xEC

#define DS18B20_SEARCH_FIRST		0xFF        // start new search
#define DS18B20_PRESENCE_ERR		0xFF
#define DS18B20_DATA_ERR			0xFE
#define DS18B20_LAST_DEVICE		0x00        // last device found
// rom-code size including CRC
#define DS18B20_ROMCODE_SIZE		8
#define DS18X20_OK					0x00
#define DS18X20_ERROR				0x01
#define DS18X20_START_FAIL			0x02
#define DS18X20_ERROR_CRC			0x03

#define DS18X20_INVALID_DECICELSIUS  2000

#define DS18X20_POWER_PARASITE		0x00
#define DS18X20_POWER_EXTERN		0x01

#define DS18X20_CONVERSION_DONE		0x00
#define DS18X20_CONVERTING			0x01

/* DS18X20 specific values (see datasheet) */
#define DS18S20_FAMILY_CODE			0x10
#define DS18B20_FAMILY_CODE			0x28
#define DS1822_FAMILY_CODE			0x22
//scratchpad size in bytes
#define DS18X20_SP_SIZE           	9


//decimal conversion table
#define DS18B20_DECIMALSTEPS_9BIT  	5000 //0.5
#define DS18B20_DECIMALSTEPS_10BIT 	2500 //0.25
#define DS18B20_DECIMALSTEPS_11BIT 	1250 //0.125
#define DS18B20_DECIMALSTEPS_12BIT 	625  //0.0625
#define DS18B20_DECIMALSTEPS DS18B20_DECIMALSTEPS_12BIT

//functions
//extern double ds18b20_gettemp();
extern uint8_t ds18b20_gettemp();
extern uint8_t ds18b20_getindextemp();
extern uint8_t ds18b20_reset();
extern uint8_t DS18X20_find_sensor();



#endif
