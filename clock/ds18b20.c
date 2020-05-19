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


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "ds18b20.h"
#include "rtc.h"


/*
 * ds18b20 init
 */
uint8_t ds18b20_reset() {
	uint8_t i;

	//low for 480us
	DS18B20_PORT &= ~ (1<<DS18B20_DQ); //low
	DS18B20_DDR |= (1<<DS18B20_DQ); //output
	_delay_us(480); //550 us instead of 480

	//release line and wait for 60uS
	DS18B20_DDR &= ~(1<<DS18B20_DQ); //input
	_delay_us(60);

	//get value and wait 240us
	i = (DS18B20_PIN & (1<<DS18B20_DQ));
	_delay_us(420); //480 instead of 420

	//return the read value, 0=ok, 1=error
	return i;
}

/*
 * write one bit
 */
void ds18b20_writebit(uint8_t bit){
	cli();
	//low for 1uS
	DS18B20_PORT &= ~ (1<<DS18B20_DQ); //low
	DS18B20_DDR |= (1<<DS18B20_DQ); //output
	_delay_us(1); 

	//if we want to write 1, release the line (if not will keep low)
	if(bit)
		DS18B20_DDR &= ~(1<<DS18B20_DQ); //input

	//wait 60uS and release the line
	_delay_us(60); //80 instead of 60
	DS18B20_DDR &= ~(1<<DS18B20_DQ); //input
	sei();
}

/*
 * read one bit
 */
uint8_t ds18b20_readbit(void){
	uint8_t bit=0;
	cli();
	//low for 1uS
	DS18B20_PORT &= ~ (1<<DS18B20_DQ); //low
	DS18B20_DDR |= (1<<DS18B20_DQ); //output
	_delay_us(1); //2 instead of 1

	//release line and wait for 14uS
	DS18B20_DDR &= ~(1<<DS18B20_DQ); //input
	_delay_us(14);

	//read the value
	if(DS18B20_PIN & (1<<DS18B20_DQ))
		bit=1;

	//wait 45uS and return read value
	_delay_us(45);
	sei();
	return bit;
}

uint8_t ds18b20_bitio( uint8_t b )
{
  cli();
  DS18B20_DDR |= 1<<DS18B20_DQ;
  _delay_us(1);
  if( b )
    DS18B20_DDR &= ~(1<<DS18B20_DQ);
  _delay_us(14);
  if( (DS18B20_PIN & (1<<DS18B20_DQ)) == 0 )
    b = 0;
  _delay_us(45);
  DS18B20_DDR &= ~(1<<DS18B20_DQ);
  sei();
  return b;
}

/*
 * write one byte
 */
void ds18b20_writebyte(uint8_t byte){
	uint8_t i=8;
	while(i--){
		ds18b20_writebit(byte&1);
		byte >>= 1;
	}
}

/*
 * read one byte
 */
uint8_t ds18b20_readbyte(void){
	uint8_t i=8, n=0;
	while(i--){
		n >>= 1;
		n |= (ds18b20_readbit()<<7);
	}
	return n;
}

uint8_t ds18b20_rom_search( uint8_t diff, uint8_t *id )
{
	uint8_t i, j, next_diff;
	uint8_t b;
	
	if( ds18b20_reset() ) 
	{
		return DS18B20_PRESENCE_ERR;         // error, no device found <--- early exit!
	}
	
	ds18b20_writebyte( DS18B20_CMD_SEARCHROM );        // ROM search command
	next_diff = DS18B20_LAST_DEVICE;         // unchanged on last device
	
	i = DS18B20_ROMCODE_SIZE * 8;            // 8 bytes
	
	do 
	{
		j = 8;                          // 8 bits
		do 
		{
			b = ds18b20_bitio(1);         // read bit
			if( ds18b20_bitio(1) ) 
			{      // read complement bit
				if( b ) 
				{			               // 0b11
					return DS18B20_DATA_ERR; // data error <--- early exit!
				}
			}
			else 
			{
				if( !b ) 
					{              // 0b00 = 2 devices
					if( diff > i || ((*id & 1) && diff != i) ) {
						b = 1;          // now 1
						next_diff = i;  // next pass 0
					}
				}
			}
			ds18b20_writebit( b );             // write bit
			*id >>= 1;
			if( b ) 
			{
				*id |= 0x80;            // store bit
			}
			
			i--;
			
		} 
		while( --j );
		
		id++;                           // next byte
	
	} 
	while( i );
	
	return next_diff;                   // to continue search
}


/* find DS18X20 Sensors on 1-Wire-Bus
   input/ouput: diff is the result of the last rom-search
                *diff = DS18B20_SEARCH_FIRST for first call
   output: id is the rom-code of the sensor found */
uint8_t DS18X20_find_sensor( uint8_t *diff, uint8_t id[] )
{
	uint8_t go;
	uint8_t ret;

	ret = DS18X20_OK;
	go = 1;
	do 
	{
		*diff = ds18b20_rom_search( *diff, &id[0] );
		if ( *diff == DS18B20_PRESENCE_ERR || *diff == DS18B20_DATA_ERR ||
		     *diff == DS18B20_LAST_DEVICE ) 
		{ 
			go  = 0;
			ret = DS18X20_ERROR;
		} else 
		{
			if ( id[0] == DS18B20_FAMILY_CODE || id[0] == DS18S20_FAMILY_CODE ||
			     id[0] == DS1822_FAMILY_CODE ) 
			{ 
				go = 0;
			}
		}
	} 
	while (go);

	return ret;
}



/*
 * get temperature for 1 sensor on the wire.
 */
//double ds18b20_gettemp() { //if we ever need floating point precision)
uint8_t ds18b20_gettemp() {
	uint8_t temperature[2];
	int8_t digit;
	uint16_t decimal;
	//double retd = 0;

	ds18b20_reset(); //reset
	ds18b20_writebyte(DS18B20_CMD_SKIPROM); 	//skip ROM
	ds18b20_writebyte(DS18B20_CMD_CONVERTTEMP); //start temperature conversion

	// while(!ds18b20_readbit()); 					//wait until conversion is complete

	ds18b20_reset(); //reset
	ds18b20_writebyte(DS18B20_CMD_SKIPROM); 	//skip ROM
	ds18b20_writebyte(DS18B20_CMD_RSCRATCHPAD); //read scratchpad

	//read 2 byte from scratchpad
	temperature[0] = ds18b20_readbyte();
	temperature[1] = ds18b20_readbyte();

	ds18b20_reset(); //reset

	//store temperature integer digits
	digit = temperature[0]>>4;
	digit |= (temperature[1]&0x7)<<4;

	//store temperature decimal digits
	decimal = temperature[0]&0xf;
	decimal *= DS18B20_DECIMALSTEPS;

	//compose the double temperature value and return it
	//retd = digit + decimal * 0.0001;

	//return retd;
	return digit;
}


void DS12B80_command( uint8_t command, uint8_t *id)
{
	uint8_t i;

	ds18b20_reset();
	ds18b20_writebyte( DS18B20_CMD_MATCHROM );     // to a single device
	i = DS18B20_ROMCODE_SIZE;
	do 
	{
		ds18b20_writebyte( *id );
		id++;
	} 
	while( --i );
	ds18b20_writebyte( command );
}

uint8_t read_scratchpad( uint8_t id[], uint8_t sp[], uint8_t n )
{
	uint8_t i;
	uint8_t ret;

	DS12B80_command( DS18B20_CMD_RSCRATCHPAD, id );
	for ( i = 0; i < n; i++ ) 
	{
		sp[i] = ds18b20_readbyte();
	}
		ret = DS18X20_OK;

	return ret;
}


//get temperature ifmultiple sensors on the wire
uint8_t ds18b20_getindextemp( uint8_t id[] )
{
	int8_t digit=0;
	uint16_t decimal;

	uint8_t sp[DS18X20_SP_SIZE];
	uint8_t ret;
	
	ds18b20_reset();
	DS12B80_command( DS18B20_CMD_CONVERTTEMP, id );
	ret = read_scratchpad( id, sp, DS18X20_SP_SIZE );
	if ( ret == DS18X20_OK ) 
	{
	
		//store temperature integer digits
	digit = sp[0]>>4;
	digit |= (sp[1]&0x7)<<4;

	//store temperature decimal digits
	decimal = sp[0]&0xf;
	decimal *= DS18B20_DECIMALSTEPS;

	}
	return digit;
}
