	
/**************************
 * file name: rtc.c
 **************************/
  
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include "rtc.h"
 
//Strobe "pin" on "port" high
#define IO_PIN_STROBE_HIGH(port, pin)   \
        __asm__ __volatile__ (          \
        "sbi %0, %1"                    \
        : /* no output */               \
        : "I" (_SFR_IO_ADDR(port)),     \
          "I" (pin))
 
//Strobe "pin" on "port" low
#define IO_PIN_STROBE_LOW(port, pin)    \
        __asm__ __volatile__ (          \
        "cbi %0, %1"                    \
        : /* no output */               \
        : "I" (_SFR_IO_ADDR(port)),     \
          "I" (pin))
 
//Timing requirement between data output and clock stobe high
#define DATA_TO_CLK_SETUP()     \
        __asm__ __volatile__ (  \
        "nop\n\t"               \
        "nop\n\t"               \
        ::)
 
//Strobe CE pin of DS1302 high and low
#define CE_STROBE_HIGH() IO_PIN_STROBE_HIGH(PORTB, 1)
#define CE_STROBE_LOW() IO_PIN_STROBE_LOW(PORTB, 1)
 
//Strobe SCLK pin of DS1302 high and low
#define SCLK_STROBE_HIGH() IO_PIN_STROBE_HIGH(PORTB, 3)
#define SCLK_STROBE_LOW() IO_PIN_STROBE_LOW(PORTB, 3)
 
//Strobe IO pin of DS1302 high and low
#define IO_STROBE_HIGH() IO_PIN_STROBE_HIGH(PORTB, 2)
#define IO_STROBE_LOW() IO_PIN_STROBE_LOW(PORTB, 2)
 
//Calendar/Clock burst read and write command
#define DT_BURST_READ 0xbf
#define DT_BURST_WRITE 0xbe
 
//RTC register seconds byte read and write command
#define DT_SECONDS_READ 0x81
#define DT_SECONDS_WRITE 0x80
 
//RTC register hours byte read and write command
#define DT_HOURS_READ 0x85
#define DT_HOURS_WRITE 0x84
 
//RTC register WP write
#define DT_WP_WRITE 0x8e
 
//Configure port pin directions for read and write DS1302
#define WRITE_MODE() (DDRB |= 0x0e)
#define READ_MODE() (DDRB = (DDRB|0x0e)&0xfb)
//Disable io pull-up in read mode
#define DISABLE_IO_PULLUP() (PORTB &= 0xfb)
//Read i/o value from DS1302
#define IO_READ() (PINB & 0x04)

 
//Prepare CE and SCLK for new operation
static void reset(void)
{
    //Pull both CE and SCLK low to start with  
    SCLK_STROBE_LOW();
    CE_STROBE_LOW();
 
    //Comms. begin with CE stobe high
    CE_STROBE_HIGH();
}
 
//Read one byte of Calendar/Clock data
static uint8_t read_byte(void)
{
    uint8_t byte = 0;
    uint8_t i;
 
    //Port pins in read mode for data read
    READ_MODE();
    //Disable internal I/O pull-up
    DISABLE_IO_PULLUP();
 
    //Read one byte of Calendar/Clock data
    for(i = 0; i != 8; ++i)
    {
        //Strobe SCLK low to read I/O
        SCLK_STROBE_LOW();
        _delay_us(1);
 
        if(IO_READ() != 0)
        {
            byte |= 1<<i;
        }
        //Strobe SCLK high for next I/O read
        SCLK_STROBE_HIGH();
        _delay_us(1);
    }
 
    return byte;
}
 
//Write one byte of control or Calendar/Clock data
static void write_byte(uint8_t byte)
{
    uint8_t i;
 
    //Port pins in write mode
    WRITE_MODE();
 
    //Write one byte of control or Calendar/Clock data
    for(i = 0; i != 8; ++i)
    {
        //Start clock cycle with SCLK low
        SCLK_STROBE_LOW();
        _delay_us(1);
 
        //Write bit value to I/O pin of DS1302         
        if(((1<<i)&byte) == 0)
        {
            IO_STROBE_LOW();
        }
        else
        {
            IO_STROBE_HIGH();
        }
 
        DATA_TO_CLK_SETUP(); //Data to clock setup
 
        //End clock cycle with SCLK high
        SCLK_STROBE_HIGH();
        _delay_us(1);      
    }
     
}
 
//Read 7 bytes of Calendar/Clock data
static dateTime read_dt_block(void)
{
    uint8_t dt_byte;
    uint8_t byte_pos;
    dateTime dt = {0};
 
    //Always do a reset before a new operation
    reset();
 
    //Write the clock burst read command into DS1302
    write_byte(DT_BURST_READ);
 
    //Read each of the 7 Calendar/Clock bytes from DS1302
    for(byte_pos = 0; byte_pos != 7; ++byte_pos)
    {
        //Read one byte of calendar/clock data
        dt_byte = read_byte();
         
        //Copy the read byte to the right place
        switch(byte_pos)
        {
            case 0:
                        dt.second = dt_byte;
                        break;
            case 1:
                        dt.minute = dt_byte;
                        break;
            case 2:
                        dt.hour = dt_byte;
                        break;
            case 3:
                        dt.date = dt_byte;
                        break;
            case 4:
                        dt.month = dt_byte;
                        break;
            case 5:
                        dt.day = dt_byte;
                        break;
            case 6:
                        dt.year = dt_byte;
                        break;
        }
    }
 
    //Always end an operation with a reset
    reset();
    return dt;
}
 
//Write 8 bytes of Calendar/Clock data
static void write_dt_block(dateTime dt)
{
    uint8_t dt_byte;
    uint8_t byte_pos;
 
    //Always do a reset before a new operation
    reset();
 
    //Write burst write command byte to DS1302
    write_byte(DT_BURST_WRITE);
 
    //Write each of the 7 Calendar/Clock byte to DS1302
    for(byte_pos = 0; byte_pos != 7; ++byte_pos)
    {
        //Copy the right byte to write
        switch(byte_pos)
        {
            case 0:
                        dt_byte = dt.second;
                        break;
            case 1:
                        dt_byte = dt.minute;
                        break;
            case 2:
                        dt_byte = dt.hour;
                        break;
            case 3:
                        dt_byte = dt.date;
                        break;
            case 4:
                        dt_byte = dt.month;
                        break;
            case 5:
                        dt_byte = dt.day;
                        break;
            case 6:
                        dt_byte = dt.year;
                        break;
        }
 
        //Write one byte of Calendar/Clock data
        write_byte(dt_byte);
    }
 
    //Must write the 8th byte of the Calendar/Clock register
    write_byte(0);
 
    //Always end an operation with a reset
    reset();
}
 
/*******************************************************************
  Interface function to initialize RTC: 1. Disable Clock Halt
                                        2. Set to 24 hour mode
                                        3. Disable Write Protection
  No Calendar/Clock will be changed
********************************************************************/
void rtc_init(void)
{
    uint8_t byte_second;
    uint8_t byte_hour;
 
    //Disable Clock Halt
    reset();
    write_byte(DT_SECONDS_READ);
    byte_second = read_byte();
    reset();
    write_byte(DT_SECONDS_WRITE);
    write_byte(byte_second & 0x7f);
    reset();
 
    //Set to 24 hour mode
    write_byte(DT_HOURS_READ);
    byte_hour = read_byte();
    reset();
    write_byte(DT_HOURS_WRITE);
    write_byte(byte_hour & 0x7f);
    reset();
 
    //Disable Write Protection
    write_byte(DT_WP_WRITE);
    write_byte(0);
    reset();
}
 
//Interface function to read Calendar/Clock value
dateTime get_date_time(void)
{
    dateTime dt;
     
    //Read raw calendar/clock block from DS1302
    dt = read_dt_block();
 
    /*************************************************************
     Convert from the raw BCD Calendar/Clock data to normal decimal
     values. Hour is treated differently in 24 and AM/PM mode.
     Also the day of week is left as is.
    **************************************************************/
    dt.second = (((dt.second&0x70)>>4)*10) + (dt.second&0x0f);
    dt.minute = (((dt.minute&0x70)>>4)*10) + (dt.minute&0x0f);
    if((dt.hour&0x80) == 0)
    {
        dt.hour = (((dt.hour&0x30)>>4)*10) + (dt.hour&0x0f);
    }  
    dt.date = (((dt.date&0x30)>>4)*10) + (dt.date&0x0f);
    dt.month = (((dt.month&0x10)>>4)*10) + (dt.month&0x0f);
    dt.year = (((dt.year&0xf0)>>4)*10) + (dt.year&0x0f);
 
    return dt;
}
 
//Interface function to set Calendar/Clock value
void set_date_time(dateTime dt)
{
    /**************************************************************
     Convert from normal decimal Calendar/Clock value to BCD. Hour
     is treated differently in 24 and AM/PM mode. Also the day of
     week is left as is.
    ***************************************************************/   
    dt.second = 0;
    //dt.second = ((dt.second/10)<<4) | (dt.second%10);
    dt.minute = ((dt.minute/10)<<4) | (dt.minute%10);
    if((dt.hour&0x80) == 0)
    {
        dt.hour = ((dt.hour/10)<<4) | (dt.hour%10);
    }
    dt.date =  ((dt.date/10)<<4)  | (dt.date%10);
    dt.month = ((dt.month/10)<<4) | (dt.month%10);
    dt.year =  ((dt.year/10)<<4)  | (dt.year%10);
 
    write_dt_block(dt);
}
