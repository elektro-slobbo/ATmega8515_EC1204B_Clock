/******************************
 * file name: rtc.h
 ******************************/
#ifndef RTC_H
#define RTC_H
 
//Data type to hold calendar/clock data
typedef struct
{
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t date;
    uint8_t month;
    uint8_t day;
    uint8_t year;
} dateTime;
 
/*******************************************************************
  Interface function to initialize RTC: 1. Disable Clock Halt
                                        2. Set to 24 hour mode
                                        3. Disable Write Protection
  No Calendar/Clock will be changed
********************************************************************/
void rtc_init(void);
 
//Interface function to read Calendar/Clock value
dateTime get_date_time(void);
 
//Interface function to set Calendar/Clock value
void set_date_time(dateTime dt);
 
#endif
