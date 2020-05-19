//********************************************
//*  Base code from Cristian Copcea (2015)   *
//*  Extended 2017 by s.slobbo@gmail.com:		*
//*  - count days between two dates				*
//*  - support for nerf detection				*
//*  - leds show hour+minute (mode 10-13)		*
//*  - new mode leds off (mode 0)				*
//*  - stable pullup PD2+3							*
//*  - 2 CRC: one for Alarm						*
//*  - show date DD/MM for EU mode				*
//*  - eggtimer										*
//*  - #defines for functions/modules			*
//*  - deactivate multi temp sensor support	*
//*  - set LED mode via mode button now		*
//*  - dimmer for display and leds				*
//*  - set timer faster for better dimming   *
//*  - new (random) led modes (14+15)			*
//*														*
//*  Update 2020:										*
//*  Show time 10s, others shorter (3s)		*
//*														*
//* Please retain here the credits           *
//* just add yours if you make modifications *
//********************************************

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
//#define F_CPU 12000000L
#include <util/delay.h>
// 03.11.2018 -> HZ Ha&Au

//#include "i2cmaster/i2cmaster.h"
#include "ds1621.h"
#include "ds18b20.h"
#include "rtc.h"

#define TEMPCORRECTION	3

// turn on/off used modules here
//#define MULTI_TEMPSENSORS
//#define EGGTIMER_MODULE
#define NERFGUN_MODULE
#define DIFFDATE_MODULE
#define LEDS_SHOW_HOURMIN
#define LEDS_CASE9


//keys definition
#define KEYSELECT 	3
#define KEYSET 		2
#define KPIN  		PIND


//segment definitions
#define SEG_NULL	0x00
#define SEG_a 		0x01
#define SEG_b 		0x02
#define SEG_c 		0x04
#define SEG_d 		0x08
#define SEG_e 		0x10
#define SEG_f 		0x20
#define SEG_g 		0x40
#define SEG_dot	0x80

//clock working mode definitions
#define SHOWNODIGIT		0x01
#define SHOWCLOCK			0x02
#define SHOWDATE			0x03
#define SHOWTEMP			0x04
#define SHOWDIFFDAYS		0x05
#define SHOWYEAR			0x06
#define SHOWSENSORS		0x07
#define SHOWNERF			0x08
#define SHOWEGGTIMER		0x09
#define SET					0x0A
#define SETHOURS			0x0B
#define SETMINUTES		0x0C
#define SETDATE			0x0D
#define SETMONTH			0x0E
#define SETYEAR			0x0F
#define SETALHOURS		0x10
#define SETALMINUTES		0x11
#define SETAL				0x12
#define SETSECMODE		0x13
#define SETDIMMODE		0x14

//store parameter values
#define	STORE_USMODE	0x01
#define	STORE_SWING		0x02
#define	STORE_SECMODE	0x04
#define	STORE_DIMMODE	0x08
#define	STORE_ALL		0x0F

//Default values for EEPROM variables if CRC is not correct.
//Alarm default hour values should be set in the interval 0-23 even for US Mode
//#define DefCFUnit		0				// Celsius
#define DefSwing			1				// no swing
#define DefUSMode			0				// no US mode
#define DefSecMode		1
#define DefALMinutes		30
#define DefALHours		6				// alarm 6 o'clock default
#define DefALSet			0				// alarm off

#define EGGSTATETIME	1000

#define DefDiffDay		18				// difference to date (here: 18.04.2018)
#define DefDiffMonth		4
#define DefDiffYear		18


//---------------------------------------------------------------------------------
//variables minimums and maximums definitions
#define MinALMinutes		0
#define MinALHours		0
#define MinALSet			0
#define MaxALMinutes		59
#define MaxALHours		23
#define MaxALSet			1
#define MinYears			10 //(i.e.2010)
#define MinMonth			1
#define MinDate			1
#define MinHours			0
#define MinMinutes		0
#define MaxYears			100
#define MaxMonth			12
#define MaxDate			31
#define MaxHours			23
#define MaxMinutes		59


// store status register
volatile uint8_t tmp_sreg;

#ifdef NERFGUN_MODULE
//nerf target detection stuff
typedef struct
{
	uint16_t nerfDownTime;			//count target hits
	uint8_t  nerfPeakTime;			//counts time between peaks
	uint8_t  nerfPeakCount;			//count input peaks
	uint8_t  nerfTargetCount;		//x*45ms->time target not hit
	uint8_t  nerfState;				//state
} nerf;
nerf myNerf;
#endif

//------------------------------------------------------------------------------
// EEMEM variables making the clock perform otherwise
// CFUnit defines Celsius or Fahrenheit display
// CFUnit=0 means Celsius Display
// CFUnit=1 defines Fahrenheit Display
//uint8_t CFUnit;
// This variable makes the clock show alternatively the hour and the temperature
// Swing=0 means normal Display
// Swing=1 means alternative Clock and Temperature Display
uint8_t Swing;
// This variable makes the clock show the hour in 12H or 24H mode
// USMode=0 means 24H display
// USMode=1 means 12H display
uint8_t USMode;

// EEPROM variables
//EEMEM uint8_t ECFUnit=0;
EEMEM uint8_t ESwing=0;
EEMEM uint8_t EUSMode=0;
EEMEM uint8_t ESecMode=12;
EEMEM uint8_t EDimMode=0;
// ECRC and DefCRC have to be recomputed manually after a modification to any of the following eeprom variables:
// ECFUnit, ESwing, EUSMode, ESecMode
// ECRC=ECFUnit+ESwing+EUSMode+ESecMode
EEMEM uint8_t ECRCParams=12;

// EALMinutes, EALHours, EALSet
// ECRCALARM=EALMinutes+EALHours+EALSet
EEMEM uint8_t EALMinutes=0;
EEMEM uint8_t EALHours=7;
EEMEM uint8_t EALSet=0;
EEMEM uint8_t ECRCAlarm=7;

#ifdef DIFFDATE_MODULE
EEMEM uint8_t EdiffDate=DefDiffDay;
EEMEM uint8_t EdiffMonth=DefDiffMonth;
EEMEM uint8_t EdiffYear=DefDiffYear;
EEMEM uint8_t ECRCDiff = (DefDiffDay + DefDiffMonth + DefDiffYear) ;
#endif



//global variables
uint16_t digit, digit_addressed=0;
uint8_t seconds, secondsOld;
uint8_t d[18];
dateTime dt,dt1;
#ifdef DIFFDATE_MODULE
dateTime diffDt;							// daytime diff days
#endif
#ifdef EGGTIMER_MODULE
dateTime eggDt;								// daytime egg timer
uint8_t  eggState=0;
uint16_t  eggStateTimer=0;
#endif




uint8_t temperature,tempsign=0, mySeed = 170;
uint8_t timestamp1,timestamp2;
uint16_t refresh=0;
uint8_t refresh_resetted =0;
uint8_t pulsing=0, showdigit=0, showled=0;
uint8_t DimMode=0, Dim=0, dimCounter=0;


// display seconds working mode
uint8_t SecMode=1;
uint8_t MinSecMode=0;
uint8_t SecModeOld=1;
#ifndef LEDS_SHOW_HOURMIN
uint8_t MaxSecMode=9;
#else
uint8_t minutesOld=0;
uint8_t MaxSecMode=15;
#endif

//Alarm related variables
uint8_t ALMinutes=0;
uint8_t ALHours=0;
uint8_t ALSet=0;
uint8_t AlarmOn=0;

//clock working mode
uint8_t ClockMode=SHOWCLOCK;
uint8_t ClockModeOld;

//variables used for various delays
uint16_t t1=0, t2=0, t3=0;

//variables used for 18X20 sensors
#ifdef MULTI_TEMPSENSORS
#define MAXSENSORS 5
uint8_t nSensors;
int8_t gSensorIDs[MAXSENSORS][DS18B20_ROMCODE_SIZE];
#endif

//TEMPDISPLAY works like this:
//if it is even, it shows the sensor number /2 +1
//if it is odd, it shows the temperature of the sensor number/2
//pressing the key will increment the variable
//leaving the key not pressed reverts the variable back to zero
uint8_t TEMPDISPLAY=0;

//7-segments character definitions
//    _a_
// f |_g_| b
// e |___| c
//	   d

unsigned char seg[]=
{
	(SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f),			// 0
	(SEG_b|SEG_c),											// 1
	(SEG_a|SEG_b|SEG_d|SEG_e|SEG_g),					// 2
	(SEG_a|SEG_b|SEG_c|SEG_d|SEG_g),					// 3
	(SEG_b|SEG_c|SEG_c|SEG_f|SEG_g),					// 4
	(SEG_a|SEG_c|SEG_d|SEG_f|SEG_g),					// 5
	(SEG_a|SEG_c|SEG_d|SEG_e|SEG_f|SEG_g),			// 6
	(SEG_a|SEG_b|SEG_c),									// 7
	(SEG_a|SEG_b|SEG_c|SEG_d|SEG_e|SEG_f|SEG_g),	// 8
	(SEG_a|SEG_b|SEG_c|SEG_d|SEG_f|SEG_g),			// 9
	(SEG_dot),												// 10 NULL DISPLAY (decimal point lit)
	(SEG_a|SEG_b|SEG_f|SEG_g),							// 11 THE SYMBOL "o" (degrees)
	(SEG_a|SEG_d|SEG_e|SEG_f),							// 12 THE SYMBOL "C"
	(SEG_c|SEG_e|SEG_g),									// 13 THE SYMBOL "n"
	(SEG_a|SEG_e|SEG_f|SEG_g),							// 14 THE SYMBOL "F"
	(SEG_c|SEG_d|SEG_e|SEG_g),							// 15 THE SYMBOL "o" (like small o)
	(SEG_NULL),												// 16 NULL SYMBOL (no decimal point)
	(SEG_a|SEG_b|SEG_c|SEG_e|SEG_f|SEG_g),			// 17 THE SYMBOL "A"
	(SEG_b|SEG_c|SEG_d|SEG_e|SEG_g),					// 18 THE SYMBOL "d"
	(SEG_d|SEG_e|SEG_f|SEG_g),							// 19 THE SYMBOL "t"
	(SEG_d|SEG_e|SEG_f),									// 20 THE SYMBOL "L"
	(SEG_a|SEG_d|SEG_e|SEG_f|SEG_g),					// 21 THE SYMBOL "E"
	(SEG_a|SEG_b|SEG_e|SEG_f|SEG_g),					// 22 THE SYMBOL "P"
	(SEG_g),													// 23 THE SIGN "-"
	(SEG_e),													// 24 THE SYMBOL "i"
	(SEG_b|SEG_c|SEG_d|SEG_f|SEG_g),					// 25 THE SYMBOL "y"
	(SEG_a|SEG_c|SEG_d|SEG_f|SEG_g)					// 26 THE SYMBOL "S"
};


uint8_t randSec()
{
	uint8_t myBool2=0, myBool4=0, myBool8=0, myBool128=0;

	if (mySeed & 2)
	myBool2=1;

	if (mySeed & 4)
	myBool4=1;

	if (mySeed & 8)
	myBool8=1;

	if (mySeed & 128)
	myBool128=1;

	mySeed <<= 1;
	if( ((myBool2 ^ myBool4) ^ myBool8) ^ myBool128 )
	mySeed++;

	if(mySeed<60)
	return mySeed;
	else
	return (mySeed & 59);
}


//tool function to calculate the power of 2
uint8_t p2(uint8_t exponent)
{
	uint8_t a=1,b=1,i;
	for(i=0;i<=exponent-1;i++)
	{
		b*=2;
		a+=b;
	}
	return a;
}


//tool function to convert from decimal to binary display
uint8_t dectobin(uint8_t number)
{
	uint8_t a,i;
	a=1;
	if (number==0) 	a=1;
	else
	for (i=0;i<number;i++) a=a*2;
	return a;
}


//tool function to show if an integer is odd or even
uint8_t odd(uint8_t num)
{
	return num%2;
}


uint8_t bb(uint8_t in11, uint8_t sec1, uint8_t sec2) //function to reduce the code
{
	return dectobin(seconds-in11*8)*((seconds>= sec1) && (seconds<= sec2));
}


void cc(uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3,uint8_t d4,uint8_t d5,uint8_t d6,uint8_t d7, uint8_t in111)
{
	d[10]=d0 | bb(in111,0,7);
	d[11]=d1 | bb(in111,8,15);
	d[12]=d2 | bb(in111,16,23);
	d[13]=d3 | bb(in111,24,31);
	d[14]=d4 | bb(in111,32,39);
	d[15]=d5 | bb(in111,40,47);
	d[16]=d6 | bb(in111,48,55);
	d[17]=d7 | bb(in111,56,59);
}


void displayMinutes()
{
	uint8_t in111 = seconds/8;
	d[10]|= bb(in111,0,7);
	d[11]|= bb(in111,8,15);
	d[12]|= bb(in111,16,23);
	d[13]|= bb(in111,24,31);
	d[14]|= bb(in111,32,39);
	d[15]|= bb(in111,40,47);
	d[16]|= bb(in111,48,55);
	d[17]|= bb(in111,56,59);
}

void displayMinutesInv()
{
	uint8_t in111 = seconds/8;
	d[10]&= ~bb(in111,0,7);
	d[11]&= ~bb(in111,8,15);
	d[12]&= ~bb(in111,16,23);
	d[13]&= ~bb(in111,24,31);
	d[14]&= ~bb(in111,32,39);
	d[15]&= ~bb(in111,40,47);
	d[16]&= ~bb(in111,48,55);
	d[17]&= ~bb(in111,56,59);
}


void fillbefore(uint8_t in11,uint8_t value)
{
	uint8_t i;
	for (i=0;i<in11;i++)
	{
		d[i+10]=value;
	}
}


void fillafter(uint8_t in11,uint8_t value)
{
	uint8_t i;
	for (i=in11+11;i<18;i++)
	{
		d[i]=value;
	}
}


#ifdef DIFFDATE_MODULE
//tool function that returns if a year is a leap year
uint8_t leap(uint8_t y)
{
	return ( (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0) );
}


//tool function that counts the number of the day in the given year
uint16_t dayOfYear(dateTime dt)
{
	uint8_t isLeap=leap(dt.year);

	uint8_t month[12] = {31,28+isLeap,31,30,31,30,31,31,30,31,30,31};
	uint8_t i;
	uint16_t dayOfYearResult=0;
	for (i=0; i<(dt.month-1); i++)
	{
		dayOfYearResult += month[i];
	}
	return dayOfYearResult + dt.date;
}


//tool function that calculates which date is lower
uint8_t isLowerDateLeft(dateTime Date1, dateTime Date2)
{
	if(Date1.year == Date2.year)
	{
		if(Date1.month == Date2.month)
		{
			if(Date1.date == Date2.date)
			return 1;
			else
			{	if(Date1.date < Date2.date)
				return 1;
				else
				return 0;
			}
		}
		else
		if(Date1.month < Date2.month)
		return 1;
		else
		return 0;
	}
	else
	if(Date1.year < Date2.year)
	return 1;
	else
	return 0;
}


//tool function that calculates the days between two dates
uint16_t daysBetweenDates(dateTime Date1, dateTime Date2)
{
	uint16_t sumOfdays=0;
	dateTime lowerDate;
	dateTime futureDate;
	if( isLowerDateLeft(Date1, Date2) )
	{
		lowerDate = Date1;
		futureDate = Date2;
	}
	else
	{
		lowerDate = Date2;
		futureDate = Date1;
	}


	//cumulate days in years
	uint8_t currentYear = lowerDate.year;
	while(futureDate.year >  currentYear)
	{
		sumOfdays+=365;
		if(leap(currentYear))
		sumOfdays++;

		currentYear++;
	}
	uint16_t lowerDateDayOfYear = dayOfYear(lowerDate);
	uint16_t futureDateDayOfYear = dayOfYear(futureDate);
	sumOfdays += (futureDateDayOfYear - lowerDateDayOfYear);
	return sumOfdays;
}
#endif


void SetD(uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3)
{
	d[0]=d0;
	d[1]=d1;
	d[2]=d2;
	d[3]=d3;
}


void SetFill(uint8_t in11,uint8_t fb2, uint8_t dd2, uint8_t fa2)
{
	fillbefore(in11,fb2);
	d[in11+10]=dd2;
	fillafter(in11,fa2);
}

void SetTwoDigit(uint8_t digit, uint8_t ten, uint8_t one)
{
	d[ten] = digit/10;
	d[one] = digit-d[ten]*10;
}


void DisplayHours(uint8_t i, uint8_t j)
{
	if (USMode)
	{
		if (i>12) i-=12;
		if (i==0) i=12;
	}
	SetTwoDigit(i, 4,5);
	if (USMode)
	{
		if (j>12)
		SetD(seg[d[4]],seg[d[5]],seg[16],seg[22]);
		else
		SetD(seg[d[4]],seg[d[5]],seg[16],seg[17]);
	}
	else
	SetD(seg[d[4]],seg[d[5]]+SEG_dot,seg[16],seg[16]);
}


void growingCycle (uint8_t seconds)
{
	uint8_t j=refresh/19;						// /13 for 8MHz, /19 for 12MHz
	if (j<=seconds)
	{
		uint8_t in1 = j/8;
		j -= in1*8;
		if (odd(dt.minute))
		{
			SetFill(in1,0xFF,p2(j),0x00);
		}
		else
		{
			SetFill(in1,0x00,~p2(j),0xFF);
		}
	}
}

void redLedsOn (uint8_t in1)
{
	cc(0b00100001,0b10000100,0b00010000,0b01000010,0b00001000,0b00100001,0b10000100,0b00010000,in1);	// 1, 2, 5
}


void redLedsAllOn ()
{
	d[10]=0b00100001;
	d[11]=0b10000100;
	d[12]=0b00010000;
	d[13]=0b01000010;
	d[14]=0b00001000;
	d[15]=0b00100001;
	d[16]=0b10000100;
	d[17]=0b00010000;
}


void setRedLed(uint8_t hour, uint8_t set)
{
	if(hour >= 12)
	hour-=12;

	uint8_t arrayPos = 10+((hour*5)/8);
	uint8_t arrayValue =  (1<<(((hour*5)%8)));

	if(!set)
	{
		//d[10]=d[11]=d[12]=d[13]=d[14]=d[15]=d[16]=d[17]= 0xFF;		// set all on
		d[arrayPos] &= ~arrayValue;										// set red led (hour) on
	}
	else
	{
		//d[10]=d[11]=d[12]=d[13]=d[14]=d[15]=d[16]=d[17]=0;			// set all off
		d[arrayPos] |= arrayValue;										// set red led (hour) off
	}

}

//led function
void computingLeds(void)
{
	//computing seconds -> multiple choices available pertaining to the design we wish
	if( (SecMode==14) ||  ((SecMode==15) && odd(seconds)) )
	{
		seconds=randSec();
	}
	if( (SecMode==0) ||  (SecMode==10) ||  (SecMode==99) )
	{
		//d[10]=d[11]=d[12]=d[13]=d[14]=d[15]=d[16]=d[17]=0b00000000;	// set all ni
		d[10]=d[11]=d[12]=d[13]=d[14]=d[15]=d[16]=d[17]=0;			// set all off
	}

	uint8_t i;
	uint8_t in1=seconds/8;
	uint8_t seconds2=seconds-in1*8;

	switch(SecMode)
	{
		case 0: // don't show any led
		//[10]=d[11]=d[12]=d[13]=d[14]=d[15]=d[16]=d[17]=0b00000000;	// set all ni
		break;

		case 1: //seconds go normally
		SetFill(in1,0x00,dectobin(seconds2),0x00);
		break;

		case 2: //seconds are all lit except the current one
		SetFill(in1,0xFF,~dectobin(seconds2),0xFF);
		break;

		case 3: //seconds fill progressively
		for (i=0;i<in1;i++)
		SetFill(in1,0xFF,p2(seconds2),0x00);
		break;

		case 4:	//seconds empty progressively
		SetFill(in1,0x00,~p2(seconds2),0xFF);
		break;

		case 5://seconds fill progressively at odd minutes and empty progressively at even minutes
		if (odd(dt.minute))
		{
			SetFill(in1,0xFF,p2(seconds2),0x00);
		}
		else
		{
			SetFill(in1,0x00,~p2(seconds2),0xFF);
		}
		break;

		case 6: //seconds go normally and red leds lit
		redLedsOn(in1);
		break;

		case 7:	//seconds go normally and red leds lit. Red leds shift when the current second falls on them
		if (seconds%5 != 0)
		redLedsOn(in1);
		else if (odd(seconds/5))
		cc(0b01000010,0b00001000,0b00100001,0b10000100,0b00010000,0b01000010,0b00001000,0b00100001,in1);
		else
		cc(0b00010000,0b01000010,0b00001000,0b00100001,0b10000100,0b00010000,0b01000010,0b00001000,in1);
		break;

		case 8:	//seconds go normally and red leds lit. Red leds go off when the current second falls on them
		if (seconds%5 != 0)
		redLedsOn(in1);
		else
		cc(0,0,0,0,0,0,0,0,in1);
		break;

		case 9:
		#ifdef LEDS_CASE9
		growingCycle (seconds);
		#endif
		break;

		#ifdef LEDS_SHOW_HOURMIN
		// show hour and minute with leds:
		case 10:
		seconds = dt.minute;
		if( (minutesOld!=seconds) || (SETSECMODE==ClockMode) )		// do only if minutes changed
		{
			minutesOld=seconds;
			cli();
			//d[10]=d[11]=d[12]=d[13]=d[14]=d[15]=d[16]=d[17]=0;			// set all off
			setRedLed(dt.hour, 1);										// red led (hour on)
			sei();
			if( (seconds%5) == 0.0 )									// don't display minutes on red leds (red leds only for hours)
			{
				seconds--;												// displayMinutes twice then
				displayMinutes();
				seconds +=2;
			}
			displayMinutes();
		}
		break;

		case 11:															// show hour and minute with flashing leds
		d[10]=d[11]=d[12]=d[13]=d[14]=d[15]=d[16]=d[17]= 0xFF;			// set all on
		if ( odd(seconds) )												// 'blinking': alternation between hour and min
		{
			seconds =  dt.minute;
			if( (seconds%5) == 0 )										// don't display minutes on red leds (red leds only for hours)
			{
				seconds--;												// displayMinutes twice, before and after hour led
				displayMinutesInv();
				seconds +=2;
			}
			displayMinutesInv();
		}
		else
		{
			cli();
			setRedLed(dt.hour, 0);										// red led (hour) off
			sei();
		}
		break;
		
		
		case 12:															// show hour (red led) and three minute (green leds) flashing
		d[10]=d[11]=d[12]=d[13]=d[14]=d[15]=d[16]=d[17]= 0xFF;			// set all on
		if ( odd(seconds) )												// 'blinking'
		{
			seconds =  dt.minute;										// set minute(s)
			cli();
			setRedLed(dt.hour, 0);										// red led (hour) off
			sei();

			if(seconds)
			seconds--;
			
			int secExtention = seconds;
			for(int i=seconds+3;secExtention < i ; secExtention++)
			{
				seconds = secExtention;
				if(seconds>59)
				seconds -= 59;

				if(seconds%5) 											// don't display minutes on red leds (red leds only for hours)
				{
					cli();
					displayMinutesInv();
					sei();
				}
			}
		}
		break;

		case 13:
		if ( odd(seconds) )												// 'blinking'
		{
			seconds =  dt.minute;
			cli();
			redLedsOn(seconds/8);
			setRedLed(dt.hour, 0);										// red led (hour off)
			sei();
		}
		else
		redLedsAllOn();

		break;

		case 14: // led's randomly
		case 15:
		SetFill(in1,0x00,dectobin(seconds2),0x00);
		break;

		#endif
		#ifdef NERFGUN_MODULE
		case 99:															// a nerf hit has count
		//d[10]=d[11]=d[12]=d[13]=d[14]=d[15]=d[16]=d[17]= 0;				// set all off
		if ( odd(seconds) )
		redLedsAllOn();
		break;
		#endif
		
		default:
		break;
	}
}


void computingSomeDigits(uint8_t digit)
{
	uint8_t	d1= digit/10;
	uint8_t	d2= digit-d1*10;

	if( (ClockMode==SETALHOURS) ||
	((USMode) && (ClockMode==SETMONTH)) ||
	((!USMode) && (ClockMode==SETDATE)) )
	{
		d[4] = d1;
		d[5] = d2;
	}
	else
	{
		d[6] = d1;
		d[7] = d2;
	}
	
	if(  (ClockMode==SETMINUTES) || (ClockMode==SETALMINUTES) )
	{
		SetD(seg[16],seg[16]+SEG_dot,seg[d[6]],seg[d[7]]);
	}
	else
	if( ((!USMode) && (ClockMode==SETMONTH)) ||
	((USMode) && (ClockMode==SETDATE)) )
	{
		SetD(seg[16],seg[16],seg[d[6]],seg[d[7]]);
	}
	else
	if ( ((USMode) && (ClockMode==SETMONTH)) ||
	((!USMode) && (ClockMode==SETDATE)) )
	{
		SetD(seg[d[4]],seg[d[5]],seg[16],seg[16]);
	}
	else
	{
		SetD(seg[d[4]],seg[d[5]]+SEG_dot,seg[16],seg[16]);
	}

}

//main display function
void display(void)
{
	uint8_t hr, min, month, date, temp;
	uint16_t year;
	uint8_t temphr,tempmin;

	if (ClockMode==SHOWSENSORS)
	{
		SetD(seg[19],seg[21]+SEG_dot,SEG_NULL,seg[digit]);
	}
	else
	if (ClockMode==SHOWCLOCK)
	{
		//digit=refresh/13;
		if (t2==0)//show clock
		{
			if (USMode)
			{	//1-12 only in us mode
				tempmin=digit%100;
				temphr=digit/100;
				if (temphr>12) temphr-=12;
				if (temphr==0) temphr=12;
				digit=tempmin+100*temphr;
			}

			//computing hours and minutes
			hr=digit/100;
			min=digit-hr*100;
			SetTwoDigit(hr, 4,5);
			SetTwoDigit(min, 6,7);
			if (hr>9) d[0]= seg[d[4]]; else d[0]= SEG_NULL;
			d[1]= seg[d[5]];
			//add dot point on odd seconds
			if (refresh<512) d[1]+=SEG_dot;
			d[2]= seg[d[6]];
			d[3]= seg[d[7]];
		}
		else
		if (t2<5) //show "date" while setting the clock
		{
			SetD(seg[18],seg[17],seg[19],seg[21]);
		}
		else
		if (t2<10)  //show "S :AL" (Set Alarm) while setting the clock
		{
			SetD(seg[5],seg[10],seg[17],seg[20]);
		}
		else
		if (t2<=15) //show  "LED"              "diff"  dim
		{
			//SetD(seg[18],seg[24],seg[14],seg[14]);
			SetD(seg[20],seg[21],seg[18],seg[16]);
		}
		else
		if (t2<=20) //show  "dim"
		{
			SetD(seg[18],seg[24],seg[13],seg[13]);
		}
		else		//show "S :CL" (Set clock) while setting the clock
		{
			SetD(seg[5],seg[10],seg[12],seg[20]);
		}
	}
	else
	if (ClockMode==SHOWDATE)
	{
		//computing month and day
		if(USMode)
		{
			month=digit/100;
			date=digit-month*100;
		}
		else
		{
			//switched MM and dd
			date=digit/100;
			month=digit-date*100;
		}
		SetTwoDigit(month, 4,5);
		SetTwoDigit(date, 6,7);
		if (d[4]!=0) 	d[0]= seg[d[4]];
		else			d[0]= seg[16];
		d[1]= seg[d[5]];
		if (d[6]!=0) 	d[2]= seg[d[6]];
		else			d[2]= seg[16];
		//d[2]= seg[d[6]];
		d[3]= seg[d[7]];
	}
	else
	if( (ClockMode==SHOWDIFFDAYS) && (odd(seconds)) )
	{
		SetD(seg[18],seg[17],seg[25],seg[26]);
	}
	else
	if( (ClockMode==SHOWYEAR) || (ClockMode==SHOWDIFFDAYS) || (ClockMode==SHOWNERF) || (ClockMode==SHOWEGGTIMER) )
	{
		//computing digits of year or diff days
		year=digit/100;
		SetTwoDigit(year, 4,5);
		SetTwoDigit(digit-year*100, 6,7);
		SetD(seg[d[4]],seg[d[5]],seg[d[6]],seg[d[7]]);
	}
	else
	if (ClockMode==SHOWTEMP)
	{
		#ifdef MULTI_TEMPSENSORS
		if (nSensors==0)
		{
			SetD(SEG_NULL,seg[13],seg[15],SEG_NULL);
		}
		else
		if (!odd(TEMPDISPLAY))
		{
			//showing the sensor number /2
			SetD(seg[19],seg[21]+SEG_dot,SEG_NULL,seg[TEMPDISPLAY/2+1]);
		}
		else
		#endif
		{
			//displaying the temperature of the sensor number/2 -1
			//here we should display negative temperatures.
			//Negative temperatures are in the range 100-127 where
			//127=-1C
			temp=digit;
			d[4]= temp/100;
			temp-=d[4]*100;

			SetTwoDigit(temp, 5,6);
			if (d[4]==0) d[0]=SEG_NULL;
			else d[0]= seg[d[4]];
			if ((d[5]==0) && (d[4]==0)) d[1]=SEG_NULL;
			else d[1]= seg[d[5]];
			d[2]= seg[d[6]];
			d[3]= seg[12+USMode*2];

			if (tempsign==1)  //negativetemperatures
			d[0]=seg[23]; //the first digit is a minus sign
		}
	}
	else
	if (ClockMode==SETHOURS)
	{
		temphr=dt1.hour;
		DisplayHours(temphr,dt1.hour);
	}
	else
	if (ClockMode==SETMINUTES)
	{
		computingSomeDigits(dt1.minute);
	}
	else
	if (ClockMode==SETDATE)
	{
		computingSomeDigits(dt1.date);
	}
	else
	if (ClockMode==SETMONTH)
	{
		computingSomeDigits(dt1.month);
	}
	else
	if (ClockMode==SETYEAR)
	{
		d[4]= 2;
		d[5]= dt1.year/100;
		SetTwoDigit((dt1.year-d[5]*100), 6,7);
		//d[6]= (dt1.year-d[5]*100)/10;
		//d[7]= dt1.year-d[5]*100-d[6]*10;
		SetD(seg[d[4]],seg[d[5]],seg[d[6]],seg[d[7]]);
	}
	else
	if (ClockMode==SETALHOURS)
	{
		computingSomeDigits(ALHours);
		temphr=ALHours;
		DisplayHours(ALHours,ALHours);

	}
	else
	if (ClockMode==SETALMINUTES)
	{
		computingSomeDigits(ALMinutes);
	}
	else
	if (ClockMode==SETAL)
	{
		if (ALSet==1) //on
		{
			SetD(seg[15],seg[13],seg[16],seg[16]);
		}
		else	//off
		{
			SetD(seg[15],seg[14],seg[14],seg[16]);
		}
	}
	else
	if (ClockMode==SETSECMODE)
	{
		SetTwoDigit(SecMode, 6,7);
		SetD(seg[16],seg[16],seg[d[6]],seg[d[7]]);
	}
	else
	if (ClockMode==SETDIMMODE)
	{
		d[7]=DimMode;
		SetD(seg[16],seg[16],seg[16],seg[d[7]]);
	}
	computingLeds();
}



//function that beeps
void beep(void)
{
	PORTB &= ~_BV(4);
	_delay_ms(150);
	PORTB |= _BV(4);
	_delay_ms(150);
}


////function that beeps for x ms
//void beepMS(uint16_t ms)
//{
//uint16_t i = ms/10;
//while(i--)
//beep();
//}


#ifdef MULTI_TEMPSENSORS
uint8_t Search_sensors(void)
{
	uint8_t i;
	uint8_t id[DS18B20_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	ds18b20_reset();

	nSensors = 0;
	
	diff = DS18B20_SEARCH_FIRST;
	while ( diff != DS18B20_LAST_DEVICE && nSensors < MAXSENSORS ) {
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == DS18B20_PRESENCE_ERR ) {
			//uart_puts_P( "No Sensor found" NEWLINESTR );
			break;
		}
		
		if( diff == DS18B20_DATA_ERR ) {
			//uart_puts_P( "Bus Error" NEWLINESTR );
			break;
		}
		
		for ( i=0; i < DS18B20_ROMCODE_SIZE; i++ )
		gSensorIDs[nSensors][i] = id[i];
		
		nSensors++;
	}
	
	return nSensors;
}
#endif




//function which reads if there is any key pressed
uint8_t readkeys(void)
{
	uint16_t keypressed=0;

	////SELECT and SET keys simultaneously. Not used.
	//if (bit_is_clear(KPIN, KEYSELECT) && bit_is_clear(KPIN, KEYSET))
	//{
	//keypressed=3;
	//}
	////SELECT key
	//else
	if bit_is_clear(KPIN, KEYSELECT)
	{
		keypressed=1;
	}
	//SET key
	else
	if bit_is_clear(KPIN, KEYSET)
	{
		keypressed=2;
	}
	
	return keypressed;

}


uint8_t ConvertCToF(uint8_t cdigit)
{
	if (cdigit>100)
	{
		if (USMode == 1)
		{
			tempsign=(cdigit<110);						// negative F temperatures come below -17C
			return 32-(128-cdigit)*9/5;
		}
		else
		{
			tempsign=1;									// negative temperatures
			return 128-cdigit;
		}
	}
	else
	{
		tempsign=0;										// positive C and F temperatures
		if (USMode == 1) 	return cdigit*9/5+32;
		else 				return cdigit;
	}
}


void SetParams(uint8_t tpulsing)
{
	pulsing=tpulsing;
	dt=get_date_time();
	seconds=dt.second;
	_delay_ms(10);
}


void SetPortA (uint8_t i)
{
	PORTC=0xFF;							// deselect all led rows
	PORTD = (0xFC & (~(1<<(i+4))));		// set digits -> set PORTD but not TX, RX, PLUS, MODE

	if (showdigit)
	PORTA = ~(d[i]);
	else
	PORTA= ~(SEG_NULL);
};


void SetPortACD(uint8_t i, uint8_t j)
{
	PORTD= 0xFC;			// set nil to digits -> set PORTD all high but not TX, RX, PLUS, MODE
	if(showled)
	{
		PORTC=j;				// Select Bit0 - first 8 leds
		//_delay_us(10);
		PORTA = ~(d[i]);	// set led data to display
	}
	else
	{
		PORTA=0xFF;			// Led data is nil
		PORTC=0xFF;			// deselect all led rows
	}
};



//interrupt routine for character and seconds display into an unending loop.
//it uses the second 16-bit timer, TIMER1 ~> every 666µs
ISR(TIMER1_OVF_vect)
{
	tmp_sreg = SREG;																		// store status register

	refresh++;
	#ifdef LEDS_CASE9
	//comment up to the next sign in order to use first version of function for seconds #6
	//	|
	//	v
	timestamp1=dt.second;
	if (timestamp1==timestamp2)
	{
		if (!refresh_resetted)
		{
			refresh=0;
			refresh_resetted=1;
		}
	}
	else
	{
		refresh_resetted=0;
	}
	//	^
	//	|
	//comment from the first sign in order to use first version of function for seconds #6
	#endif
	
	if(digit_addressed < 4)
	{
		if (pulsing)									// pulsing for SET modes
		showdigit=(refresh<512);
		else
		{
			if(ClockMode == SHOWNODIGIT)
			showdigit=0;							// do not show digits
		}
		SetPortA(digit_addressed);						// set portA for case 0-3
	}
	else
	{
		switch (digit_addressed)
		{
			case 4:
			SetPortACD(10,0xFE);
			break;
			case 5:
			SetPortACD(11,0xFD);
			break;
			case 6:
			SetPortACD(12,0xFB);
			break;
			case 7:
			SetPortACD(13,0xF7);
			break;
			case 8:
			SetPortACD(14,0xEF);
			break;
			case 9:
			SetPortACD(15,0xDF);
			break;
			case 10:
			SetPortACD(16,0xBF);
			break;
			case 11:
			SetPortACD(17,0x7F);
			break;
		}
	}
	//comment the following line in order to use first version of function for seconds #9
	//	|
	//	v
	timestamp2=dt.second;
	//	^
	//	|

	digit_addressed++;
	if(digit_addressed>=12)
	{
		digit_addressed=0;

		// dim LCD display and led's
		dimCounter++;
		if(dimCounter > Dim)
		{
			dimCounter=0;
			showdigit=1;
			showled = 1;
		}
		else
		{
			showdigit=0;
			showled = 0;
		}
	}

	#ifdef NERFGUN_MODULE
	if(myNerf.nerfPeakCount)
	{
		if(myNerf.nerfPeakTime>7)								// detect peaks in an interval of 7*IRS-TIMER1(350µs),
		myNerf.nerfPeakCount=0;								// after that reset nerf peak counter
		else
		myNerf.nerfPeakTime++;
	}
	
	if(myNerf.nerfTargetCount)									// count downtime (time target not hit)
	myNerf.nerfDownTime++;
	#endif

	#ifdef EGGTIMER_MODULE
	if(eggStateTimer>=5)
	eggStateTimer--;
	#endif

	SREG = tmp_sreg;											// restore status register
}

////interrupt routine if MODE button pressed
//ISR(INT0_vect)
//{
////MODE pressed
////key=2;
//_delay_ms(10);
//}
//
////interrupt routine if PLUS button pressed
//ISR(INT1_vect)
//{
////PLUS pressed
////key=1;
//_delay_ms(10);
//}


#ifdef EGGTIMER_MODULE
void addEggTimerMin(void)
{
	eggDt.minute++;
	if(eggDt.minute>59)
	{
		eggDt.minute=0;
		eggDt.hour++;
		if(eggDt.hour>23)
		eggDt.hour=0;
	}
}

int16_t diffTimeInSec (void)
{
	int8_t  diffMin = (eggDt.hour - dt.hour) * 60;
	diffMin +=(int8_t)(eggDt.minute - dt.minute);
	int16_t diffSec =  ((int8_t)(eggDt.second - dt.second)) + (diffMin*60);
	if(diffSec<0)
	return 0;
	else
	return	diffSec;
}
#endif

//function to check if the alarm should start
void CheckAlarm(void)
{
	#ifdef	EGGTIMER_MODULE
	if( (eggState) && (!diffTimeInSec()) )
	{
		eggState=0;
		AlarmOn=1;
		t3=0;
	}
	
	#endif

	if (ALSet)
	{
		if ((ALHours==dt.hour) && (ALMinutes==dt.minute) && (dt.second==0))
		{
			AlarmOn=1;
			t3=0;
		}
	}

	if (AlarmOn)
	{
		beep();
		t3++;
		if (t3==2900) AlarmOn=0; //2900=10 minutes
	}
}

// store clock parameter to eeprom
void storeParameter(uint8_t parameterIndex)
{
	if(parameterIndex & STORE_USMODE)
	{
		eeprom_write_byte(&EUSMode,USMode);
	}
	if(parameterIndex & STORE_SWING)
	{
		eeprom_write_byte(&ESwing,Swing);
	}
	if(parameterIndex & STORE_SECMODE)
	{
		eeprom_write_byte(&ESecMode,SecMode);
	}
	if(parameterIndex & STORE_DIMMODE)
	{
		eeprom_write_byte(&EDimMode,DimMode);
	}
	eeprom_write_byte(&ECRCParams,USMode+Swing+SecMode+DimMode);
}


//interrupt routine if NERF pulse was detected
ISR(INT2_vect)
{
	tmp_sreg = SREG;						// store status register

	#ifdef NERFGUN_MODULE
	myNerf.nerfPeakCount++;
	if(myNerf.nerfPeakCount > 1)			// 2 peaks min before a hit counts
	{
		myNerf.nerfPeakCount=0;
		myNerf.nerfTargetCount++;
		myNerf.nerfState=1;
		if(ClockMode!=SHOWNERF)
		{
			ClockModeOld=ClockMode;
			ClockMode=SHOWNERF;
		}
		if (AlarmOn)
		{
			AlarmOn=0;
		}
		myNerf.nerfDownTime=0;
	}
	myNerf.nerfPeakTime=0;
	#endif


	#ifdef	EGGTIMER_MODULE
	if(eggState==0)							// if not in eggtimer mode
	{
		if (AlarmOn)						// if alarm on
		{
			AlarmOn=0;							// .. set alarm off
			ClockMode=ClockModeOld;
			eggStateTimer=EGGSTATETIME;			// debounce: EGGSTATETIME*333µ-> wait  for next possible hit
		}
		else if(eggStateTimer<5)
		{
			// start eggtimer:
			eggState=1;
			ClockModeOld=ClockMode;
			ClockMode=SHOWEGGTIMER;			// set to eggtimer mode
			eggDt =  get_date_time();		// set current date
			addEggTimerMin();				// add 1min to current eggtime
			eggStateTimer=EGGSTATETIME;		// debounce: EGGSTATETIME*333µ-> wait  for next possible hit
		}
	}
	else if(eggStateTimer<5)
	{
		eggStateTimer=EGGSTATETIME;			// debounce: EGGSTATETIME*333µ-> wait  for next possible hit
		if(ClockMode==SHOWEGGTIMER)
		addEggTimerMin();				// set eggtime + 1min
		else
		ClockMode=SHOWEGGTIMER;			// or set to SHOWEGGTIMER, without '+1min'
	}
	#endif

	SREG = tmp_sreg;						// restore status register
}

void noKeyPressed(void)
{
	if(ClockMode>SET)
	SetParams(1);
	else
	SetParams(0);
	

	//void all long key presses
	switch (ClockMode)
	{
		#ifdef NERFGUN_MODULE										// show number of target hits if just hit
		case SHOWNERF:
		if(myNerf.nerfDownTime > 65500)					//~ 45sec no target hit, set back to SHOWCLOCK
		{
			myNerf.nerfDownTime=0;
			myNerf.nerfTargetCount=0;
			ClockMode=ClockModeOld;
		}
		else
		{
			digit = myNerf.nerfTargetCount;				// show number of target hits
			switch(myNerf.nerfState)					// clock was currently hit
			{
				case 1:
				SecMode = 99;						// show red leds after hit
				beep();
				myNerf.nerfState=0;
				myNerf.nerfDownTime=0;
				myNerf.nerfState=2;
				beep();
				break;

				case 2:
				if(myNerf.nerfDownTime>10000)		//set led mode back after ~ 3sec
				{
					SecMode = SecModeOld;
					myNerf.nerfState=0;
				}
				break;

				default:
				break;
			}
		}
		//SetParams(0);
		break;
		#endif

		#ifdef DIFFDATE_MODULE
		case SHOWDIFFDAYS:
		//SetParams(0);
		digit = daysBetweenDates(dt, diffDt);
		break;
		#endif

		#ifdef EGGTIMER_MODULE
		case SHOWEGGTIMER:
		//SetParams(0);
		digit = diffTimeInSec();
		break;
		#endif

		case SHOWNODIGIT:
		//SetParams(0);
		break;

		case SHOWCLOCK:
		if (t2==0)
		{
			//show clock
			//SetParams(0);
			digit=dt.hour*100+dt.minute;
		}
		else
		if (t2<=5)
		{
			//show the date
			//SetParams(0);
			t1=120;
			t2=0;
			ClockMode=SHOWDATE;
		}
		else
		if (t2<=10)
		{
			//set the alarm
			SetParams(1);
			t1=900;
			t2=0;
			ClockMode=SETALMINUTES;
		}
		else
		if (t2<=15)
		{
			//set sec mode
			////set DIFFDAYS
			//SetParams(1);
			t1=900;
			t2=0;
			////digit = daysBetweenDates(dt1,dt);
			ClockMode=SETSECMODE;
		}
		else
		if (t2<=20)
		{
			t1=900;
			t2=0;
			ClockMode=SETDIMMODE;
		}
		else
		{
			//set the date
			pulsing=1;
			t1=900;
			t2=0;
			dt1=dt;
			ClockMode=SETYEAR;
		}
		break;

		case SHOWTEMP:
		#ifdef MULTI_TEMPSENSORS
		switch (nSensors)
		{
			case 0:
			digit = 255;
			case 1:
			digit = ConvertCToF(ds18b20_gettemp());
			default:
			digit = ConvertCToF(ds18b20_getindextemp(&gSensorIDs[TEMPDISPLAY/2][0]));
		}
		#else
		digit = 0;
		digit = ConvertCToF(ds18b20_gettemp()) - TEMPCORRECTION;
		#endif
		//SetParams(0);
		break;

		case SHOWDATE:
		//show the date
		//SetParams(0);
		digit=dt.month*100+dt.date;
		break;

		case SHOWYEAR:
		//show the year
		//SetParams(0);
		digit=2000+dt.year;
		break;

		//case SETHOURS:
		//SetParams(1);
		//break;
		//case SETMINUTES:
		//SetParams(1);
		//break;
		//case SETDATE:
		//SetParams(1);
		//break;
		//case SETMONTH:
		//SetParams(1);
		//break;
		//case SETYEAR:
		//SetParams(1);
		//break;
		//case SETALHOURS:
		//SetParams(1);
		//break;
		//case SETALMINUTES:
		//SetParams(1);
		//break;
		//case SETAL:
		//SetParams(1);
		//break;
		//case SETSECMODE:
		////read seconds to be displayed
		//SetParams(1);
		//break;

		default:
		break;
	}
}

void plusKeyPressed(void)
{
	if (AlarmOn)
	{
		AlarmOn=0;
	}
	if(ClockMode>SET)
	{
		pulsing=0;
		t1=900;
	}
	switch (ClockMode)
	{
		case SHOWNERF:
		#ifdef NERFGUN_MODULE
		myNerf.nerfTargetCount=0;
		myNerf.nerfState=0;
		#endif
		ClockMode=SHOWCLOCK;
		break;

		case SHOWCLOCK:
		ClockMode=SHOWDIFFDAYS;
		break;

		case SHOWDIFFDAYS:
		ClockMode=SHOWTEMP;
		break;

		case SHOWTEMP:
		#ifdef MULTI_TEMPSENSORS
		t1=90;
		if (++TEMPDISPLAY/2==nSensors)
		{
			TEMPDISPLAY=0;
			ClockMode=SHOWNODIGIT;
		}
		#else
		ClockMode=SHOWNODIGIT;
		#endif
		break;

		case SHOWNODIGIT:
		ClockMode=SHOWCLOCK;
		break;

		case SHOWDATE:
		break;

		case SHOWYEAR:
		break;

		case SETHOURS:
		if (++dt1.hour>MaxHours) dt1.hour=MinHours;
		break;
		case SETMINUTES:
		//pulsing=0;
		//t1=900;
		if (++dt1.minute>MaxMinutes) dt1.minute=MinMinutes;
		break;
		case SETDATE:
		//pulsing=0;
		//t1=900;
		if (++dt1.date>MaxDate) dt1.date=MinDate;
		break;
		case SETMONTH:
		//pulsing=0;
		//t1=900;
		if (++dt1.month>MaxMonth) dt1.month=MinMonth;
		break;
		case SETYEAR:
		//pulsing=0;
		//t1=900;
		if (++dt1.year>MaxYears) dt1.year=MinYears;
		break;
		case SETALHOURS:
		//pulsing=0;
		//t1=900;
		if (++ALHours>MaxALHours) ALHours=MinALHours;
		break;
		case SETALMINUTES:
		//pulsing=0;
		//t1=900;
		if (++ALMinutes>MaxALMinutes) ALMinutes=MinALMinutes;
		break;
		case SETAL:
		//pulsing=0;
		//t1=900;
		if (++ALSet>MaxALSet) ALSet=MinALSet;
		break;
		case SETSECMODE:
		SetParams(0);
		//			t1=900;
		if (++SecMode>MaxSecMode) SecMode=MinSecMode;
		break;
		case SETDIMMODE:
		//			t1=900;
		if (++DimMode>9) DimMode=0;
		Dim = DimMode;
		break;
		default:
		break;
	}
}

void modeKeyPressed(void)
{
	if (AlarmOn)
	{
		AlarmOn=0;
	}
	switch (ClockMode)
	{
		case SHOWEGGTIMER:
		case SHOWDIFFDAYS:
		case SHOWTEMP:
		ClockMode=SHOWCLOCK;
		case SHOWCLOCK:
		//short key press 1-5 seconds: show date/time for 10 seconds
		//long key press 6-10 seconds: enter alarm set
		//more than 10 seconds pressed: enter date set
		t2++;
		digit=t2;
		break;

		case SHOWDATE:
		t1=120;
		ClockMode=SHOWYEAR;
		break;

		case SHOWYEAR:
		ClockMode=SHOWCLOCK;
		break;

		case SETHOURS:
		//write the time/date/year into the DS1302
		pulsing=0;
		set_date_time(dt1);
		t1=0;
		beep();
		break;
		case SETMINUTES:
		ClockMode=SETHOURS;
		break;
		case SETDATE:
		ClockMode=SETMINUTES;
		break;
		case SETMONTH:
		ClockMode=SETDATE;
		break;
		case SETYEAR:
		ClockMode=SETMONTH;
		break;
		case SETALHOURS:
		ClockMode=SETAL;
		break;
		case SETALMINUTES:
		ClockMode=SETALHOURS;
		break;
		case SETAL:
		//write Alarm Values into EEPROM
		pulsing=0;
		eeprom_write_byte(&EALHours,ALHours);
		eeprom_write_byte(&EALMinutes,ALMinutes);
		eeprom_write_byte(&EALSet,ALSet);
		eeprom_write_byte(&ECRCAlarm,ALHours+ALMinutes+ALSet);
		t1=0;
		beep();
		break;
		case SETSECMODE:
		//write SecMode into EEPROM
		pulsing=0;
		storeParameter(STORE_SECMODE);
		SecModeOld = SecMode;
		t1=0;
		beep();
		break;
		case SETDIMMODE:
		pulsing=0;
		t1=0;
		beep();
		storeParameter(STORE_DIMMODE);
		//ClockMode=SHOWCLOCK;
		break;
		default:
		break;
	}
}

void init(void)
{
	//PORTS
	PORTA = 0xFF;												// set all pullup's port A  (X0-7)
	DDRA  = 0xFF;												// set all port A as output (X0-7)
	DDRB |= _BV(0) | _BV(4);									// set port B0+4 as output  (Bit0-7)
	PORTB|= _BV(0) |_BV(1) | _BV(2) | _BV(3)| _BV(4);			// set pullup's 0-4 port A  (X0-7)
	PORTC = 0xFF;												// set all pullup's port C  (Bit0-7)
	DDRC  = 0xFF;												// set all port C as output (Bit0-7)
	PORTD|= _BV(2)|_BV(3)|_BV(4)|_BV(5)|_BV(6)|_BV(7);			// set all pullup's port D  (Bit8-11 and buttons->activ low)
	DDRD |= _BV(4)|_BV(5)|_BV(6)|_BV(7);						// set port D as output (Bit8-11)
	//PORTE|= (1<<PE0);											// pullup Port E INT2
	DDRE = 0;													// port E 0-2 as input

	//Timer
	TCNT0 = 0xFF ;												// Timer0 (is 8bit)
	TCCR0 = 0x00;												// Timer0 stopped
	TCNT1 = 0xFF ;												// Timer1 (is 16bit) enabled
	TCCR1A = (1<<WGM11); 										// Timer1 enable as 9 bit PWM
	//TCCR1B = (0<<CS12)|(1<<CS11)|(0<<CS10);						// Timer1 prescaler of clk/8
	TCCR1B = (1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);			// Timer1 prescaler of clk/8, as 9 bit FAST PWM -> ~2,9kHz
	TIMSK = (1<<TOIE1); 										// Timer1 overflow interrupts enabled -> at 12Mhz F_CPU/8 at 9bit*2 ~> 1,5kHz

	//External Interrupts
	MCUCR |=  (1<<ISC11)| (0<<ISC10)| (1<<ISC01)| (0<<ISC00);	// The falling edge INT0+1 generates an interrupt request
	EMCUCR |= (1<<ISC2);										// ISC2=0 -> a falling edge INT2 activates the interrupt, ISC2=1 -> a rising edge activates the interrupt
	//GICR|= (1<<INT1) | (1<<INT0) |(1<<INT2);					// external interrupts enable (PLUS, MODE, NERF)
	#ifdef EGGTIMER_MODULE
	//PORTE|= (1<<PE0);											// pullup Port E INT2
	GICR|= (1<<INT2);											// external interrupts enable (egg timer)
	#endif
	#ifdef NERFGUN_MODULE
	GICR|= (1<<INT2);											// external interrupts enable (NERF detector)

	myNerf.nerfTargetCount = 0;									// init nerf target stuff
	myNerf.nerfPeakCount = 0;
	myNerf.nerfState = 0;
	myNerf.nerfDownTime = 0;
	#endif

	timestamp1=timestamp2=dt.second;
	//timestamp2=dt.second;




	//read EEprom values to current variables
	//CFUnit= eeprom_read_byte(&ECFUnit);

	USMode= eeprom_read_byte(&EUSMode);
	Swing= eeprom_read_byte(&ESwing);
	SecMode= eeprom_read_byte(&ESecMode);
	DimMode = eeprom_read_byte(&EDimMode);

	uint8_t CRC = eeprom_read_byte(&ECRCParams);
	if (CRC != (USMode+Swing+SecMode+DimMode) )
	{
		//CRC in EEPROM incorrect =>
		//replace all values with default values
		//CFUnit=DefCFUnit;
		Swing=DefSwing;
		USMode=DefUSMode;
		SecMode=DefSecMode;
		DimMode=0;
		storeParameter(STORE_ALL);
	}
	SecModeOld = SecMode;

	//read EEprom values alarm to current variables
	ALMinutes= eeprom_read_byte(&EALMinutes);
	ALHours= eeprom_read_byte(&EALHours);
	ALSet= eeprom_read_byte(&EALSet);
	CRC= eeprom_read_byte(&ECRCAlarm);
	if ( CRC!=(ALMinutes+ALHours+ALSet) )
	{
		//CRCALARM in EEPROM incorrect =>
		//replace all values with default values
		ALMinutes=DefALMinutes;
		ALHours=DefALHours;
		ALSet=DefALSet;
		//rewrite default values in EEPROM
		eeprom_write_byte(&EALMinutes,ALMinutes);
		eeprom_write_byte(&EALHours,ALHours);
		eeprom_write_byte(&EALSet,ALSet);
		eeprom_write_byte(&ECRCAlarm,(ALMinutes+ALHours+ALSet));
	}

	#ifdef DIFFDATE_MODULE
	diffDt.second=0;											// set the date for calculation of 'days between dates'
	diffDt.hour=0;
	diffDt.minute=0;
	diffDt.date = eeprom_read_byte(&EdiffDate);
	diffDt.month = eeprom_read_byte(&EdiffMonth);
	diffDt.year= eeprom_read_byte(&EdiffYear);
	CRC = eeprom_read_byte(&ECRCDiff);
	if ( CRC != (EdiffDate+EdiffMonth+EdiffYear) )
	{
		diffDt.date = DefDiffDay;
		diffDt.month = DefDiffMonth;
		diffDt.year= DefDiffYear;
		eeprom_write_byte(&EdiffDate,diffDt.date);
		eeprom_write_byte(&EdiffMonth,diffDt.month);
		eeprom_write_byte(&EdiffYear,diffDt.year);
		eeprom_write_byte(&ECRCDiff,(diffDt.date+diffDt.month+diffDt.year));
	}
	#endif


	#ifdef MULTI_TEMPSENSORS
	ClockMode=SHOWSENSORS;
	nSensors = Search_sensors();
	digit=nSensors;
	display();
	_delay_ms(500);
	if (nSensors==0) beep();
	#else
	ClockMode=SHOWCLOCK;
	#endif // MULTI_TEMPSENSORS

	//if only SELECT pushed on startup, toggle CFUnit and USMode.
	if (bit_is_set(KPIN, KEYSELECT) && bit_is_clear(KPIN, KEYSET))
	{
		//CFUnit=!CFUnit;
		USMode=!USMode;
		//eeprom_write_byte(&ECFUnit,CFUnit);
		storeParameter(STORE_USMODE);
	}

	//if only SET pushed on startup, toggle Swing.
	if (bit_is_clear(KPIN, KEYSELECT) && bit_is_set(KPIN, KEYSET))
	{
		Swing=!Swing;
		storeParameter(STORE_SWING);
	}

	_delay_ms(1000);
	sei();			//enable interrupts
}

//main loop
int main()
{
	uint8_t key;
	init();

	while(1)
	{
		key=readkeys();

		//PLUS key was pressed
		if (key==1)
		{
			plusKeyPressed();
		}
		//MODE key pressed
		else if (key==2)
		{
			modeKeyPressed();
		}
		//no key pressed
		//if (key==0)
		else
		{
			noKeyPressed();
		}
		if(key)
		_delay_ms(250);

		display();
		CheckAlarm();

		//timer to set back display
		if( (t1==0) && (ClockMode > SHOWNERF) )		// do not set back in SHOW modes
		{
			ClockMode=SHOWCLOCK;
			TEMPDISPLAY=0;
		}
		t1--;
		

		//// calc every 5 sec
		//if( dt.second%5 == 0)
		if( ((dt.second % 3 == 0) && (ClockMode != SHOWCLOCK)) ||
		((dt.second % 9 == 0) && (ClockMode == SHOWCLOCK)) )
		{
			// swing some ClockModes (1-5)
			if( (Swing)  && (secondsOld!=dt.second) )
			{
				secondsOld = dt.second;
				if(ClockMode<=SHOWDIFFDAYS)
				{
					ClockMode++;
					if(ClockMode > SHOWDIFFDAYS)
					ClockMode=SHOWCLOCK;
				}
			}

			// dim display automaticly from 19 -> 7 o'clock
			if(DimMode == 9)
			{
				if( (dt.hour > 19) || (dt.hour < 7) )
				{
					Dim = 5;
				}
				else if(dt.hour < 9)
				{
					Dim = 3;
				}
				else if(dt.hour < 18)
				{
					Dim = 0;
				}
				else
				{
					Dim = 3;
				}
			}
		}

	}
}


