//********************************************
//* (c) 2015 Cristian Copcea                 *
//* Please retain here the credits           *
//* just add yours if you make modifications *
//* to this file                             *
//********************************************

//DS1621 addresses for Write and Read
#define  DS1621_W			0x90
#define  DS1621_R			0x91

//DS1621 commands
#define	 START_CONVERT		0xEE
#define	 STOP_CONVERT		0x22
#define	 READ_TEMP			0xAA
#define	 READ_COUNTER		0xA8
#define	 READ_SLOPE			0xA9
#define  ACCESS_CONFIG		0xAC

extern unsigned char ds1621_readValue ( unsigned char );
extern uint8_t get_ds1621_temperature ( void );

