#pragma once
#include <cstdint>

//---------------------------------------------------------------------------//
// Change History                                                            //
//                                                                           //
// JRK  12/16/05 - Initial Version                                           //
//                                                                           //
//---------------------------------------------------------------------------//
#ifndef picioH
#define picioH
#endif

//--------------------- IO Module specific stuff ----------------------------//
typedef struct _IOMOD {
    short int	inbits;			//input bits
    uint8_t ad1;			//A/D input uint8_ts
    uint8_t ad2;
    uint8_t		  ad3;
    unsigned long timer; 		//timer value
    short int	inbits_s;		//synchronized input uint8_ts
    unsigned long timer_s;		//synchronized timer value
    //The following data is stored locally for reference
    uint8_t		pwm1;			//current PWM output values
    uint8_t		pwm2;
    uint8_t		timermode;		//current timer mode
    short int	bitdir;			//current bit direction values
    short int	outbits;		//current output uint8_t values
} IOMOD;

//IO Module Command set:
#define	SET_IO_DIR	  0x00	//Set direction of IO bits (2 data uint8_ts)
#define	SET_ADDR	  0x01	//Set address and group address (2 uint8_ts)
#define	DEF_STAT	  0x02	//Define status items to return (1 uint8_t)
#define	READ_STAT	  0x03	//Read value of current status items
#define	SET_PWM   	  0x04	//Immediatley set PWM1 and PWM2 (2 uint8_ts)
#define SYNCH_OUT	  0x05	//Output prev. stored PWM & output uint8_ts (0 uint8_ts)
#define SET_OUTPUT	  0x06  //Immediately set output uint8_ts
#define	SET_SYNCH_OUT 0x07	//Store PWM & outputs for synch'd output (4 uint8_ts)
#define	SET_TMR_MODE  0x08	//Set the counter/timer mode (1 uint8_t)
//Not used			  0x09
#define	SET_BAUD	  0x0A 	//Set the baud rate (1 uint8_t)
//Not used			  0x0B
#define SYNCH_INPUT	  0x0C	//Store the input uint8_ts and timer val (0 uint8_ts)
//Not used			  0x0D
#define	NOP			  0x0E	//No operation - returns prev. defined status (0 uint8_ts)
#define HARD_RESET	  0x0F	//RESET - no status is returned

//IO Module STATUSITEMS bit definitions
#define	SEND_INPUTS	  0x01	//2 uint8_ts data
#define	SEND_AD1	  0x02	//1 uint8_t
#define	SEND_AD2	  0x04	//1 uint8_t
#define SEND_AD3	  0x08	//1 uint8_t
#define SEND_TIMER	  0x10	//4 uint8_ts
#define SEND_ID		  0x20	//2 uint8_ts
#define	SEND_SYNC_IN  0x40	//2 uint8_ts
#define	SEND_SYNC_TMR 0x80	//4 uint8_ts

//IO Module Timer mode definitions
//Timer mode and resolution may be OR'd together
#define	OFFMODE		  0x00
#define	COUNTERMODE	  0x03
#define	TIMERMODE	  0x01
#define	RESx1		  0x00
#define RESx2		  0x10
#define RESx4		  0x20
#define RESx8		  0x30

//IO module function prototypes:
extern "C" __declspec(dllexport) IOMOD* IoNewMod();
extern "C" __declspec(dllexport) BOOL IoGetStat(uint8_t addr);
extern "C" __declspec(dllexport) BOOL IoInBitVal(uint8_t addr, int bitnum);
extern "C" __declspec(dllexport) BOOL IoInBitSVal(uint8_t addr, int bitnum);
extern "C" __declspec(dllexport) BOOL IoOutBitVal(uint8_t addr, int bitnum);
extern "C" __declspec(dllexport) BOOL IoGetBitDir(uint8_t addr, int bitnum);
extern "C" __declspec(dllexport) uint8_t IoGetADCVal(uint8_t addr, int channel);
extern "C" __declspec(dllexport) uint8_t IoGetPWMVal(uint8_t addr, int channel);
extern "C" __declspec(dllexport) unsigned long IoGetTimerVal(uint8_t addr);
extern "C" __declspec(dllexport) unsigned long IoGetTimerSVal(uint8_t addr);
extern "C" __declspec(dllexport) uint8_t IoGetTimerMode(uint8_t addr);

extern "C" __declspec(dllexport) BOOL IoSetOutBit(uint8_t addr, int bitnum);
extern "C" __declspec(dllexport) BOOL IoClrOutBit(uint8_t addr, int bitnum);
extern "C" __declspec(dllexport) BOOL IoBitDirIn(uint8_t addr, int bitnum);
extern "C" __declspec(dllexport) BOOL IoBitDirOut(uint8_t addr, int bitnum);
extern "C" __declspec(dllexport) BOOL IoSetPWMVal(uint8_t addr, uint8_t pwm1, uint8_t pwm2);
extern "C" __declspec(dllexport) BOOL IoSetTimerMode(uint8_t addr, uint8_t tmrmode);
extern "C" __declspec(dllexport) BOOL IoSetSynchOutput(uint8_t addr, short int outbits, uint8_t pwm1, uint8_t pwm2);

