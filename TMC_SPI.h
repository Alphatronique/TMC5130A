#ifndef TMC_SPI_H
#define TMC_SPI_H

#include "TMC_SPI.h"
#include "fastio.h"
#include "Arduino.h"

//#include "Marlin.h"
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <inttypes.h>
#include <util/delay.h>

#define  FORCE_INLINE __attribute__((always_inline)) inline
#define nop() asm volatile("nop")


// TRINAMIC TMC5130 Register Address Defines
#define GCONF				0x00 	//Global configuration flags
#define X_COMPARE 			0x05	//Position  comparison  register
#define IHOLD_IRUN			0x10	//Driver current control
#define RAMPMODE			0x20	//Driving mode (Velocity, Positioning, Hold)
#define XACTUAL				0x21	//Actual motor position
#define VACTUAL 			0x22	//Actual  motor  velocity  from  ramp  generator
#define VSTART				0x23	//Motor start velocity
#define A_1					0x24	//First  acceleration  between  VSTART  and  V1
#define V_1					0x25	//First  acceleration  /  deceleration  phase  target velocity
#define AMAX				0x26	//Second  acceleration  between  V1  and  VMAX
#define VMAX 				0x27	//This is the target velocity in velocity mode. It can be changed any time during a motion.
#define DMAX				0x28	//Deceleration between VMAX and V1
#define D_1					0x2A 	//Deceleration  between  V1  and  VSTOP
									//Attention:  Do  not  set  0  in  positioning  mode, even if V1=0!
#define VSTOP				0x2B	//Motor stop velocity (unsigned)
									//Attention: Set VSTOP > VSTART!
									//Attention:  Do  not  set  0  in  positioning  mode, minimum 10 recommend!
#define TZEROWAIT			0x2C	//Defines  the  waiting  time  after  ramping  down
									//to  zero  velocity  before  next  movement  or
									//direction  inversion  can  start.  Time  range  is about 0 to 2 seconds.
#define XTARGET				0x2D	//Target position for ramp mode
#define SW_MODE 			0x34	//Switch mode configuration
#define RAMP_STAT			0x35	//Ramp status and switch event status
#define XLATCH				0x36	//Latches  XACTUAL  upon  a programmable switch event
#define CHOPCONF			0x6C	//Chopper and driver configuration
#define COOLCONF			0x6D	//coolStep smart current control register and stallGuard2 configuration
#define DRV_STATUS 			0x6F	//stallGuard2 value and driver error flags

void InitSPI();
uint8_t ReadWriteSPI(uint8_t Data);
uint32_t SPIRead(uint8_t address);
FORCE_INLINE void SPIWrite(uint8_t address, uint32_t instruction)
{
	uint8_t DATA1, DATA2, DATA3, DATA0;

	DATA0 = 0;
	DATA1 = 0;
	DATA2 = 0;
	DATA3 = 0;

	DATA0 = instruction & 0xFF;
	DATA1 = (instruction & 0xFF00) >> 8;
	DATA2 = (instruction & 0xFF0000) >> 16;
	DATA3 = (instruction & 0xFF000000) >> 24;

	address = address | 0x80;

	PORTB &= ~(1 << 2); //csPin Low

	SPDR=address;
	while(!(SPSR & 0x80));

	SPDR=DATA3;
	while(!(SPSR & 0x80));

	SPDR=DATA2;
	while(!(SPSR & 0x80));

	SPDR=DATA1;
	while(!(SPSR & 0x80));

	SPDR=DATA0;
	while(!(SPSR & 0x80));

	PORTB |= (1 << 2);//csPin High
}
uint8_t SPIRead_Status_Bits();

#endif //TMC_SPI_H
