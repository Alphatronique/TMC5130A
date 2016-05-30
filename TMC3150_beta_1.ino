#include <SimpleModbusSlave.h>
#include "TMC_SPI.h"

/* 
   SimpleModbusSlaveV10 supports function 3, 6 & 16.
   
   This example code will receive the adc ch0 value from the arduino master. 
   It will then use this value to adjust the brightness of the led on pin 9.
   The value received from the master will be stored in address 1 in its own
   address space namely holdingRegs[].
   
   In addition to this the slaves own adc ch0 value will be stored in 
   address 0 in its own address space holdingRegs[] for the master to
   be read. The master will use this value to alter the brightness of its
   own led connected to pin 9.
   
   The modbus_update() method updates the holdingRegs register array and checks
   communication.

   Note:  
   The Arduino serial ring buffer is 64 bytes or 32 registers.
   Most of the time you will connect the arduino to a master via serial
   using a MAX485 or similar.
 
   In a function 3 request the master will attempt to read from your
   slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
   and two BYTES CRC the master can only request 58 bytes or 29 registers.
 
   In a function 16 request the master will attempt to write to your 
   slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS, 
   NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
   54 bytes or 27 registers.
 
   Using a USB to Serial converter the maximum bytes you can send is 
   limited to its internal buffer which differs between manufactures. 
*/

#define  LED 9  
#define  SPI_MODE 2  
#define  SD_MODE 3 
#define  LDO_EN 4  
#define  SW_SEL 9  
#define  DRVIVE_ENABLE 15 

// Using the enum instruction allows for an easy method for adding and 
// removing registers. Doing it this way saves you #defining the size 
// of your slaves register array each time you want to add more registers
// and at a glimpse informs you of your slaves register layout.

//////////////// registers of your slave ///////////////////
enum 
{     
  // just add or remove registers and your good to go...
  // The first register starts at address 0
  TMC5130_ADRESS_WRITE,
  TMC5130_BYTE_1_WRITE,
  TMC5130_BYTE_2_WRITE,
  TMC5130_BYTE_3_WRITE,
  TMC5130_BYTE_4_WRITE,
  
  TMC5130_ADRESS_READ,
  TMC5130_BYTE_1_READ,
  TMC5130_BYTE_2_READ,
  TMC5130_BYTE_3_READ,
  TMC5130_BYTE_4_READ, 
  
  HOLDING_REGS_SIZE // leave this one
  // total number of registers for function 3 and 16 share the same register array
  // i.e. the same address space
};

unsigned int holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array
////////////////////////////////////////////////////////////

void setup()
{
  
  uint32_t STALL_SPEED;
  int homing_speed;
  int VELOCITY_MODE_POS, VELOCITY_MODE_NEG;
  homing_speed = 4000;
  
  pinMode(LDO_EN, OUTPUT);   // LDO_EN PIN
  digitalWrite(LDO_EN, LOW);  // DISABLE tmc5130 LDO "RESET"
  
  pinMode(SPI_MODE , OUTPUT);   // SPI_MODE 
  digitalWrite(SPI_MODE, HIGH);   // SPI INTERFACE SELECTED
  
  pinMode(SD_MODE, OUTPUT);   // SD_MODE
  digitalWrite(SD_MODE, LOW);  // DISABLE tmc5130 LDO "RESET"
  
  pinMode(SW_SEL, OUTPUT);  // SW_SEL 
  digitalWrite(SW_SEL, LOW);
  
  pinMode(DRVIVE_ENABLE , OUTPUT);  // DRVIVE_ENABLE 
  digitalWrite(DRVIVE_ENABLE, LOW); 
 
  _delay_ms(200);
  digitalWrite(LDO_EN, HIGH);  // ONSET SETUP DONE ENABLE LDO 
  
 
  /* parameters(HardwareSerial* SerialPort,
                long baudrate, 
		unsigned char byteFormat,
                unsigned char ID, 
                unsigned char transmit enable pin, 
                unsigned int holding registers size,
                unsigned int* holding register array)
  */
  
  /* Valid modbus byte formats are:
     SERIAL_8N2: 1 start bit, 8 data bits, 2 stop bits
     SERIAL_8E1: 1 start bit, 8 data bits, 1 Even parity bit, 1 stop bit
     SERIAL_8O1: 1 start bit, 8 data bits, 1 Odd parity bit, 1 stop bit
     
     You can obviously use SERIAL_8N1 but this does not adhere to the
     Modbus specifications. That said, I have tested the SERIAL_8N1 option 
     on various commercial masters and slaves that were suppose to adhere
     to this specification and was always able to communicate... Go figure.
     
     These byte formats are already defined in the Arduino global name space. 
  */
	
  modbus_configure(&Serial, 115200, SERIAL_8N2, 1, 2, HOLDING_REGS_SIZE, holdingRegs);

  // modbus_update_comms(baud, byteFormat, id) is not needed but allows for easy update of the
  // port variables and slave id dynamically in any function.
  modbus_update_comms(115200, SERIAL_8N2, 1);
  
        InitSPI();
       
  
  
	SPIWrite(IHOLD_IRUN, 0x71703);	//IHOLD and IRUN current
	SPIWrite(RAMPMODE, 0x0);		//select position mode
	SPIWrite(V_1, 0x0);		//Disables A1 and D1 in position mode, amax and vmax only
	SPIWrite(D_1, 0x10);	//D1 not zero
	SPIWrite(AMAX, 0xFFFF);	//Acceleration
	SPIWrite(VMAX, 0xFFFF);  //Velocity
	SPIWrite(CHOPCONF, 0x140101D5);
	SPIWrite(GCONF, 0x1084);

	SPIWrite(SW_MODE, 0x00);	//SWITCH REGISTER

	STALL_SPEED = 16777216 / homing_speed;
	STALL_SPEED = STALL_SPEED / 16;  // match homing speed to actual microstep speed (at 1/16 microstep)
	STALL_SPEED = STALL_SPEED * 1.10; // Activate stallGuard sligthly below desired homing velocity (provide 10% tolerance)

	SPIWrite(GCONF, 0x1080);	//stealthchop off for stallguard homing
	SPIWrite(0x6D, 0x00080000);//sgt <-- Entry the value determined for SGT: lower value=higher sensitivity (lower force for stall detection)
	SPIWrite(0x14, STALL_SPEED);//TCOOLTHRS
	SPIWrite(SW_MODE, 0x400);	//SWITCH REGISTER
	SPIWrite(AMAX, 100);	//AMAX for stallGuard homing shall be significantly lower than AMAX for printing

	// Set velocity mode in direction to the endstop
	SPIWrite(RAMPMODE, VELOCITY_MODE_NEG);	//VELOCITY MODE negative Direction
	SPIWrite(VMAX, homing_speed);	        //Homing Speed in VMAX

	//Enable Trinamic Drivers to start homing movement
	//todo enable all chips every time?
//	pinMode(38, OUTPUT);
//	digitalWrite(38, LOW);
//	pinMode(62, OUTPUT);
//	digitalWrite(62, LOW);
//	pinMode(56, OUTPUT);
//	digitalWrite(56, LOW);
//	pinMode(24, OUTPUT);
//	digitalWrite(24, LOW);

	//While motor is still moving (vzero != 1)
	while((SPIRead(RAMP_STAT) & 0x400) != 0x400);     // <--  spi read

	// Endstop reached. Reset and retract
	SPIWrite(SW_MODE, 0x0);	        //SWITCH REGISTER
	SPIWrite(RAMPMODE, 0x3);	//HOLD Mode
	SPIWrite(GCONF, 0x1084);        //Turn on stealthchop again
	SPIWrite(XACTUAL, 0x0);	        //XACTUAL = 0
	SPIWrite(XTARGET, 0x0);	        //XTARGET = 0
	SPIWrite(RAMPMODE, 0x0);	//Position MODE
	_delay_ms(200);

//	SPIWrite(VMAX, nominal_speed_x);//Velocity of X
//	SPIWrite(AMAX, accel_x);//ACC of X
//	SPIWrite(DMAX, accel_x);//DEC of X
//	SPIWrite(VSTART, initial_speed_x);//Initial vel of X
//	SPIWrite(VSTOP, final_speed_x);//Final vel of X


  
}

void loop()
{
  // modbus_update() is the only method used in loop(). It returns the total error
  // count since the slave started. You don't have to use it but it's useful
  // for fault finding by the modbus master.
  
  modbus_update();
  
//  holdingRegs[ADC_VAL] = analogRead(A0); // update data to be read by the master to adjust the PWM
  
//  analogWrite(LED, holdingRegs[PWM_VAL]>>2); // constrain adc value from the arduino master to 255

  SPIWrite(holdingRegs[TMC5130_ADRESS_WRITE], holdingRegs[TMC5130_BYTE_1_WRITE]);// target
  

  /* Note:
     The use of the enum instruction is not needed. You could set a maximum allowable
     size for holdinRegs[] by defining HOLDING_REGS_SIZE using a constant and then access 
     holdingRegs[] by "Index" addressing. 
     I.e.
     holdingRegs[0] = analogRead(A0);
     analogWrite(LED, holdingRegs[1]/4);
  */
  
}

//config1: .DB 0xEC, 0x00 ,0x01 ,0x00 ,0xC5 //SPI send: 0x EC 00 01 00 C5;  CHOPCONF: TOFF=5, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle) 
//config2: .DB 0x90, 0x00 ,0x06 ,0x1f ,0x0A //SPI send: 0x 90 00 06 1F 0A;  IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6 
//config3: .DB 0x91, 0x00 ,0x00 ,0x00 ,0x0A //SPI send: 0x 91 00 00 00 0A;  TPOWERDOWN=10: Delay before power down in stand still 
//config4: .DB 0x80, 0x00 ,0x00 ,0x00 ,0x04 //SPI send: 0x 80 00 00 00 04;  EN_PWM_MODE=1 enables stealthChop (with default PWM_CONF) 
//config5: .DB 0x93, 0x00 ,0x00 ,0x01 ,0xF4 //SPI send: 0x 93 00 00 01 F4;  TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM 
//config6: .DB 0xF0, 0x00 ,0x04 ,0x01 ,0xC8 //SPI send: 0x F0 00 04 01 C8;/ PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1    

//setup1: .DB 0xA4, 0x00 ,0x00 ,0x03 ,0xE8 //SPI send: A4 00 00 03 E8; A1 = 1 000 First acceleration
//setup2: .DB 0xA5, 0x00 ,0x00 ,0xC3 ,0x50 //SPI send: A5 00 00 C3 50; V1 = 50 000 Acceleration threshold velocity V1
//setup3: .DB 0xA6, 0x00 ,0x00 ,0x01 ,0xF4 //SPI send: A6 00 00 01 F4; AMAX = 500 Acceleration above V1
//setup4: .DB 0xA7, 0x00 ,0x00 ,0x0D ,0x40 //SPI send: A7 00 03 0D 40; VMAX = 200 000
//setup5: .DB 0xA8, 0x00 ,0x00 ,0x02 ,0xBC //SPI send: A8 00 00 02 BC; DMAX = 700 Deceleration above V1
//setup6: .DB 0xAA, 0x00 ,0x00 ,0x05 ,0x78 //SPI send: AA 00 00 05 78; D1 = 1400 Deceleration below V1
//setup7: .DB 0xAB, 0x00 ,0x00 ,0x00 ,0x0A //SPI send: AB 00 00 00 0A; VSTOP = 10 Stop velocity (Near to zero)
//setup8: .DB 0xA0, 0x00 ,0x00 ,0x00 ,0x00 //SPI send: A0 00 00 00 00; RAMPMODE = 0 (Target position move)
//move  : .DB 0xAD, 0xFF ,0xFF ,0x38 ,0x00 //SPI send: AD FF FF 38 00; XTARGET = -51200 (Move one rotation left (200*256 microsteps) 
