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
  Modbus_Move_mm,                 // good up to 65meter !!
  TMC5130_ADRESS_READ,
  TMC5130_ADRESS_WRITE,
  TMC5130_LOW_WORD,
  TMC5130_HI_WORD,
  
    
  HOLDING_REGS_SIZE // leave this one
  // total number of registers for function 3 and 16 share the same register array
  // i.e. the same address space
};

unsigned int holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array
////////////////////////////////////////////////////////////


uint32_t SPI_Reister ;
uint32_t SPI_Reister_old ;
uint16_t Move_mm ;


void setup()
{
  
  uint32_t STALL_SPEED;
  int homing_speed;
  int VELOCITY_MODE_POS, VELOCITY_MODE_NEG;
  homing_speed = 4000;
  _delay_ms(100);
  
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
 
  _delay_ms(1000);
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
	
  modbus_configure(&Serial, 19200, SERIAL_8N2, 1, 16, HOLDING_REGS_SIZE, holdingRegs);

        InitSPI();
       
  
        SPIWrite(CHOPCONF, 0x101D5);        
	SPIWrite(IHOLD_IRUN, 0x70800);	//IHOLD and IRUN current

	SPIWrite(RAMPMODE, 0x0);  //select position mode
        SPIWrite(A_1, 0x3E8);     //First  acceleration  /  deceleration  phase threshold velocity 
	SPIWrite(V_1, 0x0);	  //Disables A1 and D1 in position mode, amax and vmax only
	SPIWrite(D_1, 0x578);	//D1 not zero
	SPIWrite(AMAX, 0x1F4);	//Acceleration
	SPIWrite(VMAX, 0x186A0);  //Velocity
        SPIWrite(VSTOP, 0xA);
        SPIWrite(DMAX, 0x2BC);
	SPIWrite(GCONF, 0x4);

        SPIWrite(XACTUAL, 0x0);	        //XACTUAL = 0
        SPIWrite(XTARGET,512000 );   // let make 10 turn for fun ...
}

void loop()
{
  // modbus_update() is the only method used in loop(). It returns the total error
  // count since the slave started. You don't have to use it but it's useful
  // for fault finding by the modbus master.
  
  modbus_update();  
  
// old test code  SPI_Reister = 0x12345678;  
// old test code  holdingRegs [TMC5130_LOW_WORD]  =  (SPI_Reister >> 16) & 0xFFFF;   ; // update data to be read by the master to adjust the PWM
// old test code  holdingRegs [TMC5130_HI_WORD]   =  SPI_Reister  & 0xFFFF;
 
// if (SPI_Reister_old != SPI_Reister ) 
//   { 
     SPI_Reister =   holdingRegs[Modbus_Move_mm] * 700.49 ;   ///compute step per mm   
     holdingRegs[TMC5130_LOW_WORD] = SPI_Reister;  //  return calculated mm value for debog 
     SPIWrite(XTARGET,SPI_Reister );
     SPI_Reister_old == SPI_Reister;  
//   }
   
  

  /* Note:
     The use of the enum instruction is not needed. You could set a maximum allowable
     size for holdinRegs[] by defining HOLDING_REGS_SIZE using a constant and then access 
     holdingRegs[] by "Index" addressing. 
     I.e.
     holdingRegs[0] = analogRead(A0);
     analogWrite(LED, holdingRegs[1]/4);
  */
  
}

