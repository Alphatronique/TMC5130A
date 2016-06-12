#include <SimpleModbusSlave.h>
#include <TMC_SPI.h>
#include <Adafruit_NeoPixel.h>

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
  Modbus_temperature,
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


static uint32_t SPI_Reister ;
static uint32_t SPI_Reister_old ;
uint16_t Move_mm ;
int TMP36_Pin = A7;



Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, 14, NEO_GRB + NEO_KHZ800);


void setup()
{   	
  modbus_configure(&Serial, 38400, SERIAL_8N1, 1, 16, HOLDING_REGS_SIZE, holdingRegs);

        InitSPI();   
    
         delay(1000);
  
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
        
 //       delay(50);
        SPIWrite(XTARGET,10000 );   // let make 10 turn for fun ...
        
         pixels.begin();        
         pixels.setPixelColor(0, pixels.Color(0,10,0)); // Moderately bright green color.
         pixels.show(); // This sends the updated pixel color to the hardware.
        
}

void loop()
{
  // modbus_update() is the only method used in loop(). It returns the total error
  // count since the slave started. You don't have to use it but it's useful
  // for fault finding by the modbus master.
  
  modbus_update(); 
  
  SPI_Reister =   holdingRegs[Modbus_Move_mm] * 700.49 ;   ///compute requested step per mm  from modbus register 0  
  
 int reading = analogRead(TMP36_Pin);  //getting the voltage reading from the temperature sensor
 float voltage = reading * 5.0;    // converting that reading to voltage
 voltage /= 1024.0; 
 float temperatureC = (voltage - 0.5) * 100 ; // converting Celsus
 
 holdingRegs[Modbus_temperature] = temperatureC; // put it on modbus 
 holdingRegs[TMC5130_LOW_WORD] = SPI_Reister;  //  return calculated mm value for debog 
 
 if (SPI_Reister_old != SPI_Reister ) 
   { 
     pixels.setPixelColor(0, pixels.Color(10,0,0)); 
     pixels.show(); // This sends the updated pixel color to the hardware.
     SPIWrite(XTARGET,SPI_Reister );   
     _delay_ms(50);	  
     pixels.setPixelColor(0, pixels.Color(0,10,0));
     pixels.show(); // This sends the updated pixel color to the hardware. 
     SPI_Reister_old = SPI_Reister;     
   } 
   
// old test code  SPI_Reister = 0x12345678;  
// old test code  holdingRegs [TMC5130_LOW_WORD]  =  (SPI_Reister >> 16) & 0xFFFF;   ; // update data to be read by the master to adjust the PWM
// old test code  holdingRegs [TMC5130_HI_WORD]   =  SPI_Reister  & 0xFFFF;
}

