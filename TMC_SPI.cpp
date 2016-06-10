#include "TMC_SPI.h"
#include "fastio.h"

// SPI Initialization
void InitSPI()
{
		//Initialize chip select pin
	uint8_t clr;
	
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SS, OUTPUT);
  pinMode(15 , OUTPUT);  // DRVIVE_ENABLE 
   
  digitalWrite(SCK, LOW);
  digitalWrite(MOSI, LOW);
  digitalWrite(SS, HIGH);
  digitalWrite(15, LOW); 

	//Initialize SPI registers
	clr=SPSR;
	clr=SPDR;
	delay(10);

	//Initialize SPI clock
	SPCR = 0;
	SPCR = (1<<SPE)|(1<<MSTR);//	4 MHz
}


// SPI Write and Read
 uint8_t ReadWriteSPI(uint8_t Data)
{
	SPDR=Data;
	while(!(SPSR & 0x80));
	return SPDR;
}



 uint32_t SPIRead(uint8_t address)
 {
		uint8_t DATA1, DATA2, DATA3, DATA0;
		uint32_t register_value = 0;

		DATA0 = 0;
		DATA1 = 0;
		DATA2 = 0;
		DATA3 = 0;

		PORTB &= ~(1 << 2); //csPin Low

		ReadWriteSPI(address);
		DATA3 = ReadWriteSPI(0x00);
		DATA2 = ReadWriteSPI(0x00);
		DATA1 = ReadWriteSPI(0x00);
		DATA0 = ReadWriteSPI(0x00);

		PORTB |= (1 << 2);//csPin High

		nop();
		nop();
		nop();

		PORTB &= ~(1 << 2); //csPin Low

		ReadWriteSPI(address);
		DATA3 = ReadWriteSPI(0x00);
		DATA2 = ReadWriteSPI(0x00);
		DATA1 = ReadWriteSPI(0x00);
		DATA0 = ReadWriteSPI(0x00);

		PORTB |= (1 << 2);//csPin High

		register_value |= DATA3;
		register_value = register_value << 8;
		register_value |= DATA2;
		register_value = register_value << 8;
		register_value |= DATA1;
		register_value = register_value << 8;
		register_value |= DATA0;

		return register_value;
 }

uint8_t SPIRead_Status_Bits()
{
	PORTB &= ~(1 << 2); //csPin Low
	uint8_t Status_Bits = ReadWriteSPI(0x27);
	PORTB |= (1 << 2);//csPin High
	return Status_Bits;
}


