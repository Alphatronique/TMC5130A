# TMC5130A
TMC5130A Arduino Librarie

Basic SPI for control TMC5130A Chip from Arduino

the .ino have exemple code for move motor to desired position in milimeter via modbus
that code was verry usefull for drive a StepIR antenna ;-)   

this use modbus librarie from  https://github.com/pepsilla/Arduino/tree/master/MODBUS/ASCII/libraries/SimpleModbusSlave

modbuss adress  = 1 
register 0 = position in mm  ( 0 to 65535 mm)

19200 8 N 2

Best 73 !  Marc L   VE2OLM
