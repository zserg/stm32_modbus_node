import minimalmodbus
import serial
import time

instrument = minimalmodbus.Instrument('/dev/ttyUSB1', 10, minimalmodbus.MODE_ASCII) # port name, slave address (in decimal)
instrument.serial.baudrate = 38400   # Baud
instrument.serial.bytesize = 8
instrument.serial.parity   = serial.PARITY_NONE
instrument.serial.stopbits = 1
instrument.serial.timeout = 1
#instrument.debug = True

## Read temperature (PV = ProcessValue) ##
while True:
   temperature = instrument.read_register(1, 0) # Registernumber, number of decimals
   print("Temperature is {} C".format(temperature/16.0))
   time.sleep(2)
#temperature = instrument.read_register(12, 10) # Registernumber, number of decimals

