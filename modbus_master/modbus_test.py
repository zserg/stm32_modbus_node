import minimalmodbus
import serial

instrument = minimalmodbus.Instrument('/dev/ttyUSB2', 10) # port name, slave address (in decimal)
instrument.serial.baudrate = 38400   # Baud
instrument.serial.bytesize = 8
instrument.serial.parity   = serial.PARITY_EVEN
instrument.serial.stopbits = 1
instrument.debug = True

## Read temperature (PV = ProcessValue) ##
temperature = instrument.read_register(1, 1) # Registernumber, number of decimals
temperature = instrument.read_register(12, 10) # Registernumber, number of decimals

