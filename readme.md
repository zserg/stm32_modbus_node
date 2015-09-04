
stm32_node
==========

Control node based on STM32F103C8TC board (image below)
![board](https://github.com/zserg/stm32_node/blob/master/Doc/stm32f103_brd.jpg)
![board](https://github.com/zserg/stm32_node/blob/master/Doc/stm32f103_in_case.jpg)


#### Configuration:
* LED (PA8)
* UART1 (PA10 - RX, PA9 - TX)
* Ultrasomic Sensor HC-SR04 (TRIG - PB6, ECHO - PB7)
* OneWrie Temperature Sensor DS18B20 (PB11)

#### Functionality
Board measures temperature and distance and send the following information to UART(115200) one time per 2 seconds:

<code>
Raw Temp: 0x18 (24 C);  Distance: 1234567 (us) INFO: 58us per 1 cm;
</code>

#### DSUB connector pinout
 1. VCC (9-24V)
 2. RS485(B)
 3. RS485(A)
 4. reserved
 5. +5V (output)
 6. GND
 7. HC-SR04(Trigger)
 8. HC-SR04(Echo)
 9. Onewire
 
#### Toolchain
[https://github.com/zserg/stm32_tools](https://github.com/zserg/stm32_tools)



