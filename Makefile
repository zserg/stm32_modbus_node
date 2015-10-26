#
#       !!!! Do NOT edit this makefile with an editor which replace tabs by spaces !!!!   
#
##############################################################################################
#
# On command line:
#
# make all = Create project
#
# make clean = Clean project files.
#
# To rebuild project do "make clean" and "make all".
#
# Included originally in the yagarto projects. Original Author : Michael Fischer
# Modified to suit our purposes by Hussam Al-Hertani
# Use at your own risk!!!!!
##############################################################################################
# Start of default section
#
CCPREFIX = arm-none-eabi-
CC   = $(CCPREFIX)gcc
CP   = $(CCPREFIX)objcopy
AS   = $(CCPREFIX)gcc -x assembler-with-cpp
GDBTUI = $(CCPREFIX)gdbtui
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary -S
MCU  = cortex-m3
DEVICE = STM32F103xB

# List all C defines here
DDEFS = -D$(DEVICE) -DUSE_STDPERIPH_DRIVER -DGCC_ARMCM3
#
# Define project name and Ram/Flash mode here
PROJECT        = main
 
# List C source files here
LIBSDIRS    = $(STM32CUBEROOT)/Drivers
CORELIBDIR = $(LIBSDIRS)/CMSIS/Include
RTOSLIBDIR = $(STM32CUBEROOT)/Middlewares/Third_Party/FreeRTOS/Source
DEVDIR  = $(LIBSDIRS)/CMSIS/Device/ST/STM32F1xx
STMSPDDIR    = $(LIBSDIRS)/STM32F1xx_HAL_Driver
STMSPSRCDDIR = $(STMSPDDIR)/Src
STMSPINCDDIR = $(STMSPDDIR)/Inc
RTOSSRCDIR = $(STM32CUBEROOT)/Middlewares/Third_Party/FreeRTOS/Source
#list of src files to include in build process

SRC  = ./Src/main.c
SRC += ./Src/usart_log.c
SRC += ./Src/stm32f1xx_it.c
SRC += ./Src/stm32f1xx_hal_msp.c
#SRC += ./stm32fonewire/tm_stm32f4_onewire.c
#SRC += ./stm32fonewire/tm_stm32f4_ds18b20.c
#SRC += ./stm32f_hcsr04/tm_stm32f4_hcsr04.c
SRC += ./modbus_port/port/portevent.c
SRC += ./modbus_port/port/portother.c
SRC += ./modbus_port/port/portserial.c
#SRC += ./modbus_port/port/portserial_new.c
SRC += ./modbus_port/port/porttimer.c
SRC += ./modbus/mb.c
SRC += ./modbus/rtu/mbcrc.c
SRC += ./modbus/rtu/mbrtu.c
SRC += ./modbus/ascii/mbascii.c
SRC += ./modbus/functions/mbfunccoils.c
SRC += ./modbus/functions/mbfuncdiag.c
SRC += ./modbus/functions/mbfuncdisc.c
SRC += ./modbus/functions/mbfuncholding.c
SRC += ./modbus/functions/mbfuncinput.c
SRC += ./modbus/functions/mbfuncother.c
SRC += ./modbus/functions/mbutils.c
SRC += ./Src/freertos.c
SRC += ./Src/syscalls.c
SRC += $(STMSPSRCDDIR)/stm32f1xx_hal_rcc_ex.c
SRC += $(STMSPSRCDDIR)/stm32f1xx_hal_gpio.c
SRC += $(STMSPSRCDDIR)/stm32f1xx_hal.c
SRC += $(STMSPSRCDDIR)/stm32f1xx_hal_cortex.c
SRC += $(STMSPSRCDDIR)/stm32f1xx_hal_rcc.c
SRC += $(STMSPSRCDDIR)/stm32f1xx_hal_tim.c
SRC += $(STMSPSRCDDIR)/stm32f1xx_hal_tim_ex.c
SRC += $(STMSPSRCDDIR)/stm32f1xx_hal_dma.c
SRC += $(STMSPSRCDDIR)/stm32f1xx_hal_uart.c
SRC += $(STMSPSRCDDIR)/stm32f1xx_hal_can.c

SRC += $(DEVDIR)/Source/Templates/system_stm32f1xx.c
SRC += $(RTOSSRCDIR)/CMSIS_RTOS/cmsis_os.c
SRC += $(RTOSSRCDIR)/portable/GCC/ARM_CM3/port.c
SRC += $(RTOSSRCDIR)/queue.c
SRC += $(RTOSSRCDIR)/tasks.c
SRC += $(RTOSSRCDIR)/list.c
SRC += $(RTOSSRCDIR)/portable/MemMang/heap_1.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_adc.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_cec.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_crc.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_comp.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_dac.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_dbgmcu.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_dma.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_exti.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_flash.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_syscfg.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_i2c.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_iwdg.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_pwr.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_rtc.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_spi.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_usart.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_wwdg.c
#SRC += $(STMSPSRCDDIR)/stm32f1xx_misc.c

# List assembly startup source file here
STARTUP = ./startup/startup_stm32f103c8.s

# List all directories here
INCDIRS = $(DEVDIR)/Include \
	  $(LIBSDIRS)/STM32F1xx_HAL_Driver/Inc \
	  $(CORELIBDIR) \
          $(STMSPINCDDIR) \
	  $(RTOSLIBDIR)/include \
	  $(RTOSLIBDIR)/CMSIS_RTOS \
	  ./stm32fonewire \
	  ./stm32f_hcsr04 \
	  ./modbus_port/port \
	  ./modbus/include \
	  ./modbus/rtu \
	  ./modbus/ascii \
          ./Inc
# List the user directory to look for the libraries here
LIBDIRS += $(LIBSDIRS)
 
# List all user libraries here
LIBS =
 
# Define optimisation level here
OPT = -Os
 

# Define linker script file here
LINKER_SCRIPT = ./linker/STM32F103VB_FLASH.ld

 
INCDIR  = $(patsubst %,-I%, $(INCDIRS))
LIBDIR  = $(patsubst %,-L%, $(LIBDIRS))
LIB     = $(patsubst %,-l%, $(LIBS))
##reference only flags for run from ram...not used here
##DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=0 -DVECT_TAB_SRAM

## run from Flash
DEFS    = $(DDEFS) -DRUN_FROM_FLASH=1

OBJS  = $(STARTUP:.s=.o) $(SRC:.c=.o)
MCFLAGS = -mcpu=$(MCU)
 
ASFLAGS = $(MCFLAGS) -g -ggdb -mthumb  -Wa,-amhls=$(<:.s=.lst) 
CPFLAGS = $(MCFLAGS) $(OPT) -g -ggdb -mthumb   -fomit-frame-pointer -Wall -Wstrict-prototypes -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)
LDFLAGS = $(MCFLAGS) -g -ggdb -mthumb -nostartfiles -T$(LINKER_SCRIPT) -Wl,-Map=$(PROJECT).map,--cref,--no-warn-mismatch $(LIBDIR) $(LIB)
 
#
# makefile rules
#
 
all: $(OBJS) $(PROJECT).elf  $(PROJECT).hex $(PROJECT).bin
	$(TRGT)size $(PROJECT).elf
 
%o: %c
	$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@

%o: %s
	$(AS) -c $(ASFLAGS) $< -o $@


%elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

%hex: %elf
	$(HEX) $< $@
	
%bin: %elf
	$(BIN)  $< $@
	
flash_openocd: $(PROJECT).bin
	openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f1x_stlink.cfg -c "init" -c "reset halt" -c "sleep 100" -c "wait_halt 2" -c "flash write_image erase $(PROJECT).bin 0x08000000" -c "sleep 100" -c "verify_image $(PROJECT).bin 0x08000000" -c "sleep 100" -c "reset run" -c shutdown

flash_stlink: $(PROJECT).bin
	st-flash  write $(PROJECT).bin 0x8000000

erase_openocd:
	openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f1x_stlink.cfg -c "init" -c "reset halt" -c "sleep 100" -c "stm32f1x mass_erase 0" -c "sleep 100" -c shutdown 

erase_stlink:
	st-flash erase

debug_openocd: $(PROJECT).elf flash_openocd
	xterm -e openocd -s ~/EmbeddedArm/openocd-bin/share/openocd/scripts/ -f interface/stlink-v2.cfg -f target/stm32f1x_stlink.cfg -c "init" -c "halt" -c "reset halt" &
	$(GDBTUI) --eval-command="target remote localhost:3333" $(PROJECT).elf 

debug_stlink: $(PROJECT).elf
	xterm -e st-util &
	$(GDBTUI) --eval-command="target remote localhost:4242"  $(PROJECT).elf -ex 'load'
		
clean:
	-rm -rf $(OBJS)
	-rm -rf $(PROJECT).elf
	-rm -rf $(PROJECT).map
	-rm -rf $(PROJECT).hex
	-rm -rf $(PROJECT).bin
	-rm -rf $(SRC:.c=.lst)
	-rm -rf $(SRC:.c=.d)
	-rm -rf $(ASRC:.S=.lst)
# *** EOF ***
