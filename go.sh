echo "=====1======="
arm-none-eabi-gcc \
-c \
-mcpu=cortex-m3 \
-Os \
-g \
-ggdb \
-mthumb   \
-DSTM32F103xB \
-DGCC_ARMCM3 \
-I"/home/zserg/not_work/stm32_projects/workspace/Template/Inc" \
-I/home/zserg/not_work/stm32_projects/STM32Cube_FW_F1_V1.0.0/Drivers/STM32F1xx_HAL_Driver/Inc \
-I/home/zserg/not_work/stm32_projects/STM32Cube_FW_F1_V1.0.0/Drivers/CMSIS/Include \
-I/home/zserg/not_work/stm32_projects/STM32Cube_FW_F1_V1.0.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include \
-Wall \
-fmessage-length=0 \
-MMD \
-MP \
-MF"Src/main.d" \
-MT"Src/main.d" \
-o "Src/main.o" \
"Src/main.c"
echo "=====2======="

arm-none-eabi-gcc \
-c \
-mcpu=cortex-m3 \
-Os \
-g \
-ggdb \
-mthumb   \
-fomit-frame-pointer \
-Wall \
-Wstrict-prototypes \
-fverbose-asm \
-Wa,\
-ahlms=Src/main.lst \
-DSTM32F103xB \
-DUSE_STDPERIPH_DRIVER  \
-I . \
-I/home/zserg/not_work/stm32_projects/STM32Cube_FW_F1_V1.0.0/Drivers/CMSIS/Device/ST/STM32F1xx/Include \
-I/home/zserg/not_work/stm32_projects/STM32Cube_FW_F1_V1.0.0/Drivers/CMSIS/Include \
-I/home/zserg/not_work/stm32_projects/STM32Cube_FW_F1_V1.0.0/Drivers/STM32F1xx_HAL_Driver/Inc \
-I./Inc \
Src/main.c \
-o Src/main.o
echo "=====3======="

