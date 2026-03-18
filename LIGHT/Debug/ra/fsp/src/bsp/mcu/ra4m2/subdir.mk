################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ra/fsp/src/bsp/mcu/ra4m2/bsp_linker.c 

C_DEPS += \
./ra/fsp/src/bsp/mcu/ra4m2/bsp_linker.d 

OBJS += \
./ra/fsp/src/bsp/mcu/ra4m2/bsp_linker.o 

SREC += \
LIGHT.srec 

MAP += \
LIGHT.map 


# Each subdirectory must supply rules for building sources it contributes
ra/fsp/src/bsp/mcu/ra4m2/%.o: ../ra/fsp/src/bsp/mcu/ra4m2/%.c
	$(file > $@.in,-mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -O2 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-strict-aliasing -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal -g -D_RENESAS_RA_ -D_RA_CORE=CM33 -D_RA_ORDINAL=1 -I"D:/e2_studio/workspace/LIGHT/ra_gen" -I"." -I"D:/e2_studio/workspace/LIGHT/ra_cfg/fsp_cfg/bsp" -I"D:/e2_studio/workspace/LIGHT/ra_cfg/fsp_cfg" -I"D:/e2_studio/workspace/LIGHT/src" -I"D:/e2_studio/workspace/LIGHT/ra/fsp/inc" -I"D:/e2_studio/workspace/LIGHT/ra/fsp/inc/api" -I"D:/e2_studio/workspace/LIGHT/ra/fsp/inc/instances" -I"D:/e2_studio/workspace/LIGHT/ra/arm/CMSIS_6/CMSIS/Core/Include" -std=c99 -Wno-stringop-overflow -Wno-format-truncation --param=min-pagesize=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" -x c "$<")
	@echo Building file: $< && arm-none-eabi-gcc @"$@.in"

