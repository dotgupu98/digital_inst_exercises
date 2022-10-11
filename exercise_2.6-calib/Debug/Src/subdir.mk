################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/30010_io.c \
../Src/flash.c \
../Src/lcd.c \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32f30x.c 

OBJS += \
./Src/30010_io.o \
./Src/flash.o \
./Src/lcd.o \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32f30x.o 

C_DEPS += \
./Src/30010_io.d \
./Src/flash.d \
./Src/lcd.d \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32f30x.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F302R8Tx -DSTM32F3 -DNUCLEO_F302R8 -DDEBUG -DUSE_STDPERIPH_DRIVER=1 -c -I"/home/tibi/Documents/DTU/Digital_Instrumentation/exercise_2.6-calib/SPL/inc" -I"/home/tibi/Documents/DTU/Digital_Instrumentation/exercise_2.6-calib/Inc" -I"/home/tibi/Documents/DTU/Digital_Instrumentation/exercise_2.6-calib/cmsis" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/30010_io.d ./Src/30010_io.o ./Src/30010_io.su ./Src/flash.d ./Src/flash.o ./Src/flash.su ./Src/lcd.d ./Src/lcd.o ./Src/lcd.su ./Src/main.d ./Src/main.o ./Src/main.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32f30x.d ./Src/system_stm32f30x.o ./Src/system_stm32f30x.su

.PHONY: clean-Src

