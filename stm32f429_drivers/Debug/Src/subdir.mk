################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/001-1LED_Toggle.c \
../Src/001-2LED_Toggle.c \
../Src/001-3LED_Toggle.c \
../Src/002External_Button.c \
../Src/003Button_Interrupt.c \
../Src/005SPI_Send.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/001-1LED_Toggle.o \
./Src/001-2LED_Toggle.o \
./Src/001-3LED_Toggle.o \
./Src/002External_Button.o \
./Src/003Button_Interrupt.o \
./Src/005SPI_Send.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/001-1LED_Toggle.d \
./Src/001-2LED_Toggle.d \
./Src/001-3LED_Toggle.d \
./Src/002External_Button.d \
./Src/003Button_Interrupt.d \
./Src/005SPI_Send.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -I"/home/viruple/Projects/STM32F429_CortexM4/MCU1 - Embedded Driver Development/stm32f429_drivers/drivers" -I"/home/viruple/Projects/STM32F429_CortexM4/MCU1 - Embedded Driver Development/stm32f429_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/001-1LED_Toggle.cyclo ./Src/001-1LED_Toggle.d ./Src/001-1LED_Toggle.o ./Src/001-1LED_Toggle.su ./Src/001-2LED_Toggle.cyclo ./Src/001-2LED_Toggle.d ./Src/001-2LED_Toggle.o ./Src/001-2LED_Toggle.su ./Src/001-3LED_Toggle.cyclo ./Src/001-3LED_Toggle.d ./Src/001-3LED_Toggle.o ./Src/001-3LED_Toggle.su ./Src/002External_Button.cyclo ./Src/002External_Button.d ./Src/002External_Button.o ./Src/002External_Button.su ./Src/003Button_Interrupt.cyclo ./Src/003Button_Interrupt.d ./Src/003Button_Interrupt.o ./Src/003Button_Interrupt.su ./Src/005SPI_Send.cyclo ./Src/005SPI_Send.d ./Src/005SPI_Send.o ./Src/005SPI_Send.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

