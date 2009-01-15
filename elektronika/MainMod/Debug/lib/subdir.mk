################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/MainModLib.c \
../lib/lcd.c 

OBJS += \
./lib/MainModLib.o \
./lib/lcd.o 

C_DEPS += \
./lib/MainModLib.d \
./lib/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
lib/%.o: ../lib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -O2 -fshort-enums -funsigned-char -funsigned-bitfields -v -mmcu=atmega64 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


