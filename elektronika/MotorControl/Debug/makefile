################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
MotorControl.lss \

FLASH_IMAGE += \
MotorControl.hex \

EEPROM_IMAGE += \
MotorControl.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: MotorControl.elf secondary-outputs

# Tool invocations
MotorControl.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,MotorControl.map -mmcu=atmega8 -o"MotorControl.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

MotorControl.lss: MotorControl.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S MotorControl.elf  >"MotorControl.lss"
	@echo 'Finished building: $@'
	@echo ' '

MotorControl.hex: MotorControl.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex MotorControl.elf  "MotorControl.hex"
	@echo 'Finished building: $@'
	@echo ' '

MotorControl.eep: MotorControl.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex MotorControl.elf  "MotorControl.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: MotorControl.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=berkeley -t MotorControl.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) MotorControl.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
