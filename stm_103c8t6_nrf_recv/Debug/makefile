################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
stm_103c8t6_nrf_recv.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
stm_103c8t6_nrf_recv.list \

OBJCOPY_BIN += \
stm_103c8t6_nrf_recv.bin \


# All Target
all: stm_103c8t6_nrf_recv.elf secondary-outputs

# Tool invocations
stm_103c8t6_nrf_recv.elf: $(OBJS) $(USER_OBJS) E:\programming\STM32\stm_103c8t6_nrf_recv\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "stm_103c8t6_nrf_recv.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"E:\programming\STM32\stm_103c8t6_nrf_recv\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="stm_103c8t6_nrf_recv.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

stm_103c8t6_nrf_recv.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "stm_103c8t6_nrf_recv.list"
	@echo 'Finished building: $@'
	@echo ' '

stm_103c8t6_nrf_recv.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "stm_103c8t6_nrf_recv.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
