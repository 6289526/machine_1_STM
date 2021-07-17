################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f205vetx.s 

OBJS += \
./Core/Startup/startup_stm32f205vetx.o 

S_DEPS += \
./Core/Startup/startup_stm32f205vetx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32f205vetx.o: ../Core/Startup/startup_stm32f205vetx.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -c -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32f205vetx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

