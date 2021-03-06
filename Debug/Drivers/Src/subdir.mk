################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/Src/stm32f407xx_adc_driver.cpp \
../Drivers/Src/stm32f407xx_gpio_driver.cpp \
../Drivers/Src/stm32f407xx_i2c_driver.cpp \
../Drivers/Src/stm32f407xx_usart_driver.cpp 

OBJS += \
./Drivers/Src/stm32f407xx_adc_driver.o \
./Drivers/Src/stm32f407xx_gpio_driver.o \
./Drivers/Src/stm32f407xx_i2c_driver.o \
./Drivers/Src/stm32f407xx_usart_driver.o 

CPP_DEPS += \
./Drivers/Src/stm32f407xx_adc_driver.d \
./Drivers/Src/stm32f407xx_gpio_driver.d \
./Drivers/Src/stm32f407xx_i2c_driver.d \
./Drivers/Src/stm32f407xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o: ../Drivers/Src/%.cpp Drivers/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"/home/khorton/Documents/DriversARM/stm32f407xxDrivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32f407xx_adc_driver.d ./Drivers/Src/stm32f407xx_adc_driver.o ./Drivers/Src/stm32f407xx_gpio_driver.d ./Drivers/Src/stm32f407xx_gpio_driver.o ./Drivers/Src/stm32f407xx_i2c_driver.d ./Drivers/Src/stm32f407xx_i2c_driver.o ./Drivers/Src/stm32f407xx_usart_driver.d ./Drivers/Src/stm32f407xx_usart_driver.o

.PHONY: clean-Drivers-2f-Src

