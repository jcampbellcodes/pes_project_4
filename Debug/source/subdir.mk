################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/delay.c \
../source/handle_led.c \
../source/i2c.c \
../source/logger.c \
../source/main.c \
../source/mtb.c \
../source/post.c \
../source/semihost_hardfault.c \
../source/setup_teardown.c \
../source/state_machine.c \
../source/tmp102.c 

OBJS += \
./source/delay.o \
./source/handle_led.o \
./source/i2c.o \
./source/logger.o \
./source/main.o \
./source/mtb.o \
./source/post.o \
./source/semihost_hardfault.o \
./source/setup_teardown.o \
./source/state_machine.o \
./source/tmp102.o 

C_DEPS += \
./source/delay.d \
./source/handle_led.d \
./source/i2c.d \
./source/logger.d \
./source/main.d \
./source/mtb.d \
./source/post.d \
./source/semihost_hardfault.d \
./source/setup_teardown.d \
./source/state_machine.d \
./source/tmp102.d 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DPRINTF_FLOAT_ENABLE=1 -DCPU_MKL25Z128VLK4_cm0plus -DCPU_MKL25Z128VLK4 -DFSL_RTOS_BM -DSDK_OS_BAREMETAL -DSDK_DEBUGCONSOLE_UART -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -DSDK_DEBUGCONSOLE=1 -I../board -I../include -I../source -I../ -I../drivers -I../CMSIS -I../utilities -I../CMSIS_driver -I../startup -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m0plus -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


