################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.c \
../Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.c \
../Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.c \
../Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.c 

OBJS += \
./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.o \
./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.o \
./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.o \
./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.o 

C_DEPS += \
./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.d \
./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.d \
./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.d \
./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/NN/Source/ActivationFunctions/%.o Drivers/CMSIS/NN/Source/ActivationFunctions/%.su: ../Drivers/CMSIS/NN/Source/ActivationFunctions/%.c Drivers/CMSIS/NN/Source/ActivationFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-NN-2f-Source-2f-ActivationFunctions

clean-Drivers-2f-CMSIS-2f-NN-2f-Source-2f-ActivationFunctions:
	-$(RM) ./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.d ./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.o ./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q15.su ./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.d ./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.o ./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_nn_activations_q7.su ./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.d ./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.o ./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q15.su ./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.d ./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.o ./Drivers/CMSIS/NN/Source/ActivationFunctions/arm_relu_q7.su

.PHONY: clean-Drivers-2f-CMSIS-2f-NN-2f-Source-2f-ActivationFunctions

