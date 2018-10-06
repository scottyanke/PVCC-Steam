################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/delay.c \
../src/interrupts.c \
../src/main.c \
../src/uart.c 

RELS += \
./src/delay.rel \
./src/interrupts.rel \
./src/main.rel \
./src/uart.rel 

C_DEPS += \
./src/delay.d \
./src/interrupts.d \
./src/main.d \
./src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.rel: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: SDCC Compiler'
	sdcc -c -mstm8 -pSTM8S103 --std-sdcc11 --disable-warning 190 --disable-warning 88 --disable-warning 133 --stack-auto --noinduction --use-non-free -DF_CPU=16000000 -DSDCC -D_SDCC_ -DSTM8S103 -I/usr/local/share/sdcc/include -I"/home/scott/projects-stm8/pvcc-steam/inc" --verbose --model-small -o  "$@" "$<" && \
	echo -n $(@:%.rel=%.d) $(dir $@) > $(@:%.rel=%.d) && \
	sdcc -c -mstm8 -pSTM8S103 --std-sdcc11 --disable-warning 190 --disable-warning 88 --disable-warning 133 --stack-auto --noinduction --use-non-free -MM -DF_CPU=16000000 -DSDCC -D_SDCC_ -DSTM8S103 -I/usr/local/share/sdcc/include -I"/home/scott/projects-stm8/pvcc-steam/inc" --verbose --model-small    "$<" >> $(@:%.rel=%.d)
	@echo 'Finished building: $<'
	@echo ' '


