################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
ASM_SRCS += \
/home/scott/Samples/stm8-bare-min/stm8/delay.asm \
/home/scott/Samples/stm8-bare-min/stm8/eeprom.asm \
/home/scott/Samples/stm8-bare-min/stm8/i2c.asm \
/home/scott/Samples/stm8-bare-min/stm8/spi.asm \
/home/scott/Samples/stm8-bare-min/stm8/uart.asm 

C_SRCS += \
/home/scott/Samples/stm8-bare-min/stm8/delay.c \
/home/scott/Samples/stm8-bare-min/stm8/eeprom.c \
/home/scott/Samples/stm8-bare-min/stm8/i2c.c \
/home/scott/Samples/stm8-bare-min/stm8/spi.c \
/home/scott/Samples/stm8-bare-min/stm8/uart.c 

RELS += \
./stm-8/delay.rel \
./stm-8/eeprom.rel \
./stm-8/i2c.rel \
./stm-8/spi.rel \
./stm-8/uart.rel 

C_DEPS += \
./stm-8/delay.d \
./stm-8/eeprom.d \
./stm-8/i2c.d \
./stm-8/spi.d \
./stm-8/uart.d 


# Each subdirectory must supply rules for building sources it contributes
stm-8/delay.rel: /home/scott/Samples/stm8-bare-min/stm8/delay.asm
	@echo 'Building file: $<'
	@echo 'Invoking: SDCC Assembler'
	sdasstm8 -plosgff -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

stm-8/delay.rel: /home/scott/Samples/stm8-bare-min/stm8/delay.c
	@echo 'Building file: $<'
	@echo 'Invoking: SDCC Compiler'
	sdcc -c -mstm8 -pSTM8S103 --std-sdcc11 --disable-warning 190 --disable-warning 88 --disable-warning 133 --stack-auto --noinduction --use-non-free -DF_CPU=2000000 -D_SDCC_ -DSTM8S103 -DUSE_FLOATS=1 -I/usr/local/share/sdcc/include -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 --model-small -o  "$@" "$<" && \
	echo -n $(@:%.rel=%.d) $(dir $@) > $(@:%.rel=%.d) && \
	sdcc -c -mstm8 -pSTM8S103 --std-sdcc11 --disable-warning 190 --disable-warning 88 --disable-warning 133 --stack-auto --noinduction --use-non-free -MM -DF_CPU=2000000 -D_SDCC_ -DSTM8S103 -DUSE_FLOATS=1 -I/usr/local/share/sdcc/include -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 --model-small    "$<" >> $(@:%.rel=%.d)
	@echo 'Finished building: $<'
	@echo ' '

stm-8/eeprom.rel: /home/scott/Samples/stm8-bare-min/stm8/eeprom.asm
	@echo 'Building file: $<'
	@echo 'Invoking: SDCC Assembler'
	sdasstm8 -plosgff -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

stm-8/eeprom.rel: /home/scott/Samples/stm8-bare-min/stm8/eeprom.c
	@echo 'Building file: $<'
	@echo 'Invoking: SDCC Compiler'
	sdcc -c -mstm8 -pSTM8S103 --std-sdcc11 --disable-warning 190 --disable-warning 88 --disable-warning 133 --stack-auto --noinduction --use-non-free -DF_CPU=2000000 -D_SDCC_ -DSTM8S103 -DUSE_FLOATS=1 -I/usr/local/share/sdcc/include -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 --model-small -o  "$@" "$<" && \
	echo -n $(@:%.rel=%.d) $(dir $@) > $(@:%.rel=%.d) && \
	sdcc -c -mstm8 -pSTM8S103 --std-sdcc11 --disable-warning 190 --disable-warning 88 --disable-warning 133 --stack-auto --noinduction --use-non-free -MM -DF_CPU=2000000 -D_SDCC_ -DSTM8S103 -DUSE_FLOATS=1 -I/usr/local/share/sdcc/include -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 --model-small    "$<" >> $(@:%.rel=%.d)
	@echo 'Finished building: $<'
	@echo ' '

stm-8/i2c.rel: /home/scott/Samples/stm8-bare-min/stm8/i2c.asm
	@echo 'Building file: $<'
	@echo 'Invoking: SDCC Assembler'
	sdasstm8 -plosgff -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

stm-8/i2c.rel: /home/scott/Samples/stm8-bare-min/stm8/i2c.c
	@echo 'Building file: $<'
	@echo 'Invoking: SDCC Compiler'
	sdcc -c -mstm8 -pSTM8S103 --std-sdcc11 --disable-warning 190 --disable-warning 88 --disable-warning 133 --stack-auto --noinduction --use-non-free -DF_CPU=2000000 -D_SDCC_ -DSTM8S103 -DUSE_FLOATS=1 -I/usr/local/share/sdcc/include -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 --model-small -o  "$@" "$<" && \
	echo -n $(@:%.rel=%.d) $(dir $@) > $(@:%.rel=%.d) && \
	sdcc -c -mstm8 -pSTM8S103 --std-sdcc11 --disable-warning 190 --disable-warning 88 --disable-warning 133 --stack-auto --noinduction --use-non-free -MM -DF_CPU=2000000 -D_SDCC_ -DSTM8S103 -DUSE_FLOATS=1 -I/usr/local/share/sdcc/include -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 --model-small    "$<" >> $(@:%.rel=%.d)
	@echo 'Finished building: $<'
	@echo ' '

stm-8/spi.rel: /home/scott/Samples/stm8-bare-min/stm8/spi.asm
	@echo 'Building file: $<'
	@echo 'Invoking: SDCC Assembler'
	sdasstm8 -plosgff -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

stm-8/spi.rel: /home/scott/Samples/stm8-bare-min/stm8/spi.c
	@echo 'Building file: $<'
	@echo 'Invoking: SDCC Compiler'
	sdcc -c -mstm8 -pSTM8S103 --std-sdcc11 --disable-warning 190 --disable-warning 88 --disable-warning 133 --stack-auto --noinduction --use-non-free -DF_CPU=2000000 -D_SDCC_ -DSTM8S103 -DUSE_FLOATS=1 -I/usr/local/share/sdcc/include -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 --model-small -o  "$@" "$<" && \
	echo -n $(@:%.rel=%.d) $(dir $@) > $(@:%.rel=%.d) && \
	sdcc -c -mstm8 -pSTM8S103 --std-sdcc11 --disable-warning 190 --disable-warning 88 --disable-warning 133 --stack-auto --noinduction --use-non-free -MM -DF_CPU=2000000 -D_SDCC_ -DSTM8S103 -DUSE_FLOATS=1 -I/usr/local/share/sdcc/include -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 --model-small    "$<" >> $(@:%.rel=%.d)
	@echo 'Finished building: $<'
	@echo ' '

stm-8/uart.rel: /home/scott/Samples/stm8-bare-min/stm8/uart.asm
	@echo 'Building file: $<'
	@echo 'Invoking: SDCC Assembler'
	sdasstm8 -plosgff -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

stm-8/uart.rel: /home/scott/Samples/stm8-bare-min/stm8/uart.c
	@echo 'Building file: $<'
	@echo 'Invoking: SDCC Compiler'
	sdcc -c -mstm8 -pSTM8S103 --std-sdcc11 --disable-warning 190 --disable-warning 88 --disable-warning 133 --stack-auto --noinduction --use-non-free -DF_CPU=2000000 -D_SDCC_ -DSTM8S103 -DUSE_FLOATS=1 -I/usr/local/share/sdcc/include -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 --model-small -o  "$@" "$<" && \
	echo -n $(@:%.rel=%.d) $(dir $@) > $(@:%.rel=%.d) && \
	sdcc -c -mstm8 -pSTM8S103 --std-sdcc11 --disable-warning 190 --disable-warning 88 --disable-warning 133 --stack-auto --noinduction --use-non-free -MM -DF_CPU=2000000 -D_SDCC_ -DSTM8S103 -DUSE_FLOATS=1 -I/usr/local/share/sdcc/include -I"/home/scott/projects-stm8/lvcc-tunnels/inc" -I/home/scott/Samples/stm8-bare-min/stm8 --model-small    "$<" >> $(@:%.rel=%.d)
	@echo 'Finished building: $<'
	@echo ' '


