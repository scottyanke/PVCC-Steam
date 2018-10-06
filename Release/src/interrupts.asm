;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.7.0 #10231 (Linux)
;--------------------------------------------------------
	.module interrupts
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _TLI_IRQHandler
	.globl _AWU_IRQHandler
	.globl _CLK_IRQHandler
	.globl _EXTI_PORTA_IRQHandler
	.globl _EXTI_PORTB_IRQHandler
	.globl _EXTI_PORTC_IRQHandler
	.globl _EXTI_PORTD_IRQHandler
	.globl _EXTI_PORTE_IRQHandler
	.globl _SPI_IRQHandler
	.globl _TIM1_UPD_OVF_TRG_BRK_IRQHandler
	.globl _TIM1_CAP_COM_IRQHandler
	.globl _TIM2_UPD_OVF_BRK_IRQHandler
	.globl _TIM2_CAP_COM_IRQHandler
	.globl _UART1_TX_IRQHandler
	.globl _UART1_RX_IRQHandler
	.globl _I2C_IRQHandler
	.globl _ADC1_IRQHandler
	.globl _TIM4_UPD_OVF_IRQHandler
	.globl _EEPROM_EEC_IRQHandler
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	../src/interrupts.c: 26: INTERRUPT_HANDLER(TLI_IRQHandler, 0){}
;	-----------------------------------------
;	 function TLI_IRQHandler
;	-----------------------------------------
_TLI_IRQHandler:
	iret
;	../src/interrupts.c: 29: INTERRUPT_HANDLER(AWU_IRQHandler, 1){}
;	-----------------------------------------
;	 function AWU_IRQHandler
;	-----------------------------------------
_AWU_IRQHandler:
	iret
;	../src/interrupts.c: 32: INTERRUPT_HANDLER(CLK_IRQHandler, 2){}
;	-----------------------------------------
;	 function CLK_IRQHandler
;	-----------------------------------------
_CLK_IRQHandler:
	iret
;	../src/interrupts.c: 35: INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3){}
;	-----------------------------------------
;	 function EXTI_PORTA_IRQHandler
;	-----------------------------------------
_EXTI_PORTA_IRQHandler:
	iret
;	../src/interrupts.c: 38: INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4){}
;	-----------------------------------------
;	 function EXTI_PORTB_IRQHandler
;	-----------------------------------------
_EXTI_PORTB_IRQHandler:
	iret
;	../src/interrupts.c: 41: INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5){
;	-----------------------------------------
;	 function EXTI_PORTC_IRQHandler
;	-----------------------------------------
_EXTI_PORTC_IRQHandler:
;	../src/interrupts.c: 42: }
	iret
;	../src/interrupts.c: 45: INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6){
;	-----------------------------------------
;	 function EXTI_PORTD_IRQHandler
;	-----------------------------------------
_EXTI_PORTD_IRQHandler:
;	../src/interrupts.c: 46: }
	iret
;	../src/interrupts.c: 49: INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 7){}
;	-----------------------------------------
;	 function EXTI_PORTE_IRQHandler
;	-----------------------------------------
_EXTI_PORTE_IRQHandler:
	iret
;	../src/interrupts.c: 65: INTERRUPT_HANDLER(SPI_IRQHandler, 10){}
;	-----------------------------------------
;	 function SPI_IRQHandler
;	-----------------------------------------
_SPI_IRQHandler:
	iret
;	../src/interrupts.c: 68: INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11){
;	-----------------------------------------
;	 function TIM1_UPD_OVF_TRG_BRK_IRQHandler
;	-----------------------------------------
_TIM1_UPD_OVF_TRG_BRK_IRQHandler:
;	../src/interrupts.c: 69: }
	iret
;	../src/interrupts.c: 72: INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, 12){}
;	-----------------------------------------
;	 function TIM1_CAP_COM_IRQHandler
;	-----------------------------------------
_TIM1_CAP_COM_IRQHandler:
	iret
;	../src/interrupts.c: 84: INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13){
;	-----------------------------------------
;	 function TIM2_UPD_OVF_BRK_IRQHandler
;	-----------------------------------------
_TIM2_UPD_OVF_BRK_IRQHandler:
;	../src/interrupts.c: 85: }
	iret
;	../src/interrupts.c: 89: INTERRUPT_HANDLER(TIM2_CAP_COM_IRQHandler, 14){
;	-----------------------------------------
;	 function TIM2_CAP_COM_IRQHandler
;	-----------------------------------------
_TIM2_CAP_COM_IRQHandler:
;	../src/interrupts.c: 90: }
	iret
;	../src/interrupts.c: 105: INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17){}
;	-----------------------------------------
;	 function UART1_TX_IRQHandler
;	-----------------------------------------
_UART1_TX_IRQHandler:
	iret
;	../src/interrupts.c: 108: INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18){
;	-----------------------------------------
;	 function UART1_RX_IRQHandler
;	-----------------------------------------
_UART1_RX_IRQHandler:
	sub	sp, #2
;	../src/interrupts.c: 110: if(UART1->SR & UART1_SR_RXNE){ // data received
	ld	a, 0x5230
	bcp	a, #0x20
	jreq	00115$
;	../src/interrupts.c: 111: rb = UART1->DR; // read received byte & clear RXNE flag
	ld	a, 0x5231
;	../src/interrupts.c: 113: UART_rx[UART_rx_cur_i++] = rb; // put received byte into cycled buffer
	ldw	x, #_UART_rx+0
	ldw	(0x01, sp), x
	exg	a, xl
	ld	a, _UART_rx_cur_i+0
	exg	a, xl
	inc	_UART_rx_cur_i+0
	rlwa	x
	clr	a
	rrwa	x
	addw	x, (0x01, sp)
	ld	(x), a
;	../src/interrupts.c: 114: if(UART_rx_cur_i == UART_rx_start_i){ // Oops: buffer overflow! Just forget old data
	ld	a, _UART_rx_start_i+0
	cp	a, _UART_rx_cur_i+0
	jrne	00110$
;	../src/interrupts.c: 115: UART_rx_start_i++;
	inc	_UART_rx_start_i+0
;	../src/interrupts.c: 116: check_UART_pointer(UART_rx_start_i);
	ld	a, _UART_rx_start_i+0
	cp	a, #0x1e
	jrne	00110$
	clr	_UART_rx_start_i+0
;	../src/interrupts.c: 118: check_UART_pointer(UART_rx_cur_i);
00110$:
	ld	a, _UART_rx_cur_i+0
	cp	a, #0x1e
	jrne	00115$
	clr	_UART_rx_cur_i+0
00115$:
;	../src/interrupts.c: 120: }
	addw	sp, #2
	iret
;	../src/interrupts.c: 124: INTERRUPT_HANDLER(I2C_IRQHandler, 19){}
;	-----------------------------------------
;	 function I2C_IRQHandler
;	-----------------------------------------
_I2C_IRQHandler:
	iret
;	../src/interrupts.c: 148: INTERRUPT_HANDLER(ADC1_IRQHandler, 22){
;	-----------------------------------------
;	 function ADC1_IRQHandler
;	-----------------------------------------
_ADC1_IRQHandler:
;	../src/interrupts.c: 150: }
	iret
;	../src/interrupts.c: 158: void TIM4_UPD_OVF_IRQHandler() __interrupt(23){
;	-----------------------------------------
;	 function TIM4_UPD_OVF_IRQHandler
;	-----------------------------------------
_TIM4_UPD_OVF_IRQHandler:
;	../src/interrupts.c: 159: if(TIM4->SR1 & TIM4_SR1_UIF){ // update interrupt
	ld	a, 0x5344
	srl	a
	jrnc	00102$
;	../src/interrupts.c: 160: Global_time++; // increase timer
	ldw	x, _Global_time+2
	addw	x, #0x0001
	ld	a, _Global_time+1
	adc	a, #0x00
	ld	yl, a
	ld	a, _Global_time+0
	adc	a, #0x00
	ld	yh, a
	ldw	_Global_time+2, x
	ldw	_Global_time+0, y
00102$:
;	../src/interrupts.c: 162: TIM4->SR1 = 0; // clear all interrupt flags
	mov	0x5344+0, #0x00
;	../src/interrupts.c: 163: }
	iret
;	../src/interrupts.c: 167: INTERRUPT_HANDLER(EEPROM_EEC_IRQHandler, 24){}
;	-----------------------------------------
;	 function EEPROM_EEC_IRQHandler
;	-----------------------------------------
_EEPROM_EEC_IRQHandler:
	iret
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
