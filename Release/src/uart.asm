;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.7.0 #10231 (Linux)
;--------------------------------------------------------
	.module uart
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _UART_rx_cur_i
	.globl _UART_rx_start_i
	.globl _UART_rx
	.globl _uart_init
	.globl _uart_write
	.globl _uart_read
	.globl _UART_read_byte
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
_UART_rx::
	.ds 30
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
_UART_rx_start_i::
	.ds 1
_UART_rx_cur_i::
	.ds 1
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
;	../src/uart.c: 8: void uart_init() {
;	-----------------------------------------
;	 function uart_init
;	-----------------------------------------
_uart_init:
;	../src/uart.c: 10: tmp = UART1->SR;
	ldw	x, #0x5230
	ld	a, (x)
;	../src/uart.c: 11: tmp = UART1->DR;
	ldw	x, #0x5231
	ld	a, (x)
;	../src/uart.c: 12: UART1->CR1 = 0x00;
	mov	0x5234+0, #0x00
;	../src/uart.c: 14: UART1->BRR2 = 0x03; //((div >> 8) & 0xF0) + (div & 0x0F);
	mov	0x5233+0, #0x03
;	../src/uart.c: 15: UART1->BRR1 = 0x6a; //div >> 4;
	mov	0x5232+0, #0x6a
;	../src/uart.c: 16: UART1->CR2 = UART1_CR2_TEN | UART1_CR2_REN | UART1_CR2_RIEN; // Allow RX/TX, generate ints on rx
	mov	0x5235+0, #0x2c
;	../src/uart.c: 17: }
	ret
;	../src/uart.c: 19: void uart_write(uint8_t data) {
;	-----------------------------------------
;	 function uart_write
;	-----------------------------------------
_uart_write:
;	../src/uart.c: 20: UART1->DR = data;
	ldw	x, #0x5231
	ld	a, (0x03, sp)
	ld	(x), a
;	../src/uart.c: 21: while (!(UART1->SR & UART1_SR_TC)) ;
00101$:
	ld	a, 0x5230
	bcp	a, #0x40
	jreq	00101$
;	../src/uart.c: 22: UART1->SR &= ~(UART1_SR_TC);
	and	a, #0xbf
	ld	0x5230, a
;	../src/uart.c: 23: }
	ret
;	../src/uart.c: 25: uint8_t uart_read() {
;	-----------------------------------------
;	 function uart_read
;	-----------------------------------------
_uart_read:
;	../src/uart.c: 28: while (!(UART1->SR & UART1_SR_RXNE)) ;
00101$:
	ld	a, 0x5230
	bcp	a, #0x20
	jreq	00101$
;	../src/uart.c: 29: return UART1->DR;
	ld	a, 0x5231
;	../src/uart.c: 30: }
	ret
;	../src/uart.c: 37: uint8_t UART_read_byte(uint8_t *byte){
;	-----------------------------------------
;	 function UART_read_byte
;	-----------------------------------------
_UART_read_byte:
	sub	sp, #2
;	../src/uart.c: 38: if(UART_rx_start_i == UART_rx_cur_i) // buffer is empty
	ld	a, _UART_rx_cur_i+0
	cp	a, _UART_rx_start_i+0
	jrne	00102$
;	../src/uart.c: 39: return 0;
	clr	a
	jra	00108$
00102$:
;	../src/uart.c: 40: *byte = UART_rx[UART_rx_start_i++];
	ldw	y, (0x05, sp)
	ldw	x, #_UART_rx+0
	ldw	(0x01, sp), x
	ld	a, _UART_rx_start_i+0
	inc	_UART_rx_start_i+0
	clrw	x
	ld	xl, a
	addw	x, (0x01, sp)
	ld	a, (x)
	ld	(y), a
;	../src/uart.c: 41: check_UART_pointer(UART_rx_start_i);
	ld	a, _UART_rx_start_i+0
	cp	a, #0x1e
	jrne	00106$
	clr	_UART_rx_start_i+0
00106$:
;	../src/uart.c: 42: return 1;
	ld	a, #0x01
00108$:
;	../src/uart.c: 43: }
	addw	sp, #2
	ret
	.area CODE
	.area INITIALIZER
__xinit__UART_rx_start_i:
	.db #0x00	; 0
__xinit__UART_rx_cur_i:
	.db #0x00	; 0
	.area CABS (ABS)
