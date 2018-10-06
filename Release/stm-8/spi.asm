;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.6.0 #9615 (Linux)
;--------------------------------------------------------
	.module spi
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _SPI_init
	.globl _SPI_read
	.globl _SPI_write
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
;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 4: void SPI_init() {
;	-----------------------------------------
;	 function SPI_init
;	-----------------------------------------
_SPI_init:
;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 5: SPI_CR1 = (1 << SPI_CR1_MSTR) | (1 << SPI_CR1_SPE) | (1 << SPI_CR1_BR1);
	mov	0x5200+0, #0x54
;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 6: SPI_CR2 = (1 << SPI_CR2_SSM) | (1 << SPI_CR2_SSI) | (1 << SPI_CR2_BDM) | (1 << SPI_CR2_BDOE);
	mov	0x5201+0, #0xc3
	ret
;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 9: uint8_t SPI_read() {
;	-----------------------------------------
;	 function SPI_read
;	-----------------------------------------
_SPI_read:
;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 10: SPI_write(0xFF);
	push	#0xff
	call	_SPI_write
	pop	a
;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 11: while (!(SPI_SR & (1 << SPI_SR_RXNE)));
00101$:
	ldw	x, #0x5203
	ld	a, (x)
	srl	a
	jrnc	00101$
;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 12: return SPI_DR;
	ldw	x, #0x5204
	ld	a, (x)
	ret
;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 15: void SPI_write(uint8_t data) {
;	-----------------------------------------
;	 function SPI_write
;	-----------------------------------------
_SPI_write:
;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 16: SPI_DR = data;
	ldw	x, #0x5204
	ld	a, (0x03, sp)
	ld	(x), a
;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 17: while (!(SPI_SR & (1 << SPI_SR_TXE)));
00101$:
	ldw	x, #0x5203
	ld	a, (x)
	bcp	a, #0x02
	jreq	00101$
	ret
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
