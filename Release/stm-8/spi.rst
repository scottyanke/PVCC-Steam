                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.6.0 #9615 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module spi
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _SPI_init
                                     12 	.globl _SPI_read
                                     13 	.globl _SPI_write
                                     14 ;--------------------------------------------------------
                                     15 ; ram data
                                     16 ;--------------------------------------------------------
                                     17 	.area DATA
                                     18 ;--------------------------------------------------------
                                     19 ; ram data
                                     20 ;--------------------------------------------------------
                                     21 	.area INITIALIZED
                                     22 ;--------------------------------------------------------
                                     23 ; absolute external ram data
                                     24 ;--------------------------------------------------------
                                     25 	.area DABS (ABS)
                                     26 ;--------------------------------------------------------
                                     27 ; global & static initialisations
                                     28 ;--------------------------------------------------------
                                     29 	.area HOME
                                     30 	.area GSINIT
                                     31 	.area GSFINAL
                                     32 	.area GSINIT
                                     33 ;--------------------------------------------------------
                                     34 ; Home
                                     35 ;--------------------------------------------------------
                                     36 	.area HOME
                                     37 	.area HOME
                                     38 ;--------------------------------------------------------
                                     39 ; code
                                     40 ;--------------------------------------------------------
                                     41 	.area CODE
                                     42 ;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 4: void SPI_init() {
                                     43 ;	-----------------------------------------
                                     44 ;	 function SPI_init
                                     45 ;	-----------------------------------------
      0081A0                         46 _SPI_init:
                                     47 ;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 5: SPI_CR1 = (1 << SPI_CR1_MSTR) | (1 << SPI_CR1_SPE) | (1 << SPI_CR1_BR1);
      0081A0 35 54 52 00      [ 1]   48 	mov	0x5200+0, #0x54
                                     49 ;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 6: SPI_CR2 = (1 << SPI_CR2_SSM) | (1 << SPI_CR2_SSI) | (1 << SPI_CR2_BDM) | (1 << SPI_CR2_BDOE);
      0081A4 35 C3 52 01      [ 1]   50 	mov	0x5201+0, #0xc3
      0081A8 81               [ 4]   51 	ret
                                     52 ;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 9: uint8_t SPI_read() {
                                     53 ;	-----------------------------------------
                                     54 ;	 function SPI_read
                                     55 ;	-----------------------------------------
      0081A9                         56 _SPI_read:
                                     57 ;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 10: SPI_write(0xFF);
      0081A9 4B FF            [ 1]   58 	push	#0xff
      0081AB CD 81 BB         [ 4]   59 	call	_SPI_write
      0081AE 84               [ 1]   60 	pop	a
                                     61 ;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 11: while (!(SPI_SR & (1 << SPI_SR_RXNE)));
      0081AF                         62 00101$:
      0081AF AE 52 03         [ 2]   63 	ldw	x, #0x5203
      0081B2 F6               [ 1]   64 	ld	a, (x)
      0081B3 44               [ 1]   65 	srl	a
      0081B4 24 F9            [ 1]   66 	jrnc	00101$
                                     67 ;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 12: return SPI_DR;
      0081B6 AE 52 04         [ 2]   68 	ldw	x, #0x5204
      0081B9 F6               [ 1]   69 	ld	a, (x)
      0081BA 81               [ 4]   70 	ret
                                     71 ;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 15: void SPI_write(uint8_t data) {
                                     72 ;	-----------------------------------------
                                     73 ;	 function SPI_write
                                     74 ;	-----------------------------------------
      0081BB                         75 _SPI_write:
                                     76 ;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 16: SPI_DR = data;
      0081BB AE 52 04         [ 2]   77 	ldw	x, #0x5204
      0081BE 7B 03            [ 1]   78 	ld	a, (0x03, sp)
      0081C0 F7               [ 1]   79 	ld	(x), a
                                     80 ;	/home/scott/Samples/stm8-bare-min/stm8/spi.c: 17: while (!(SPI_SR & (1 << SPI_SR_TXE)));
      0081C1                         81 00101$:
      0081C1 AE 52 03         [ 2]   82 	ldw	x, #0x5203
      0081C4 F6               [ 1]   83 	ld	a, (x)
      0081C5 A5 02            [ 1]   84 	bcp	a, #0x02
      0081C7 27 F8            [ 1]   85 	jreq	00101$
      0081C9 81               [ 4]   86 	ret
                                     87 	.area CODE
                                     88 	.area INITIALIZER
                                     89 	.area CABS (ABS)
