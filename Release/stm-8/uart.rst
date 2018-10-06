                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.6.0 #9615 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module uart
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _UART_rx_cur_i
                                     12 	.globl _UART_rx_start_i
                                     13 	.globl _UART_rx
                                     14 	.globl _uart_init
                                     15 	.globl _uart_write
                                     16 	.globl _uart_read
                                     17 	.globl _UART_read_byte
                                     18 ;--------------------------------------------------------
                                     19 ; ram data
                                     20 ;--------------------------------------------------------
                                     21 	.area DATA
      000001                         22 _UART_rx::
      000001                         23 	.ds 8
                                     24 ;--------------------------------------------------------
                                     25 ; ram data
                                     26 ;--------------------------------------------------------
                                     27 	.area INITIALIZED
      000009                         28 _UART_rx_start_i::
      000009                         29 	.ds 1
      00000A                         30 _UART_rx_cur_i::
      00000A                         31 	.ds 1
                                     32 ;--------------------------------------------------------
                                     33 ; absolute external ram data
                                     34 ;--------------------------------------------------------
                                     35 	.area DABS (ABS)
                                     36 ;--------------------------------------------------------
                                     37 ; global & static initialisations
                                     38 ;--------------------------------------------------------
                                     39 	.area HOME
                                     40 	.area GSINIT
                                     41 	.area GSFINAL
                                     42 	.area GSINIT
                                     43 ;--------------------------------------------------------
                                     44 ; Home
                                     45 ;--------------------------------------------------------
                                     46 	.area HOME
                                     47 	.area HOME
                                     48 ;--------------------------------------------------------
                                     49 ; code
                                     50 ;--------------------------------------------------------
                                     51 	.area CODE
                                     52 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 8: void uart_init() {
                                     53 ;	-----------------------------------------
                                     54 ;	 function uart_init
                                     55 ;	-----------------------------------------
      0081CA                         56 _uart_init:
                                     57 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 12: UART1_BRR2 = ((div >> 8) & 0xF0) + (div & 0x0F);
      0081CA 35 00 52 33      [ 1]   58 	mov	0x5233+0, #0x00
                                     59 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 13: UART1_BRR1 = div >> 4;
      0081CE 35 0D 52 32      [ 1]   60 	mov	0x5232+0, #0x0d
                                     61 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 16: UART1_CR2 = UART1_CR2_TEN | UART1_CR2_REN | UART1_CR2_RIEN; // Allow RX/TX, generate ints on rx
      0081D2 35 23 52 35      [ 1]   62 	mov	0x5235+0, #0x23
      0081D6 81               [ 4]   63 	ret
                                     64 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 19: void uart_write(uint8_t data) {
                                     65 ;	-----------------------------------------
                                     66 ;	 function uart_write
                                     67 ;	-----------------------------------------
      0081D7                         68 _uart_write:
                                     69 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 20: UART1_DR = data;
      0081D7 AE 52 31         [ 2]   70 	ldw	x, #0x5231
      0081DA 7B 03            [ 1]   71 	ld	a, (0x03, sp)
      0081DC F7               [ 1]   72 	ld	(x), a
                                     73 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 21: while (!(UART1_SR & (1 << UART1_SR_TC)));
      0081DD                         74 00101$:
      0081DD AE 52 30         [ 2]   75 	ldw	x, #0x5230
      0081E0 F6               [ 1]   76 	ld	a, (x)
      0081E1 A5 40            [ 1]   77 	bcp	a, #0x40
      0081E3 27 F8            [ 1]   78 	jreq	00101$
      0081E5 81               [ 4]   79 	ret
                                     80 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 24: uint8_t uart_read() {
                                     81 ;	-----------------------------------------
                                     82 ;	 function uart_read
                                     83 ;	-----------------------------------------
      0081E6                         84 _uart_read:
                                     85 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 25: while (!(UART1_SR & (1 << UART1_SR_RXNE)));
      0081E6                         86 00101$:
      0081E6 AE 52 30         [ 2]   87 	ldw	x, #0x5230
      0081E9 F6               [ 1]   88 	ld	a, (x)
      0081EA A5 20            [ 1]   89 	bcp	a, #0x20
      0081EC 27 F8            [ 1]   90 	jreq	00101$
                                     91 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 26: return UART1_DR;
      0081EE AE 52 31         [ 2]   92 	ldw	x, #0x5231
      0081F1 F6               [ 1]   93 	ld	a, (x)
      0081F2 81               [ 4]   94 	ret
                                     95 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 34: uint8_t UART_read_byte(uint8_t *byte){
                                     96 ;	-----------------------------------------
                                     97 ;	 function UART_read_byte
                                     98 ;	-----------------------------------------
      0081F3                         99 _UART_read_byte:
      0081F3 52 02            [ 2]  100 	sub	sp, #2
                                    101 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 35: if(UART_rx_start_i == UART_rx_cur_i) // buffer is empty
      0081F5 C6 00 0A         [ 1]  102 	ld	a, _UART_rx_cur_i+0
      0081F8 C1 00 09         [ 1]  103 	cp	a, _UART_rx_start_i+0
      0081FB 26 03            [ 1]  104 	jrne	00102$
                                    105 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 36: return 0;
      0081FD 4F               [ 1]  106 	clr	a
      0081FE 20 24            [ 2]  107 	jra	00108$
      008200                        108 00102$:
                                    109 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 37: *byte = UART_rx[UART_rx_start_i++];
      008200 16 05            [ 2]  110 	ldw	y, (0x05, sp)
      008202 AE 00 01         [ 2]  111 	ldw	x, #_UART_rx+0
      008205 1F 01            [ 2]  112 	ldw	(0x01, sp), x
      008207 C6 00 09         [ 1]  113 	ld	a, _UART_rx_start_i+0
      00820A 97               [ 1]  114 	ld	xl, a
      00820B 72 5C 00 09      [ 1]  115 	inc	_UART_rx_start_i+0
      00820F 4F               [ 1]  116 	clr	a
      008210 95               [ 1]  117 	ld	xh, a
      008211 72 FB 01         [ 2]  118 	addw	x, (0x01, sp)
      008214 F6               [ 1]  119 	ld	a, (x)
      008215 90 F7            [ 1]  120 	ld	(y), a
                                    121 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 38: check_UART_pointer(UART_rx_start_i);
      008217 C6 00 09         [ 1]  122 	ld	a, _UART_rx_start_i+0
      00821A A1 08            [ 1]  123 	cp	a, #0x08
      00821C 26 04            [ 1]  124 	jrne	00106$
      00821E 72 5F 00 09      [ 1]  125 	clr	_UART_rx_start_i+0
      008222                        126 00106$:
                                    127 ;	/home/scott/Samples/stm8-bare-min/stm8/uart.c: 39: return 1;
      008222 A6 01            [ 1]  128 	ld	a, #0x01
      008224                        129 00108$:
      008224 5B 02            [ 2]  130 	addw	sp, #2
      008226 81               [ 4]  131 	ret
                                    132 	.area CODE
                                    133 	.area INITIALIZER
      0098A6                        134 __xinit__UART_rx_start_i:
      0098A6 00                     135 	.db #0x00	; 0
      0098A7                        136 __xinit__UART_rx_cur_i:
      0098A7 00                     137 	.db #0x00	; 0
                                    138 	.area CABS (ABS)
