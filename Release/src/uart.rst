                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.7.0 #10231 (Linux)
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
      000013                         22 _UART_rx::
      000013                         23 	.ds 30
                                     24 ;--------------------------------------------------------
                                     25 ; ram data
                                     26 ;--------------------------------------------------------
                                     27 	.area INITIALIZED
      000031                         28 _UART_rx_start_i::
      000031                         29 	.ds 1
      000032                         30 _UART_rx_cur_i::
      000032                         31 	.ds 1
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
                                     52 ;	../src/uart.c: 8: void uart_init() {
                                     53 ;	-----------------------------------------
                                     54 ;	 function uart_init
                                     55 ;	-----------------------------------------
      008779                         56 _uart_init:
                                     57 ;	../src/uart.c: 10: tmp = UART1->SR;
      008779 AE 52 30         [ 2]   58 	ldw	x, #0x5230
      00877C F6               [ 1]   59 	ld	a, (x)
                                     60 ;	../src/uart.c: 11: tmp = UART1->DR;
      00877D AE 52 31         [ 2]   61 	ldw	x, #0x5231
      008780 F6               [ 1]   62 	ld	a, (x)
                                     63 ;	../src/uart.c: 12: UART1->CR1 = 0x00;
      008781 35 00 52 34      [ 1]   64 	mov	0x5234+0, #0x00
                                     65 ;	../src/uart.c: 14: UART1->BRR2 = 0x03; //((div >> 8) & 0xF0) + (div & 0x0F);
      008785 35 03 52 33      [ 1]   66 	mov	0x5233+0, #0x03
                                     67 ;	../src/uart.c: 15: UART1->BRR1 = 0x6a; //div >> 4;
      008789 35 6A 52 32      [ 1]   68 	mov	0x5232+0, #0x6a
                                     69 ;	../src/uart.c: 16: UART1->CR2 = UART1_CR2_TEN | UART1_CR2_REN | UART1_CR2_RIEN; // Allow RX/TX, generate ints on rx
      00878D 35 2C 52 35      [ 1]   70 	mov	0x5235+0, #0x2c
                                     71 ;	../src/uart.c: 17: }
      008791 81               [ 4]   72 	ret
                                     73 ;	../src/uart.c: 19: void uart_write(uint8_t data) {
                                     74 ;	-----------------------------------------
                                     75 ;	 function uart_write
                                     76 ;	-----------------------------------------
      008792                         77 _uart_write:
                                     78 ;	../src/uart.c: 20: UART1->DR = data;
      008792 AE 52 31         [ 2]   79 	ldw	x, #0x5231
      008795 7B 03            [ 1]   80 	ld	a, (0x03, sp)
      008797 F7               [ 1]   81 	ld	(x), a
                                     82 ;	../src/uart.c: 21: while (!(UART1->SR & UART1_SR_TC)) ;
      008798                         83 00101$:
      008798 C6 52 30         [ 1]   84 	ld	a, 0x5230
      00879B A5 40            [ 1]   85 	bcp	a, #0x40
      00879D 27 F9            [ 1]   86 	jreq	00101$
                                     87 ;	../src/uart.c: 22: UART1->SR &= ~(UART1_SR_TC);
      00879F A4 BF            [ 1]   88 	and	a, #0xbf
      0087A1 C7 52 30         [ 1]   89 	ld	0x5230, a
                                     90 ;	../src/uart.c: 23: }
      0087A4 81               [ 4]   91 	ret
                                     92 ;	../src/uart.c: 25: uint8_t uart_read() {
                                     93 ;	-----------------------------------------
                                     94 ;	 function uart_read
                                     95 ;	-----------------------------------------
      0087A5                         96 _uart_read:
                                     97 ;	../src/uart.c: 28: while (!(UART1->SR & UART1_SR_RXNE)) ;
      0087A5                         98 00101$:
      0087A5 C6 52 30         [ 1]   99 	ld	a, 0x5230
      0087A8 A5 20            [ 1]  100 	bcp	a, #0x20
      0087AA 27 F9            [ 1]  101 	jreq	00101$
                                    102 ;	../src/uart.c: 29: return UART1->DR;
      0087AC C6 52 31         [ 1]  103 	ld	a, 0x5231
                                    104 ;	../src/uart.c: 30: }
      0087AF 81               [ 4]  105 	ret
                                    106 ;	../src/uart.c: 37: uint8_t UART_read_byte(uint8_t *byte){
                                    107 ;	-----------------------------------------
                                    108 ;	 function UART_read_byte
                                    109 ;	-----------------------------------------
      0087B0                        110 _UART_read_byte:
      0087B0 52 02            [ 2]  111 	sub	sp, #2
                                    112 ;	../src/uart.c: 38: if(UART_rx_start_i == UART_rx_cur_i) // buffer is empty
      0087B2 C6 00 32         [ 1]  113 	ld	a, _UART_rx_cur_i+0
      0087B5 C1 00 31         [ 1]  114 	cp	a, _UART_rx_start_i+0
      0087B8 26 03            [ 1]  115 	jrne	00102$
                                    116 ;	../src/uart.c: 39: return 0;
      0087BA 4F               [ 1]  117 	clr	a
      0087BB 20 23            [ 2]  118 	jra	00108$
      0087BD                        119 00102$:
                                    120 ;	../src/uart.c: 40: *byte = UART_rx[UART_rx_start_i++];
      0087BD 16 05            [ 2]  121 	ldw	y, (0x05, sp)
      0087BF AE 00 13         [ 2]  122 	ldw	x, #_UART_rx+0
      0087C2 1F 01            [ 2]  123 	ldw	(0x01, sp), x
      0087C4 C6 00 31         [ 1]  124 	ld	a, _UART_rx_start_i+0
      0087C7 72 5C 00 31      [ 1]  125 	inc	_UART_rx_start_i+0
      0087CB 5F               [ 1]  126 	clrw	x
      0087CC 97               [ 1]  127 	ld	xl, a
      0087CD 72 FB 01         [ 2]  128 	addw	x, (0x01, sp)
      0087D0 F6               [ 1]  129 	ld	a, (x)
      0087D1 90 F7            [ 1]  130 	ld	(y), a
                                    131 ;	../src/uart.c: 41: check_UART_pointer(UART_rx_start_i);
      0087D3 C6 00 31         [ 1]  132 	ld	a, _UART_rx_start_i+0
      0087D6 A1 1E            [ 1]  133 	cp	a, #0x1e
      0087D8 26 04            [ 1]  134 	jrne	00106$
      0087DA 72 5F 00 31      [ 1]  135 	clr	_UART_rx_start_i+0
      0087DE                        136 00106$:
                                    137 ;	../src/uart.c: 42: return 1;
      0087DE A6 01            [ 1]  138 	ld	a, #0x01
      0087E0                        139 00108$:
                                    140 ;	../src/uart.c: 43: }
      0087E0 5B 02            [ 2]  141 	addw	sp, #2
      0087E2 81               [ 4]  142 	ret
                                    143 	.area CODE
                                    144 	.area INITIALIZER
      00948A                        145 __xinit__UART_rx_start_i:
      00948A 00                     146 	.db #0x00	; 0
      00948B                        147 __xinit__UART_rx_cur_i:
      00948B 00                     148 	.db #0x00	; 0
                                    149 	.area CABS (ABS)
