                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.7.0 #10231 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module interrupts
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _TLI_IRQHandler
                                     12 	.globl _AWU_IRQHandler
                                     13 	.globl _CLK_IRQHandler
                                     14 	.globl _EXTI_PORTA_IRQHandler
                                     15 	.globl _EXTI_PORTB_IRQHandler
                                     16 	.globl _EXTI_PORTC_IRQHandler
                                     17 	.globl _EXTI_PORTD_IRQHandler
                                     18 	.globl _EXTI_PORTE_IRQHandler
                                     19 	.globl _SPI_IRQHandler
                                     20 	.globl _TIM1_UPD_OVF_TRG_BRK_IRQHandler
                                     21 	.globl _TIM1_CAP_COM_IRQHandler
                                     22 	.globl _TIM2_UPD_OVF_BRK_IRQHandler
                                     23 	.globl _TIM2_CAP_COM_IRQHandler
                                     24 	.globl _UART1_TX_IRQHandler
                                     25 	.globl _UART1_RX_IRQHandler
                                     26 	.globl _I2C_IRQHandler
                                     27 	.globl _ADC1_IRQHandler
                                     28 	.globl _TIM4_UPD_OVF_IRQHandler
                                     29 	.globl _EEPROM_EEC_IRQHandler
                                     30 ;--------------------------------------------------------
                                     31 ; ram data
                                     32 ;--------------------------------------------------------
                                     33 	.area DATA
                                     34 ;--------------------------------------------------------
                                     35 ; ram data
                                     36 ;--------------------------------------------------------
                                     37 	.area INITIALIZED
                                     38 ;--------------------------------------------------------
                                     39 ; absolute external ram data
                                     40 ;--------------------------------------------------------
                                     41 	.area DABS (ABS)
                                     42 ;--------------------------------------------------------
                                     43 ; global & static initialisations
                                     44 ;--------------------------------------------------------
                                     45 	.area HOME
                                     46 	.area GSINIT
                                     47 	.area GSFINAL
                                     48 	.area GSINIT
                                     49 ;--------------------------------------------------------
                                     50 ; Home
                                     51 ;--------------------------------------------------------
                                     52 	.area HOME
                                     53 	.area HOME
                                     54 ;--------------------------------------------------------
                                     55 ; code
                                     56 ;--------------------------------------------------------
                                     57 	.area CODE
                                     58 ;	../src/interrupts.c: 26: INTERRUPT_HANDLER(TLI_IRQHandler, 0){}
                                     59 ;	-----------------------------------------
                                     60 ;	 function TLI_IRQHandler
                                     61 ;	-----------------------------------------
      00808C                         62 _TLI_IRQHandler:
      00808C 80               [11]   63 	iret
                                     64 ;	../src/interrupts.c: 29: INTERRUPT_HANDLER(AWU_IRQHandler, 1){}
                                     65 ;	-----------------------------------------
                                     66 ;	 function AWU_IRQHandler
                                     67 ;	-----------------------------------------
      00808D                         68 _AWU_IRQHandler:
      00808D 80               [11]   69 	iret
                                     70 ;	../src/interrupts.c: 32: INTERRUPT_HANDLER(CLK_IRQHandler, 2){}
                                     71 ;	-----------------------------------------
                                     72 ;	 function CLK_IRQHandler
                                     73 ;	-----------------------------------------
      00808E                         74 _CLK_IRQHandler:
      00808E 80               [11]   75 	iret
                                     76 ;	../src/interrupts.c: 35: INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3){}
                                     77 ;	-----------------------------------------
                                     78 ;	 function EXTI_PORTA_IRQHandler
                                     79 ;	-----------------------------------------
      00808F                         80 _EXTI_PORTA_IRQHandler:
      00808F 80               [11]   81 	iret
                                     82 ;	../src/interrupts.c: 38: INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4){}
                                     83 ;	-----------------------------------------
                                     84 ;	 function EXTI_PORTB_IRQHandler
                                     85 ;	-----------------------------------------
      008090                         86 _EXTI_PORTB_IRQHandler:
      008090 80               [11]   87 	iret
                                     88 ;	../src/interrupts.c: 41: INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5){
                                     89 ;	-----------------------------------------
                                     90 ;	 function EXTI_PORTC_IRQHandler
                                     91 ;	-----------------------------------------
      008091                         92 _EXTI_PORTC_IRQHandler:
                                     93 ;	../src/interrupts.c: 42: }
      008091 80               [11]   94 	iret
                                     95 ;	../src/interrupts.c: 45: INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6){
                                     96 ;	-----------------------------------------
                                     97 ;	 function EXTI_PORTD_IRQHandler
                                     98 ;	-----------------------------------------
      008092                         99 _EXTI_PORTD_IRQHandler:
                                    100 ;	../src/interrupts.c: 46: }
      008092 80               [11]  101 	iret
                                    102 ;	../src/interrupts.c: 49: INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 7){}
                                    103 ;	-----------------------------------------
                                    104 ;	 function EXTI_PORTE_IRQHandler
                                    105 ;	-----------------------------------------
      008093                        106 _EXTI_PORTE_IRQHandler:
      008093 80               [11]  107 	iret
                                    108 ;	../src/interrupts.c: 65: INTERRUPT_HANDLER(SPI_IRQHandler, 10){}
                                    109 ;	-----------------------------------------
                                    110 ;	 function SPI_IRQHandler
                                    111 ;	-----------------------------------------
      008094                        112 _SPI_IRQHandler:
      008094 80               [11]  113 	iret
                                    114 ;	../src/interrupts.c: 68: INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11){
                                    115 ;	-----------------------------------------
                                    116 ;	 function TIM1_UPD_OVF_TRG_BRK_IRQHandler
                                    117 ;	-----------------------------------------
      008095                        118 _TIM1_UPD_OVF_TRG_BRK_IRQHandler:
                                    119 ;	../src/interrupts.c: 69: }
      008095 80               [11]  120 	iret
                                    121 ;	../src/interrupts.c: 72: INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, 12){}
                                    122 ;	-----------------------------------------
                                    123 ;	 function TIM1_CAP_COM_IRQHandler
                                    124 ;	-----------------------------------------
      008096                        125 _TIM1_CAP_COM_IRQHandler:
      008096 80               [11]  126 	iret
                                    127 ;	../src/interrupts.c: 84: INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13){
                                    128 ;	-----------------------------------------
                                    129 ;	 function TIM2_UPD_OVF_BRK_IRQHandler
                                    130 ;	-----------------------------------------
      008097                        131 _TIM2_UPD_OVF_BRK_IRQHandler:
                                    132 ;	../src/interrupts.c: 85: }
      008097 80               [11]  133 	iret
                                    134 ;	../src/interrupts.c: 89: INTERRUPT_HANDLER(TIM2_CAP_COM_IRQHandler, 14){
                                    135 ;	-----------------------------------------
                                    136 ;	 function TIM2_CAP_COM_IRQHandler
                                    137 ;	-----------------------------------------
      008098                        138 _TIM2_CAP_COM_IRQHandler:
                                    139 ;	../src/interrupts.c: 90: }
      008098 80               [11]  140 	iret
                                    141 ;	../src/interrupts.c: 105: INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17){}
                                    142 ;	-----------------------------------------
                                    143 ;	 function UART1_TX_IRQHandler
                                    144 ;	-----------------------------------------
      008099                        145 _UART1_TX_IRQHandler:
      008099 80               [11]  146 	iret
                                    147 ;	../src/interrupts.c: 108: INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18){
                                    148 ;	-----------------------------------------
                                    149 ;	 function UART1_RX_IRQHandler
                                    150 ;	-----------------------------------------
      00809A                        151 _UART1_RX_IRQHandler:
      00809A 52 02            [ 2]  152 	sub	sp, #2
                                    153 ;	../src/interrupts.c: 110: if(UART1->SR & UART1_SR_RXNE){ // data received
      00809C C6 52 30         [ 1]  154 	ld	a, 0x5230
      00809F A5 20            [ 1]  155 	bcp	a, #0x20
      0080A1 27 3A            [ 1]  156 	jreq	00115$
                                    157 ;	../src/interrupts.c: 111: rb = UART1->DR; // read received byte & clear RXNE flag
      0080A3 C6 52 31         [ 1]  158 	ld	a, 0x5231
                                    159 ;	../src/interrupts.c: 113: UART_rx[UART_rx_cur_i++] = rb; // put received byte into cycled buffer
      0080A6 AE 00 13         [ 2]  160 	ldw	x, #_UART_rx+0
      0080A9 1F 01            [ 2]  161 	ldw	(0x01, sp), x
      0080AB 41               [ 1]  162 	exg	a, xl
      0080AC C6 00 32         [ 1]  163 	ld	a, _UART_rx_cur_i+0
      0080AF 41               [ 1]  164 	exg	a, xl
      0080B0 72 5C 00 32      [ 1]  165 	inc	_UART_rx_cur_i+0
      0080B4 02               [ 1]  166 	rlwa	x
      0080B5 4F               [ 1]  167 	clr	a
      0080B6 01               [ 1]  168 	rrwa	x
      0080B7 72 FB 01         [ 2]  169 	addw	x, (0x01, sp)
      0080BA F7               [ 1]  170 	ld	(x), a
                                    171 ;	../src/interrupts.c: 114: if(UART_rx_cur_i == UART_rx_start_i){ // Oops: buffer overflow! Just forget old data
      0080BB C6 00 31         [ 1]  172 	ld	a, _UART_rx_start_i+0
      0080BE C1 00 32         [ 1]  173 	cp	a, _UART_rx_cur_i+0
      0080C1 26 0F            [ 1]  174 	jrne	00110$
                                    175 ;	../src/interrupts.c: 115: UART_rx_start_i++;
      0080C3 72 5C 00 31      [ 1]  176 	inc	_UART_rx_start_i+0
                                    177 ;	../src/interrupts.c: 116: check_UART_pointer(UART_rx_start_i);
      0080C7 C6 00 31         [ 1]  178 	ld	a, _UART_rx_start_i+0
      0080CA A1 1E            [ 1]  179 	cp	a, #0x1e
      0080CC 26 04            [ 1]  180 	jrne	00110$
      0080CE 72 5F 00 31      [ 1]  181 	clr	_UART_rx_start_i+0
                                    182 ;	../src/interrupts.c: 118: check_UART_pointer(UART_rx_cur_i);
      0080D2                        183 00110$:
      0080D2 C6 00 32         [ 1]  184 	ld	a, _UART_rx_cur_i+0
      0080D5 A1 1E            [ 1]  185 	cp	a, #0x1e
      0080D7 26 04            [ 1]  186 	jrne	00115$
      0080D9 72 5F 00 32      [ 1]  187 	clr	_UART_rx_cur_i+0
      0080DD                        188 00115$:
                                    189 ;	../src/interrupts.c: 120: }
      0080DD 5B 02            [ 2]  190 	addw	sp, #2
      0080DF 80               [11]  191 	iret
                                    192 ;	../src/interrupts.c: 124: INTERRUPT_HANDLER(I2C_IRQHandler, 19){}
                                    193 ;	-----------------------------------------
                                    194 ;	 function I2C_IRQHandler
                                    195 ;	-----------------------------------------
      0080E0                        196 _I2C_IRQHandler:
      0080E0 80               [11]  197 	iret
                                    198 ;	../src/interrupts.c: 148: INTERRUPT_HANDLER(ADC1_IRQHandler, 22){
                                    199 ;	-----------------------------------------
                                    200 ;	 function ADC1_IRQHandler
                                    201 ;	-----------------------------------------
      0080E1                        202 _ADC1_IRQHandler:
                                    203 ;	../src/interrupts.c: 150: }
      0080E1 80               [11]  204 	iret
                                    205 ;	../src/interrupts.c: 158: void TIM4_UPD_OVF_IRQHandler() __interrupt(23){
                                    206 ;	-----------------------------------------
                                    207 ;	 function TIM4_UPD_OVF_IRQHandler
                                    208 ;	-----------------------------------------
      0080E2                        209 _TIM4_UPD_OVF_IRQHandler:
                                    210 ;	../src/interrupts.c: 159: if(TIM4->SR1 & TIM4_SR1_UIF){ // update interrupt
      0080E2 C6 53 44         [ 1]  211 	ld	a, 0x5344
      0080E5 44               [ 1]  212 	srl	a
      0080E6 24 1B            [ 1]  213 	jrnc	00102$
                                    214 ;	../src/interrupts.c: 160: Global_time++; // increase timer
      0080E8 CE 00 03         [ 2]  215 	ldw	x, _Global_time+2
      0080EB 1C 00 01         [ 2]  216 	addw	x, #0x0001
      0080EE C6 00 02         [ 1]  217 	ld	a, _Global_time+1
      0080F1 A9 00            [ 1]  218 	adc	a, #0x00
      0080F3 90 97            [ 1]  219 	ld	yl, a
      0080F5 C6 00 01         [ 1]  220 	ld	a, _Global_time+0
      0080F8 A9 00            [ 1]  221 	adc	a, #0x00
      0080FA 90 95            [ 1]  222 	ld	yh, a
      0080FC CF 00 03         [ 2]  223 	ldw	_Global_time+2, x
      0080FF 90 CF 00 01      [ 2]  224 	ldw	_Global_time+0, y
      008103                        225 00102$:
                                    226 ;	../src/interrupts.c: 162: TIM4->SR1 = 0; // clear all interrupt flags
      008103 35 00 53 44      [ 1]  227 	mov	0x5344+0, #0x00
                                    228 ;	../src/interrupts.c: 163: }
      008107 80               [11]  229 	iret
                                    230 ;	../src/interrupts.c: 167: INTERRUPT_HANDLER(EEPROM_EEC_IRQHandler, 24){}
                                    231 ;	-----------------------------------------
                                    232 ;	 function EEPROM_EEC_IRQHandler
                                    233 ;	-----------------------------------------
      008108                        234 _EEPROM_EEC_IRQHandler:
      008108 80               [11]  235 	iret
                                    236 	.area CODE
                                    237 	.area INITIALIZER
                                    238 	.area CABS (ABS)
