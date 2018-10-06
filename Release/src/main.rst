                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.7.0 #10231 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module main
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _main
                                     12 	.globl _ADC_init
                                     13 	.globl _ADC_read
                                     14 	.globl _InitialiseIWDG
                                     15 	.globl _InitialiseSystemClock
                                     16 	.globl _UART_read_byte
                                     17 	.globl _uart_write
                                     18 	.globl _uart_init
                                     19 	.globl _strcpy
                                     20 	.globl _sprintf
                                     21 	.globl _printf
                                     22 	.globl _version
                                     23 	.globl _address
                                     24 	.globl _esc
                                     25 	.globl _Global_time
                                     26 	.globl _putchar
                                     27 ;--------------------------------------------------------
                                     28 ; ram data
                                     29 ;--------------------------------------------------------
                                     30 	.area DATA
      000001                         31 _Global_time::
      000001                         32 	.ds 4
      000005                         33 _esc::
      000005                         34 	.ds 1
      000006                         35 _address::
      000006                         36 	.ds 1
      000007                         37 _version::
      000007                         38 	.ds 12
                                     39 ;--------------------------------------------------------
                                     40 ; ram data
                                     41 ;--------------------------------------------------------
                                     42 	.area INITIALIZED
                                     43 ;--------------------------------------------------------
                                     44 ; Stack segment in internal ram 
                                     45 ;--------------------------------------------------------
                                     46 	.area	SSEG
      FFFFFF                         47 __start__stack:
      FFFFFF                         48 	.ds	1
                                     49 
                                     50 ;--------------------------------------------------------
                                     51 ; absolute external ram data
                                     52 ;--------------------------------------------------------
                                     53 	.area DABS (ABS)
                                     54 ;--------------------------------------------------------
                                     55 ; interrupt vector 
                                     56 ;--------------------------------------------------------
                                     57 	.area HOME
      008000                         58 __interrupt_vect:
      008000 82 00 80 6F             59 	int s_GSINIT ; reset
      008004 82 00 00 00             60 	int 0x0000 ; trap
      008008 82 00 80 8C             61 	int _TLI_IRQHandler ; int0
      00800C 82 00 80 8D             62 	int _AWU_IRQHandler ; int1
      008010 82 00 80 8E             63 	int _CLK_IRQHandler ; int2
      008014 82 00 80 8F             64 	int _EXTI_PORTA_IRQHandler ; int3
      008018 82 00 80 90             65 	int _EXTI_PORTB_IRQHandler ; int4
      00801C 82 00 80 91             66 	int _EXTI_PORTC_IRQHandler ; int5
      008020 82 00 80 92             67 	int _EXTI_PORTD_IRQHandler ; int6
      008024 82 00 80 93             68 	int _EXTI_PORTE_IRQHandler ; int7
      008028 82 00 00 00             69 	int 0x0000 ; int8
      00802C 82 00 00 00             70 	int 0x0000 ; int9
      008030 82 00 80 94             71 	int _SPI_IRQHandler ; int10
      008034 82 00 80 95             72 	int _TIM1_UPD_OVF_TRG_BRK_IRQHandler ; int11
      008038 82 00 80 96             73 	int _TIM1_CAP_COM_IRQHandler ; int12
      00803C 82 00 80 97             74 	int _TIM2_UPD_OVF_BRK_IRQHandler ; int13
      008040 82 00 80 98             75 	int _TIM2_CAP_COM_IRQHandler ; int14
      008044 82 00 00 00             76 	int 0x0000 ; int15
      008048 82 00 00 00             77 	int 0x0000 ; int16
      00804C 82 00 80 99             78 	int _UART1_TX_IRQHandler ; int17
      008050 82 00 80 9A             79 	int _UART1_RX_IRQHandler ; int18
      008054 82 00 80 E0             80 	int _I2C_IRQHandler ; int19
      008058 82 00 00 00             81 	int 0x0000 ; int20
      00805C 82 00 00 00             82 	int 0x0000 ; int21
      008060 82 00 80 E1             83 	int _ADC1_IRQHandler ; int22
      008064 82 00 80 E2             84 	int _TIM4_UPD_OVF_IRQHandler ; int23
      008068 82 00 81 08             85 	int _EEPROM_EEC_IRQHandler ; int24
                                     86 ;--------------------------------------------------------
                                     87 ; global & static initialisations
                                     88 ;--------------------------------------------------------
                                     89 	.area HOME
                                     90 	.area GSINIT
                                     91 	.area GSFINAL
                                     92 	.area GSINIT
      00806F                         93 __sdcc_gs_init_startup:
      00806F                         94 __sdcc_init_data:
                                     95 ; stm8_genXINIT() start
      00806F AE 00 30         [ 2]   96 	ldw x, #l_DATA
      008072 27 07            [ 1]   97 	jreq	00002$
      008074                         98 00001$:
      008074 72 4F 00 00      [ 1]   99 	clr (s_DATA - 1, x)
      008078 5A               [ 2]  100 	decw x
      008079 26 F9            [ 1]  101 	jrne	00001$
      00807B                        102 00002$:
      00807B AE 00 02         [ 2]  103 	ldw	x, #l_INITIALIZER
      00807E 27 09            [ 1]  104 	jreq	00004$
      008080                        105 00003$:
      008080 D6 94 89         [ 1]  106 	ld	a, (s_INITIALIZER - 1, x)
      008083 D7 00 30         [ 1]  107 	ld	(s_INITIALIZED - 1, x), a
      008086 5A               [ 2]  108 	decw	x
      008087 26 F7            [ 1]  109 	jrne	00003$
      008089                        110 00004$:
                                    111 ; stm8_genXINIT() end
                                    112 	.area GSFINAL
      008089 CC 80 6C         [ 2]  113 	jp	__sdcc_program_startup
                                    114 ;--------------------------------------------------------
                                    115 ; Home
                                    116 ;--------------------------------------------------------
                                    117 	.area HOME
                                    118 	.area HOME
      00806C                        119 __sdcc_program_startup:
      00806C CC 81 9E         [ 2]  120 	jp	_main
                                    121 ;	return from main will return to caller
                                    122 ;--------------------------------------------------------
                                    123 ; code
                                    124 ;--------------------------------------------------------
                                    125 	.area CODE
                                    126 ;	../src/main.c: 33: int putchar(int c) {
                                    127 ;	-----------------------------------------
                                    128 ;	 function putchar
                                    129 ;	-----------------------------------------
      008109                        130 _putchar:
                                    131 ;	../src/main.c: 34: uart_write(c);
      008109 7B 04            [ 1]  132 	ld	a, (0x04, sp)
      00810B 88               [ 1]  133 	push	a
      00810C CD 87 92         [ 4]  134 	call	_uart_write
      00810F 84               [ 1]  135 	pop	a
                                    136 ;	../src/main.c: 35: return c;
      008110 1E 03            [ 2]  137 	ldw	x, (0x03, sp)
                                    138 ;	../src/main.c: 36: }
      008112 81               [ 4]  139 	ret
                                    140 ;	../src/main.c: 42: void InitialiseSystemClock()
                                    141 ;	-----------------------------------------
                                    142 ;	 function InitialiseSystemClock
                                    143 ;	-----------------------------------------
      008113                        144 _InitialiseSystemClock:
                                    145 ;	../src/main.c: 44: CLK->ICKR = 0;                       //  Reset the Internal Clock Register.
      008113 35 00 50 C0      [ 1]  146 	mov	0x50c0+0, #0x00
                                    147 ;	../src/main.c: 45: CLK->ICKR = CLK_ICKR_HSIEN;          //  Enable the HSI.
      008117 35 01 50 C0      [ 1]  148 	mov	0x50c0+0, #0x01
                                    149 ;	../src/main.c: 46: CLK->ECKR = 0;                       //  Disable the external clock.
      00811B 35 00 50 C1      [ 1]  150 	mov	0x50c1+0, #0x00
                                    151 ;	../src/main.c: 47: while (!(CLK->ICKR & CLK_ICKR_HSIRDY)); //  Wait for the HSI to be ready for use.
      00811F                        152 00101$:
      00811F C6 50 C0         [ 1]  153 	ld	a, 0x50c0
      008122 A5 02            [ 1]  154 	bcp	a, #0x02
      008124 27 F9            [ 1]  155 	jreq	00101$
                                    156 ;	../src/main.c: 48: CLK->CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
      008126 35 00 50 C6      [ 1]  157 	mov	0x50c6+0, #0x00
                                    158 ;	../src/main.c: 49: CLK->PCKENR1 = 0xff; //CLK_PCKENR1_TIM4 | CLK_PCKENR1_UART1 | CLK_PCKENR1_SPI | CLK_PCKENR1_I2C ;  //  Enable select peripheral clocks.
      00812A 35 FF 50 C7      [ 1]  159 	mov	0x50c7+0, #0xff
                                    160 ;	../src/main.c: 50: CLK->PCKENR2 = 0xff; //CLK_PCKENR2_AWU;      //  Only enable the AWU watchdog service
      00812E 35 FF 50 CA      [ 1]  161 	mov	0x50ca+0, #0xff
                                    162 ;	../src/main.c: 51: CLK->CCOR = 0;                       //  Turn off CCO.
      008132 35 00 50 C9      [ 1]  163 	mov	0x50c9+0, #0x00
                                    164 ;	../src/main.c: 52: CLK->HSITRIMR = 0;                   //  Turn off any HSIU trimming.
      008136 35 00 50 CC      [ 1]  165 	mov	0x50cc+0, #0x00
                                    166 ;	../src/main.c: 53: CLK->SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
      00813A 35 00 50 CD      [ 1]  167 	mov	0x50cd+0, #0x00
                                    168 ;	../src/main.c: 54: CLK->SWR = 0xe1;                     //  Use HSI as the clock source.
      00813E 35 E1 50 C4      [ 1]  169 	mov	0x50c4+0, #0xe1
                                    170 ;	../src/main.c: 55: CLK->SWCR = 0;                       //  Reset the clock switch control register.
      008142 35 00 50 C5      [ 1]  171 	mov	0x50c5+0, #0x00
                                    172 ;	../src/main.c: 56: CLK->SWCR |= CLK_SWCR_SWEN;          //  Enable switching.
      008146 72 12 50 C5      [ 1]  173 	bset	20677, #1
                                    174 ;	../src/main.c: 57: while (CLK->SWCR & CLK_SWCR_SWBSY);  //  Pause while the clock switch is busy.
      00814A                        175 00104$:
      00814A C6 50 C5         [ 1]  176 	ld	a, 0x50c5
      00814D 44               [ 1]  177 	srl	a
      00814E 25 FA            [ 1]  178 	jrc	00104$
                                    179 ;	../src/main.c: 58: }
      008150 81               [ 4]  180 	ret
                                    181 ;	../src/main.c: 63: void InitialiseIWDG()
                                    182 ;	-----------------------------------------
                                    183 ;	 function InitialiseIWDG
                                    184 ;	-----------------------------------------
      008151                        185 _InitialiseIWDG:
                                    186 ;	../src/main.c: 65: IWDG->KR = 0xcc;         //  Start the independent watchdog.
      008151 35 CC 50 E0      [ 1]  187 	mov	0x50e0+0, #0xcc
                                    188 ;	../src/main.c: 66: IWDG->KR = 0x55;         //  Allow the IWDG registers to be programmed.
      008155 35 55 50 E0      [ 1]  189 	mov	0x50e0+0, #0x55
                                    190 ;	../src/main.c: 67: IWDG->PR = 0x06;         //  Prescaler is 6 => each count is 1.02 second with RLR = 0xff
      008159 35 06 50 E1      [ 1]  191 	mov	0x50e1+0, #0x06
                                    192 ;	../src/main.c: 68: IWDG->RLR = 0xff;        //  Reload counter.  T = 2 x TLSI x PR x R LR
      00815D 35 FF 50 E2      [ 1]  193 	mov	0x50e2+0, #0xff
                                    194 ;	../src/main.c: 69: IWDG->KR = 0xaa;         //  Reset the counter.
      008161 35 AA 50 E0      [ 1]  195 	mov	0x50e0+0, #0xaa
                                    196 ;	../src/main.c: 70: }
      008165 81               [ 4]  197 	ret
                                    198 ;	../src/main.c: 73: uint16_t ADC_read() {
                                    199 ;	-----------------------------------------
                                    200 ;	 function ADC_read
                                    201 ;	-----------------------------------------
      008166                        202 _ADC_read:
      008166 52 04            [ 2]  203 	sub	sp, #4
                                    204 ;	../src/main.c: 75: ADC1->CR1 |= ADC1_CR1_ADON;
      008168 72 10 54 01      [ 1]  205 	bset	21505, #0
                                    206 ;	../src/main.c: 76: while (!(ADC1->CSR & (ADC1_CSR_EOC)));
      00816C                        207 00101$:
      00816C C6 54 00         [ 1]  208 	ld	a, 0x5400
      00816F 4D               [ 1]  209 	tnz	a
      008170 2A FA            [ 1]  210 	jrpl	00101$
                                    211 ;	../src/main.c: 77: adcL = ADC1->DRL;
      008172 AE 54 05         [ 2]  212 	ldw	x, #0x5405
      008175 F6               [ 1]  213 	ld	a, (x)
      008176 90 97            [ 1]  214 	ld	yl, a
                                    215 ;	../src/main.c: 78: adcH = ADC1->DRH;
      008178 AE 54 04         [ 2]  216 	ldw	x, #0x5404
      00817B F6               [ 1]  217 	ld	a, (x)
      00817C 97               [ 1]  218 	ld	xl, a
                                    219 ;	../src/main.c: 79: ADC1->CSR &= ~(ADC1_CSR_EOC); // Clear EOC flag
      00817D 72 1F 54 00      [ 1]  220 	bres	21504, #7
                                    221 ;	../src/main.c: 80: return (adcL | (adcH << 8));
      008181 9F               [ 1]  222 	ld	a, xl
      008182 0F 04            [ 1]  223 	clr	(0x04, sp)
      008184 0F 01            [ 1]  224 	clr	(0x01, sp)
      008186 1A 01            [ 1]  225 	or	a, (0x01, sp)
      008188 95               [ 1]  226 	ld	xh, a
      008189 90 9F            [ 1]  227 	ld	a, yl
      00818B 1A 04            [ 1]  228 	or	a, (0x04, sp)
      00818D 97               [ 1]  229 	ld	xl, a
                                    230 ;	../src/main.c: 81: }
      00818E 5B 04            [ 2]  231 	addw	sp, #4
      008190 81               [ 4]  232 	ret
                                    233 ;	../src/main.c: 82: void ADC_init() {
                                    234 ;	-----------------------------------------
                                    235 ;	 function ADC_init
                                    236 ;	-----------------------------------------
      008191                        237 _ADC_init:
                                    238 ;	../src/main.c: 84: ADC1->CSR |= (1 << 2);
      008191 72 14 54 00      [ 1]  239 	bset	21504, #2
                                    240 ;	../src/main.c: 86: ADC1->CR2 |= ADC1_CR2_ALIGN;
      008195 72 16 54 02      [ 1]  241 	bset	21506, #3
                                    242 ;	../src/main.c: 88: ADC1->CR1 |= ADC1_CR1_ADON;
      008199 72 10 54 01      [ 1]  243 	bset	21505, #0
                                    244 ;	../src/main.c: 89: }
      00819D 81               [ 4]  245 	ret
                                    246 ;	../src/main.c: 90: void main() {
                                    247 ;	-----------------------------------------
                                    248 ;	 function main
                                    249 ;	-----------------------------------------
      00819E                        250 _main:
      00819E 52 73            [ 2]  251 	sub	sp, #115
                                    252 ;	../src/main.c: 91: unsigned long esc_time = 0L;
      0081A0 5F               [ 1]  253 	clrw	x
      0081A1 1F 6C            [ 2]  254 	ldw	(0x6c, sp), x
      0081A3 1F 6A            [ 2]  255 	ldw	(0x6a, sp), x
                                    256 ;	../src/main.c: 99: esc = 0;
      0081A5 72 5F 00 05      [ 1]  257 	clr	_esc+0
                                    258 ;	../src/main.c: 101: sprintf(version,"%02d%02d%02d-%02d%02d", BUILD_YEAR - 2000, BUILD_MONTH, BUILD_DAY, BUILD_HOUR, BUILD_MIN);
      0081A9 AE 87 3C         [ 2]  259 	ldw	x, #___str_2+0
      0081AC 1F 1C            [ 2]  260 	ldw	(0x1c, sp), x
      0081AE F6               [ 1]  261 	ld	a, (x)
      0081AF 6B 2B            [ 1]  262 	ld	(0x2b, sp), a
      0081B1 7B 2B            [ 1]  263 	ld	a, (0x2b, sp)
      0081B3 A1 3F            [ 1]  264 	cp	a, #0x3f
      0081B5 26 06            [ 1]  265 	jrne	00426$
      0081B7 A6 01            [ 1]  266 	ld	a, #0x01
      0081B9 6B 1B            [ 1]  267 	ld	(0x1b, sp), a
      0081BB 20 02            [ 2]  268 	jra	00427$
      0081BD                        269 00426$:
      0081BD 0F 1B            [ 1]  270 	clr	(0x1b, sp)
      0081BF                        271 00427$:
      0081BF 0D 1B            [ 1]  272 	tnz	(0x1b, sp)
      0081C1 27 07            [ 1]  273 	jreq	00153$
      0081C3 AE 00 63         [ 2]  274 	ldw	x, #0x0063
      0081C6 1F 19            [ 2]  275 	ldw	(0x19, sp), x
      0081C8 20 22            [ 2]  276 	jra	00154$
      0081CA                        277 00153$:
      0081CA 1E 1C            [ 2]  278 	ldw	x, (0x1c, sp)
      0081CC E6 03            [ 1]  279 	ld	a, (0x3, x)
      0081CE 5F               [ 1]  280 	clrw	x
      0081CF 97               [ 1]  281 	ld	xl, a
      0081D0 1D 00 30         [ 2]  282 	subw	x, #0x0030
      0081D3 89               [ 2]  283 	pushw	x
      0081D4 58               [ 2]  284 	sllw	x
      0081D5 58               [ 2]  285 	sllw	x
      0081D6 72 FB 01         [ 2]  286 	addw	x, (1, sp)
      0081D9 58               [ 2]  287 	sllw	x
      0081DA 5B 02            [ 2]  288 	addw	sp, #2
      0081DC 1F 17            [ 2]  289 	ldw	(0x17, sp), x
      0081DE 1E 1C            [ 2]  290 	ldw	x, (0x1c, sp)
      0081E0 E6 04            [ 1]  291 	ld	a, (0x4, x)
      0081E2 5F               [ 1]  292 	clrw	x
      0081E3 97               [ 1]  293 	ld	xl, a
      0081E4 72 FB 17         [ 2]  294 	addw	x, (0x17, sp)
      0081E7 1D 00 30         [ 2]  295 	subw	x, #0x0030
      0081EA 1F 19            [ 2]  296 	ldw	(0x19, sp), x
      0081EC                        297 00154$:
      0081EC 0D 1B            [ 1]  298 	tnz	(0x1b, sp)
      0081EE 27 07            [ 1]  299 	jreq	00155$
      0081F0 AE 00 63         [ 2]  300 	ldw	x, #0x0063
      0081F3 1F 15            [ 2]  301 	ldw	(0x15, sp), x
      0081F5 20 24            [ 2]  302 	jra	00156$
      0081F7                        303 00155$:
      0081F7 5F               [ 1]  304 	clrw	x
      0081F8 7B 2B            [ 1]  305 	ld	a, (0x2b, sp)
      0081FA 97               [ 1]  306 	ld	xl, a
      0081FB 1D 00 30         [ 2]  307 	subw	x, #0x0030
      0081FE 89               [ 2]  308 	pushw	x
      0081FF 58               [ 2]  309 	sllw	x
      008200 58               [ 2]  310 	sllw	x
      008201 72 FB 01         [ 2]  311 	addw	x, (1, sp)
      008204 58               [ 2]  312 	sllw	x
      008205 5B 02            [ 2]  313 	addw	sp, #2
      008207 1F 35            [ 2]  314 	ldw	(0x35, sp), x
      008209 1E 1C            [ 2]  315 	ldw	x, (0x1c, sp)
      00820B E6 01            [ 1]  316 	ld	a, (0x1, x)
      00820D 5F               [ 1]  317 	clrw	x
      00820E 97               [ 1]  318 	ld	xl, a
      00820F 72 FB 35         [ 2]  319 	addw	x, (0x35, sp)
      008212 1D 00 30         [ 2]  320 	subw	x, #0x0030
      008215 1F 33            [ 2]  321 	ldw	(0x33, sp), x
      008217 90 93            [ 1]  322 	ldw	y, x
      008219 17 15            [ 2]  323 	ldw	(0x15, sp), y
      00821B                        324 00156$:
      00821B AE 87 30         [ 2]  325 	ldw	x, #___str_1+0
      00821E 1F 0A            [ 2]  326 	ldw	(0x0a, sp), x
      008220 F6               [ 1]  327 	ld	a, (x)
      008221 6B 09            [ 1]  328 	ld	(0x09, sp), a
      008223 7B 09            [ 1]  329 	ld	a, (0x09, sp)
      008225 A1 3F            [ 1]  330 	cp	a, #0x3f
      008227 26 06            [ 1]  331 	jrne	00431$
      008229 A6 01            [ 1]  332 	ld	a, #0x01
      00822B 6B 11            [ 1]  333 	ld	(0x11, sp), a
      00822D 20 02            [ 2]  334 	jra	00432$
      00822F                        335 00431$:
      00822F 0F 11            [ 1]  336 	clr	(0x11, sp)
      008231                        337 00432$:
      008231 0D 11            [ 1]  338 	tnz	(0x11, sp)
      008233 27 07            [ 1]  339 	jreq	00157$
      008235 AE 00 63         [ 2]  340 	ldw	x, #0x0063
      008238 1F 0F            [ 2]  341 	ldw	(0x0f, sp), x
      00823A 20 31            [ 2]  342 	jra	00158$
      00823C                        343 00157$:
      00823C 1E 0A            [ 2]  344 	ldw	x, (0x0a, sp)
      00823E E6 04            [ 1]  345 	ld	a, (0x4, x)
      008240 6B 14            [ 1]  346 	ld	(0x14, sp), a
      008242 7B 14            [ 1]  347 	ld	a, (0x14, sp)
      008244 A1 30            [ 1]  348 	cp	a, #0x30
      008246 25 14            [ 1]  349 	jrc	00159$
      008248 5F               [ 1]  350 	clrw	x
      008249 7B 14            [ 1]  351 	ld	a, (0x14, sp)
      00824B 97               [ 1]  352 	ld	xl, a
      00824C 1D 00 30         [ 2]  353 	subw	x, #0x0030
      00824F 89               [ 2]  354 	pushw	x
      008250 58               [ 2]  355 	sllw	x
      008251 58               [ 2]  356 	sllw	x
      008252 72 FB 01         [ 2]  357 	addw	x, (1, sp)
      008255 58               [ 2]  358 	sllw	x
      008256 5B 02            [ 2]  359 	addw	sp, #2
      008258 1F 12            [ 2]  360 	ldw	(0x12, sp), x
      00825A 20 03            [ 2]  361 	jra	00160$
      00825C                        362 00159$:
      00825C 5F               [ 1]  363 	clrw	x
      00825D 1F 12            [ 2]  364 	ldw	(0x12, sp), x
      00825F                        365 00160$:
      00825F 1E 0A            [ 2]  366 	ldw	x, (0x0a, sp)
      008261 E6 05            [ 1]  367 	ld	a, (0x5, x)
      008263 5F               [ 1]  368 	clrw	x
      008264 97               [ 1]  369 	ld	xl, a
      008265 1D 00 30         [ 2]  370 	subw	x, #0x0030
      008268 72 FB 12         [ 2]  371 	addw	x, (0x12, sp)
      00826B 1F 0F            [ 2]  372 	ldw	(0x0f, sp), x
      00826D                        373 00158$:
      00826D 0D 11            [ 1]  374 	tnz	(0x11, sp)
      00826F 27 08            [ 1]  375 	jreq	00161$
      008271 AE 00 63         [ 2]  376 	ldw	x, #0x0063
      008274 1F 0D            [ 2]  377 	ldw	(0x0d, sp), x
      008276 CC 83 A5         [ 2]  378 	jp	00162$
      008279                        379 00161$:
      008279 7B 09            [ 1]  380 	ld	a, (0x09, sp)
      00827B A1 4A            [ 1]  381 	cp	a, #0x4a
      00827D 26 06            [ 1]  382 	jrne	00437$
      00827F A6 01            [ 1]  383 	ld	a, #0x01
      008281 6B 0C            [ 1]  384 	ld	(0x0c, sp), a
      008283 20 02            [ 2]  385 	jra	00438$
      008285                        386 00437$:
      008285 0F 0C            [ 1]  387 	clr	(0x0c, sp)
      008287                        388 00438$:
      008287 1E 0A            [ 2]  389 	ldw	x, (0x0a, sp)
      008289 5C               [ 1]  390 	incw	x
      00828A 1F 26            [ 2]  391 	ldw	(0x26, sp), x
      00828C 1E 0A            [ 2]  392 	ldw	x, (0x0a, sp)
      00828E 5C               [ 1]  393 	incw	x
      00828F 5C               [ 1]  394 	incw	x
      008290 1F 24            [ 2]  395 	ldw	(0x24, sp), x
      008292 0D 0C            [ 1]  396 	tnz	(0x0c, sp)
      008294 27 13            [ 1]  397 	jreq	00163$
      008296 1E 26            [ 2]  398 	ldw	x, (0x26, sp)
      008298 F6               [ 1]  399 	ld	a, (x)
      008299 A1 61            [ 1]  400 	cp	a, #0x61
      00829B 26 0C            [ 1]  401 	jrne	00163$
      00829D 1E 24            [ 2]  402 	ldw	x, (0x24, sp)
      00829F F6               [ 1]  403 	ld	a, (x)
      0082A0 A1 6E            [ 1]  404 	cp	a, #0x6e
      0082A2 26 05            [ 1]  405 	jrne	00163$
      0082A4 5F               [ 1]  406 	clrw	x
      0082A5 5C               [ 1]  407 	incw	x
      0082A6 CC 83 A3         [ 2]  408 	jp	00164$
      0082A9                        409 00163$:
      0082A9 7B 09            [ 1]  410 	ld	a, (0x09, sp)
      0082AB A1 46            [ 1]  411 	cp	a, #0x46
      0082AD 26 08            [ 1]  412 	jrne	00171$
      0082AF AE 00 02         [ 2]  413 	ldw	x, #0x0002
      0082B2 1F 29            [ 2]  414 	ldw	(0x29, sp), x
      0082B4 CC 83 A1         [ 2]  415 	jp	00172$
      0082B7                        416 00171$:
      0082B7 7B 09            [ 1]  417 	ld	a, (0x09, sp)
      0082B9 A1 4D            [ 1]  418 	cp	a, #0x4d
      0082BB 26 06            [ 1]  419 	jrne	00450$
      0082BD A6 01            [ 1]  420 	ld	a, #0x01
      0082BF 6B 28            [ 1]  421 	ld	(0x28, sp), a
      0082C1 20 02            [ 2]  422 	jra	00451$
      0082C3                        423 00450$:
      0082C3 0F 28            [ 1]  424 	clr	(0x28, sp)
      0082C5                        425 00451$:
      0082C5 0D 28            [ 1]  426 	tnz	(0x28, sp)
      0082C7 27 14            [ 1]  427 	jreq	00173$
      0082C9 1E 26            [ 2]  428 	ldw	x, (0x26, sp)
      0082CB F6               [ 1]  429 	ld	a, (x)
      0082CC A1 61            [ 1]  430 	cp	a, #0x61
      0082CE 26 0D            [ 1]  431 	jrne	00173$
      0082D0 1E 24            [ 2]  432 	ldw	x, (0x24, sp)
      0082D2 F6               [ 1]  433 	ld	a, (x)
      0082D3 A1 72            [ 1]  434 	cp	a, #0x72
      0082D5 26 06            [ 1]  435 	jrne	00173$
      0082D7 AE 00 03         [ 2]  436 	ldw	x, #0x0003
      0082DA CC 83 9F         [ 2]  437 	jp	00174$
      0082DD                        438 00173$:
      0082DD 7B 09            [ 1]  439 	ld	a, (0x09, sp)
      0082DF A1 41            [ 1]  440 	cp	a, #0x41
      0082E1 26 06            [ 1]  441 	jrne	00460$
      0082E3 A6 01            [ 1]  442 	ld	a, #0x01
      0082E5 6B 2E            [ 1]  443 	ld	(0x2e, sp), a
      0082E7 20 02            [ 2]  444 	jra	00461$
      0082E9                        445 00460$:
      0082E9 0F 2E            [ 1]  446 	clr	(0x2e, sp)
      0082EB                        447 00461$:
      0082EB 0D 2E            [ 1]  448 	tnz	(0x2e, sp)
      0082ED 27 0F            [ 1]  449 	jreq	00181$
      0082EF 1E 26            [ 2]  450 	ldw	x, (0x26, sp)
      0082F1 F6               [ 1]  451 	ld	a, (x)
      0082F2 A1 70            [ 1]  452 	cp	a, #0x70
      0082F4 26 08            [ 1]  453 	jrne	00181$
      0082F6 AE 00 04         [ 2]  454 	ldw	x, #0x0004
      0082F9 1F 2C            [ 2]  455 	ldw	(0x2c, sp), x
      0082FB CC 83 9D         [ 2]  456 	jp	00182$
      0082FE                        457 00181$:
      0082FE 0D 28            [ 1]  458 	tnz	(0x28, sp)
      008300 27 16            [ 1]  459 	jreq	00186$
      008302 1E 26            [ 2]  460 	ldw	x, (0x26, sp)
      008304 F6               [ 1]  461 	ld	a, (x)
      008305 A1 61            [ 1]  462 	cp	a, #0x61
      008307 26 0F            [ 1]  463 	jrne	00186$
      008309 1E 24            [ 2]  464 	ldw	x, (0x24, sp)
      00830B F6               [ 1]  465 	ld	a, (x)
      00830C A1 79            [ 1]  466 	cp	a, #0x79
      00830E 26 08            [ 1]  467 	jrne	00186$
      008310 AE 00 05         [ 2]  468 	ldw	x, #0x0005
      008313 1F 31            [ 2]  469 	ldw	(0x31, sp), x
      008315 CC 83 99         [ 2]  470 	jp	00187$
      008318                        471 00186$:
      008318 0D 0C            [ 1]  472 	tnz	(0x0c, sp)
      00831A 27 13            [ 1]  473 	jreq	00194$
      00831C 1E 26            [ 2]  474 	ldw	x, (0x26, sp)
      00831E F6               [ 1]  475 	ld	a, (x)
      00831F A1 75            [ 1]  476 	cp	a, #0x75
      008321 26 0C            [ 1]  477 	jrne	00194$
      008323 1E 24            [ 2]  478 	ldw	x, (0x24, sp)
      008325 F6               [ 1]  479 	ld	a, (x)
      008326 A1 6E            [ 1]  480 	cp	a, #0x6e
      008328 26 05            [ 1]  481 	jrne	00194$
      00832A AE 00 06         [ 2]  482 	ldw	x, #0x0006
      00832D 20 68            [ 2]  483 	jra	00195$
      00832F                        484 00194$:
      00832F 0D 0C            [ 1]  485 	tnz	(0x0c, sp)
      008331 27 15            [ 1]  486 	jreq	00202$
      008333 1E 26            [ 2]  487 	ldw	x, (0x26, sp)
      008335 F6               [ 1]  488 	ld	a, (x)
      008336 A1 75            [ 1]  489 	cp	a, #0x75
      008338 26 0E            [ 1]  490 	jrne	00202$
      00833A 1E 24            [ 2]  491 	ldw	x, (0x24, sp)
      00833C F6               [ 1]  492 	ld	a, (x)
      00833D A1 6C            [ 1]  493 	cp	a, #0x6c
      00833F 26 07            [ 1]  494 	jrne	00202$
      008341 AE 00 07         [ 2]  495 	ldw	x, #0x0007
      008344 1F 2F            [ 2]  496 	ldw	(0x2f, sp), x
      008346 20 4D            [ 2]  497 	jra	00203$
      008348                        498 00202$:
      008348 0D 2E            [ 1]  499 	tnz	(0x2e, sp)
      00834A 27 0C            [ 1]  500 	jreq	00210$
      00834C 1E 26            [ 2]  501 	ldw	x, (0x26, sp)
      00834E F6               [ 1]  502 	ld	a, (x)
      00834F A1 75            [ 1]  503 	cp	a, #0x75
      008351 26 05            [ 1]  504 	jrne	00210$
      008353 AE 00 08         [ 2]  505 	ldw	x, #0x0008
      008356 20 3B            [ 2]  506 	jra	00211$
      008358                        507 00210$:
      008358 7B 09            [ 1]  508 	ld	a, (0x09, sp)
      00835A A1 53            [ 1]  509 	cp	a, #0x53
      00835C 26 07            [ 1]  510 	jrne	00215$
      00835E AE 00 09         [ 2]  511 	ldw	x, #0x0009
      008361 1F 39            [ 2]  512 	ldw	(0x39, sp), x
      008363 20 2C            [ 2]  513 	jra	00216$
      008365                        514 00215$:
      008365 7B 09            [ 1]  515 	ld	a, (0x09, sp)
      008367 A1 4F            [ 1]  516 	cp	a, #0x4f
      008369 26 05            [ 1]  517 	jrne	00217$
      00836B AE 00 0A         [ 2]  518 	ldw	x, #0x000a
      00836E 20 1F            [ 2]  519 	jra	00218$
      008370                        520 00217$:
      008370 7B 09            [ 1]  521 	ld	a, (0x09, sp)
      008372 A1 4E            [ 1]  522 	cp	a, #0x4e
      008374 26 07            [ 1]  523 	jrne	00219$
      008376 AE 00 0B         [ 2]  524 	ldw	x, #0x000b
      008379 1F 37            [ 2]  525 	ldw	(0x37, sp), x
      00837B 20 10            [ 2]  526 	jra	00220$
      00837D                        527 00219$:
      00837D 7B 09            [ 1]  528 	ld	a, (0x09, sp)
      00837F A1 44            [ 1]  529 	cp	a, #0x44
      008381 26 05            [ 1]  530 	jrne	00221$
      008383 AE 00 0C         [ 2]  531 	ldw	x, #0x000c
      008386 20 03            [ 2]  532 	jra	00222$
      008388                        533 00221$:
      008388 AE 00 63         [ 2]  534 	ldw	x, #0x0063
      00838B                        535 00222$:
      00838B 1F 37            [ 2]  536 	ldw	(0x37, sp), x
      00838D                        537 00220$:
      00838D 1E 37            [ 2]  538 	ldw	x, (0x37, sp)
      00838F                        539 00218$:
      00838F 1F 39            [ 2]  540 	ldw	(0x39, sp), x
      008391                        541 00216$:
      008391 1E 39            [ 2]  542 	ldw	x, (0x39, sp)
      008393                        543 00211$:
      008393 1F 2F            [ 2]  544 	ldw	(0x2f, sp), x
      008395                        545 00203$:
      008395 1E 2F            [ 2]  546 	ldw	x, (0x2f, sp)
      008397                        547 00195$:
      008397 1F 31            [ 2]  548 	ldw	(0x31, sp), x
      008399                        549 00187$:
      008399 16 31            [ 2]  550 	ldw	y, (0x31, sp)
      00839B 17 2C            [ 2]  551 	ldw	(0x2c, sp), y
      00839D                        552 00182$:
      00839D 1E 2C            [ 2]  553 	ldw	x, (0x2c, sp)
      00839F                        554 00174$:
      00839F 1F 29            [ 2]  555 	ldw	(0x29, sp), x
      0083A1                        556 00172$:
      0083A1 1E 29            [ 2]  557 	ldw	x, (0x29, sp)
      0083A3                        558 00164$:
      0083A3 1F 0D            [ 2]  559 	ldw	(0x0d, sp), x
      0083A5                        560 00162$:
      0083A5 0D 11            [ 1]  561 	tnz	(0x11, sp)
      0083A7 27 05            [ 1]  562 	jreq	00223$
      0083A9 AE 00 63         [ 2]  563 	ldw	x, #0x0063
      0083AC 20 52            [ 2]  564 	jra	00224$
      0083AE                        565 00223$:
      0083AE 1E 0A            [ 2]  566 	ldw	x, (0x0a, sp)
      0083B0 E6 07            [ 1]  567 	ld	a, (0x7, x)
      0083B2 5F               [ 1]  568 	clrw	x
      0083B3 97               [ 1]  569 	ld	xl, a
      0083B4 1D 00 30         [ 2]  570 	subw	x, #0x0030
      0083B7 89               [ 2]  571 	pushw	x
      0083B8 4B E8            [ 1]  572 	push	#0xe8
      0083BA 4B 03            [ 1]  573 	push	#0x03
      0083BC CD 8A CA         [ 4]  574 	call	__mulint
      0083BF 5B 04            [ 2]  575 	addw	sp, #4
      0083C1 1F 3D            [ 2]  576 	ldw	(0x3d, sp), x
      0083C3 1E 0A            [ 2]  577 	ldw	x, (0x0a, sp)
      0083C5 E6 08            [ 1]  578 	ld	a, (0x8, x)
      0083C7 5F               [ 1]  579 	clrw	x
      0083C8 97               [ 1]  580 	ld	xl, a
      0083C9 1D 00 30         [ 2]  581 	subw	x, #0x0030
      0083CC 89               [ 2]  582 	pushw	x
      0083CD 4B 64            [ 1]  583 	push	#0x64
      0083CF 4B 00            [ 1]  584 	push	#0x00
      0083D1 CD 8A CA         [ 4]  585 	call	__mulint
      0083D4 5B 04            [ 2]  586 	addw	sp, #4
      0083D6 72 FB 3D         [ 2]  587 	addw	x, (0x3d, sp)
      0083D9 1F 3B            [ 2]  588 	ldw	(0x3b, sp), x
      0083DB 1E 0A            [ 2]  589 	ldw	x, (0x0a, sp)
      0083DD E6 09            [ 1]  590 	ld	a, (0x9, x)
      0083DF 5F               [ 1]  591 	clrw	x
      0083E0 97               [ 1]  592 	ld	xl, a
      0083E1 1D 00 30         [ 2]  593 	subw	x, #0x0030
      0083E4 89               [ 2]  594 	pushw	x
      0083E5 58               [ 2]  595 	sllw	x
      0083E6 58               [ 2]  596 	sllw	x
      0083E7 72 FB 01         [ 2]  597 	addw	x, (1, sp)
      0083EA 58               [ 2]  598 	sllw	x
      0083EB 5B 02            [ 2]  599 	addw	sp, #2
      0083ED 72 FB 3B         [ 2]  600 	addw	x, (0x3b, sp)
      0083F0 1F 41            [ 2]  601 	ldw	(0x41, sp), x
      0083F2 1E 0A            [ 2]  602 	ldw	x, (0x0a, sp)
      0083F4 E6 0A            [ 1]  603 	ld	a, (0xa, x)
      0083F6 5F               [ 1]  604 	clrw	x
      0083F7 97               [ 1]  605 	ld	xl, a
      0083F8 1D 00 30         [ 2]  606 	subw	x, #0x0030
      0083FB 72 FB 41         [ 2]  607 	addw	x, (0x41, sp)
      0083FE 1F 3F            [ 2]  608 	ldw	(0x3f, sp), x
      008400                        609 00224$:
      008400 1D 07 D0         [ 2]  610 	subw	x, #0x07d0
      008403 1F 49            [ 2]  611 	ldw	(0x49, sp), x
      008405 AE 87 1A         [ 2]  612 	ldw	x, #___str_0+0
      008408 1F 47            [ 2]  613 	ldw	(0x47, sp), x
      00840A AE 00 07         [ 2]  614 	ldw	x, #_version+0
      00840D 1F 4D            [ 2]  615 	ldw	(0x4d, sp), x
      00840F 90 93            [ 1]  616 	ldw	y, x
      008411 1E 19            [ 2]  617 	ldw	x, (0x19, sp)
      008413 89               [ 2]  618 	pushw	x
      008414 1E 17            [ 2]  619 	ldw	x, (0x17, sp)
      008416 89               [ 2]  620 	pushw	x
      008417 1E 13            [ 2]  621 	ldw	x, (0x13, sp)
      008419 89               [ 2]  622 	pushw	x
      00841A 1E 13            [ 2]  623 	ldw	x, (0x13, sp)
      00841C 89               [ 2]  624 	pushw	x
      00841D 1E 51            [ 2]  625 	ldw	x, (0x51, sp)
      00841F 89               [ 2]  626 	pushw	x
      008420 1E 51            [ 2]  627 	ldw	x, (0x51, sp)
      008422 89               [ 2]  628 	pushw	x
      008423 90 89            [ 2]  629 	pushw	y
      008425 CD 8B 11         [ 4]  630 	call	_sprintf
      008428 5B 0E            [ 2]  631 	addw	sp, #14
                                    632 ;	../src/main.c: 103: disableInterrupts();
      00842A 9B               [ 1]  633 	sim
                                    634 ;	../src/main.c: 104: InitialiseSystemClock();
      00842B CD 81 13         [ 4]  635 	call	_InitialiseSystemClock
                                    636 ;	../src/main.c: 106: GPIOD->CR2 &= (uint8_t)(~(GPIO_PIN_4));
      00842E 72 19 50 13      [ 1]  637 	bres	20499, #4
                                    638 ;	../src/main.c: 107: GPIOD->ODR &= (uint8_t)(~(GPIO_PIN_4));
      008432 72 19 50 0F      [ 1]  639 	bres	20495, #4
                                    640 ;	../src/main.c: 108: GPIOD->DDR |= (uint8_t)GPIO_PIN_4;
      008436 72 18 50 11      [ 1]  641 	bset	20497, #4
                                    642 ;	../src/main.c: 109: GPIOD->CR1 |= (uint8_t)GPIO_PIN_4;
      00843A 72 18 50 12      [ 1]  643 	bset	20498, #4
                                    644 ;	../src/main.c: 110: GPIOD->CR2 |= (uint8_t)GPIO_PIN_4;
      00843E 72 18 50 13      [ 1]  645 	bset	20499, #4
                                    646 ;	../src/main.c: 114: GPIOD->DDR &= ~(GPIO_PIN_2 | GPIO_PIN_1);  // input mode
      008442 C6 50 11         [ 1]  647 	ld	a, 0x5011
      008445 A4 F9            [ 1]  648 	and	a, #0xf9
      008447 C7 50 11         [ 1]  649 	ld	0x5011, a
                                    650 ;	../src/main.c: 115: GPIOD->CR1 |= (GPIO_PIN_1 | GPIO_PIN_2);  // pull-ups
      00844A C6 50 12         [ 1]  651 	ld	a, 0x5012
      00844D AA 06            [ 1]  652 	or	a, #0x06
      00844F C7 50 12         [ 1]  653 	ld	0x5012, a
                                    654 ;	../src/main.c: 116: GPIOD->CR2 &= ~(GPIO_PIN_2 | GPIO_PIN_1);  // no interrupts
      008452 C6 50 13         [ 1]  655 	ld	a, 0x5013
      008455 A4 F9            [ 1]  656 	and	a, #0xf9
      008457 C7 50 13         [ 1]  657 	ld	0x5013, a
                                    658 ;	../src/main.c: 117: GPIOC->DDR &= ~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7); //  input mode
      00845A C6 50 0C         [ 1]  659 	ld	a, 0x500c
      00845D A4 0F            [ 1]  660 	and	a, #0x0f
      00845F C7 50 0C         [ 1]  661 	ld	0x500c, a
                                    662 ;	../src/main.c: 118: GPIOC->CR1 |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);  // no interrupts
      008462 C6 50 0D         [ 1]  663 	ld	a, 0x500d
      008465 AA F0            [ 1]  664 	or	a, #0xf0
      008467 C7 50 0D         [ 1]  665 	ld	0x500d, a
                                    666 ;	../src/main.c: 119: GPIOC->CR2 &= ~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);  // no interrupts
      00846A C6 50 0E         [ 1]  667 	ld	a, 0x500e
      00846D A4 0F            [ 1]  668 	and	a, #0x0f
      00846F C7 50 0E         [ 1]  669 	ld	0x500e, a
                                    670 ;	../src/main.c: 122: rs485xmit_off();
      008472 72 19 50 0F      [ 1]  671 	bres	20495, #4
                                    672 ;	../src/main.c: 123: CFG->GCR |= 1; // disable SWIM
      008476 C6 7F 60         [ 1]  673 	ld	a, 0x7f60
      008479 5F               [ 1]  674 	clrw	x
      00847A 97               [ 1]  675 	ld	xl, a
      00847B 54               [ 2]  676 	srlw	x
      00847C 99               [ 1]  677 	scf
      00847D 59               [ 2]  678 	rlcw	x
      00847E 9F               [ 1]  679 	ld	a, xl
      00847F C7 7F 60         [ 1]  680 	ld	0x7f60, a
                                    681 ;	../src/main.c: 126: TIM4->PSCR = 7;   // prescaler
      008482 35 07 53 47      [ 1]  682 	mov	0x5347+0, #0x07
                                    683 ;	../src/main.c: 127: TIM4->ARR = 125;  // auto reload register
      008486 35 7D 53 48      [ 1]  684 	mov	0x5348+0, #0x7d
                                    685 ;	../src/main.c: 129: TIM4->IER = TIM4_IER_UIE;
      00848A 35 01 53 43      [ 1]  686 	mov	0x5343+0, #0x01
                                    687 ;	../src/main.c: 131: TIM4->CR1 = TIM4_CR1_ARPE | TIM4_CR1_URS | TIM4_CR1_CEN;
      00848E 35 85 53 40      [ 1]  688 	mov	0x5340+0, #0x85
                                    689 ;	../src/main.c: 133: Global_time = 0L;
      008492 5F               [ 1]  690 	clrw	x
      008493 CF 00 03         [ 2]  691 	ldw	_Global_time+2, x
      008496 CF 00 01         [ 2]  692 	ldw	_Global_time+0, x
                                    693 ;	../src/main.c: 134: uart_init();		// initialize the uart functions - 9600 8-N-1 through RS485
      008499 CD 87 79         [ 4]  694 	call	_uart_init
                                    695 ;	../src/main.c: 136: enableInterrupts();
      00849C 9A               [ 1]  696 	rim
                                    697 ;	../src/main.c: 138: address = 'S';		// this devices id character
      00849D 35 53 00 06      [ 1]  698 	mov	_address+0, #0x53
                                    699 ;	../src/main.c: 140: ADC_init();			// initialize the analog read function
      0084A1 CD 81 91         [ 4]  700 	call	_ADC_init
                                    701 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0084A4 5F               [ 1]  702 	clrw	x
      0084A5 1F 65            [ 2]  703 	ldw	(0x65, sp), x
      0084A7 1F 63            [ 2]  704 	ldw	(0x63, sp), x
      0084A9                        705 00135$:
      0084A9 1E 65            [ 2]  706 	ldw	x, (0x65, sp)
      0084AB A3 D7 00         [ 2]  707 	cpw	x, #0xd700
      0084AE 7B 64            [ 1]  708 	ld	a, (0x64, sp)
      0084B0 A2 0A            [ 1]  709 	sbc	a, #0x0a
      0084B2 7B 63            [ 1]  710 	ld	a, (0x63, sp)
      0084B4 A2 00            [ 1]  711 	sbc	a, #0x00
      0084B6 24 17            [ 1]  712 	jrnc	00125$
                                    713 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 14: __asm__("nop");
      0084B8 9D               [ 1]  714 	nop
                                    715 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0084B9 16 65            [ 2]  716 	ldw	y, (0x65, sp)
      0084BB 72 A9 00 01      [ 2]  717 	addw	y, #0x0001
      0084BF 7B 64            [ 1]  718 	ld	a, (0x64, sp)
      0084C1 A9 00            [ 1]  719 	adc	a, #0x00
      0084C3 97               [ 1]  720 	ld	xl, a
      0084C4 7B 63            [ 1]  721 	ld	a, (0x63, sp)
      0084C6 A9 00            [ 1]  722 	adc	a, #0x00
      0084C8 95               [ 1]  723 	ld	xh, a
      0084C9 17 65            [ 2]  724 	ldw	(0x65, sp), y
      0084CB 1F 63            [ 2]  725 	ldw	(0x63, sp), x
      0084CD 20 DA            [ 2]  726 	jra	00135$
                                    727 ;	../src/main.c: 141: delay_ms(800);
      0084CF                        728 00125$:
                                    729 ;	../src/main.c: 142: rs485xmit_on();	// turn the RS485 chips transmitter on
      0084CF C6 50 0F         [ 1]  730 	ld	a, 0x500f
      0084D2 AA 10            [ 1]  731 	or	a, #0x10
      0084D4 C7 50 0F         [ 1]  732 	ld	0x500f, a
                                    733 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0084D7 5F               [ 1]  734 	clrw	x
      0084D8 1F 61            [ 2]  735 	ldw	(0x61, sp), x
      0084DA 1F 5F            [ 2]  736 	ldw	(0x5f, sp), x
      0084DC                        737 00138$:
      0084DC 1E 61            [ 2]  738 	ldw	x, (0x61, sp)
      0084DE A3 68 10         [ 2]  739 	cpw	x, #0x6810
      0084E1 7B 60            [ 1]  740 	ld	a, (0x60, sp)
      0084E3 A2 00            [ 1]  741 	sbc	a, #0x00
      0084E5 7B 5F            [ 1]  742 	ld	a, (0x5f, sp)
      0084E7 A2 00            [ 1]  743 	sbc	a, #0x00
      0084E9 24 17            [ 1]  744 	jrnc	00127$
                                    745 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 14: __asm__("nop");
      0084EB 9D               [ 1]  746 	nop
                                    747 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0084EC 16 61            [ 2]  748 	ldw	y, (0x61, sp)
      0084EE 72 A9 00 01      [ 2]  749 	addw	y, #0x0001
      0084F2 7B 60            [ 1]  750 	ld	a, (0x60, sp)
      0084F4 A9 00            [ 1]  751 	adc	a, #0x00
      0084F6 97               [ 1]  752 	ld	xl, a
      0084F7 7B 5F            [ 1]  753 	ld	a, (0x5f, sp)
      0084F9 A9 00            [ 1]  754 	adc	a, #0x00
      0084FB 95               [ 1]  755 	ld	xh, a
      0084FC 17 61            [ 2]  756 	ldw	(0x61, sp), y
      0084FE 1F 5F            [ 2]  757 	ldw	(0x5f, sp), x
      008500 20 DA            [ 2]  758 	jra	00138$
                                    759 ;	../src/main.c: 143: delay_ms(30);
      008502                        760 00127$:
                                    761 ;	../src/main.c: 144: printf("%c:Running:%s:%02x\r\n",address,version,address);
      008502 5F               [ 1]  762 	clrw	x
      008503 C6 00 06         [ 1]  763 	ld	a, _address+0
      008506 97               [ 1]  764 	ld	xl, a
      008507 16 4D            [ 2]  765 	ldw	y, (0x4d, sp)
      008509 17 4B            [ 2]  766 	ldw	(0x4b, sp), y
      00850B 90 AE 87 45      [ 2]  767 	ldw	y, #___str_3+0
      00850F 89               [ 2]  768 	pushw	x
      008510 7B 4E            [ 1]  769 	ld	a, (0x4e, sp)
      008512 88               [ 1]  770 	push	a
      008513 7B 4E            [ 1]  771 	ld	a, (0x4e, sp)
      008515 88               [ 1]  772 	push	a
      008516 89               [ 2]  773 	pushw	x
      008517 90 89            [ 2]  774 	pushw	y
      008519 CD 8B 7B         [ 4]  775 	call	_printf
      00851C 5B 08            [ 2]  776 	addw	sp, #8
                                    777 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      00851E 5F               [ 1]  778 	clrw	x
      00851F 1F 5D            [ 2]  779 	ldw	(0x5d, sp), x
      008521 1F 5B            [ 2]  780 	ldw	(0x5b, sp), x
      008523                        781 00141$:
      008523 1E 5D            [ 2]  782 	ldw	x, (0x5d, sp)
      008525 A3 22 B0         [ 2]  783 	cpw	x, #0x22b0
      008528 7B 5C            [ 1]  784 	ld	a, (0x5c, sp)
      00852A A2 00            [ 1]  785 	sbc	a, #0x00
      00852C 7B 5B            [ 1]  786 	ld	a, (0x5b, sp)
      00852E A2 00            [ 1]  787 	sbc	a, #0x00
      008530 24 17            [ 1]  788 	jrnc	00129$
                                    789 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 14: __asm__("nop");
      008532 9D               [ 1]  790 	nop
                                    791 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      008533 16 5D            [ 2]  792 	ldw	y, (0x5d, sp)
      008535 72 A9 00 01      [ 2]  793 	addw	y, #0x0001
      008539 7B 5C            [ 1]  794 	ld	a, (0x5c, sp)
      00853B A9 00            [ 1]  795 	adc	a, #0x00
      00853D 97               [ 1]  796 	ld	xl, a
      00853E 7B 5B            [ 1]  797 	ld	a, (0x5b, sp)
      008540 A9 00            [ 1]  798 	adc	a, #0x00
      008542 95               [ 1]  799 	ld	xh, a
      008543 17 5D            [ 2]  800 	ldw	(0x5d, sp), y
      008545 1F 5B            [ 2]  801 	ldw	(0x5b, sp), x
      008547 20 DA            [ 2]  802 	jra	00141$
                                    803 ;	../src/main.c: 145: delay_ms(10);
      008549                        804 00129$:
                                    805 ;	../src/main.c: 146: rs485xmit_off(); // turn the transmitter back off
      008549 C6 50 0F         [ 1]  806 	ld	a, 0x500f
      00854C A4 EF            [ 1]  807 	and	a, #0xef
      00854E C7 50 0F         [ 1]  808 	ld	0x500f, a
                                    809 ;	../src/main.c: 147: InitialiseIWDG();
      008551 CD 81 51         [ 4]  810 	call	_InitialiseIWDG
                                    811 ;	../src/main.c: 148: reset_watchdog();  // reset the watchdog timer
      008554 35 AA 50 E0      [ 1]  812 	mov	0x50e0+0, #0xaa
                                    813 ;	../src/main.c: 151: do{
      008558                        814 00121$:
                                    815 ;	../src/main.c: 152: reset_watchdog();  // reset the watchdog timer
      008558 35 AA 50 E0      [ 1]  816 	mov	0x50e0+0, #0xaa
                                    817 ;	../src/main.c: 153: if(UART_read_byte(&rb)){ // buffer isn't empty
      00855C 96               [ 1]  818 	ldw	x, sp
      00855D 5C               [ 1]  819 	incw	x
      00855E 89               [ 2]  820 	pushw	x
      00855F CD 87 B0         [ 4]  821 	call	_UART_read_byte
      008562 5B 02            [ 2]  822 	addw	sp, #2
      008564 4D               [ 1]  823 	tnz	a
      008565 26 03            [ 1]  824 	jrne	00507$
      008567 CC 86 E1         [ 2]  825 	jp	00117$
      00856A                        826 00507$:
                                    827 ;	../src/main.c: 154: switch(rb){
      00856A 7B 01            [ 1]  828 	ld	a, (0x01, sp)
      00856C A1 1B            [ 1]  829 	cp	a, #0x1b
      00856E 26 11            [ 1]  830 	jrne	00102$
                                    831 ;	../src/main.c: 156: esc = 1;
      008570 35 01 00 05      [ 1]  832 	mov	_esc+0, #0x01
                                    833 ;	../src/main.c: 157: esc_time = Global_time;	// only wait two seconds for the next character after the escape
      008574 CE 00 03         [ 2]  834 	ldw	x, _Global_time+2
      008577 1F 6C            [ 2]  835 	ldw	(0x6c, sp), x
      008579 CE 00 01         [ 2]  836 	ldw	x, _Global_time+0
      00857C 1F 6A            [ 2]  837 	ldw	(0x6a, sp), x
                                    838 ;	../src/main.c: 158: break;
      00857E CC 86 E1         [ 2]  839 	jp	00117$
                                    840 ;	../src/main.c: 159: default:
      008581                        841 00102$:
                                    842 ;	../src/main.c: 160: if (rb == address && esc)  // address must match the switches read by mcp23017
      008581 7B 01            [ 1]  843 	ld	a, (0x01, sp)
      008583 C1 00 06         [ 1]  844 	cp	a, _address+0
      008586 27 03            [ 1]  845 	jreq	00513$
      008588 CC 86 DD         [ 2]  846 	jp	00113$
      00858B                        847 00513$:
      00858B 72 5D 00 05      [ 1]  848 	tnz	_esc+0
      00858F 26 03            [ 1]  849 	jrne	00514$
      008591 CC 86 DD         [ 2]  850 	jp	00113$
      008594                        851 00514$:
                                    852 ;	../src/main.c: 162: Global_time = 0L;   // when was the last time we were called?
      008594 5F               [ 1]  853 	clrw	x
      008595 CF 00 03         [ 2]  854 	ldw	_Global_time+2, x
      008598 CF 00 01         [ 2]  855 	ldw	_Global_time+0, x
                                    856 ;	../src/main.c: 163: voltage = ADC_read();	// get the analog value from the pressure sensor
      00859B CD 81 66         [ 4]  857 	call	_ADC_read
                                    858 ;	../src/main.c: 164: if (voltage < VOLTAGE_OFFSET)		// the sensor range is from 0.5 volts to 4.5 volts
      00859E 1F 67            [ 2]  859 	ldw	(0x67, sp), x
      0085A0 A3 00 78         [ 2]  860 	cpw	x, #0x0078
      0085A3 24 05            [ 1]  861 	jrnc	00104$
                                    862 ;	../src/main.c: 165: voltage = VOLTAGE_OFFSET;
      0085A5 AE 00 78         [ 2]  863 	ldw	x, #0x0078
      0085A8 1F 67            [ 2]  864 	ldw	(0x67, sp), x
      0085AA                        865 00104$:
                                    866 ;	../src/main.c: 166: PSI = (uint16_t)((voltage - VOLTAGE_OFFSET) * 1.15);  // formula to calculate PSI
      0085AA 1E 67            [ 2]  867 	ldw	x, (0x67, sp)
      0085AC 1D 00 78         [ 2]  868 	subw	x, #0x0078
      0085AF 89               [ 2]  869 	pushw	x
      0085B0 CD 8B 31         [ 4]  870 	call	___uint2fs
      0085B3 5B 02            [ 2]  871 	addw	sp, #2
      0085B5 89               [ 2]  872 	pushw	x
      0085B6 90 89            [ 2]  873 	pushw	y
      0085B8 4B 33            [ 1]  874 	push	#0x33
      0085BA 4B 33            [ 1]  875 	push	#0x33
      0085BC 4B 93            [ 1]  876 	push	#0x93
      0085BE 4B 3F            [ 1]  877 	push	#0x3f
      0085C0 CD 88 04         [ 4]  878 	call	___fsmul
      0085C3 5B 08            [ 2]  879 	addw	sp, #8
      0085C5 89               [ 2]  880 	pushw	x
      0085C6 90 89            [ 2]  881 	pushw	y
      0085C8 CD 8B 3D         [ 4]  882 	call	___fs2uint
      0085CB 5B 04            [ 2]  883 	addw	sp, #4
                                    884 ;	../src/main.c: 168: psi1 = (uint16_t)(PSI / 100);
      0085CD 1F 72            [ 2]  885 	ldw	(0x72, sp), x
      0085CF 90 AE 00 64      [ 2]  886 	ldw	y, #0x0064
      0085D3 65               [ 2]  887 	divw	x, y
                                    888 ;	../src/main.c: 169: psi2 = (uint16_t)(PSI - (psi1 * 100));
      0085D4 1F 70            [ 2]  889 	ldw	(0x70, sp), x
      0085D6 89               [ 2]  890 	pushw	x
      0085D7 4B 64            [ 1]  891 	push	#0x64
      0085D9 4B 00            [ 1]  892 	push	#0x00
      0085DB CD 8A CA         [ 4]  893 	call	__mulint
      0085DE 5B 04            [ 2]  894 	addw	sp, #4
      0085E0 1F 51            [ 2]  895 	ldw	(0x51, sp), x
      0085E2 1E 72            [ 2]  896 	ldw	x, (0x72, sp)
      0085E4 72 F0 51         [ 2]  897 	subw	x, (0x51, sp)
      0085E7 1F 6E            [ 2]  898 	ldw	(0x6e, sp), x
                                    899 ;	../src/main.c: 170: strcpy(s,"0000aa");
      0085E9 90 AE 87 5A      [ 2]  900 	ldw	y, #___str_4+0
      0085ED 96               [ 1]  901 	ldw	x, sp
      0085EE 5C               [ 1]  902 	incw	x
      0085EF 5C               [ 1]  903 	incw	x
      0085F0 1F 4F            [ 2]  904 	ldw	(0x4f, sp), x
      0085F2 90 89            [ 2]  905 	pushw	y
      0085F4 89               [ 2]  906 	pushw	x
      0085F5 CD 87 E3         [ 4]  907 	call	_strcpy
      0085F8 5B 04            [ 2]  908 	addw	sp, #4
                                    909 ;	../src/main.c: 171: for (i = 0; i < 4; i++)		// read through the on-off inputs
      0085FA 0F 69            [ 1]  910 	clr	(0x69, sp)
      0085FC                        911 00143$:
                                    912 ;	../src/main.c: 173: if (!(GPIOC->IDR & (GPIO_PIN_4 << i)))
      0085FC C6 50 0B         [ 1]  913 	ld	a, 0x500b
      0085FF 88               [ 1]  914 	push	a
      008600 AE 00 10         [ 2]  915 	ldw	x, #0x0010
      008603 7B 6A            [ 1]  916 	ld	a, (0x6a, sp)
      008605 27 04            [ 1]  917 	jreq	00517$
      008607                        918 00516$:
      008607 58               [ 2]  919 	sllw	x
      008608 4A               [ 1]  920 	dec	a
      008609 26 FC            [ 1]  921 	jrne	00516$
      00860B                        922 00517$:
      00860B 84               [ 1]  923 	pop	a
      00860C 6B 46            [ 1]  924 	ld	(0x46, sp), a
      00860E 0F 45            [ 1]  925 	clr	(0x45, sp)
      008610 9F               [ 1]  926 	ld	a, xl
      008611 14 46            [ 1]  927 	and	a, (0x46, sp)
      008613 6B 44            [ 1]  928 	ld	(0x44, sp), a
      008615 9E               [ 1]  929 	ld	a, xh
      008616 14 45            [ 1]  930 	and	a, (0x45, sp)
      008618 6B 43            [ 1]  931 	ld	(0x43, sp), a
      00861A 1E 43            [ 2]  932 	ldw	x, (0x43, sp)
      00861C 26 0E            [ 1]  933 	jrne	00144$
                                    934 ;	../src/main.c: 174: s[i] = '1';		// if the gpio is low, that means on
      00861E 7B 69            [ 1]  935 	ld	a, (0x69, sp)
      008620 1B 50            [ 1]  936 	add	a, (0x50, sp)
      008622 88               [ 1]  937 	push	a
      008623 4F               [ 1]  938 	clr	a
      008624 19 50            [ 1]  939 	adc	a, (0x50, sp)
      008626 95               [ 1]  940 	ld	xh, a
      008627 84               [ 1]  941 	pop	a
      008628 97               [ 1]  942 	ld	xl, a
      008629 A6 31            [ 1]  943 	ld	a, #0x31
      00862B F7               [ 1]  944 	ld	(x), a
      00862C                        945 00144$:
                                    946 ;	../src/main.c: 171: for (i = 0; i < 4; i++)		// read through the on-off inputs
      00862C 0C 69            [ 1]  947 	inc	(0x69, sp)
      00862E 7B 69            [ 1]  948 	ld	a, (0x69, sp)
      008630 A1 04            [ 1]  949 	cp	a, #0x04
      008632 25 C8            [ 1]  950 	jrc	00143$
                                    951 ;	../src/main.c: 176: if (!(GPIOD->IDR & GPIO_PIN_1))	// these are for the air conditioners
      008634 C6 50 10         [ 1]  952 	ld	a, 0x5010
      008637 A5 02            [ 1]  953 	bcp	a, #0x02
      008639 26 08            [ 1]  954 	jrne	00109$
                                    955 ;	../src/main.c: 177: s[4] = 'A';
      00863B 1E 4F            [ 2]  956 	ldw	x, (0x4f, sp)
      00863D 1C 00 04         [ 2]  957 	addw	x, #0x0004
      008640 A6 41            [ 1]  958 	ld	a, #0x41
      008642 F7               [ 1]  959 	ld	(x), a
      008643                        960 00109$:
                                    961 ;	../src/main.c: 178: if (!(GPIOD->IDR & GPIO_PIN_2))
      008643 C6 50 10         [ 1]  962 	ld	a, 0x5010
      008646 A5 04            [ 1]  963 	bcp	a, #0x04
      008648 26 06            [ 1]  964 	jrne	00111$
                                    965 ;	../src/main.c: 179: s[5] = 'A';
      00864A 1E 4F            [ 2]  966 	ldw	x, (0x4f, sp)
      00864C A6 41            [ 1]  967 	ld	a, #0x41
      00864E E7 05            [ 1]  968 	ld	(0x0005, x), a
      008650                        969 00111$:
                                    970 ;	../src/main.c: 180: s[6] = 0;		// terminate the string
      008650 1E 4F            [ 2]  971 	ldw	x, (0x4f, sp)
      008652 1C 00 06         [ 2]  972 	addw	x, #0x0006
      008655 7F               [ 1]  973 	clr	(x)
                                    974 ;	../src/main.c: 181: rs485xmit_on();	// turn the RS485 chips transmitter on
      008656 C6 50 0F         [ 1]  975 	ld	a, 0x500f
      008659 AA 10            [ 1]  976 	or	a, #0x10
      00865B C7 50 0F         [ 1]  977 	ld	0x500f, a
                                    978 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      00865E 5F               [ 1]  979 	clrw	x
      00865F 1F 59            [ 2]  980 	ldw	(0x59, sp), x
      008661 1F 57            [ 2]  981 	ldw	(0x57, sp), x
      008663                        982 00146$:
      008663 1E 59            [ 2]  983 	ldw	x, (0x59, sp)
      008665 A3 68 10         [ 2]  984 	cpw	x, #0x6810
      008668 7B 58            [ 1]  985 	ld	a, (0x58, sp)
      00866A A2 00            [ 1]  986 	sbc	a, #0x00
      00866C 7B 57            [ 1]  987 	ld	a, (0x57, sp)
      00866E A2 00            [ 1]  988 	sbc	a, #0x00
      008670 24 17            [ 1]  989 	jrnc	00131$
                                    990 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 14: __asm__("nop");
      008672 9D               [ 1]  991 	nop
                                    992 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      008673 16 59            [ 2]  993 	ldw	y, (0x59, sp)
      008675 72 A9 00 01      [ 2]  994 	addw	y, #0x0001
      008679 7B 58            [ 1]  995 	ld	a, (0x58, sp)
      00867B A9 00            [ 1]  996 	adc	a, #0x00
      00867D 97               [ 1]  997 	ld	xl, a
      00867E 7B 57            [ 1]  998 	ld	a, (0x57, sp)
      008680 A9 00            [ 1]  999 	adc	a, #0x00
      008682 95               [ 1] 1000 	ld	xh, a
      008683 17 59            [ 2] 1001 	ldw	(0x59, sp), y
      008685 1F 57            [ 2] 1002 	ldw	(0x57, sp), x
      008687 20 DA            [ 2] 1003 	jra	00146$
                                   1004 ;	../src/main.c: 182: delay_ms(30);	// make sure transmitter has time to turn on
      008689                       1005 00131$:
                                   1006 ;	../src/main.c: 183: printf("%c:%02d.%02d:%05d:%s:\r\n", address,psi1,psi2,voltage,s); // S:23.56:89012:0101Aa:
      008689 1E 4F            [ 2] 1007 	ldw	x, (0x4f, sp)
      00868B C6 00 06         [ 1] 1008 	ld	a, _address+0
      00868E 6B 23            [ 1] 1009 	ld	(0x23, sp), a
      008690 0F 22            [ 1] 1010 	clr	(0x22, sp)
      008692 90 AE 87 61      [ 2] 1011 	ldw	y, #___str_5+0
      008696 89               [ 2] 1012 	pushw	x
      008697 1E 69            [ 2] 1013 	ldw	x, (0x69, sp)
      008699 89               [ 2] 1014 	pushw	x
      00869A 1E 72            [ 2] 1015 	ldw	x, (0x72, sp)
      00869C 89               [ 2] 1016 	pushw	x
      00869D 1E 76            [ 2] 1017 	ldw	x, (0x76, sp)
      00869F 89               [ 2] 1018 	pushw	x
      0086A0 1E 2A            [ 2] 1019 	ldw	x, (0x2a, sp)
      0086A2 89               [ 2] 1020 	pushw	x
      0086A3 90 89            [ 2] 1021 	pushw	y
      0086A5 CD 8B 7B         [ 4] 1022 	call	_printf
      0086A8 5B 0C            [ 2] 1023 	addw	sp, #12
                                   1024 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0086AA 5F               [ 1] 1025 	clrw	x
      0086AB 1F 55            [ 2] 1026 	ldw	(0x55, sp), x
      0086AD 1F 53            [ 2] 1027 	ldw	(0x53, sp), x
      0086AF                       1028 00149$:
      0086AF 1E 55            [ 2] 1029 	ldw	x, (0x55, sp)
      0086B1 A3 22 B0         [ 2] 1030 	cpw	x, #0x22b0
      0086B4 7B 54            [ 1] 1031 	ld	a, (0x54, sp)
      0086B6 A2 00            [ 1] 1032 	sbc	a, #0x00
      0086B8 7B 53            [ 1] 1033 	ld	a, (0x53, sp)
      0086BA A2 00            [ 1] 1034 	sbc	a, #0x00
      0086BC 24 17            [ 1] 1035 	jrnc	00133$
                                   1036 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 14: __asm__("nop");
      0086BE 9D               [ 1] 1037 	nop
                                   1038 ;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
      0086BF 16 55            [ 2] 1039 	ldw	y, (0x55, sp)
      0086C1 72 A9 00 01      [ 2] 1040 	addw	y, #0x0001
      0086C5 7B 54            [ 1] 1041 	ld	a, (0x54, sp)
      0086C7 A9 00            [ 1] 1042 	adc	a, #0x00
      0086C9 97               [ 1] 1043 	ld	xl, a
      0086CA 7B 53            [ 1] 1044 	ld	a, (0x53, sp)
      0086CC A9 00            [ 1] 1045 	adc	a, #0x00
      0086CE 95               [ 1] 1046 	ld	xh, a
      0086CF 17 55            [ 2] 1047 	ldw	(0x55, sp), y
      0086D1 1F 53            [ 2] 1048 	ldw	(0x53, sp), x
      0086D3 20 DA            [ 2] 1049 	jra	00149$
                                   1050 ;	../src/main.c: 184: delay_ms(10);	// give it time to transmit before turning transmitter off
      0086D5                       1051 00133$:
                                   1052 ;	../src/main.c: 185: rs485xmit_off(); // turn the transmitter back off
      0086D5 C6 50 0F         [ 1] 1053 	ld	a, 0x500f
      0086D8 A4 EF            [ 1] 1054 	and	a, #0xef
      0086DA C7 50 0F         [ 1] 1055 	ld	0x500f, a
      0086DD                       1056 00113$:
                                   1057 ;	../src/main.c: 187: esc = 0;	// reset the flag for the escape character
      0086DD 72 5F 00 05      [ 1] 1058 	clr	_esc+0
                                   1059 ;	../src/main.c: 188: }
      0086E1                       1060 00117$:
                                   1061 ;	../src/main.c: 190: if (esc && (Global_time - esc_time > 2000))  // give it 2 seconds to send the id character
      0086E1 72 5D 00 05      [ 1] 1062 	tnz	_esc+0
      0086E5 26 03            [ 1] 1063 	jrne	00524$
      0086E7 CC 85 58         [ 2] 1064 	jp	00121$
      0086EA                       1065 00524$:
      0086EA CE 00 03         [ 2] 1066 	ldw	x, _Global_time+2
      0086ED 72 F0 6C         [ 2] 1067 	subw	x, (0x6c, sp)
      0086F0 1F 20            [ 2] 1068 	ldw	(0x20, sp), x
      0086F2 C6 00 02         [ 1] 1069 	ld	a, _Global_time+1
      0086F5 12 6B            [ 1] 1070 	sbc	a, (0x6b, sp)
      0086F7 6B 1F            [ 1] 1071 	ld	(0x1f, sp), a
      0086F9 C6 00 01         [ 1] 1072 	ld	a, _Global_time+0
      0086FC 12 6A            [ 1] 1073 	sbc	a, (0x6a, sp)
      0086FE 6B 1E            [ 1] 1074 	ld	(0x1e, sp), a
      008700 AE 07 D0         [ 2] 1075 	ldw	x, #0x07d0
      008703 13 20            [ 2] 1076 	cpw	x, (0x20, sp)
      008705 4F               [ 1] 1077 	clr	a
      008706 12 1F            [ 1] 1078 	sbc	a, (0x1f, sp)
      008708 4F               [ 1] 1079 	clr	a
      008709 12 1E            [ 1] 1080 	sbc	a, (0x1e, sp)
      00870B 25 03            [ 1] 1081 	jrc	00525$
      00870D CC 85 58         [ 2] 1082 	jp	00121$
      008710                       1083 00525$:
                                   1084 ;	../src/main.c: 191: esc = 0;  // reset the esc flag, since it should have been followed by the id right away
      008710 72 5F 00 05      [ 1] 1085 	clr	_esc+0
                                   1086 ;	../src/main.c: 192: }while(1);
      008714 CC 85 58         [ 2] 1087 	jp	00121$
                                   1088 ;	../src/main.c: 193: }
      008717 5B 73            [ 2] 1089 	addw	sp, #115
      008719 81               [ 4] 1090 	ret
                                   1091 	.area CODE
      00871A                       1092 ___str_0:
      00871A 25 30 32 64 25 30 32  1093 	.ascii "%02d%02d%02d-%02d%02d"
             64 25 30 32 64 2D 25
             30 32 64 25 30 32 64
      00872F 00                    1094 	.db 0x00
      008730                       1095 ___str_1:
      008730 4F 63 74 20 20 35 20  1096 	.ascii "Oct  5 2018"
             32 30 31 38
      00873B 00                    1097 	.db 0x00
      00873C                       1098 ___str_2:
      00873C 31 37 3A 31 37 3A 34  1099 	.ascii "17:17:40"
             30
      008744 00                    1100 	.db 0x00
      008745                       1101 ___str_3:
      008745 25 63 3A 52 75 6E 6E  1102 	.ascii "%c:Running:%s:%02x"
             69 6E 67 3A 25 73 3A
             25 30 32 78
      008757 0D                    1103 	.db 0x0d
      008758 0A                    1104 	.db 0x0a
      008759 00                    1105 	.db 0x00
      00875A                       1106 ___str_4:
      00875A 30 30 30 30 61 61     1107 	.ascii "0000aa"
      008760 00                    1108 	.db 0x00
      008761                       1109 ___str_5:
      008761 25 63 3A 25 30 32 64  1110 	.ascii "%c:%02d.%02d:%05d:%s:"
             2E 25 30 32 64 3A 25
             30 35 64 3A 25 73 3A
      008776 0D                    1111 	.db 0x0d
      008777 0A                    1112 	.db 0x0a
      008778 00                    1113 	.db 0x00
                                   1114 	.area INITIALIZER
                                   1115 	.area CABS (ABS)
