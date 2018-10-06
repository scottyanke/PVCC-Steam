;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.7.0 #10231 (Linux)
;--------------------------------------------------------
	.module main
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _ADC_init
	.globl _ADC_read
	.globl _InitialiseIWDG
	.globl _InitialiseSystemClock
	.globl _UART_read_byte
	.globl _uart_write
	.globl _uart_init
	.globl _strcpy
	.globl _sprintf
	.globl _printf
	.globl _version
	.globl _address
	.globl _esc
	.globl _Global_time
	.globl _putchar
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
_Global_time::
	.ds 4
_esc::
	.ds 1
_address::
	.ds 1
_version::
	.ds 12
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
;--------------------------------------------------------
; Stack segment in internal ram 
;--------------------------------------------------------
	.area	SSEG
__start__stack:
	.ds	1

;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)
;--------------------------------------------------------
; interrupt vector 
;--------------------------------------------------------
	.area HOME
__interrupt_vect:
	int s_GSINIT ; reset
	int 0x0000 ; trap
	int _TLI_IRQHandler ; int0
	int _AWU_IRQHandler ; int1
	int _CLK_IRQHandler ; int2
	int _EXTI_PORTA_IRQHandler ; int3
	int _EXTI_PORTB_IRQHandler ; int4
	int _EXTI_PORTC_IRQHandler ; int5
	int _EXTI_PORTD_IRQHandler ; int6
	int _EXTI_PORTE_IRQHandler ; int7
	int 0x0000 ; int8
	int 0x0000 ; int9
	int _SPI_IRQHandler ; int10
	int _TIM1_UPD_OVF_TRG_BRK_IRQHandler ; int11
	int _TIM1_CAP_COM_IRQHandler ; int12
	int _TIM2_UPD_OVF_BRK_IRQHandler ; int13
	int _TIM2_CAP_COM_IRQHandler ; int14
	int 0x0000 ; int15
	int 0x0000 ; int16
	int _UART1_TX_IRQHandler ; int17
	int _UART1_RX_IRQHandler ; int18
	int _I2C_IRQHandler ; int19
	int 0x0000 ; int20
	int 0x0000 ; int21
	int _ADC1_IRQHandler ; int22
	int _TIM4_UPD_OVF_IRQHandler ; int23
	int _EEPROM_EEC_IRQHandler ; int24
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
__sdcc_gs_init_startup:
__sdcc_init_data:
; stm8_genXINIT() start
	ldw x, #l_DATA
	jreq	00002$
00001$:
	clr (s_DATA - 1, x)
	decw x
	jrne	00001$
00002$:
	ldw	x, #l_INITIALIZER
	jreq	00004$
00003$:
	ld	a, (s_INITIALIZER - 1, x)
	ld	(s_INITIALIZED - 1, x), a
	decw	x
	jrne	00003$
00004$:
; stm8_genXINIT() end
	.area GSFINAL
	jp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
__sdcc_program_startup:
	jp	_main
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	../src/main.c: 33: int putchar(int c) {
;	-----------------------------------------
;	 function putchar
;	-----------------------------------------
_putchar:
;	../src/main.c: 34: uart_write(c);
	ld	a, (0x04, sp)
	push	a
	call	_uart_write
	pop	a
;	../src/main.c: 35: return c;
	ldw	x, (0x03, sp)
;	../src/main.c: 36: }
	ret
;	../src/main.c: 42: void InitialiseSystemClock()
;	-----------------------------------------
;	 function InitialiseSystemClock
;	-----------------------------------------
_InitialiseSystemClock:
;	../src/main.c: 44: CLK->ICKR = 0;                       //  Reset the Internal Clock Register.
	mov	0x50c0+0, #0x00
;	../src/main.c: 45: CLK->ICKR = CLK_ICKR_HSIEN;          //  Enable the HSI.
	mov	0x50c0+0, #0x01
;	../src/main.c: 46: CLK->ECKR = 0;                       //  Disable the external clock.
	mov	0x50c1+0, #0x00
;	../src/main.c: 47: while (!(CLK->ICKR & CLK_ICKR_HSIRDY)); //  Wait for the HSI to be ready for use.
00101$:
	ld	a, 0x50c0
	bcp	a, #0x02
	jreq	00101$
;	../src/main.c: 48: CLK->CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
	mov	0x50c6+0, #0x00
;	../src/main.c: 49: CLK->PCKENR1 = 0xff; //CLK_PCKENR1_TIM4 | CLK_PCKENR1_UART1 | CLK_PCKENR1_SPI | CLK_PCKENR1_I2C ;  //  Enable select peripheral clocks.
	mov	0x50c7+0, #0xff
;	../src/main.c: 50: CLK->PCKENR2 = 0xff; //CLK_PCKENR2_AWU;      //  Only enable the AWU watchdog service
	mov	0x50ca+0, #0xff
;	../src/main.c: 51: CLK->CCOR = 0;                       //  Turn off CCO.
	mov	0x50c9+0, #0x00
;	../src/main.c: 52: CLK->HSITRIMR = 0;                   //  Turn off any HSIU trimming.
	mov	0x50cc+0, #0x00
;	../src/main.c: 53: CLK->SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
	mov	0x50cd+0, #0x00
;	../src/main.c: 54: CLK->SWR = 0xe1;                     //  Use HSI as the clock source.
	mov	0x50c4+0, #0xe1
;	../src/main.c: 55: CLK->SWCR = 0;                       //  Reset the clock switch control register.
	mov	0x50c5+0, #0x00
;	../src/main.c: 56: CLK->SWCR |= CLK_SWCR_SWEN;          //  Enable switching.
	bset	20677, #1
;	../src/main.c: 57: while (CLK->SWCR & CLK_SWCR_SWBSY);  //  Pause while the clock switch is busy.
00104$:
	ld	a, 0x50c5
	srl	a
	jrc	00104$
;	../src/main.c: 58: }
	ret
;	../src/main.c: 63: void InitialiseIWDG()
;	-----------------------------------------
;	 function InitialiseIWDG
;	-----------------------------------------
_InitialiseIWDG:
;	../src/main.c: 65: IWDG->KR = 0xcc;         //  Start the independent watchdog.
	mov	0x50e0+0, #0xcc
;	../src/main.c: 66: IWDG->KR = 0x55;         //  Allow the IWDG registers to be programmed.
	mov	0x50e0+0, #0x55
;	../src/main.c: 67: IWDG->PR = 0x06;         //  Prescaler is 6 => each count is 1.02 second with RLR = 0xff
	mov	0x50e1+0, #0x06
;	../src/main.c: 68: IWDG->RLR = 0xff;        //  Reload counter.  T = 2 x TLSI x PR x R LR
	mov	0x50e2+0, #0xff
;	../src/main.c: 69: IWDG->KR = 0xaa;         //  Reset the counter.
	mov	0x50e0+0, #0xaa
;	../src/main.c: 70: }
	ret
;	../src/main.c: 73: uint16_t ADC_read() {
;	-----------------------------------------
;	 function ADC_read
;	-----------------------------------------
_ADC_read:
	sub	sp, #4
;	../src/main.c: 75: ADC1->CR1 |= ADC1_CR1_ADON;
	bset	21505, #0
;	../src/main.c: 76: while (!(ADC1->CSR & (ADC1_CSR_EOC)));
00101$:
	ld	a, 0x5400
	tnz	a
	jrpl	00101$
;	../src/main.c: 77: adcL = ADC1->DRL;
	ldw	x, #0x5405
	ld	a, (x)
	ld	yl, a
;	../src/main.c: 78: adcH = ADC1->DRH;
	ldw	x, #0x5404
	ld	a, (x)
	ld	xl, a
;	../src/main.c: 79: ADC1->CSR &= ~(ADC1_CSR_EOC); // Clear EOC flag
	bres	21504, #7
;	../src/main.c: 80: return (adcL | (adcH << 8));
	ld	a, xl
	clr	(0x04, sp)
	clr	(0x01, sp)
	or	a, (0x01, sp)
	ld	xh, a
	ld	a, yl
	or	a, (0x04, sp)
	ld	xl, a
;	../src/main.c: 81: }
	addw	sp, #4
	ret
;	../src/main.c: 82: void ADC_init() {
;	-----------------------------------------
;	 function ADC_init
;	-----------------------------------------
_ADC_init:
;	../src/main.c: 84: ADC1->CSR |= (1 << 2);
	bset	21504, #2
;	../src/main.c: 86: ADC1->CR2 |= ADC1_CR2_ALIGN;
	bset	21506, #3
;	../src/main.c: 88: ADC1->CR1 |= ADC1_CR1_ADON;
	bset	21505, #0
;	../src/main.c: 89: }
	ret
;	../src/main.c: 90: void main() {
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
	sub	sp, #115
;	../src/main.c: 91: unsigned long esc_time = 0L;
	clrw	x
	ldw	(0x6c, sp), x
	ldw	(0x6a, sp), x
;	../src/main.c: 99: esc = 0;
	clr	_esc+0
;	../src/main.c: 101: sprintf(version,"%02d%02d%02d-%02d%02d", BUILD_YEAR - 2000, BUILD_MONTH, BUILD_DAY, BUILD_HOUR, BUILD_MIN);
	ldw	x, #___str_2+0
	ldw	(0x1c, sp), x
	ld	a, (x)
	ld	(0x2b, sp), a
	ld	a, (0x2b, sp)
	cp	a, #0x3f
	jrne	00426$
	ld	a, #0x01
	ld	(0x1b, sp), a
	jra	00427$
00426$:
	clr	(0x1b, sp)
00427$:
	tnz	(0x1b, sp)
	jreq	00153$
	ldw	x, #0x0063
	ldw	(0x19, sp), x
	jra	00154$
00153$:
	ldw	x, (0x1c, sp)
	ld	a, (0x3, x)
	clrw	x
	ld	xl, a
	subw	x, #0x0030
	pushw	x
	sllw	x
	sllw	x
	addw	x, (1, sp)
	sllw	x
	addw	sp, #2
	ldw	(0x17, sp), x
	ldw	x, (0x1c, sp)
	ld	a, (0x4, x)
	clrw	x
	ld	xl, a
	addw	x, (0x17, sp)
	subw	x, #0x0030
	ldw	(0x19, sp), x
00154$:
	tnz	(0x1b, sp)
	jreq	00155$
	ldw	x, #0x0063
	ldw	(0x15, sp), x
	jra	00156$
00155$:
	clrw	x
	ld	a, (0x2b, sp)
	ld	xl, a
	subw	x, #0x0030
	pushw	x
	sllw	x
	sllw	x
	addw	x, (1, sp)
	sllw	x
	addw	sp, #2
	ldw	(0x35, sp), x
	ldw	x, (0x1c, sp)
	ld	a, (0x1, x)
	clrw	x
	ld	xl, a
	addw	x, (0x35, sp)
	subw	x, #0x0030
	ldw	(0x33, sp), x
	ldw	y, x
	ldw	(0x15, sp), y
00156$:
	ldw	x, #___str_1+0
	ldw	(0x0a, sp), x
	ld	a, (x)
	ld	(0x09, sp), a
	ld	a, (0x09, sp)
	cp	a, #0x3f
	jrne	00431$
	ld	a, #0x01
	ld	(0x11, sp), a
	jra	00432$
00431$:
	clr	(0x11, sp)
00432$:
	tnz	(0x11, sp)
	jreq	00157$
	ldw	x, #0x0063
	ldw	(0x0f, sp), x
	jra	00158$
00157$:
	ldw	x, (0x0a, sp)
	ld	a, (0x4, x)
	ld	(0x14, sp), a
	ld	a, (0x14, sp)
	cp	a, #0x30
	jrc	00159$
	clrw	x
	ld	a, (0x14, sp)
	ld	xl, a
	subw	x, #0x0030
	pushw	x
	sllw	x
	sllw	x
	addw	x, (1, sp)
	sllw	x
	addw	sp, #2
	ldw	(0x12, sp), x
	jra	00160$
00159$:
	clrw	x
	ldw	(0x12, sp), x
00160$:
	ldw	x, (0x0a, sp)
	ld	a, (0x5, x)
	clrw	x
	ld	xl, a
	subw	x, #0x0030
	addw	x, (0x12, sp)
	ldw	(0x0f, sp), x
00158$:
	tnz	(0x11, sp)
	jreq	00161$
	ldw	x, #0x0063
	ldw	(0x0d, sp), x
	jp	00162$
00161$:
	ld	a, (0x09, sp)
	cp	a, #0x4a
	jrne	00437$
	ld	a, #0x01
	ld	(0x0c, sp), a
	jra	00438$
00437$:
	clr	(0x0c, sp)
00438$:
	ldw	x, (0x0a, sp)
	incw	x
	ldw	(0x26, sp), x
	ldw	x, (0x0a, sp)
	incw	x
	incw	x
	ldw	(0x24, sp), x
	tnz	(0x0c, sp)
	jreq	00163$
	ldw	x, (0x26, sp)
	ld	a, (x)
	cp	a, #0x61
	jrne	00163$
	ldw	x, (0x24, sp)
	ld	a, (x)
	cp	a, #0x6e
	jrne	00163$
	clrw	x
	incw	x
	jp	00164$
00163$:
	ld	a, (0x09, sp)
	cp	a, #0x46
	jrne	00171$
	ldw	x, #0x0002
	ldw	(0x29, sp), x
	jp	00172$
00171$:
	ld	a, (0x09, sp)
	cp	a, #0x4d
	jrne	00450$
	ld	a, #0x01
	ld	(0x28, sp), a
	jra	00451$
00450$:
	clr	(0x28, sp)
00451$:
	tnz	(0x28, sp)
	jreq	00173$
	ldw	x, (0x26, sp)
	ld	a, (x)
	cp	a, #0x61
	jrne	00173$
	ldw	x, (0x24, sp)
	ld	a, (x)
	cp	a, #0x72
	jrne	00173$
	ldw	x, #0x0003
	jp	00174$
00173$:
	ld	a, (0x09, sp)
	cp	a, #0x41
	jrne	00460$
	ld	a, #0x01
	ld	(0x2e, sp), a
	jra	00461$
00460$:
	clr	(0x2e, sp)
00461$:
	tnz	(0x2e, sp)
	jreq	00181$
	ldw	x, (0x26, sp)
	ld	a, (x)
	cp	a, #0x70
	jrne	00181$
	ldw	x, #0x0004
	ldw	(0x2c, sp), x
	jp	00182$
00181$:
	tnz	(0x28, sp)
	jreq	00186$
	ldw	x, (0x26, sp)
	ld	a, (x)
	cp	a, #0x61
	jrne	00186$
	ldw	x, (0x24, sp)
	ld	a, (x)
	cp	a, #0x79
	jrne	00186$
	ldw	x, #0x0005
	ldw	(0x31, sp), x
	jp	00187$
00186$:
	tnz	(0x0c, sp)
	jreq	00194$
	ldw	x, (0x26, sp)
	ld	a, (x)
	cp	a, #0x75
	jrne	00194$
	ldw	x, (0x24, sp)
	ld	a, (x)
	cp	a, #0x6e
	jrne	00194$
	ldw	x, #0x0006
	jra	00195$
00194$:
	tnz	(0x0c, sp)
	jreq	00202$
	ldw	x, (0x26, sp)
	ld	a, (x)
	cp	a, #0x75
	jrne	00202$
	ldw	x, (0x24, sp)
	ld	a, (x)
	cp	a, #0x6c
	jrne	00202$
	ldw	x, #0x0007
	ldw	(0x2f, sp), x
	jra	00203$
00202$:
	tnz	(0x2e, sp)
	jreq	00210$
	ldw	x, (0x26, sp)
	ld	a, (x)
	cp	a, #0x75
	jrne	00210$
	ldw	x, #0x0008
	jra	00211$
00210$:
	ld	a, (0x09, sp)
	cp	a, #0x53
	jrne	00215$
	ldw	x, #0x0009
	ldw	(0x39, sp), x
	jra	00216$
00215$:
	ld	a, (0x09, sp)
	cp	a, #0x4f
	jrne	00217$
	ldw	x, #0x000a
	jra	00218$
00217$:
	ld	a, (0x09, sp)
	cp	a, #0x4e
	jrne	00219$
	ldw	x, #0x000b
	ldw	(0x37, sp), x
	jra	00220$
00219$:
	ld	a, (0x09, sp)
	cp	a, #0x44
	jrne	00221$
	ldw	x, #0x000c
	jra	00222$
00221$:
	ldw	x, #0x0063
00222$:
	ldw	(0x37, sp), x
00220$:
	ldw	x, (0x37, sp)
00218$:
	ldw	(0x39, sp), x
00216$:
	ldw	x, (0x39, sp)
00211$:
	ldw	(0x2f, sp), x
00203$:
	ldw	x, (0x2f, sp)
00195$:
	ldw	(0x31, sp), x
00187$:
	ldw	y, (0x31, sp)
	ldw	(0x2c, sp), y
00182$:
	ldw	x, (0x2c, sp)
00174$:
	ldw	(0x29, sp), x
00172$:
	ldw	x, (0x29, sp)
00164$:
	ldw	(0x0d, sp), x
00162$:
	tnz	(0x11, sp)
	jreq	00223$
	ldw	x, #0x0063
	jra	00224$
00223$:
	ldw	x, (0x0a, sp)
	ld	a, (0x7, x)
	clrw	x
	ld	xl, a
	subw	x, #0x0030
	pushw	x
	push	#0xe8
	push	#0x03
	call	__mulint
	addw	sp, #4
	ldw	(0x3d, sp), x
	ldw	x, (0x0a, sp)
	ld	a, (0x8, x)
	clrw	x
	ld	xl, a
	subw	x, #0x0030
	pushw	x
	push	#0x64
	push	#0x00
	call	__mulint
	addw	sp, #4
	addw	x, (0x3d, sp)
	ldw	(0x3b, sp), x
	ldw	x, (0x0a, sp)
	ld	a, (0x9, x)
	clrw	x
	ld	xl, a
	subw	x, #0x0030
	pushw	x
	sllw	x
	sllw	x
	addw	x, (1, sp)
	sllw	x
	addw	sp, #2
	addw	x, (0x3b, sp)
	ldw	(0x41, sp), x
	ldw	x, (0x0a, sp)
	ld	a, (0xa, x)
	clrw	x
	ld	xl, a
	subw	x, #0x0030
	addw	x, (0x41, sp)
	ldw	(0x3f, sp), x
00224$:
	subw	x, #0x07d0
	ldw	(0x49, sp), x
	ldw	x, #___str_0+0
	ldw	(0x47, sp), x
	ldw	x, #_version+0
	ldw	(0x4d, sp), x
	ldw	y, x
	ldw	x, (0x19, sp)
	pushw	x
	ldw	x, (0x17, sp)
	pushw	x
	ldw	x, (0x13, sp)
	pushw	x
	ldw	x, (0x13, sp)
	pushw	x
	ldw	x, (0x51, sp)
	pushw	x
	ldw	x, (0x51, sp)
	pushw	x
	pushw	y
	call	_sprintf
	addw	sp, #14
;	../src/main.c: 103: disableInterrupts();
	sim
;	../src/main.c: 104: InitialiseSystemClock();
	call	_InitialiseSystemClock
;	../src/main.c: 106: GPIOD->CR2 &= (uint8_t)(~(GPIO_PIN_4));
	bres	20499, #4
;	../src/main.c: 107: GPIOD->ODR &= (uint8_t)(~(GPIO_PIN_4));
	bres	20495, #4
;	../src/main.c: 108: GPIOD->DDR |= (uint8_t)GPIO_PIN_4;
	bset	20497, #4
;	../src/main.c: 109: GPIOD->CR1 |= (uint8_t)GPIO_PIN_4;
	bset	20498, #4
;	../src/main.c: 110: GPIOD->CR2 |= (uint8_t)GPIO_PIN_4;
	bset	20499, #4
;	../src/main.c: 114: GPIOD->DDR &= ~(GPIO_PIN_2 | GPIO_PIN_1);  // input mode
	ld	a, 0x5011
	and	a, #0xf9
	ld	0x5011, a
;	../src/main.c: 115: GPIOD->CR1 |= (GPIO_PIN_1 | GPIO_PIN_2);  // pull-ups
	ld	a, 0x5012
	or	a, #0x06
	ld	0x5012, a
;	../src/main.c: 116: GPIOD->CR2 &= ~(GPIO_PIN_2 | GPIO_PIN_1);  // no interrupts
	ld	a, 0x5013
	and	a, #0xf9
	ld	0x5013, a
;	../src/main.c: 117: GPIOC->DDR &= ~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7); //  input mode
	ld	a, 0x500c
	and	a, #0x0f
	ld	0x500c, a
;	../src/main.c: 118: GPIOC->CR1 |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);  // no interrupts
	ld	a, 0x500d
	or	a, #0xf0
	ld	0x500d, a
;	../src/main.c: 119: GPIOC->CR2 &= ~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);  // no interrupts
	ld	a, 0x500e
	and	a, #0x0f
	ld	0x500e, a
;	../src/main.c: 122: rs485xmit_off();
	bres	20495, #4
;	../src/main.c: 123: CFG->GCR |= 1; // disable SWIM
	ld	a, 0x7f60
	clrw	x
	ld	xl, a
	srlw	x
	scf
	rlcw	x
	ld	a, xl
	ld	0x7f60, a
;	../src/main.c: 126: TIM4->PSCR = 7;   // prescaler
	mov	0x5347+0, #0x07
;	../src/main.c: 127: TIM4->ARR = 125;  // auto reload register
	mov	0x5348+0, #0x7d
;	../src/main.c: 129: TIM4->IER = TIM4_IER_UIE;
	mov	0x5343+0, #0x01
;	../src/main.c: 131: TIM4->CR1 = TIM4_CR1_ARPE | TIM4_CR1_URS | TIM4_CR1_CEN;
	mov	0x5340+0, #0x85
;	../src/main.c: 133: Global_time = 0L;
	clrw	x
	ldw	_Global_time+2, x
	ldw	_Global_time+0, x
;	../src/main.c: 134: uart_init();		// initialize the uart functions - 9600 8-N-1 through RS485
	call	_uart_init
;	../src/main.c: 136: enableInterrupts();
	rim
;	../src/main.c: 138: address = 'S';		// this devices id character
	mov	_address+0, #0x53
;	../src/main.c: 140: ADC_init();			// initialize the analog read function
	call	_ADC_init
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	x
	ldw	(0x65, sp), x
	ldw	(0x63, sp), x
00135$:
	ldw	x, (0x65, sp)
	cpw	x, #0xd700
	ld	a, (0x64, sp)
	sbc	a, #0x0a
	ld	a, (0x63, sp)
	sbc	a, #0x00
	jrnc	00125$
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	ldw	y, (0x65, sp)
	addw	y, #0x0001
	ld	a, (0x64, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x63, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x65, sp), y
	ldw	(0x63, sp), x
	jra	00135$
;	../src/main.c: 141: delay_ms(800);
00125$:
;	../src/main.c: 142: rs485xmit_on();	// turn the RS485 chips transmitter on
	ld	a, 0x500f
	or	a, #0x10
	ld	0x500f, a
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	x
	ldw	(0x61, sp), x
	ldw	(0x5f, sp), x
00138$:
	ldw	x, (0x61, sp)
	cpw	x, #0x6810
	ld	a, (0x60, sp)
	sbc	a, #0x00
	ld	a, (0x5f, sp)
	sbc	a, #0x00
	jrnc	00127$
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	ldw	y, (0x61, sp)
	addw	y, #0x0001
	ld	a, (0x60, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x5f, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x61, sp), y
	ldw	(0x5f, sp), x
	jra	00138$
;	../src/main.c: 143: delay_ms(30);
00127$:
;	../src/main.c: 144: printf("%c:Running:%s:%02x\r\n",address,version,address);
	clrw	x
	ld	a, _address+0
	ld	xl, a
	ldw	y, (0x4d, sp)
	ldw	(0x4b, sp), y
	ldw	y, #___str_3+0
	pushw	x
	ld	a, (0x4e, sp)
	push	a
	ld	a, (0x4e, sp)
	push	a
	pushw	x
	pushw	y
	call	_printf
	addw	sp, #8
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	x
	ldw	(0x5d, sp), x
	ldw	(0x5b, sp), x
00141$:
	ldw	x, (0x5d, sp)
	cpw	x, #0x22b0
	ld	a, (0x5c, sp)
	sbc	a, #0x00
	ld	a, (0x5b, sp)
	sbc	a, #0x00
	jrnc	00129$
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	ldw	y, (0x5d, sp)
	addw	y, #0x0001
	ld	a, (0x5c, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x5b, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x5d, sp), y
	ldw	(0x5b, sp), x
	jra	00141$
;	../src/main.c: 145: delay_ms(10);
00129$:
;	../src/main.c: 146: rs485xmit_off(); // turn the transmitter back off
	ld	a, 0x500f
	and	a, #0xef
	ld	0x500f, a
;	../src/main.c: 147: InitialiseIWDG();
	call	_InitialiseIWDG
;	../src/main.c: 148: reset_watchdog();  // reset the watchdog timer
	mov	0x50e0+0, #0xaa
;	../src/main.c: 151: do{
00121$:
;	../src/main.c: 152: reset_watchdog();  // reset the watchdog timer
	mov	0x50e0+0, #0xaa
;	../src/main.c: 153: if(UART_read_byte(&rb)){ // buffer isn't empty
	ldw	x, sp
	incw	x
	pushw	x
	call	_UART_read_byte
	addw	sp, #2
	tnz	a
	jrne	00507$
	jp	00117$
00507$:
;	../src/main.c: 154: switch(rb){
	ld	a, (0x01, sp)
	cp	a, #0x1b
	jrne	00102$
;	../src/main.c: 156: esc = 1;
	mov	_esc+0, #0x01
;	../src/main.c: 157: esc_time = Global_time;	// only wait two seconds for the next character after the escape
	ldw	x, _Global_time+2
	ldw	(0x6c, sp), x
	ldw	x, _Global_time+0
	ldw	(0x6a, sp), x
;	../src/main.c: 158: break;
	jp	00117$
;	../src/main.c: 159: default:
00102$:
;	../src/main.c: 160: if (rb == address && esc)  // address must match the switches read by mcp23017
	ld	a, (0x01, sp)
	cp	a, _address+0
	jreq	00513$
	jp	00113$
00513$:
	tnz	_esc+0
	jrne	00514$
	jp	00113$
00514$:
;	../src/main.c: 162: Global_time = 0L;   // when was the last time we were called?
	clrw	x
	ldw	_Global_time+2, x
	ldw	_Global_time+0, x
;	../src/main.c: 163: voltage = ADC_read();	// get the analog value from the pressure sensor
	call	_ADC_read
;	../src/main.c: 164: if (voltage < VOLTAGE_OFFSET)		// the sensor range is from 0.5 volts to 4.5 volts
	ldw	(0x67, sp), x
	cpw	x, #0x0078
	jrnc	00104$
;	../src/main.c: 165: voltage = VOLTAGE_OFFSET;
	ldw	x, #0x0078
	ldw	(0x67, sp), x
00104$:
;	../src/main.c: 166: PSI = (uint16_t)((voltage - VOLTAGE_OFFSET) * 1.15);  // formula to calculate PSI
	ldw	x, (0x67, sp)
	subw	x, #0x0078
	pushw	x
	call	___uint2fs
	addw	sp, #2
	pushw	x
	pushw	y
	push	#0x33
	push	#0x33
	push	#0x93
	push	#0x3f
	call	___fsmul
	addw	sp, #8
	pushw	x
	pushw	y
	call	___fs2uint
	addw	sp, #4
;	../src/main.c: 168: psi1 = (uint16_t)(PSI / 100);
	ldw	(0x72, sp), x
	ldw	y, #0x0064
	divw	x, y
;	../src/main.c: 169: psi2 = (uint16_t)(PSI - (psi1 * 100));
	ldw	(0x70, sp), x
	pushw	x
	push	#0x64
	push	#0x00
	call	__mulint
	addw	sp, #4
	ldw	(0x51, sp), x
	ldw	x, (0x72, sp)
	subw	x, (0x51, sp)
	ldw	(0x6e, sp), x
;	../src/main.c: 170: strcpy(s,"0000aa");
	ldw	y, #___str_4+0
	ldw	x, sp
	incw	x
	incw	x
	ldw	(0x4f, sp), x
	pushw	y
	pushw	x
	call	_strcpy
	addw	sp, #4
;	../src/main.c: 171: for (i = 0; i < 4; i++)		// read through the on-off inputs
	clr	(0x69, sp)
00143$:
;	../src/main.c: 173: if (!(GPIOC->IDR & (GPIO_PIN_4 << i)))
	ld	a, 0x500b
	push	a
	ldw	x, #0x0010
	ld	a, (0x6a, sp)
	jreq	00517$
00516$:
	sllw	x
	dec	a
	jrne	00516$
00517$:
	pop	a
	ld	(0x46, sp), a
	clr	(0x45, sp)
	ld	a, xl
	and	a, (0x46, sp)
	ld	(0x44, sp), a
	ld	a, xh
	and	a, (0x45, sp)
	ld	(0x43, sp), a
	ldw	x, (0x43, sp)
	jrne	00144$
;	../src/main.c: 174: s[i] = '1';		// if the gpio is low, that means on
	ld	a, (0x69, sp)
	add	a, (0x50, sp)
	push	a
	clr	a
	adc	a, (0x50, sp)
	ld	xh, a
	pop	a
	ld	xl, a
	ld	a, #0x31
	ld	(x), a
00144$:
;	../src/main.c: 171: for (i = 0; i < 4; i++)		// read through the on-off inputs
	inc	(0x69, sp)
	ld	a, (0x69, sp)
	cp	a, #0x04
	jrc	00143$
;	../src/main.c: 176: if (!(GPIOD->IDR & GPIO_PIN_1))	// these are for the air conditioners
	ld	a, 0x5010
	bcp	a, #0x02
	jrne	00109$
;	../src/main.c: 177: s[4] = 'A';
	ldw	x, (0x4f, sp)
	addw	x, #0x0004
	ld	a, #0x41
	ld	(x), a
00109$:
;	../src/main.c: 178: if (!(GPIOD->IDR & GPIO_PIN_2))
	ld	a, 0x5010
	bcp	a, #0x04
	jrne	00111$
;	../src/main.c: 179: s[5] = 'A';
	ldw	x, (0x4f, sp)
	ld	a, #0x41
	ld	(0x0005, x), a
00111$:
;	../src/main.c: 180: s[6] = 0;		// terminate the string
	ldw	x, (0x4f, sp)
	addw	x, #0x0006
	clr	(x)
;	../src/main.c: 181: rs485xmit_on();	// turn the RS485 chips transmitter on
	ld	a, 0x500f
	or	a, #0x10
	ld	0x500f, a
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	x
	ldw	(0x59, sp), x
	ldw	(0x57, sp), x
00146$:
	ldw	x, (0x59, sp)
	cpw	x, #0x6810
	ld	a, (0x58, sp)
	sbc	a, #0x00
	ld	a, (0x57, sp)
	sbc	a, #0x00
	jrnc	00131$
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	ldw	y, (0x59, sp)
	addw	y, #0x0001
	ld	a, (0x58, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x57, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x59, sp), y
	ldw	(0x57, sp), x
	jra	00146$
;	../src/main.c: 182: delay_ms(30);	// make sure transmitter has time to turn on
00131$:
;	../src/main.c: 183: printf("%c:%02d.%02d:%05d:%s:\r\n", address,psi1,psi2,voltage,s); // S:23.56:89012:0101Aa:
	ldw	x, (0x4f, sp)
	ld	a, _address+0
	ld	(0x23, sp), a
	clr	(0x22, sp)
	ldw	y, #___str_5+0
	pushw	x
	ldw	x, (0x69, sp)
	pushw	x
	ldw	x, (0x72, sp)
	pushw	x
	ldw	x, (0x76, sp)
	pushw	x
	ldw	x, (0x2a, sp)
	pushw	x
	pushw	y
	call	_printf
	addw	sp, #12
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	clrw	x
	ldw	(0x55, sp), x
	ldw	(0x53, sp), x
00149$:
	ldw	x, (0x55, sp)
	cpw	x, #0x22b0
	ld	a, (0x54, sp)
	sbc	a, #0x00
	ld	a, (0x53, sp)
	sbc	a, #0x00
	jrnc	00133$
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 14: __asm__("nop");
	nop
;	/home/scott/projects-stm8/pvcc-steam/inc/delay.h: 13: for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++) {
	ldw	y, (0x55, sp)
	addw	y, #0x0001
	ld	a, (0x54, sp)
	adc	a, #0x00
	ld	xl, a
	ld	a, (0x53, sp)
	adc	a, #0x00
	ld	xh, a
	ldw	(0x55, sp), y
	ldw	(0x53, sp), x
	jra	00149$
;	../src/main.c: 184: delay_ms(10);	// give it time to transmit before turning transmitter off
00133$:
;	../src/main.c: 185: rs485xmit_off(); // turn the transmitter back off
	ld	a, 0x500f
	and	a, #0xef
	ld	0x500f, a
00113$:
;	../src/main.c: 187: esc = 0;	// reset the flag for the escape character
	clr	_esc+0
;	../src/main.c: 188: }
00117$:
;	../src/main.c: 190: if (esc && (Global_time - esc_time > 2000))  // give it 2 seconds to send the id character
	tnz	_esc+0
	jrne	00524$
	jp	00121$
00524$:
	ldw	x, _Global_time+2
	subw	x, (0x6c, sp)
	ldw	(0x20, sp), x
	ld	a, _Global_time+1
	sbc	a, (0x6b, sp)
	ld	(0x1f, sp), a
	ld	a, _Global_time+0
	sbc	a, (0x6a, sp)
	ld	(0x1e, sp), a
	ldw	x, #0x07d0
	cpw	x, (0x20, sp)
	clr	a
	sbc	a, (0x1f, sp)
	clr	a
	sbc	a, (0x1e, sp)
	jrc	00525$
	jp	00121$
00525$:
;	../src/main.c: 191: esc = 0;  // reset the esc flag, since it should have been followed by the id right away
	clr	_esc+0
;	../src/main.c: 192: }while(1);
	jp	00121$
;	../src/main.c: 193: }
	addw	sp, #115
	ret
	.area CODE
___str_0:
	.ascii "%02d%02d%02d-%02d%02d"
	.db 0x00
___str_1:
	.ascii "Oct  5 2018"
	.db 0x00
___str_2:
	.ascii "17:17:40"
	.db 0x00
___str_3:
	.ascii "%c:Running:%s:%02x"
	.db 0x0d
	.db 0x0a
	.db 0x00
___str_4:
	.ascii "0000aa"
	.db 0x00
___str_5:
	.ascii "%c:%02d.%02d:%05d:%s:"
	.db 0x0d
	.db 0x0a
	.db 0x00
	.area INITIALIZER
	.area CABS (ABS)
