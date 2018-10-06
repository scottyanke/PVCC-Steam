;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.6.0 #9615 (Linux)
;--------------------------------------------------------
	.module i2c
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _i2c_init
	.globl _i2c_clear
	.globl _i2c_start
	.globl _i2c_stop
	.globl _i2c_write
	.globl _i2c_write_addr
	.globl _i2c_read
	.globl _i2c_read_arr
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
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 4: void i2c_init() {
;	-----------------------------------------
;	 function i2c_init
;	-----------------------------------------
_i2c_init:
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 5: I2C_FREQR = (1 << I2C_FREQR_FREQ1);
	mov	0x5212+0, #0x02
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 6: I2C_CCRL = 0x0A; // 100kHz
	mov	0x521b+0, #0x0a
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 7: I2C_OARH = (1 << I2C_OARH_ADDMODE); // 7-bit addressing
	mov	0x5214+0, #0x80
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 8: I2C_CR1 = (1 << I2C_CR1_PE);
	mov	0x5210+0, #0x01
	ret
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 11: void i2c_clear() {
;	-----------------------------------------
;	 function i2c_clear
;	-----------------------------------------
_i2c_clear:
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 12: (void) I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 13: I2C_DR = 0x00;
	mov	0x5216+0, #0x00
	ret
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 16: void i2c_start() {
;	-----------------------------------------
;	 function i2c_start
;	-----------------------------------------
_i2c_start:
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 18: I2C_CR2 |= (1 << I2C_CR2_START);
	bset	0x5211, #0
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 19: i = 0;
	clr	a
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 20: while (!(I2C_SR1 & (1 << I2C_SR1_SB)) && i < 200)
00102$:
	ldw	x, #0x5217
	push	a
	ld	a, (x)
	ld	xl, a
	pop	a
	srlw	x
	jrnc	00119$
	ret
00119$:
	cp	a, #0xc8
	jrc	00120$
	ret
00120$:
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 21: i++;
	inc	a
	jra	00102$
	ret
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 24: void i2c_stop() {
;	-----------------------------------------
;	 function i2c_stop
;	-----------------------------------------
_i2c_stop:
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 25: I2C_CR2 |= (1 << I2C_CR2_STOP);
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 26: while (I2C_SR3 & (1 << I2C_SR3_MSL));
00101$:
	ldw	x, #0x5219
	ld	a, (x)
	srl	a
	jrc	00101$
	ret
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 29: void i2c_write(uint8_t data) {
;	-----------------------------------------
;	 function i2c_write
;	-----------------------------------------
_i2c_write:
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 31: I2C_DR = data;
	ldw	x, #0x5216
	ld	a, (0x03, sp)
	ld	(x), a
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 32: i = 0;
	clr	a
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 33: while (!(I2C_SR1 & (1 << I2C_SR1_TXE)) && i < 200)
00102$:
	ldw	x, #0x5217
	push	a
	ld	a, (x)
	ld	xh, a
	pop	a
	tnzw	x
	jrpl	00119$
	ret
00119$:
	cp	a, #0xc8
	jrc	00120$
	ret
00120$:
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 34: i++;
	inc	a
	jra	00102$
	ret
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 37: void i2c_write_addr(uint8_t addr) {
;	-----------------------------------------
;	 function i2c_write_addr
;	-----------------------------------------
_i2c_write_addr:
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 39: I2C_DR = addr;
	ldw	x, #0x5216
	ld	a, (0x03, sp)
	ld	(x), a
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 40: i = 0;
	clr	a
	ld	xl, a
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 41: while (!(I2C_SR1 & (1 << I2C_SR1_ADDR)) && i < 200)
00102$:
	ldw	y, #0x5217
	ld	a, (y)
	bcp	a, #0x02
	jrne	00104$
	ld	a, xl
	cp	a, #0xc8
	jrnc	00104$
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 42: i++;;
	incw	x
	jra	00102$
00104$:
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 43: (void) I2C_SR3; // check BUS_BUSY
	ldw	x, #0x5219
	ld	a, (x)
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 44: I2C_CR2 |= (1 << I2C_CR2_ACK);
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
	ret
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 47: uint8_t i2c_read() {
;	-----------------------------------------
;	 function i2c_read
;	-----------------------------------------
_i2c_read:
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 48: I2C_CR2 &= ~(1 << I2C_CR2_ACK);
	ldw	x, #0x5211
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 49: i2c_stop();
	call	_i2c_stop
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 50: while (!(I2C_SR1 & (1 << I2C_SR1_RXNE)));
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	bcp	a, #0x40
	jreq	00101$
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 51: return I2C_DR;
	ldw	x, #0x5216
	ld	a, (x)
	ret
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 54: void i2c_read_arr(uint8_t *buf, int len) {
;	-----------------------------------------
;	 function i2c_read_arr
;	-----------------------------------------
_i2c_read_arr:
	sub	sp, #2
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 55: while (len-- > 1) {
00104$:
	ldw	y, (0x07, sp)
	ldw	(0x01, sp), y
	ldw	x, (0x01, sp)
	decw	x
	ldw	(0x07, sp), x
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 58: *(buf++) = I2C_DR;
	ldw	y, (0x05, sp)
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 55: while (len-- > 1) {
	ldw	x, (0x01, sp)
	cpw	x, #0x0001
	jrsle	00106$
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 56: I2C_CR2 |= (1 << I2C_CR2_ACK);
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 57: while (!(I2C_SR1 & (1 << I2C_SR1_RXNE)));
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	bcp	a, #0x40
	jreq	00101$
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 58: *(buf++) = I2C_DR;
	ldw	x, #0x5216
	ld	a, (x)
	ld	(y), a
	incw	y
	ldw	(0x05, sp), y
	jra	00104$
00106$:
;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 60: *buf = i2c_read();
	pushw	y
	call	_i2c_read
	popw	y
	ld	(y), a
	addw	sp, #2
	ret
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
