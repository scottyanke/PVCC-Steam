                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.6.0 #9615 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module i2c
                                      6 	.optsdcc -mstm8
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _i2c_init
                                     12 	.globl _i2c_clear
                                     13 	.globl _i2c_start
                                     14 	.globl _i2c_stop
                                     15 	.globl _i2c_write
                                     16 	.globl _i2c_write_addr
                                     17 	.globl _i2c_read
                                     18 	.globl _i2c_read_arr
                                     19 ;--------------------------------------------------------
                                     20 ; ram data
                                     21 ;--------------------------------------------------------
                                     22 	.area DATA
                                     23 ;--------------------------------------------------------
                                     24 ; ram data
                                     25 ;--------------------------------------------------------
                                     26 	.area INITIALIZED
                                     27 ;--------------------------------------------------------
                                     28 ; absolute external ram data
                                     29 ;--------------------------------------------------------
                                     30 	.area DABS (ABS)
                                     31 ;--------------------------------------------------------
                                     32 ; global & static initialisations
                                     33 ;--------------------------------------------------------
                                     34 	.area HOME
                                     35 	.area GSINIT
                                     36 	.area GSFINAL
                                     37 	.area GSINIT
                                     38 ;--------------------------------------------------------
                                     39 ; Home
                                     40 ;--------------------------------------------------------
                                     41 	.area HOME
                                     42 	.area HOME
                                     43 ;--------------------------------------------------------
                                     44 ; code
                                     45 ;--------------------------------------------------------
                                     46 	.area CODE
                                     47 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 4: void i2c_init() {
                                     48 ;	-----------------------------------------
                                     49 ;	 function i2c_init
                                     50 ;	-----------------------------------------
      0080CB                         51 _i2c_init:
                                     52 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 5: I2C_FREQR = (1 << I2C_FREQR_FREQ1);
      0080CB 35 02 52 12      [ 1]   53 	mov	0x5212+0, #0x02
                                     54 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 6: I2C_CCRL = 0x0A; // 100kHz
      0080CF 35 0A 52 1B      [ 1]   55 	mov	0x521b+0, #0x0a
                                     56 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 7: I2C_OARH = (1 << I2C_OARH_ADDMODE); // 7-bit addressing
      0080D3 35 80 52 14      [ 1]   57 	mov	0x5214+0, #0x80
                                     58 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 8: I2C_CR1 = (1 << I2C_CR1_PE);
      0080D7 35 01 52 10      [ 1]   59 	mov	0x5210+0, #0x01
      0080DB 81               [ 4]   60 	ret
                                     61 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 11: void i2c_clear() {
                                     62 ;	-----------------------------------------
                                     63 ;	 function i2c_clear
                                     64 ;	-----------------------------------------
      0080DC                         65 _i2c_clear:
                                     66 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 12: (void) I2C_SR1;
      0080DC AE 52 17         [ 2]   67 	ldw	x, #0x5217
      0080DF F6               [ 1]   68 	ld	a, (x)
                                     69 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 13: I2C_DR = 0x00;
      0080E0 35 00 52 16      [ 1]   70 	mov	0x5216+0, #0x00
      0080E4 81               [ 4]   71 	ret
                                     72 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 16: void i2c_start() {
                                     73 ;	-----------------------------------------
                                     74 ;	 function i2c_start
                                     75 ;	-----------------------------------------
      0080E5                         76 _i2c_start:
                                     77 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 18: I2C_CR2 |= (1 << I2C_CR2_START);
      0080E5 72 10 52 11      [ 1]   78 	bset	0x5211, #0
                                     79 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 19: i = 0;
      0080E9 4F               [ 1]   80 	clr	a
                                     81 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 20: while (!(I2C_SR1 & (1 << I2C_SR1_SB)) && i < 200)
      0080EA                         82 00102$:
      0080EA AE 52 17         [ 2]   83 	ldw	x, #0x5217
      0080ED 88               [ 1]   84 	push	a
      0080EE F6               [ 1]   85 	ld	a, (x)
      0080EF 97               [ 1]   86 	ld	xl, a
      0080F0 84               [ 1]   87 	pop	a
      0080F1 54               [ 2]   88 	srlw	x
      0080F2 24 01            [ 1]   89 	jrnc	00119$
      0080F4 81               [ 4]   90 	ret
      0080F5                         91 00119$:
      0080F5 A1 C8            [ 1]   92 	cp	a, #0xc8
      0080F7 25 01            [ 1]   93 	jrc	00120$
      0080F9 81               [ 4]   94 	ret
      0080FA                         95 00120$:
                                     96 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 21: i++;
      0080FA 4C               [ 1]   97 	inc	a
      0080FB 20 ED            [ 2]   98 	jra	00102$
      0080FD 81               [ 4]   99 	ret
                                    100 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 24: void i2c_stop() {
                                    101 ;	-----------------------------------------
                                    102 ;	 function i2c_stop
                                    103 ;	-----------------------------------------
      0080FE                        104 _i2c_stop:
                                    105 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 25: I2C_CR2 |= (1 << I2C_CR2_STOP);
      0080FE AE 52 11         [ 2]  106 	ldw	x, #0x5211
      008101 F6               [ 1]  107 	ld	a, (x)
      008102 AA 02            [ 1]  108 	or	a, #0x02
      008104 F7               [ 1]  109 	ld	(x), a
                                    110 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 26: while (I2C_SR3 & (1 << I2C_SR3_MSL));
      008105                        111 00101$:
      008105 AE 52 19         [ 2]  112 	ldw	x, #0x5219
      008108 F6               [ 1]  113 	ld	a, (x)
      008109 44               [ 1]  114 	srl	a
      00810A 25 F9            [ 1]  115 	jrc	00101$
      00810C 81               [ 4]  116 	ret
                                    117 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 29: void i2c_write(uint8_t data) {
                                    118 ;	-----------------------------------------
                                    119 ;	 function i2c_write
                                    120 ;	-----------------------------------------
      00810D                        121 _i2c_write:
                                    122 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 31: I2C_DR = data;
      00810D AE 52 16         [ 2]  123 	ldw	x, #0x5216
      008110 7B 03            [ 1]  124 	ld	a, (0x03, sp)
      008112 F7               [ 1]  125 	ld	(x), a
                                    126 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 32: i = 0;
      008113 4F               [ 1]  127 	clr	a
                                    128 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 33: while (!(I2C_SR1 & (1 << I2C_SR1_TXE)) && i < 200)
      008114                        129 00102$:
      008114 AE 52 17         [ 2]  130 	ldw	x, #0x5217
      008117 88               [ 1]  131 	push	a
      008118 F6               [ 1]  132 	ld	a, (x)
      008119 95               [ 1]  133 	ld	xh, a
      00811A 84               [ 1]  134 	pop	a
      00811B 5D               [ 2]  135 	tnzw	x
      00811C 2A 01            [ 1]  136 	jrpl	00119$
      00811E 81               [ 4]  137 	ret
      00811F                        138 00119$:
      00811F A1 C8            [ 1]  139 	cp	a, #0xc8
      008121 25 01            [ 1]  140 	jrc	00120$
      008123 81               [ 4]  141 	ret
      008124                        142 00120$:
                                    143 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 34: i++;
      008124 4C               [ 1]  144 	inc	a
      008125 20 ED            [ 2]  145 	jra	00102$
      008127 81               [ 4]  146 	ret
                                    147 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 37: void i2c_write_addr(uint8_t addr) {
                                    148 ;	-----------------------------------------
                                    149 ;	 function i2c_write_addr
                                    150 ;	-----------------------------------------
      008128                        151 _i2c_write_addr:
                                    152 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 39: I2C_DR = addr;
      008128 AE 52 16         [ 2]  153 	ldw	x, #0x5216
      00812B 7B 03            [ 1]  154 	ld	a, (0x03, sp)
      00812D F7               [ 1]  155 	ld	(x), a
                                    156 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 40: i = 0;
      00812E 4F               [ 1]  157 	clr	a
      00812F 97               [ 1]  158 	ld	xl, a
                                    159 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 41: while (!(I2C_SR1 & (1 << I2C_SR1_ADDR)) && i < 200)
      008130                        160 00102$:
      008130 90 AE 52 17      [ 2]  161 	ldw	y, #0x5217
      008134 90 F6            [ 1]  162 	ld	a, (y)
      008136 A5 02            [ 1]  163 	bcp	a, #0x02
      008138 26 08            [ 1]  164 	jrne	00104$
      00813A 9F               [ 1]  165 	ld	a, xl
      00813B A1 C8            [ 1]  166 	cp	a, #0xc8
      00813D 24 03            [ 1]  167 	jrnc	00104$
                                    168 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 42: i++;;
      00813F 5C               [ 2]  169 	incw	x
      008140 20 EE            [ 2]  170 	jra	00102$
      008142                        171 00104$:
                                    172 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 43: (void) I2C_SR3; // check BUS_BUSY
      008142 AE 52 19         [ 2]  173 	ldw	x, #0x5219
      008145 F6               [ 1]  174 	ld	a, (x)
                                    175 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 44: I2C_CR2 |= (1 << I2C_CR2_ACK);
      008146 AE 52 11         [ 2]  176 	ldw	x, #0x5211
      008149 F6               [ 1]  177 	ld	a, (x)
      00814A AA 04            [ 1]  178 	or	a, #0x04
      00814C F7               [ 1]  179 	ld	(x), a
      00814D 81               [ 4]  180 	ret
                                    181 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 47: uint8_t i2c_read() {
                                    182 ;	-----------------------------------------
                                    183 ;	 function i2c_read
                                    184 ;	-----------------------------------------
      00814E                        185 _i2c_read:
                                    186 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 48: I2C_CR2 &= ~(1 << I2C_CR2_ACK);
      00814E AE 52 11         [ 2]  187 	ldw	x, #0x5211
      008151 F6               [ 1]  188 	ld	a, (x)
      008152 A4 FB            [ 1]  189 	and	a, #0xfb
      008154 F7               [ 1]  190 	ld	(x), a
                                    191 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 49: i2c_stop();
      008155 CD 80 FE         [ 4]  192 	call	_i2c_stop
                                    193 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 50: while (!(I2C_SR1 & (1 << I2C_SR1_RXNE)));
      008158                        194 00101$:
      008158 AE 52 17         [ 2]  195 	ldw	x, #0x5217
      00815B F6               [ 1]  196 	ld	a, (x)
      00815C A5 40            [ 1]  197 	bcp	a, #0x40
      00815E 27 F8            [ 1]  198 	jreq	00101$
                                    199 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 51: return I2C_DR;
      008160 AE 52 16         [ 2]  200 	ldw	x, #0x5216
      008163 F6               [ 1]  201 	ld	a, (x)
      008164 81               [ 4]  202 	ret
                                    203 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 54: void i2c_read_arr(uint8_t *buf, int len) {
                                    204 ;	-----------------------------------------
                                    205 ;	 function i2c_read_arr
                                    206 ;	-----------------------------------------
      008165                        207 _i2c_read_arr:
      008165 52 02            [ 2]  208 	sub	sp, #2
                                    209 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 55: while (len-- > 1) {
      008167                        210 00104$:
      008167 16 07            [ 2]  211 	ldw	y, (0x07, sp)
      008169 17 01            [ 2]  212 	ldw	(0x01, sp), y
      00816B 1E 01            [ 2]  213 	ldw	x, (0x01, sp)
      00816D 5A               [ 2]  214 	decw	x
      00816E 1F 07            [ 2]  215 	ldw	(0x07, sp), x
                                    216 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 58: *(buf++) = I2C_DR;
      008170 16 05            [ 2]  217 	ldw	y, (0x05, sp)
                                    218 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 55: while (len-- > 1) {
      008172 1E 01            [ 2]  219 	ldw	x, (0x01, sp)
      008174 A3 00 01         [ 2]  220 	cpw	x, #0x0001
      008177 2D 1B            [ 1]  221 	jrsle	00106$
                                    222 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 56: I2C_CR2 |= (1 << I2C_CR2_ACK);
      008179 AE 52 11         [ 2]  223 	ldw	x, #0x5211
      00817C F6               [ 1]  224 	ld	a, (x)
      00817D AA 04            [ 1]  225 	or	a, #0x04
      00817F F7               [ 1]  226 	ld	(x), a
                                    227 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 57: while (!(I2C_SR1 & (1 << I2C_SR1_RXNE)));
      008180                        228 00101$:
      008180 AE 52 17         [ 2]  229 	ldw	x, #0x5217
      008183 F6               [ 1]  230 	ld	a, (x)
      008184 A5 40            [ 1]  231 	bcp	a, #0x40
      008186 27 F8            [ 1]  232 	jreq	00101$
                                    233 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 58: *(buf++) = I2C_DR;
      008188 AE 52 16         [ 2]  234 	ldw	x, #0x5216
      00818B F6               [ 1]  235 	ld	a, (x)
      00818C 90 F7            [ 1]  236 	ld	(y), a
      00818E 90 5C            [ 2]  237 	incw	y
      008190 17 05            [ 2]  238 	ldw	(0x05, sp), y
      008192 20 D3            [ 2]  239 	jra	00104$
      008194                        240 00106$:
                                    241 ;	/home/scott/Samples/stm8-bare-min/stm8/i2c.c: 60: *buf = i2c_read();
      008194 90 89            [ 2]  242 	pushw	y
      008196 CD 81 4E         [ 4]  243 	call	_i2c_read
      008199 90 85            [ 2]  244 	popw	y
      00819B 90 F7            [ 1]  245 	ld	(y), a
      00819D 5B 02            [ 2]  246 	addw	sp, #2
      00819F 81               [ 4]  247 	ret
                                    248 	.area CODE
                                    249 	.area INITIALIZER
                                    250 	.area CABS (ABS)
