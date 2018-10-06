#include <stdio.h>
#include <string.h>
#include <Hstm8/stm8s.h>
#include <delay.h>
#include <uart.h>
#include "interrupts.h"
#include "build_defs.h"

/**
 * PD1 -> digital input
 * PD2 -> digital input
 * PD3 -> analog input
 * PD4 -> RS485 DE/RE
 * PD5 (tx) -> RS485 DI
 * PD6 (rx) -> RS485 RO
 * PC4 -> digital input
 * PC5 -> digital input
 * PC6 -> digital input
 * PC7 -> digital input
 */
#define rs485xmit_on() GPIOD->ODR |= GPIO_PIN_4
#define rs485xmit_off() GPIOD->ODR &= ~(GPIO_PIN_4)
#define reset_watchdog() IWDG->KR = 0xaa
#define V_REF 5.0
#define VOLTAGE_OFFSET 120


volatile unsigned long Global_time; // global time in ms
// uint8_t buf[20];
uint8_t esc, address;
uint8_t version[12];

int putchar(int c) {
    uart_write(c);
    return c;
}

//--------------------------------------------------------------------------------
//
//  Setup the system clock to run at 16MHz using the internal oscillator.
//
void InitialiseSystemClock()
{
    CLK->ICKR = 0;                       //  Reset the Internal Clock Register.
    CLK->ICKR = CLK_ICKR_HSIEN;          //  Enable the HSI.
    CLK->ECKR = 0;                       //  Disable the external clock.
    while (!(CLK->ICKR & CLK_ICKR_HSIRDY)); //  Wait for the HSI to be ready for use.
    CLK->CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
    CLK->PCKENR1 = 0xff; //CLK_PCKENR1_TIM4 | CLK_PCKENR1_UART1 | CLK_PCKENR1_SPI | CLK_PCKENR1_I2C ;  //  Enable select peripheral clocks.
    CLK->PCKENR2 = 0xff; //CLK_PCKENR2_AWU;      //  Only enable the AWU watchdog service
    CLK->CCOR = 0;                       //  Turn off CCO.
    CLK->HSITRIMR = 0;                   //  Turn off any HSIU trimming.
    CLK->SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
    CLK->SWR = 0xe1;                     //  Use HSI as the clock source.
    CLK->SWCR = 0;                       //  Reset the clock switch control register.
    CLK->SWCR |= CLK_SWCR_SWEN;          //  Enable switching.
    while (CLK->SWCR & CLK_SWCR_SWBSY);  //  Pause while the clock switch is busy.
}
//--------------------------------------------------------------------------------
//
//  Initialize the Independent Watchdog (IWDG)
//
void InitialiseIWDG()
{
    IWDG->KR = 0xcc;         //  Start the independent watchdog.
    IWDG->KR = 0x55;         //  Allow the IWDG registers to be programmed.
    IWDG->PR = 0x06;         //  Prescaler is 6 => each count is 1.02 second with RLR = 0xff
    IWDG->RLR = 0xff;        //  Reload counter.  T = 2 x TLSI x PR x R LR
    IWDG->KR = 0xaa;         //  Reset the counter.
}


uint16_t ADC_read() {
    uint8_t adcH, adcL;
    ADC1->CR1 |= ADC1_CR1_ADON;
    while (!(ADC1->CSR & (ADC1_CSR_EOC)));
    adcL = ADC1->DRL;
    adcH = ADC1->DRH;
    ADC1->CSR &= ~(ADC1_CSR_EOC); // Clear EOC flag
    return (adcL | (adcH << 8));
}
void ADC_init() {
    /* Configure ADC channel 4 (PD3)  */
    ADC1->CSR |= (1 << 2);
    /* Right-align data */
    ADC1->CR2 |= ADC1_CR2_ALIGN;
    /* Wake ADC from power down */
    ADC1->CR1 |= ADC1_CR1_ADON;
}
void main() {
	unsigned long esc_time = 0L;
	uint16_t PSI;
	uint16_t psi1, psi2;
	uint8_t i;
	uint8_t s[7];

	uint8_t rb;
	uint16_t voltage;
	esc = 0;

	sprintf(version,"%02d%02d%02d-%02d%02d", BUILD_YEAR - 2000, BUILD_MONTH, BUILD_DAY, BUILD_HOUR, BUILD_MIN);

	disableInterrupts();
	InitialiseSystemClock();
    //GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIOD->CR2 &= (uint8_t)(~(GPIO_PIN_4));
    GPIOD->ODR &= (uint8_t)(~(GPIO_PIN_4));
    GPIOD->DDR |= (uint8_t)GPIO_PIN_4;
    GPIOD->CR1 |= (uint8_t)GPIO_PIN_4;
    GPIOD->CR2 |= (uint8_t)GPIO_PIN_4;

    // setup the input pins to float high.  an external relay pulls them low to indicate something
    // pins D1, D2, C4, C5, C6 and C7 are used as on-off inputs
    GPIOD->DDR &= ~(GPIO_PIN_2 | GPIO_PIN_1);  // input mode
    GPIOD->CR1 |= (GPIO_PIN_1 | GPIO_PIN_2);  // pull-ups
    GPIOD->CR2 &= ~(GPIO_PIN_2 | GPIO_PIN_1);  // no interrupts
    GPIOC->DDR &= ~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7); //  input mode
    GPIOC->CR1 |= (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);  // no interrupts
    GPIOC->CR2 &= ~(GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);  // no interrupts


    rs485xmit_off();
	CFG->GCR |= 1; // disable SWIM

    // Timer 4 (8 bit) used as system tick timer
	TIM4->PSCR = 7;   // prescaler
	TIM4->ARR = 125;  // auto reload register
	// interrupts: update
	TIM4->IER = TIM4_IER_UIE;
	// auto-reload + interrupt on overflow + enable
	TIM4->CR1 = TIM4_CR1_ARPE | TIM4_CR1_URS | TIM4_CR1_CEN;

	Global_time = 0L;
    uart_init();		// initialize the uart functions - 9600 8-N-1 through RS485
    // enable all interrupts
    enableInterrupts();

    address = 'S';		// this devices id character

    ADC_init();			// initialize the analog read function
    delay_ms(800);
	rs485xmit_on();	// turn the RS485 chips transmitter on
	delay_ms(30);
	printf("%c:Running:%s:%02x\r\n",address,version,address);
	delay_ms(10);
	rs485xmit_off(); // turn the transmitter back off
	InitialiseIWDG();
	reset_watchdog();  // reset the watchdog timer

    // Loop
    do{
    	reset_watchdog();  // reset the watchdog timer
	    if(UART_read_byte(&rb)){ // buffer isn't empty
		    switch(rb){
			    case 0x1b: // escape
				    esc = 1;
				    esc_time = Global_time;	// only wait two seconds for the next character after the escape
				    break;
			    default:
				    if (rb == address && esc)  // address must match the switches read by mcp23017
				    {
				    	Global_time = 0L;   // when was the last time we were called?
				    	voltage = ADC_read();	// get the analog value from the pressure sensor
				    	if (voltage < VOLTAGE_OFFSET)		// the sensor range is from 0.5 volts to 4.5 volts
				    		voltage = VOLTAGE_OFFSET;
				    	PSI = (uint16_t)((voltage - VOLTAGE_OFFSET) * 1.15);  // formula to calculate PSI
				    	// SDCC cannot handle printing floats in a device this small
				    	psi1 = (uint16_t)(PSI / 100);
				    	psi2 = (uint16_t)(PSI - (psi1 * 100));
				    	strcpy(s,"0000aa");
				    	for (i = 0; i < 4; i++)		// read through the on-off inputs
				    	{
				    		if (!(GPIOC->IDR & (GPIO_PIN_4 << i)))
				    			s[i] = '1';		// if the gpio is low, that means on
				    	}
				    	if (!(GPIOD->IDR & GPIO_PIN_1))	// these are for the air conditioners
				    		s[4] = 'A';
				    	if (!(GPIOD->IDR & GPIO_PIN_2))
				    		s[5] = 'A';
				    	s[6] = 0;		// terminate the string
				    	rs485xmit_on();	// turn the RS485 chips transmitter on
				    	delay_ms(30);	// make sure transmitter has time to turn on
				    	printf("%c:%02d.%02d:%05d:%s:\r\n", address,psi1,psi2,voltage,s); // S:23.56:89012:0101Aa:
				    	delay_ms(10);	// give it time to transmit before turning transmitter off
				    	rs485xmit_off(); // turn the transmitter back off
				    }
				    esc = 0;	// reset the flag for the escape character
		    }
	    }
	    if (esc && (Global_time - esc_time > 2000))  // give it 2 seconds to send the id character
	    	esc = 0;  // reset the esc flag, since it should have been followed by the id right away
    }while(1);
}
