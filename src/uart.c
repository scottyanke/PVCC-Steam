#include "uart.h"
#include "Hstm8/stm8s.h"

uint8_t UART_rx[UART_BUF_LEN]; // cycle buffer for received data
uint8_t UART_rx_start_i = 0;   // started index of received data (from which reading starts)
uint8_t UART_rx_cur_i = 0;     // index of current first byte in rx array (to which data will be written)

void uart_init() {
	uint8_t tmp;
    tmp = UART1->SR;
    tmp = UART1->DR;
    UART1->CR1 = 0x00;
    /* madness.. This is actually 9600 @ 16Mhz*/
    UART1->BRR2 = 0x03; //((div >> 8) & 0xF0) + (div & 0x0F);
    UART1->BRR1 = 0x6a; //div >> 4;
    UART1->CR2 = UART1_CR2_TEN | UART1_CR2_REN | UART1_CR2_RIEN; // Allow RX/TX, generate ints on rx
}

void uart_write(uint8_t data) {
    UART1->DR = data;
    while (!(UART1->SR & UART1_SR_TC)) ;
    UART1->SR &= ~(UART1_SR_TC);
}

uint8_t uart_read() {
	//uint8_t i;
	//i = 0;
    while (!(UART1->SR & UART1_SR_RXNE)) ;
    return UART1->DR;
}

/**
 * Read one byte from Rx buffer
 * @param byte - where to store read data
 * @return 1 in case of non-empty buffer
 */
uint8_t UART_read_byte(uint8_t *byte){
	if(UART_rx_start_i == UART_rx_cur_i) // buffer is empty
		return 0;
	*byte = UART_rx[UART_rx_start_i++];
	check_UART_pointer(UART_rx_start_i);
	return 1;
}
