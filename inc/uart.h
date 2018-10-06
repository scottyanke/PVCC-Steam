#ifndef UART_H
#define UART_H

#include <stdint.h>

#ifndef BAUDRATE
#define BAUDRATE 9600
#endif

#ifndef F_CPU
#warning "F_CPU not defined, using 2MHz by default"
#define F_CPU 2000000UL
#endif

#define UART_BUF_LEN 30			// max 7 bytes received in on operation
extern uint8_t UART_rx[UART_BUF_LEN];
extern uint8_t UART_rx_start_i;
extern uint8_t UART_rx_cur_i;
extern volatile unsigned long Global_time; // global time in ms

/**
 * Initialize UART1.
 * Mode: 8-N-1, flow-control: none.
 *
 * PD5 -> TX
 * PD6 -> RX
 */
void uart_init();

void uart_write(uint8_t data);

uint8_t uart_read();
uint8_t UART_read_byte(uint8_t *byte);
#define check_UART_pointer(x)  do{if(x == UART_BUF_LEN) x = 0;}while(0)
#endif /* UART_H */
