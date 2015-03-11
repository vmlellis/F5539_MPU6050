/*
 * uart_rx.h
 *
 *  Created on: 01/03/2015
 *      Author: Victor
 */

#ifndef UART_UART_RX_H_
#define UART_UART_RX_H_

/*#include <stdlib.h>*/
#include <inttypes.h>

#define SERIAL_BUFFER_SIZE 16

int16_t uart_read(void);
uint8_t uart_readBytes(char *buffer, uint8_t length);
uint8_t uart_readBytesUntil(char terminator, char *buffer, uint8_t length);
uint8_t uart_available();

#endif /* UART_UART_RX_H_ */
