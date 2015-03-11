/*****************************************************************************
 * uart_write.h
 *
 * Biblioteca de escrita no UART
 *
 *  Created on: 01/03/2015
 *      Author: Victor
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 *****************************************************************************/

#ifndef UART_UART_TX_H_
#define UART_UART_TX_H_

void uart_putc(unsigned char);
void uart_puts(char *);
void uart_printf(char *format, ...);

#endif /* UART_UART_TX_H_ */
