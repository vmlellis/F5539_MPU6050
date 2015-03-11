/*
 * uart_rx.c
 *
 *  Created on: 02/03/2015
 *      Author: Victor
 */

#include "uart_rx.h"
#include <msp430.h>

extern void uart_rx_auxiliar(uint8_t c);

uint8_t bufferRX[SERIAL_BUFFER_SIZE];
volatile uint8_t headBuffer = 0;
volatile uint8_t tailBuffer = 0;

/*
 * Interrupção RX do UART
 * Desc: Salva os caracteres no buffer
 */
void uart_rx() {
	while (!(UCA1IFG&UCTXIFG));         // USCI_A0 TX buffer ready?
	uint8_t c = UCA1RXBUF;              // TX -> RXed character
	uint8_t i = (headBuffer + 1) % SERIAL_BUFFER_SIZE;

	if (i != tailBuffer) {
		bufferRX[headBuffer] = c;
		headBuffer = i;
	}

	uart_rx_auxiliar(c);
}

/*
 * Le um caractere do buffer
 */
int16_t uart_read() {
	if (headBuffer == tailBuffer)
		return -1;
	uint8_t c = bufferRX[tailBuffer];
	tailBuffer = (tailBuffer + 1) % SERIAL_BUFFER_SIZE;
	return c;
}

/*
 * Ler bytes
 * Input	buffer: ponteiro para o buffer
 * 			length: tamanho do buffer
 * 	Output: Numero de caracteres lidos
 */
uint8_t uart_readBytes(char *buffer, uint8_t length) {
	if (length < 1) return 0;
	uint8_t count = 0;
	while (count < length) {
		int16_t c = uart_read();
		if (c == -1) break;
		*buffer++ = c;
		count++;
	}
	return count;
}


/*
 * Ler bytes ateh um caractere
 * Input	terminator: caractere ateh onde deve ser realizado a leitura
 * 			buffer: ponteiro para o buffer
 * 			length: tamanho do buffer
 * 	Output: Numero de caracteres lidos (sem o terminator)
 */
uint8_t uart_readBytesUntil(char terminator, char *buffer, uint8_t length) {
	if (length < 1) return 0;
	uint8_t count = 0;
	while (count < length) {
		__int16_t c = uart_read();
		if (c == -1 || c == terminator) break;
		*buffer++ = c;
		count++;
	}
	return count;
}

/*
 * Informa se contem dados lidos no buffer
 */
uint8_t uart_available() {
	return (SERIAL_BUFFER_SIZE + headBuffer - tailBuffer) % SERIAL_BUFFER_SIZE;
}

