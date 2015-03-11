/*****************************************************************************
 * uart_write.h
 *
 * Codigo de escrita no UART
 *
 *  Created on: 01/03/2015
 *      Author: Victor
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 *****************************************************************************/

#include "uart_tx.h"
#include <msp430.h>
#include <stdarg.h>
#include <math.h>

static const unsigned long dv[] = {
//  4294967295      // 32 bit unsigned max
		1000000000, // +0
		100000000, // +1
		10000000, // +2
//  8388607         // 23 bit unsigned max
		1000000, // +3
		100000, // +4
//  65535           // 16 bit unsigned max
		10000, // +5
		1000, // +6
//  255             // 8 bit unsigned max
		100, // +7
		10, // +8
		1, // +9
};

/**
 * puts() é utilizado pelo printf() para enviar um caractere. Esta função
 *     determina onde será mostrado o caractere.
 **/
void uart_putc(unsigned char byte) {
  while (!(UCA1IFG&UCTXIFG));   // USCI_A0 TX buffer ready?
  if (UCA1IFG & UCTXIFG)
  {
	  UCA1TXBUF = byte;    // TX -> RXed character
  }
}

/**
 * puts() é utilizado pelo printf() para enviar uma string. Esta função
 *     determina onde será mostrado a string.
 **/
void uart_puts(char *s) {
  char c;

  // Loops through each character in string 's'
  while ((c = *s++)) {
	  uart_putc(c);
  }
}

/*
 * Imprimir inteiros
 * Converte inteiros para strings ASCII
 */
static void xtoa(unsigned long x, const unsigned long *dp) {
	char c;
	unsigned long d;
	if (x) {
		while (x < *dp)
			++dp;
		do {
			d = *dp++;
			c = '0';
			while (x >= d)
				++c, x -= d;
			uart_putc(c);
		} while (!(d & 1));
	} else
		uart_putc('0');
}

/*
 * Imprimir caracter hexadecimal
 */
static void puth(unsigned n) {
	static const char hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8',
			'9', 'A', 'B', 'C', 'D', 'E', 'F' };
	uart_putc(hex[n & 15]);
}

// Para imprimir os decimais de um ponto flutuante
static void xtoa_decimals(float f, int decimals)
{
  int pos = 0, value = 0, multiplier = 1;
  while (pos < decimals)
  {
	multiplier *= 10;
	pos++;
  }
  value = (int)f;
  f = f - (float)value;
  value = (int) floorf(f * multiplier + 0.5);
  xtoa((unsigned long)value, dv + 3); // 23 bits
}

// Para imprimir ponto fluante
static void ftoa(float f, int decimals)
{
  float f_temp = f;
  int i = (int) f;
  if(i < 0) i = -i, f_temp = -f_temp, uart_putc('-');
  xtoa((unsigned)i, dv + 7); // 8 bits
  uart_putc('.');
  xtoa_decimals(f_temp, decimals);
}

// Funncao para imprimir
void uart_printf(char *format, ...)
{
  char c;
  int i, decimals;
  long n;
  float f;

  va_list a;
  va_start(a, format);
  while((c = *format++)) {
    if(c == '%') {
      switch(c = *format++) {
        case 's': // String
          uart_puts(va_arg(a, char*));
          break;
        case 'c':// Char
          uart_putc((char)va_arg(a, int));
          break;
        case 'i':// 16 bit Integer
        case 'u':// 16 bit Unsigned
          i = va_arg(a, int);
          if(c == 'i' && i < 0) i = -i, uart_putc('-');
          xtoa((unsigned)i, dv + 5);
          break;
        case 'l':// 32 bit Long
        case 'n':// 32 bit uNsigned loNg
          n = va_arg(a, long);
          if(c == 'l' && n < 0) n = -n, uart_putc('-');
          xtoa((unsigned long)n, dv);
          break;
        case 'x':// 16 bit heXadecimal
          i = va_arg(a, int);
          puth(i >> 12);
          puth(i >> 8);
          puth(i >> 4);
          puth(i);
          break;
        case 'f': // Float
          f = va_arg(a, double);
          ftoa(f, 2);
          break;
        case '.': // Para informar as casas decimais
          c = *format++;
          decimals = 0;
          while ('0' <= c &&  c <= '9') {
            decimals = decimals*10 + (c - '0');
            c = *format++;
          }
          if (c == 'f') { // Float
            f = va_arg(a, double);
            ftoa(f,decimals);
          }
          break;
        case 0: return;
        default: goto bad_fmt;
      }
    } else
      bad_fmt: uart_putc(c);
  }
  va_end(a);
}
