/*****************************************************************************
 * config.h
 *
 * Arquivo para a configuração
 *
 *  Criado em: 28/02/2015
 *      Autor: Victor
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 *****************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

#include <msp430.h>
#include <inttypes.h>

#define F_CPU 25000000L   	// Frequencia da CPU: 25 MHz
#define BAUD_RATE 115200L	// Baud Rate do UART: 115200
#define I2C_FREQ 400000L  	// Frequencia do I2C: 400 kHz

#define BMP085_OSS 3	// Ultra High Resolution (Delay de ~ 26ms na leitura)

/*
 * Definições para controle do clock
 */
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

/*
 * Definições para o WatchDog
 */
#define TICKS_PER_WDT_OVERFLOW 8192

// the whole number of microseconds per WDT overflow
#define MICROSECONDS_PER_WDT_OVERFLOW (clockCyclesToMicroseconds(TICKS_PER_WDT_OVERFLOW))

// o número em milissegundos para o estouro do WDT
#define MILLIS_INC (MICROSECONDS_PER_WDT_OVERFLOW / 1000)

// numero fracional de milisegundos para o overflow de WDT
#define FRACT_INC (MICROSECONDS_PER_WDT_OVERFLOW % 1000)
#define FRACT_MAX 1000

void disableWatchDog(void);
void enableWatchDog(void);
void saveUsbPower(void);
void initClocks(void);
void setupUart(void);
void setupI2C(void);
void delay(uint32_t);
void delayMicroseconds(uint32_t);
uint8_t read_bits(uint8_t, uint8_t, uint8_t);
uint8_t write_bits(uint8_t, uint8_t, uint8_t);

#endif /* CONFIG_H_ */
