/*****************************************************************************
 * twi_master.h
 *
 * Two Interface Wire (MASTER)
 * Descrição: Biblioteca utilizada para comunicacao I2C como mestre
 *
 *  Criado em: 28/02/2015
 *      Autor: Victor
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 *  Baseado na biblioteca twi.h da Energia
 *****************************************************************************/

#ifndef twi_master_h
#define twi_master_h


#include <msp430.h>
#include <inttypes.h>

#define TWI_BUFFER_LENGTH 16 	// Tamanho do buffer

// Tipos de errors
#define TWI_ERRROR_NO_ERROR 	0
#define TWI_ERROR_BUF_TO_LONG 	1
#define TWI_ERROR_DATA_NACK 	2

void twi_master_init(void);
uint8_t twi_master_readFrom(uint8_t, uint8_t*, uint8_t, uint8_t);
uint8_t twi_master_writeTo(uint8_t, uint8_t*, uint8_t, uint8_t);
void twi_master_writeRegister(uint8_t, uint8_t, uint8_t*, uint8_t);
uint8_t twi_master_readRegister(uint8_t, uint8_t, uint8_t*, uint8_t);
uint16_t twi_master_read16(uint8_t, uint8_t);
void twi_master_writeBits(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
void twi_master_writeBit(uint8_t, uint8_t, uint8_t, uint8_t);

#endif


