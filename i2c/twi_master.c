/*****************************************************************************
 * twi_master.c
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

#include "twi_master.h"
#include "../setup.h"

extern volatile int counter;

//volatile uint8_t wakeUpI2C = 0;  // Para sair do modo de baixo consumo
extern void wakeUpI2C();

static volatile uint8_t twi_sendStop;  // Flag para a transação terminar com um stop
static volatile uint8_t twi_error;	   // Para os erros

static uint8_t twi_masterBuffer[TWI_BUFFER_LENGTH]; // Buffer
static volatile uint8_t twi_masterBufferIndex; 		// Index do buffer (volatil pois eh incrementado na interrupção)
static uint8_t twi_masterBufferLength;				// Tamnho dos dados a serem enviados ou recebidos

/*
 * Função twi_master_init
 * Desc     Inicia com os valores iniciais
 * Input    none
 * Output   none
 */
void twi_master_init(void)
{
	twi_sendStop = 1;		// default value

    /* Seta as interrupções TX/RX */
	UCB0IE |= (UCRXIE|UCTXIE);
	//UCB0IE |= (UCNACKIE|UCRXIE|UCTXIE);
	//UCB0IE |= (UCALIE|UCNACKIE|UCSTTIE|UCSTPIE|UCRXIE|UCTXIE);
}

/*
 * Function twi_master_writeTo
 * Desc     Escreve uma serie de bytes de um dispositivo no barramento
 * Input    address: endereço de 7bit do dispositivo i2c
 *          data: ponteiro para o array de bytes
 *          length: Numero de bytes no array
 *          sendStop: indica seh eh para enviar uma condição STOP no final
 * Output   0 .. Sucesso
 *          1 .. tamanho muito grande para o buffer
 *          2 .. dados enviados, NACK recebido
 */
uint8_t twi_master_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop)
{
	uint8_t i;
	twi_error = TWI_ERRROR_NO_ERROR;
	twi_sendStop = sendStop;

	UCB0CTL1 |= UCTR;           // Configura como modo de trasnmissão
	UCB0I2CSA = address;        // Seta o endeço do dispositivo escravo

	// Se o tamanho eh zero retona sucesso, nenhum dado para ser enviado
	if(length == 0) {
		return 0;
	}

	/* Garante se os dados se encaixam no buffer */
	if(length > TWI_BUFFER_LENGTH){
		return TWI_ERROR_BUF_TO_LONG;
	}

	/* Inicializa as variaveis para o controle do buffer */
	twi_masterBufferIndex = 0;
	twi_masterBufferLength = length;

	// Insere os dados no buffer
	for(i = 0; i < length; ++i){
		twi_masterBuffer[i] = data[i];
	}

	/* Condição de start do I2C  */
    UCB0CTL1 |= UCTXSTT;

	/* Aguarda a transação completar */
	__bis_SR_register(LPM0_bits + GIE);

	/* Garante que a condição stop/start foi enviada. */
	if(sendStop)
	{
		while (UCB0CTL1 & UCTXSTP);	// aguarda finalizar com uma condição stop
	}
	else
	{
		while (UCB0CTL1 & UCTXSTT);	// aguarda finalizar com uma condição (re)start
	}

	return twi_error;
}

/*
 * Function twi_master_readFrom
 * Desc     Le uma serie de bytes de um dispositivo no barramento
 * Input    address: endereço de 7bit do dispositivo i2c
 *          data: ponteiro para o array de bytes
 *          length: Numero de bytes para ler
 * Output   número de bytes lidos
 */
uint8_t twi_master_readFrom(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop)
{
	uint8_t i;

    UCB0CTL1 &= ~(UCTR);        // Configure in receive mode
    UCB0I2CSA = address;        // Seta o endeço do dispositivo escravo

    /* Garante se os dados se encaixam no buffer */
	if(TWI_BUFFER_LENGTH < length) {
		return 0;
	}

	/* Inicializa as variaveis para o controle do buffer */
	twi_masterBufferIndex = 0;
	twi_masterBufferLength = length-1;  // Abaixo a explicação da causa do (-1)
	/*
	 * Explicação:
	 * O NACK eh transmitido em resposta ao ultimo byte recebido.
	 * Quando o ultimo byte eh enviado deve ser sinalizado o NACK anteriormente.
	 * A logica pode ser visualizar na função i2c_rx
	 */

	/* Condição de start do I2C  */
    UCB0CTL1 |= UCTXSTT;

    /* Quando somente recebe 1 byte deve enviar a condição de STOP para garantir o NACK */
    if(length == 1) {
        while(UCB0CTL1 & UCTXSTT);	// Aguarda o bit de start ser enviado
        UCB0CTL1 |= UCTXSTP;        // Envia a condição de stop
    }

    /* Aguarda no modo de baixo consumo a transação de leitura completar */
    __bis_SR_register(LPM0_bits + GIE);

    /* Garante a condição de stop */
    while (UCB0CTL1 & UCTXSTP);

    /* Registra a quantidade de bytes lidos */
	if (twi_masterBufferIndex < length)
		length = twi_masterBufferIndex;

	/* Completa o buffer data com os dados lidos */
	for(i = 0; i < length; i++){
		data[i] = twi_masterBuffer[i];
	}

	return length;
}


/*
 * Interrupção USCI_B0 I2C TX
 * Transmite caracteres
 * UCB0TXIFG eh setado quando UCB0TXBUF estah vazio.
 */
void i2c_tx(void) {
	// se existe dados para enviar, envie, senão pare
	if(twi_masterBufferIndex < twi_masterBufferLength){
		// Copia os dados para o registrados de saida.
		UCB0TXBUF = twi_masterBuffer[twi_masterBufferIndex++];
	}else{
		if (twi_sendStop) {
			/* Gera uma condição de stop */
			UCB0CTL1 |= UCTXSTP;
		} else {
			/*
			 * Gera um condição de start
			 * Obs.: Não habilite a interrupção.
			 * Eh gerado o start, mas evite a interrupção ateh estar na proxima transação
			 */
			UCB0CTL1 |= UCTXSTT;
		}
		UCB0IFG &= ~UCTXIFG;
	}
}

/*
 * Interrupção USCI_B0 I2C RX
 * UCB0RXIFG eh setado qando UCB0RXBUF recebe um caracter.
 */
void i2c_rx(void) {
	twi_masterBuffer[twi_masterBufferIndex++] = UCB0RXBUF;

	if(twi_masterBufferIndex == twi_masterBufferLength ) {
		/* Only one byte left. Generate STOP condition.
		 * In master mode a STOP is preceded by a NACK */
		UCB0CTL1 |= UCTXSTP;
	}

	/* Todos os bytes recebidos */
	if(twi_masterBufferIndex > twi_masterBufferLength ) {
		wakeUpI2C();
	}
}

/*
 * Interrupção Not-acknowledge.
 * UCNACKIFG eh automaticamente limpo quando uma condição START eh recebida.
 */
/*void i2c_nack(void) {
	UCB0IFG &= ~UCNACKIFG;

	UCB0CTL1 |= UCTXSTP;
	twi_error = TWI_ERROR_DATA_NACK;
	wakeUpI2C();
}*/

/*
 * Função twi_master_writeRegister
 * Desc     Escrever em um registro de um dispositivo
 * Input    devAddr: endereço de 7bit do dispositivo i2c
 * 			uint8_t: endereço de 7bit do registrador
 *          data: ponteiro para o array de bytes
 *          length: Numero de bytes no array
 * Output   none
 */
void twi_master_writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint8_t length)
{
	uint8_t buffer[length+1], i = 0;
	buffer[0] = regAddr;
	for (i = 0; i < length; i++) {
		buffer[i+1] = data[i];
	}
	twi_master_writeTo(devAddr, buffer, sizeof(buffer), 1);
}

/*
 * Função twi_master_readRegister
 * Desc     Leitura de um registro em um dispositivo no barramento
 * Input    devAddr: endereço de 7bit do dispositivo i2c
 * 			uint8_t: endereço de 7bit do registrador
 *          data: ponteiro para o array de bytes
 *          length: Numero de bytes no array
 * Output   número de bytes lidos
 */
uint8_t twi_master_readRegister(uint8_t devAddr, uint8_t regAddr, uint8_t* data, uint8_t length)
{
	twi_master_writeTo(devAddr, &regAddr, sizeof(regAddr), 1);
	return twi_master_readFrom(devAddr, data, length, 1);
}

/*
 * Função twi_master_read16
 * Desc     Ler 2 bytes de um registrador
 * Input    devAddr: endereço de 7bit do dispositivo i2c
 * 			uint8_t: endereço de 7bit do registrador
 * Output   2 bytes lidos do registrador
 */
uint16_t twi_master_read16(uint8_t devAddr, uint8_t regAddr) {
	uint8_t msb, lsb;
	uint8_t buffer[2] = { 0, 0 };
	twi_master_readRegister(devAddr, regAddr, buffer, sizeof(buffer));
	msb = buffer[0];
	lsb = buffer[1];
	return ((uint16_t) msb)<<8 | lsb;
}

/*
 * Função twi_master_writeBits
 * Desc     Escreve em bits definidos
 * Input    devAddr: endereço de 7bit do dispositivo i2c
 * 			uint8_t: endereço de 7bit do registrador
 * 			uint8_t bit_start: bit de inicio
 * 			uint8_t length: tamanho
 * 			uint8_t source: dados
 * Output   2 bytes lidos do registrador
 */
void twi_master_writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bit_start, uint8_t length, uint8_t data) {
	// Ler o byte
	uint8_t b;
	twi_master_readRegister(devAddr, regAddr, &b, 1);

	// Apagar os bits
	uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
	b &= ~(mask);

	// Setar os bits
	b |= write_bits(bit_start, length, data);
	twi_master_writeRegister(devAddr, regAddr, &b, 1);
}


/*
 * Função twi_master_writeBits
 * Desc     Escreve em um bit definidp
 * Input    devAddr: endereço de 7bit do dispositivo i2c
 * 			uint8_t: endereço de 7bit do registrador
 * 			uint8_t bit: bit
 * 			uint8_t source: valor do bit (0 ou 1)
 * Output   2 bytes lidos do registrador
 */
void twi_master_writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bit, uint8_t data) {
	twi_master_writeBits(devAddr, regAddr, bit, 1, data);
}

