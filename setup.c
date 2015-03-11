/*****************************************************************************
 * setup.c
 *
 * Metodos para configuração
 *
 *  Criado em: 28/02/2015
 *      Autor: Victor
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 *  Baseado na biblioteca wiring.h da Energia
 *****************************************************************************/

#include "setup.h"
#include "i2c/twi_master.h"
//#include "i2c/twi.h"

extern volatile unsigned long wdt_overflow_count;

/*
 * Desabilita o temporizador WatchDog (WDT)
 */
void disableWatchDog() {
	WDTCTL = WDTPW | WDTHOLD;
}

/*
 * Habilita o WatchDog (WDT)
 */
void enableWatchDog() {
	/*
	 * WDT Password + WDT interval mode + Watchdog clock source /8192 for F_CPU > 8MHz + source from SMCLK.
	 * Note that we WDT is running in interval mode. WDT will not trigger a reset on expire in this mode.
	 */
	WDTCTL = WDTPW | WDTTMSEL | WDTCNTCL | WDT_MDLY_8;

	/* WDT interrupt enable */
	SFRIE1 |= WDTIE;
}

void saveUsbPower() {
	/* Enable access to USB registers */
	USBKEYPID = 0x9628;
	/* Disable the VUSB LDO and SLDO to save power */
	USBPWRCTL &= ~(SLDOEN+VUSBEN);
	/* Disable access to USB registers */
	USBKEYPID = 0x9600;
}

void enableXtal() {
	/* LFXT can take up to 1000ms to start.
	 * Go to the loop below 4 times for a total of 2 sec timout.
	 * If a timeout happens due to no XTAL present or a faulty XTAL
	 * the clock system will fall back to REFOCLK (~32kHz) */
	P5SEL |= BIT4 + BIT5;
	/* Set XTAL2 pins to output to reduce power consumption */
	P5DIR |= BIT2 + BIT3;
	/* Turn XT1 ON */
	UCSCTL6 &= ~(XT1OFF);
	/* Set XTAL CAPS to 12 pF */
	UCSCTL6 |= XCAP_3;

	uint16_t timeout = 0x4;
	do {
		timeout--;
		/* Clear Oscillator fault flags */
		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
		/* Clear the Oscillator fault interrupt flag */
		SFRIFG1 &= ~OFIFG;
		/* @ 1MHz startup: delay for 500ms */
		__delay_cycles(500000L * (F_CPU/1000000L));
		if(!timeout) break;
	/* Test the fault flag */
	}while (SFRIFG1 & OFIFG);
	/* Reduce drive strength to reduce power consumption */
	UCSCTL6 &= ~(XT1DRIVE_3);

}

void initClocks(void) {
	 PMMCTL0_H = PMMPW_H;             // open PMM
	 SVSMLCTL &= ~SVSMLRRL_7;         // reset
	 PMMCTL0_L = PMMCOREV_0;          //

	 PMMIFG &= ~(SVSMLDLYIFG|SVMLVLRIFG|SVMLIFG);  // clear flags
	 SVSMLCTL = (SVSMLCTL & ~SVSMLRRL_7) | SVSMLRRL_1;
	 while ((PMMIFG & SVSMLDLYIFG) == 0); // wait till settled
	 while ((PMMIFG & SVMLIFG) == 0); // wait for flag
	 PMMCTL0_L = (PMMCTL0_L & ~PMMCOREV_3) | PMMCOREV_1; // set VCore for lext Speed
	 while ((PMMIFG & SVMLVLRIFG) == 0);  // wait till level reached

	 PMMIFG &= ~(SVSMLDLYIFG|SVMLVLRIFG|SVMLIFG);  // clear flags
	 SVSMLCTL = (SVSMLCTL & ~SVSMLRRL_7) | SVSMLRRL_2;
	 while ((PMMIFG & SVSMLDLYIFG) == 0); // wait till settled
	 while ((PMMIFG & SVMLIFG) == 0); // wait for flag
	 PMMCTL0_L = (PMMCTL0_L & ~PMMCOREV_3) | PMMCOREV_2; // set VCore for lext Speed
	 while ((PMMIFG & SVMLVLRIFG) == 0);  // wait till level reached

	 PMMIFG &= ~(SVSMLDLYIFG|SVMLVLRIFG|SVMLIFG);  // clear flags
	 SVSMLCTL = (SVSMLCTL & ~SVSMLRRL_7) | SVSMLRRL_3;
	 while ((PMMIFG & SVSMLDLYIFG) == 0); // wait till settled
	 while ((PMMIFG & SVMLIFG) == 0); // wait for flag
	 PMMCTL0_L = (PMMCTL0_L & ~PMMCOREV_3) | PMMCOREV_3; // set VCore for lext Speed
	 while ((PMMIFG & SVMLVLRIFG) == 0);  // wait till level reached
	 SVSMHCTL &= ~(SVMHE+SVSHE);         // Disable High side SVS
	 SVSMLCTL &= ~(SVMLE+SVSLE);         // Disable Low side SVS

	 PMMCTL0_H = 0;                   // lock PMM

	 UCSCTL0 = 0;                     // set lowest Frequency

	 // Configuração para 25Mhz
	 #if F_CPU == 25000000L
		 UCSCTL1 = DCORSEL_6;             //Range 6
		 UCSCTL2 = 0x1176;                //Loop Control Setting
		 UCSCTL3 = SELREF__REFOCLK;       //REFO for FLL
		 UCSCTL4 = SELA__XT1CLK|SELM__DCOCLK|SELS__DCOCLK;  //Select clock sources
		 UCSCTL7 &= ~(0x07);               //Clear Fault flags
	 #endif

     // Configuração para 16MHz
	 #if F_CPU == 16000000L
		 UCSCTL1 = DCORSEL_6;             //Range 6
		 UCSCTL2 = 0x11E7;                //Loop Control Setting
		 UCSCTL3 = SELREF__REFOCLK;       //REFO for FLL
		 UCSCTL4 = SELA__XT1CLK|SELM__DCOCLKDIV|SELS__DCOCLKDIV;  //Select clock sources
		 UCSCTL7 &= ~(0x07);               //Clear Fault flags
	 #endif

	 // This part is required for FR59xx device to unlock the IOs
	 PMMCTL0_H = PMMPW_H;           // open PMM
	 PM5CTL0 &= ~LOCKLPM5;          // clear lock bit for ports
	 PMMCTL0_H = 0;                 // lock PMM


	 /* Attempt to enable the 32kHz XTAL */
	 enableXtal();
}

/*
 * Configuracao para o funcionamento do UART (do USB)
 */
void setupUart() {
	  unsigned int mod;
	  unsigned long divider;

	  divider=(F_CPU<<4)/BAUD_RATE; // Baud Rate de 9600
	  mod = divider&0xFFF0;    // UCBRFx = INT([(N/16) – INT(N/16)] × 16)
	  divider>>=8;


	  P4SEL = BIT4+BIT5;                        // 4.4,4.5 = USCI_A0 TXD/RXD
	  UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
	  UCA1CTL1 |= UCSSEL_2;                     // SMCLK

	  UCA1BR0 = divider;

	  mod = ((divider&0xf8)+0x8)&0xf0;          // UCBRFx (bit 4-7)
	  UCA1BR1 = divider>>8;

	  UCA1MCTL = UCOS16 | mod;
	                                            // over sampling
	  UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	  UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

/*
 * Configuracao para o funcionamento do I2C
 */
void setupI2C() {
	P3SEL |= (BIT0 + BIT1);                 // Assign I2C pins to USCI_B0 (P3.0 -> SDA / P3.1 -> SCL)
	//P3REN |= (BIT0 + BIT1);

	UCB0CTL1 |= UCSWRST;                     // Enable SW reset

	/*
	 * Configure as I2C Master.
	 * UCMST = Master
	 * UCMODE_3 = I2C mode
	 * UCSYNC = Synchronous mode
	 */
	//UCB0CTL0 = UCMODE_3 | UCSYNC;
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;   // I2C Master, synchronous mode
	UCB0CTL1 |= UCSSEL_2;           		// Use SMCLK

	UCB0BR0 = (unsigned char)((F_CPU / I2C_FREQ) & 0xFF);
	UCB0BR1 = (unsigned char)((F_CPU / I2C_FREQ) >> 8);

	UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation*/

	twi_master_init();
	//twi_init();

}

unsigned long micros()
{
	unsigned long m;
	__disable_interrupt(); // Desabilita as interrupções para garantir um leitura consistente
	m = wdt_overflow_count;
	__enable_interrupt();
	return (m * MICROSECONDS_PER_WDT_OVERFLOW);
}

/*
 *  Delay in milisegundos com o WDT
 */
void delay(uint32_t milliseconds)
{
	uint32_t start = micros();
	while(milliseconds > 0) {
		if ((micros() - start) >= 1000) {
			milliseconds--;
			start += 1000;
		}
		__bis_SR_register(LPM0_bits + GIE);
	}
}


/* Delay for the given number of microseconds.  Assumes a 1, 8 or 16 MHz clock. */
void delayMicroseconds(uint32_t us)
{
#if F_CPU >= 20000000L
	/* For a one-microsecond delay, simply wait 2 cycle and return. The overhead
	 * of the function call yields a delay of exactly one microsecond. */
	__asm__ __volatile__ (
		"nop" "\n\t"
		"nop");
	if (--us == 0)
		return;

	/* The following loop takes a 1/5 of a microsecond (4 cycles)
	 * per iteration, so execute it five times for each microsecond of
	 * delay requested. */
	us = (us<<2) + us; // x5 us

	/* Account for the time taken in the preceeding commands. */
	us -= 2;

#elif F_CPU >= 16000000L
	/* For the 16 MHz clock on most boards */

	/* For a one-microsecond delay, simply return.  the overhead
	 * of the function call yields a delay of approximately 1 1/8 us. */
	if (--us == 0)
		return;

	/* The following loop takes a quarter of a microsecond (4 cycles)
	 * per iteration, so execute it four times for each microsecond of
	 * delay requested. */
	us <<= 2;

	/* Account for the time taken in the preceeding commands. */
	us -= 2;
#else
	/* For the 1 MHz */

	/* For a one- or two-microsecond delay, simply return.  the overhead of
	 * the function calls takes more than two microseconds.  can't just
	 * subtract two, since us is unsigned; we'd overflow. */
	if (--us == 0)
		return;
	if (--us == 0)
		return;

	/* The following loop takes 4 microsecond (4 cycles)
	 * per iteration, so execute it ones for each 4 microsecond of
	 * delay requested. */
	us >>= 2;

	/* Partially compensate for the time taken by the preceeding commands.
	 * we can't subtract any more than this or we'd overflow w/ small delays. */
	us--;
#endif

	/* Busy wait */
        __asm__ __volatile__ (
                /* even steven */
                "L1: nop \n\t"
                /* 1 instruction */
                "dec.w %[us] \n\t"
                /* 2 instructions */
                "jnz L1 \n\t"
                : [us] "=r" (us) : "[us]" (us)
        );
}


/*
 * Leitura dos bits de um byte
 * Entrada: byte, bit_start e Length
 * Saida: Byte com os bits
 */
uint8_t read_bits(uint8_t byte, uint8_t bit_start, uint8_t length)
{
  // 01101001 read byte
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  //    010   masked
  //   -> 010 shifted
  uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
  uint8_t b = byte;
  b &= mask;  // zero all non-important bits in data
  b >>= (bit_start - length + 1); // shift
  return b;
}


/*
 * Byte de escrita de dados com os bits definidos
 * Entrada: bit_start, Length e data
 * Saida: Byte com os bits
 */
uint8_t write_bits(uint8_t bit_start, uint8_t length, uint8_t data)
{
	uint8_t b = data;
	uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
	b <<= (bit_start - length + 1); // shift data into correct position
	b&= mask; // zero all non-important bits in data
	return b;
}


