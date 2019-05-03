/*
 * SPI.c
 */

#include <avr/io.h>
#include "SPI.h"
#include "ILI9341.h"

//#define USE_HARD_SPI

void SPI_init(void)
{
       SPCR = (_BV(SPE) | _BV(MSTR));		// SPI enabled. Master/Slave select. MSB first. SPI Clock rate F_CPU / 2. SCK low. Clock phase leading.
	   SPSR = _BV(SPI2X);					// F_CPU/2 enabled
}

/* Exchange byte via SPI */
uint8_t SPI_rxtx(uint8_t tx, enum dev_t device)								// Wymiana danych przez szyne SPI
{
//#ifdef USE_HARD_SPI
	SPDR = tx;																// Dummy byte

    while (!(SPSR & (1 << SPIF)));											// Oczekiwanie na koniec transmisji
    return SPDR;
}
