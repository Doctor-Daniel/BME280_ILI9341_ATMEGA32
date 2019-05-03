/*
 * i2c.c
 *
 * Created: 2017-07-08 21:04:31
 *  Author: Daniel
 */ 
#include <avr/io.h>
#include "i2c.h"

void i2cInit(void)                                                   //Inicjacja kontrolera I2C
{
	TWCR = _BV(TWEN);                                                //w rejestrze kontrolnym TWCR wlaczamy obsluge I2C (bit TWEN)
	TWBR = (F_CPU/F_SCL - 16) / 2;                                   // ustawienie predkosci transmisji
}

uint8_t i2cStart(void)   //funkcja wysylajaca sygnal START
{
	TWCR = _BV(TWEN) | _BV(TWSTA) | _BV(TWINT);                    // TWEN - wlacza I2C; TWSTA - sygnal start (ale trzeba potem wyzerowac go) ; TWINT - flaga przerwania, 1 oaznacza zakonczenie danej operacji
	while (!(TWCR & _BV(TWINT)));                                  // czekamy az bit TWINT sie wyzeruje przez zapis 1. Jak bedzie na nim 1, to warunek da 0 i petla while sie skonczy :)
	uint8_t status = TWSR & I2C_STATUS_MASK;                       // sprawdzamy status czy powiodlo sie wyslanie start.
	if (status == I2C_START || status == I2C_START_REPEATED)
	{
		return I2C_OK;                                             // jesli wszystko OK, to zwraca 0
	}
	else
	{
		return status;                                            // w razie bledu wzraca status
	}
}

void i2cStop(void)  // funkcja generujaca sygnal STOP
{
	TWCR = _BV(TWEN) | _BV(TWSTO) | _BV(TWINT);    // wlaczana jest obsluga I2C; generowany sygnal stop; flaga przerwania
	while (TWCR & _BV(TWSTO));                     // oczekiwanie za zakonczenie bitu TWSTO. Zakonczenie stopu nie jest tez sygnalizowane bitem TWINT, ale wyzerowaniem bitu TWSTO
}

void i2cSend(uint8_t data)                        // funkcja nizszego poziomu. Z niej korzystaja i2cSendAddress i i2cSendData
{
	TWDR = data;
	TWCR = _BV(TWEN) | _BV(TWINT);
	while (!(TWCR & _BV(TWINT)));
}

uint8_t i2cSendAddress(uint8_t address)
{
	i2cSend(address);
	uint8_t status = TWSR & I2C_STATUS_MASK;
	if (address & 0b00000001)                                            //sprawdzany jest bit ACK od slave, to ten ostatni bit
	{
		return (status == I2C_ADDR_READ_ACK) ? I2C_OK : status;         // sprawdzanie czy byl to adres do zapisu czy odczytu. Decyduje o tym jedynka na najmlodszym bicie
	}
	else
	{
		
		return (status == I2C_ADDR_WRITE_ACK) ? I2C_OK : status;
	}
}
uint8_t i2cSendData(uint8_t data)
{
	i2cSend(data);
	uint8_t status = TWSR & I2C_STATUS_MASK;
	return (status == I2C_DATA_SENT_ACK) ? I2C_OK : status;
}

uint8_t i2cReceive(uint8_t last, uint8_t * data)             // drugim parametrem jest wzkaznik na miejsce w pamieci gdzie sa dane.
{
	TWCR = last ? _BV(TWEN) | _BV(TWINT) : _BV(TWEN) | _BV(TWINT) | _BV(TWEA);            //jesli last rozny od zera to nie wysyla bitu TWEA czym nie potwierdza odbioru. 
	while (!(TWCR & _BV(TWINT)));
	*data = TWDR;                                            // do danych odwolujemy sie wskaznikiem, bo przyjemismy ze funkcja bedzie wzracac kod bledu a dwoch rzeczy funkcja zwracac nie moze
	uint8_t status = TWSR & I2C_STATUS_MASK;                 // nakladamy maske na rejestr TWSR i to przypisujemy do mneinnej status.
	if(last)
	{
		return (status == I2C_DATA_RECEIVED_NACK) ? I2C_OK : status;
	}
	else
	{
		return (status == I2C_DATA_RECEIVED_ACK) ? I2C_OK : status; 
	}
}

