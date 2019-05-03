/*
 * BME208_ATMEGA32
 *
 * Created: 2019-04-22 22:08:23
 * Author : Daniel
 * Stacja pogodowa oparta na czujniku BME280.
 * Odczyt temperatury, cisnienia, wilgotnosci i wysokosci n.p.m.
 * 
 */ 

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "i2c.h"
#include "bme280.h"
#include "ILI9341.h"
#include "SPI.h"
#include "colors.h"
#include "font16x16.h"
#include "font8x12.h"
#include "literals.h"

#define PGM_GETSTR(str, idx) (char *)pgm_read_word(&str[idx])
void touch_calibration(void);
uint16_t drawColor = GREEN;

int main(void) 
{
	
	i2cInit();    
	bme280_init();
	ILI9341_init();							// inicjalizacja wyswietlacza
	ILI9341_set_rotation(PORTRAIT_REV);
	ILI9341_cls(BLACK);
	ILI9341_set_font((font_t) {font16x16, 16, 16, RED, WHITE});	

	sei();		// wlaczenie przerwan
	int32_t T;
	int32_t p;
	int32_t h;
	float alt;
	char buff[4];
	while(1) 
	{
		bme280_measure(&T, &p, &h, &alt);	// wykonanie pomiaru z czujnika
		dtostrf(T/100, 2, 0 , buff);	// wypisanie wartosci na LCD TFT ILI9341
		ILI9341_txt(30, 70, buff);
		dtostrf(T%100, 2, 0, buff);
		ILI9341_txt(63, 70, ".");
		ILI9341_txt(73, 70, buff);
		ILI9341_txt(110, 70, "*C");
		dtostrf(p, 4, 0, buff);
		ILI9341_txt(30, 90, buff);
		ILI9341_txt(100, 90, "hPa");
		dtostrf(h, 4, 0, buff);
		ILI9341_txt(30, 120, buff);
		ILI9341_txt(99, 120, "%");
		dtostrf(alt, 3, 0, buff);
		ILI9341_txt(30, 150, buff);
		ILI9341_txt(85, 150, "m n.p.m.");
		_delay_ms(500);
	}

	return 0;
}
