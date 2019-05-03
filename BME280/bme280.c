/*
*	BME280 - biblioteka 
*   Komunikacja po magistrali I2C
*	Author: Daniel
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include <util/delay.h>
#include "i2c.h"
#include "bme280.h"

#define BME280_ADDR	0x76	// Adres czujnika 0x76

#define BME280_ID_REG		0xD0
#define BME280_ID_VAL		0x60

#define BME280_CAL_REG_FIRST	0x88
#define BME280_CAL_REG_LAST		0x9F
#define BME280_CAL_REG_H1		0xA1
#define BME280_CAL_REG_E1		0xE1
#define BME280_CAL_REG_E3		0xE3
#define BME280_CAL_REG_E4		0xE4
#define BME280_CAL_REG_E5		0xE5
#define BME280_CAL_REG_E7		0xE7
#define BME280_CAL_DATA_SIZE	(BME280_CAL_REG_LAST + 2 - BME280_CAL_REG_FIRST) + (BME280_CAL_REG_E7 + 1 - BME280_CAL_REG_E1)
#define BME280_CAL_DATA_SIZE_1	(BME280_CAL_REG_LAST + 1 - BME280_CAL_REG_FIRST)
#define BME280_CAL_DATA_SIZE_2	(BME280_CAL_REG_E7 + 1 - BME280_CAL_REG_E1) 

#define BME280_STATUS_REG	0xF3
#define BME280_CONTROL_REG	0xF4
#define BME280_CONTROL_HUM	0xF2
#define BME280_CONFIG_REG	0xF5

#define BME280_PRES_REG		0xF7
#define BME280_TEMP_REG		0xFA
#define BME280_HUM_REG		0xFD
#define BME280_RAWDATA_BYTES	8	// 3 bytes pressure, 3 bytes temperature, 2 bytes humidity

#define I2C_WRITE	0x00
#define I2C_READ	0x01

#define bme280_20bit_reg(b1, b2, b3)	( ((int32_t)(b1) << 12) | ((int32_t)(b2) << 4) | ((int32_t)(b3) >> 4) )
#define bme280_16bit_reg(b1, b2)		( ((int32_t)(b1) <<8) | ((int32_t)(b2)) );

void bme280_writeRegister(uint8_t reg, uint8_t value)
{
	i2cStart();
	i2cSendAddress((BME280_ADDR << 1) | I2C_WRITE);
	i2cSendData(reg);
	i2cSendData(value);
	i2cStop();
}

void bme280_readRegister(uint8_t reg, uint8_t *data)
{
	i2cStart();
	i2cSendAddress((BME280_ADDR << 1) | I2C_WRITE);
	i2cSendData(reg);
	i2cStart();
	i2cSendAddress((BME280_ADDR << 1) | I2C_READ);
	i2cReceive(1, data);
	i2cStop();
}
void bme280_writeRegisters(uint8_t reg, uint8_t * data, uint8_t size)
{
	i2cStart();
	i2cSendAddress((BME280_ADDR << 1) | I2C_WRITE);
	i2cSendData(reg);
	for(uint8_t i = 0; i < size; i++)
	{
		i2cSendData(data[i]);
	}
	i2cStop();
}

void bme280_readRegisters(uint8_t reg, uint8_t * data, uint8_t size)
{
	i2cStart();
	i2cSendAddress((BME280_ADDR << 1) | I2C_WRITE);
	i2cSendData(reg);
	i2cStart();
	i2cSendAddress((BME280_ADDR << 1) | I2C_READ);
	for(uint8_t i=0; i < size; i++) 
	{
		i2cReceive(i == size - 1, data++);
	}
	i2cStop();
}

// Wspolczynniki kalibracyjne umieszczone w strukturze . Ty mamym mozna dodac dwa czujniki na magistrali I2C.
static union _bme280_cal_union 
{
	uint8_t bytes[BME280_CAL_DATA_SIZE];
	struct {
		uint16_t dig_t1;	
		int16_t  dig_t2;	
		int16_t  dig_t3;
		uint16_t dig_p1;
		int16_t  dig_p2;
		int16_t  dig_p3;	
		int16_t  dig_p4;
		int16_t  dig_p5;
		int16_t  dig_p6;
		int16_t  dig_p7;
		int16_t  dig_p8;	
		int16_t  dig_p9;
		uint8_t  dig_h1;	
		int16_t	 dig_h2;
		uint8_t  dig_h3;	
		int16_t  dig_h4;
		int16_t  dig_h5;
		int8_t   dig_h6;
	};
} bme280_cal;


static void bme280_getcalibration(void)				// Odczyt wspolczynnikow kalibracyjnych
{
	memset(bme280_cal.bytes, 0, sizeof(bme280_cal));
	uint8_t rejestry[2];
	bme280_readRegisters(BME280_CAL_REG_FIRST, bme280_cal.bytes, BME280_CAL_DATA_SIZE_1);	// te tak samo jak dla BME280 lub BMP180.
	bme280_readRegister(BME280_CAL_REG_H1, &bme280_cal.dig_h1);
	bme280_readRegisters(BME280_CAL_REG_E1, rejestry, 2);
	bme280_cal.dig_h2 = ((int16_t) (rejestry[0]<<4) | (int16_t)(rejestry[1] & 0x0F));
	bme280_readRegister(BME280_CAL_REG_E3, &bme280_cal.dig_h3);
	bme280_readRegisters(BME280_CAL_REG_E4, rejestry, 2);
	bme280_cal.dig_h4 = ((int16_t) (rejestry[0] << 4) | (int16_t) (rejestry[1] & 0x0F));
	bme280_readRegisters(BME280_CAL_REG_E5, rejestry, 2);
	bme280_cal.dig_h5 = ((int16_t) (rejestry[0] >> 4) | (int16_t) (rejestry[1] << 4));
	bme280_readRegister(BME280_CAL_REG_E7, rejestry);
	bme280_cal.dig_h6 = ((int8_t) (rejestry[0]));
}

void bme280_init(void)
{
	bme280_getcalibration();
	bme280_set_config(0, 0, 0); // 0.5 ms delay(nieaktywny w forced mode), 1x filter, no 3-wire SPI
	bme280_set_ctrl(1, 1, 1, 3); // T oversampling x1, P oversampling x1, H oversampling x1, forced mode (1)  
}

uint8_t bme280_get_status(void)
{
	uint8_t data[1];
	bme280_readRegister(BME280_STATUS_REG, data);
	return data[0];
}

void bme280_set_ctrl(uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode)
{
	bme280_writeRegister(BME280_CONTROL_REG, ((osrs_t & 0x7) << 5) | ((osrs_p & 0x7) << 2) | (mode & 0x3));
	//bme280_writeRegister(BME280_CONTROL_HUM, (osrs_h & 0x7));
	bme280_writeRegister(BME280_CONTROL_HUM, 0x01);
}

void bme280_set_config(uint8_t t_sb, uint8_t filter, uint8_t spi3w_en)
{
	bme280_writeRegister(BME280_CONFIG_REG, ((t_sb & 0x7) << 5) | ((filter & 0x7) << 2) | (spi3w_en & 1));
	
}
	
void bme280_measure(int32_t * T, int32_t * p, int32_t * h, float *alt)		// wersja na wskaznikach,
{
	uint8_t rejestry[8];
	int32_t temp_raw, pres_raw, hum_raw, var1, var2, t_fine, p_temp;
	
	bme280_readRegisters(BME280_PRES_REG, rejestry, 8);	// odczyt danych z rejestrow ze wskazaniami z czujnika ( 0xF7 - 0xFE )
	
	pres_raw = bme280_20bit_reg(rejestry[0], rejestry[1], rejestry[2]);
	temp_raw = bme280_20bit_reg(rejestry[3], rejestry[4], rejestry[5]);
	hum_raw = bme280_16bit_reg(rejestry[6], rejestry[7]);

	// Obliczanie temperatury
	var1 = ((((temp_raw >> 3) - ((int32_t)bme280_cal.dig_t1 << 1))) * ((int32_t)bme280_cal.dig_t2)) >> 11;
	var2 = (((((temp_raw >> 4) - ((int32_t)bme280_cal.dig_t1)) * ((temp_raw >> 4) - ((int32_t)bme280_cal.dig_t1))) >> 12) * ((int32_t)bme280_cal.dig_t3)) >> 14;
	t_fine = var1 + var2;
	*T = (t_fine * 5 + 128) >> 8;

	// Obliczanie cisnienia
	var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)bme280_cal.dig_p6);
	var2 = var2 + ((var1 * ((int32_t)bme280_cal.dig_p5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)bme280_cal.dig_p4) << 16);
	var1 = (((bme280_cal.dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)bme280_cal.dig_p2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t)bme280_cal.dig_p1)) >> 15);

//	if (var1 == 0)
//	{
//		*p = 0;
//	} 
	p_temp = (((uint32_t)(((int32_t)1048576) - pres_raw) - (var2 >> 12))) * 3125;
	if (p_temp < 0x80000000) 
	{
		p_temp = (p_temp << 1) / ((uint32_t)var1);
	} 
	else 
	{
		p_temp = (p_temp / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)bme280_cal.dig_p9) * ((int32_t)(((p_temp>>3) * (p_temp >> 3)) >> 13))) >> 12;
	var2 = (((int32_t)(p_temp >> 2)) * ((int32_t)bme280_cal.dig_p8)) >> 13;
	p_temp = (uint32_t)((int32_t) p_temp + ((var1 + var2 + bme280_cal.dig_p7) >> 4));
	
	*p = ((p_temp)/100);

	// obliczanie wilgotnosci
	var1 = 0;
	//if (hum_raw == 0x8000)			// z tym warunkiem nie odczytuje wilgotnosci.
	//{
		//*h = 1;
	//}
	//else
	{
		var1 = (t_fine - ((int32_t)76800));
		var1 = (((((hum_raw << 14) - (((int32_t)bme280_cal.dig_h4) << 20) -
		(((int32_t)bme280_cal.dig_h5) * var1)) + ((int32_t)16384)) >> 15) *
		(((((((var1 * ((int32_t)bme280_cal.dig_h6)) >> 10) *
		(((var1 * ((int32_t)bme280_cal.dig_h3)) >> 11) + ((int32_t)32768))) >> 10) +
		((int32_t)2097152)) * ((int32_t)bme280_cal.dig_h2) + 8192) >> 14));
		
		var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)bme280_cal.dig_h1)) >> 4));
		
		var1 = (var1 > 0) ? 2 : var1;
		var1 = (var1 > 419430400) ? 419430400 : var1;
		*h = -((var1>>12)/1024);
	}
	// obliczanie wysokosci
	float atmospheric = (p_temp/100.0);
	*alt = 44330.0 * (1 - pow(atmospheric/1013.25, 0.1903));
	
	
	
	
}
