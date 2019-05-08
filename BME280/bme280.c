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
#define BME280_SOFT_RESET	0xE0

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
/*#define BME280_CAL_DATA_SIZE_2	(BME280_CAL_REG_E7 + 1 - BME280_CAL_REG_E1) */

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
		int8_t  dig_h6;
	};
}bme280_cal; 


static void bme280_getcalibration(void)				// Odczyt wspolczynnikow kalibracyjnych
{
	memset(bme280_cal.bytes, 0, sizeof(bme280_cal));
	uint8_t rejestry[2];
	uint8_t rejestr1[1];
	bme280_readRegisters(BME280_CAL_REG_FIRST, bme280_cal.bytes, BME280_CAL_DATA_SIZE_1);	// te tak samo jak dla BME280 lub BMP180.
	
	bme280_readRegister(BME280_CAL_REG_H1, rejestr1);
	bme280_cal.dig_h1 = ((uint8_t) rejestr1[0]);						// chyba cos nie tak z tymi wspolczynnikami h1, bo nie chce sie wilgotnosc wyswietlac prawidlowo
	bme280_readRegisters(BME280_CAL_REG_E1, rejestry, 2);
	bme280_cal.dig_h2 = (((int16_t) (rejestry[0]<<4)) | ((int16_t)(rejestry[1] & 0x0F)));
	bme280_readRegister(BME280_CAL_REG_E3, rejestr1);
	bme280_cal.dig_h3 = ((uint8_t) rejestr1[0]);
	bme280_readRegisters(BME280_CAL_REG_E4, rejestry, 2);
	bme280_cal.dig_h4 = (((int16_t) (rejestry[0] << 4)) | ((int16_t) (rejestry[1] & 0x0F)));
	bme280_readRegisters(BME280_CAL_REG_E5, rejestry, 2);
	bme280_cal.dig_h5 = (((int16_t) (rejestry[0] >> 4)) | ((int16_t) (rejestry[1] << 4)));
	bme280_readRegister(BME280_CAL_REG_E7, rejestr1);
	bme280_cal.dig_h6 = ((int8_t) rejestr1[0]);
}

void bme280_init(void)
{
	bme280_writeRegister(BME280_SOFT_RESET, 0xB6);
	_delay_ms(500);
	bme280_getcalibration();
	bme280_set_ctrl(1, 1, 1, 3); // T oversampling x1, P oversampling x1, H oversampling x1, forced mode (1)  
	bme280_set_config(0, 0, 0); // 0.5 ms delay(nieaktywny w forced mode), 1x filter, no 3-wire SPI
	_delay_ms(500);
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
	
void bme280_measure(double * T, double * p, double * h, float *alt)		// wersja na wskaznikach,
{
	uint8_t rejestry[8];
	volatile int32_t temp_raw, pres_raw, hum_raw, t_fine, p_fine;
	double var1, var2;
	
	bme280_readRegisters(BME280_PRES_REG, rejestry, 8);	// odczyt danych z rejestrow ze wskazaniami z czujnika ( 0xF7 - 0xFE )
	
	pres_raw = bme280_20bit_reg(rejestry[0], rejestry[1], rejestry[2]);
	temp_raw = bme280_20bit_reg(rejestry[3], rejestry[4], rejestry[5]);
	hum_raw = bme280_16bit_reg(rejestry[6], rejestry[7]);

	// Obliczanie temperatury
	var1 = (((double) temp_raw)/16384.0 - ((double)bme280_cal.dig_t1)/1024.0) * ((double)bme280_cal.dig_t2);
	var2 = ((((double)temp_raw)/131072.0 - ((double)bme280_cal.dig_t1)/8192.0) * (((double)temp_raw)/131072.0 - ((double)bme280_cal.dig_t1)/8192.0)) * ((double)bme280_cal.dig_t3);
	t_fine = (int32_t)(var1 + var2);
	*T = (var1+var2) / 5120.0;

	// Obliczanie cisnienia
	var1 = 0;
	var2 = 0;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)bme280_cal.dig_p6) / 32768.0;
	var2 = var2 + var1 * ((double)bme280_cal.dig_p5) * 2.0;
	var2 = (var2/4.0) + (((double)bme280_cal.dig_p4) * 65536.0);
	var1 = (((double)bme280_cal.dig_p3) * var1 * var1 / 524288.0 + ((double) bme280_cal.dig_p2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)bme280_cal.dig_p1);
	if (var1 == 0.0)
	{	
		p_fine = 0;
	}
	p_fine = 1048576.0 - (double)pres_raw;
	p_fine = (p_fine - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)bme280_cal.dig_p9) * p_fine * p_fine / 2147483648.0;
	var2 = p_fine * ((double)bme280_cal.dig_p8) / 32768.0;
	p_fine = p_fine + (var1 + var2 + ((double)bme280_cal.dig_p7)) / 16.0;
	*p = p_fine/100;
	
	// obliczanie wilgotnosci
	var1 = 0;
	var1 = (((double)t_fine) - 76800.0);
	var1 = (hum_raw - (((double)bme280_cal.dig_h4) * 64.0 + ((double)bme280_cal.dig_h5) / 16384.0 * var1)) * 
	(((double)bme280_cal.dig_h2) / 65536.0 * (1.0 + ((double)bme280_cal.dig_h6) / 67108864.0 * var1 * 
	(1.0 + ((double)bme280_cal.dig_h3) / 67108864.0 * var1)));
	var1 = var1 * (1.0 - ((double)bme280_cal.dig_h1) * var1 / 524288.0);	// dzielone przez 5, bo inaczej wilgotnosc potrafi byc ok 125%
	//if (var1 > 100.0)
		//var1 = 100.0;
	//else if (var1 < 0.0)
	//var1 = 0.0;
	*h = var1;
	
	// obliczanie wysokosci
	float atmospheric = (p_fine/100.0);
	*alt = 44330.0 * (1 - pow(atmospheric/1013.25, 0.1903));	
}
