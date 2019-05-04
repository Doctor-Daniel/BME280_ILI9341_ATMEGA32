#ifndef BME280_H_
#define BME280_H_

void bme280_init(void);													// call this first
uint8_t bme280_get_status(void);											// read the status register
void bme280_set_config(uint8_t t_sb, uint8_t filter, uint8_t spi3w_en);		// set the configuration register
void bme280_set_ctrl(uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode);			// set the oversampling and mode (this starts the conversion)
void bme280_measure(double * T, double * p, double *h, float *alt);													// do a measurement

#endif /* BME280_H_ */
