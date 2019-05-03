#include <avr/io.h>


void i2cInit(void);
uint8_t i2cStart(void);
void i2cStop(void);
void i2cSend(uint8_t data);
uint8_t i2cReceive(uint8_t last, uint8_t * data);
uint8_t i2cSendAddress(uint8_t address);
uint8_t i2cSendData(uint8_t data);

#define I2C_START 0x08
#define I2C_START_REPEATED 0x10
#define I2C_ADDR_WRITE_ACK 0x18
#define I2C_ADDR_WRITE_NACK 0x20
#define I2C_DATA_SENT_ACK 0x28
#define I2C_DATA_SENT_NACK 0x30
#define I2C_ARBITRATION_LOST 0x38
#define I2C_ADDR_READ_ACK 0x40
#define I2C_ADDR_READ_NACK 0x48
#define I2C_DATA_RECEIVED_ACK 0x50
#define I2C_DATA_RECEIVED_NACK 0x58
#define I2C_STATUS_MASK 0b11111000
#define I2C_OK 0
#define F_SCL 100000


