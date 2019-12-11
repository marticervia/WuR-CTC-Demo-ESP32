#ifndef __I2C_H__
#define __I2C_H__

#include "lib_conf.h"

#define I2C_FLAG_WRITE          0x0001
#define I2C_FLAG_READ           0x0002

void i2c_com_init(void);
esp_err_t i2c_com_write_register(uint8_t i2c_slave_addr, uint8_t reg_addr, uint8_t *write_buf, uint16_t write_buf_len);
esp_err_t i2c_com_read_register(uint8_t i2c_slave_addr, uint8_t reg_addr, uint8_t *read_buf, uint16_t read_buf_len);

#endif