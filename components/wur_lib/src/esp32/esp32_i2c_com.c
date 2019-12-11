/*
 * i2c_com.c
 *
 *  Created on: 25 jul. 2019
 *      Author: marti
 */

#include "lib_conf.h"

/* ESP 32 I2C coms implementation*/

#ifdef USE_ESP_VERSION

#include "i2c_com.h"
#include "string.h"

#define I2C_MASTER_NUM I2C_NUM_1

#define I2C_SCL_MASK GPIO_SEL_21
#define I2C_SDA_MASK GPIO_SEL_22

#define I2C_SCL 21
#define I2C_SDA 22 

#define I2C_MASTER_FREQ_HZ 500000

#define I2C_CLOCK_DELAY 20 //micros
#define I2C_START_DELAY 20 //micros
#define I2C_WAKEUP_HOLD_DELAY 10 //micros

#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */

#define I2C_SLAVE_ADDR 0x14
#define I2C_READ_MASK 0x01
#define I2C_WRITE_MASK 0x00


#define I2C_WAIT ({ets_delay_us(I2C_CLOCK_DELAY);})
#define I2C_WAKEUP ({ets_delay_us(I2C_WAKEUP_HOLD_DELAY);})
#define I2C_WAIT_START ({ets_delay_us(I2C_START_DELAY);})
#define I2C_MAX_LEN 32

void i2c_com_init(void){

  /* prepare the I2C GPIO */
  gpio_pad_select_gpio(I2C_WAKEUP_GPIO);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(I2C_WAKEUP_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(I2C_WAKEUP_GPIO, 0);

  int i2c_master_port = I2C_NUM_1;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_SDA;
  conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
  conf.scl_io_num = I2C_SCL;
  conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
  printf("Initialized i2c\n");
}

static inline esp_err_t _i2c_com_master_transfer(uint8_t i2c_slave_addr, uint16_t i2c_mode, uint8_t register_num, uint8_t* buffer, uint16_t buffer_len){
  uint8_t addr_byte;
  i2c_cmd_handle_t i2c_cmd; 
  esp_err_t ret;


  i2c_cmd = i2c_cmd_link_create();
  if(i2c_cmd == NULL){
    printf("i2c command null.\n");
    return 1;
  }

  i2c_master_start(i2c_cmd);

  addr_byte = (i2c_slave_addr << 1) | I2C_WRITE_MASK;

  /* write header*/
  i2c_master_write_byte(i2c_cmd, addr_byte, ACK_CHECK_EN);
  i2c_master_write_byte(i2c_cmd, register_num, ACK_CHECK_EN);

  i2c_master_stop(i2c_cmd);

  ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(i2c_cmd);

  if(ret != ESP_OK){
    printf("Error %d at transaction.\n", ret);
    return ESP_FAIL;
  }

  i2c_cmd = i2c_cmd_link_create();
  if(i2c_cmd == NULL){
    printf("i2c command null.\n");
    return 1;
  }
  i2c_master_start(i2c_cmd);

  if(i2c_mode == I2C_FLAG_WRITE){
      addr_byte = (i2c_slave_addr << 1) | I2C_WRITE_MASK;
  }else if(i2c_mode == I2C_FLAG_READ){
      addr_byte = (i2c_slave_addr << 1) | I2C_READ_MASK;
  }else{
    printf("unknown operation mode.\n");
    i2c_cmd_link_delete(i2c_cmd);
    return ESP_FAIL;
  }

  if((i2c_mode == I2C_FLAG_WRITE) && buffer != NULL){
      //printf("<Writting buffer addr>.\n");
      i2c_master_write_byte(i2c_cmd, addr_byte, ACK_CHECK_EN);
      //printf("<Writting buffer , len %d>.\n", buffer_len);
      i2c_master_write(i2c_cmd, buffer, buffer_len, ACK_CHECK_EN);
  }else if((i2c_mode == I2C_FLAG_READ) && buffer != NULL){
      //printf("<Writting buffer addr>.\n");
      i2c_master_write_byte(i2c_cmd, addr_byte, ACK_CHECK_EN);
      //printf("<Reading buffer , len %d>.\n", buffer_len);
      i2c_master_read(i2c_cmd, buffer, buffer_len, I2C_MASTER_LAST_NACK);
  }else{
    printf("No buffer, canceling transaction.\n");
    i2c_cmd_link_delete(i2c_cmd);
    return ESP_FAIL;
  }

  i2c_master_stop(i2c_cmd);

  ret = i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(i2c_cmd);

  if(ret != ESP_OK){
    printf("Error %d at transaction.\n", ret);
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t i2c_com_write_register(uint8_t i2c_slave_addr, uint8_t reg_addr, uint8_t *write_buf, uint16_t write_buf_len){

  gpio_set_level(I2C_WAKEUP_GPIO, 1);
  I2C_WAKEUP;
  gpio_set_level(I2C_WAKEUP_GPIO, 0);

  return _i2c_com_master_transfer(i2c_slave_addr, I2C_FLAG_WRITE, reg_addr, write_buf, write_buf_len);
}

esp_err_t i2c_com_read_register(uint8_t i2c_slave_addr, uint8_t reg_addr, uint8_t *read_buf, uint16_t read_buf_len){

  gpio_set_level(I2C_WAKEUP_GPIO, 1);
  I2C_WAKEUP;
  gpio_set_level(I2C_WAKEUP_GPIO, 0);

  return _i2c_com_master_transfer(i2c_slave_addr, I2C_FLAG_READ, reg_addr, read_buf, read_buf_len);
}

#endif
