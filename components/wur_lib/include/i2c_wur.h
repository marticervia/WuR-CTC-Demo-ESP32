#ifndef __I2C_WUR_H__
#define __I2C_WUR_H__
#include "lib_conf.h"

#define WUR_STATUS_REGISTER 1
#define WUR_ADDR_REGISTER 2
#define WUR_FRAME_REGISTER 3

typedef enum wurx_states{
	WURX_SLEEP = 0,
	WURX_DECODING_FRAME = 1,
	WURX_HAS_FRAME = 2,
}wurx_states_t;

typedef struct wur_status{
	wurx_states_t wur_status;
	uint8_t wur_frame_len;
}i2c_wur_status_t;


void wur_i2c_init(void);
uint8_t wur_set_address(uint16_t addr);
uint8_t wur_get_address(uint16_t* addr);
uint8_t wur_get_status(i2c_wur_status_t* status);
uint8_t wur_get_frame(uint8_t* buffer, uint8_t len);


#define WUR_OK 0
#define WUR_KO 1

#endif