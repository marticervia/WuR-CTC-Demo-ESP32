/*
 * wur.h
 *
 *  Created on: 29 jul. 2019
 *      Author: marti
 */

#ifndef WUR_H_
#define WUR_H_


#define WUR_DEFAULT_TIMEOUT 10000
#define WUR_WAKE_TIMEOUT 10000
#define WUR_DATA_TIMEOUT 10000
#define WUR_PAYLOAD_OFFSET 3

#define GPIO_WAKE GPIO_NUM_2
#define GPIO_WAKE_PIN_SEL  (1<<GPIO_NUM_2)

typedef enum wur_tx_res{
	WUR_ERROR_TX_OK = 0,
	WUR_ERROR_TX_ACK_DATA_TIMEOUT = -1,
	WUR_ERROR_TX_ACK_WAKE_TIMEOUT = -2,
	WUR_ERROR_TX_NACK = -3,
	WUR_ERROR_TX_FAILED = -4,
	WUR_ERROR_TX_BUSY = -5
}wur_tx_res_t;

typedef enum wur_rx_res{
	WUR_ERROR_RX_OK = 0,
	WUR_ERROR_RX_FAILED = -1
}wur_rx_res_t;

typedef void (*wur_tx_cb) (wur_tx_res_t tx_res);
typedef void (*wur_rx_cb) (wur_rx_res_t rx_res, uint8_t* rx_bytes, uint8_t rx_bytes_len);

void wur_init(uint16_t addr);
void wur_tick(uint32_t systick);
void wur_set_tx_cb(wur_tx_cb tx_cb);
void wur_set_rx_cb(wur_rx_cb rx_cb);

wur_tx_res_t wur_send_wake(uint16_t addr, uint16_t ms);
wur_tx_res_t wur_send_data(uint16_t addr, uint8_t* data, uint8_t data_len, uint8_t is_ack, int8_t ack_seq_num);
wur_tx_res_t wur_send_ack(uint16_t addr, int8_t ack_seq_num);

#endif /* WUR_H_ */
