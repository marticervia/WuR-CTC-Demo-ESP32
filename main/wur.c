/*
 * wur.c
 *
 *  Created on: 29 jul. 2019
 *      Author: marti
 */
#include "esp_system.h"

#include "wur.h"
#include "ook_wur.h"
#include "i2c_wur.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "utils.h"
#include <string.h>
#include "esp_wifi.h"

#define ESP_INTR_FLAG_DEFAULT 0

#define FRAME_BUF_LEN (WUR_HEADER_LEN + WUR_MAX_DATA_LEN)

typedef enum wur_tx_status{
	WUR_STATUS_IDLE,
	WUR_STATUS_WAIT_DATA_ACK,
	WUR_STATUS_WAIT_WAKE_ACK
}wur_tx_status_t;

typedef struct wur_context{
	uint32_t tx_timestamp;
	uint32_t rx_timestamp;
	wur_tx_status_t wur_status;
	uint8_t expected_seq_num;
	wur_tx_cb tx_cb;
	wur_rx_cb rx_cb;
	uint8_t frame_buffer[FRAME_BUF_LEN];
	uint8_t frame_len;
	uint16_t wur_addr;
}wur_context_t;

static wur_context_t wur_context;
static volatile uint8_t wur_op_pending;
static SemaphoreHandle_t wur_semaphore;
static SemaphoreHandle_t wur_mutex;

/* handles the interruption genrated by the WUR GPIO pin. */
static IRAM_ATTR void wur_int_handler(void* arg)
{
	BaseType_t xTaskWokenByReceive = pdFALSE;
    uint32_t gpio_num = (uint32_t) arg;
    if(gpio_num == GPIO_WAKE){
		wur_op_pending = 1;
		xSemaphoreGiveFromISR(wur_semaphore, &xTaskWokenByReceive);
    }
}

static void wur_tick_task(void* args){
	printf("Wur Tick task started!");
	while(1){
		uint32_t tick = get_timestamp_ms();
		wur_tick(tick);
	}
}


void wur_init(uint16_t addr){
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO0 here
    io_conf.pin_bit_mask = GPIO_WAKE_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_WAKE, wur_int_handler, (void*) GPIO_WAKE);

    wur_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(wur_semaphore);

    wur_mutex = xSemaphoreCreateRecursiveMutex();
    xSemaphoreGiveRecursive(wur_mutex);


	ook_wur_init();
	wur_i2c_init();

	wur_context.tx_timestamp = 0;
	wur_context.rx_timestamp = 0;
	wur_context.wur_status = WUR_STATUS_IDLE;
	wur_context.expected_seq_num = 0;
	wur_context.tx_cb = NULL;
	wur_context.rx_cb = NULL;
	memset(wur_context.frame_buffer, 0, FRAME_BUF_LEN);
	wur_context.frame_len = 0;
	wur_context.wur_addr = addr;

	wur_set_address(addr);

	//TODO: get a real tack priority
	xTaskCreate(wur_tick_task,
	            "wur_task",
	            2048,
	            NULL,
	            17,
	            NULL
	         );
}



void wur_tick(uint32_t systick){

	xSemaphoreTakeRecursive(wur_mutex, portMAX_DELAY);

	if(wur_context.wur_status == WUR_STATUS_WAIT_DATA_ACK){
		if(systick - wur_context.tx_timestamp > WUR_DATA_TIMEOUT){
			printf("Timeout to ack last data frame");
			if(wur_context.tx_cb){
				wur_context.wur_status = WUR_STATUS_IDLE;
				wur_context.tx_cb(WUR_ERROR_TX_ACK_DATA_TIMEOUT);
			}
			else{
				uint32_t remaining_time = WUR_WAKE_TIMEOUT - (systick - wur_context.tx_timestamp);
				xSemaphoreGiveRecursive(wur_mutex);
				xSemaphoreTake(wur_semaphore, remaining_time/portTICK_PERIOD_MS);
				xSemaphoreTakeRecursive(wur_mutex, portMAX_DELAY);
			}
		}
	}else if(wur_context.wur_status == WUR_STATUS_WAIT_WAKE_ACK){
		if(systick - wur_context.tx_timestamp > WUR_WAKE_TIMEOUT){
			printf("Timeout to ack last wake frame");
			if(wur_context.tx_cb){
				wur_context.wur_status = WUR_STATUS_IDLE;
				wur_context.tx_cb(WUR_ERROR_TX_ACK_WAKE_TIMEOUT);
			}
		}
		else{
			uint32_t remaining_time = WUR_WAKE_TIMEOUT - (systick - wur_context.tx_timestamp);
			xSemaphoreGiveRecursive(wur_mutex);
			xSemaphoreTake(wur_semaphore, remaining_time/portTICK_PERIOD_MS);
			xSemaphoreTakeRecursive(wur_mutex, portMAX_DELAY);

		}
	}else{
		xSemaphoreGiveRecursive(wur_mutex);
		xSemaphoreTake(wur_semaphore, WUR_DEFAULT_TIMEOUT/portTICK_PERIOD_MS);
		xSemaphoreTakeRecursive(wur_mutex, portMAX_DELAY);
	}

	if(!wur_op_pending){
		printf("WuR Idle timeout!\n");
		goto exit;
	}

	i2c_wur_status_t wurx_state;
	if(wur_get_status(&wurx_state) != WUR_OK){
		printf("Warning: failed to get state from WuR after interrupt!");
		wur_op_pending = false;
		goto exit;
	}

	if(wurx_state.wur_status != WURX_HAS_FRAME){
		printf("Warning: Woke up without frame available!");
		wur_op_pending = false;
		goto exit;
	}

	if(wur_get_frame(wur_context.frame_buffer, wurx_state.wur_frame_len) != WUR_OK){
		printf("Warning: failed to get frame from WuR!");
		wur_op_pending = false;
		goto exit;
	}

	wur_context.frame_len = wurx_state.wur_frame_len;
	uint8_t frame_type = wur_context.frame_buffer[2] & 0x07;
	uint8_t seq_num = wur_context.frame_buffer[2] & 0x01;

	uint16_t addr;
	memcpy(&addr, wur_context.frame_buffer, 2);
	addr = ntohs(addr);

	/* check the frame flags! */
	if(frame_type & ACK_FLAG){
		if(wur_context.expected_seq_num == seq_num){
			if(wur_context.tx_cb){
				printf("Got ACK!");
				wur_context.wur_status = WUR_STATUS_IDLE;
				wur_context.tx_cb(WUR_ERROR_TX_OK);
				wur_context.expected_seq_num ^= 1;
			}
		}
		else{
			if(wur_context.tx_cb){
				printf("Got NACK!");
				wur_context.wur_status = WUR_STATUS_IDLE;
				wur_context.tx_cb(WUR_ERROR_TX_NACK);
				wur_context.expected_seq_num ^= 1;
			}
		}
	}

	if((frame_type & DATA_FLAG) || (frame_type & WAKE_FLAG)){
		printf("Got DATA!");
		/* an ack can piggiback a response frame, so continue*/
		if((wur_context.frame_len > 3) && wur_context.rx_cb){
			printf("parse DATA!");
			wur_context.rx_cb(WUR_ERROR_RX_OK, wur_context.frame_buffer, wur_context.frame_len);
			wur_context.rx_timestamp = systick;
		}
		if(!(frame_type & ACK_FLAG)){
			printf("Acknowledge DATA frame!");
			wur_send_ack(addr, seq_num);
		}
	}

	memset(wur_context.frame_buffer, 0, FRAME_BUF_LEN);
	wur_context.frame_len = 0;
	wur_op_pending = false;

	exit:
	xSemaphoreGiveRecursive(wur_mutex);
}

void wur_set_tx_cb(wur_tx_cb tx_cb){
	wur_context.tx_cb = tx_cb;
}

void wur_set_rx_cb(wur_rx_cb rx_cb){
	wur_context.rx_cb = rx_cb;
}

wur_tx_res_t wur_send_wake(uint16_t addr, uint16_t ms){
	ook_tx_errors_t tx_res;
	wur_tx_res_t res;

	xSemaphoreTakeRecursive(wur_mutex, portMAX_DELAY);

	if(wur_context.wur_status != WUR_STATUS_IDLE){
		res = WUR_ERROR_TX_BUSY;
		goto exit;
	}

	tx_res = ook_wur_wake(addr, ms, wur_context.expected_seq_num);
	if(tx_res != OOK_WUR_TX_ERROR_SUCCESS){
		printf("Warning: failed to start WAKE transmission!");
		wur_context.wur_status = WUR_STATUS_IDLE;
		res = WUR_ERROR_TX_FAILED;
		goto exit;
	}

	wur_context.tx_timestamp = get_timestamp_ms();
	wur_context.wur_status = WUR_STATUS_WAIT_WAKE_ACK;

	res = WUR_ERROR_TX_OK;
	exit:
	xSemaphoreGiveRecursive(wur_mutex);
	return res;
}

wur_tx_res_t wur_send_data(uint16_t addr, uint8_t* data, uint8_t data_len, uint8_t is_ack, int8_t ack_seq_num){
	ook_tx_errors_t tx_res;
	wur_tx_res_t res;

	xSemaphoreTakeRecursive(wur_mutex, portMAX_DELAY);

	if(wur_context.wur_status != WUR_STATUS_IDLE){
		res = WUR_ERROR_TX_BUSY;
		goto exit;
	}

	if(ack_seq_num < 0){
		ack_seq_num = wur_context.expected_seq_num;
	}

	tx_res = ook_wur_data(addr, data, data_len, is_ack, ack_seq_num);
	if(tx_res != OOK_WUR_TX_ERROR_SUCCESS){
		printf("Warning: failed to start DATA transmission!");
		wur_context.wur_status = WUR_STATUS_IDLE;
		res = WUR_ERROR_TX_FAILED;
		goto exit;
	}

	wur_context.tx_timestamp = get_timestamp_ms();
	wur_context.wur_status = WUR_STATUS_WAIT_DATA_ACK;

	res = WUR_ERROR_TX_OK;
	exit:
	xSemaphoreGiveRecursive(wur_mutex);
	return res;
}

wur_tx_res_t wur_send_ack(uint16_t addr, int8_t ack_seq_num){
	ook_tx_errors_t tx_res;
	wur_tx_res_t res;

	xSemaphoreTakeRecursive(wur_mutex, portMAX_DELAY);

	if(wur_context.wur_status != WUR_STATUS_IDLE){
		res = WUR_ERROR_TX_BUSY;
		goto exit;
	}

	if(ack_seq_num < 0){
		ack_seq_num = wur_context.expected_seq_num;
	}

	tx_res = ook_wur_ack(addr, ack_seq_num);
	if(tx_res != OOK_WUR_TX_ERROR_QUEUED){
		printf("Warning: failed to start ACK transmission!");
		res = WUR_ERROR_TX_FAILED;
		goto exit;
	}
	res = WUR_ERROR_TX_OK;

	exit:
	xSemaphoreGiveRecursive(wur_mutex);
	return res;
}

