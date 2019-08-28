
#ifndef __ESP_WLAN_RAW_H__
#define  __ESP_WLAN_RAW_H__

#include "esp_wifi.h"
#include "esp_err.h"
#include "esp_wifi_internal.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"

#define WLAN_HEADERS_BYTES 32
#define WLAN_SETABLE_BYTES 12

#define WLAN_MTU_BYTES 2312
#define WLAN_TOTAL_BYTES 2336

#define WLAN_MTU_BITS (WLAN_TOTAL_BYTES * 8)

#define SIGNAL_FIELD_BITS 16
#define SIGNAL_FIELD_BYTES 2

#define WLAN_RECV_ADDR_OFFSET 4
#define WLAN_TRAN_ADDR_OFFSET 10

#define ESP_CUSTOM_POWER_6M 127
#define ESP_CUSTOM_POWER_24M 127

typedef enum symbol_size{
	WUR_SIZE_6M = 24,
	WUR_SIZE_9M = 36,
	WUR_SIZE_12M = 48,
	WUR_SIZE_18M = 72,
	WUR_SIZE_24M = 96,
	WUR_SIZE_36M = 144,
	WUR_SIZE_48M = 192,
	WUR_SIZE_56M = 216
}symbol_size_t;

struct wlan_wur_ctxt;

typedef esp_err_t (*wlan_bit_set_fn_t)(struct wlan_wur_ctxt *ctxt, uint8_t value);

typedef struct wlan_wur_ctxt{
	uint8_t frame_buffer[WLAN_TOTAL_BYTES];
	uint8_t scrambler_buffer[WLAN_TOTAL_BYTES];
	uint8_t initial_scrambler_state;
	uint8_t current_scrambler_state;
	uint16_t current_len;
	symbol_size_t symbol_len;
	wlan_bit_set_fn_t send_bit_fn;
}wlan_wur_ctxt_t;


esp_err_t wlan_wur_init_context(wlan_wur_ctxt_t *wur_context, uint8_t initial_state, symbol_size_t symbol_size);
esp_err_t wlan_wur_transmit_frame(wlan_wur_ctxt_t *wur_context, uint8_t* data_bytes, uint8_t data_bytes_len);
#endif
