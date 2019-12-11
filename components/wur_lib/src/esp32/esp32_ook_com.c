#include "lib_conf.h"

/* 
*	ESP_32 specific version of OOK frame management API
*	as defined in ook_com.h.
.*/

#ifdef USE_ESP_VERSION

#include "ook_com.h"
#include "string.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define TAG "WLAN_RAW"
//#define USE_GPIO

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

#define ESP_CUSTOM_POWER_6M 48
#define ESP_CUSTOM_POWER_24M 48


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


typedef esp_err_t (*wlan_bit_set_fn_t)(struct wlan_wur_ctxt *ctxt, uint8_t value);

typedef struct wlan_wur_ctxt{
	uint8_t frame_buffer[WLAN_TOTAL_BYTES];
	uint8_t scrambler_buffer[WLAN_TOTAL_BYTES];
	uint8_t initial_scrambler_state;
	uint8_t padding_bytes;
	uint8_t current_scrambler_state;
	uint16_t current_len;
	symbol_size_t symbol_len;
	wlan_bit_set_fn_t send_bit_fn;
}wlan_wur_ctxt_t;



#ifdef USE_GPIO
#define OUTPUT_GPIO GPIO_NUM_4
#define SET_OUTPUT GPIO.out_w1ts = 0x00000010
#define CLEAR_OUTPUT GPIO.out_w1tc = 0x00000010

static portMUX_TYPE wlanGroupMux = portMUX_INITIALIZER_UNLOCKED;	
#endif

static wlan_wur_ctxt_t wur_ctxt = {0};

/* bits are: [1,1,0,0,1,1,1,0,1,0,1,1,0,1,0,1,0,0,0,0,0,0,0,0], from first = MSB to LSB*/
static const uint8_t one_seq_6mbps[] = {0xce,0xb5,0x00};
/* bits are: [1,0,1,1,0,0,0,0, 1,0,1,1,1,1,1,1 ,1,1,0,0,1,0,1,1 ,0,0,1,1,0,0,1,1, 1,
     1,0,0,0,0,0,1, 1,0,1,0,0,0,0,1, 0,0,1,0,0,0,0,0,1, 1,0,1,0,0,1,1,0,0,1,1,0,1,0,1,0,1,1,0,1,
     0,1,0,1,0,1,1,0,1,0,1,0,1,1,1,1,1,1,1] */
static const uint8_t one_seq_24mbps[] = {0xb0, 0xbf, 0xcb, 0x33, 0xc1, 0xa1, 0x20, 0xd3, 0x35, 0x6a, 0xb5, 0x7f};
static uint8_t standard_wlan_headers[] = {/* data frame to AP */
										  0xff,0xff,0xff,0xff,0xff,0xff,0x30, 0xae, 0xa4, 0x05, 0xb6, 0x30};


static void _send_preamble_legacy_wlan(wlan_wur_ctxt_t *wur_ctxt){

	wur_ctxt->send_bit_fn(wur_ctxt, 0);
    wur_ctxt->send_bit_fn(wur_ctxt, 1);
	wur_ctxt->send_bit_fn(wur_ctxt, 0);
    wur_ctxt->send_bit_fn(wur_ctxt, 1);
	wur_ctxt->send_bit_fn(wur_ctxt, 0);
    wur_ctxt->send_bit_fn(wur_ctxt, 1);
	wur_ctxt->send_bit_fn(wur_ctxt, 0);
    wur_ctxt->send_bit_fn(wur_ctxt, 1);
	wur_ctxt->send_bit_fn(wur_ctxt, 0);
    wur_ctxt->send_bit_fn(wur_ctxt, 1);
	wur_ctxt->send_bit_fn(wur_ctxt, 0);
    wur_ctxt->send_bit_fn(wur_ctxt, 1);
    wur_ctxt->send_bit_fn(wur_ctxt, 1);
}

#ifdef USE_GPIO

static inline unsigned get_ccount(void)
{
        unsigned r;
        asm volatile ("rsr %0, ccount" : "=r"(r));
        return r;
}

static inline void cdelay(uint32_t cycles)
{
    uint32_t stop;
    asm volatile ("rsr %0, ccount" : "=r"(stop));
    stop += cycles;
    /* Note: all variables are unsigned (can wrap around)! */
    while (((uint32_t)get_ccount()) < stop);
}

static void IRAM_ATTR _send_preamble_legacy_gpio(void){
    CLEAR_OUTPUT;
    cdelay(945);
    SET_OUTPUT;
    cdelay(945);
    CLEAR_OUTPUT;
    cdelay(945);
    SET_OUTPUT;
    cdelay(945);
    CLEAR_OUTPUT;
    cdelay(945);
    SET_OUTPUT;
    cdelay(945);
    CLEAR_OUTPUT;
    cdelay(945);
    SET_OUTPUT;
    cdelay(945);
    CLEAR_OUTPUT;
    cdelay(945);
    SET_OUTPUT;
    cdelay(945);
    CLEAR_OUTPUT;
    cdelay(945);
    SET_OUTPUT;
    cdelay(945);
    SET_OUTPUT;
    cdelay(945);
}
#endif

static IRAM_ATTR void _send_byte_legacy_wlan(wlan_wur_ctxt_t *wur_ctxt, uint8_t byte){
    for(int16_t i = 7; i >= 0; i--){
    	uint8_t bit;
        bit = (byte & (1 << i)) >> i;
#ifndef USE_GPIO
        wur_ctxt->send_bit_fn(wur_ctxt, bit);
#else
        if(bit){
        	SET_OUTPUT;
        }else{
        	CLEAR_OUTPUT;
        }
        cdelay(928);
#endif
    }
}

static esp_err_t set_wifi_fixed_rate(wifi_phy_rate_t value)
{
	esp_err_t err;
    err = esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_STA, true, value);
	printf("ESP_fixed rate returns 0x%x", err);
	return err;
}

static void print_wlan_frame(uint8_t* frame, uint16_t frame_len){
    int16_t i;
    printf("Frame is:\n\n");
    
    printf("%02x ", frame[0]);
    for(i=1; i<frame_len-1; i++){
    	printf("%02x", frame[i]);
        if(i % 16 == 0){
        	printf("\n");
        }else{
        	printf(" ");
        }
    }
    printf("%02x\n\n", frame[i]);
}


static void scramble_bytes(wlan_wur_ctxt_t *ctxt, uint16_t byte_len, uint8_t* payload){
	uint8_t scrambler_state = ctxt->current_scrambler_state;

	/* scramble each byte*/
	for(uint16_t i = 0; i < byte_len; i++){
		uint8_t scrambler_byte = 0x00;
		/* advance scrambler state and XOR rtesult to byte array, if present*/
		for(uint8_t j = 0; j < 8; j++){
			uint8_t feedback = 0, bit = 0;
			feedback = ((!!(scrambler_state & 64))) ^ (!!(scrambler_state & 8));
			bit = feedback ^ (payload[i] >> (j%8));
			scrambler_state = ((scrambler_state << 1) & 0x7e) | feedback;
			/* scrambler sequencer is applied MSB first*/
			scrambler_byte |= bit << (j%8);
		}
		if(payload != NULL){
			payload[i] = scrambler_byte;
		}
	}

	ctxt->current_scrambler_state = scrambler_state;
}

static esp_err_t scramble_payload(wlan_wur_ctxt_t *ctxt, uint16_t len){

	if(ctxt->current_len + len > WLAN_TOTAL_BYTES){
		printf("Invalid offset to set values\n");
		return ESP_FAIL;
	}

	for(uint16_t i = ctxt->current_len; i < ctxt->current_len + len; i++){
		ctxt->frame_buffer[i] ^= ctxt->scrambler_buffer[i];
	}

	ctxt->current_len += len;
	return ESP_OK;
}

static IRAM_ATTR esp_err_t set_frame_bit_6_mbps(wlan_wur_ctxt_t *ctxt, uint8_t value){

	if(value){
		memcpy(&ctxt->frame_buffer[ctxt->current_len], one_seq_6mbps, 3);
	}

	if(scramble_payload(ctxt, 3) != ESP_OK){
		return ESP_FAIL;
	}

	return ESP_OK;

}

static IRAM_ATTR esp_err_t set_frame_bit_24_mbps(wlan_wur_ctxt_t *ctxt, uint8_t value){

	if(value){
		memcpy(&ctxt->frame_buffer[ctxt->current_len], one_seq_24mbps, 12);
	}

	if(scramble_payload(ctxt, 12) != ESP_OK){
		return ESP_FAIL;
	}

	return ESP_OK;
}

static uint8_t reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

esp_err_t wlan_wur_init_context(wlan_wur_ctxt_t *wur_context, uint8_t initial_state, symbol_size_t symbol_size){
#ifndef USE_GPIO
	memset(wur_context->frame_buffer, 0, WLAN_TOTAL_BYTES);

	initial_state &= 0x7f;

	wur_context->initial_scrambler_state = reverse(initial_state) >> 1;
	wur_context->current_scrambler_state = wur_context->initial_scrambler_state;

    switch(symbol_size){
		case WUR_SIZE_6M:
			wur_context->send_bit_fn = set_frame_bit_6_mbps;
			break;
		case WUR_SIZE_24M:
			wur_context->send_bit_fn = set_frame_bit_24_mbps;
			break;
		default:
			printf("Symbol size still not supported!\n");
			return ESP_ERR_INVALID_ARG;
	}

	wur_context->symbol_len = symbol_size;
	wur_context->current_len = 0;
	switch(symbol_size){
		case WUR_SIZE_6M:
			set_wifi_fixed_rate(WIFI_PHY_RATE_6M);
			esp_wifi_set_max_tx_power(ESP_CUSTOM_POWER_6M);
			break;
		case WUR_SIZE_24M:
			set_wifi_fixed_rate(WIFI_PHY_RATE_24M);
			esp_wifi_set_max_tx_power(ESP_CUSTOM_POWER_24M);
			break;
		default:
			printf("Rate still not supported!\n");
			return ESP_ERR_INVALID_ARG;
	}

	memset(wur_context->scrambler_buffer, 0, WLAN_TOTAL_BYTES);
	wur_context->current_scrambler_state = wur_context->initial_scrambler_state;
	printf("Innitializing frame with scrambler state 0x%02X.\n", wur_context->current_scrambler_state);

	uint8_t symbol_bytes = wur_context->symbol_len/8;

	/* get the padding byte number to align payload to OFDM symbol start boundary*/
	uint8_t padding_bytes = (symbol_bytes - ((SIGNAL_FIELD_BYTES + WLAN_HEADERS_BYTES) % symbol_bytes)) % symbol_bytes;
	printf("Using %d Padding bytes for a symbol size of %d bytes.\n", padding_bytes, symbol_bytes);

	uint16_t total_offset_bits = 9 + ((WLAN_HEADERS_BYTES + padding_bytes)*8);
	/* advance the scrambler state to include the SIGNAL field and the non settable byts of the header*/
	uint8_t scrambler_state = wur_context->current_scrambler_state;

	for(uint16_t i = 0; i < total_offset_bits; i++){
		uint8_t feedback = 0;
		feedback = ((!!(scrambler_state & 64))) ^ (!!(scrambler_state & 8));
		scrambler_state = ((scrambler_state << 1) & 0x7e) | feedback;
	}

	wur_context->current_scrambler_state = scrambler_state;
	printf("Current scrambler state is 0x%02X.\n", wur_context->current_scrambler_state);

	/* Create the scrambler values for the whole frame*/
	/* signal field bytes are scrambled but are not part of the MPDU*/
	scramble_bytes(wur_context, WLAN_TOTAL_BYTES, wur_context->scrambler_buffer);
	memset(&wur_context->frame_buffer[wur_context->current_len], 0, padding_bytes);

	wur_context->padding_bytes = padding_bytes;
#else
	gpio_pad_select_gpio(OUTPUT_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(OUTPUT_GPIO, GPIO_MODE_OUTPUT);
#endif
    return ESP_OK;
}

wur_errors_t ook_wur_init_context(void){
	if(wlan_wur_init_context(&wur_ctxt, 0x0f, WUR_SIZE_6M) != ESP_OK){
		return WUR_KO;
	}
	return WUR_OK;
}



esp_err_t wlan_wur_init_frame(wlan_wur_ctxt_t *wur_context){
	uint8_t l_Mac[6];
	wur_context->current_len = 0;

	esp_wifi_get_mac(ESP_IF_WIFI_STA, l_Mac);

	switch(wur_context->symbol_len){
		case WUR_SIZE_6M:
			memset(wur_context->frame_buffer, 0, WLAN_TOTAL_BYTES);
			break;
		case WUR_SIZE_24M:
			memset(wur_context->frame_buffer, 0xff, WLAN_TOTAL_BYTES);
			break;
		default:
			printf("Rate still not supported!\n");
			return ESP_ERR_INVALID_ARG;
	}

	/* set the header data apropiately*/
	memcpy(wur_context->frame_buffer, standard_wlan_headers, WLAN_SETABLE_BYTES);
	wur_context->current_len += WLAN_SETABLE_BYTES;
	/* signal field bytes are scrambled but are not part of the MPDU*/
	wur_context->current_len += wur_context->padding_bytes;

	return ESP_OK;
}

esp_err_t IRAM_ATTR wlan_wur_transmit_frame(wlan_wur_ctxt_t *wur_context, uint8_t* data_bytes, uint8_t data_bytes_len){
	esp_err_t esp_res;

#ifndef USE_GPIO
	esp_res = wlan_wur_init_frame(wur_context);
	if(esp_res != ESP_OK){
		printf("Failed to prepare frame of len %d because of %d.\n", wur_context->current_len, esp_res);
        return ESP_FAIL;
	}
	_send_preamble_legacy_wlan(wur_context);
#else
	portENTER_CRITICAL(&wlanGroupMux);
	SET_OUTPUT;
    ets_delay_us(20);
	_send_preamble_legacy_gpio();
#endif

	/* now send the actual frame, bit by bit*/
	for(uint16_t i = 0; i < data_bytes_len; i++){
		_send_byte_legacy_wlan(wur_context, data_bytes[i]);
	}

	//print_wlan_frame(wur_context->frame_buffer, wur_context->current_len);
	//print_wlan_frame(wur_context->scrambler_buffer, wur_context->current_len);

#ifndef USE_GPIO
	int32_t res = esp_wifi_internal_tx(ESP_IF_WIFI_STA, wur_context->frame_buffer, wur_context->current_len);
	if(res != ESP_OK){
		printf("Failed to sand frame of len %d because of %d.\n", wur_context->current_len, res);
	}
	printf("Successfully sent frame of len %d!\n", wur_context->current_len);
#else
    CLEAR_OUTPUT;
    portEXIT_CRITICAL(&wlanGroupMux);
#endif
    return ESP_OK;
}


wur_errors_t IRAM_ATTR ook_wur_transmit_frame(uint8_t* data_bytes, uint8_t data_bytes_len){
	if(wlan_wur_transmit_frame(&wur_ctxt, data_bytes, data_bytes_len) != ESP_OK){
		return WUR_KO;
	}
	return WUR_OK;
}


#endif /*USE_ESP_VERSION*/