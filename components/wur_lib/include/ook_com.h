
#ifndef __ESP_WLAN_RAW_H__
#define  __ESP_WLAN_RAW_H__

#include "lib_conf.h"

#ifdef USE_ESP_VERSION

wur_errors_t ook_wur_init_context(void);
wur_errors_t ook_wur_transmit_frame(uint8_t* data_bytes, uint8_t data_bytes_len);

#endif /* USE_ESP_VERSION*/

#endif
