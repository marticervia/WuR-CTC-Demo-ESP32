
/*
 * ook_wur.c
 *
 *  Created on: 29 jul. 2019
 *      Author: marti
 */

#include "lib_conf.h"

#include "ook_com.h"
#include "ook_wur.h"
#include "utils.h"

#include <string.h>


static const uint8_t crc8_table[] = {
      0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
    157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
     35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
    190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
     70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
    219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
    101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
     17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
     50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
    202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
     87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53
};

static uint8_t _ook_calculate_crc8(uint8_t* payload, uint8_t payload_len){
    uint8_t crc = 0;
    int16_t i = 0;

    for(i = 0; i < payload_len; i++){
        crc = crc8_table[crc ^ payload[i]];
    }
    
    return crc;
}

static ook_tx_errors_t _ook_wur_transmit(uint8_t* data, uint8_t len){
	wur_errors_t wur_res;
	uint8_t crc8;
	uint16_t i;

	/*printf("Calculate CRC of data with len %d:\n", len);
	printf("0x");
	for(i = 0; i < len -2; i++){
		printf("%02X:", data[i]);
	}
	printf("%02X\n", data[i]);
	*/

	crc8 = _ook_calculate_crc8(data, len - 1);
	//printf("Got CRC of %02X.\n", crc8);
	data[len-1] = crc8; 

	//printf("Send frame:\n");
	//print_frame(data, len);

	wur_res = ook_wur_transmit_frame(data, len);
	if(wur_res != WUR_OK){
		return OOK_WUR_TX_ERROR_FAILED;
	}

	return OOK_WUR_TX_ERROR_SUCCESS;
}


void ook_wur_init(void){
	ook_wur_init_context();
}

ook_tx_errors_t ook_wur_wake(uint16_t dest, uint16_t ms_wake, uint8_t seq){
	uint8_t wake_frame[5] = {0};


	//printf("Sending OOK wake frame to addr %02X .\n", dest);

	wake_frame[0] = (uint8_t)(dest & 0x00FF);
	wake_frame[1] = (uint8_t)((dest & 0x0F00) >> 4);

	wake_frame[1] |= ((WAKE_FLAG) << 1) | seq;
	wake_frame[2] = WUR_WAKE_LEN;
	ms_wake = htons(ms_wake);
	memcpy(&wake_frame[3], &ms_wake, 2);

	return _ook_wur_transmit(wake_frame, WUR_WAKE_LEN + WUR_HEADER_LEN);
}

ook_tx_errors_t ook_wur_data(uint16_t dest, uint8_t* data, uint8_t len, bool ack, uint8_t seq){
	uint8_t data_frame[WUR_MAX_DATA_LEN] = {0};
	uint8_t flags = 0;

	if(len > WUR_MAX_DATA_LEN){
		return OOK_WUR_TX_ERROR_FRAME_FORMAT;
	}

	//printf("Sending OOK data frame to addr %02X with len %d.\n", dest, len);

	data_frame[0] = (uint8_t)(dest & 0x00FF);
	data_frame[1] = (uint8_t)((dest & 0x0F00) >> 4);

	/* set type and seq flag */
	flags |= DATA_FLAG;
	if(ack){
		flags |= ACK_FLAG;
	}

	data_frame[1] |= (flags << 1) | seq;
	data_frame[2] = len;

	memcpy(&data_frame[3], data, len);

	return _ook_wur_transmit(data_frame, len + WUR_HEADER_LEN);
}

static uint8_t ack_buffer[6];

ook_tx_errors_t ook_wur_ack(uint16_t dest, uint8_t seq){
	return ook_wur_data(dest, ack_buffer, 0, true, seq);
}



