/*
 * wur-implementation.h
 *
 *  Created on: 29 jul. 2019
 *      Author: marti
 */

#ifndef WUR_IMPLEMENTATION_H_
#define WUR_IMPLEMENTATION_H_

#define WUR_ADDR 0x0555
#define MAX_APP_DATA_BUF 34

#define ADDR_OFFSET 0
#define PAYLOAD_OFFSET 2

//Addr plus ms awake
#define MIN_WAKE_REQ_LEN 4

//Addr plus min payload of 1 byte
#define MIN_DATA_REQ_LEN 3


typedef enum  app_status{
	APP_IDLE = 0,
	APP_SENDING_WAKE = 1,
	APP_WAITING_WAKE = 2,
	APP_RESPONDING_WAKE = 3,
	APP_SENDING_DATA = 4,
	APP_WAITING_DATA = 5,
	APP_RESPONDING_DATA = 6
}app_status_t;

typedef enum  app_trans_result{
	APP_TRANS_OK = 0,
	APP_TRANS_KO_TX = 1,
	APP_TRANS_KO_ACK = 2,
	APP_TRANS_KO_TIMEOUT = 3
}app_trans_result_t;

typedef struct app_ctxt{
	//TODO: Use ESP_32 Native
	/*
	EmberCoapRequestInfo app_request_info;
	*/
	app_status_t 		 app_status;
	uint8_t 		 	 app_data_buf[MAX_APP_DATA_BUF];
	uint8_t 		 	 app_data_buf_len;
}app_ctxt_t;

//TODO: Use ESP_32 Native
/*
void WuRWakeDevice(EmberCoapCode code,
                                         uint8_t *uri,
                                         EmberCoapReadOptions *options,
                                         const uint8_t *payload,
                                         uint16_t payloadLength,
                                         const EmberCoapRequestInfo *info);

void WuRRequestDevice (EmberCoapCode code,
                                         uint8_t *uri,
                                         EmberCoapReadOptions *options,
                                         const uint8_t *payload,
                                         uint16_t payloadLength,
                                         const EmberCoapRequestInfo *info);
*/
void WuRSystemTick(uint32_t timestamp);

#endif /* WUR_IMPLEMENTATION_H_ */
