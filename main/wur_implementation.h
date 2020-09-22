/*
 * wur-implementation.h
 *
 *      Author: marti
 */


/*
MIT License

Copyright (c) 2020 marticervia

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#ifndef WUR_IMPLEMENTATION_H_
#define WUR_IMPLEMENTATION_H_

#define WUR_ADDR 0x0111
#define MAX_APP_DATA_BUF 96

#define ADDR_OFFSET 0
#define PAYLOAD_OFFSET 2
#define HTTP_TIMEOUT 30000
//Addr plus ms awake
#define MIN_WAKE_REQ_LEN 4

//Addr plus min payload of 1 byte
#define MIN_DATA_REQ_LEN 3
#define TOTAL_TEST_FRAMES 100
#define TEST_REASON_LEN 128
#define TEST_FRAME_SIZE 64
#define TEST_ADDR 0x0555
#define TEST_WAKE_INTERVAL 30000

typedef enum  app_status{
	APP_IDLE = 0,
	APP_SENDING_WAKE = 1,
	APP_WAITING_WAKE = 2,
	APP_RESPONDING_WAKE = 3,
	APP_SENDING_DATA = 4,
	APP_WAITING_DATA = 5,
	APP_RESPONDING_DATA = 6,
	TEST_GENERATE_FRAME = 7,
	TEST_SEND_FRAME = 8,
	TEST_WAIT_FRAME = 9,
	TEST_COMPLETE_OK_FRAME = 10,
	TEST_COMPLETE_KO_FRAME = 11,
	TEST_COMPLETE_FAILURE = 12,
	TEST_SENDING_WAKE = 13,
	TEST_WAITING_WAKE = 14
}app_status_t;

typedef enum  test_status{
	TEST_IDLE = 0,
	TEST_IN_PROGRESS = 1,
	TEST_FAILED = 2,
	TEST_FINISHED = 3
}test_status_t;

typedef enum  app_trans_result{
	APP_TRANS_OK = 0,
	APP_TRANS_KO_TX = 1,
	APP_TRANS_KO_ACK = 2,
	APP_TRANS_KO_TIMEOUT = 3
}app_trans_result_t;

typedef struct app_ctxt{
	app_status_t 		 app_status;
	uint8_t 		 	 app_data_buf[MAX_APP_DATA_BUF];
	uint8_t 		 	 app_data_buf_len;
	int16_t 			 app_response_code;
}app_ctxt_t;

typedef struct test_ctxt{
	test_status_t 		 test_status;
	uint16_t 		 	 total_frames;
	uint16_t 		 	 current_frame;
	uint16_t 			 OK_frames;
	uint16_t 			 KO_frames;
	uint32_t 			 start_timestamp;
	uint32_t 			 finish_timestamp;
	char			     failure_reason[TEST_REASON_LEN];
}test_ctxt_t;


esp_err_t WuRWakeDevice(httpd_req_t *req);
esp_err_t WuRRequestDevice (httpd_req_t *req);

void WuRSystemTick(uint32_t timestamp);

#endif /* WUR_IMPLEMENTATION_H_ */
