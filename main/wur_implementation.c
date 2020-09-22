/*
 * wur-implementation.c
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

#include <string.h>
#include <esp_http_server.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "driver/i2c.h"
#include "cJSON.h"

#include "wur.h"
#include "i2c_wur.h"
#include "ook_wur.h"
#include "utils.h"
#include "wur_implementation.h"
#include "index.h"

#define TAG "APP"
#define ESP_WIFI_SSID      "radiuspieap"
#define ESP_WIFI_PASS      ""

static app_ctxt_t app_ctxt;
static test_ctxt_t test_ctxt;
static SemaphoreHandle_t app_mutex;
static SemaphoreHandle_t app_semaphore;
static SemaphoreHandle_t http_semaphore;
static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
static uint8_t bssid[6] = {0};

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        memcpy(bssid, event->event_info.connected.bssid, 6);
        break;

    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        {
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            ESP_LOGI(TAG,"connect to the AP fail\n");
            break;
        }
    default:
        break;
    }
    return ESP_OK;
}

esp_err_t WuRServeIndex(httpd_req_t *req){
    esp_err_t res;
    ESP_LOGI(TAG,"Received /index.html request");
    size_t size = strlen(MAIN_page);
    res = httpd_resp_send(req, MAIN_page, size);
    ESP_LOGI(TAG,"Sent response with size %d with code %d", size, res);
    return ESP_OK;
}


/* wur related handlers */
esp_err_t WuRWakeDevice(httpd_req_t *req){
    ESP_LOGI(TAG,"Received /wur/wake request");
    uint32_t current_timestamp = get_timestamp_ms();
    printf("[%d]: Got Wake Device REQ!\n", current_timestamp);
    char content[300];
    char resp[256];

    if(app_ctxt.app_status != APP_IDLE){
        sprintf(resp, "[%d]: Refusing Wake Device REQ, already busy!", current_timestamp);
        printf("%s\n", resp);
        httpd_resp_set_status(req, "412 Precondition Failed");
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    }

    int payloadLength = httpd_req_recv(req, content, sizeof(content));
    cJSON *root = cJSON_Parse(content);
    if(root == NULL){
        sprintf(resp, "[%d]: Payload of %d bytes is NOT JSON!", current_timestamp, payloadLength);
        printf("%s\n", resp);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    }

    cJSON *address = cJSON_GetObjectItem(root,"wur_address");
    if((address == NULL) || (!cJSON_IsString(address))){
        sprintf(resp, "[%d]: Error parsing addres from JSON!", current_timestamp);
        printf("%s\n", resp);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    }
    char* addr_str = address->valuestring;
    uint16_t addr = (int)strtol(addr_str, NULL, 16);

    if(!addr){
        sprintf(resp, "[%d]: Error parsing a valid address from %s!", current_timestamp, addr_str);
        printf("%s\n", resp);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    }

    cJSON *wake_ms = cJSON_GetObjectItem(root,"wake_time");
    if((wake_ms == NULL) || (!cJSON_IsNumber(wake_ms))){
        sprintf(resp, "[%d]: Error parsing wake time from JSON!", current_timestamp);
        printf("%s\n", resp);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    }

    uint16_t wake_time = wake_ms->valueint;
    /* Use the format expected by wur module*/
    addr = htons(addr);
    memcpy(app_ctxt.app_data_buf, &addr, sizeof(uint16_t));
    wake_time = htons(wake_time);
    memcpy(&app_ctxt.app_data_buf[2], &wake_time, sizeof(uint16_t));
    app_ctxt.app_data_buf_len = 4;
    app_ctxt.app_status = APP_SENDING_WAKE;
    xSemaphoreGive(app_semaphore);

    if(xSemaphoreTake(http_semaphore, HTTP_TIMEOUT/portTICK_PERIOD_MS) == pdTRUE){
        if(app_ctxt.app_response_code != 200){
            httpd_resp_set_status(req, "500 Internal Server Error");
        }
        httpd_resp_send(req, (char*)app_ctxt.app_data_buf, app_ctxt.app_data_buf_len);
    }else{
        httpd_resp_send_408(req);
    }
    return ESP_OK;
}

esp_err_t WuRRequestDevice(httpd_req_t *req){
    ESP_LOGI(TAG,"Received /wur/data request");
    uint32_t current_timestamp = get_timestamp_ms();
    printf("[%d]: Got Data Device REQ!\n", current_timestamp);
    char content[300];
    char resp[256];

    if(app_ctxt.app_status != APP_IDLE){
        sprintf(resp, "[%d]: Refusing DATA to Device REQ, already busy!\n", current_timestamp);
        printf("%s\n", resp);
        httpd_resp_set_status(req, "412 Precondition Failed");
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    }

    int payloadLength = httpd_req_recv(req, content, sizeof(content));

    cJSON *root = cJSON_Parse(content);
    if(root == NULL){
        sprintf(resp, "[%d]: Payload of %d bytes is NOT JSON!", current_timestamp, payloadLength);
        printf("%s\n", resp);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    }

    cJSON *address = cJSON_GetObjectItem(root,"wur_address");
    if((address == NULL) || (!cJSON_IsString(address))){
        sprintf(resp, "[%d]: Error parsing addres from JSON!", current_timestamp);
        printf("%s\n", resp);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    }
    char* addr_str = address->valuestring;
    uint16_t addr = (int)strtol(addr_str, NULL, 16);
    if(!addr){
        sprintf(resp, "[%d]: Error parsing a valid address from %s!", current_timestamp, addr_str);
        printf("%s\n", resp);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    }

    addr = htons(addr);
    memcpy(app_ctxt.app_data_buf, &addr, sizeof(uint16_t));

    cJSON *data_json = cJSON_GetObjectItem(root,"data");
    if((data_json == NULL) || (!cJSON_IsString(data_json))){
        sprintf(resp, "[%d]: Error parsing data string from JSON!", current_timestamp);
        printf("%s\n", resp);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, resp, strlen(resp));
        return ESP_OK;
    }

    char* data_str = data_json->valuestring;
    uint16_t data_str_len = strlen(data_str);

    for( int16_t i = 0; i < data_str_len; i = i+2){
        char hex_byte[3] = {0};
        memcpy(hex_byte, data_str + i, 2);
        hex_byte[2] = '\0';
        app_ctxt.app_data_buf[2 + (i/2)] = (int)strtol(hex_byte, NULL, 16);
    }
    app_ctxt.app_data_buf_len = 2 + (data_str_len/2);
    printf("Sending data with len %d", app_ctxt.app_data_buf_len);
    print_frame(app_ctxt.app_data_buf, app_ctxt.app_data_buf_len);
    app_ctxt.app_status = APP_SENDING_DATA;
    xSemaphoreGive(app_semaphore);

    if(xSemaphoreTake(http_semaphore, HTTP_TIMEOUT/portTICK_PERIOD_MS) == pdTRUE){
        if(app_ctxt.app_response_code != 200){
            httpd_resp_set_status(req, "500 Internal Server Error");
        }
        httpd_resp_send(req, (char*)app_ctxt.app_data_buf, app_ctxt.app_data_buf_len);
    }else{
        httpd_resp_send_408(req);
    }

    return ESP_OK;
}

static void init_test_context(test_ctxt_t* ctxt){
    ctxt->start_timestamp = get_timestamp_ms();
    ctxt->finish_timestamp = 0;
    ctxt->current_frame = 0;
    ctxt->total_frames = TOTAL_TEST_FRAMES;
    ctxt->KO_frames= 0;
    ctxt->OK_frames = 0;
    ctxt->test_status = TEST_IN_PROGRESS;
    memset(ctxt->failure_reason, 0, TEST_REASON_LEN);
}


static void generate_test_frame(uint8_t* buffer, uint16_t req_len_bytes){
    esp_fill_random(buffer, req_len_bytes);    
}

static void fail_test_context(test_ctxt_t* ctxt, char* reason, uint32_t fail_timestamp){
    ctxt->test_status = TEST_FAILED;
    strcpy(ctxt->failure_reason, reason);
    ctxt->finish_timestamp = fail_timestamp;
    printf("[%d]: Finished test with failure on frame: %d/%d !\n", fail_timestamp, test_ctxt.current_frame, test_ctxt.total_frames);
    printf("[%d]: Test run in %d milliseconds!\n", fail_timestamp, test_ctxt.finish_timestamp - test_ctxt.start_timestamp);
    app_ctxt.app_status = APP_IDLE;

}

static void aprove_test_context(test_ctxt_t* ctxt, uint32_t success_timestamp){
    ctxt->test_status = TEST_FINISHED;
    ctxt->finish_timestamp = success_timestamp;
    printf("[%d]: Finished test with results: %d/%d !\n", success_timestamp, test_ctxt.OK_frames, test_ctxt.total_frames);
    printf("[%d]: Test run in %d milliseconds!\n", success_timestamp, test_ctxt.finish_timestamp - test_ctxt.start_timestamp);
    uint32_t bitrate = (test_ctxt.OK_frames * TEST_FRAME_SIZE * 8* 1000) / ((test_ctxt.finish_timestamp - test_ctxt.start_timestamp));
    printf("[%d]: Calculated bitrate is: %d !\n", success_timestamp, bitrate);
    app_ctxt.app_status = APP_IDLE;
}


static void update_text_context(test_ctxt_t* ctxt, bool OK_result){
    ctxt->current_frame++;
    if(OK_result){
        ctxt->OK_frames++;
    }
    else{
        ctxt->KO_frames++;
    }
    if(ctxt->current_frame == ctxt->total_frames){
        aprove_test_context(ctxt, get_timestamp_ms());
    }    
}


esp_err_t WuRStartTest(httpd_req_t *req){
    char* response = "{\"ok\":true}";

    ESP_LOGI(TAG,"Received /test/start request");

    if(test_ctxt.test_status == TEST_IN_PROGRESS){
        httpd_resp_send_500(req);
        return ESP_OK;
    }

    init_test_context(&test_ctxt);
    app_ctxt.app_status = TEST_SENDING_WAKE;
    xSemaphoreGive(app_semaphore);
    
    httpd_resp_send(req, response, strlen(response));
    ESP_LOGI(TAG,"Responded /test/start request");

    return ESP_OK;
}

esp_err_t WuRReportTest(httpd_req_t *req){
    cJSON * response = cJSON_CreateObject();
    char* cjson_buff;

    if(!response){
        printf("Failure to initialize JSON repsonse");
        httpd_resp_send_500(req);
        return ESP_OK;
    }

    switch(test_ctxt.test_status){
        case TEST_IDLE:
            cJSON_AddStringToObject(response, "status", "idle");
            break;
        case TEST_IN_PROGRESS:
            cJSON_AddStringToObject(response, "status", "in_progress");
            break;
        case TEST_FINISHED:
            cJSON_AddStringToObject(response, "status", "finished");
            break;
        case TEST_FAILED:
            cJSON_AddStringToObject(response, "status", "failed");
            break;
    }

    cJSON_AddNumberToObject(response, "currentFrame", test_ctxt.current_frame);
    cJSON_AddNumberToObject(response, "totalFrames", test_ctxt.total_frames);
    cJSON_AddNumberToObject(response, "framesOK", test_ctxt.OK_frames);
    cJSON_AddNumberToObject(response, "framesKO", test_ctxt.KO_frames);
    if(test_ctxt.finish_timestamp == 0){
        cJSON_AddNumberToObject(response, "runTime", get_timestamp_ms() - test_ctxt.start_timestamp);
    }
    else{
        cJSON_AddNumberToObject(response, "runTime", test_ctxt.finish_timestamp - test_ctxt.start_timestamp);
    }
    
    cjson_buff = cJSON_Print(response);
    if(!cjson_buff){
        printf("Failure to initialize JSON repsonse buffer");
        httpd_resp_send_500(req);
        return ESP_OK;
    }

    httpd_resp_send(req, cjson_buff, strlen(cjson_buff));
    free(cjson_buff);
    return ESP_OK;
}

static inline uint16_t _getuint16t(uint8_t* buff){
    uint16_t wur_addr;
    memcpy(&wur_addr, buff, sizeof(uint16_t));
    wur_addr = ntohs(wur_addr);
    return wur_addr;
}

static inline void _setint16t(uint8_t* buff, int16_t num){
    num = htons(num);
    memcpy(buff, &num, sizeof(int16_t));
}

static void _respondWithError(app_trans_result_t res){
    const uint8_t res_payload[2] = {0};
    _setint16t((uint8_t*) res_payload, res);
    app_ctxt.app_response_code = 500;
    xSemaphoreGive(http_semaphore);
}

static void _respondWithPayload(uint8_t* payload, uint8_t payload_len){

    memcpy(app_ctxt.app_data_buf, payload, payload_len);
    app_ctxt.app_data_buf_len = payload_len;
    app_ctxt.app_response_code = 200;
    xSemaphoreGive(http_semaphore);
}


static void _printBuffer(uint8_t* res, uint8_t res_length){
    uint16_t i;
    printf("Buffer is:\n");

    for(i=0; i < (res_length -1); i++){
        printf("%01X:", res[i]);
    }
    printf("%01X\n", res[i]);
}

static void _wur_tx_cb(wur_tx_res_t tx_res){
    uint32_t current_timestamp = get_timestamp_ms();

    //printf("[%d]: TX Callback!!\n", current_timestamp);
    xSemaphoreTakeRecursive(app_mutex, portMAX_DELAY);
    //printf("[%d]: TX Callback takes APP mutex!!\n", current_timestamp);

    switch(app_ctxt.app_status){
        case APP_WAITING_WAKE:
        case APP_WAITING_DATA:
            if(tx_res == WUR_ERROR_TX_OK){
                if(app_ctxt.app_status == APP_WAITING_WAKE){
                    //printf("[%d]: Going to respond to Wake!\n", current_timestamp);
                    app_ctxt.app_status = APP_RESPONDING_WAKE;

                }
                else{
                    //printf("[%d]: Going to respond to Data!\n", current_timestamp);
                    app_ctxt.app_status = APP_RESPONDING_DATA;
                }
                memset(app_ctxt.app_data_buf, 0, MAX_APP_DATA_BUF);
                app_ctxt.app_data_buf_len = 0;
            }
            else if((tx_res ==  WUR_ERROR_TX_ACK_DATA_TIMEOUT)
                    || (tx_res ==  WUR_ERROR_TX_ACK_WAKE_TIMEOUT)){
                printf("[%d]: Received Timeout, going to idle!\n", current_timestamp);
                app_ctxt.app_status = APP_IDLE;
                _respondWithError(APP_TRANS_KO_TIMEOUT);
            }
            else{
                printf("[%d]: Received Error, going to idle!\n", current_timestamp);
                app_ctxt.app_status = APP_IDLE;
                _respondWithError(APP_TRANS_KO_TIMEOUT);
            }
            break;
        case TEST_WAITING_WAKE:
            if(tx_res == WUR_ERROR_TX_OK){
                app_ctxt.app_status = TEST_GENERATE_FRAME;
            }
            else{
                app_ctxt.app_status = TEST_COMPLETE_FAILURE;
            }
            break;
        case TEST_WAIT_FRAME:
            if(tx_res == WUR_ERROR_TX_OK){
                //printf("[%d]: Received ACK!\n", current_timestamp);
                app_ctxt.app_status = TEST_COMPLETE_OK_FRAME;
            }
            else if(tx_res == WUR_ERROR_TX_ACK_DATA_TIMEOUT){
                //printf("[%d]: Received NACK!\n", current_timestamp);
                app_ctxt.app_status = TEST_COMPLETE_KO_FRAME;
            }
            else{
                //printf("[%d]: Received Error!\n", current_timestamp);
                app_ctxt.app_status = TEST_COMPLETE_FAILURE;
            }
            memset(app_ctxt.app_data_buf, 0, MAX_APP_DATA_BUF);
            app_ctxt.app_data_buf_len = 0;
            break;
        default:
            //printf("[%d]: Received ACK while not waiting. Is this an error?!\n", current_timestamp);
            break;
    }
    //printf("[%d]: TX Callback awakes APP task!!\n", current_timestamp);
    xSemaphoreGiveRecursive(app_mutex);
    xSemaphoreGive(app_semaphore);
}


static void _wur_rx_cb(wur_rx_res_t rx_res, uint8_t* rx_bytes, uint8_t rx_bytes_len){
    uint32_t current_timestamp = get_timestamp_ms();

    //printf("[%d]: Received frame with status %d!\n", current_timestamp, rx_res);
    //_printBuffer(rx_bytes, rx_bytes_len);
    if(rx_bytes_len > MAX_APP_DATA_BUF){
        printf("[%d]: Received frame above max frame size %d!\n", current_timestamp, rx_bytes_len);
        return;
    }

    xSemaphoreTakeRecursive(app_mutex, portMAX_DELAY);
    memcpy(app_ctxt.app_data_buf, rx_bytes, rx_bytes_len);
    app_ctxt.app_data_buf_len = rx_bytes_len;
    xSemaphoreGiveRecursive(app_mutex);

    xSemaphoreGive(app_semaphore);
}


httpd_uri_t uri_page = {
    .uri      = "/index.html",
    .method   = HTTP_GET,
    .handler  = WuRServeIndex,
    .user_ctx = NULL
};

/* URI handler structure for GET /uri */
httpd_uri_t uri_wake = {
    .uri      = "/wur/wake",
    .method   = HTTP_POST,
    .handler  = WuRWakeDevice,
    .user_ctx = NULL
};

/* URI handler structure for POST /uri */
httpd_uri_t uri_data = {
    .uri      = "/wur/data",
    .method   = HTTP_POST,
    .handler  = WuRRequestDevice,
    .user_ctx = NULL
};

/* URI handler structure for POST /uri */
httpd_uri_t uri_test_start = {
    .uri      = "/wur/test/start",
    .method   = HTTP_POST,
    .handler  = WuRStartTest,
    .user_ctx = NULL
};

/* URI handler structure for POST /uri */
httpd_uri_t uri_test_status = {
    .uri      = "/wur/test/status",
    .method   = HTTP_GET,
    .handler  = WuRReportTest,
    .user_ctx = NULL
};



void WuRInitApp(void){

    s_wifi_event_group = xEventGroupCreate();
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             ESP_WIFI_SSID, ESP_WIFI_PASS);
    ESP_LOGI(TAG, "Waiting for AP connection...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP");

    wur_init(WUR_ADDR);

    app_ctxt.app_status = APP_IDLE;
    memset(app_ctxt.app_data_buf, 0, MAX_APP_DATA_BUF);
    app_ctxt.app_data_buf_len = 0;
    wur_set_tx_cb(_wur_tx_cb);
    wur_set_rx_cb(_wur_rx_cb);

    app_mutex = xSemaphoreCreateRecursiveMutex();
    xSemaphoreGiveRecursive(app_mutex);
    app_semaphore = xSemaphoreCreateBinary();
    http_semaphore = xSemaphoreCreateBinary();

    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Empty handle to esp_http_server */
    httpd_handle_t server = NULL;

    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK) {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &uri_wake);
        httpd_register_uri_handler(server, &uri_data);
        httpd_register_uri_handler(server, &uri_page);
        httpd_register_uri_handler(server, &uri_test_start);
        httpd_register_uri_handler(server, &uri_test_status);
    }
}

uint8_t test_data_buf[TEST_FRAME_SIZE];

void WuRAppTick(void){
    
    /* Unblocked by USER actions to send WAKE, DATA and their responses*/
    uint32_t current_timestamp = get_timestamp_ms();
    uint16_t wur_addr, wake_ms;
    wur_tx_res_t tx_res;
    uint32_t current_app_status = 0xFFFFFFFF;

    while(current_app_status != app_ctxt.app_status){
        current_app_status = app_ctxt.app_status;
        switch(app_ctxt.app_status){
            case APP_IDLE:
                printf("[%d]: Device IDLE\n", current_timestamp);
                break;


            case APP_SENDING_WAKE:
                printf("[%d]: Sending Wake Device REQ!\n", current_timestamp);
                wur_addr = _getuint16t(app_ctxt.app_data_buf);
                wur_addr = wur_addr & (0x03FF);

                wake_ms = _getuint16t(&app_ctxt.app_data_buf[PAYLOAD_OFFSET]);

                tx_res = wur_send_wake(wur_addr, wake_ms);
                if(tx_res != WUR_ERROR_TX_OK){
                    printf("[%d]: Failure to send Wake Device REQ!\n", current_timestamp);
                    _respondWithError(APP_TRANS_KO_TX);
                    app_ctxt.app_status = APP_IDLE;
                    break;
                }
                app_ctxt.app_status = APP_WAITING_WAKE;
                break;

            case APP_SENDING_DATA:
                printf("[%d]: Sending Data to Device REQ!\n", current_timestamp);
                wur_addr = _getuint16t(app_ctxt.app_data_buf);
                wur_addr = wur_addr & (0x03FF);

                app_ctxt.app_status = APP_WAITING_DATA;
                tx_res = wur_send_data(wur_addr, &app_ctxt.app_data_buf[PAYLOAD_OFFSET], app_ctxt.app_data_buf_len - PAYLOAD_OFFSET, false, -1);
                if(tx_res != WUR_ERROR_TX_OK){
                    printf("[%d]: Failure to send Data to Device REQ!\n", current_timestamp);
                    _respondWithError(APP_TRANS_KO_TX);
                    app_ctxt.app_status = APP_IDLE;
                    break;
                }
                break;

            case APP_WAITING_WAKE:
            case APP_WAITING_DATA:
                /* wait for the OK/KO Tx callback to change the state*/
                if(current_timestamp % 500 == 0){
                    printf("[%d]: Device Waiting ACK\n", current_timestamp);
                }
                break;

            case APP_RESPONDING_WAKE:
                //printf("[%d]: Sending Response to Wake Device REQ!\n", current_timestamp);
                _respondWithPayload(NULL, 0);
                app_ctxt.app_status = APP_IDLE;
                break;

            case APP_RESPONDING_DATA:
                //printf("[%d]: Sending Response to Data to Device REQ!\n", current_timestamp);
                _respondWithPayload(app_ctxt.app_data_buf, app_ctxt.app_data_buf_len );
                app_ctxt.app_status = APP_IDLE;
                break;

            case TEST_SENDING_WAKE:
                //printf("[%d]: Sending Test Wake Device REQ!\n", current_timestamp);
                wur_addr = TEST_ADDR & (0x03FF);
                wake_ms = TEST_WAKE_INTERVAL;

                app_ctxt.app_status = TEST_WAITING_WAKE;
                tx_res = wur_send_wake(wur_addr, wake_ms);
                if(tx_res != WUR_ERROR_TX_OK){
                    printf("[%d]: Failure to send Wake Device REQ!\n", current_timestamp);
                    _respondWithError(APP_TRANS_KO_TX);
                    app_ctxt.app_status = TEST_COMPLETE_FAILURE;
                    break;
                }
                //printf("[%d]: Sent Test Wake Device REQ!\n", current_timestamp);
                break;

            case TEST_WAITING_WAKE:
                //printf("Waiting frame!");
                break;


            case TEST_GENERATE_FRAME:
                //printf("Generating frame!");
                generate_test_frame(test_data_buf, TEST_FRAME_SIZE);
                app_ctxt.app_status = TEST_SEND_FRAME;
                break;

            case TEST_SEND_FRAME:
                //printf("[%d]: Sending Data to Device test, frame %d/%d!\n", current_timestamp, test_ctxt.current_frame, test_ctxt.total_frames);
                wur_addr = TEST_ADDR & (0x03FF);
                app_ctxt.app_status = TEST_WAIT_FRAME;
                tx_res = wur_send_data(wur_addr, (uint8_t*)&test_data_buf, TEST_FRAME_SIZE, false, -1);
                if(tx_res != WUR_ERROR_TX_OK){
                    printf("[%d]: Failure to send Data to Device REQ!\n", current_timestamp);
                    update_text_context(&test_ctxt, false);
                    app_ctxt.app_status = TEST_GENERATE_FRAME;
                }
                //printf("[%d]: Sent Test Data Device REQ!\n", current_timestamp);
                break;

            case TEST_WAIT_FRAME:
                //printf("Wait Frame!");
                break;

            case TEST_COMPLETE_OK_FRAME:
                //printf("Frame sent OK!");
                update_text_context(&test_ctxt, true);
                if(test_ctxt.test_status == TEST_IN_PROGRESS){
                    //printf("Prepare next frame!");
                    app_ctxt.app_status = TEST_GENERATE_FRAME;
                }
                break;
            case TEST_COMPLETE_KO_FRAME:
                //printf("Frame sent KO!");
                update_text_context(&test_ctxt, false);
                if(test_ctxt.test_status == TEST_IN_PROGRESS){
                    //printf("Prepare next frame!");
                    app_ctxt.app_status = TEST_GENERATE_FRAME;
                }
                break;
            case TEST_COMPLETE_FAILURE:
                //printf("Test failure!");
                fail_test_context(&test_ctxt, "Error while taking the test.\n", current_timestamp);
                app_ctxt.app_status = APP_IDLE;
                break;

            default:
                break;
        }
    }
}

void IRAM_ATTR app_main()
{

    WuRInitApp();

    while(1){

        xSemaphoreTake(app_semaphore, WUR_DEFAULT_TIMEOUT/portTICK_PERIOD_MS);
        //ESP_LOGI(TAG, "Wake from APP semaphore, status %d!\n", app_ctxt.app_status);
        xSemaphoreTakeRecursive(app_mutex, portMAX_DELAY);
        WuRAppTick();
        xSemaphoreGiveRecursive(app_mutex);
    }
}


