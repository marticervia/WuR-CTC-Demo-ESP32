#include <string.h>
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


#include "wur.h"
#include "i2c_wur.h"
#include "ook_wur.h"
#include "utils.h"
#include "wur_implementation.h"

static app_ctxt_t app_ctxt;
static SemaphoreHandle_t app_mutex;
static SemaphoreHandle_t app_semaphore;

/* wur related handlers */

//TODO: USE_ESP32_SDK EQUIVALENT for HTTP
/*
void WuRWakeDevice(EmberCoapCode code,
                     uint8_t *uri,
                     EmberCoapReadOptions *options,
                     const uint8_t *payload,
                     uint16_t payloadLength,
                     const EmberCoapRequestInfo *info){

    uint32_t current_timestamp = get_timestamp_ms();
    printf("[%d]: Got Wake Device REQ!", current_timestamp);

    if(app_ctxt.app_status != APP_IDLE){
        printf("[%d]: Refusing Wake Device REQ, already busy!", current_timestamp);
        emberCoapRespondWithCode(info, EMBER_COAP_CODE_412_PRECONDITION_FAILED);
        return;
    }

    if((payloadLength > MAX_APP_DATA_BUF) || (payloadLength < MIN_WAKE_REQ_LEN)){
        printf("[%d]: Payload of %d bytes disallowed for WAKE!", current_timestamp, payloadLength);
        emberCoapRespondWithCode(info, EMBER_COAP_CODE_400_BAD_REQUEST);
        return;
    }

    memcpy(app_ctxt.app_data_buf, payload, payloadLength);
    app_ctxt.app_data_buf_len = payloadLength;

    emberSaveRequestInfo(info, &(app_ctxt.app_request_info));
    app_ctxt.app_status = APP_SENDING_WAKE;
}
*/

//TODO: USE_ESP32_SDK EQUIVALENT for HTTP
/*
void WuRRequestDevice(EmberCoapCode code,
                         uint8_t *uri,
                         EmberCoapReadOptions *options,
                         const uint8_t *payload,
                         uint16_t payloadLength,
                         const EmberCoapRequestInfo *info){
    uint32_t current_timestamp = get_timestamp_ms();
    printf("[%d]: Got Data Device REQ!", current_timestamp);

    if(app_ctxt.app_status != APP_IDLE){
        printf("[%d]: Refusing DATA to Device REQ, already busy!", current_timestamp);
        emberCoapRespondWithCode(info, EMBER_COAP_CODE_412_PRECONDITION_FAILED);
        return;
    }

    if((payloadLength > MAX_APP_DATA_BUF) || (payloadLength < MIN_DATA_REQ_LEN)){
        printf("[%d]: Payload of %d bytes disallowed for DATA!", current_timestamp, payloadLength);
        emberCoapRespondWithCode(info, EMBER_COAP_CODE_400_BAD_REQUEST);
        return;
    }

    memcpy(app_ctxt.app_data_buf, payload, payloadLength);
    app_ctxt.app_data_buf_len = payloadLength;

    emberSaveRequestInfo(info, &(app_ctxt.app_request_info));
    app_ctxt.app_status = APP_SENDING_DATA;
}
*/


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
    //TODO: USE_ESP32_SDK EQUIVALENT
    /*
     emberCoapRespondWithPayload(&app_ctxt.app_request_info, EMBER_COAP_CODE_500_INTERNAL_SERVER_ERROR,
            res_payload, 2);
    */
}

static void _respondWithPayload(uint8_t* payload, uint8_t payload_len){
    if(payload_len > 0){
        //TODO: USE_ESP32_SDK EQUIVALENT
        /*
         emberCoapRespondWithPayload(&app_ctxt.app_request_info, EMBER_COAP_CODE_203_VALID,
                 &payload[WUR_PAYLOAD_OFFSET], payload_len - WUR_PAYLOAD_OFFSET);
        */
    }
    //TODO: USE_ESP32_SDK EQUIVALENT
    /*
    emberCoapRespondWithCode(&app_ctxt.app_request_info, EMBER_COAP_CODE_203_VALID);
    */
}


static void _printBuffer(uint8_t* res, uint8_t res_length){
    uint16_t i;
    printf("Buffer is:");

    for(i=0; i < (res_length -1); i++){
        printf("%01X:", res[i]);
    }
    printf("%01X", res[i]);
}

static void _wur_tx_cb(wur_tx_res_t tx_res){
    uint32_t current_timestamp = get_timestamp_ms();

    xSemaphoreTakeRecursive(app_mutex, portMAX_DELAY);
    switch(app_ctxt.app_status){
        case APP_WAITING_WAKE:
        case APP_WAITING_DATA:
            if(tx_res == WUR_ERROR_TX_OK){
                if(app_ctxt.app_status == APP_WAITING_WAKE){
                    printf("[%d]: Going to respond to Wake!", current_timestamp);
                    app_ctxt.app_status = APP_RESPONDING_WAKE;

                }
                else{
                    printf("[%d]: Going to respond to Data!", current_timestamp);
                    app_ctxt.app_status = APP_RESPONDING_DATA;
                }
                memset(app_ctxt.app_data_buf, 0, MAX_APP_DATA_BUF);
                app_ctxt.app_data_buf_len = 0;
            }
            else if((tx_res ==  WUR_ERROR_TX_ACK_DATA_TIMEOUT)
                    || (tx_res ==  WUR_ERROR_TX_ACK_WAKE_TIMEOUT)){
                printf("[%d]: Received Timeout, going to idle!", current_timestamp);
                app_ctxt.app_status = APP_IDLE;
                _respondWithError(APP_TRANS_KO_TIMEOUT);
            }
            else{
                printf("[%d]: Received Error, going to idle!", current_timestamp);
                app_ctxt.app_status = APP_IDLE;
                _respondWithError(APP_TRANS_KO_TIMEOUT);
            }
            break;
        default:
            printf("[%d]: Received ACK while not waiting. Is this an error?!", current_timestamp);
            break;
    }
    xSemaphoreGiveRecursive(app_mutex);
    xSemaphoreGive(app_semaphore);
}


static void _wur_rx_cb(wur_rx_res_t rx_res, uint8_t* rx_bytes, uint8_t rx_bytes_len){
    uint32_t current_timestamp = get_timestamp_ms();

    printf("[%d]: Received response with status %d!", current_timestamp, rx_res);
    _printBuffer(rx_bytes, rx_bytes_len);
    if(rx_bytes_len > MAX_APP_DATA_BUF){
        printf("[%d]: Received response above max frame size %d!", current_timestamp, rx_bytes_len);
    }

    xSemaphoreTakeRecursive(app_mutex, portMAX_DELAY);
    memcpy(app_ctxt.app_data_buf, rx_bytes, rx_bytes_len);
    app_ctxt.app_data_buf_len = rx_bytes_len;
    xSemaphoreGiveRecursive(app_mutex);

    xSemaphoreGive(app_semaphore);
}


void WuRInitApp(void){
    wur_init(WUR_ADDR);

    //TODO: USE_ESP32_SDK EQUIVALENT
    /*
    memset(&app_ctxt.app_request_info, 0, sizeof(EmberCoapRequestInfo));
    */
    app_ctxt.app_status = APP_IDLE;
    memset(app_ctxt.app_data_buf, 0, MAX_APP_DATA_BUF);
    app_ctxt.app_data_buf_len = 0;
    wur_set_tx_cb(_wur_tx_cb);
    wur_set_rx_cb(_wur_rx_cb);

    app_mutex = xSemaphoreCreateRecursiveMutex();
    xSemaphoreGiveRecursive(app_mutex);

    app_semaphore = xSemaphoreCreateBinary();
}

void WuRAppTick(void){
    
    /* Unblocked by USER actions to send WAKE, DATA and their responses*/
    uint32_t current_timestamp = get_timestamp_ms();
    uint16_t wur_addr, wake_ms;
    wur_tx_res_t tx_res;
    uint32_t current_app_status = 10;

    while(current_app_status != app_ctxt.app_status){
        current_app_status = app_ctxt.app_status;
        switch(app_ctxt.app_status){
            case APP_IDLE:
                printf("[%d]: Device IDLE", current_timestamp);
                break;
            case APP_SENDING_WAKE:
                printf("[%d]: Sending Wake Device REQ!", current_timestamp);
                wur_addr = _getuint16t(app_ctxt.app_data_buf);
                wake_ms = _getuint16t(&app_ctxt.app_data_buf[PAYLOAD_OFFSET]);

                tx_res = wur_send_wake(wur_addr, wake_ms);
                if(tx_res != WUR_ERROR_TX_OK){
                    printf("[%d]: Failure to send Wake Device REQ!", current_timestamp);
                    _respondWithError(APP_TRANS_KO_TX);
                    app_ctxt.app_status = APP_IDLE;
                }
                app_ctxt.app_status = APP_WAITING_WAKE;
                break;
            case APP_SENDING_DATA:
                printf("[%d]: Sending Data to Device REQ!", current_timestamp);
                wur_addr = _getuint16t(app_ctxt.app_data_buf);

                tx_res = wur_send_data(wur_addr, &app_ctxt.app_data_buf[PAYLOAD_OFFSET], app_ctxt.app_data_buf_len - PAYLOAD_OFFSET, false, -1);
                if(tx_res != WUR_ERROR_TX_OK){
                    printf("[%d]: Failure to send Data to Device REQ!", current_timestamp);
                    _respondWithError(APP_TRANS_KO_TX);
                    app_ctxt.app_status = APP_IDLE;
                }
                app_ctxt.app_status = APP_WAITING_DATA;
                break;
            case APP_WAITING_WAKE:
            case APP_WAITING_DATA:
                /* wait for the OK/KO Tx callback to change the state*/
                if(current_timestamp % 500 == 0){
                    printf("[%d]: Device Waiting ACK", current_timestamp);
                }
                break;
            case APP_RESPONDING_WAKE:
                printf("[%d]: Sending Response to Wake Device REQ!", current_timestamp);
                _respondWithPayload(NULL, 0);
                app_ctxt.app_status = APP_IDLE;
                break;
            case APP_RESPONDING_DATA:
                printf("[%d]: Sending Response to Data to Device REQ!", current_timestamp);
                _respondWithPayload(app_ctxt.app_data_buf, app_ctxt.app_data_buf_len );
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

        xSemaphoreTakeRecursive(app_mutex, portMAX_DELAY);
        WuRAppTick();
        xSemaphoreTakeRecursive(app_mutex, portMAX_DELAY);
    }
}


