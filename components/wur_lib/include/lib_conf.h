#ifndef _LIB_CONF_H_
#define _LIB_CONF_H_

#define USE_ESP_VERSION
//#define USE_EFR_VERSION

#if (defined USE_ESP_VERSION) && defined(USE_EFR_VERSION)
#error "Only one target platform can be used."
#endif

#if !(defined USE_ESP_VERSION) && !defined(USE_EFR_VERSION)
#error "Please, chose one valid traget platform at the start of lib_conf.h."
#endif

/* add common ESP_32 includes*/
#ifdef USE_ESP_VERSION

#include "esp_system.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"

#define I2C_WAKEUP_GPIO GPIO_NUM_19

#define USE_FREERTOS
#endif /* USE_ESP_VERSION */

/* add common EFR_32 includes*/
#ifdef USE_EFR_VERSION
#define NO_RTOS
#endif /*USE_EFR_VERSION*/


#ifdef USE_FREERTOS

#define WuRBinarySemaphoreHandle_t SemaphoreHandle_t
#define WuRRecursiveMutexHandle_t SemaphoreHandle_t

#define WuRBinarySemaphoreCreate() xSemaphoreCreateBinary()
#define WuRBinarySemaphoreGive(x) xSemaphoreGive(x)
#define WuRBinarySemaphoreTake(x, y) xSemaphoreTake(x, y)

#define WuRRecursiveMutexCreate() xSemaphoreCreateRecursiveMutex()
#define WuRRecursiveMutexGive(x) xSemaphoreGiveRecursive(x)
#define WuRRecursiveMutexTake(x, y) xSemaphoreTakeRecursive(x, y)

#define WuRTaskCreate(task_function, task_name, stack_size, args_p, priority, task_handle) xTaskCreate(task_function, task_name, stack_size, args_p, priority, task_handle)

#define WuRTickPeriodMS portTICK_PERIOD_MS

#else /* USE_FREERTOS */

#define WuRBinarySemaphoreHandle_t uint32_t
#define WuRRecursiveMutexHandle_t uint32_t

#define WuRBinarySemaphoreCreate() 
#define WuRBinarySemaphoreGive(x) 
#define WuRBinarySemaphoreTake(x) 

#define WuRRecursiveMutexCreate() 
#define WuRRecursiveMutexGive(x) 
#define WuRRecursiveMutexTake(x) 

#define WuRTaskCreate(task_function, task_name, stack_size, args_p, priority, task_handle) 

#define WuRTickPeriodMS 0

#endif /* USE_FREERTOS*/

typedef enum wur_errors{
    WUR_OK = 0,
    WUR_KO = 1
}wur_errors_t;

#endif /*_LIB_CONF_H_*/