#include "esp_system.h"
#include "utils.h"
#include <sys/time.h>

uint32_t get_timestamp_ms(void) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (uint32_t)(tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}
