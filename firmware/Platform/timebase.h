#ifndef GOFLY_PLATFORM_TIMEBASE_H
#define GOFLY_PLATFORM_TIMEBASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint32_t gofly_time_now_us(void);
uint32_t gofly_time_now_ms(void);
void gofly_time_set_now_us(uint32_t timestamp_us);

#ifndef GOFLY_HOST_TEST
#define GOFLY_HOST_TEST 0
#endif

#if GOFLY_HOST_TEST
void gofly_time_set_for_test(uint32_t timestamp_us);
#endif

static inline uint32_t gofly_time_elapsed_us(uint32_t now_us,
                                             uint32_t then_us)
{
    return now_us - then_us;
}

#ifdef __cplusplus
}
#endif

#endif
