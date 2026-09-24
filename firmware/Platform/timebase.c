#include "timebase.h"

#include <stdatomic.h>

static _Atomic uint32_t gofly_time_us;

uint32_t gofly_time_now_us(void)
{
    return atomic_load_explicit(&gofly_time_us, memory_order_relaxed);
}

uint32_t gofly_time_now_ms(void)
{
    return gofly_time_now_us() / 1000U;
}

void gofly_time_set_now_us(uint32_t timestamp_us)
{
    atomic_store_explicit(&gofly_time_us, timestamp_us, memory_order_relaxed);
}

#if GOFLY_HOST_TEST
void gofly_time_set_for_test(uint32_t timestamp_us)
{
    gofly_time_set_now_us(timestamp_us);
}
#endif
