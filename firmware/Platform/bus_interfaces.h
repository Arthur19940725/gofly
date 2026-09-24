#ifndef GOFLY_PLATFORM_BUS_INTERFACES_H
#define GOFLY_PLATFORM_BUS_INTERFACES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "status.h"

/* A successful transfer means that exactly length bytes were exchanged. */
typedef gofly_status_t (*gofly_spi_transfer_fn)(void *context,
                                                 const uint8_t *tx,
                                                 uint8_t *rx,
                                                 size_t length,
                                                 uint32_t timeout_ms);
typedef gofly_status_t (*gofly_spi_configure_fn)(void *context,
                                                  uint8_t mode,
                                                  uint32_t max_hz);

typedef gofly_status_t (*gofly_i2c_mem_read_fn)(void *context,
                                                uint16_t address,
                                                uint16_t reg,
                                                uint8_t *data,
                                                size_t length,
                                                uint32_t timeout_ms);

typedef gofly_status_t (*gofly_i2c_mem_write_fn)(void *context,
                                                 uint16_t address,
                                                 uint16_t reg,
                                                 const uint8_t *data,
                                                 size_t length,
                                                 uint32_t timeout_ms);

/* Optional control callbacks are deliberately transport-neutral. */
typedef gofly_status_t (*gofly_pin_write_fn)(void *context, bool level);
typedef gofly_status_t (*gofly_delay_ms_fn)(void *context, uint32_t delay_ms);
typedef uint32_t (*gofly_time_ms_fn)(void *context);
typedef uint32_t (*gofly_time_us_fn)(void *context);
typedef bool (*gofly_data_ready_fn)(void *context);

typedef struct {
    void *context;
    gofly_spi_transfer_fn transfer;
    gofly_spi_configure_fn configure;
} gofly_spi_bus_t;

typedef struct {
    void *context;
    gofly_i2c_mem_read_fn read;
    gofly_i2c_mem_write_fn write;
} gofly_i2c_bus_t;

#ifdef __cplusplus
}
#endif

#endif
