#ifndef GOFLY_DRIVER_W25Q128_H
#define GOFLY_DRIVER_W25Q128_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bus_interfaces.h"
#include "status.h"

#define GOFLY_W25Q128_CAPACITY_BYTES 0x01000000UL
#define GOFLY_W25Q128_PAGE_SIZE 256U
#define GOFLY_W25Q128_SECTOR_SIZE 4096U
#define GOFLY_W25Q128_POWER_UP_INHIBIT_MS 20U
#define GOFLY_W25Q128_DEFAULT_TIMEOUT_MS 500U
#define GOFLY_W25Q128_DEFAULT_POLL_INTERVAL_MS 1U
#define GOFLY_W25Q128_DEFAULT_POLL_LIMIT 1024U
#define GOFLY_W25Q128_CMD_WRITE_ENABLE 0x06U
#define GOFLY_W25Q128_CMD_READ_STATUS1 0x05U
#define GOFLY_W25Q128_CMD_READ_DATA 0x03U
#define GOFLY_W25Q128_CMD_PAGE_PROGRAM 0x02U
#define GOFLY_W25Q128_CMD_SECTOR_ERASE_4K 0x20U
#define GOFLY_W25Q128_CMD_READ_ID 0x9FU
#define GOFLY_W25Q128_READ_CHUNK_SIZE 256U
#define GOFLY_W25Q128_STATUS_BUSY 0x01U
#define GOFLY_W25Q128_STATUS_WEL 0x02U

typedef struct {
    gofly_spi_bus_t spi;
    gofly_delay_ms_fn delay_ms;
    void *delay_context;
    gofly_time_ms_fn time_ms;
    void *time_context;
    uint32_t timeout_ms;
    uint32_t poll_interval_ms;
    uint32_t poll_limit;
    uint32_t power_up_inhibit_ms;
} gofly_w25q128_config_t;

typedef struct {
    uint8_t manufacturer_id;
    uint8_t memory_type;
    uint8_t capacity_code;
} gofly_w25q128_id_t;

typedef struct {
    gofly_spi_bus_t spi;
    gofly_delay_ms_fn delay_ms;
    void *delay_context;
    gofly_time_ms_fn time_ms;
    void *time_context;
    uint32_t timeout_ms;
    uint32_t poll_interval_ms;
    uint32_t poll_limit;
    uint32_t power_up_inhibit_ms;
    uint32_t power_up_ready_at_ms;
    bool initialized;
} gofly_w25q128_t;

void gofly_w25q128_default_config(gofly_w25q128_config_t *config);
gofly_status_t gofly_w25q128_init(gofly_w25q128_t *device,
                                  const gofly_w25q128_config_t *config);
gofly_status_t gofly_w25q128_read_id(gofly_w25q128_t *device,
                                     gofly_w25q128_id_t *id);
gofly_status_t gofly_w25q128_read_status(gofly_w25q128_t *device,
                                         uint8_t *status);
gofly_status_t gofly_w25q128_wait_ready(gofly_w25q128_t *device,
                                        uint32_t timeout_ms);
gofly_status_t gofly_w25q128_read(gofly_w25q128_t *device,
                                  uint32_t address,
                                  uint8_t *data,
                                  size_t length);
gofly_status_t gofly_w25q128_page_program(gofly_w25q128_t *device,
                                          uint32_t address,
                                          const uint8_t *data,
                                          size_t length);
gofly_status_t gofly_w25q128_sector_erase(gofly_w25q128_t *device,
                                          uint32_t address);
bool gofly_w25q128_id_is_w25q128(const gofly_w25q128_id_t *id);

#ifdef __cplusplus
}
#endif

#endif
