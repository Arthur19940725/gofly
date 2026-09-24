#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../../Drivers/W25q128/w25q128.h"

typedef struct {
    uint8_t status_values[16U];
    size_t status_count;
    size_t status_index;
    uint8_t id[3U];
    uint8_t read_values[8U];
    size_t read_index;
    uint8_t commands[32U];
    size_t command_count;
    size_t lengths[32U];
    uint32_t addresses[32U];
    uint32_t now_ms;
    size_t delay_calls;
    size_t page_program_count;
    size_t sector_erase_count;
    bool busy_forever;
    gofly_status_t forced_status;
} gofly_w25q128_mock_t;

static void gofly_w25q128_test_check(bool condition, unsigned *failures)
{
    if (!condition) {
        ++(*failures);
    }
}

static gofly_status_t gofly_w25q128_mock_transfer(void *context,
                                                  const uint8_t *tx,
                                                  uint8_t *rx,
                                                  size_t length,
                                                  uint32_t timeout_ms)
{
    gofly_w25q128_mock_t *mock = (gofly_w25q128_mock_t *)context;
    size_t index;
    (void)timeout_ms;
    if (mock->forced_status != GOFLY_OK) {
        return mock->forced_status;
    }
    if (mock->command_count < 32U) {
        mock->commands[mock->command_count] = tx[0U];
        mock->lengths[mock->command_count] = length;
        mock->addresses[mock->command_count] = length >= 4U ?
            ((uint32_t)tx[1U] << 16U) | ((uint32_t)tx[2U] << 8U) | tx[3U] : 0U;
        ++mock->command_count;
    }
    memset(rx, 0, length);
    if (tx[0U] == GOFLY_W25Q128_CMD_READ_STATUS1 && length == 2U) {
        rx[1U] = mock->busy_forever ? GOFLY_W25Q128_STATUS_BUSY :
                 mock->status_index < mock->status_count ?
                 mock->status_values[mock->status_index++] : 0U;
    } else if (tx[0U] == GOFLY_W25Q128_CMD_READ_ID && length == 4U) {
        rx[1U] = mock->id[0U];
        rx[2U] = mock->id[1U];
        rx[3U] = mock->id[2U];
    } else if (tx[0U] == GOFLY_W25Q128_CMD_READ_DATA &&
               length >= 4U) {
        for (index = 4U; index < length; ++index) {
            rx[index] = mock->read_index < sizeof(mock->read_values) ?
                        mock->read_values[mock->read_index++] : 0U;
        }
    } else {
        for (index = 0U; index < length; ++index) {
            (void)tx[index];
        }
        if (tx[0U] == GOFLY_W25Q128_CMD_PAGE_PROGRAM) {
            ++mock->page_program_count;
        } else if (tx[0U] == GOFLY_W25Q128_CMD_SECTOR_ERASE_4K) {
            ++mock->sector_erase_count;
        }
    }
    return GOFLY_OK;
}

static gofly_status_t gofly_w25q128_mock_delay(void *context, uint32_t delay_ms)
{
    gofly_w25q128_mock_t *mock = (gofly_w25q128_mock_t *)context;
    mock->now_ms += delay_ms;
    ++mock->delay_calls;
    return GOFLY_OK;
}

static uint32_t gofly_w25q128_mock_time(void *context)
{
    return ((gofly_w25q128_mock_t *)context)->now_ms;
}

int gofly_test_w25q128_run(void)
{
    unsigned failures = 0U;
    gofly_w25q128_mock_t mock = {0};
    gofly_w25q128_config_t config;
    gofly_w25q128_t device;
    gofly_w25q128_id_t id = {0};
    uint8_t data[4U] = {0U};
    uint8_t program[300U] = {0U};
    size_t index;

    mock.id[0U] = 0xEFU;
    mock.id[1U] = 0x40U;
    mock.id[2U] = 0x18U;
    /* The read and program checks reset status_index below. The erase check
     * continues from that sequence: read consumes ready; programming consumes
     * ready plus (WEL, ready) per page chunk; erase consumes ready, WEL,
     * ready. */
    mock.status_values[0U] = 0U;
    mock.status_values[1U] = GOFLY_W25Q128_STATUS_WEL;
    mock.status_values[2U] = 0U;
    mock.status_values[3U] = GOFLY_W25Q128_STATUS_WEL;
    mock.status_values[4U] = 0U;
    mock.status_values[5U] = GOFLY_W25Q128_STATUS_WEL;
    mock.status_values[6U] = 0U;
    mock.status_values[7U] = 0U;
    mock.status_values[8U] = GOFLY_W25Q128_STATUS_WEL;
    mock.status_values[9U] = 0U;
    mock.status_count = 10U;
    for (index = 0U; index < sizeof(data); ++index) {
        mock.read_values[index] = (uint8_t)(index + 1U);
    }
    for (index = 0U; index < sizeof(program); ++index) {
        program[index] = (uint8_t)index;
    }

    gofly_w25q128_default_config(&config);
    config.spi.context = &mock;
    config.spi.transfer = gofly_w25q128_mock_transfer;
    config.delay_ms = gofly_w25q128_mock_delay;
    config.delay_context = &mock;
    config.time_ms = gofly_w25q128_mock_time;
    config.time_context = &mock;
    config.power_up_inhibit_ms = 20U;
    gofly_w25q128_test_check(gofly_w25q128_init(&device, &config) == GOFLY_OK,
                             &failures);
    gofly_w25q128_test_check(gofly_w25q128_read_id(&device, &id) ==
                             GOFLY_E_NOT_READY, &failures);
    mock.now_ms = 20U;
    gofly_w25q128_test_check(gofly_w25q128_read_id(&device, &id) == GOFLY_OK &&
                             gofly_w25q128_id_is_w25q128(&id), &failures);
    mock.status_index = 0U;
    gofly_w25q128_test_check(gofly_w25q128_read(&device, 0x0100U, data,
                                                sizeof(data)) == GOFLY_OK &&
                             data[0U] == 1U && data[3U] == 4U, &failures);
    gofly_w25q128_test_check(gofly_w25q128_read(
                                 &device, GOFLY_W25Q128_CAPACITY_BYTES - 1U,
                                 data, 2U) == GOFLY_E_RANGE, &failures);
    mock.status_index = 0U;
    gofly_w25q128_test_check(gofly_w25q128_page_program(&device, 0x00F0U,
                                                         program, sizeof(program)) ==
                             GOFLY_OK && mock.page_program_count == 3U,
                             &failures);
    gofly_w25q128_test_check(gofly_w25q128_sector_erase(&device, 1U) ==
                             GOFLY_E_RANGE, &failures);
    gofly_w25q128_test_check(gofly_w25q128_sector_erase(
                                 &device, GOFLY_W25Q128_CAPACITY_BYTES) ==
                             GOFLY_E_RANGE, &failures);
    mock.busy_forever = true;
    gofly_w25q128_test_check(gofly_w25q128_wait_ready(&device, 2U) ==
                             GOFLY_E_TIMEOUT, &failures);
    mock.busy_forever = false;
    gofly_w25q128_test_check(gofly_w25q128_sector_erase(
                                 &device, GOFLY_W25Q128_SECTOR_SIZE) ==
                             GOFLY_OK && mock.sector_erase_count == 1U,
                             &failures);
    mock.forced_status = GOFLY_E_TIMEOUT;
    gofly_w25q128_test_check(gofly_w25q128_read_status(&device, &data[0U]) ==
                             GOFLY_E_TIMEOUT, &failures);
    return (int)failures;
}
