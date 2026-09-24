#include "w25q128.h"

#include <string.h>

static gofly_status_t gofly_w25q128_transfer(gofly_w25q128_t *device,
                                             const uint8_t *tx,
                                             uint8_t *rx,
                                             size_t length)
{
    if (device == NULL || device->spi.transfer == NULL || tx == NULL ||
        rx == NULL || length == 0U) {
        return GOFLY_E_ARGUMENT;
    }
    return device->spi.transfer(device->spi.context, tx, rx, length,
                                device->timeout_ms);
}

static bool gofly_w25q128_time_reached(uint32_t now, uint32_t target)
{
    return (int32_t)(now - target) >= 0;
}

static gofly_status_t gofly_w25q128_check_access(const gofly_w25q128_t *device,
                                                 uint32_t address,
                                                 size_t length)
{
    if (device == NULL || !device->initialized) {
        return GOFLY_E_STATE;
    }
    if (length > GOFLY_W25Q128_CAPACITY_BYTES ||
        address >= GOFLY_W25Q128_CAPACITY_BYTES ||
        length > (size_t)(GOFLY_W25Q128_CAPACITY_BYTES - address)) {
        return GOFLY_E_RANGE;
    }
    return GOFLY_OK;
}

static gofly_status_t gofly_w25q128_check_power_up(gofly_w25q128_t *device)
{
    uint32_t now;

    if (device == NULL || !device->initialized) {
        return GOFLY_E_STATE;
    }
    if (device->power_up_inhibit_ms == 0U) {
        return GOFLY_OK;
    }
    if (device->time_ms == NULL) {
        return GOFLY_E_NOT_READY;
    }
    now = device->time_ms(device->time_context);
    return gofly_w25q128_time_reached(now, device->power_up_ready_at_ms) ?
           GOFLY_OK : GOFLY_E_NOT_READY;
}

void gofly_w25q128_default_config(gofly_w25q128_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->timeout_ms = GOFLY_W25Q128_DEFAULT_TIMEOUT_MS;
    config->poll_interval_ms = GOFLY_W25Q128_DEFAULT_POLL_INTERVAL_MS;
    config->poll_limit = GOFLY_W25Q128_DEFAULT_POLL_LIMIT;
    config->power_up_inhibit_ms = GOFLY_W25Q128_POWER_UP_INHIBIT_MS;
}

gofly_status_t gofly_w25q128_init(gofly_w25q128_t *device,
                                  const gofly_w25q128_config_t *config)
{
    uint32_t now = 0U;

    if (device == NULL || config == NULL || config->spi.transfer == NULL ||
        config->timeout_ms == 0U || config->poll_interval_ms == 0U ||
        config->poll_limit == 0U ||
        (config->power_up_inhibit_ms != 0U && config->time_ms == NULL)) {
        return GOFLY_E_ARGUMENT;
    }
    memset(device, 0, sizeof(*device));
    device->spi = config->spi;
    device->delay_ms = config->delay_ms;
    device->delay_context = config->delay_context;
    device->time_ms = config->time_ms;
    device->time_context = config->time_context;
    device->timeout_ms = config->timeout_ms;
    device->poll_interval_ms = config->poll_interval_ms;
    device->poll_limit = config->poll_limit;
    device->power_up_inhibit_ms = config->power_up_inhibit_ms;
    if (device->time_ms != NULL) {
        now = device->time_ms(device->time_context);
    }
    device->power_up_ready_at_ms = now + device->power_up_inhibit_ms;
    device->initialized = true;
    return GOFLY_OK;
}

gofly_status_t gofly_w25q128_read_status(gofly_w25q128_t *device,
                                         uint8_t *status)
{
    uint8_t tx[2U] = {GOFLY_W25Q128_CMD_READ_STATUS1, 0U};
    uint8_t rx[2U] = {0U, 0U};
    gofly_status_t result;

    if (device == NULL || !device->initialized) {
        return GOFLY_E_STATE;
    }
    if (status == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    result = gofly_w25q128_check_power_up(device);
    if (result != GOFLY_OK) {
        return result;
    }
    result = gofly_w25q128_transfer(device, tx, rx, sizeof(tx));
    if (result != GOFLY_OK) {
        return result;
    }
    *status = rx[1U];
    return GOFLY_OK;
}

gofly_status_t gofly_w25q128_read_id(gofly_w25q128_t *device,
                                     gofly_w25q128_id_t *id)
{
    uint8_t tx[4U] = {GOFLY_W25Q128_CMD_READ_ID, 0U, 0U, 0U};
    uint8_t rx[4U] = {0U, 0U, 0U, 0U};
    gofly_status_t result;

    if (device == NULL || !device->initialized) {
        return GOFLY_E_STATE;
    }
    if (id == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    result = gofly_w25q128_check_power_up(device);
    if (result != GOFLY_OK) {
        return result;
    }
    result = gofly_w25q128_transfer(device, tx, rx, sizeof(tx));
    if (result != GOFLY_OK) {
        return result;
    }
    id->manufacturer_id = rx[1U];
    id->memory_type = rx[2U];
    id->capacity_code = rx[3U];
    return GOFLY_OK;
}

static gofly_status_t gofly_w25q128_write_enable(gofly_w25q128_t *device)
{
    uint8_t tx[1U] = {GOFLY_W25Q128_CMD_WRITE_ENABLE};
    uint8_t rx[1U] = {0U};
    uint8_t status;
    gofly_status_t result;

    result = gofly_w25q128_check_power_up(device);
    if (result != GOFLY_OK) {
        return result;
    }
    result = gofly_w25q128_transfer(device, tx, rx, sizeof(tx));
    if (result != GOFLY_OK) {
        return result;
    }
    result = gofly_w25q128_read_status(device, &status);
    if (result != GOFLY_OK) {
        return result;
    }
    return (status & GOFLY_W25Q128_STATUS_WEL) != 0U ?
           GOFLY_OK : GOFLY_E_STATE;
}

gofly_status_t gofly_w25q128_wait_ready(gofly_w25q128_t *device,
                                        uint32_t timeout_ms)
{
    uint8_t status;
    uint32_t attempt;
    uint32_t elapsed = 0U;
    gofly_status_t result;

    if (device == NULL || !device->initialized) {
        return GOFLY_E_STATE;
    }
    if (timeout_ms == 0U) {
        return GOFLY_E_ARGUMENT;
    }
    result = gofly_w25q128_check_power_up(device);
    if (result != GOFLY_OK) {
        return result;
    }
    for (attempt = 0U; attempt < device->poll_limit; ++attempt) {
        result = gofly_w25q128_read_status(device, &status);
        if (result != GOFLY_OK) {
            return result;
        }
        if ((status & GOFLY_W25Q128_STATUS_BUSY) == 0U) {
            return GOFLY_OK;
        }
        if (elapsed >= timeout_ms ||
            device->poll_interval_ms > timeout_ms - elapsed) {
            return GOFLY_E_TIMEOUT;
        }
        if (device->delay_ms == NULL) {
            return GOFLY_E_TIMEOUT;
        }
        result = device->delay_ms(device->delay_context,
                                  device->poll_interval_ms);
        if (result != GOFLY_OK) {
            return result;
        }
        elapsed += device->poll_interval_ms;
    }
    return GOFLY_E_TIMEOUT;
}

gofly_status_t gofly_w25q128_read(gofly_w25q128_t *device,
                                  uint32_t address,
                                  uint8_t *data,
                                  size_t length)
{
    size_t chunk;
    size_t index;
    gofly_status_t result;

    if (device == NULL || !device->initialized) {
        return GOFLY_E_STATE;
    }
    if (data == NULL || length == 0U) {
        return GOFLY_E_ARGUMENT;
    }
    result = gofly_w25q128_check_access(device, address, length);
    if (result != GOFLY_OK) {
        return result;
    }
    result = gofly_w25q128_wait_ready(device, device->timeout_ms);
    if (result != GOFLY_OK) {
        return result;
    }
    while (length > 0U) {
        chunk = length < GOFLY_W25Q128_READ_CHUNK_SIZE ?
                length : GOFLY_W25Q128_READ_CHUNK_SIZE;
        {
            uint8_t tx[GOFLY_W25Q128_READ_CHUNK_SIZE + 4U] = {0U};
            uint8_t rx[GOFLY_W25Q128_READ_CHUNK_SIZE + 4U] = {0U};
            tx[0U] = GOFLY_W25Q128_CMD_READ_DATA;
            tx[1U] = (uint8_t)(address >> 16U);
            tx[2U] = (uint8_t)(address >> 8U);
            tx[3U] = (uint8_t)address;
            result = gofly_w25q128_transfer(device, tx, rx, chunk + 4U);
            if (result != GOFLY_OK) {
                return result;
            }
            for (index = 0U; index < chunk; ++index) {
                data[index] = rx[index + 4U];
            }
        }
        address += (uint32_t)chunk;
        data += chunk;
        length -= chunk;
    }
    return GOFLY_OK;
}

gofly_status_t gofly_w25q128_page_program(gofly_w25q128_t *device,
                                          uint32_t address,
                                          const uint8_t *data,
                                          size_t length)
{
    size_t chunk;
    size_t page_remaining;
    gofly_status_t result;

    if (device == NULL || !device->initialized) {
        return GOFLY_E_STATE;
    }
    if (data == NULL || length == 0U) {
        return GOFLY_E_ARGUMENT;
    }
    result = gofly_w25q128_check_access(device, address, length);
    if (result != GOFLY_OK) {
        return result;
    }
    result = gofly_w25q128_wait_ready(device, device->timeout_ms);
    if (result != GOFLY_OK) {
        return result;
    }
    while (length > 0U) {
        page_remaining = GOFLY_W25Q128_PAGE_SIZE -
                         (address % GOFLY_W25Q128_PAGE_SIZE);
        chunk = length < page_remaining ? length : page_remaining;
        result = gofly_w25q128_write_enable(device);
        if (result != GOFLY_OK) {
            return result;
        }
        {
            uint8_t tx[GOFLY_W25Q128_PAGE_SIZE + 4U] = {0U};
            uint8_t rx[GOFLY_W25Q128_PAGE_SIZE + 4U] = {0U};
            tx[0U] = GOFLY_W25Q128_CMD_PAGE_PROGRAM;
            tx[1U] = (uint8_t)(address >> 16U);
            tx[2U] = (uint8_t)(address >> 8U);
            tx[3U] = (uint8_t)address;
            memcpy(&tx[4U], data, chunk);
            result = gofly_w25q128_transfer(device, tx, rx, chunk + 4U);
        }
        if (result != GOFLY_OK) {
            return result;
        }
        result = gofly_w25q128_wait_ready(device, device->timeout_ms);
        if (result != GOFLY_OK) {
            return result;
        }
        address += (uint32_t)chunk;
        data += chunk;
        length -= chunk;
    }
    return GOFLY_OK;
}

gofly_status_t gofly_w25q128_sector_erase(gofly_w25q128_t *device,
                                          uint32_t address)
{
    uint8_t tx[4U];
    uint8_t rx[4U] = {0U, 0U, 0U, 0U};
    gofly_status_t result;

    if (device == NULL || !device->initialized) {
        return GOFLY_E_STATE;
    }
    if ((address % GOFLY_W25Q128_SECTOR_SIZE) != 0U) {
        return GOFLY_E_RANGE;
    }
    result = gofly_w25q128_check_access(device, address,
                                        GOFLY_W25Q128_SECTOR_SIZE);
    if (result != GOFLY_OK) {
        return result;
    }
    result = gofly_w25q128_wait_ready(device, device->timeout_ms);
    if (result != GOFLY_OK) {
        return result;
    }
    result = gofly_w25q128_write_enable(device);
    if (result != GOFLY_OK) {
        return result;
    }
    tx[0U] = GOFLY_W25Q128_CMD_SECTOR_ERASE_4K;
    tx[1U] = (uint8_t)(address >> 16U);
    tx[2U] = (uint8_t)(address >> 8U);
    tx[3U] = (uint8_t)address;
    result = gofly_w25q128_transfer(device, tx, rx, sizeof(tx));
    if (result != GOFLY_OK) {
        return result;
    }
    return gofly_w25q128_wait_ready(device, device->timeout_ms);
}

bool gofly_w25q128_id_is_w25q128(const gofly_w25q128_id_t *id)
{
    return id != NULL && id->manufacturer_id == 0xEFU &&
           id->memory_type == 0x40U && id->capacity_code == 0x18U;
}
