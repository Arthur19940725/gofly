#include "max7456.h"

#include <string.h>

#define GOFLY_MAX7456_SPI_WRITE_MASK 0x80U
#define GOFLY_MAX7456_SPI_READ_MASK 0x00U

static gofly_status_t gofly_max7456_transfer(gofly_max7456_t *device,
                                             const uint8_t *tx,
                                             uint8_t *rx)
{
    if (device == NULL || device->spi.transfer == NULL || tx == NULL ||
        rx == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    return device->spi.transfer(device->spi.context, tx, rx,
                                GOFLY_MAX7456_SPI_TRANSACTION_LENGTH,
                                device->timeout_ms);
}

void gofly_max7456_default_config(gofly_max7456_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->timeout_ms = 10U;
}

gofly_status_t gofly_max7456_reset(gofly_max7456_t *device)
{
    gofly_status_t status;

    if (device == NULL || device->reset_pin_write == NULL ||
        device->delay_ms == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    device->reset_complete = false;
    status = device->reset_pin_write(device->reset_context, false);
    if (status != GOFLY_OK) {
        return status;
    }
    status = device->delay_ms(device->delay_context,
                              GOFLY_MAX7456_RESET_MIN_MS);
    if (status != GOFLY_OK) {
        return status;
    }
    status = device->reset_pin_write(device->reset_context, true);
    if (status != GOFLY_OK) {
        return status;
    }
    status = device->delay_ms(device->delay_context,
                              GOFLY_MAX7456_POST_RESET_MS);
    if (status != GOFLY_OK) {
        return status;
    }
    device->reset_complete = true;
    return GOFLY_OK;
}

gofly_status_t gofly_max7456_init(gofly_max7456_t *device,
                                  const gofly_max7456_config_t *config)
{
    gofly_status_t status;

    if (device == NULL || config == NULL || config->spi.transfer == NULL ||
        config->reset_pin_write == NULL || config->delay_ms == NULL ||
        config->timeout_ms == 0U) {
        return GOFLY_E_ARGUMENT;
    }
    memset(device, 0, sizeof(*device));
    device->spi = config->spi;
    device->reset_pin_write = config->reset_pin_write;
    device->reset_context = config->reset_context;
    device->delay_ms = config->delay_ms;
    device->delay_context = config->delay_context;
    device->timeout_ms = config->timeout_ms;
    device->device_kind = GOFLY_DEVICE_ANALOG_CVBS;
    status = gofly_max7456_reset(device);
    if (status != GOFLY_OK) {
        device->initialized = false;
        device->reset_complete = false;
        return status;
    }
    device->initialized = true;
    return GOFLY_OK;
}

gofly_status_t gofly_max7456_read_register(gofly_max7456_t *device,
                                           uint8_t reg,
                                           uint8_t *value)
{
    uint8_t tx[GOFLY_MAX7456_SPI_TRANSACTION_LENGTH] = {
        (uint8_t)((reg & 0x7FU) | GOFLY_MAX7456_SPI_READ_MASK), 0U};
    uint8_t rx[GOFLY_MAX7456_SPI_TRANSACTION_LENGTH] = {0U, 0U};
    gofly_status_t status;

    if (device == NULL || !device->initialized || !device->reset_complete) {
        return GOFLY_E_STATE;
    }
    if (value == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    status = gofly_max7456_transfer(device, tx, rx);
    if (status != GOFLY_OK) {
        return status;
    }
    *value = rx[1U];
    return GOFLY_OK;
}

gofly_status_t gofly_max7456_write_register(gofly_max7456_t *device,
                                            uint8_t reg,
                                            uint8_t value)
{
    uint8_t tx[GOFLY_MAX7456_SPI_TRANSACTION_LENGTH] = {
        (uint8_t)((reg & 0x7FU) | GOFLY_MAX7456_SPI_WRITE_MASK), value};
    uint8_t rx[GOFLY_MAX7456_SPI_TRANSACTION_LENGTH] = {0U, 0U};

    if (device == NULL || !device->initialized || !device->reset_complete) {
        return GOFLY_E_STATE;
    }
    return gofly_max7456_transfer(device, tx, rx);
}

gofly_status_t gofly_max7456_configure_video_standard(
    gofly_max7456_t *device, gofly_max7456_video_standard_t standard)
{
    uint8_t value;

    if (standard != GOFLY_MAX7456_VIDEO_NTSC &&
        standard != GOFLY_MAX7456_VIDEO_PAL) {
        return GOFLY_E_ARGUMENT;
    }
    value = standard == GOFLY_MAX7456_VIDEO_PAL ?
            GOFLY_MAX7456_VM0_PAL : GOFLY_MAX7456_VM0_NTSC;
    return gofly_max7456_write_register(device, GOFLY_MAX7456_REG_VM0, value);
}
