#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../../Drivers/Max7456/max7456.h"

typedef struct {
    uint8_t last_tx[GOFLY_MAX7456_SPI_TRANSACTION_LENGTH];
    uint8_t response;
    size_t calls;
    uint32_t timeout_ms;
    bool pin_level;
    uint32_t delays[4U];
    size_t delay_count;
} gofly_max7456_mock_t;

static void gofly_max7456_test_check(bool condition, unsigned *failures)
{
    if (!condition) {
        ++(*failures);
    }
}

static gofly_status_t gofly_max7456_mock_transfer(void *context,
                                                  const uint8_t *tx,
                                                  uint8_t *rx,
                                                  size_t length,
                                                  uint32_t timeout_ms)
{
    gofly_max7456_mock_t *mock = (gofly_max7456_mock_t *)context;

    if (length != GOFLY_MAX7456_SPI_TRANSACTION_LENGTH) {
        return GOFLY_E_RANGE;
    }
    mock->last_tx[0U] = tx[0U];
    mock->last_tx[1U] = tx[1U];
    mock->timeout_ms = timeout_ms;
    ++mock->calls;
    rx[0U] = 0U;
    rx[1U] = mock->response;
    return GOFLY_OK;
}

static gofly_status_t gofly_max7456_mock_pin(void *context, bool level)
{
    ((gofly_max7456_mock_t *)context)->pin_level = level;
    return GOFLY_OK;
}

static gofly_status_t gofly_max7456_mock_delay(void *context, uint32_t delay_ms)
{
    gofly_max7456_mock_t *mock = (gofly_max7456_mock_t *)context;
    if (mock->delay_count >= 4U) {
        return GOFLY_E_RANGE;
    }
    mock->delays[mock->delay_count++] = delay_ms;
    return GOFLY_OK;
}

int gofly_test_max7456_run(void)
{
    unsigned failures = 0U;
    gofly_max7456_mock_t mock = {0};
    gofly_max7456_config_t config;
    gofly_max7456_t device;
    uint8_t value = 0U;

    gofly_max7456_default_config(&config);
    config.spi.context = &mock;
    config.spi.transfer = gofly_max7456_mock_transfer;
    config.reset_pin_write = gofly_max7456_mock_pin;
    config.reset_context = &mock;
    config.delay_ms = gofly_max7456_mock_delay;
    config.delay_context = &mock;
    gofly_max7456_test_check(gofly_max7456_init(&device, &config) == GOFLY_OK,
                             &failures);
    gofly_max7456_test_check(device.device_kind == GOFLY_DEVICE_ANALOG_CVBS,
                             &failures);
    gofly_max7456_test_check(mock.delays[0U] == GOFLY_MAX7456_RESET_MIN_MS &&
                             mock.delays[1U] == GOFLY_MAX7456_POST_RESET_MS,
                             &failures);
    gofly_max7456_test_check(mock.pin_level, &failures);
    gofly_max7456_test_check(gofly_max7456_write_register(&device, 0x10U,
                                                          0xA5U) == GOFLY_OK &&
                             mock.last_tx[0U] == 0x90U &&
                             mock.last_tx[1U] == 0xA5U,
                             &failures);
    mock.response = 0x5AU;
    gofly_max7456_test_check(gofly_max7456_read_register(&device, 0x10U,
                                                         &value) == GOFLY_OK &&
                             value == 0x5AU && mock.last_tx[0U] == 0x10U,
                             &failures);
    gofly_max7456_test_check(gofly_max7456_configure_video_standard(
                                 &device, GOFLY_MAX7456_VIDEO_PAL) == GOFLY_OK &&
                             mock.last_tx[0U] ==
                                 (GOFLY_MAX7456_REG_VM0 | 0x80U) &&
                             mock.last_tx[1U] == GOFLY_MAX7456_VM0_PAL,
                             &failures);
    gofly_max7456_test_check(gofly_max7456_configure_video_standard(
                                 &device, (gofly_max7456_video_standard_t)2) ==
                             GOFLY_E_ARGUMENT, &failures);
    return (int)failures;
}
