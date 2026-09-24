#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../../Drivers/Icm42688/icm42688.h"

typedef struct {
    uint8_t registers[256U];
    uint8_t sample[GOFLY_ICM42688_SAMPLE_BURST_LENGTH];
    size_t transfer_count;
    size_t sample_transfers;
    uint8_t last_tx[GOFLY_ICM42688_SAMPLE_BURST_LENGTH + 1U];
    size_t last_length;
    uint8_t configured_mode;
    uint32_t configured_hz;
    uint8_t int_config;
    gofly_status_t forced_status;
    bool ready;
    bool short_sample;
} gofly_icm42688_mock_t;

static void gofly_icm_test_check(bool condition, unsigned *failures)
{
    if (!condition) {
        ++(*failures);
    }
}

static gofly_status_t gofly_icm_mock_configure(void *context, uint8_t mode,
                                               uint32_t max_hz)
{
    gofly_icm42688_mock_t *mock = (gofly_icm42688_mock_t *)context;
    mock->configured_mode = mode;
    mock->configured_hz = max_hz;
    return mock->forced_status;
}

static gofly_status_t gofly_icm_mock_transfer(void *context, const uint8_t *tx,
                                              uint8_t *rx, size_t length,
                                              uint32_t timeout_ms)
{
    gofly_icm42688_mock_t *mock = (gofly_icm42688_mock_t *)context;
    size_t index;
    (void)timeout_ms;
    if (mock->forced_status != GOFLY_OK) {
        return mock->forced_status;
    }
    ++mock->transfer_count;
    mock->last_length = length;
    if (length > sizeof(mock->last_tx)) {
        return GOFLY_E_RANGE;
    }
    memcpy(mock->last_tx, tx, length);
    memset(rx, 0, length);
    if (length == 2U) {
        if ((tx[0U] & 0x80U) != 0U) {
            rx[1U] = mock->registers[tx[0U] & 0x7FU];
        } else {
            mock->registers[tx[0U] & 0x7FU] = tx[1U];
            if ((tx[0U] & 0x7FU) == GOFLY_ICM42688_INT_CONFIG_REGISTER) {
                mock->int_config = tx[1U];
            }
        }
    } else if (length == GOFLY_ICM42688_SAMPLE_BURST_LENGTH + 1U) {
        if (mock->short_sample) {
            return GOFLY_E_RANGE;
        }
        ++mock->sample_transfers;
        for (index = 0U; index < sizeof(mock->sample); ++index) {
            rx[index + 1U] = mock->sample[index];
        }
    }
    return GOFLY_OK;
}

static bool gofly_icm_mock_ready(void *context)
{
    return ((gofly_icm42688_mock_t *)context)->ready;
}

static void gofly_icm_sample_set_i16(uint8_t *data, size_t index, int16_t value)
{
    data[index] = (uint8_t)((uint16_t)value >> 8U);
    data[index + 1U] = (uint8_t)value;
}

int gofly_test_icm42688_run(void)
{
    unsigned failures = 0U;
    gofly_icm42688_mock_t mock = {0};
    gofly_icm42688_config_t config;
    gofly_icm42688_t device;
    gofly_imu_sample_t sample = {0};

    mock.registers[GOFLY_ICM42688_WHO_AM_I_REGISTER] =
        GOFLY_ICM42688_WHO_AM_I_VALUE;
    mock.ready = true;
    gofly_icm_sample_set_i16(&mock.sample[0U], 0U, 16384);
    gofly_icm_sample_set_i16(&mock.sample[2U], 0U, -16384);
    gofly_icm_sample_set_i16(&mock.sample[4U], 0U, 8192);
    gofly_icm_sample_set_i16(&mock.sample[6U], 0U, 1000);
    gofly_icm_sample_set_i16(&mock.sample[8U], 0U, -1000);
    gofly_icm_sample_set_i16(&mock.sample[10U], 0U, 500);

    gofly_icm42688_default_config(&config);
    config.spi.context = &mock;
    config.spi.transfer = gofly_icm_mock_transfer;
    config.spi.configure = gofly_icm_mock_configure;
    config.data_ready = gofly_icm_mock_ready;
    config.data_ready_context = &mock;
    gofly_icm_test_check(gofly_icm42688_init(&device, &config) == GOFLY_OK,
                         &failures);
    gofly_icm_test_check(mock.configured_mode == 3U &&
                         mock.configured_hz == 24000000UL &&
                         mock.int_config == 0x03U, &failures);
    gofly_icm_test_check(gofly_icm42688_read_sample_at(&device, &sample,
                                                       1000U) == GOFLY_OK &&
                         sample.valid && sample.timestamp_us == 1000U &&
                         sample.accel_mps2[0U] > 78.4f &&
                         sample.accel_mps2[0U] < 78.5f &&
                         sample.gyro_rad_s[0U] > 1.06f &&
                         sample.gyro_rad_s[0U] < 1.07f, &failures);
    gofly_icm_test_check(gofly_icm42688_read_sample_at(&device, &sample,
                                                       999U) ==
                         GOFLY_E_NOT_READY, &failures);
    gofly_icm_test_check(gofly_icm42688_read_sample_at(&device, &sample,
                                                       1000U) ==
                         GOFLY_E_NOT_READY, &failures);
    mock.ready = false;
    gofly_icm_test_check(gofly_icm42688_read_sample_at(&device, &sample,
                                                       2000U) ==
                         GOFLY_E_TIMEOUT, &failures);
    mock.ready = true;
    mock.forced_status = GOFLY_E_TIMEOUT;
    gofly_icm_test_check(gofly_icm42688_read_sample_at(&device, &sample,
                                                       2000U) ==
                         GOFLY_E_TIMEOUT, &failures);

    mock.forced_status = GOFLY_OK;
    mock.short_sample = true;
    gofly_icm_test_check(gofly_icm42688_read_sample_at(&device, &sample,
                                                       3000U) ==
                         GOFLY_E_RANGE && !sample.valid, &failures);
    mock.short_sample = false;
    mock.registers[GOFLY_ICM42688_WHO_AM_I_REGISTER] = 0U;
    gofly_icm_test_check(gofly_icm42688_init(&device, &config) == GOFLY_E_STATE,
                         &failures);
    return (int)failures;
}
