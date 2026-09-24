#include "icm42688.h"

#include <stddef.h>
#include <string.h>

#define GOFLY_ICM42688_SPI_READ 0x80U
#define GOFLY_ICM42688_SPI_MODE 3U
#define GOFLY_ICM42688_SPI_MAX_HZ 24000000UL
#define GOFLY_ICM42688_DEVICE_CONFIG_SPI_4WIRE 0x00U
#define GOFLY_ICM42688_PWR_MGMT0_LOW_NOISE 0x0FU
/* INT_CONFIG: bit 0 selects INT1 polarity; bit 1 selects INT1 drive circuit. */
#define GOFLY_ICM42688_INT_CONFIG_ACTIVE_HIGH 0x01U
#define GOFLY_ICM42688_INT_CONFIG_PUSH_PULL 0x02U
#define GOFLY_ICM42688_INT_SOURCE0_DATA_READY 0x08U
#define GOFLY_ICM42688_ACCEL_DATA_START 0x1FU
#define GOFLY_ICM42688_ACCEL_SENSITIVITY_16G_LSB_PER_G 2048.0f
#define GOFLY_ICM42688_ACCEL_SENSITIVITY_8G_LSB_PER_G 4096.0f
#define GOFLY_ICM42688_ACCEL_SENSITIVITY_4G_LSB_PER_G 8192.0f
#define GOFLY_ICM42688_ACCEL_SENSITIVITY_2G_LSB_PER_G 16384.0f
#define GOFLY_ICM42688_GYRO_SENSITIVITY_2000DPS_LSB_PER_DPS 16.4f
#define GOFLY_ICM42688_GYRO_SENSITIVITY_1000DPS_LSB_PER_DPS 32.8f
#define GOFLY_ICM42688_GYRO_SENSITIVITY_500DPS_LSB_PER_DPS 65.5f
#define GOFLY_ICM42688_GYRO_SENSITIVITY_250DPS_LSB_PER_DPS 131.0f
#define GOFLY_ICM42688_GYRO_SENSITIVITY_125DPS_LSB_PER_DPS 262.0f
#define GOFLY_ICM42688_GRAVITY_MPS2 9.80665f
#define GOFLY_ICM42688_PI 3.14159265358979323846f

static bool gofly_icm42688_valid_accel_fs(uint8_t value)
{
    return value == (uint8_t)GOFLY_ICM42688_ACCEL_FS_16G ||
           value == (uint8_t)GOFLY_ICM42688_ACCEL_FS_8G ||
           value == (uint8_t)GOFLY_ICM42688_ACCEL_FS_4G ||
           value == (uint8_t)GOFLY_ICM42688_ACCEL_FS_2G;
}

static bool gofly_icm42688_valid_gyro_fs(uint16_t value)
{
    return value == (uint16_t)GOFLY_ICM42688_GYRO_FS_2000DPS ||
           value == (uint16_t)GOFLY_ICM42688_GYRO_FS_1000DPS ||
           value == (uint16_t)GOFLY_ICM42688_GYRO_FS_500DPS ||
           value == (uint16_t)GOFLY_ICM42688_GYRO_FS_250DPS ||
           value == (uint16_t)GOFLY_ICM42688_GYRO_FS_125DPS;
}

static uint8_t gofly_icm42688_accel_fs_bits(uint8_t full_scale_g)
{
    switch (full_scale_g) {
    case 16U:
        return 0U;
    case 8U:
        return 1U;
    case 4U:
        return 2U;
    default:
        return 3U;
    }
}

static uint8_t gofly_icm42688_gyro_fs_bits(uint16_t full_scale_dps)
{
    switch (full_scale_dps) {
    case 2000U:
        return 0U;
    case 1000U:
        return 1U;
    case 500U:
        return 2U;
    case 250U:
        return 3U;
    default:
        return 4U;
    }
}

static int16_t gofly_icm42688_i16(const uint8_t *bytes)
{
    const uint16_t raw = (uint16_t)((uint16_t)bytes[0U] << 8U) |
                         (uint16_t)bytes[1U];
    return (int16_t)raw;
}

static float gofly_icm42688_accel_sensitivity(uint8_t full_scale_g)
{
    switch (full_scale_g) {
    case 16U:
        return GOFLY_ICM42688_ACCEL_SENSITIVITY_16G_LSB_PER_G;
    case 8U:
        return GOFLY_ICM42688_ACCEL_SENSITIVITY_8G_LSB_PER_G;
    case 4U:
        return GOFLY_ICM42688_ACCEL_SENSITIVITY_4G_LSB_PER_G;
    default:
        return GOFLY_ICM42688_ACCEL_SENSITIVITY_2G_LSB_PER_G;
    }
}

static float gofly_icm42688_gyro_sensitivity(uint16_t full_scale_dps)
{
    switch (full_scale_dps) {
    case 2000U:
        return GOFLY_ICM42688_GYRO_SENSITIVITY_2000DPS_LSB_PER_DPS;
    case 1000U:
        return GOFLY_ICM42688_GYRO_SENSITIVITY_1000DPS_LSB_PER_DPS;
    case 500U:
        return GOFLY_ICM42688_GYRO_SENSITIVITY_500DPS_LSB_PER_DPS;
    case 250U:
        return GOFLY_ICM42688_GYRO_SENSITIVITY_250DPS_LSB_PER_DPS;
    default:
        return GOFLY_ICM42688_GYRO_SENSITIVITY_125DPS_LSB_PER_DPS;
    }
}


static gofly_status_t gofly_icm42688_transfer(gofly_icm42688_t *device,
                                               const uint8_t *tx,
                                               uint8_t *rx,
                                               size_t length)
{
    if (device == NULL || device->spi.transfer == NULL ||
        tx == NULL || rx == NULL || length == 0U) {
        return GOFLY_E_ARGUMENT;
    }
    return device->spi.transfer(device->spi.context, tx, rx, length,
                                device->timeout_ms);
}

void gofly_icm42688_default_config(gofly_icm42688_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->timeout_ms = GOFLY_ICM42688_DEFAULT_TIMEOUT_MS;
    config->sample_timeout_us = GOFLY_ICM42688_DEFAULT_SAMPLE_TIMEOUT_US;
    config->accel_full_scale_g = (uint8_t)GOFLY_ICM42688_ACCEL_FS_16G;
    config->gyro_full_scale_dps = (uint16_t)GOFLY_ICM42688_GYRO_FS_2000DPS;
    config->accel_odr = GOFLY_ICM42688_ODR_1KHZ;
    config->gyro_odr = GOFLY_ICM42688_ODR_1KHZ;
    config->accel_filter = 3U;
    config->gyro_filter = 3U;
    config->int1_active_high = true;
    config->int1_push_pull = true;
}

gofly_status_t gofly_icm42688_read_register(gofly_icm42688_t *device,
                                            uint8_t reg,
                                            uint8_t *value)
{
    uint8_t tx[2U] = {(uint8_t)(reg | GOFLY_ICM42688_SPI_READ), 0U};
    uint8_t rx[2U] = {0U, 0U};
    gofly_status_t status;

    if (device == NULL || value == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    status = gofly_icm42688_transfer(device, tx, rx, sizeof(tx));
    if (status != GOFLY_OK) {
        return status;
    }
    *value = rx[1U];
    return GOFLY_OK;
}

gofly_status_t gofly_icm42688_write_register(gofly_icm42688_t *device,
                                             uint8_t reg,
                                             uint8_t value)
{
    uint8_t tx[2U] = {(uint8_t)(reg & 0x7FU), value};
    uint8_t rx[2U] = {0U, 0U};

    if (device == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    return gofly_icm42688_transfer(device, tx, rx, sizeof(tx));
}

static gofly_status_t gofly_icm42688_configure(gofly_icm42688_t *device,
                                                const gofly_icm42688_config_t *config)
{
    const uint8_t int_config =
        (uint8_t)((config->int1_active_high ?
                   GOFLY_ICM42688_INT_CONFIG_ACTIVE_HIGH : 0U) |
                  (config->int1_push_pull ?
                   GOFLY_ICM42688_INT_CONFIG_PUSH_PULL : 0U));
    gofly_status_t status;

    status = gofly_icm42688_write_register(
        device, GOFLY_ICM42688_DEVICE_CONFIG_REGISTER,
        GOFLY_ICM42688_DEVICE_CONFIG_SPI_4WIRE);
    if (status != GOFLY_OK) {
        return status;
    }
    status = gofly_icm42688_write_register(
        device, GOFLY_ICM42688_PWR_MGMT0_REGISTER,
        GOFLY_ICM42688_PWR_MGMT0_LOW_NOISE);
    if (status != GOFLY_OK) {
        return status;
    }
    status = gofly_icm42688_write_register(
        device, GOFLY_ICM42688_GYRO_CONFIG0_REGISTER,
        (uint8_t)((gofly_icm42688_gyro_fs_bits(config->gyro_full_scale_dps) << 5U) |
                  (config->gyro_odr & 0x0FU)));
    if (status != GOFLY_OK) {
        return status;
    }
    status = gofly_icm42688_write_register(
        device, GOFLY_ICM42688_ACCEL_CONFIG0_REGISTER,
        (uint8_t)((gofly_icm42688_accel_fs_bits(config->accel_full_scale_g) << 5U) |
                  (config->accel_odr & 0x0FU)));
    if (status != GOFLY_OK) {
        return status;
    }
    status = gofly_icm42688_write_register(
        device, GOFLY_ICM42688_GYRO_CONFIG1_REGISTER,
        (uint8_t)(config->gyro_filter & 0x0FU));
    if (status != GOFLY_OK) {
        return status;
    }
    status = gofly_icm42688_write_register(
        device, GOFLY_ICM42688_ACCEL_CONFIG1_REGISTER,
        (uint8_t)(config->accel_filter & 0x0FU));
    if (status != GOFLY_OK) {
        return status;
    }
    status = gofly_icm42688_write_register(
        device, GOFLY_ICM42688_INT_CONFIG_REGISTER, int_config);
    if (status != GOFLY_OK) {
        return status;
    }
    return gofly_icm42688_write_register(
        device, GOFLY_ICM42688_INT_SOURCE0_REGISTER,
        GOFLY_ICM42688_INT_SOURCE0_DATA_READY);
}

gofly_status_t gofly_icm42688_init(gofly_icm42688_t *device,
                                   const gofly_icm42688_config_t *config)
{
    uint8_t who_am_i = 0U;
    gofly_status_t status;

    if (device == NULL || config == NULL || config->spi.transfer == NULL ||
        config->spi.configure == NULL || config->timeout_ms == 0U ||
        config->sample_timeout_us == 0U ||
        !gofly_icm42688_valid_accel_fs(config->accel_full_scale_g) ||
        !gofly_icm42688_valid_gyro_fs(config->gyro_full_scale_dps) ||
        config->accel_odr != GOFLY_ICM42688_ODR_1KHZ ||
        config->gyro_odr != GOFLY_ICM42688_ODR_1KHZ ||
        config->accel_filter > 0x0FU || config->gyro_filter > 0x0FU) {
        return GOFLY_E_ARGUMENT;
    }
    if (config->spi.configure != NULL) {
        status = config->spi.configure(config->spi.context,
                                       GOFLY_ICM42688_SPI_MODE,
                                       GOFLY_ICM42688_SPI_MAX_HZ);
        if (status != GOFLY_OK) {
            return status;
        }
    }

    memset(device, 0, sizeof(*device));
    device->spi = config->spi;
    device->delay_ms = config->delay_ms;
    device->delay_context = config->delay_context;
    device->time_us = config->time_us;
    device->time_context = config->time_context;
    device->data_ready = config->data_ready;
    device->data_ready_context = config->data_ready_context;
    device->timeout_ms = config->timeout_ms;
    device->sample_timeout_us = config->sample_timeout_us;
    device->accel_full_scale_g = config->accel_full_scale_g;
    device->gyro_full_scale_dps = config->gyro_full_scale_dps;

    status = gofly_icm42688_read_register(device,
                                           GOFLY_ICM42688_WHO_AM_I_REGISTER,
                                           &who_am_i);
    if (status != GOFLY_OK) {
        return status;
    }
    if (who_am_i != GOFLY_ICM42688_WHO_AM_I_VALUE) {
        return GOFLY_E_STATE;
    }
    status = gofly_icm42688_configure(device, config);
    if (status != GOFLY_OK) {
        return status;
    }
    if (device->delay_ms != NULL) {
        status = device->delay_ms(device->delay_context, 1U);
        if (status != GOFLY_OK) {
            return status;
        }
    }
    device->initialized = true;
    return GOFLY_OK;
}

static gofly_status_t gofly_icm42688_check_data_ready(gofly_icm42688_t *device)
{
    uint8_t int_status = 0U;
    gofly_status_t status;

    if (device->data_ready != NULL) {
        return device->data_ready(device->data_ready_context) ?
               GOFLY_OK : GOFLY_E_NOT_READY;
    }
    status = gofly_icm42688_read_register(device,
                                          GOFLY_ICM42688_INT_STATUS_REGISTER,
                                          &int_status);
    if (status != GOFLY_OK) {
        return status;
    }
    return (int_status & GOFLY_ICM42688_INT_STATUS_DATA_READY) != 0U ?
           GOFLY_OK : GOFLY_E_NOT_READY;
}

static gofly_status_t gofly_icm42688_wait_data_ready(gofly_icm42688_t *device)
{
    uint32_t elapsed_us = 0U;
    const uint32_t step_us = 1000U;
    gofly_status_t status;

    while (elapsed_us < device->sample_timeout_us) {
        status = gofly_icm42688_check_data_ready(device);
        if (status == GOFLY_OK) {
            return GOFLY_OK;
        }
        if (status != GOFLY_E_NOT_READY) {
            return status;
        }
        if (device->delay_ms == NULL) {
            return GOFLY_E_TIMEOUT;
        }
        if (device->sample_timeout_us - elapsed_us < step_us) {
            return GOFLY_E_TIMEOUT;
        }
        status = device->delay_ms(device->delay_context, step_us / 1000U);
        if (status != GOFLY_OK) {
            return status;
        }
        elapsed_us += step_us;
    }
    return GOFLY_E_TIMEOUT;
}

gofly_status_t gofly_icm42688_read_sample_at(gofly_icm42688_t *device,
                                             gofly_imu_sample_t *sample,
                                             uint32_t timestamp_us)
{
    uint8_t tx[GOFLY_ICM42688_SAMPLE_BURST_LENGTH + 1U] = {0U};
    uint8_t rx[GOFLY_ICM42688_SAMPLE_BURST_LENGTH + 1U] = {0U};
    int16_t accel_raw[3U];
    int16_t gyro_raw[3U];
    gofly_status_t status;
    float accel_scale;
    float gyro_scale;
    size_t index;

    if (device == NULL || sample == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    sample->valid = false;
    if (!device->initialized) {
        return GOFLY_E_STATE;
    }
    /* Timestamps must advance monotonically; unsigned subtraction keeps the
     * comparison valid across the 32-bit timer wrap. */
    if (device->have_sample &&
        (int32_t)(timestamp_us - device->last_timestamp_us) <= 0) {
        return GOFLY_E_NOT_READY;
    }
    status = gofly_icm42688_wait_data_ready(device);
    if (status != GOFLY_OK) {
        return status;
    }
    tx[0U] = (uint8_t)(GOFLY_ICM42688_ACCEL_DATA_START |
                       GOFLY_ICM42688_SPI_READ);
    status = gofly_icm42688_transfer(device, tx, rx, sizeof(tx));
    if (status != GOFLY_OK) {
        return status;
    }
    /* The command byte occupies rx[0]; a transport that cannot provide the
     * complete burst must report GOFLY_E_RANGE instead of a partial sample. */
    for (index = 1U; index < sizeof(rx); ++index) {
        /* Reading the fixed-size response is intentional; this loop also
         * keeps the conversion source visibly bounded for static analysis. */
        (void)rx[index];
    }
    accel_raw[0U] = gofly_icm42688_i16(&rx[1U]);
    accel_raw[1U] = gofly_icm42688_i16(&rx[3U]);
    accel_raw[2U] = gofly_icm42688_i16(&rx[5U]);
    gyro_raw[0U] = gofly_icm42688_i16(&rx[7U]);
    gyro_raw[1U] = gofly_icm42688_i16(&rx[9U]);
    gyro_raw[2U] = gofly_icm42688_i16(&rx[11U]);
    accel_scale = GOFLY_ICM42688_GRAVITY_MPS2 /
                  gofly_icm42688_accel_sensitivity(
                      device->accel_full_scale_g);
    gyro_scale = (GOFLY_ICM42688_PI / 180.0f) /
                 gofly_icm42688_gyro_sensitivity(
                     device->gyro_full_scale_dps);
    sample->timestamp_us = timestamp_us;
    for (index = 0U; index < 3U; ++index) {
        sample->accel_mps2[index] = (float)accel_raw[index] * accel_scale;
        sample->gyro_rad_s[index] = (float)gyro_raw[index] * gyro_scale;
    }
    sample->valid = true;
    device->last_timestamp_us = timestamp_us;
    device->have_sample = true;
    return GOFLY_OK;
}

gofly_status_t gofly_icm42688_read_sample(gofly_icm42688_t *device,
                                          gofly_imu_sample_t *sample)
{
    uint32_t timestamp_us;

    if (device == NULL || sample == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    if (device->time_us == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    timestamp_us = device->time_us(device->time_context);
    return gofly_icm42688_read_sample_at(device, sample, timestamp_us);
}
