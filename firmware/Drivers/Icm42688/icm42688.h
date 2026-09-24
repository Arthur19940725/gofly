#ifndef GOFLY_DRIVER_ICM42688_H
#define GOFLY_DRIVER_ICM42688_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "bus_interfaces.h"
#include "state_types.h"
#include "status.h"

#define GOFLY_ICM42688_WHO_AM_I_VALUE 0x47U
#define GOFLY_ICM42688_WHO_AM_I_REGISTER 0x75U
#define GOFLY_ICM42688_DEVICE_CONFIG_REGISTER 0x11U
#define GOFLY_ICM42688_INT_CONFIG_REGISTER 0x14U
#define GOFLY_ICM42688_INT_STATUS_REGISTER 0x2DU
#define GOFLY_ICM42688_PWR_MGMT0_REGISTER 0x4EU
#define GOFLY_ICM42688_GYRO_CONFIG0_REGISTER 0x4FU
#define GOFLY_ICM42688_ACCEL_CONFIG0_REGISTER 0x50U
#define GOFLY_ICM42688_GYRO_CONFIG1_REGISTER 0x51U
#define GOFLY_ICM42688_ACCEL_CONFIG1_REGISTER 0x53U
#define GOFLY_ICM42688_INT_SOURCE0_REGISTER 0x65U
#define GOFLY_ICM42688_TEMP_DATA1_REGISTER 0x1DU
#define GOFLY_ICM42688_SAMPLE_BURST_LENGTH 14U
#define GOFLY_ICM42688_INT_STATUS_DATA_READY 0x08U
#define GOFLY_ICM42688_ODR_1KHZ 0x06U
#define GOFLY_ICM42688_DEFAULT_TIMEOUT_MS 10U
#define GOFLY_ICM42688_DEFAULT_SAMPLE_TIMEOUT_US 3000U

/* Full-scale values are expressed in the physical units used by callers. */
typedef enum {
    GOFLY_ICM42688_ACCEL_FS_16G = 16,
    GOFLY_ICM42688_ACCEL_FS_8G = 8,
    GOFLY_ICM42688_ACCEL_FS_4G = 4,
    GOFLY_ICM42688_ACCEL_FS_2G = 2
} gofly_icm42688_accel_fs_t;

typedef enum {
    GOFLY_ICM42688_GYRO_FS_2000DPS = 2000,
    GOFLY_ICM42688_GYRO_FS_1000DPS = 1000,
    GOFLY_ICM42688_GYRO_FS_500DPS = 500,
    GOFLY_ICM42688_GYRO_FS_250DPS = 250,
    GOFLY_ICM42688_GYRO_FS_125DPS = 125
} gofly_icm42688_gyro_fs_t;

typedef struct {
    gofly_spi_bus_t spi;
    gofly_delay_ms_fn delay_ms;
    void *delay_context;
    gofly_time_us_fn time_us;
    void *time_context;
    gofly_data_ready_fn data_ready;
    void *data_ready_context;
    uint32_t timeout_ms;
    uint32_t sample_timeout_us;
    uint8_t accel_full_scale_g;
    uint16_t gyro_full_scale_dps;
    uint8_t accel_odr;
    uint8_t gyro_odr;
    uint8_t accel_filter;
    uint8_t gyro_filter;
    bool int1_active_high;
    bool int1_push_pull;
} gofly_icm42688_config_t;

typedef struct {
    gofly_spi_bus_t spi;
    gofly_delay_ms_fn delay_ms;
    void *delay_context;
    gofly_time_us_fn time_us;
    void *time_context;
    gofly_data_ready_fn data_ready;
    void *data_ready_context;
    uint32_t timeout_ms;
    uint32_t sample_timeout_us;
    uint8_t accel_full_scale_g;
    uint16_t gyro_full_scale_dps;
    bool initialized;
    bool have_sample;
    uint32_t last_timestamp_us;
} gofly_icm42688_t;

void gofly_icm42688_default_config(gofly_icm42688_config_t *config);
gofly_status_t gofly_icm42688_init(gofly_icm42688_t *device,
                                   const gofly_icm42688_config_t *config);
gofly_status_t gofly_icm42688_read_sample(gofly_icm42688_t *device,
                                          gofly_imu_sample_t *sample);
gofly_status_t gofly_icm42688_read_sample_at(gofly_icm42688_t *device,
                                             gofly_imu_sample_t *sample,
                                             uint32_t timestamp_us);
gofly_status_t gofly_icm42688_read_register(gofly_icm42688_t *device,
                                            uint8_t reg,
                                            uint8_t *value);
gofly_status_t gofly_icm42688_write_register(gofly_icm42688_t *device,
                                             uint8_t reg,
                                             uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
