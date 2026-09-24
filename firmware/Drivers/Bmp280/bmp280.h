#ifndef GOFLY_DRIVER_BMP280_H
#define GOFLY_DRIVER_BMP280_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "bus_interfaces.h"
#include "status.h"

#define GOFLY_BMP280_I2C_ADDRESS 0x76U
#define GOFLY_BMP280_CHIP_ID 0x58U
#define GOFLY_BMP280_REG_CALIBRATION 0x88U
#define GOFLY_BMP280_REG_CHIP_ID 0xD0U
#define GOFLY_BMP280_REG_RESET 0xE0U
#define GOFLY_BMP280_REG_STATUS 0xF3U
#define GOFLY_BMP280_REG_CTRL_MEAS 0xF4U
#define GOFLY_BMP280_REG_CONFIG 0xF5U
#define GOFLY_BMP280_REG_PRESSURE 0xF7U
#define GOFLY_BMP280_CALIBRATION_LENGTH 24U
#define GOFLY_BMP280_MEASUREMENT_LENGTH 6U
#define GOFLY_BMP280_STATUS_MEASURING 0x08U
#define GOFLY_BMP280_CTRL_MEAS_FORCED_X1 0x25U
#define GOFLY_BMP280_CONFIG_STANDBY_125MS 0x20U
#define GOFLY_BMP280_DEFAULT_TIMEOUT_MS 20U
#define GOFLY_BMP280_DEFAULT_POLL_INTERVAL_MS 2U
#define GOFLY_BMP280_DEFAULT_POLL_LIMIT 64U

typedef struct {
    gofly_i2c_bus_t i2c;
    gofly_delay_ms_fn delay_ms;
    void *delay_context;
    uint32_t timeout_ms;
    uint32_t poll_interval_ms;
    uint32_t poll_limit;
} gofly_bmp280_config_t;

typedef struct {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
} gofly_bmp280_calibration_t;

typedef struct {
    /* SI output units: pressure_pa is pascals; temperature_c is degrees C. */
    float pressure_pa;
    float temperature_c;
    int32_t pressure_pa_integer;
    int32_t temperature_centi_c;
    bool valid;
} gofly_bmp280_measurement_t;

typedef struct {
    gofly_i2c_bus_t i2c;
    gofly_delay_ms_fn delay_ms;
    void *delay_context;
    uint32_t timeout_ms;
    uint32_t poll_interval_ms;
    uint32_t poll_limit;
    gofly_bmp280_calibration_t calibration;
    int32_t t_fine;
    bool initialized;
} gofly_bmp280_t;

void gofly_bmp280_default_config(gofly_bmp280_config_t *config);
gofly_status_t gofly_bmp280_init(gofly_bmp280_t *device,
                                 const gofly_bmp280_config_t *config);
gofly_status_t gofly_bmp280_read_pressure_temperature(
    gofly_bmp280_t *device, gofly_bmp280_measurement_t *measurement);
gofly_status_t gofly_bmp280_read_pressure_temperature_values(
    gofly_bmp280_t *device, int32_t *pressure_pa, float *temperature_c);

#ifdef __cplusplus
}
#endif

#endif
