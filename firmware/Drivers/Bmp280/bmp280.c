#include "bmp280.h"

#include <string.h>

static uint16_t gofly_bmp280_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0U] |
                      ((uint16_t)data[1U] << 8U));
}

static int16_t gofly_bmp280_i16_le(const uint8_t *data)
{
    return (int16_t)gofly_bmp280_u16_le(data);
}

static uint32_t gofly_bmp280_adc24(const uint8_t *data)
{
    return ((uint32_t)data[0U] << 12U) |
           ((uint32_t)data[1U] << 4U) |
           ((uint32_t)data[2U] >> 4U);
}

static gofly_status_t gofly_bmp280_read(gofly_bmp280_t *device,
                                        uint16_t reg,
                                        uint8_t *data,
                                        size_t length)
{
    if (device == NULL || device->i2c.read == NULL || data == NULL ||
        length == 0U) {
        return GOFLY_E_ARGUMENT;
    }
    return device->i2c.read(device->i2c.context, GOFLY_BMP280_I2C_ADDRESS,
                            reg, data, length, device->timeout_ms);
}

static gofly_status_t gofly_bmp280_write(gofly_bmp280_t *device,
                                         uint16_t reg,
                                         const uint8_t *data,
                                         size_t length)
{
    if (device == NULL || device->i2c.write == NULL || data == NULL ||
        length == 0U) {
        return GOFLY_E_ARGUMENT;
    }
    return device->i2c.write(device->i2c.context, GOFLY_BMP280_I2C_ADDRESS,
                             reg, data, length, device->timeout_ms);
}

void gofly_bmp280_default_config(gofly_bmp280_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->timeout_ms = GOFLY_BMP280_DEFAULT_TIMEOUT_MS;
    config->poll_interval_ms = GOFLY_BMP280_DEFAULT_POLL_INTERVAL_MS;
    config->poll_limit = GOFLY_BMP280_DEFAULT_POLL_LIMIT;
}

static void gofly_bmp280_decode_calibration(
    gofly_bmp280_calibration_t *calibration, const uint8_t *data)
{
    calibration->dig_t1 = gofly_bmp280_u16_le(&data[0U]);
    calibration->dig_t2 = gofly_bmp280_i16_le(&data[2U]);
    calibration->dig_t3 = gofly_bmp280_i16_le(&data[4U]);
    calibration->dig_p1 = gofly_bmp280_u16_le(&data[6U]);
    calibration->dig_p2 = gofly_bmp280_i16_le(&data[8U]);
    calibration->dig_p3 = gofly_bmp280_i16_le(&data[10U]);
    calibration->dig_p4 = gofly_bmp280_i16_le(&data[12U]);
    calibration->dig_p5 = gofly_bmp280_i16_le(&data[14U]);
    calibration->dig_p6 = gofly_bmp280_i16_le(&data[16U]);
    calibration->dig_p7 = gofly_bmp280_i16_le(&data[18U]);
    calibration->dig_p8 = gofly_bmp280_i16_le(&data[20U]);
    calibration->dig_p9 = gofly_bmp280_i16_le(&data[22U]);
}

gofly_status_t gofly_bmp280_init(gofly_bmp280_t *device,
                                 const gofly_bmp280_config_t *config)
{
    uint8_t chip_id = 0U;
    uint8_t calibration[GOFLY_BMP280_CALIBRATION_LENGTH] = {0U};
    const uint8_t config_register = GOFLY_BMP280_CONFIG_STANDBY_125MS;
    const uint8_t control_register = GOFLY_BMP280_CTRL_MEAS_FORCED_X1;
    gofly_status_t status;

    if (device == NULL || config == NULL || config->i2c.read == NULL ||
        config->i2c.write == NULL || config->timeout_ms == 0U ||
        config->poll_interval_ms == 0U || config->poll_limit == 0U) {
        return GOFLY_E_ARGUMENT;
    }
    memset(device, 0, sizeof(*device));
    device->i2c = config->i2c;
    device->delay_ms = config->delay_ms;
    device->delay_context = config->delay_context;
    device->timeout_ms = config->timeout_ms;
    device->poll_interval_ms = config->poll_interval_ms;
    device->poll_limit = config->poll_limit;

    status = gofly_bmp280_read(device, GOFLY_BMP280_REG_CHIP_ID,
                               &chip_id, sizeof(chip_id));
    if (status != GOFLY_OK) {
        return status;
    }
    if (chip_id != GOFLY_BMP280_CHIP_ID) {
        return GOFLY_E_STATE;
    }
    status = gofly_bmp280_read(device, GOFLY_BMP280_REG_CALIBRATION,
                               calibration, sizeof(calibration));
    if (status != GOFLY_OK) {
        return status;
    }
    gofly_bmp280_decode_calibration(&device->calibration, calibration);
    if (device->calibration.dig_p1 == 0U) {
        return GOFLY_E_STATE;
    }
    status = gofly_bmp280_write(device, GOFLY_BMP280_REG_CONFIG,
                                &config_register, sizeof(config_register));
    if (status != GOFLY_OK) {
        return status;
    }
    status = gofly_bmp280_write(device, GOFLY_BMP280_REG_CTRL_MEAS,
                                &control_register, sizeof(control_register));
    if (status != GOFLY_OK) {
        return status;
    }
    device->initialized = true;
    return GOFLY_OK;
}

static gofly_status_t gofly_bmp280_wait_idle(gofly_bmp280_t *device)
{
    uint8_t status_register = 0U;
    uint32_t attempt;
    uint32_t elapsed_ms = 0U;
    gofly_status_t status;

    for (attempt = 0U; attempt < device->poll_limit; ++attempt) {
        status = gofly_bmp280_read(device, GOFLY_BMP280_REG_STATUS,
                                   &status_register, sizeof(status_register));
        if (status != GOFLY_OK) {
            return status;
        }
        if ((status_register & GOFLY_BMP280_STATUS_MEASURING) == 0U) {
            return GOFLY_OK;
        }
        if (device->delay_ms == NULL ||
            elapsed_ms >= device->timeout_ms ||
            device->poll_interval_ms > device->timeout_ms - elapsed_ms) {
            return GOFLY_E_TIMEOUT;
        }
        status = device->delay_ms(device->delay_context,
                                  device->poll_interval_ms);
        if (status != GOFLY_OK) {
            return status;
        }
        elapsed_ms += device->poll_interval_ms;
    }
    return GOFLY_E_TIMEOUT;
}

static gofly_status_t gofly_bmp280_compensate(
    gofly_bmp280_t *device, uint32_t adc_pressure, uint32_t adc_temperature,
    int32_t *temperature_centi_c, int32_t *pressure_pa)
{
    const gofly_bmp280_calibration_t *calibration = &device->calibration;
    int32_t var1_temperature;
    int32_t var2_temperature;
    int64_t var1_pressure;
    int64_t var2_pressure;
    int64_t pressure;
    int32_t temperature;

    var1_temperature = ((((int32_t)(adc_temperature >> 3U) -
                          ((int32_t)calibration->dig_t1 << 1)) *
                         (int32_t)calibration->dig_t2) >> 11);
    var2_temperature = (((((int32_t)(adc_temperature >> 4U) -
                            (int32_t)calibration->dig_t1) *
                           ((int32_t)(adc_temperature >> 4U) -
                            (int32_t)calibration->dig_t1)) >> 12) *
                         (int32_t)calibration->dig_t3) >> 14;
    device->t_fine = var1_temperature + var2_temperature;
    temperature = (device->t_fine * 5 + 128) >> 8;

    var1_pressure = (int64_t)device->t_fine - 128000;
    var2_pressure = var1_pressure * var1_pressure * calibration->dig_p6;
    var2_pressure += (var1_pressure * calibration->dig_p5) << 17;
    var2_pressure += ((int64_t)calibration->dig_p4) << 35;
    var1_pressure = ((var1_pressure * var1_pressure * calibration->dig_p3) >> 8) +
                    ((var1_pressure * calibration->dig_p2) << 12);
    var1_pressure = ((((int64_t)1 << 47) + var1_pressure) *
                     calibration->dig_p1) >> 33;
    if (var1_pressure == 0) {
        return GOFLY_E_STATE;
    }
    pressure = 1048576 - (int64_t)adc_pressure;
    pressure = (((pressure << 31) - var2_pressure) * 3125) /
               var1_pressure;
    var1_pressure = ((int64_t)calibration->dig_p9 *
                     (pressure >> 13) * (pressure >> 13)) >> 25;
    var2_pressure = ((int64_t)calibration->dig_p8 * pressure) >> 19;
    pressure = ((pressure + var1_pressure + var2_pressure) >> 8) +
               ((int64_t)calibration->dig_p7 << 4);
    if (pressure < 0 || pressure > INT32_MAX * 256LL) {
        return GOFLY_E_RANGE;
    }
    *temperature_centi_c = temperature;
    *pressure_pa = (int32_t)((pressure + 128) >> 8);
    return GOFLY_OK;
}

gofly_status_t gofly_bmp280_read_pressure_temperature(
    gofly_bmp280_t *device, gofly_bmp280_measurement_t *measurement)
{
    uint8_t control_register = GOFLY_BMP280_CTRL_MEAS_FORCED_X1;
    uint8_t data[GOFLY_BMP280_MEASUREMENT_LENGTH] = {0U};
    int32_t pressure_pa;
    int32_t temperature_centi_c;
    gofly_status_t status;

    if (device == NULL || !device->initialized) {
        return GOFLY_E_STATE;
    }
    if (measurement == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    measurement->valid = false;
    status = gofly_bmp280_write(device, GOFLY_BMP280_REG_CTRL_MEAS,
                                &control_register, sizeof(control_register));
    if (status != GOFLY_OK) {
        return status;
    }
    status = gofly_bmp280_wait_idle(device);
    if (status != GOFLY_OK) {
        return status;
    }
    status = gofly_bmp280_read(device, GOFLY_BMP280_REG_PRESSURE,
                               data, sizeof(data));
    if (status != GOFLY_OK) {
        return status;
    }
    status = gofly_bmp280_compensate(device, gofly_bmp280_adc24(&data[0U]),
                                     gofly_bmp280_adc24(&data[3U]),
                                     &temperature_centi_c, &pressure_pa);
    if (status != GOFLY_OK) {
        return status;
    }
    measurement->pressure_pa_integer = pressure_pa;
    measurement->temperature_centi_c = temperature_centi_c;
    measurement->pressure_pa = (float)pressure_pa;
    measurement->temperature_c = (float)temperature_centi_c / 100.0f;
    measurement->valid = true;
    return GOFLY_OK;
}

gofly_status_t gofly_bmp280_read_pressure_temperature_values(
    gofly_bmp280_t *device, int32_t *pressure_pa, float *temperature_c)
{
    gofly_bmp280_measurement_t measurement;
    gofly_status_t status;

    if (pressure_pa == NULL || temperature_c == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    status = gofly_bmp280_read_pressure_temperature(device, &measurement);
    if (status != GOFLY_OK) {
        return status;
    }
    *pressure_pa = measurement.pressure_pa_integer;
    *temperature_c = measurement.temperature_c;
    return GOFLY_OK;
}
