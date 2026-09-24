#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../../Drivers/Bmp280/bmp280.h"

typedef struct {
    uint8_t calibration[GOFLY_BMP280_CALIBRATION_LENGTH];
    uint8_t measurement[GOFLY_BMP280_MEASUREMENT_LENGTH];
    uint8_t chip_id;
    uint8_t status_values[4U];
    size_t status_count;
    size_t status_index;
    size_t read_calls;
    size_t write_calls;
    uint16_t last_address;
    uint16_t last_register;
    gofly_status_t forced_status;
} gofly_bmp280_mock_t;

static void gofly_bmp280_test_check(bool condition, unsigned *failures)
{
    if (!condition) {
        ++(*failures);
    }
}

static gofly_status_t gofly_bmp280_mock_read(void *context, uint16_t address,
                                             uint16_t reg, uint8_t *data,
                                             size_t length, uint32_t timeout_ms)
{
    gofly_bmp280_mock_t *mock = (gofly_bmp280_mock_t *)context;
    (void)timeout_ms;
    ++mock->read_calls;
    mock->last_address = address;
    mock->last_register = reg;
    if (mock->forced_status != GOFLY_OK) {
        return mock->forced_status;
    }
    if (reg == GOFLY_BMP280_REG_CHIP_ID && length == 1U) {
        data[0U] = mock->chip_id;
    } else if (reg == GOFLY_BMP280_REG_CALIBRATION &&
               length == GOFLY_BMP280_CALIBRATION_LENGTH) {
        memcpy(data, mock->calibration, length);
    } else if (reg == GOFLY_BMP280_REG_STATUS && length == 1U) {
        data[0U] = mock->status_index < mock->status_count ?
                   mock->status_values[mock->status_index++] : 0U;
    } else if (reg == GOFLY_BMP280_REG_PRESSURE &&
               length == GOFLY_BMP280_MEASUREMENT_LENGTH) {
        memcpy(data, mock->measurement, length);
    } else {
        return GOFLY_E_RANGE;
    }
    return GOFLY_OK;
}

static gofly_status_t gofly_bmp280_mock_write(void *context, uint16_t address,
                                              uint16_t reg,
                                              const uint8_t *data,
                                              size_t length, uint32_t timeout_ms)
{
    gofly_bmp280_mock_t *mock = (gofly_bmp280_mock_t *)context;
    (void)data;
    (void)timeout_ms;
    ++mock->write_calls;
    mock->last_address = address;
    mock->last_register = reg;
    return mock->forced_status == GOFLY_OK && length == 1U ?
           GOFLY_OK : mock->forced_status != GOFLY_OK ?
           mock->forced_status : GOFLY_E_RANGE;
}

static gofly_status_t gofly_bmp280_mock_delay(void *context, uint32_t delay_ms)
{
    (void)context;
    (void)delay_ms;
    return GOFLY_OK;
}

static void gofly_bmp280_put_u16(uint8_t *data, uint16_t value)
{
    data[0U] = (uint8_t)value;
    data[1U] = (uint8_t)(value >> 8U);
}

static void gofly_bmp280_put_i16(uint8_t *data, int16_t value)
{
    gofly_bmp280_put_u16(data, (uint16_t)value);
}

int gofly_test_bmp280_run(void)
{
    unsigned failures = 0U;
    gofly_bmp280_mock_t mock = {0};
    gofly_bmp280_config_t config;
    gofly_bmp280_t device;
    gofly_bmp280_measurement_t measurement = {0};

    mock.chip_id = GOFLY_BMP280_CHIP_ID;
    /* Bosch BMP280 sample calibration vector. */
    gofly_bmp280_put_u16(&mock.calibration[0U], 27504U);
    gofly_bmp280_put_i16(&mock.calibration[2U], 26435);
    gofly_bmp280_put_i16(&mock.calibration[4U], -1000);
    gofly_bmp280_put_u16(&mock.calibration[6U], 36477U);
    gofly_bmp280_put_i16(&mock.calibration[8U], -10685);
    gofly_bmp280_put_i16(&mock.calibration[10U], 3024);
    gofly_bmp280_put_i16(&mock.calibration[12U], 2855);
    gofly_bmp280_put_i16(&mock.calibration[14U], 140);
    gofly_bmp280_put_i16(&mock.calibration[16U], -7);
    gofly_bmp280_put_i16(&mock.calibration[18U], 15500);
    gofly_bmp280_put_i16(&mock.calibration[20U], -14600);
    gofly_bmp280_put_i16(&mock.calibration[22U], 6000);
    /* adc_P=415148, adc_T=519888 -> 100653 Pa and 25.08 C. */
    mock.measurement[0U] = 0x65U;
    mock.measurement[1U] = 0x5AU;
    mock.measurement[2U] = 0xC0U;
    mock.measurement[3U] = 0x7EU;
    mock.measurement[4U] = 0xEDU;
    mock.measurement[5U] = 0x00U;
    mock.status_values[0U] = 0U;
    mock.status_count = 1U;

    gofly_bmp280_default_config(&config);
    config.i2c.context = &mock;
    config.i2c.read = gofly_bmp280_mock_read;
    config.i2c.write = gofly_bmp280_mock_write;
    config.delay_ms = gofly_bmp280_mock_delay;
    config.delay_context = &mock;
    gofly_bmp280_test_check(gofly_bmp280_init(&device, &config) == GOFLY_OK,
                            &failures);
    gofly_bmp280_test_check(mock.last_address == GOFLY_BMP280_I2C_ADDRESS,
                            &failures);
    gofly_bmp280_test_check(gofly_bmp280_read_pressure_temperature(
                                &device, &measurement) == GOFLY_OK &&
                            measurement.valid &&
                            measurement.pressure_pa_integer == 100653 &&
                            measurement.temperature_centi_c == 2508,
                            &failures);
    mock.forced_status = GOFLY_E_TIMEOUT;
    gofly_bmp280_test_check(gofly_bmp280_read_pressure_temperature(
                                &device, &measurement) == GOFLY_E_TIMEOUT,
                            &failures);
    mock.forced_status = GOFLY_OK;
    mock.status_values[0U] = GOFLY_BMP280_STATUS_MEASURING;
    mock.status_count = 1U;
    mock.status_index = 0U;
    config.poll_limit = 1U;
    gofly_bmp280_test_check(gofly_bmp280_init(&device, &config) == GOFLY_OK,
                            &failures);
    gofly_bmp280_test_check(gofly_bmp280_read_pressure_temperature(
                                &device, &measurement) == GOFLY_E_TIMEOUT,
                            &failures);
    return (int)failures;
}
