#include <stdbool.h>
#include <stdint.h>

#include "../../Drivers/AdcSense/adc_sense.h"

static void gofly_adc_test_check(bool condition, unsigned *failures)
{
    if (!condition) {
        ++(*failures);
    }
}

int gofly_test_adc_sense_run(void)
{
    unsigned failures = 0U;
    gofly_adc_sense_config_t config;
    gofly_adc_sense_t sense;
    gofly_adc_sense_measurement_t measurement;
    uint32_t voltage_mv = 0U;
    int32_t current_ma = 0;

    gofly_adc_sense_default_config(&config);
    config.divider_numerator = 1U;
    config.divider_denominator = 11U;
    config.reference_mv = 3300U;
    config.resolution_bits = 12U;
    config.current_sense_mv_per_amp = 100U;
    gofly_adc_test_check(gofly_adc_sense_init(&sense, &config) == GOFLY_OK,
                         &failures);
    gofly_adc_test_check(gofly_adc_sense_voltage_mv(&sense, 0U, &voltage_mv) ==
                             GOFLY_OK && voltage_mv == 0U, &failures);
    gofly_adc_test_check(gofly_adc_sense_voltage_mv(&sense, sense.max_raw,
                                                    &voltage_mv) == GOFLY_OK &&
                             voltage_mv == 36300U, &failures);
    gofly_adc_test_check(gofly_adc_sense_current_ma(&sense, sense.max_raw,
                                                    &current_ma) == GOFLY_OK &&
                             current_ma == 33000, &failures);
    gofly_adc_test_check(gofly_adc_sense_convert(&sense, sense.max_raw, 0U,
                                                 &measurement) == GOFLY_OK &&
                             measurement.valid && measurement.voltage_mv ==
                             36300U && measurement.current_ma == 0,
                         &failures);
    gofly_adc_test_check(gofly_adc_sense_voltage_mv(&sense, sense.max_raw + 1U,
                                                    &voltage_mv) ==
                             GOFLY_E_RANGE, &failures);
    gofly_adc_test_check(gofly_adc_sense_current_ma(&sense, 0U, NULL) ==
                             GOFLY_E_ARGUMENT, &failures);

    config.voltage_offset_mv = 100;
    config.current_offset_mv = -50;
    gofly_adc_test_check(gofly_adc_sense_init(&sense, &config) == GOFLY_OK &&
                         gofly_adc_sense_voltage_mv(&sense, 0U, &voltage_mv) ==
                             GOFLY_OK && voltage_mv == 100U &&
                         gofly_adc_sense_current_ma(&sense, 0U, &current_ma) ==
                             GOFLY_OK && current_ma == -50,
                         &failures);

    config.divider_denominator = 0U;
    gofly_adc_test_check(gofly_adc_sense_init(&sense, &config) ==
                             GOFLY_E_ARGUMENT, &failures);
    return (int)failures;
}
