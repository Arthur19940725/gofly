#ifndef GOFLY_DRIVER_ADC_SENSE_H
#define GOFLY_DRIVER_ADC_SENSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "status.h"

typedef struct {
    /* Measured input = ADC pin voltage * divider_denominator /
     * divider_numerator.  For a 1/11 divider, use 1 and 11. */
    uint32_t divider_numerator;
    uint32_t divider_denominator;
    uint32_t reference_mv;
    uint8_t resolution_bits;
    uint32_t current_sense_mv_per_amp;
    int32_t voltage_offset_mv;
    int32_t current_offset_mv;
} gofly_adc_sense_config_t;

typedef struct {
    uint32_t voltage_mv;
    int32_t current_ma;
    bool valid;
} gofly_adc_sense_measurement_t;

typedef struct {
    gofly_adc_sense_config_t config;
    uint32_t max_raw;
    bool initialized;
} gofly_adc_sense_t;

void gofly_adc_sense_default_config(gofly_adc_sense_config_t *config);
gofly_status_t gofly_adc_sense_init(gofly_adc_sense_t *sense,
                                    const gofly_adc_sense_config_t *config);
gofly_status_t gofly_adc_sense_voltage_mv(const gofly_adc_sense_t *sense,
                                          uint32_t raw,
                                          uint32_t *voltage_mv);
gofly_status_t gofly_adc_sense_current_ma(const gofly_adc_sense_t *sense,
                                          uint32_t raw,
                                          int32_t *current_ma);
gofly_status_t gofly_adc_sense_convert(const gofly_adc_sense_t *sense,
                                       uint32_t voltage_raw,
                                       uint32_t current_raw,
                                       gofly_adc_sense_measurement_t *measurement);

#ifdef __cplusplus
}
#endif

#endif
