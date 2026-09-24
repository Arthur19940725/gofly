#include "adc_sense.h"

#include <string.h>

void gofly_adc_sense_default_config(gofly_adc_sense_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->divider_numerator = 1U;
    config->divider_denominator = 1U;
    config->reference_mv = 3300U;
    config->resolution_bits = 12U;
    config->current_sense_mv_per_amp = 1U;
}

gofly_status_t gofly_adc_sense_init(gofly_adc_sense_t *sense,
                                    const gofly_adc_sense_config_t *config)
{
    if (sense == NULL || config == NULL || config->divider_numerator == 0U ||
        config->divider_denominator == 0U || config->reference_mv == 0U ||
        config->resolution_bits == 0U || config->resolution_bits > 31U ||
        config->current_sense_mv_per_amp == 0U) {
        return GOFLY_E_ARGUMENT;
    }
    memset(sense, 0, sizeof(*sense));
    sense->config = *config;
    sense->max_raw = (uint32_t)(((uint32_t)1U << config->resolution_bits) - 1U);
    sense->initialized = true;
    return GOFLY_OK;
}

static gofly_status_t gofly_adc_sense_check(const gofly_adc_sense_t *sense,
                                            uint32_t raw)
{
    if (sense == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    if (!sense->initialized) {
        return GOFLY_E_STATE;
    }
    if (raw > sense->max_raw) {
        return GOFLY_E_RANGE;
    }
    return GOFLY_OK;
}

gofly_status_t gofly_adc_sense_voltage_mv(const gofly_adc_sense_t *sense,
                                          uint32_t raw,
                                          uint32_t *voltage_mv)
{
    uint64_t adc_mv;
    int64_t scaled_mv;
    gofly_status_t status;

    if (voltage_mv == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    status = gofly_adc_sense_check(sense, raw);
    if (status != GOFLY_OK) {
        return status;
    }
    adc_mv = ((uint64_t)raw * sense->config.reference_mv +
              (sense->max_raw / 2U)) / sense->max_raw;
    scaled_mv = (int64_t)((adc_mv * sense->config.divider_denominator +
                           (sense->config.divider_numerator / 2U)) /
                          sense->config.divider_numerator);
    scaled_mv += sense->config.voltage_offset_mv;
    if (scaled_mv < 0 || scaled_mv > UINT32_MAX) {
        return GOFLY_E_RANGE;
    }
    *voltage_mv = (uint32_t)scaled_mv;
    return GOFLY_OK;
}

gofly_status_t gofly_adc_sense_current_ma(const gofly_adc_sense_t *sense,
                                          uint32_t raw,
                                          int32_t *current_ma)
{
    uint64_t adc_mv;
    int64_t current;
    gofly_status_t status;

    if (current_ma == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    status = gofly_adc_sense_check(sense, raw);
    if (status != GOFLY_OK) {
        return status;
    }
    adc_mv = ((uint64_t)raw * sense->config.reference_mv +
              (sense->max_raw / 2U)) / sense->max_raw;
    current = (int64_t)((adc_mv * 1000U +
                         (sense->config.current_sense_mv_per_amp / 2U)) /
                        sense->config.current_sense_mv_per_amp);
    current += sense->config.current_offset_mv;
    if (current < INT32_MIN || current > INT32_MAX) {
        return GOFLY_E_RANGE;
    }
    *current_ma = (int32_t)current;
    return GOFLY_OK;
}

gofly_status_t gofly_adc_sense_convert(const gofly_adc_sense_t *sense,
                                       uint32_t voltage_raw,
                                       uint32_t current_raw,
                                       gofly_adc_sense_measurement_t *measurement)
{
    gofly_status_t status;

    if (measurement == NULL) {
        return GOFLY_E_ARGUMENT;
    }
    measurement->valid = false;
    status = gofly_adc_sense_voltage_mv(sense, voltage_raw,
                                        &measurement->voltage_mv);
    if (status != GOFLY_OK) {
        return status;
    }
    status = gofly_adc_sense_current_ma(sense, current_raw,
                                        &measurement->current_ma);
    if (status != GOFLY_OK) {
        return status;
    }
    measurement->valid = true;
    return GOFLY_OK;
}
