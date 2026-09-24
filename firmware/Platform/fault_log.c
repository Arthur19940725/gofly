#include "fault_log.h"

#include <string.h>

static size_t gofly_fault_physical_index(const gofly_fault_log_t *log,
                                         size_t logical_index)
{
    if (log->count == GOFLY_FAULT_LOG_CAPACITY) {
        return (log->next_index + logical_index) %
               GOFLY_FAULT_LOG_CAPACITY;
    }
    return logical_index;
}

static gofly_fault_record_t *gofly_fault_find(gofly_fault_log_t *log,
                                              uint16_t code,
                                              uint16_t source)
{
    size_t index;
    size_t physical_index;
    gofly_fault_record_t *record;

    for (index = 0U; index < log->count; ++index) {
        physical_index = gofly_fault_physical_index(log, index);
        record = &log->records[physical_index];
        if (record->active && record->code == code &&
            record->source == source) {
            return record;
        }
    }
    return NULL;
}

void gofly_fault_log_init(gofly_fault_log_t *log)
{
    if (log == NULL) {
        return;
    }

    memset(log, 0, sizeof(*log));
    log->status = GOFLY_OK;
}

void gofly_fault_log_record(gofly_fault_log_t *log,
                            uint16_t code,
                            uint16_t source,
                            uint32_t timestamp_us)
{
    gofly_fault_record_t *existing;
    gofly_fault_record_t *record;
    size_t physical_index;

    if (log == NULL) {
        return;
    }

    existing = gofly_fault_find(log, code, source);
    if (existing != NULL) {
        if (existing->count != UINT32_MAX) {
            ++existing->count;
        }
        existing->last_timestamp_us = timestamp_us;
        existing->active = true;
        log->status = GOFLY_OK;
        return;
    }

    if (log->count < GOFLY_FAULT_LOG_CAPACITY) {
        physical_index = log->count;
        ++log->count;
        log->next_index = (log->count == GOFLY_FAULT_LOG_CAPACITY) ?
                           0U : log->count;
    } else {
        physical_index = log->next_index;
        log->next_index = (log->next_index + 1U) % GOFLY_FAULT_LOG_CAPACITY;
        log->overflowed = true;
    }

    record = &log->records[physical_index];
    record->code = code;
    record->source = source;
    record->first_timestamp_us = timestamp_us;
    record->last_timestamp_us = timestamp_us;
    record->count = 1U;
    record->active = true;
    log->status = GOFLY_OK;
}

void gofly_fault_log_clear(gofly_fault_log_t *log)
{
    if (log == NULL) {
        return;
    }

    memset(log->records, 0, sizeof(log->records));
    log->count = 0U;
    log->next_index = 0U;
    log->overflowed = false;
    log->status = GOFLY_OK;
}

bool gofly_fault_log_get(const gofly_fault_log_t *log,
                         size_t index,
                         gofly_fault_record_t *record)
{
    size_t physical_index;

    if (log == NULL || record == NULL) {
        return false;
    }
    if (index >= log->count) {
        return false;
    }

    physical_index = gofly_fault_physical_index(log, index);
    *record = log->records[physical_index];
    return true;
}

size_t gofly_fault_log_count(const gofly_fault_log_t *log)
{
    return (log != NULL) ? log->count : 0U;
}

bool gofly_fault_log_overflowed(const gofly_fault_log_t *log)
{
    return log != NULL && log->overflowed;
}

gofly_status_t gofly_fault_log_status(const gofly_fault_log_t *log)
{
    return (log != NULL) ? log->status : GOFLY_E_ARGUMENT;
}
