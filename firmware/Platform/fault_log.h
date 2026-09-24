#ifndef GOFLY_PLATFORM_FAULT_LOG_H
#define GOFLY_PLATFORM_FAULT_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "status.h"

#define GOFLY_FAULT_LOG_CAPACITY 16U

typedef struct {
    uint16_t code;
    uint16_t source;
    uint32_t first_timestamp_us;
    uint32_t last_timestamp_us;
    uint32_t count;
    bool active;
} gofly_fault_record_t;

typedef struct {
    gofly_fault_record_t records[GOFLY_FAULT_LOG_CAPACITY];
    size_t count;
    size_t next_index;
    bool overflowed;
    gofly_status_t status;
} gofly_fault_log_t;

void gofly_fault_log_init(gofly_fault_log_t *log);
void gofly_fault_log_record(gofly_fault_log_t *log,
                            uint16_t code,
                            uint16_t source,
                            uint32_t timestamp_us);
void gofly_fault_log_clear(gofly_fault_log_t *log);
bool gofly_fault_log_get(const gofly_fault_log_t *log,
                         size_t index,
                         gofly_fault_record_t *record);
size_t gofly_fault_log_count(const gofly_fault_log_t *log);
bool gofly_fault_log_overflowed(const gofly_fault_log_t *log);
gofly_status_t gofly_fault_log_status(const gofly_fault_log_t *log);

#ifdef __cplusplus
}
#endif

#endif
