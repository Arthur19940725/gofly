#ifndef GOFLY_PLATFORM_STATE_SNAPSHOT_H
#define GOFLY_PLATFORM_STATE_SNAPSHOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#include "status.h"

#define GOFLY_SNAPSHOT_MAX_ITEM_SIZE 256U

typedef struct {
    _Atomic uint8_t storage[2][GOFLY_SNAPSHOT_MAX_ITEM_SIZE];
    size_t item_size;
    _Atomic uint8_t active_index;
    _Atomic uint32_t sequence;
    _Atomic bool published;
    _Atomic int status;
} gofly_snapshot_t;

void gofly_snapshot_init(gofly_snapshot_t *snapshot, size_t item_size);
void gofly_snapshot_publish(gofly_snapshot_t *snapshot, const void *value);
bool gofly_snapshot_read(const gofly_snapshot_t *snapshot, void *value);
gofly_status_t gofly_snapshot_status(const gofly_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
