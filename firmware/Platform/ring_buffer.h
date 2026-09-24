#ifndef GOFLY_PLATFORM_RING_BUFFER_H
#define GOFLY_PLATFORM_RING_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#include "status.h"

typedef struct {
    uint8_t *storage;
    size_t capacity;
    _Atomic size_t head;
    _Atomic size_t tail;
    _Atomic int status;
} gofly_ring_t;

void gofly_ring_init(gofly_ring_t *ring, uint8_t *storage, size_t capacity);
size_t gofly_ring_write(gofly_ring_t *ring, const uint8_t *data, size_t length);
size_t gofly_ring_read(gofly_ring_t *ring, uint8_t *data, size_t length);
size_t gofly_ring_available(const gofly_ring_t *ring);
gofly_status_t gofly_ring_status(const gofly_ring_t *ring);

#ifdef __cplusplus
}
#endif

#endif
