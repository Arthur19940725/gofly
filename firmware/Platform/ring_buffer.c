#include "ring_buffer.h"

#include <stdatomic.h>

static bool gofly_ring_is_valid(const gofly_ring_t *ring)
{
    return ring != NULL && ring->storage != NULL && ring->capacity != 0U;
}

static void gofly_ring_set_status(gofly_ring_t *ring, gofly_status_t status)
{
    atomic_store_explicit(&ring->status, (int)status, memory_order_relaxed);
}

static gofly_status_t gofly_ring_load_status(const gofly_ring_t *ring)
{
    return (gofly_status_t)atomic_load_explicit(&ring->status,
                                                 memory_order_relaxed);
}

void gofly_ring_init(gofly_ring_t *ring, uint8_t *storage, size_t capacity)
{
    if (ring == NULL) {
        return;
    }

    ring->storage = storage;
    ring->capacity = capacity;
    atomic_init(&ring->head, 0U);
    atomic_init(&ring->tail, 0U);
    atomic_init(&ring->status, (storage != NULL && capacity != 0U) ?
                (int)GOFLY_OK : (int)GOFLY_E_ARGUMENT);
}

size_t gofly_ring_available(const gofly_ring_t *ring)
{
    if (!gofly_ring_is_valid(ring)) {
        return 0U;
    }

    const size_t head = atomic_load_explicit(&ring->head, memory_order_acquire);
    const size_t tail = atomic_load_explicit(&ring->tail, memory_order_acquire);

    return head - tail;
}

size_t gofly_ring_write(gofly_ring_t *ring, const uint8_t *data, size_t length)
{
    size_t available;
    size_t head;
    size_t index;
    size_t offset;

    if (!gofly_ring_is_valid(ring)) {
        if (ring != NULL) {
            gofly_ring_set_status(ring, GOFLY_E_ARGUMENT);
        }
        return 0U;
    }
    if (length == 0U) {
        gofly_ring_set_status(ring, GOFLY_OK);
        return 0U;
    }
    if (data == NULL) {
        gofly_ring_set_status(ring, GOFLY_E_ARGUMENT);
        return 0U;
    }

    available = gofly_ring_available(ring);
    if (length > ring->capacity - available) {
        gofly_ring_set_status(ring, GOFLY_E_RANGE);
        return 0U;
    }

    head = atomic_load_explicit(&ring->head, memory_order_relaxed);
    for (offset = 0U; offset < length; ++offset) {
        index = head % ring->capacity;
        ring->storage[index] = data[offset];
        ++head;
    }
    atomic_store_explicit(&ring->head, head, memory_order_release);
    gofly_ring_set_status(ring, GOFLY_OK);
    return length;
}

size_t gofly_ring_read(gofly_ring_t *ring, uint8_t *data, size_t length)
{
    size_t available;
    size_t count;
    size_t tail;
    size_t index;
    size_t offset;

    if (!gofly_ring_is_valid(ring)) {
        if (ring != NULL) {
            gofly_ring_set_status(ring, GOFLY_E_ARGUMENT);
        }
        return 0U;
    }
    if (length == 0U) {
        gofly_ring_set_status(ring, GOFLY_OK);
        return 0U;
    }
    if (data == NULL) {
        gofly_ring_set_status(ring, GOFLY_E_ARGUMENT);
        return 0U;
    }

    available = gofly_ring_available(ring);
    count = (length < available) ? length : available;
    tail = atomic_load_explicit(&ring->tail, memory_order_relaxed);
    for (offset = 0U; offset < count; ++offset) {
        index = tail % ring->capacity;
        data[offset] = ring->storage[index];
        ++tail;
    }
    atomic_store_explicit(&ring->tail, tail, memory_order_release);
    gofly_ring_set_status(ring, GOFLY_OK);
    return count;
}

gofly_status_t gofly_ring_status(const gofly_ring_t *ring)
{
    return (ring != NULL) ? gofly_ring_load_status(ring) : GOFLY_E_ARGUMENT;
}
