#include "state_snapshot.h"

#include <stdatomic.h>

static bool gofly_snapshot_is_valid(const gofly_snapshot_t *snapshot)
{
    return snapshot != NULL && snapshot->item_size != 0U &&
           snapshot->item_size <= GOFLY_SNAPSHOT_MAX_ITEM_SIZE;
}

static void gofly_snapshot_set_status(gofly_snapshot_t *snapshot,
                                      gofly_status_t status)
{
    atomic_store_explicit(&snapshot->status, (int)status, memory_order_relaxed);
}

static gofly_status_t gofly_snapshot_load_status(
    const gofly_snapshot_t *snapshot)
{
    return (gofly_status_t)atomic_load_explicit(&snapshot->status,
                                                 memory_order_relaxed);
}

static void gofly_snapshot_initialize_storage(gofly_snapshot_t *snapshot)
{
    size_t buffer_index;
    size_t byte_index;

    for (buffer_index = 0U; buffer_index < 2U; ++buffer_index) {
        for (byte_index = 0U; byte_index < GOFLY_SNAPSHOT_MAX_ITEM_SIZE;
             ++byte_index) {
            atomic_init(&snapshot->storage[buffer_index][byte_index], 0U);
        }
    }
}

static void gofly_snapshot_copy_to_buffer(
    _Atomic uint8_t *destination,
    const uint8_t *source,
    size_t length)
{
    size_t index;

    for (index = 0U; index < length; ++index) {
        atomic_store_explicit(&destination[index], source[index],
                              memory_order_relaxed);
    }
}

static void gofly_snapshot_copy_from_buffer(
    uint8_t *destination,
    const _Atomic uint8_t *source,
    size_t length)
{
    size_t index;

    for (index = 0U; index < length; ++index) {
        destination[index] = atomic_load_explicit(&source[index],
                                                  memory_order_relaxed);
    }
}

void gofly_snapshot_init(gofly_snapshot_t *snapshot, size_t item_size)
{
    if (snapshot == NULL) {
        return;
    }

    snapshot->item_size = 0U;
    atomic_init(&snapshot->active_index, 0U);
    atomic_init(&snapshot->sequence, 0U);
    atomic_init(&snapshot->published, false);
    atomic_init(&snapshot->status, (int)GOFLY_OK);

    if (item_size == 0U || item_size > GOFLY_SNAPSHOT_MAX_ITEM_SIZE) {
        gofly_snapshot_set_status(snapshot, GOFLY_E_ARGUMENT);
        return;
    }

    snapshot->item_size = item_size;
    gofly_snapshot_initialize_storage(snapshot);
}

void gofly_snapshot_publish(gofly_snapshot_t *snapshot, const void *value)
{
    uint8_t current_index;
    uint8_t next_index;

    if (!gofly_snapshot_is_valid(snapshot) || value == NULL) {
        if (snapshot != NULL) {
            gofly_snapshot_set_status(snapshot, GOFLY_E_ARGUMENT);
        }
        return;
    }

    current_index = (uint8_t)(atomic_load_explicit(&snapshot->active_index,
                                                   memory_order_relaxed) & 1U);
    next_index = (uint8_t)(current_index ^ 1U);
    atomic_fetch_add_explicit(&snapshot->sequence, 1U, memory_order_acq_rel);
    gofly_snapshot_copy_to_buffer(snapshot->storage[next_index],
                                  (const uint8_t *)value,
                                  snapshot->item_size);
    atomic_store_explicit(&snapshot->active_index, next_index,
                          memory_order_release);
    atomic_fetch_add_explicit(&snapshot->sequence, 1U, memory_order_release);
    atomic_store_explicit(&snapshot->published, true, memory_order_release);
    gofly_snapshot_set_status(snapshot, GOFLY_OK);
}

bool gofly_snapshot_read(const gofly_snapshot_t *snapshot, void *value)
{
    uint32_t before;
    uint32_t after;
    uint8_t index;
    size_t attempt;

    if (!gofly_snapshot_is_valid(snapshot) || value == NULL) {
        return false;
    }
    if (!atomic_load_explicit(&snapshot->published, memory_order_acquire)) {
        return false;
    }

    for (attempt = 0U; attempt < 3U; ++attempt) {
        before = atomic_load_explicit(&snapshot->sequence, memory_order_acquire);
        if ((before & 1U) != 0U) {
            continue;
        }
        index = (uint8_t)(atomic_load_explicit(&snapshot->active_index,
                                               memory_order_acquire) & 1U);
        gofly_snapshot_copy_from_buffer(value, snapshot->storage[index],
                                        snapshot->item_size);
        after = atomic_load_explicit(&snapshot->sequence, memory_order_acquire);
        if (before == after && (after & 1U) == 0U) {
            return true;
        }
    }

    return false;
}

gofly_status_t gofly_snapshot_status(const gofly_snapshot_t *snapshot)
{
    return (snapshot != NULL) ? gofly_snapshot_load_status(snapshot) :
                                GOFLY_E_ARGUMENT;
}
