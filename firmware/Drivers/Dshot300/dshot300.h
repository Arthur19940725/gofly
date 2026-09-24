#ifndef GOFLY_DRIVER_DSHOT300_H
#define GOFLY_DRIVER_DSHOT300_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "state_types.h"
#include "status.h"

#define GOFLY_DSHOT300_MAX_THROTTLE 2047U

/*
 * Encode one DShot300 packet.  The checked batch API below is the boundary
 * that reports an out-of-range throttle.  This value-only helper returns a
 * zero frame for an invalid throttle so an unchecked caller cannot emit an
 * arbitrary packet.
 */
uint16_t gofly_dshot300_encode(uint16_t throttle, bool telemetry_request);

gofly_status_t gofly_dshot300_build(gofly_motor_command_t command,
                                    uint16_t frames[4]);

#ifdef __cplusplus
}
#endif

#endif
