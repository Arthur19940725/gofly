#include "dshot300.h"

static uint16_t gofly_dshot300_packet(uint16_t throttle,
                                      bool telemetry_request)
{
    const uint16_t packet = (uint16_t)((throttle << 5U) |
                                       (telemetry_request ? 0x10U : 0U));
    const uint16_t crc = (uint16_t)((packet ^ (packet >> 4U) ^
                                     (packet >> 8U)) & 0x0FU);
    return (uint16_t)((packet << 4U) | crc);
}

uint16_t gofly_dshot300_encode(uint16_t throttle, bool telemetry_request)
{
    if (throttle > GOFLY_DSHOT300_MAX_THROTTLE) {
        return 0U;
    }
    return gofly_dshot300_packet(throttle, telemetry_request);
}

gofly_status_t gofly_dshot300_build(gofly_motor_command_t command,
                                    uint16_t frames[4])
{
    uint8_t index;

    if (frames == NULL) {
        return GOFLY_E_ARGUMENT;
    }

    for (index = 0U; index < 4U; ++index) {
        if (command.value[index] > GOFLY_DSHOT300_MAX_THROTTLE) {
            return GOFLY_E_RANGE;
        }
    }

    for (index = 0U; index < 4U; ++index) {
        frames[index] = gofly_dshot300_packet(command.value[index],
                                              command.telemetry_request[index]);
    }
    return GOFLY_OK;
}
