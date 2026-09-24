#include <stdbool.h>
#include <stdint.h>

#include "../../Drivers/Dshot300/dshot300.h"

static void gofly_dshot_check(bool condition, unsigned *failures)
{
    if (!condition) {
        ++(*failures);
    }
}

static uint16_t gofly_dshot_expected(uint16_t throttle, bool telemetry)
{
    const uint16_t packet = (uint16_t)((throttle << 5U) |
                                       (telemetry ? 0x10U : 0U));
    const uint16_t crc = (uint16_t)((packet ^ (packet >> 4U) ^
                                     (packet >> 8U)) & 0x0FU);
    return (uint16_t)((packet << 4U) | crc);
}

int gofly_test_dshot300_run(void)
{
    unsigned failures = 0U;
    gofly_motor_command_t command = {0};
    uint16_t frames[4] = {0U};

    gofly_dshot_check(gofly_dshot300_encode(0U, false) ==
                      gofly_dshot_expected(0U, false), &failures);
    gofly_dshot_check(gofly_dshot300_encode(1U, true) ==
                      gofly_dshot_expected(1U, true), &failures);
    gofly_dshot_check(gofly_dshot300_encode(2047U, false) ==
                      gofly_dshot_expected(2047U, false), &failures);
    gofly_dshot_check(gofly_dshot300_encode(2048U, false) == 0U, &failures);

    command.value[0U] = 0U;
    command.value[1U] = 1U;
    command.value[2U] = 2047U;
    command.value[3U] = 2U;
    command.telemetry_request[0U] = false;
    command.telemetry_request[1U] = true;
    command.telemetry_request[2U] = false;
    command.telemetry_request[3U] = true;
    gofly_dshot_check(gofly_dshot300_build(command, frames) == GOFLY_OK,
                      &failures);
    gofly_dshot_check(frames[0U] == gofly_dshot_expected(0U, false), &failures);
    gofly_dshot_check(frames[1U] == gofly_dshot_expected(1U, true), &failures);
    gofly_dshot_check(frames[2U] == gofly_dshot_expected(2047U, false), &failures);
    gofly_dshot_check(frames[3U] == gofly_dshot_expected(2U, true), &failures);

    command.value[3U] = 2048U;
    gofly_dshot_check(gofly_dshot300_build(command, frames) == GOFLY_E_RANGE,
                      &failures);
    gofly_dshot_check(gofly_dshot300_build(command, NULL) == GOFLY_E_ARGUMENT,
                      &failures);
    return (int)failures;
}
