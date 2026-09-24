#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../../Drivers/EscTelemetry/esc_telemetry.h"

static void gofly_esc_check(bool condition, unsigned *failures)
{
    if (!condition) {
        ++(*failures);
    }
}

static size_t gofly_esc_make_frame(uint8_t *frame, size_t capacity,
                                   bool corrupt_crc)
{
    uint16_t crc;

    if (frame == NULL || capacity < GOFLY_ESC_TELEMETRY_FRAME_LENGTH) {
        return 0U;
    }
    frame[0U] = 0x34U;
    frame[1U] = 0x12U;
    frame[2U] = 0x78U;
    frame[3U] = 0x05U;
    frame[4U] = 0xBCU;
    frame[5U] = 0x02U;
    frame[6U] = 25U;
    frame[7U] = 0U;
    crc = gofly_esc_telemetry_crc16(frame, 8U);
    if (corrupt_crc) {
        crc ^= 1U;
    }
    frame[8U] = (uint8_t)crc;
    frame[9U] = (uint8_t)(crc >> 8U);
    return GOFLY_ESC_TELEMETRY_FRAME_LENGTH;
}

int gofly_test_esc_telemetry_run(void)
{
    unsigned failures = 0U;
    gofly_esc_telemetry_parser_t parser;
    gofly_esc_telemetry_record_t record;
    uint8_t frame[GOFLY_ESC_TELEMETRY_FRAME_LENGTH];
    size_t length;

    gofly_esc_telemetry_init(&parser);
    length = gofly_esc_make_frame(frame, sizeof(frame), false);
    gofly_esc_check(gofly_esc_telemetry_feed(&parser, frame, 4U, 100U) == 4U,
                    &failures);
    gofly_esc_check(gofly_esc_telemetry_feed(&parser, &frame[4U], length - 4U,
                                              100U) == length - 4U, &failures);
    gofly_esc_check(gofly_esc_telemetry_take(&parser, &record), &failures);
    gofly_esc_check(record.rpm == 0x1234U && record.voltage_mv == 0x0578U &&
                    record.current_ma == 0x02BCU && record.temperature_c == 25,
                    &failures);
    gofly_esc_check(gofly_esc_telemetry_is_fresh(&parser, 200U), &failures);
    gofly_esc_check(!gofly_esc_telemetry_is_fresh(&parser, 201U), &failures);
    gofly_esc_check(gofly_esc_telemetry_read(&parser, 201U, &record) &&
                    record.stale && !record.valid, &failures);

    gofly_esc_telemetry_init(&parser);
    length = gofly_esc_make_frame(frame, sizeof(frame), true);
    gofly_esc_telemetry_feed(&parser, frame, length, 1U);
    gofly_esc_check(parser.checksum_errors == 1U, &failures);
    gofly_esc_check(!gofly_esc_telemetry_take(&parser, &record), &failures);

    gofly_esc_telemetry_init(&parser);
    gofly_esc_telemetry_feed(&parser, frame, 9U, 1U);
    gofly_esc_check(!gofly_esc_telemetry_take(&parser, &record), &failures);
    gofly_esc_check(parser.frame_length == 9U, &failures);
    return (int)failures;
}
