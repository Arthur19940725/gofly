#include "esc_telemetry.h"

#include <string.h>

static uint16_t gofly_esc_u16_le(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0U] | ((uint16_t)bytes[1U] << 8U));
}

uint16_t gofly_esc_telemetry_crc16(const uint8_t *bytes, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index;
    uint8_t bit;

    if (bytes == NULL && length != 0U) {
        return 0U;
    }
    for (index = 0U; index < length; ++index) {
        crc ^= (uint16_t)bytes[index] << 8U;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (uint16_t)(((crc & 0x8000U) != 0U) ?
                             (uint16_t)((crc << 1U) ^ 0x1021U) :
                             (uint16_t)(crc << 1U));
        }
    }
    return crc;
}

void gofly_esc_telemetry_init(gofly_esc_telemetry_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
}

static bool gofly_esc_telemetry_parse_frame(
    gofly_esc_telemetry_parser_t *parser, uint32_t timestamp_ms)
{
    const uint16_t expected_crc = gofly_esc_u16_le(&parser->frame[8U]);
    const uint16_t calculated_crc = gofly_esc_telemetry_crc16(parser->frame, 8U);
    gofly_esc_telemetry_record_t record;

    if (expected_crc != calculated_crc) {
        ++parser->checksum_errors;
        return false;
    }

    record.rpm = gofly_esc_u16_le(&parser->frame[0U]);
    record.voltage_mv = gofly_esc_u16_le(&parser->frame[2U]);
    record.current_ma = gofly_esc_u16_le(&parser->frame[4U]);
    record.temperature_c = (int16_t)gofly_esc_u16_le(&parser->frame[6U]);
    record.crc = expected_crc;
    record.timestamp_ms = timestamp_ms;
    record.valid = true;
    record.stale = false;
    parser->newest = record;
    parser->record_pending = true;
    parser->have_record = true;
    ++parser->accepted_frames;
    return true;
}

size_t gofly_esc_telemetry_feed(gofly_esc_telemetry_parser_t *parser,
                                const uint8_t *bytes, size_t length,
                                uint32_t timestamp_ms)
{
    size_t consumed = 0U;

    if (parser == NULL || (bytes == NULL && length != 0U)) {
        return 0U;
    }
    while (consumed < length) {
        parser->frame[parser->frame_length++] = bytes[consumed++];
        if (parser->frame_length == GOFLY_ESC_TELEMETRY_FRAME_LENGTH) {
            (void)gofly_esc_telemetry_parse_frame(parser, timestamp_ms);
            parser->frame_length = 0U;
        } else if (parser->frame_length > GOFLY_ESC_TELEMETRY_FRAME_LENGTH) {
            ++parser->framing_errors;
            parser->frame_length = 0U;
        }
    }
    return consumed;
}

bool gofly_esc_telemetry_take(gofly_esc_telemetry_parser_t *parser,
                              gofly_esc_telemetry_record_t *record)
{
    if (parser == NULL || record == NULL || !parser->record_pending) {
        return false;
    }
    *record = parser->newest;
    parser->record_pending = false;
    return true;
}

bool gofly_esc_telemetry_read(const gofly_esc_telemetry_parser_t *parser,
                              uint32_t now_ms,
                              gofly_esc_telemetry_record_t *record)
{
    uint32_t age;

    if (parser == NULL || record == NULL || !parser->have_record) {
        return false;
    }
    *record = parser->newest;
    age = (uint32_t)(now_ms - record->timestamp_ms);
    record->stale = age > GOFLY_ESC_TELEMETRY_TIMEOUT_MS;
    record->valid = !record->stale;
    return true;
}

bool gofly_esc_telemetry_is_fresh(
    const gofly_esc_telemetry_parser_t *parser, uint32_t now_ms)
{
    gofly_esc_telemetry_record_t record;
    return gofly_esc_telemetry_read(parser, now_ms, &record) && record.valid;
}
