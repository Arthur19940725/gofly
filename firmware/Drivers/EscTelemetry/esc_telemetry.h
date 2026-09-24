#ifndef GOFLY_DRIVER_ESC_TELEMETRY_H
#define GOFLY_DRIVER_ESC_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "status.h"
#include "flight_config.h"

#define GOFLY_ESC_TELEMETRY_FRAME_LENGTH 10U

/*
 * Fixed wire format used by this protocol boundary (little endian):
 *   rpm:u16, voltage_mV:u16, current_mA:u16, temperature_C:i16,
 *   crc16:u16.  The CRC covers the first eight bytes and is CRC-16/CCITT
 *   (poly 0x1021, init 0xffff), transmitted low byte first.
 *
 * The parser deliberately exposes the wire units directly.  No runtime
 * divisor or ESC-specific scaling is silently guessed here.
 */
typedef struct {
    uint16_t rpm;
    uint16_t voltage_mv;
    uint16_t current_ma;
    int16_t temperature_c;
    uint16_t crc;
    uint32_t timestamp_ms;
    bool valid;
    bool stale;
} gofly_esc_telemetry_record_t;

typedef struct {
    uint8_t frame[GOFLY_ESC_TELEMETRY_FRAME_LENGTH];
    uint8_t frame_length;
    gofly_esc_telemetry_record_t newest;
    bool record_pending;
    bool have_record;
    uint32_t checksum_errors;
    uint32_t framing_errors;
    uint32_t accepted_frames;
} gofly_esc_telemetry_parser_t;

typedef gofly_esc_telemetry_parser_t gofly_esc_parser_t;
typedef gofly_esc_telemetry_record_t gofly_esc_telemetry_t;

void gofly_esc_telemetry_init(gofly_esc_telemetry_parser_t *parser);
size_t gofly_esc_telemetry_feed(gofly_esc_telemetry_parser_t *parser,
                                const uint8_t *bytes, size_t length,
                                uint32_t timestamp_ms);
bool gofly_esc_telemetry_take(gofly_esc_telemetry_parser_t *parser,
                              gofly_esc_telemetry_record_t *record);
bool gofly_esc_telemetry_read(const gofly_esc_telemetry_parser_t *parser,
                              uint32_t now_ms,
                              gofly_esc_telemetry_record_t *record);
bool gofly_esc_telemetry_is_fresh(
    const gofly_esc_telemetry_parser_t *parser, uint32_t now_ms);
uint16_t gofly_esc_telemetry_crc16(const uint8_t *bytes, size_t length);

#ifdef __cplusplus
}
#endif

#endif
