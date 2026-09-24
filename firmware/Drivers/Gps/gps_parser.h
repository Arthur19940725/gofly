#ifndef GOFLY_DRIVER_GPS_PARSER_H
#define GOFLY_DRIVER_GPS_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "state_types.h"

#define GOFLY_GPS_UBX_MAX_PAYLOAD 92U
#define GOFLY_GPS_MAX_FRAME_SIZE 128U
#define GOFLY_GPS_NMEA_MAX_SENTENCE 96U
#define GOFLY_GPS_STALE_TIMEOUT_MS 1000U
#define GOFLY_GPS_UBX_CLASS_NAV 0x01U
#define GOFLY_GPS_UBX_ID_NAV_PVT 0x07U

typedef enum {
    GOFLY_GPS_PARSE_SEARCH = 0,
    GOFLY_GPS_PARSE_UBX_SYNC,
    GOFLY_GPS_PARSE_UBX_FRAME,
    GOFLY_GPS_PARSE_NMEA
} gofly_gps_parse_state_t;

typedef struct {
    uint8_t frame[GOFLY_GPS_MAX_FRAME_SIZE];
    uint16_t frame_length;
    uint16_t expected_length;
    gofly_gps_parse_state_t state;
    uint32_t timestamp_ms;
    gofly_gps_solution_t solution;
    bool solution_pending;
    bool have_solution;
    bool have_ubx_solution;
    uint32_t last_solution_timestamp_ms;
    uint32_t last_ubx_timestamp_ms;
    uint32_t checksum_errors;
    uint32_t framing_errors;
    uint32_t accepted_frames;
    uint32_t ubx_frames;
    uint32_t nmea_frames;
} gofly_gps_parser_t;

void gofly_gps_parser_init(gofly_gps_parser_t *parser);
size_t gofly_gps_parser_feed(gofly_gps_parser_t *parser,
                             const uint8_t *bytes, size_t length);
size_t gofly_gps_parser_feed_at(gofly_gps_parser_t *parser,
                                const uint8_t *bytes, size_t length,
                                uint32_t timestamp_ms);
void gofly_gps_parser_set_timestamp(gofly_gps_parser_t *parser,
                                    uint32_t timestamp_ms);
bool gofly_gps_parser_take(gofly_gps_parser_t *parser,
                           gofly_gps_solution_t *solution);
bool gofly_gps_parser_take_at(gofly_gps_parser_t *parser,
                              uint32_t now_ms,
                              gofly_gps_solution_t *solution);
bool gofly_gps_parser_read(const gofly_gps_parser_t *parser,
                           uint32_t now_ms,
                           gofly_gps_solution_t *solution);
bool gofly_gps_parser_is_fresh(const gofly_gps_parser_t *parser,
                               uint32_t now_ms);
uint8_t gofly_gps_ubx_checksum(const uint8_t *bytes, size_t length,
                               uint8_t *checksum_b);

#ifdef __cplusplus
}
#endif

#endif
