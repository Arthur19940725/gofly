#ifndef GOFLY_DRIVER_CRSF_PARSER_H
#define GOFLY_DRIVER_CRSF_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "state_types.h"

#define GOFLY_CRSF_FRAME_BUFFER_SIZE 64U
#define GOFLY_CRSF_RC_ADDRESS 0xC8U
#define GOFLY_CRSF_RC_CHANNELS_TYPE 0x16U
#define GOFLY_CRSF_LINK_STATISTICS_TYPE 0x14U
#define GOFLY_CRSF_RC_CHANNEL_COUNT 16U
#define GOFLY_CRSF_LINK_TIMEOUT_MS 100U

/* CRSF uses an address byte followed by a length byte.  The length counts
 * type, payload, and CRC, so the complete frame is length + 2 bytes. */
typedef struct {
    uint8_t frame[GOFLY_CRSF_FRAME_BUFFER_SIZE];
    uint8_t frame_length;
    uint8_t expected_length;
    bool receiving;
    gofly_rc_input_t input;
    bool input_pending;
    /* No supported CRSF frame carries a universal failsafe bit.  This is
     * supplied by the receiver-specific adapter, not inferred from payloads. */
    bool failsafe;
    bool have_frame;
    bool have_rc_frame;
    uint32_t last_frame_timestamp_ms;
    uint32_t last_rc_timestamp_ms;
    uint32_t checksum_errors;
    uint32_t framing_errors;
    uint32_t accepted_frames;
} gofly_crsf_parser_t;

void gofly_crsf_init(gofly_crsf_parser_t *parser);
size_t gofly_crsf_feed(gofly_crsf_parser_t *parser,
                       const uint8_t *bytes, size_t length,
                       uint32_t timestamp_ms);
bool gofly_crsf_take_input(gofly_crsf_parser_t *parser,
                           gofly_rc_input_t *input);
bool gofly_crsf_link_is_fresh(const gofly_crsf_parser_t *parser,
                              uint32_t now_ms);
void gofly_crsf_set_failsafe(gofly_crsf_parser_t *parser, bool failsafe);
bool gofly_crsf_is_failsafe(const gofly_crsf_parser_t *parser);
uint8_t gofly_crsf_crc8_dvb_s2(const uint8_t *bytes, size_t length);

#ifdef __cplusplus
}
#endif

#endif
