#include "crsf_parser.h"

#include <string.h>

#define GOFLY_CRSF_NO_RESYNC_INDEX 0xFFU

static uint16_t gofly_crsf_channel(const uint8_t *payload, uint8_t channel)
{
    const uint16_t bit_offset = (uint16_t)channel * 11U;
    const uint8_t byte_offset = (uint8_t)(bit_offset / 8U);
    const uint8_t shift = (uint8_t)(bit_offset % 8U);
    uint32_t packed = (uint32_t)payload[byte_offset] |
                      ((uint32_t)payload[byte_offset + 1U] << 8U);

    if ((uint16_t)byte_offset + 2U < 22U) {
        packed |= (uint32_t)payload[byte_offset + 2U] << 16U;
    }
    return (uint16_t)((packed >> shift) & 0x07FFU);
}

static uint16_t gofly_crsf_normalize_throttle(uint16_t value)
{
    const uint32_t min_value = 172U;
    const uint32_t max_value = 1811U;
    uint32_t bounded = value;

    if (bounded < min_value) {
        bounded = min_value;
    } else if (bounded > max_value) {
        bounded = max_value;
    }
    return (uint16_t)(((bounded - min_value) * 2047U +
                       ((max_value - min_value) / 2U)) /
                      (max_value - min_value));
}

static int16_t gofly_crsf_normalize(uint16_t value)
{
    /* CRSF nominal endpoints are 172 and 1811; clamp malformed-but-checked
     * channel values before converting to the signed control range. */
    const int32_t min_value = 172;
    const int32_t max_value = 1811;
    int32_t bounded = (int32_t)value;
    int32_t scaled;

    if (bounded < min_value) {
        bounded = min_value;
    } else if (bounded > max_value) {
        bounded = max_value;
    }
    scaled = ((bounded - min_value) * 2000) / (max_value - min_value) - 1000;
    if (scaled < -1000) {
        scaled = -1000;
    } else if (scaled > 1000) {
        scaled = 1000;
    }
    return (int16_t)scaled;
}

uint8_t gofly_crsf_crc8_dvb_s2(const uint8_t *bytes, size_t length)
{
    uint8_t crc = 0U;
    size_t index;
    uint8_t bit;

    if (bytes == NULL && length != 0U) {
        return 0U;
    }
    for (index = 0U; index < length; ++index) {
        crc ^= bytes[index];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (uint8_t)((crc & 0x80U) != 0U ?
                            (uint8_t)((crc << 1U) ^ 0xD5U) :
                            (uint8_t)(crc << 1U));
        }
    }
    return crc;
}

static void gofly_crsf_reset_frame(gofly_crsf_parser_t *parser)
{
    parser->frame_length = 0U;
    parser->expected_length = 0U;
    parser->receiving = false;
}

static void gofly_crsf_start_frame(gofly_crsf_parser_t *parser)
{
    parser->frame[0U] = GOFLY_CRSF_RC_ADDRESS;
    parser->frame_length = 1U;
    parser->expected_length = 0U;
    parser->receiving = true;
}

static bool gofly_crsf_candidate_crc_valid(
    const gofly_crsf_parser_t *parser, uint8_t index)
{
    const uint8_t length = parser->frame[index + 1U];
    const size_t complete_length = (size_t)length + 2U;
    const size_t end = (size_t)index + complete_length;
    uint8_t received_crc;
    uint8_t calculated_crc;

    if (parser->frame[index] != GOFLY_CRSF_RC_ADDRESS ||
        length < 2U || length > GOFLY_CRSF_FRAME_BUFFER_SIZE - 2U ||
        end > parser->frame_length) {
        return false;
    }
    received_crc = parser->frame[end - 1U];
    calculated_crc = gofly_crsf_crc8_dvb_s2(
        &parser->frame[index + 2U], (size_t)length - 1U);
    return received_crc == calculated_crc;
}

static uint8_t gofly_crsf_find_resync_index(
    const gofly_crsf_parser_t *parser)
{
    uint8_t fallback = GOFLY_CRSF_NO_RESYNC_INDEX;
    uint8_t index;

    for (index = 1U; (size_t)index + 1U < parser->frame_length; ++index) {
        const uint8_t length = parser->frame[index + 1U];

        if (parser->frame[index] != GOFLY_CRSF_RC_ADDRESS ||
            length < 2U || length > GOFLY_CRSF_FRAME_BUFFER_SIZE - 2U) {
            continue;
        }
        if (fallback == GOFLY_CRSF_NO_RESYNC_INDEX) {
            fallback = index;
        }
        if (gofly_crsf_candidate_crc_valid(parser, index)) {
            return index;
        }
    }
    return fallback;
}

static bool gofly_crsf_try_parse(gofly_crsf_parser_t *parser,
                                 uint32_t timestamp_ms)
{
    const uint8_t length = parser->expected_length;
    const uint8_t type = parser->frame[2U];
    const uint8_t payload_length = (uint8_t)(length - 2U);
    const uint8_t received_crc = parser->frame[(size_t)length + 1U];
    const uint8_t calculated_crc = gofly_crsf_crc8_dvb_s2(&parser->frame[2U],
                                                          (size_t)length - 1U);

    if (received_crc != calculated_crc) {
        ++parser->checksum_errors;
        return false;
    }

    if (type == GOFLY_CRSF_RC_CHANNELS_TYPE && payload_length == 22U) {
        gofly_rc_input_t decoded = {0};
        decoded.throttle = gofly_crsf_normalize_throttle(
            gofly_crsf_channel(&parser->frame[3U], 2U));
        decoded.roll = gofly_crsf_normalize(gofly_crsf_channel(&parser->frame[3U], 0U));
        decoded.pitch = gofly_crsf_normalize(gofly_crsf_channel(&parser->frame[3U], 1U));
        decoded.yaw = gofly_crsf_normalize(gofly_crsf_channel(&parser->frame[3U], 3U));
        decoded.arm_request = gofly_crsf_channel(&parser->frame[3U], 4U) > 1000U;
        decoded.rth_request = gofly_crsf_channel(&parser->frame[3U], 5U) > 1000U;
        decoded.link_valid = !parser->failsafe;
        parser->input = decoded;
        parser->input_pending = true;
        parser->have_frame = true;
        parser->have_rc_frame = true;
        parser->last_frame_timestamp_ms = timestamp_ms;
        parser->last_rc_timestamp_ms = timestamp_ms;
        ++parser->accepted_frames;
        return true;
    }

    if (type == GOFLY_CRSF_LINK_STATISTICS_TYPE && payload_length == 10U) {
        /* CRSF 0x14 is link statistics only; its ten bytes do not contain a
         * failsafe flag.  Link validity is therefore derived from accepted RC
         * frames and the bounded age check below. */
        parser->have_frame = true;
        parser->last_frame_timestamp_ms = timestamp_ms;
        ++parser->accepted_frames;
        return true;
    }

    ++parser->accepted_frames;
    return true;
}

void gofly_crsf_init(gofly_crsf_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
}

size_t gofly_crsf_feed(gofly_crsf_parser_t *parser,
                       const uint8_t *bytes, size_t length,
                       uint32_t timestamp_ms)
{
    uint8_t pending[GOFLY_CRSF_FRAME_BUFFER_SIZE * 3U];
    size_t pending_length = 0U;
    size_t pending_offset = 0U;
    size_t consumed = 0U;

    if (parser == NULL || (bytes == NULL && length != 0U)) {
        return 0U;
    }

    while (consumed < length || pending_offset < pending_length) {
        const bool replaying = pending_offset < pending_length;
        const uint8_t byte = replaying ? pending[pending_offset++] :
                                         bytes[consumed++];

        if (!parser->receiving) {
            if (byte == GOFLY_CRSF_RC_ADDRESS) {
                gofly_crsf_start_frame(parser);
            }
            continue;
        }

        if (parser->frame_length >= GOFLY_CRSF_FRAME_BUFFER_SIZE) {
            ++parser->framing_errors;
            gofly_crsf_reset_frame(parser);
            if (byte == GOFLY_CRSF_RC_ADDRESS) {
                gofly_crsf_start_frame(parser);
            }
            continue;
        }

        parser->frame[parser->frame_length++] = byte;
        if (parser->frame_length == 2U) {
            if (byte < 2U || byte > GOFLY_CRSF_FRAME_BUFFER_SIZE - 2U) {
                ++parser->framing_errors;
                gofly_crsf_reset_frame(parser);
                if (byte == GOFLY_CRSF_RC_ADDRESS) {
                    gofly_crsf_start_frame(parser);
                }
                continue;
            }
            parser->expected_length = byte;
        } else if (parser->expected_length != 0U &&
                   parser->frame_length == (uint8_t)(parser->expected_length + 2U)) {
            const uint8_t current_length = parser->frame_length;
            const bool parsed = gofly_crsf_try_parse(parser, timestamp_ms);
            const uint8_t candidate = parsed ?
                GOFLY_CRSF_NO_RESYNC_INDEX :
                gofly_crsf_find_resync_index(parser);

            if (!parsed && candidate != GOFLY_CRSF_NO_RESYNC_INDEX &&
                candidate < current_length) {
                const size_t suffix_length = (size_t)current_length - candidate;
                const size_t remaining = pending_length - pending_offset;
                if (suffix_length + remaining <= sizeof(pending)) {
                    memmove(&pending[suffix_length], &pending[pending_offset],
                            remaining);
                    memcpy(pending, &parser->frame[candidate], suffix_length);
                    pending_length = suffix_length + remaining;
                    pending_offset = 0U;
                } else {
                    ++parser->framing_errors;
                }
            }
            gofly_crsf_reset_frame(parser);
        }
    }
    return consumed;
}

bool gofly_crsf_take_input(gofly_crsf_parser_t *parser,
                           gofly_rc_input_t *input)
{
    if (parser == NULL || input == NULL || !parser->input_pending) {
        return false;
    }
    *input = parser->input;
    parser->input_pending = false;
    return true;
}

bool gofly_crsf_link_is_fresh(const gofly_crsf_parser_t *parser,
                              uint32_t now_ms)
{
    if (parser == NULL || !parser->have_rc_frame || parser->failsafe) {
        return false;
    }
    /* Treat the configured timeout as the last fresh instant.  This keeps
     * the boundary consistent with the other timestamped protocol records:
     * age == timeout is still usable, while the next millisecond is stale. */
    return (uint32_t)(now_ms - parser->last_rc_timestamp_ms) <=
           GOFLY_CRSF_LINK_TIMEOUT_MS;
}

void gofly_crsf_set_failsafe(gofly_crsf_parser_t *parser, bool failsafe)
{
    if (parser == NULL) {
        return;
    }
    parser->failsafe = failsafe;
    if (failsafe) {
        parser->input.link_valid = false;
    }
}

bool gofly_crsf_is_failsafe(const gofly_crsf_parser_t *parser)
{
    return parser != NULL && parser->failsafe;
}
