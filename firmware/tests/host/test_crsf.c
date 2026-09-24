#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../../Drivers/Crsf/crsf_parser.h"

static void gofly_crsf_check(bool condition, unsigned *failures)
{
    if (!condition) {
        ++(*failures);
    }
}

static void gofly_crsf_set_channel(uint8_t *payload, uint8_t channel,
                                   uint16_t value)
{
    const uint16_t bit_offset = (uint16_t)channel * 11U;
    const uint8_t byte_offset = (uint8_t)(bit_offset / 8U);
    const uint8_t shift = (uint8_t)(bit_offset % 8U);
    uint32_t packed = (uint32_t)value << shift;
    uint8_t index;

    for (index = 0U; index < 3U &&
         (uint16_t)byte_offset + (uint16_t)index < 22U; ++index) {
        payload[byte_offset + index] =
            (uint8_t)(payload[byte_offset + index] |
                      (uint8_t)(packed >> (8U * index)));
    }
}

static size_t gofly_crsf_make_channels(uint8_t *frame, size_t capacity,
                                       const uint16_t channels[16],
                                       bool corrupt_crc)
{
    uint8_t payload[22] = {0U};
    uint8_t index;
    uint8_t crc;

    if (frame == NULL || channels == NULL || capacity < 26U) {
        return 0U;
    }
    for (index = 0U; index < 16U; ++index) {
        gofly_crsf_set_channel(payload, index, channels[index]);
    }
    frame[0U] = GOFLY_CRSF_RC_ADDRESS;
    frame[1U] = 24U;
    frame[2U] = GOFLY_CRSF_RC_CHANNELS_TYPE;
    memcpy(&frame[3U], payload, sizeof(payload));
    crc = gofly_crsf_crc8_dvb_s2(&frame[2U], 23U);
    frame[25U] = (uint8_t)(corrupt_crc ? crc ^ 0x01U : crc);
    return 26U;
}

static size_t gofly_crsf_make_link_statistics(uint8_t *frame,
                                               size_t capacity)
{
    uint8_t index;
    uint8_t crc;

    if (frame == NULL || capacity < 14U) {
        return 0U;
    }
    frame[0U] = GOFLY_CRSF_RC_ADDRESS;
    frame[1U] = 12U; /* type + ten-byte payload + CRC */
    frame[2U] = GOFLY_CRSF_LINK_STATISTICS_TYPE;
    for (index = 0U; index < 10U; ++index) {
        frame[3U + index] = (uint8_t)(index + 1U);
    }
    crc = gofly_crsf_crc8_dvb_s2(&frame[2U], 11U);
    frame[13U] = crc;
    return 14U;
}
int gofly_test_crsf_run(void)
{
    unsigned failures = 0U;
    gofly_crsf_parser_t parser;
    gofly_rc_input_t input;
    uint8_t frame[26] = {0U};
    uint8_t link_frame[14] = {0U};
    uint8_t stream[64] = {0U};
    uint16_t channels[16];
    size_t index;
    size_t frame_length;

    for (index = 0U; index < 16U; ++index) {
        channels[index] = 992U;
    }
    channels[0U] = 172U;
    channels[1U] = 1811U;
    channels[2U] = 1811U;
    channels[3U] = 992U;
    channels[4U] = 1811U;
    channels[5U] = 172U;
    frame_length = gofly_crsf_make_channels(frame, sizeof(frame), channels, false);

    gofly_crsf_init(&parser);
    stream[0U] = 0x55U;
    memcpy(&stream[1U], frame, frame_length);
    gofly_crsf_check(gofly_crsf_feed(&parser, stream, frame_length + 1U,
                                     10U) == frame_length + 1U, &failures);
    gofly_crsf_check(gofly_crsf_take_input(&parser, &input), &failures);
    gofly_crsf_check(input.roll == -1000 && input.pitch == 1000,
                     &failures);
    gofly_crsf_check(input.throttle == 2047U && input.arm_request &&
                     !input.rth_request && input.link_valid, &failures);
    gofly_crsf_check(gofly_crsf_link_is_fresh(&parser, 110U), &failures);
    gofly_crsf_check(!gofly_crsf_link_is_fresh(&parser, 111U), &failures);

    gofly_crsf_init(&parser);
    frame_length = gofly_crsf_make_channels(frame, sizeof(frame), channels, true);
    gofly_crsf_feed(&parser, frame, frame_length, 20U);
    gofly_crsf_check(!gofly_crsf_take_input(&parser, &input), &failures);
    gofly_crsf_check(parser.checksum_errors == 1U, &failures);

    gofly_crsf_init(&parser);
    gofly_crsf_feed(&parser, frame, 10U, 30U);
    gofly_crsf_check(!gofly_crsf_take_input(&parser, &input), &failures);
    gofly_crsf_check(parser.frame_length != 0U, &failures);

    gofly_crsf_init(&parser);
    frame_length = gofly_crsf_make_channels(frame, sizeof(frame), channels, false);
    gofly_crsf_feed(&parser, frame, 1U, 40U);
    gofly_crsf_feed(&parser, &frame[1U], frame_length - 1U, 40U);
    gofly_crsf_check(gofly_crsf_take_input(&parser, &input), &failures);

    gofly_crsf_init(&parser);
    stream[0U] = GOFLY_CRSF_RC_ADDRESS;
    stream[1U] = 63U;
    memset(&stream[2U], 0, sizeof(stream) - 2U);
    gofly_crsf_feed(&parser, stream, sizeof(stream), 60U);
    gofly_crsf_check(parser.framing_errors == 1U, &failures);
    gofly_crsf_check(parser.frame_length == 0U, &failures);

    gofly_crsf_init(&parser);
    frame_length = gofly_crsf_make_channels(frame, sizeof(frame), channels, false);
    frame[12U] = 0x01U;
    frame[25U] = gofly_crsf_crc8_dvb_s2(&frame[2U], 23U);
    gofly_crsf_feed(&parser, frame, frame_length, 20U);
    gofly_crsf_check(gofly_crsf_take_input(&parser, &input), &failures);
    gofly_crsf_check(input.link_valid, &failures);

    gofly_crsf_init(&parser);
    frame_length = gofly_crsf_make_channels(frame, sizeof(frame), channels, false);
    {
        size_t recovery_length = 42U;
        memset(stream, 0, recovery_length);
        stream[0U] = GOFLY_CRSF_RC_ADDRESS;
        stream[1U] = 40U;
        stream[2U] = 0x99U;
        stream[3U] = GOFLY_CRSF_RC_ADDRESS;
        stream[4U] = 2U;
        stream[5U] = 0x55U;
        stream[6U] = 0x00U;
        memcpy(&stream[8U], frame, frame_length);
        gofly_crsf_feed(&parser, stream, recovery_length, 25U);
        gofly_crsf_check(gofly_crsf_take_input(&parser, &input), &failures);
        gofly_crsf_check(input.throttle == 2047U && input.link_valid,
                         &failures);
        gofly_crsf_check(parser.checksum_errors == 1U, &failures);
    }

    gofly_crsf_init(&parser);
    gofly_crsf_feed(&parser, frame, frame_length, 30U);
    gofly_crsf_check(gofly_crsf_take_input(&parser, &input), &failures);
    gofly_crsf_set_failsafe(&parser, true);
    gofly_crsf_check(gofly_crsf_is_failsafe(&parser) &&
                     !gofly_crsf_link_is_fresh(&parser, 30U), &failures);
    gofly_crsf_feed(&parser, frame, frame_length, 31U);
    gofly_crsf_check(gofly_crsf_take_input(&parser, &input) &&
                     !input.link_valid, &failures);
    gofly_crsf_set_failsafe(&parser, false);
    gofly_crsf_feed(&parser, frame, frame_length, 32U);
    gofly_crsf_check(gofly_crsf_take_input(&parser, &input) &&
                     input.link_valid, &failures);

    gofly_crsf_init(&parser);
    frame_length = gofly_crsf_make_link_statistics(link_frame,
                                                   sizeof(link_frame));
    gofly_crsf_feed(&parser, link_frame, frame_length, 50U);
    gofly_crsf_check(!gofly_crsf_is_failsafe(&parser), &failures);
    gofly_crsf_check(!gofly_crsf_link_is_fresh(&parser, 150U), &failures);
    gofly_crsf_check(!gofly_crsf_link_is_fresh(&parser, 151U), &failures);
    return (int)failures;
}
