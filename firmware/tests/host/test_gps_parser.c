#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../../Drivers/Gps/gps_parser.h"

static void gofly_gps_check(bool condition, unsigned *failures)
{
    if (!condition) {
        ++(*failures);
    }
}

static void gofly_gps_put_u32(uint8_t *bytes, size_t offset, uint32_t value)
{
    bytes[offset + 0U] = (uint8_t)value;
    bytes[offset + 1U] = (uint8_t)(value >> 8U);
    bytes[offset + 2U] = (uint8_t)(value >> 16U);
    bytes[offset + 3U] = (uint8_t)(value >> 24U);
}

static void gofly_gps_put_i32(uint8_t *bytes, size_t offset, int32_t value)
{
    gofly_gps_put_u32(bytes, offset, (uint32_t)value);
}

static void gofly_gps_rechecksum_ubx(uint8_t *frame)
{
    const uint16_t payload_length = (uint16_t)frame[4U] |
                                    (uint16_t)((uint16_t)frame[5U] << 8U);
    uint8_t checksum_b;
    const uint8_t checksum_a = gofly_gps_ubx_checksum(
        &frame[2U], 4U + (size_t)payload_length, &checksum_b);

    frame[6U + payload_length] = checksum_a;
    frame[7U + payload_length] = checksum_b;
}

static size_t gofly_gps_make_ubx(uint8_t *frame, size_t capacity,
                                 bool corrupt_checksum,
                                 bool corrupt_length)
{
    const uint16_t payload_length = corrupt_length ? 91U : 92U;
    const size_t frame_length = (size_t)payload_length + 8U;
    uint8_t checksum_b;
    uint8_t checksum_a;

    if (frame == NULL || capacity < 100U) {
        return 0U;
    }
    memset(frame, 0, 100U);
    frame[0U] = 0xB5U;
    frame[1U] = 0x62U;
    frame[2U] = GOFLY_GPS_UBX_CLASS_NAV;
    frame[3U] = GOFLY_GPS_UBX_ID_NAV_PVT;
    frame[4U] = (uint8_t)payload_length;
    frame[5U] = 0U;
    frame[6U + 20U] = 3U;
    frame[6U + 21U] = 1U;
    frame[6U + 23U] = 10U;
    gofly_gps_put_i32(frame, 6U + 24U, 115166667);
    gofly_gps_put_i32(frame, 6U + 28U, 481173000);
    gofly_gps_put_i32(frame, 6U + 36U, 545400);
    gofly_gps_put_u32(frame, 6U + 40U, 1200U);
    gofly_gps_put_i32(frame, 6U + 60U, 11534);
    gofly_gps_put_i32(frame, 6U + 64U, 1234567);
    checksum_a = gofly_gps_ubx_checksum(&frame[2U],
                                       4U + (size_t)payload_length,
                                       &checksum_b);
    frame[6U + payload_length] =
        (uint8_t)(corrupt_checksum ? checksum_a ^ 0x01U : checksum_a);
    frame[7U + payload_length] = checksum_b;
    return frame_length;
}

static size_t gofly_gps_copy_string(uint8_t *destination, size_t capacity,
                                    const char *source)
{
    size_t length = strlen(source);
    if (length > capacity) {
        return 0U;
    }
    memcpy(destination, source, length);
    return length;
}

static uint8_t gofly_gps_nmea_checksum(const uint8_t *line, size_t length)
{
    uint8_t checksum = 0U;
    size_t index;

    for (index = 1U; index < length && line[index] != (uint8_t)'*'; ++index) {
        checksum ^= line[index];
    }
    return checksum;
}

static void gofly_gps_rechecksum_nmea(uint8_t *line, size_t length)
{
    const uint8_t checksum = gofly_gps_nmea_checksum(line, length);
    static const char hex[] = "0123456789ABCDEF";
    const size_t star = length - 5U;

    line[star + 1U] = (uint8_t)hex[checksum >> 4U];
    line[star + 2U] = (uint8_t)hex[checksum & 0x0FU];
}

int gofly_test_gps_parser_run(void)
{
    static const char gga[] =
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    static const char rmc[] =
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
    unsigned failures = 0U;
    gofly_gps_parser_t parser;
    gofly_gps_solution_t solution;
    uint8_t frame[128] = {0U};
    uint8_t stream[160] = {0U};
    size_t length;

    length = gofly_gps_make_ubx(frame, sizeof(frame), false, false);
    gofly_gps_parser_init(&parser);
    stream[0U] = 0x55U;
    memcpy(&stream[1U], frame, length);
    gofly_gps_check(gofly_gps_parser_feed_at(&parser, stream, length + 1U,
                                             100U) == length + 1U, &failures);
    gofly_gps_check(gofly_gps_parser_take(&parser, &solution), &failures);
    gofly_gps_check(solution.latitude_e7 == 481173000 &&
                    solution.longitude_e7 == 115166667 &&
                    solution.altitude_mm == 545400 &&
                    solution.ground_speed_mm_s == 11534U &&
                    solution.track_deg_e5 == 1234567U &&
                    solution.horizontal_accuracy_mm == 1200U &&
                    solution.fix_type == 3U && solution.satellites == 10U &&
                    solution.valid, &failures);
    gofly_gps_check(parser.ubx_frames == 1U && parser.accepted_frames == 1U,
                    &failures);
    gofly_gps_check(gofly_gps_parser_is_fresh(&parser, 1100U), &failures);
    gofly_gps_check(!gofly_gps_parser_is_fresh(&parser, 1101U), &failures);

    gofly_gps_parser_init(&parser);
    length = gofly_gps_make_ubx(frame, sizeof(frame), false, false);
    gofly_gps_parser_feed_at(&parser, frame, 7U, 200U);
    gofly_gps_parser_feed_at(&parser, &frame[7U], length - 7U, 200U);
    gofly_gps_check(gofly_gps_parser_take(&parser, &solution), &failures);

    gofly_gps_parser_init(&parser);
    length = gofly_gps_make_ubx(frame, sizeof(frame), true, false);
    gofly_gps_parser_feed_at(&parser, frame, length, 300U);
    gofly_gps_check(parser.checksum_errors == 1U &&
                    !gofly_gps_parser_take(&parser, &solution), &failures);

    gofly_gps_parser_init(&parser);
    length = gofly_gps_make_ubx(frame, sizeof(frame), false, true);
    gofly_gps_parser_feed_at(&parser, frame, length, 400U);
    gofly_gps_check(parser.framing_errors == 1U &&
                    !gofly_gps_parser_take(&parser, &solution), &failures);

    gofly_gps_parser_init(&parser);
    length = gofly_gps_copy_string(stream, sizeof(stream), gga);
    gofly_gps_parser_feed_at(&parser, stream, length, 500U);
    gofly_gps_check(gofly_gps_parser_take(&parser, &solution), &failures);
    gofly_gps_check(solution.latitude_e7 == 481173000 &&
                    solution.longitude_e7 == 115166667 &&
                    solution.altitude_mm == 545400 &&
                    solution.horizontal_accuracy_mm == 900U &&
                    solution.fix_type == 1U &&
                    solution.satellites == 8U && solution.valid, &failures);

    gofly_gps_parser_init(&parser);
    length = gofly_gps_copy_string(stream, sizeof(stream), rmc);
    gofly_gps_parser_feed_at(&parser, stream, length, 600U);
    gofly_gps_check(gofly_gps_parser_take(&parser, &solution), &failures);
    /* 22.4 knots = 11,523.5456 mm/s, rounded to 11,524 mm/s. */
    gofly_gps_check(solution.ground_speed_mm_s == 11524U &&
                    solution.track_deg_e5 == 8440000U && solution.valid,
                    &failures);

    gofly_gps_parser_init(&parser);
    length = gofly_gps_copy_string(stream, sizeof(stream), gga);
    stream[length - 4U] = (uint8_t)'0';
    gofly_gps_parser_feed_at(&parser, stream, length, 700U);
    gofly_gps_check(parser.checksum_errors == 1U &&
                    !gofly_gps_parser_take(&parser, &solution), &failures);

    gofly_gps_parser_init(&parser);
    length = gofly_gps_copy_string(stream, sizeof(stream), gga);
    stream[20U] = (uint8_t)'X';
    gofly_gps_rechecksum_nmea(stream, length);
    gofly_gps_parser_feed_at(&parser, stream, length, 800U);
    gofly_gps_check(parser.framing_errors == 1U &&
                    parser.checksum_errors == 0U &&
                    !gofly_gps_parser_take(&parser, &solution), &failures);

    gofly_gps_parser_init(&parser);
    length = gofly_gps_copy_string(stream, sizeof(stream), gga);
    stream[18U] = (uint8_t)'0';
    gofly_gps_rechecksum_nmea(stream, length);
    gofly_gps_parser_feed_at(&parser, stream, length, 850U);
    gofly_gps_check(parser.framing_errors == 1U &&
                    !gofly_gps_parser_take(&parser, &solution), &failures);

    gofly_gps_parser_init(&parser);
    length = gofly_gps_make_ubx(frame, sizeof(frame), false, false);
    gofly_gps_parser_feed_at(&parser, frame, length, 1000U);
    gofly_gps_parser_take(&parser, &solution);
    length = gofly_gps_copy_string(stream, sizeof(stream), rmc);
    gofly_gps_parser_feed_at(&parser, stream, length, 999U);
    gofly_gps_check(!gofly_gps_parser_take(&parser, &solution), &failures);

    gofly_gps_parser_init(&parser);
    {
        uint8_t recovery[256] = {0U};
        size_t malformed_length = 20U;
        size_t valid_length;

        valid_length = gofly_gps_make_ubx(frame, sizeof(frame), false, false);
        recovery[0U] = 0xB5U;
        recovery[1U] = 0x62U;
        recovery[2U] = GOFLY_GPS_UBX_CLASS_NAV;
        recovery[3U] = GOFLY_GPS_UBX_ID_NAV_PVT;
        recovery[4U] = 92U;
        recovery[5U] = 0U;
        memcpy(&recovery[malformed_length], frame, valid_length);
        gofly_gps_check(gofly_gps_parser_feed_at(
                            &parser, recovery, malformed_length + 10U, 1100U) ==
                            malformed_length + 10U,
                        &failures);
        gofly_gps_check(gofly_gps_parser_feed_at(
                            &parser, &recovery[malformed_length + 10U],
                            valid_length - 10U, 1100U) == valid_length - 10U,
                        &failures);
        gofly_gps_check(gofly_gps_parser_take(&parser, &solution), &failures);
        gofly_gps_check(solution.valid && solution.latitude_e7 == 481173000,
                        &failures);
        gofly_gps_check(parser.checksum_errors == 1U, &failures);
    }

    gofly_gps_parser_init(&parser);
    {
        const size_t nmea_length = gofly_gps_copy_string(stream, sizeof(stream),
                                                         gga);
        uint8_t no_fix_ubx[128] = {0U};
        const size_t no_fix_length = gofly_gps_make_ubx(no_fix_ubx,
                                                        sizeof(no_fix_ubx),
                                                        false, false);

        gofly_gps_parser_feed_at(&parser, stream, nmea_length, 1150U);
        gofly_gps_check(gofly_gps_parser_take(&parser, &solution), &failures);
        gofly_gps_check(solution.valid && solution.latitude_e7 == 481173000,
                        &failures);

        no_fix_ubx[6U + 20U] = 1U;
        no_fix_ubx[6U + 21U] = 0U;
        gofly_gps_rechecksum_ubx(no_fix_ubx);
        gofly_gps_parser_feed_at(&parser, no_fix_ubx, no_fix_length, 1151U);
        gofly_gps_check(gofly_gps_parser_take(&parser, &solution), &failures);
        gofly_gps_check(!solution.valid && !parser.have_ubx_solution,
                        &failures);

        gofly_gps_parser_feed_at(&parser, stream, nmea_length, 1152U);
        gofly_gps_check(gofly_gps_parser_take(&parser, &solution), &failures);
        gofly_gps_check(solution.valid && parser.have_ubx_solution == false,
                        &failures);
    }

    gofly_gps_parser_init(&parser);
    {
        uint8_t invalid_ubx[128] = {0U};
        size_t invalid_length = gofly_gps_make_ubx(invalid_ubx,
                                                   sizeof(invalid_ubx),
                                                   false, false);
        const size_t nmea_length = gofly_gps_copy_string(stream, sizeof(stream),
                                                         gga);
        invalid_ubx[6U + 21U] = 0U;
        gofly_gps_rechecksum_ubx(invalid_ubx);
        gofly_gps_parser_feed_at(&parser, invalid_ubx, invalid_length, 1200U);
        gofly_gps_parser_feed_at(&parser, stream, nmea_length, 1201U);
        gofly_gps_check(gofly_gps_parser_take(&parser, &solution), &failures);
        gofly_gps_check(solution.valid && parser.have_ubx_solution == false,
                        &failures);
    }
    return (int)failures;
}
