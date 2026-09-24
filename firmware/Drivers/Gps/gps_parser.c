#include "gps_parser.h"

#include <string.h>

#define GOFLY_GPS_UBX_HEADER_LENGTH 4U
#define GOFLY_GPS_UBX_OVERHEAD 8U
#define GOFLY_GPS_NMEA_FIELD_CAPACITY 20U
#define GOFLY_GPS_NO_RESYNC_INDEX UINT16_MAX
#define GOFLY_GPS_PENDING_BUFFER_SIZE (GOFLY_GPS_MAX_FRAME_SIZE * 3U)
#define GOFLY_GPS_E7_PER_DEGREE 10000000U
#define GOFLY_GPS_MMS_PER_KNOT_NUMERATOR 514444U
#define GOFLY_GPS_MMS_PER_KNOT_DENOMINATOR 1000000U
/* NMEA HDOP is a dimensionless dilution metric, not a distance.  The
 * shared state type has no separate DOP field, so parse_fixed()'s three
 * fractional digits are retained as milli-DOP in the legacy millimetre slot.
 * UBX NAV-PVT supplies actual millimetres and remains authoritative whenever
 * available. */

typedef struct {
    const uint8_t *start;
    size_t length;
} gofly_gps_field_t;

static bool gofly_gps_time_after(uint32_t left, uint32_t right)
{
    return (int32_t)(left - right) > 0;
}

static uint16_t gofly_gps_read_u16_le(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0U] |
                      ((uint16_t)bytes[1U] << 8U));
}

static uint32_t gofly_gps_read_u32_le(const uint8_t *bytes)
{
    return (uint32_t)bytes[0U] |
           ((uint32_t)bytes[1U] << 8U) |
           ((uint32_t)bytes[2U] << 16U) |
           ((uint32_t)bytes[3U] << 24U);
}

static int32_t gofly_gps_read_i32_le(const uint8_t *bytes)
{
    return (int32_t)gofly_gps_read_u32_le(bytes);
}

static bool gofly_gps_parse_uint(const uint8_t *text, size_t length,
                                 uint32_t maximum, uint32_t *value)
{
    uint32_t result = 0U;
    size_t index;

    if (text == NULL || value == NULL || length == 0U) {
        return false;
    }
    for (index = 0U; index < length; ++index) {
        uint8_t digit;
        uint32_t next;
        if (text[index] < (uint8_t)'0' || text[index] > (uint8_t)'9') {
            return false;
        }
        digit = (uint8_t)(text[index] - (uint8_t)'0');
        if (result > (maximum - (uint32_t)digit) / 10U) {
            return false;
        }
        next = result * 10U + (uint32_t)digit;
        result = next;
    }
    *value = result;
    return true;
}

static bool gofly_gps_parse_fixed(const uint8_t *text, size_t length,
                                  uint32_t fractional_digits,
                                  bool allow_sign, int64_t *value)
{
    size_t index = 0U;
    bool negative = false;
    bool saw_integer = false;
    bool saw_fraction = false;
    uint64_t integer_part = 0U;
    uint64_t fraction_part = 0U;
    uint32_t fraction_count = 0U;
    uint64_t scale = 1U;

    if (text == NULL || value == NULL || length == 0U ||
        fractional_digits > 6U) {
        return false;
    }
    if (allow_sign && (text[0U] == (uint8_t)'-' ||
                       text[0U] == (uint8_t)'+')) {
        negative = text[0U] == (uint8_t)'-';
        index = 1U;
        if (index == length) {
            return false;
        }
    }
    while (index < length && text[index] >= (uint8_t)'0' &&
           text[index] <= (uint8_t)'9') {
        const uint64_t digit = (uint64_t)(text[index] - (uint8_t)'0');
        if (integer_part > (UINT64_MAX - digit) / 10U) {
            return false;
        }
        integer_part = integer_part * 10U + digit;
        saw_integer = true;
        ++index;
    }
    if (index < length && text[index] == (uint8_t)'.') {
        ++index;
        while (index < length && text[index] >= (uint8_t)'0' &&
               text[index] <= (uint8_t)'9') {
            const uint64_t digit = (uint64_t)(text[index] - (uint8_t)'0');
            if (fraction_count >= fractional_digits) {
                return false;
            }
            fraction_part = fraction_part * 10U + digit;
            ++fraction_count;
            saw_fraction = true;
            ++index;
        }
    }
    if (index != length || (!saw_integer && !saw_fraction)) {
        return false;
    }
    for (index = 0U; index < fractional_digits; ++index) {
        scale *= 10U;
    }
    for (; fraction_count < fractional_digits; ++fraction_count) {
        fraction_part *= 10U;
    }
    if (integer_part > (UINT64_MAX - fraction_part) / scale) {
        return false;
    }
    {
        const uint64_t combined = integer_part * scale + fraction_part;
        if (combined > (uint64_t)INT64_MAX) {
            return false;
        }
        *value = negative ? -(int64_t)combined : (int64_t)combined;
    }
    return true;
}

static bool gofly_gps_parse_latitude(const gofly_gps_field_t *coordinate,
                                     const gofly_gps_field_t *direction,
                                     bool longitude, int32_t *result)
{
    size_t dot = 0U;
    bool has_dot = false;
    size_t index;
    int64_t scaled_minutes;
    uint32_t degrees = 0U;
    uint64_t e7;
    uint8_t expected_degrees = longitude ? 3U : 2U;
    uint8_t direction_byte;

    if (coordinate == NULL || direction == NULL || result == NULL ||
        coordinate->start == NULL || direction->start == NULL ||
        direction->length != 1U) {
        return false;
    }
    for (index = 0U; index < coordinate->length; ++index) {
        if (coordinate->start[index] == (uint8_t)'.') {
            if (has_dot) {
                return false;
            }
            dot = index;
            has_dot = true;
        }
    }
    if (!has_dot || dot != (size_t)expected_degrees + 2U) {
        return false;
    }
    if (!gofly_gps_parse_uint(coordinate->start, dot - 2U,
                              longitude ? 180U : 90U, &degrees)) {
        return false;
    }
    if (!gofly_gps_parse_fixed(&coordinate->start[dot - 2U],
                               coordinate->length - (dot - 2U), 6U,
                               false, &scaled_minutes)) {
        return false;
    }
    if (scaled_minutes < 0 || scaled_minutes >= 60000000LL) {
        return false;
    }
    e7 = (uint64_t)degrees * (uint64_t)GOFLY_GPS_E7_PER_DEGREE;
    e7 += ((uint64_t)scaled_minutes * (uint64_t)GOFLY_GPS_E7_PER_DEGREE +
           30000000U) / 60000000U;
    if ((!longitude && e7 > 900000000U) ||
        (longitude && e7 > 1800000000U)) {
        return false;
    }
    direction_byte = direction->start[0U];
    if ((!longitude && direction_byte != (uint8_t)'N' &&
         direction_byte != (uint8_t)'S') ||
        (longitude && direction_byte != (uint8_t)'E' &&
         direction_byte != (uint8_t)'W')) {
        return false;
    }
    *result = (int32_t)e7;
    if (direction_byte == (uint8_t)'S' || direction_byte == (uint8_t)'W') {
        *result = -*result;
    }
    return true;
}

static bool gofly_gps_parse_nmea_fields(const uint8_t *line, size_t length,
                                        gofly_gps_field_t fields[],
                                        size_t *field_count)
{
    size_t count = 0U;
    size_t start = 0U;
    size_t index;

    if (line == NULL || fields == NULL || field_count == NULL || length < 2U ||
        line[0U] != (uint8_t)'$') {
        return false;
    }
    for (index = 0U; index <= length; ++index) {
        if (index == length || line[index] == (uint8_t)',') {
            if (count >= GOFLY_GPS_NMEA_FIELD_CAPACITY) {
                return false;
            }
            fields[count].start = &line[start];
            fields[count].length = index - start;
            ++count;
            start = index + 1U;
        }
    }
    *field_count = count;
    return true;
}

static int gofly_gps_hex(uint8_t value)
{
    if (value >= (uint8_t)'0' && value <= (uint8_t)'9') {
        return (int)(value - (uint8_t)'0');
    }
    if (value >= (uint8_t)'A' && value <= (uint8_t)'F') {
        return (int)(value - (uint8_t)'A') + 10;
    }
    if (value >= (uint8_t)'a' && value <= (uint8_t)'f') {
        return (int)(value - (uint8_t)'a') + 10;
    }
    return -1;
}

uint8_t gofly_gps_ubx_checksum(const uint8_t *bytes, size_t length,
                               uint8_t *checksum_b)
{
    uint8_t checksum_a = 0U;
    uint8_t checksum_second = 0U;
    size_t index;

    if (bytes == NULL && length != 0U) {
        if (checksum_b != NULL) {
            *checksum_b = 0U;
        }
        return 0U;
    }
    for (index = 0U; index < length; ++index) {
        checksum_a = (uint8_t)(checksum_a + bytes[index]);
        checksum_second = (uint8_t)(checksum_second + checksum_a);
    }
    if (checksum_b != NULL) {
        *checksum_b = checksum_second;
    }
    return checksum_a;
}

static bool gofly_gps_nmea_checksum(const uint8_t *line, size_t length,
                                    size_t *star_position)
{
    size_t star = 0U;
    size_t index;
    uint8_t checksum = 0U;
    int high;
    int low;

    if (line == NULL || length < 7U || line[0U] != (uint8_t)'$') {
        return false;
    }
    for (index = 1U; index < length; ++index) {
        if (line[index] == (uint8_t)'*') {
            star = index;
            break;
        }
        checksum ^= line[index];
    }
    if (star == 0U || star + 2U >= length || star + 3U != length) {
        return false;
    }
    high = gofly_gps_hex(line[star + 1U]);
    low = gofly_gps_hex(line[star + 2U]);
    if (high < 0 || low < 0 ||
        checksum != (uint8_t)((high << 4) | low)) {
        return false;
    }
    if (star_position != NULL) {
        *star_position = star;
    }
    return true;
}

static bool gofly_gps_parse_gga(const gofly_gps_field_t fields[],
                                size_t field_count,
                                gofly_gps_solution_t *solution)
{
    uint32_t quality;
    uint32_t satellites;
    int64_t altitude_mm_scaled;
    int64_t hdop_scaled;
    int32_t latitude;
    int32_t longitude;
    gofly_gps_field_t coordinate;
    gofly_gps_field_t direction;

    if (field_count < 11U || fields[0U].length < 3U ||
        fields[0U].start[fields[0U].length - 3U] != (uint8_t)'G' ||
        fields[0U].start[fields[0U].length - 2U] != (uint8_t)'G' ||
        fields[0U].start[fields[0U].length - 1U] != (uint8_t)'A' ||
        fields[10U].length != 1U || fields[10U].start[0U] != (uint8_t)'M') {
        return false;
    }
    if (!gofly_gps_parse_uint(fields[6U].start, fields[6U].length, 255U,
                              &quality) ||
        !gofly_gps_parse_uint(fields[7U].start, fields[7U].length, 255U,
                              &satellites) ||
        !gofly_gps_parse_fixed(fields[9U].start, fields[9U].length, 3U,
                               true, &altitude_mm_scaled) ||
        !gofly_gps_parse_fixed(fields[8U].start, fields[8U].length, 3U,
                               false, &hdop_scaled)) {
        return false;
    }
    coordinate = fields[2U];
    direction = fields[3U];
    if (!gofly_gps_parse_latitude(&coordinate, &direction, false, &latitude)) {
        return false;
    }
    coordinate = fields[4U];
    direction = fields[5U];
    if (!gofly_gps_parse_latitude(&coordinate, &direction, true, &longitude)) {
        return false;
    }
    if (altitude_mm_scaled < (int64_t)INT32_MIN ||
        altitude_mm_scaled > (int64_t)INT32_MAX ||
        hdop_scaled < 0 || hdop_scaled > 65535LL) {
        return false;
    }
    memset(solution, 0, sizeof(*solution));
    solution->latitude_e7 = latitude;
    solution->longitude_e7 = longitude;
    solution->altitude_mm = (int32_t)altitude_mm_scaled;
    solution->horizontal_accuracy_mm = (uint16_t)hdop_scaled;
    solution->fix_type = (uint8_t)quality;
    solution->satellites = (uint8_t)satellites;
    solution->valid = quality != 0U;
    return true;
}

static bool gofly_gps_parse_rmc(const gofly_gps_field_t fields[],
                                size_t field_count,
                                gofly_gps_solution_t *solution)
{
    int32_t latitude;
    int32_t longitude;
    int64_t speed_thousandths;
    int64_t track_e5;
    gofly_gps_field_t coordinate;
    gofly_gps_field_t direction;

    if (field_count < 10U || fields[0U].length < 3U ||
        fields[0U].start[fields[0U].length - 3U] != (uint8_t)'R' ||
        fields[0U].start[fields[0U].length - 2U] != (uint8_t)'M' ||
        fields[0U].start[fields[0U].length - 1U] != (uint8_t)'C' ||
        fields[2U].length != 1U) {
        return false;
    }
    if (fields[2U].start[0U] == (uint8_t)'V') {
        memset(solution, 0, sizeof(*solution));
        solution->valid = false;
        return true;
    }
    if (fields[2U].start[0U] != (uint8_t)'A') {
        return false;
    }
    coordinate = fields[3U];
    direction = fields[4U];
    if (!gofly_gps_parse_latitude(&coordinate, &direction, false, &latitude)) {
        return false;
    }
    coordinate = fields[5U];
    direction = fields[6U];
    if (!gofly_gps_parse_latitude(&coordinate, &direction, true, &longitude) ||
        !gofly_gps_parse_fixed(fields[7U].start, fields[7U].length, 3U,
                               false, &speed_thousandths) ||
        !gofly_gps_parse_fixed(fields[8U].start, fields[8U].length, 5U,
                               false, &track_e5)) {
        return false;
    }
    if (speed_thousandths < 0 || track_e5 < 0 || track_e5 > 36000000LL ||
        (uint64_t)speed_thousandths >
            (UINT64_MAX - (GOFLY_GPS_MMS_PER_KNOT_DENOMINATOR / 2U)) /
                GOFLY_GPS_MMS_PER_KNOT_NUMERATOR) {
        return false;
    }
    {
        const uint64_t speed_product =
            (uint64_t)speed_thousandths * GOFLY_GPS_MMS_PER_KNOT_NUMERATOR;
        const uint64_t speed_mm_s =
            (speed_product + (GOFLY_GPS_MMS_PER_KNOT_DENOMINATOR / 2U)) /
            GOFLY_GPS_MMS_PER_KNOT_DENOMINATOR;
        if (speed_mm_s > UINT32_MAX) {
            return false;
        }
        memset(solution, 0, sizeof(*solution));
        solution->latitude_e7 = latitude;
        solution->longitude_e7 = longitude;
        solution->ground_speed_mm_s = (uint32_t)speed_mm_s;
    }
    solution->track_deg_e5 = (track_e5 == 36000000LL) ? 0U : (uint32_t)track_e5;
    solution->fix_type = 2U;
    solution->valid = true;
    return true;
}

static bool gofly_gps_accept_solution(gofly_gps_parser_t *parser,
                                      const gofly_gps_solution_t *solution,
                                      bool from_ubx)
{
    if (parser == NULL || solution == NULL) {
        return false;
    }
    if (!from_ubx && parser->have_ubx_solution &&
        !gofly_gps_time_after(parser->timestamp_ms,
                              parser->last_ubx_timestamp_ms)) {
        return false;
    }
    parser->solution = *solution;
    parser->solution.timestamp_ms = parser->timestamp_ms;
    parser->solution_pending = true;
    parser->have_solution = true;
    parser->last_solution_timestamp_ms = parser->timestamp_ms;
    if (from_ubx) {
        ++parser->ubx_frames;
        if (solution->valid) {
            parser->have_ubx_solution = true;
            parser->last_ubx_timestamp_ms = parser->timestamp_ms;
        }
    } else {
        ++parser->nmea_frames;
    }
    ++parser->accepted_frames;
    return true;
}

static bool gofly_gps_parse_nmea_line(gofly_gps_parser_t *parser,
                                      const uint8_t *line, size_t length)
{
    gofly_gps_field_t fields[GOFLY_GPS_NMEA_FIELD_CAPACITY];
    size_t field_count;
    size_t star;
    gofly_gps_solution_t solution;

    if (!gofly_gps_nmea_checksum(line, length, &star)) {
        ++parser->checksum_errors;
        return false;
    }
    (void)star;
    if (!gofly_gps_parse_nmea_fields(line, length - 3U, fields,
                                     &field_count)) {
        ++parser->framing_errors;
        return false;
    }
    if (fields[0U].length >= 3U &&
        fields[0U].start[fields[0U].length - 3U] == (uint8_t)'G' &&
        fields[0U].start[fields[0U].length - 2U] == (uint8_t)'G' &&
        fields[0U].start[fields[0U].length - 1U] == (uint8_t)'A') {
        if (!gofly_gps_parse_gga(fields, field_count, &solution)) {
            ++parser->framing_errors;
            return false;
        }
    } else if (fields[0U].length >= 3U &&
               fields[0U].start[fields[0U].length - 3U] == (uint8_t)'R' &&
               fields[0U].start[fields[0U].length - 2U] == (uint8_t)'M' &&
               fields[0U].start[fields[0U].length - 1U] == (uint8_t)'C') {
        if (!gofly_gps_parse_rmc(fields, field_count, &solution)) {
            ++parser->framing_errors;
            return false;
        }
    } else {
        ++parser->framing_errors;
        return false;
    }
    return gofly_gps_accept_solution(parser, &solution, false);
}

static bool gofly_gps_parse_ubx_frame(gofly_gps_parser_t *parser)
{
    uint8_t checksum_b;
    uint8_t checksum_a;
    const uint8_t *header = &parser->frame[2U];
    const uint8_t *payload = &parser->frame[6U];
    gofly_gps_solution_t solution;
    uint32_t horizontal_accuracy;
    uint32_t speed;
    int32_t track;
    int32_t latitude;
    int32_t longitude;
    uint8_t fix_type;
    uint8_t flags;

    checksum_a = gofly_gps_ubx_checksum(header,
                                        (size_t)GOFLY_GPS_UBX_HEADER_LENGTH +
                                        (size_t)gofly_gps_read_u16_le(&parser->frame[4U]),
                                        &checksum_b);
    if (checksum_a != parser->frame[parser->frame_length - 2U] ||
        checksum_b != parser->frame[parser->frame_length - 1U]) {
        ++parser->checksum_errors;
        return false;
    }
    if (parser->frame[2U] != GOFLY_GPS_UBX_CLASS_NAV ||
        parser->frame[3U] != GOFLY_GPS_UBX_ID_NAV_PVT ||
        gofly_gps_read_u16_le(&parser->frame[4U]) != 92U) {
        ++parser->framing_errors;
        return false;
    }
    fix_type = payload[20U];
    flags = payload[21U];
    horizontal_accuracy = gofly_gps_read_u32_le(&payload[40U]);
    speed = gofly_gps_read_u32_le(&payload[60U]);
    track = gofly_gps_read_i32_le(&payload[64U]);
    latitude = gofly_gps_read_i32_le(&payload[28U]);
    longitude = gofly_gps_read_i32_le(&payload[24U]);
    if (latitude < -900000000 || latitude > 900000000 ||
        longitude < -1800000000 || longitude > 1800000000) {
        ++parser->framing_errors;
        return false;
    }
    memset(&solution, 0, sizeof(solution));
    solution.latitude_e7 = latitude;
    solution.longitude_e7 = longitude;
    solution.altitude_mm = gofly_gps_read_i32_le(&payload[36U]);
    solution.ground_speed_mm_s = speed;
    if (track < 0) {
        track = (int32_t)((36000000LL + (int64_t)track % 36000000LL) %
                          36000000LL);
    }
    solution.track_deg_e5 = (uint32_t)track % 36000000U;
    solution.horizontal_accuracy_mm = horizontal_accuracy > 65535U ?
                                      65535U : (uint16_t)horizontal_accuracy;
    solution.fix_type = fix_type;
    solution.satellites = payload[23U];
    solution.valid = (flags & 0x01U) != 0U && fix_type >= 2U;
    return gofly_gps_accept_solution(parser, &solution, true);
}

static bool gofly_gps_candidate_crc_valid(
    const gofly_gps_parser_t *parser, uint16_t index)
{
    uint16_t payload_length;
    size_t total_length;
    size_t end;
    uint8_t checksum_b;
    uint8_t checksum_a;

    if (parser == NULL || (size_t)index + 6U > parser->frame_length ||
        parser->frame[index] != 0xB5U ||
        parser->frame[index + 1U] != 0x62U) {
        return false;
    }
    payload_length = gofly_gps_read_u16_le(&parser->frame[index + 4U]);
    if (payload_length > GOFLY_GPS_UBX_MAX_PAYLOAD) {
        return false;
    }
    total_length = (size_t)payload_length + GOFLY_GPS_UBX_OVERHEAD;
    end = (size_t)index + total_length;
    if (end > parser->frame_length) {
        return false;
    }
    checksum_a = gofly_gps_ubx_checksum(
        &parser->frame[index + 2U],
        (size_t)GOFLY_GPS_UBX_HEADER_LENGTH + payload_length,
        &checksum_b);
    return checksum_a == parser->frame[end - 2U] &&
           checksum_b == parser->frame[end - 1U];
}

static uint16_t gofly_gps_find_resync_index(
    const gofly_gps_parser_t *parser)
{
    uint16_t fallback = GOFLY_GPS_NO_RESYNC_INDEX;
    uint16_t index;

    for (index = 1U; (size_t)index + 1U < parser->frame_length; ++index) {
        uint16_t payload_length;
        size_t total_length;

        if (parser->frame[index] != 0xB5U ||
            parser->frame[index + 1U] != 0x62U) {
            continue;
        }
        if ((size_t)index + 6U > parser->frame_length) {
            if (fallback == GOFLY_GPS_NO_RESYNC_INDEX) {
                fallback = index;
            }
            continue;
        }
        payload_length = gofly_gps_read_u16_le(&parser->frame[index + 4U]);
        if (payload_length > GOFLY_GPS_UBX_MAX_PAYLOAD) {
            continue;
        }
        total_length = (size_t)payload_length + GOFLY_GPS_UBX_OVERHEAD;
        if (fallback == GOFLY_GPS_NO_RESYNC_INDEX) {
            fallback = index;
        }
        if ((size_t)index + total_length <= parser->frame_length &&
            gofly_gps_candidate_crc_valid(parser, index)) {
            return index;
        }
    }
    if (parser->frame_length != 0U &&
        parser->frame[parser->frame_length - 1U] == 0xB5U &&
        fallback == GOFLY_GPS_NO_RESYNC_INDEX) {
        fallback = (uint16_t)(parser->frame_length - 1U);
    }
    return fallback;
}

static bool gofly_gps_queue_resync(
    gofly_gps_parser_t *parser, uint16_t candidate, uint8_t pending[],
    size_t *pending_length, size_t *pending_offset)
{
    const size_t suffix_length =
        candidate == GOFLY_GPS_NO_RESYNC_INDEX ? 0U :
        (size_t)parser->frame_length - candidate;
    const size_t remaining = *pending_length - *pending_offset;
    const size_t total_length = suffix_length + remaining;

    if (candidate == GOFLY_GPS_NO_RESYNC_INDEX ||
        total_length > GOFLY_GPS_PENDING_BUFFER_SIZE) {
        return false;
    }
    memmove(&pending[suffix_length], &pending[*pending_offset], remaining);
    if (suffix_length != 0U) {
        memcpy(pending, &parser->frame[candidate], suffix_length);
    }
    *pending_length = total_length;
    *pending_offset = 0U;
    return true;
}

static void gofly_gps_reset(gofly_gps_parser_t *parser)
{
    parser->state = GOFLY_GPS_PARSE_SEARCH;
    parser->frame_length = 0U;
    parser->expected_length = 0U;
}

static void gofly_gps_restart_with_byte(gofly_gps_parser_t *parser,
                                        uint8_t byte)
{
    gofly_gps_reset(parser);
    if (byte == 0xB5U) {
        parser->frame[0U] = byte;
        parser->frame_length = 1U;
        parser->state = GOFLY_GPS_PARSE_UBX_SYNC;
    } else if (byte == (uint8_t)'$') {
        parser->frame[0U] = byte;
        parser->frame_length = 1U;
        parser->state = GOFLY_GPS_PARSE_NMEA;
    }
}

void gofly_gps_parser_init(gofly_gps_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->state = GOFLY_GPS_PARSE_SEARCH;
}

void gofly_gps_parser_set_timestamp(gofly_gps_parser_t *parser,
                                    uint32_t timestamp_ms)
{
    if (parser != NULL) {
        parser->timestamp_ms = timestamp_ms;
    }
}

size_t gofly_gps_parser_feed(gofly_gps_parser_t *parser,
                             const uint8_t *bytes, size_t length)
{
    return gofly_gps_parser_feed_at(parser, bytes, length,
                                    parser == NULL ? 0U : parser->timestamp_ms);
}

size_t gofly_gps_parser_feed_at(gofly_gps_parser_t *parser,
                                const uint8_t *bytes, size_t length,
                                uint32_t timestamp_ms)
{
    uint8_t pending[GOFLY_GPS_PENDING_BUFFER_SIZE];
    size_t pending_length = 0U;
    size_t pending_offset = 0U;
    size_t consumed = 0U;

    if (parser == NULL || (bytes == NULL && length != 0U)) {
        return 0U;
    }
    parser->timestamp_ms = timestamp_ms;
    while (consumed < length || pending_offset < pending_length) {
        const bool replaying = pending_offset < pending_length;
        const uint8_t byte = replaying ? pending[pending_offset++] :
                                         bytes[consumed++];

        if (parser->state == GOFLY_GPS_PARSE_SEARCH) {
            if (byte == 0xB5U) {
                parser->frame[0U] = byte;
                parser->frame_length = 1U;
                parser->state = GOFLY_GPS_PARSE_UBX_SYNC;
            } else if (byte == (uint8_t)'$') {
                parser->frame[0U] = byte;
                parser->frame_length = 1U;
                parser->state = GOFLY_GPS_PARSE_NMEA;
            }
            continue;
        }

        if (parser->state == GOFLY_GPS_PARSE_UBX_SYNC) {
            if (byte != 0x62U) {
                ++parser->framing_errors;
                gofly_gps_restart_with_byte(parser, byte);
                continue;
            }
            parser->frame[parser->frame_length++] = byte;
            parser->state = GOFLY_GPS_PARSE_UBX_FRAME;
            continue;
        }

        if (parser->state == GOFLY_GPS_PARSE_NMEA) {
            if (byte == (uint8_t)'$' && parser->frame_length > 1U) {
                ++parser->framing_errors;
                parser->frame[0U] = byte;
                parser->frame_length = 1U;
                continue;
            }
            if (byte == (uint8_t)'\r' || byte == (uint8_t)'\n') {
                if (parser->frame_length > 1U) {
                    (void)gofly_gps_parse_nmea_line(parser, parser->frame,
                                                     parser->frame_length);
                }
                gofly_gps_reset(parser);
                continue;
            }
            if (parser->frame_length >= GOFLY_GPS_NMEA_MAX_SENTENCE - 1U) {
                ++parser->framing_errors;
                gofly_gps_restart_with_byte(parser, byte);
                continue;
            }
            parser->frame[parser->frame_length++] = byte;
            continue;
        }

        if (parser->frame_length >= GOFLY_GPS_MAX_FRAME_SIZE) {
            ++parser->framing_errors;
            gofly_gps_restart_with_byte(parser, byte);
            continue;
        }
        parser->frame[parser->frame_length++] = byte;
        if (parser->frame_length == 6U) {
            const uint16_t payload_length = gofly_gps_read_u16_le(&parser->frame[4U]);
            if (payload_length > GOFLY_GPS_UBX_MAX_PAYLOAD) {
                ++parser->framing_errors;
                gofly_gps_restart_with_byte(parser, byte);
                continue;
            }
            parser->expected_length = (uint16_t)(payload_length +
                                                 GOFLY_GPS_UBX_OVERHEAD);
        } else if (parser->expected_length != 0U &&
                   parser->frame_length == parser->expected_length) {
            const bool parsed = gofly_gps_parse_ubx_frame(parser);
            const uint16_t candidate = parsed ?
                GOFLY_GPS_NO_RESYNC_INDEX :
                gofly_gps_find_resync_index(parser);

            if (!parsed && candidate != GOFLY_GPS_NO_RESYNC_INDEX &&
                candidate < parser->frame_length &&
                !gofly_gps_queue_resync(parser, candidate, pending,
                                        &pending_length, &pending_offset)) {
                ++parser->framing_errors;
            }
            gofly_gps_reset(parser);
        }
    }
    return consumed;
}

bool gofly_gps_parser_take(gofly_gps_parser_t *parser,
                           gofly_gps_solution_t *solution)
{
    if (parser == NULL || solution == NULL || !parser->solution_pending) {
        return false;
    }
    *solution = parser->solution;
    parser->solution_pending = false;
    return true;
}

bool gofly_gps_parser_read(const gofly_gps_parser_t *parser,
                           uint32_t now_ms,
                           gofly_gps_solution_t *solution)
{
    uint32_t age;

    if (parser == NULL || solution == NULL || !parser->have_solution) {
        return false;
    }
    *solution = parser->solution;
    age = (uint32_t)(now_ms - solution->timestamp_ms);
    if (age > GOFLY_GPS_STALE_TIMEOUT_MS) {
        solution->valid = false;
    }
    return true;
}

bool gofly_gps_parser_take_at(gofly_gps_parser_t *parser,
                              uint32_t now_ms,
                              gofly_gps_solution_t *solution)
{
    if (!gofly_gps_parser_take(parser, solution)) {
        return false;
    }
    if ((uint32_t)(now_ms - solution->timestamp_ms) >
        GOFLY_GPS_STALE_TIMEOUT_MS) {
        solution->valid = false;
    }
    return true;
}

bool gofly_gps_parser_is_fresh(const gofly_gps_parser_t *parser,
                               uint32_t now_ms)
{
    gofly_gps_solution_t solution;
    return gofly_gps_parser_read(parser, now_ms, &solution) && solution.valid;
}
