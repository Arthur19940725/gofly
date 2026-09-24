#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "board_map.h"
#include "fault_log.h"
#include "flight_config.h"
#include "ring_buffer.h"
#include "safety_config.h"
#include "state_snapshot.h"
#include "timebase.h"

_Static_assert(GOFLY_BOARD_MAP_VERSION == 1U, "unexpected board map version");
_Static_assert(GOFLY_IMU_SPI_INSTANCE == 1U, "unexpected IMU SPI instance");
_Static_assert(GOFLY_FLASH_SPI_INSTANCE == 3U, "unexpected flash SPI instance");
_Static_assert(GOFLY_OSD_SPI_INSTANCE == 2U, "unexpected OSD SPI instance");
_Static_assert(GOFLY_MOTOR1_PIN == 6U && GOFLY_MOTOR2_PIN == 7U &&
               GOFLY_MOTOR3_PIN == 8U && GOFLY_MOTOR4_PIN == 9U,
               "unexpected motor pin map");
_Static_assert(GOFLY_IMU_RATE_HZ == 1000U, "unexpected IMU rate");
_Static_assert(GOFLY_ATTITUDE_RATE_HZ == 500U, "unexpected attitude rate");
_Static_assert(GOFLY_DSHOT_RATE_HZ == 1000U, "unexpected DShot rate");
_Static_assert(GOFLY_GPS_BAUD == 57600UL, "unexpected GPS baud");
_Static_assert(GOFLY_CRSF_BAUD == 420000UL, "unexpected CRSF baud");
_Static_assert(GOFLY_IMU_TIMEOUT_US == 3000UL, "unexpected IMU timeout");
_Static_assert(GOFLY_FLIGHT_ENABLE == 0, "flight output must default off");
_Static_assert(GOFLY_REAL_OUTPUT == 0, "real output must default off");
_Static_assert(GOFLY_RTH_REAL_OUTPUT == 0, "real RTH output must default off");
_Static_assert(GOFLY_DSHOT_HW_BACKEND == 0, "hardware DShot must default off");

static unsigned test_failures;

int gofly_test_dshot300_run(void);
int gofly_test_crsf_run(void);
int gofly_test_esc_telemetry_run(void);
int gofly_test_gps_parser_run(void);
int gofly_test_icm42688_run(void);
int gofly_test_bmp280_run(void);
int gofly_test_w25q128_run(void);
int gofly_test_max7456_run(void);
int gofly_test_adc_sense_run(void);

#define CHECK(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "FAIL: %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            ++test_failures; \
        } \
    } while (0)

static void test_ring_buffer_rejects_over_capacity(void)
{
    uint8_t storage[4] = {0U};
    const uint8_t input[4] = {1U, 2U, 3U, 4U};
    uint8_t output[4] = {0U};
    gofly_ring_t ring;

    gofly_ring_init(&ring, storage, sizeof(storage));
    CHECK(gofly_ring_status(&ring) == GOFLY_OK);
    CHECK(gofly_ring_write(&ring, NULL, 1U) == 0U);
    CHECK(gofly_ring_status(&ring) == GOFLY_E_ARGUMENT);
    CHECK(gofly_ring_write(&ring, input, sizeof(input)) == sizeof(input));
    CHECK(gofly_ring_available(&ring) == sizeof(input));
    CHECK(gofly_ring_write(&ring, input, 1U) == 0U);
    CHECK(gofly_ring_status(&ring) == GOFLY_E_RANGE);
    CHECK(gofly_ring_read(&ring, output, sizeof(output)) == sizeof(output));
    CHECK(memcmp(input, output, sizeof(input)) == 0);
    CHECK(gofly_ring_available(&ring) == 0U);

    CHECK(gofly_ring_write(&ring, input, 2U) == 2U);
    CHECK(gofly_ring_read(&ring, output, 1U) == 1U);
    CHECK(output[0] == input[0]);
    CHECK(gofly_ring_write(&ring, &input[2], 2U) == 2U);
    CHECK(gofly_ring_available(&ring) == 3U);
}

static void test_snapshot_returns_only_published_state(void)
{
    gofly_snapshot_t snapshot;
    uint32_t first[4] = {1U, 2U, 3U, 4U};
    uint32_t second[4] = {5U, 6U, 7U, 8U};
    uint32_t result[4] = {0U};

    gofly_snapshot_init(&snapshot, sizeof(first));
    CHECK(gofly_snapshot_status(&snapshot) == GOFLY_OK);
    CHECK(!gofly_snapshot_read(&snapshot, result));
    gofly_snapshot_publish(&snapshot, NULL);
    CHECK(gofly_snapshot_status(&snapshot) == GOFLY_E_ARGUMENT);

    gofly_snapshot_publish(&snapshot, first);
    CHECK(gofly_snapshot_read(&snapshot, result));
    CHECK(memcmp(first, result, sizeof(first)) == 0);

    gofly_snapshot_publish(&snapshot, second);
    memset(result, 0, sizeof(result));
    CHECK(gofly_snapshot_read(&snapshot, result));
    CHECK(memcmp(second, result, sizeof(second)) == 0);
}

static void test_fault_log_sets_overflow_without_allocating(void)
{
    gofly_fault_log_t log;
    gofly_fault_record_t record;
    size_t index;

    gofly_fault_log_init(&log);
    CHECK(gofly_fault_log_status(&log) == GOFLY_OK);
    CHECK(!gofly_fault_log_get(&log, 0U, &record));
    for (index = 0U; index < GOFLY_FAULT_LOG_CAPACITY; ++index) {
        gofly_fault_log_record(&log, (uint16_t)(index + 1U), 1U,
                               (uint32_t)index);
    }
    CHECK(gofly_fault_log_count(&log) == GOFLY_FAULT_LOG_CAPACITY);
    CHECK(!gofly_fault_log_overflowed(&log));

    gofly_fault_log_record(&log, 17U, 1U, 100U);
    CHECK(gofly_fault_log_count(&log) == GOFLY_FAULT_LOG_CAPACITY);
    CHECK(gofly_fault_log_overflowed(&log));
    CHECK(gofly_fault_log_get(&log, 0U, &record));
    CHECK(record.code == 2U);
    CHECK(gofly_fault_log_get(&log, GOFLY_FAULT_LOG_CAPACITY - 1U,
                             &record));
    CHECK(record.code == 17U);

    gofly_fault_log_record(&log, 17U, 1U, 101U);
    CHECK(gofly_fault_log_count(&log) == GOFLY_FAULT_LOG_CAPACITY);
    CHECK(gofly_fault_log_get(&log, GOFLY_FAULT_LOG_CAPACITY - 1U,
                             &record));
    CHECK(record.count == 2U);
    CHECK(record.last_timestamp_us == 101U);
}

static void test_timebase_is_wrap_safe(void)
{
    gofly_time_set_for_test(1234567U);
    CHECK(gofly_time_now_us() == 1234567U);
    CHECK(gofly_time_now_ms() == 1234U);
    CHECK(gofly_time_elapsed_us(5U, UINT32_MAX - 2U) == 8U);
}

static void run_dshot_tests(void)
{
    test_failures += (unsigned)gofly_test_dshot300_run();
}

static void run_crsf_tests(void)
{
    test_failures += (unsigned)gofly_test_crsf_run();
}

static void run_esc_tests(void)
{
    test_failures += (unsigned)gofly_test_esc_telemetry_run();
}

static void run_gps_tests(void)
{
    test_failures += (unsigned)gofly_test_gps_parser_run();
}

static void run_device_tests(void)
{
    test_failures += (unsigned)gofly_test_icm42688_run();
    test_failures += (unsigned)gofly_test_bmp280_run();
    test_failures += (unsigned)gofly_test_w25q128_run();
    test_failures += (unsigned)gofly_test_max7456_run();
    test_failures += (unsigned)gofly_test_adc_sense_run();
}

static void run_protocol_tests(void)
{
    run_dshot_tests();
    run_crsf_tests();
    run_esc_tests();
    run_gps_tests();
}

static void run_platform_tests(void)
{
    test_ring_buffer_rejects_over_capacity();
    test_snapshot_returns_only_published_state();
    test_fault_log_sets_overflow_without_allocating();
    test_timebase_is_wrap_safe();
}

int main(int argc, char **argv)
{
    const char *filter = NULL;

    if (argc == 3 && strcmp(argv[1], "--filter") == 0) {
        filter = argv[2];
    } else if (argc != 1) {
        fprintf(stderr, "usage: %s [--filter platform|protocol|device|dshot|crsf|esc|gps|icm42688|bmp280|w25q128|max7456|adc_sense]\n",
                argv[0]);
        return 2;
    }

    if (filter == NULL) {
        run_platform_tests();
        run_protocol_tests();
        run_device_tests();
    } else if (strcmp(filter, "platform") == 0) {
        run_platform_tests();
    } else if (strcmp(filter, "protocol") == 0) {
        run_protocol_tests();
    } else if (strcmp(filter, "device") == 0) {
        run_device_tests();
    } else if (strcmp(filter, "icm42688") == 0) {
        test_failures += (unsigned)gofly_test_icm42688_run();
    } else if (strcmp(filter, "bmp280") == 0) {
        test_failures += (unsigned)gofly_test_bmp280_run();
    } else if (strcmp(filter, "w25q128") == 0) {
        test_failures += (unsigned)gofly_test_w25q128_run();
    } else if (strcmp(filter, "max7456") == 0) {
        test_failures += (unsigned)gofly_test_max7456_run();
    } else if (strcmp(filter, "adc_sense") == 0) {
        test_failures += (unsigned)gofly_test_adc_sense_run();
    } else if (strcmp(filter, "dshot") == 0) {
        run_dshot_tests();
    } else if (strcmp(filter, "crsf") == 0) {
        run_crsf_tests();
    } else if (strcmp(filter, "esc") == 0) {
        run_esc_tests();
    } else if (strcmp(filter, "gps") == 0) {
        run_gps_tests();
    } else {
        return 2;
    }

    if (test_failures != 0U) {
        fprintf(stderr, "%s tests: %u failure(s)\n",
                filter == NULL ? "all" : filter, test_failures);
        return 1;
    }

    printf("%s tests: PASS\n", filter == NULL ? "all" : filter);
    return 0;
}
