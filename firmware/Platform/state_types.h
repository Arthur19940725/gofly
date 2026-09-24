#ifndef GOFLY_PLATFORM_STATE_TYPES_H
#define GOFLY_PLATFORM_STATE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t timestamp_us;
    float accel_mps2[3];
    float gyro_rad_s[3];
    bool valid;
} gofly_imu_sample_t;

typedef struct {
    uint32_t timestamp_ms;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_mm;
    uint32_t ground_speed_mm_s;
    uint32_t track_deg_e5;
    uint16_t horizontal_accuracy_mm;
    uint8_t fix_type;
    uint8_t satellites;
    bool valid;
} gofly_gps_solution_t;

typedef struct {
    uint16_t throttle;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    bool arm_request;
    bool rth_request;
    bool link_valid;
} gofly_rc_input_t;

typedef struct {
    uint16_t value[4];
    bool telemetry_request[4];
} gofly_motor_command_t;

#ifdef __cplusplus
}
#endif

#endif
