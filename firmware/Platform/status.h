#ifndef GOFLY_PLATFORM_STATUS_H
#define GOFLY_PLATFORM_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GOFLY_OK = 0,
    GOFLY_E_ARGUMENT = -1,
    GOFLY_E_TIMEOUT = -2,
    GOFLY_E_BUS = -3,
    GOFLY_E_CRC = -4,
    GOFLY_E_STATE = -5,
    GOFLY_E_RANGE = -6,
    GOFLY_E_NOT_READY = -7,
    GOFLY_E_UNSUPPORTED = -8
} gofly_status_t;

#ifdef __cplusplus
}
#endif

#endif
