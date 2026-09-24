#ifndef GOFLY_DRIVER_MAX7456_H
#define GOFLY_DRIVER_MAX7456_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "bus_interfaces.h"
#include "status.h"

#define GOFLY_MAX7456_REG_VM0 0x00U
#define GOFLY_MAX7456_REG_VM1 0x01U
#define GOFLY_MAX7456_VM0_PAL 0x40U
#define GOFLY_MAX7456_VM0_NTSC 0x00U
#define GOFLY_MAX7456_VM0_RESET 0x02U
#define GOFLY_MAX7456_SPI_TRANSACTION_LENGTH 2U
#define GOFLY_MAX7456_RESET_MIN_MS 50U
#define GOFLY_MAX7456_POST_RESET_MS 1U

typedef enum {
    GOFLY_MAX7456_VIDEO_NTSC = 0,
    GOFLY_MAX7456_VIDEO_PAL = 1
} gofly_max7456_video_standard_t;

typedef enum {
    GOFLY_DEVICE_ANALOG_CVBS = 1
} gofly_device_kind_t;

typedef struct {
    gofly_spi_bus_t spi;
    gofly_pin_write_fn reset_pin_write;
    void *reset_context;
    gofly_delay_ms_fn delay_ms;
    void *delay_context;
    uint32_t timeout_ms;
} gofly_max7456_config_t;

typedef struct {
    gofly_spi_bus_t spi;
    gofly_pin_write_fn reset_pin_write;
    void *reset_context;
    gofly_delay_ms_fn delay_ms;
    void *delay_context;
    uint32_t timeout_ms;
    gofly_device_kind_t device_kind;
    bool initialized;
    bool reset_complete;
} gofly_max7456_t;

void gofly_max7456_default_config(gofly_max7456_config_t *config);
gofly_status_t gofly_max7456_init(gofly_max7456_t *device,
                                  const gofly_max7456_config_t *config);
gofly_status_t gofly_max7456_reset(gofly_max7456_t *device);
gofly_status_t gofly_max7456_read_register(gofly_max7456_t *device,
                                           uint8_t reg,
                                           uint8_t *value);
gofly_status_t gofly_max7456_write_register(gofly_max7456_t *device,
                                            uint8_t reg,
                                            uint8_t value);
gofly_status_t gofly_max7456_configure_video_standard(
    gofly_max7456_t *device, gofly_max7456_video_standard_t standard);

#ifdef __cplusplus
}
#endif

#endif
