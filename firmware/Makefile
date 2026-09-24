PYTHON ?= python
TARGET_CC ?= arm-none-eabi-gcc
TARGET_SIZE ?= arm-none-eabi-size
HOST_CC ?= cc
BUILD_DIR := build
OBJ_DIR := $(BUILD_DIR)/obj
TARGET_ELF := $(BUILD_DIR)/gofly_fc.elf
TARGET_MAP := $(BUILD_DIR)/gofly_fc.map
HOST_TEST_BIN := $(BUILD_DIR)/host/gofly_host_tests

TARGET_CPPFLAGS := -DGOFLY_TARGET=1 -DGOFLY_HOST_TEST=0 -I Core/Inc -I Config -I Platform \
                   -I Drivers/STM32F4xx_HAL_Driver/Inc -I Drivers/CMSIS/Include \
                   -I Drivers/CMSIS/Device/ST/STM32F4xx/Include
TARGET_CFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
                 -ffunction-sections -fdata-sections -std=c11 -Wall -Wextra -Os
TARGET_LDFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
                  -TSTM32F405RGTX_FLASH.ld -Wl,--gc-sections -Wl,-Map=$(TARGET_MAP) \
                  --specs=nano.specs --specs=nosys.specs

HOST_CPPFLAGS := -DGOFLY_TARGET=0 -DGOFLY_HOST_TEST=1 -I . -I Config -I Platform
HOST_CFLAGS := -std=c11 -Wall -Wextra -Wconversion -Wshadow -Werror
HOST_LDFLAGS := -lm

# Future target-only adapters stay out of the host link. All protocol, Flight,
# Navigation, Safety, and pure Platform files are intentionally host-buildable.
HOST_SOURCES := $(wildcard Platform/*.c) \
                $(wildcard Drivers/Dshot300/*.c) \
                $(wildcard Drivers/Crsf/*.c) \
                $(wildcard Drivers/EscTelemetry/*.c) \
                $(wildcard Drivers/Gps/*.c) \
                $(wildcard Drivers/Icm42688/*.c) \
                $(wildcard Drivers/Bmp280/*.c) \
                $(wildcard Drivers/W25q128/*.c) \
                $(wildcard Drivers/Max7456/*.c) \
                $(wildcard Drivers/AdcSense/*.c) \
                $(wildcard Flight/*.c) \
                $(wildcard Navigation/*.c) \
                $(wildcard Safety/*.c) \
                $(wildcard App/*.c)
HOST_SOURCES := $(filter-out %/dshot_hal_backend.c %/hal_bus_adapters.c %/board_bsp.c,$(HOST_SOURCES))
HOST_TEST_SOURCES := $(HOST_SOURCES) $(wildcard tests/host/*.c)
HOST_OBJECTS := $(patsubst %.c,$(OBJ_DIR)/host/%.o,$(HOST_TEST_SOURCES))

.PHONY: all target bench host-test size clean

all: bench

build:
	@mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/toolchain-report.json: tools/probe_toolchain.py | build
	@$(PYTHON) tools/probe_toolchain.py --output $@

.PHONY: prerequisites
prerequisites: $(BUILD_DIR)/toolchain-report.json
	@status=0; \
	reason=""; \
	if ! command -v "$(TARGET_CC)" >/dev/null 2>&1; then \
		reason="target compiler $(TARGET_CC) not found"; \
		status=2; \
	fi; \
	if ! $(PYTHON) tools/import_cube_sources.py --firmware-root . --manifest $(BUILD_DIR)/cube-source-manifest.json --validate-only >/dev/null; then \
		if [ -n "$$reason" ]; then reason="$$reason; "; fi; \
		reason="$${reason}real Cube source manifest missing or invalid"; \
		status=2; \
	fi; \
	if [ "$$status" -ne 0 ]; then \
		echo "target_build_status: unavailable"; \
		echo "target_build_reason: $$reason" >&2; \
		exit "$$status"; \
	fi; \
	echo "target_build_status: available"

# These recipes are intentionally guarded by the real compiler and manifest.
target: prerequisites
	@$(MAKE) MODE=target $(TARGET_ELF)

bench: prerequisites
	@$(MAKE) MODE=bench $(TARGET_ELF)

TARGET_C_SOURCES := $(wildcard Core/Src/*.c) \
                    $(wildcard Drivers/*/*.c) \
                    $(wildcard Drivers/*/*/*.c) \
                    $(wildcard Platform/*.c) \
                    $(wildcard Flight/*.c) \
                    $(wildcard Navigation/*.c) \
                    $(wildcard Safety/*.c) \
                    $(wildcard App/*.c)
TARGET_C_SOURCES += Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
TARGET_OBJECTS := $(patsubst %.c,$(OBJ_DIR)/target/%.o,$(TARGET_C_SOURCES))

$(TARGET_ELF): $(TARGET_OBJECTS) startup_stm32f405rgtx.s STM32F405RGTX_FLASH.ld
	@mkdir -p $(dir $@)
	$(TARGET_CC) $(TARGET_LDFLAGS) $(TARGET_OBJECTS) startup_stm32f405rgtx.s -o $@

$(OBJ_DIR)/target/%.o: %.c
	@mkdir -p $(dir $@)
	$(TARGET_CC) $(TARGET_CPPFLAGS) $(TARGET_CFLAGS) -DGOFLY_BUILD_MODE=$(MODE) -c $< -o $@

host-test:
	@command -v "$(HOST_CC)" >/dev/null 2>&1 || (echo "host test unavailable: $(HOST_CC) not found" >&2; exit 2)
	@test -n "$(HOST_TEST_SOURCES)" || (echo "host test unavailable: no host sources" >&2; exit 2)
	@mkdir -p $(OBJ_DIR)/host
	@$(MAKE) $(HOST_TEST_BIN) TEST_FILTER=$(TEST_FILTER)
	@$(HOST_TEST_BIN) $(if $(TEST_FILTER),--filter $(TEST_FILTER),)

$(HOST_TEST_BIN): $(HOST_OBJECTS)
	@mkdir -p $(dir $@)
	$(HOST_CC) $(HOST_CFLAGS) $^ $(HOST_LDFLAGS) -o $@

$(OBJ_DIR)/host/%.o: %.c
	@mkdir -p $(dir $@)
	$(HOST_CC) $(HOST_CPPFLAGS) $(HOST_CFLAGS) -c $< -o $@

size: $(TARGET_ELF)
	@$(TARGET_SIZE) $(TARGET_ELF)

clean:
	rm -rf $(OBJ_DIR) $(TARGET_ELF) $(TARGET_MAP) $(HOST_TEST_BIN)
