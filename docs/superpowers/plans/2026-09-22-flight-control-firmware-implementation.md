# gofly_fc Flight-Control Firmware Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a self-contained STM32F405RGT6 Cube HAL firmware project for the `gofly_fc` research flight controller, with tested sensor/protocol/control/navigation modules and a bench-safe DShot300/RTH integration.

**Architecture:** Use a no-RTOS, fixed-memory application with a thin Cube HAL board layer, HAL-independent drivers and control logic, fixed-period scheduling, immutable state snapshots, and an explicit output authorization gate. Host tests exercise every protocol, estimator, controller, navigation, and safety path without STM32 hardware; target builds keep real motor output disabled until the TIM3/DMA mapping and bench checks are verified.

**Tech Stack:** C11, STM32F405RGT6, STM32CubeF4 HAL/CMSIS, `arm-none-eabi-gcc`, GNU Make, CubeMX `.ioc`, host C compiler, Python 3 tooling for environment checks and HIL orchestration.

**Spec:** [docs/superpowers/specs/2026-09-22-flight-control-firmware-design.md](../specs/2026-09-22-flight-control-firmware-design.md)

## Global Constraints

- The target is **STM32F405RGT6 LQFP64**; no PE pins may appear in the board map.
- Use the approved resource map: SPI1 PA5/PA6/PA7 for IMU, SPI3 PB3/PB4/PB5 for flash, SPI2 PB13/PB14/PB15 for MAX7456, I2C1 PB8/PB9 for BMP280, USART1 PA9/PA10 for CRSF, USART3 PB10/PB11 for GPS, UART4 PC10/PC11 for VTX MSP, USART2 PA2/PA3 for ESC telemetry, TIM3 PC6–PC9 for motors, ADC PC0/PC1 for battery/current, USB PA11/PA12, SWD PA13/PA14.
- Use an 8 MHz HSE and a 168 MHz system clock with a valid 48 MHz USB clock.
- Use no RTOS, no dynamic allocation, no unbounded parsing, and no blocking work in ISR/DMA callbacks.
- Use DShot300 frame generation and a separate ESC telemetry input; do not implement bidirectional DShot in this plan.
- Use UBX `NAV-PVT` as the primary GPS source, NMEA GGA/RMC as fallback, and USART3 at 57600 baud by default.
- `GOFLY_FLIGHT_ENABLE=0`, `GOFLY_REAL_OUTPUT=0`, and `GOFLY_RTH_REAL_OUTPUT=0` are immutable default safety values.
- When output authorization is false, every motor command must be zeroed before the HAL output backend sees it.
- The absence of a magnetometer means yaw confidence is bounded; RTH must remain observation-only unless an explicit future hardware/software review changes the gate.
- MAX7456 is a 5 V analog-CVBS OSD device only; no digital-video data path may be added.
- Do not connect raw VBAT or an unverified 5 V signal to an STM32 input in firmware assumptions or test fixtures.
- The current directory is not a Git repository; do not add commit steps, initialize Git, or alter unrelated KiCad files. Use build/test logs and file checksums as checkpoints instead.
- The current workstation did not expose `arm-none-eabi-gcc`, Make, CMake, or CubeMX during design. Target compilation must be reported as unavailable if the toolchain remains absent; never fabricate a target-build result.
- This firmware and the existing PCB are a research prototype and must never be reported as flight-ready.

## File Map

All firmware changes live under `firmware/`; the existing KiCad project and its backups remain untouched.

| Path | Responsibility |
|---|---|
| `firmware/gofly_fc.ioc` | CubeMX peripheral, clock, pin, DMA, middleware, and project configuration |
| `firmware/Makefile` | `target`, `bench`, `host-test`, `size`, and `clean` build entry points |
| `firmware/README.md` | toolchain version, board contract, build commands, safety procedure, and verified/unverified status |
| `firmware/Config/board_map.h` | one authoritative MCU pin/peripheral map and compile-time identity checks |
| `firmware/Config/flight_config.h` | rates, units, PID defaults, sensor ranges, and mixer defaults |
| `firmware/Config/safety_config.h` | compile-time output/RTH gates and timeout thresholds |
| `firmware/Platform/` | status codes, timebase, ring buffers, snapshots, fixed fault log, and HAL-independent interfaces |
| `firmware/Core/Inc` and `firmware/Core/Src` | Cube-generated startup/application entry and HAL initialization |
| `firmware/Drivers/<device>/` | device register/protocol logic plus a thin transport adapter |
| `firmware/Flight/` | quaternion estimator, PID controllers, Quad-X mixer, and flight state |
| `firmware/Navigation/` | GPS health, WGS-84 local NED conversion, home, and RTH intent state machine |
| `firmware/Safety/` | health aggregation, failsafe causes, and output authorization |
| `firmware/App/` | fixed-period scheduler, state snapshots, parameter boundary, USB/telemetry status, and boot sequence |
| `firmware/tests/host/` | dependency-free C test runner and unit tests |
| `firmware/tests/hil/` | deterministic fixed-step scenarios and expected event assertions |
| `firmware/tools/` | Cube package/toolchain probe, host-test launcher, HIL runner, and report generation |
| `firmware/Drivers/STM32F4xx_HAL_Driver/` | vendored ST HAL sources from one recorded STM32CubeF4 release |
| `firmware/Drivers/CMSIS/` | vendored CMSIS device/core headers from the same release |
| `firmware/startup_stm32f405rgtx.s` | target vector table and reset entry |
| `firmware/STM32F405RGTX_FLASH.ld` | flash/RAM memory layout and stack/heap policy |

## Interfaces Established Before Implementation

The following names and signatures are the cross-task contract. Keep them stable unless every dependent task and test is updated together.

```c
/* Platform/status.h */
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

/* Platform/state_types.h */
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
```

```c
/* Drivers/Dshot300/dshot300.h */
uint16_t gofly_dshot300_encode(uint16_t throttle, bool telemetry_request);
gofly_status_t gofly_dshot300_build(gofly_motor_command_t command,
                                    uint16_t frames[4]);
```

```c
/* Drivers/Gps/gps_parser.h */
void gofly_gps_parser_init(gofly_gps_parser_t *parser);
size_t gofly_gps_parser_feed(gofly_gps_parser_t *parser,
                             const uint8_t *bytes, size_t length);
bool gofly_gps_parser_take(gofly_gps_parser_t *parser,
                           gofly_gps_solution_t *solution);
```

```c
/* Navigation/rth_state.h */
typedef enum {
    GOFLY_RTH_DISABLED,
    GOFLY_RTH_GPS_UNAVAILABLE,
    GOFLY_RTH_HOME_READY,
    GOFLY_RTH_REQUESTED,
    GOFLY_RTH_CLIMB_INTENT,
    GOFLY_RTH_RETURN_INTENT,
    GOFLY_RTH_ARRIVAL_INTENT,
    GOFLY_RTH_ABORTED,
    GOFLY_RTH_COMPLETE
} gofly_rth_state_t;

typedef struct {
    float north_m;
    float east_m;
    float down_m;
    float distance_m;
    float bearing_rad;
    float target_altitude_m;
    bool valid;
} gofly_rth_intent_t;
```

```c
/* Safety/output_gate.h */
typedef struct {
    bool compile_time_enabled;
    bool bench_mode;
    bool self_test_ok;
    bool imu_fresh;
    bool attitude_valid;
    bool rc_valid;
    bool arm_requested;
    bool dshot_backend_ok;
    bool high_priority_fault;
} gofly_output_conditions_t;

bool gofly_output_is_authorized(const gofly_output_conditions_t *conditions);
void gofly_output_apply_gate(bool authorized,
                             gofly_motor_command_t *command);
```

---

### Task 1: Establish a reproducible Cube HAL project scaffold

**Files:**
- Create: `firmware/gofly_fc.ioc`
- Create: `firmware/Makefile`
- Create: `firmware/README.md`
- Create: `firmware/tools/probe_toolchain.py`
- Create: `firmware/tools/import_cube_sources.py`
- Create: `firmware/Core/Inc/main.h`
- Create: `firmware/Core/Src/main.c`
- Create: `firmware/startup_stm32f405rgtx.s`
- Create: `firmware/STM32F405RGTX_FLASH.ld`
- Create: `firmware/Drivers/STM32F4xx_HAL_Driver/` and `firmware/Drivers/CMSIS/` only from a real ST package

**Interfaces:**
- Consumes: the approved MCU/resource table and the toolchain constraints in the specification.
- Produces: a project that can report its build prerequisites, configure every selected peripheral, and compile a minimal `main` when the recorded ST package and compiler are available.

- [ ] **Step 1: Define the prerequisite report contract and its failing test.**

  Add a Python test fixture under `firmware/tools/test_probe_toolchain.py` that feeds the probe a temporary PATH containing no embedded compiler and asserts that the result has `target_compiler=false`, `host_compiler` is a boolean, and `can_target_build=false`. The expected failure before implementation is an import or function-not-found error.

  ```python
  report = probe_toolchain(path_override=[])
  assert report["target_compiler"] is False
  assert report["can_target_build"] is False
  assert isinstance(report["host_compiler"], bool)
  ```

- [ ] **Step 2: Implement the probe and run its focused test.**

  Implement `probe_toolchain(path_override=None)` and a CLI that writes `build/toolchain-report.json`. Probe `arm-none-eabi-gcc --version`, `make --version`, `python --version`, and `STM32CubeMX --version` without installing anything. Run:

  ```bash
  python firmware/tools/test_probe_toolchain.py
  python firmware/tools/probe_toolchain.py --output firmware/build/toolchain-report.json
  ```

  Expected: the test passes; the report records the actual missing embedded tools instead of hiding them.

- [ ] **Step 3: Write the exact CubeMX configuration contract.**

  Configure `gofly_fc.ioc` for `STM32F405RGT6`, 8 MHz HSE, 168 MHz SYSCLK, USB 48 MHz, SPI1/2/3 master, I2C1, USART1/2/3, UART4, ADC1 scan for PC0/PC1, TIM3 channels 1–4 on PC6–PC9, TIM6 at 1 kHz, EXTI4 for PC4, SWD, USB device FS, and safe GPIO defaults. Set USART1 to 420000 baud half-duplex, USART3 to 57600 8N1, and keep all motor pins low during initialization.

- [ ] **Step 4: Vendor only real ST sources and record the version.**

  `import_cube_sources.py` must search, in order, `STM32CUBE_F4_PATH`, common Windows STM32Cube repository locations, and a user-supplied `--cube-root`. Copy the exact HAL/CMSIS files required by the `.ioc` into the project and write `firmware/build/cube-source-manifest.json` containing source path, package version, file SHA-256, and timestamp. If no package exists, exit nonzero with the searched paths; do not generate substitute HAL code.

- [ ] **Step 5: Add the target linker/startup contract.**

  Set flash origin/size and SRAM origin/size for STM32F405RGT6, provide `_estack`, `.isr_vector`, `.text`, `.data`, `.bss`, and a zero-sized heap policy. Add a link-time assertion that `.bss + .data + stack reserve` fits in SRAM. The startup file must call `SystemInit`, copy `.data`, clear `.bss`, call `__libc_init_array`, then `main`.

- [ ] **Step 6: Add Make targets and execute the available checks.**

  Define `make target`, `make bench`, `make host-test`, `make size`, and `make clean`. `target` and `bench` must depend on the manifest and fail with a readable prerequisite message when the ARM compiler or HAL sources are absent. `host-test` must not depend on ARM tools. Run `make -C firmware host-test` after the host test runner exists; before then run `make -C firmware --dry-run target` and verify no command references an absolute user path.

**Acceptance:** The scaffold names all resources, reports the current missing toolchain honestly, has no fabricated vendor files, and contains a valid build path that becomes self-contained once a real STM32CubeF4 package is supplied.

---

### Task 2: Add platform types, fixed-memory primitives, and board-map checks

**Files:**
- Create: `firmware/Platform/status.h`
- Create: `firmware/Platform/state_types.h`
- Create: `firmware/Platform/timebase.h`, `firmware/Platform/timebase.c`
- Create: `firmware/Platform/ring_buffer.h`, `firmware/Platform/ring_buffer.c`
- Create: `firmware/Platform/state_snapshot.h`, `firmware/Platform/state_snapshot.c`
- Create: `firmware/Platform/fault_log.h`, `firmware/Platform/fault_log.c`
- Create: `firmware/Config/board_map.h`
- Create: `firmware/Config/flight_config.h`
- Create: `firmware/Config/safety_config.h`
- Create: `firmware/tests/host/test_platform.c`
- Modify: `firmware/Makefile`

**Interfaces:**
- Consumes: Task 1 build modes and the approved pin table.
- Produces: stable C types and bounded primitives used by all later modules, plus compile-time rejection of wrong MCU/pin contracts.

- [ ] **Step 1: Write failing tests for ring-buffer bounds, snapshot atomicity, and fault-log overflow.**

  Add tests that push exactly capacity bytes, reject the next byte without overwriting, publish/read two snapshots without a partial read, and verify a fixed fault log retains the newest entries while setting `overflowed=true`.

  ```c
  TEST(ring_buffer_rejects_over_capacity);
  TEST(snapshot_returns_only_published_state);
  TEST(fault_log_sets_overflow_without_allocating);
  ```

- [ ] **Step 2: Implement the platform primitives with static storage only.**

  Use caller-owned storage and `size_t` capacity. The ring buffer API is:

  ```c
  void gofly_ring_init(gofly_ring_t *ring, uint8_t *storage, size_t capacity);
  size_t gofly_ring_write(gofly_ring_t *ring, const uint8_t *data, size_t length);
  size_t gofly_ring_read(gofly_ring_t *ring, uint8_t *data, size_t length);
  size_t gofly_ring_available(const gofly_ring_t *ring);
  ```

  The snapshot API is:

  ```c
  void gofly_snapshot_init(gofly_snapshot_t *snapshot, size_t item_size);
  void gofly_snapshot_publish(gofly_snapshot_t *snapshot, const void *value);
  bool gofly_snapshot_read(const gofly_snapshot_t *snapshot, void *value);
  ```

  Reject null pointers and zero capacity with `GOFLY_E_ARGUMENT`; never call `malloc`, `free`, or a blocking primitive.

- [ ] **Step 3: Implement fixed fault records and monotonic time abstraction.**

  Define `gofly_fault_record_t` with code, source, first timestamp, last timestamp, count, and active flag. Expose `gofly_fault_log_record`, `gofly_fault_log_clear`, and `gofly_fault_log_get`. The timebase exposes `gofly_time_now_us`, `gofly_time_now_ms`, and a test-only `gofly_time_set_for_test` compiled only for host tests.

- [ ] **Step 4: Encode the board map and compile-time identity checks.**

  `board_map.h` must define each port/pin, peripheral instance, baud rate, and signal name from the global constraints. Add preprocessor checks that reject any `GPIOE` motor/signal assignment, duplicate motor pin numbers, a non-PA11/PA12 USB pair, or a non-PC0/PC1 ADC pair. Expose `GOFLY_BOARD_MAP_VERSION` and `GOFLY_MCU_ID_EXPECTED` for build banners.

- [ ] **Step 5: Add configuration units and run platform tests.**

  Define all constants with units in names: `GOFLY_IMU_RATE_HZ=1000`, `GOFLY_ATTITUDE_RATE_HZ=500`, `GOFLY_DSHOT_RATE_HZ=1000`, `GOFLY_GPS_BAUD=57600`, `GOFLY_CRSF_BAUD=420000`, `GOFLY_IMU_TIMEOUT_US=3000`, and the three output/RTH compile-time gates set to zero. Run:

  ```bash
  make -C firmware host-test TEST_FILTER=platform
  ```

  Expected: all platform tests pass and the compiler emits no dynamic-allocation symbol reference.

**Acceptance:** Later modules can include platform headers without HAL dependencies, all storage is caller-owned/fixed-size, and a wrong board resource fails at compile time.

---

### Task 3: Implement tested DShot300, CRSF, ESC telemetry, and GPS protocol parsers

**Files:**
- Create: `firmware/Drivers/Dshot300/dshot300.h`, `firmware/Drivers/Dshot300/dshot300.c`
- Create: `firmware/Drivers/Crsf/crsf_parser.h`, `firmware/Drivers/Crsf/crsf_parser.c`
- Create: `firmware/Drivers/EscTelemetry/esc_telemetry.h`, `firmware/Drivers/EscTelemetry/esc_telemetry.c`
- Create: `firmware/Drivers/Gps/gps_parser.h`, `firmware/Drivers/Gps/gps_parser.c`
- Create: `firmware/tests/host/test_dshot300.c`
- Create: `firmware/tests/host/test_crsf.c`
- Create: `firmware/tests/host/test_esc_telemetry.c`
- Create: `firmware/tests/host/test_gps_parser.c`
- Modify: `firmware/Makefile`

**Interfaces:**
- Consumes: `Platform/status.h`, `Platform/state_types.h`, `Platform/ring_buffer.h`, and the fixed timing/config constants.
- Produces: HAL-independent parsers and DShot frame generation that later hardware adapters can consume without duplicating protocol logic.

- [ ] **Step 1: Write DShot failing tests from the protocol definition.**

  Test throttle values 0, 1, 2047, and 2048 rejection; telemetry bit placement; CRC calculation; and four-frame batch construction. Assert exact 16-bit frames using the formula `packet=(throttle<<5)|(telemetry<<4)`, then `crc=(packet ^ (packet>>4) ^ (packet>>8)) & 0x0f`, and `frame=(packet<<4)|crc`.

- [ ] **Step 2: Implement DShot300 encoding and run its tests.**

  Clamp nothing silently: return `GOFLY_E_RANGE` for values outside 0–2047. Keep the frame encoder independent from timer pulse timing. Run:

  ```bash
  make -C firmware host-test TEST_FILTER=dshot
  ```

  Expected: exact frame tests pass and no HAL header is included by the DShot module.

- [ ] **Step 3: Write CRSF parser failing tests.**

  Build valid RC channel frames with a test helper, then test valid decoding, CRC rejection, truncated input, a noise prefix, frame-length rejection above 64 bytes, link-age timeout, and the failsafe flag. Assert that channel values are converted to bounded normalized values without reading beyond the supplied buffer.

- [ ] **Step 4: Implement the bounded CRSF state machine.**

  Expose:

  ```c
  void gofly_crsf_init(gofly_crsf_parser_t *parser);
  size_t gofly_crsf_feed(gofly_crsf_parser_t *parser,
                         const uint8_t *bytes, size_t length,
                         uint32_t timestamp_ms);
  bool gofly_crsf_take_input(gofly_crsf_parser_t *parser,
                             gofly_rc_input_t *input);
  bool gofly_crsf_link_is_fresh(const gofly_crsf_parser_t *parser,
                                uint32_t now_ms);
  ```

  Use a fixed 64-byte frame buffer, explicit CRC8-DVB-S2, and a discard-to-next-address recovery path.

- [ ] **Step 5: Write and implement ESC telemetry parsing.**

  Define a fixed frame structure with RPM, voltage, current, temperature, CRC, and timestamp. Reject invalid length/CRC and mark values stale after `GOFLY_ESC_TELEMETRY_TIMEOUT_MS=100`. The parser returns the newest valid record and never performs scaling with an unconfigured divisor.

- [ ] **Step 6: Write GPS protocol tests before implementation.**

  Test a valid UBX `NAV-PVT` frame, bad checksum, bad length, split delivery, noise before sync, valid NMEA GGA and RMC checksums, malformed numeric fields, and stale timestamps. Assert that both protocols produce the same `gofly_gps_solution_t` units: latitude/longitude e-7 degrees, altitude millimeters, speed millimeters/second, track e-5 degrees.

- [ ] **Step 7: Implement UBX/NMEA parsing and run all protocol tests.**

  Use a byte-wise state machine with fixed buffers. UBX `NAV-PVT` is authoritative when valid; NMEA fallback cannot overwrite a newer valid UBX solution with older data. Expose parser counters for checksum errors, framing errors, and accepted frames. Run:

  ```bash
  make -C firmware host-test TEST_FILTER=protocol
  ```

  Expected: DShot, CRSF, ESC telemetry, UBX, and NMEA tests pass, including malformed-stream cases.

**Acceptance:** Protocol modules are deterministic, bounded, independently testable, and produce no motor or navigation side effects while parsing.

---

### Task 4: Add HAL-independent sensor and peripheral device drivers

**Files:**
- Create: `firmware/Platform/bus_interfaces.h`
- Create: `firmware/Drivers/Icm42688/icm42688.h`, `firmware/Drivers/Icm42688/icm42688.c`
- Create: `firmware/Drivers/Bmp280/bmp280.h`, `firmware/Drivers/Bmp280/bmp280.c`
- Create: `firmware/Drivers/W25q128/w25q128.h`, `firmware/Drivers/W25q128/w25q128.c`
- Create: `firmware/Drivers/Max7456/max7456.h`, `firmware/Drivers/Max7456/max7456.c`
- Create: `firmware/Drivers/AdcSense/adc_sense.h`, `firmware/Drivers/AdcSense/adc_sense.c`
- Create: `firmware/tests/host/test_icm42688.c`
- Create: `firmware/tests/host/test_bmp280.c`
- Create: `firmware/tests/host/test_w25q128.c`
- Create: `firmware/tests/host/test_max7456.c`
- Create: `firmware/tests/host/test_adc_sense.c`
- Modify: `firmware/Makefile`

**Interfaces:**
- Consumes: protocol/platform types and a mockable bus contract.
- Produces: device drivers that return explicit status codes, validate IDs and ranges, and can be tested with scripted bus transactions before HAL wiring.

- [ ] **Step 1: Define mockable bus interfaces and failing mock tests.**

  Define function-pointer interfaces with explicit context and timeout:

  ```c
  typedef gofly_status_t (*gofly_spi_transfer_fn)(void *context,
                                                   const uint8_t *tx,
                                                   uint8_t *rx,
                                                   size_t length,
                                                   uint32_t timeout_ms);
  typedef gofly_status_t (*gofly_i2c_mem_read_fn)(void *context,
                                                  uint16_t address,
                                                  uint16_t reg,
                                                  uint8_t *data,
                                                  size_t length,
                                                  uint32_t timeout_ms);
  ```

  Add scripted mock implementations that can return bytes, timeout, or bus error. Tests must fail before each driver exists.

- [ ] **Step 2: Implement ICM-42688-P initialization and sample conversion.**

  `gofly_icm42688_init` must read WHO_AM_I, configure SPI mode, ODR, full-scale ranges, filters, and INT1; `gofly_icm42688_read_sample` must return SI units and a timestamp. Reject invalid WHO_AM_I, burst lengths, and stale interrupt data. Keep INT2 unassigned as the hardware contract requires.

- [ ] **Step 3: Implement BMP280 ID, calibration, and compensation.**

  `gofly_bmp280_init` reads chip ID and calibration coefficients, selects I²C address `0x76`, and configures a safe measurement mode. `gofly_bmp280_read_pressure_temperature` applies Bosch compensation formulas using bounded integer intermediates and returns Pa/°C values. Tests include the manufacturer reference calibration vector and a bus timeout.

- [ ] **Step 4: Implement W25Q128 safe SPI operations.**

  Provide `read_id`, `read_status`, `wait_ready`, `read`, `page_program`, and `sector_erase`. Enforce page-boundary splitting, 4 KiB erase alignment, write-enable sequencing, power-up delay, and BUSY polling. Tests assert that commands are not issued during the power-up inhibit window and that out-of-range writes return `GOFLY_E_RANGE`.

- [ ] **Step 5: Implement MAX7456 analog OSD register/reset interface.**

  Provide `reset`, `read_register`, `write_register`, and `configure_video_standard`. Enforce the reset minimum and SPI transaction length, record the device as `GOFLY_DEVICE_ANALOG_CVBS`, and do not expose a digital-video API. Tests verify reset sequencing and register framing with a mock SPI bus.

- [ ] **Step 6: Implement ADC measurement scaling with explicit units.**

  Define `gofly_adc_sense_config_t` with divider numerator/denominator, ADC reference millivolts, resolution, and current-sense millivolts-per-amp. Reject zero denominators and out-of-range raw samples. Tests cover zero, full-scale, calibration offset, and over-range behavior.

- [ ] **Step 7: Run all device-driver host tests.**

  ```bash
  make -C firmware host-test TEST_FILTER=device
  ```

  Expected: all mock bus tests pass, no driver includes a HAL header, and static analysis finds no unbounded loop whose exit depends on a device response without a timeout.

**Acceptance:** Sensor, storage, OSD, and ADC code has verified register/protocol behavior and can be tested entirely without a board.

---

### Task 5: Implement quaternion attitude estimation and control math

**Files:**
- Create: `firmware/Flight/vector_math.h`, `firmware/Flight/vector_math.c`
- Create: `firmware/Flight/attitude_estimator.h`, `firmware/Flight/attitude_estimator.c`
- Create: `firmware/Flight/pid_controller.h`, `firmware/Flight/pid_controller.c`
- Create: `firmware/Flight/mixer_quad_x.h`, `firmware/Flight/mixer_quad_x.c`
- Create: `firmware/Flight/flight_state.h`, `firmware/Flight/flight_state.c`
- Create: `firmware/tests/host/test_attitude.c`
- Create: `firmware/tests/host/test_pid.c`
- Create: `firmware/tests/host/test_mixer.c`
- Modify: `firmware/Makefile`

**Interfaces:**
- Consumes: `gofly_imu_sample_t`, fixed `dt` values, RC normalized commands, and configuration constants.
- Produces: normalized quaternion, attitude/rate targets, bounded PID output, and explicit Quad-X motor commands with no hardware dependency.

- [ ] **Step 1: Write failing vector/quaternion tests.**

  Cover identity multiplication, 90-degree single-axis rotation, normalization, zero/NaN rejection, and integration of a constant angular rate for one fixed step. Assert finite outputs and norm error below `1e-5` for valid inputs.

- [ ] **Step 2: Implement the vector and quaternion primitives.**

  Use `gofly_quaternion_t { float w,x,y,z; }` and explicit functions:

  ```c
  bool gofly_quaternion_normalize(gofly_quaternion_t *q);
  gofly_quaternion_t gofly_quaternion_integrate(gofly_quaternion_t q,
                                                float gyro_rad_s[3],
                                                float dt_s);
  ```

  Reject non-finite values rather than propagating them into the control loop.

- [ ] **Step 3: Write failing Mahony-style estimator tests.**

  Simulate stationary level samples, constant yaw rotation, acceleration disturbance, and stale/invalid IMU input. Assert that roll/pitch converge toward level, yaw is not claimed absolute without a magnetic sensor, and confidence decreases when acceleration magnitude leaves the configured gravity window.

- [ ] **Step 4: Implement the estimator with explicit confidence.**

  Expose:

  ```c
  void gofly_attitude_init(gofly_attitude_state_t *state);
  bool gofly_attitude_update(gofly_attitude_state_t *state,
                             const gofly_imu_sample_t *sample,
                             float dt_s);
  ```

  Integrate gyro, apply gravity correction only when acceleration quality is valid, track `yaw_confidence`, `tilt_confidence`, and `sample_age_us`, and clear the state on invalid numerical results.

- [ ] **Step 5: Write failing PID tests.**

  Test proportional response, integral accumulation, integral clamp, derivative response, output clamp, reset, and saturation handling. Use fixed `dt` and assert no state changes when `dt<=0` or the input is non-finite.

- [ ] **Step 6: Implement the PID controller and rate/angle cascade.**

  Define `gofly_pid_update` with explicit limits and a `saturated` argument. Implement Rate mode directly from RC rate targets and Angle mode as an outer proportional angle-to-rate conversion. Keep yaw as rate control by default; do not implement a fake absolute yaw hold.

- [ ] **Step 7: Write failing Quad-X mixer tests and implement the mixer.**

  Test throttle-only equality, roll/pitch/yaw sign changes, minimum/maximum saturation, and configured motor order. Use a `gofly_mixer_config_t` containing four rotation signs and an explicit physical index map. Run:

  ```bash
  make -C firmware host-test TEST_FILTER=flight
  ```

  Expected: all estimator, PID, and mixer tests pass with deterministic floating-point tolerances.

**Acceptance:** The control math is numerically bounded, has no HAL dependency, exposes confidence/health, and cannot silently produce NaN or out-of-range motor commands.

---

### Task 6: Implement GPS health, WGS-84 local NED, home, and RTH intent

**Files:**
- Create: `firmware/Navigation/geo_ned.h`, `firmware/Navigation/geo_ned.c`
- Create: `firmware/Navigation/gps_health.h`, `firmware/Navigation/gps_health.c`
- Create: `firmware/Navigation/home.h`, `firmware/Navigation/home.c`
- Create: `firmware/Navigation/rth_state.h`, `firmware/Navigation/rth_state.c`
- Create: `firmware/tests/host/test_geo_ned.c`
- Create: `firmware/tests/host/test_home.c`
- Create: `firmware/tests/host/test_rth.c`
- Modify: `firmware/Makefile`

**Interfaces:**
- Consumes: `gofly_gps_solution_t`, attitude confidence, RC/failsafe state, battery health, and monotonic time.
- Produces: local NED coordinates, home quality, RTH state and `gofly_rth_intent_t`; it never sends motor commands.

- [ ] **Step 1: Write failing WGS-84 conversion tests.**

  Test identical coordinates, a known small north/east offset, high-latitude longitude scaling, invalid latitude/longitude, and a distance below one millimeter. Compare against fixed reference values with a documented tolerance of `0.05 m` for the local approximation.

- [ ] **Step 2: Implement local NED conversion and bearing.**

  Use a stored home latitude/longitude in e-7 degrees and convert to radians before applying Earth-radius and latitude-cosine scaling. Return `GOFLY_E_RANGE` for invalid coordinates and never divide by a near-zero cosine without returning invalid.

- [ ] **Step 3: Write failing GPS health/home tests.**

  Test the required consecutive stable samples for `HOME_CANDIDATE`, lock once, reject insufficient fix/accuracy/satellites, reject stale samples, and reject a position jump beyond the configured maximum step. Verify that a locked home does not move when new GPS samples arrive.

- [ ] **Step 4: Implement health and home managers.**

  Define explicit thresholds in `safety_config.h`, including fix type, minimum satellites, maximum horizontal accuracy, and maximum sample age. Expose `gofly_home_update`, `gofly_home_is_valid`, and `gofly_gps_health_update`; every rejection carries a reason enum for telemetry/tests.

- [ ] **Step 5: Write failing RTH state-transition tests.**

  Cover GPS unavailable, home missing, valid request, climb intent, return intent, arrival radius, cancellation, timeout, RC loss, low battery, low yaw confidence, and explicit abort. Assert that the returned intent is marked invalid in disallowed states and that `GOFLY_RTH_REAL_OUTPUT` remains false.

- [ ] **Step 6: Implement the observation-first RTH state machine.**

  `gofly_rth_update` must accept a complete input snapshot and return state, reason, intent, and `real_output_allowed`. It can generate target altitude, direction, and distance for HIL, but `real_output_allowed` must be compile-time false under the approved default configuration. Missing magnetometer/yaw confidence routes to `GOFLY_RTH_ABORTED` or an observation-only reason rather than silently continuing.

- [ ] **Step 7: Run navigation tests.**

  ```bash
  make -C firmware host-test TEST_FILTER=navigation
  ```

  Expected: all geographic, home, health, and RTH tests pass without HAL or GPS serial code.

**Acceptance:** Navigation calculations are unit-correct and deterministic, home is never silently replaced, and RTH produces observable intent without authorizing real flight output.

---

### Task 7: Implement safety aggregation, failsafe, and output gating

**Files:**
- Create: `firmware/Safety/health_monitor.h`, `firmware/Safety/health_monitor.c`
- Create: `firmware/Safety/failsafe.h`, `firmware/Safety/failsafe.c`
- Create: `firmware/Safety/output_gate.h`, `firmware/Safety/output_gate.c`
- Create: `firmware/tests/host/test_safety.c`
- Modify: `firmware/Config/safety_config.h`
- Modify: `firmware/Makefile`

**Interfaces:**
- Consumes: sensor freshness, attitude state, RC link, battery/ESC health, scheduler deadlines, compile-time gates, and DShot backend state.
- Produces: prioritized fault records and a single boolean output authorization used by every motor-output path.

- [ ] **Step 1: Write failing safety tests before implementation.**

  Test each condition independently and in combinations. The expected priority is hardware/clock, IMU stale, control deadline, RC loss, GPS loss, battery fault, then ordinary requests. Assert that any high-priority condition clears authorization and that clearing a fault does not automatically re-arm.

- [ ] **Step 2: Implement fault causes and health aggregation.**

  Define a bitmask enum with named causes such as `GOFLY_FAULT_CLOCK`, `GOFLY_FAULT_IMU_STALE`, `GOFLY_FAULT_CONTROL_DEADLINE`, `GOFLY_FAULT_RC_LOSS`, `GOFLY_FAULT_GPS_INVALID`, `GOFLY_FAULT_BATTERY`, and `GOFLY_FAULT_DSHOT_BACKEND`. Store active/latched state in the fixed fault log.

- [ ] **Step 3: Implement the output gate as the only authorization boundary.**

  `gofly_output_is_authorized` must require compile-time flight enable, non-bench mode, self-test success, fresh IMU, valid attitude, fresh RC, explicit arm request, healthy DShot backend, and no high-priority fault. `gofly_output_apply_gate(false, &command)` must overwrite all four throttle values and telemetry requests with zero/false.

- [ ] **Step 4: Add failsafe latching and explicit recovery.**

  RC timeout, IMU timeout, scheduler overrun, and battery critical faults latch until disarm and a fresh self-test/RC validity sequence. Expose fault reason and recovery phase for USB/telemetry. Never perform a hidden automatic re-arm.

- [ ] **Step 5: Run safety tests and inspect symbols.**

  ```bash
  make -C firmware host-test TEST_FILTER=safety
  nm -u firmware/build/host-test 2>/dev/null | findstr /I "malloc free calloc realloc" || true
  ```

  Expected: all safety tests pass; the host binary has no allocator dependency from firmware modules.

**Acceptance:** There is exactly one tested gate between control output and motor backend, and all failure paths deterministically produce safe zero commands.

---

### Task 8: Add HAL transport adapters and peripheral initialization

**Files:**
- Create: `firmware/Platform/hal_bus_adapters.h`, `firmware/Platform/hal_bus_adapters.c`
- Create: `firmware/Drivers/Bsp/board_bsp.h`, `firmware/Drivers/Bsp/board_bsp.c`
- Create/modify: `firmware/Core/Inc/main.h`, `firmware/Core/Inc/stm32f4xx_it.h`
- Create/modify: `firmware/Core/Src/main.c`, `firmware/Core/Src/stm32f4xx_hal_msp.c`, `firmware/Core/Src/stm32f4xx_it.c`
- Create/modify: `firmware/Core/Inc/stm32f4xx_hal_conf.h`
- Create: `firmware/Core/Src/system_checks.c`
- Modify: `firmware/gofly_fc.ioc`
- Modify: `firmware/Makefile`

**Interfaces:**
- Consumes: tested HAL-independent drivers and Task 1 Cube configuration.
- Produces: bounded SPI/I²C/UART/ADC/TIM/EXTI adapters, safe GPIO startup, DMA callbacks that only enqueue events, and a target build with no application-layer HAL leakage.

- [ ] **Step 1: Add host-side adapter contract tests.**

  Create fake adapter tests that verify a timeout maps to `GOFLY_E_TIMEOUT`, HAL error maps to `GOFLY_E_BUS`, and a DMA callback writes only the expected ring buffer/event flag. These tests compile the adapter contract with `GOFLY_HOST_TEST` and do not include STM32 headers.

- [ ] **Step 2: Implement safe board startup.**

  Set motor pins to push-pull low before enabling timers, set chip-selects high, hold OSD reset asserted, configure status LED and buzzer safe states, initialize clocks, then initialize buses. Add `gofly_board_safe_outputs()` and call it before any peripheral start.

- [ ] **Step 3: Implement SPI/I²C adapters with explicit ownership.**

  Each adapter receives a bus mutex-free ownership token because the first version is single-threaded. Use HAL timeout arguments from configuration, convert HAL status, and assert that a transfer length is nonzero and within the device-specific maximum before calling HAL.

- [ ] **Step 4: Implement UART DMA/interrupt adapters.**

  Configure circular or idle-line receive buffers for CRSF, GPS, and ESC telemetry. Call only `gofly_ring_write` and set a timestamp in `HAL_UARTEx_RxEventCallback`/error callbacks. Parsing happens in scheduled tasks, never in the ISR.

- [ ] **Step 5: Implement ADC and IMU EXTI event adapters.**

  Start ADC scan/DMA for PC0/PC1 and publish raw samples; EXTI4 sets an IMU-data-ready event with a timestamp. Reject samples after the configured age and record overrun counts.

- [ ] **Step 6: Validate generated MSP/DMA code before enabling target DShot.**

  Inspect Cube output for each selected peripheral, verify no pin collision, verify the TIM3 DMA stream/request assignments are unique and supported by the exact HAL package, and write `firmware/build/dma-contract.json`. If any item is not verified, set `GOFLY_DSHOT_HW_BACKEND=0` and keep the safe observation backend.

- [ ] **Step 7: Build the target or record the exact blocker.**

  ```bash
  make -C firmware target
  ```

  Expected with a complete environment: successful ELF and map output with no unresolved HAL symbols. If the ARM compiler or Cube package is still absent, preserve the probe report and state that target compilation is not available; run the host adapter tests instead.

**Acceptance:** Hardware access is isolated behind bounded adapters, startup cannot pulse motors, and DMA is either verified from generated configuration or explicitly disabled.

---

### Task 9: Implement DShot300 TIM3 backend and bench-safe motor output

**Files:**
- Create: `firmware/Drivers/Dshot300/dshot_hal_backend.h`, `firmware/Drivers/Dshot300/dshot_hal_backend.c`
- Create: `firmware/Drivers/Dshot300/dshot_bench_backend.h`, `firmware/Drivers/Dshot300/dshot_bench_backend.c`
- Create: `firmware/tests/host/test_dshot_backend.c`
- Modify: `firmware/Config/safety_config.h`
- Modify: `firmware/gofly_fc.ioc`
- Modify: `firmware/Makefile`

**Interfaces:**
- Consumes: `gofly_dshot300_build`, `gofly_output_apply_gate`, TIM3/DMA contract, and board motor mapping.
- Produces: four-channel DShot backend with a bench implementation that captures frames and a target implementation that refuses to start until the DMA contract is verified.

- [ ] **Step 1: Write failing backend tests.**

  Test that a disabled backend captures zeroed frames, an unauthorized command is zeroed before transmission, four channels map to PC6–PC9 in configured order, and a DMA completion/error transitions backend health correctly.

- [ ] **Step 2: Implement the bench backend.**

  Store the last four encoded frames in caller-provided memory, expose `gofly_dshot_bench_last_frames`, and increment a transmission counter. The bench backend must never access GPIO or HAL and must be the default for host tests and `bench` firmware mode.

- [ ] **Step 3: Implement target pulse-buffer generation.**

  Convert each 16-bit frame to a fixed 16-slot duty buffer plus reset slots using constants derived from the verified TIM3 clock. Reject clocks or buffer sizes that do not meet DShot300 timing tolerance. Do not use variable-length arrays or heap allocation.

- [ ] **Step 4: Implement the TIM3/DMA adapter behind a compile-time gate.**

  Expose `gofly_dshot_hal_init`, `gofly_dshot_hal_send`, `gofly_dshot_hal_is_healthy`, and `gofly_dshot_hal_on_error`. When `GOFLY_DSHOT_HW_BACKEND=0`, `init` returns `GOFLY_E_UNSUPPORTED` and no HAL timer start is attempted. When enabled, assert the generated DMA handles and PC6–PC9 mapping before starting.

- [ ] **Step 5: Run backend tests and inspect generated timing data.**

  ```bash
  make -C firmware host-test TEST_FILTER=dshot-backend
  ```

  Expected: bench safety tests pass. Target timing is only reported as verified if a real compiler/package build and the DMA contract report both exist.

**Acceptance:** DShot frames are correct and observable on the bench path; no build or runtime condition can accidentally enable real motor output without an explicit, reviewed gate.

---

### Task 10: Add the fixed-rate scheduler, boot self-test, and application integration

**Files:**
- Create: `firmware/App/scheduler.h`, `firmware/App/scheduler.c`
- Create: `firmware/App/state_snapshot.h`, `firmware/App/state_snapshot.c`
- Create: `firmware/App/parameters.h`, `firmware/App/parameters.c`
- Create: `firmware/App/telemetry.h`, `firmware/App/telemetry.c`
- Create: `firmware/App/app_main.h`, `firmware/App/app_main.c`
- Create: `firmware/App/usb_console.h`, `firmware/App/usb_console.c`
- Create: `firmware/tests/host/test_scheduler.c`
- Modify: `firmware/Core/Src/main.c`
- Modify: `firmware/Makefile`

**Interfaces:**
- Consumes: all drivers, estimator/controller, navigation, safety gate, and HAL event queues.
- Produces: a deterministic boot sequence and fixed-period application loop that can run in bench mode and expose status without bypassing safety.

- [ ] **Step 1: Write scheduler and boot-order failing tests.**

  Test that each task runs at its configured period, a missed deadline increments an overrun fault, task execution does not run twice for one tick, and boot self-test failure leaves output authorization false. Assert the order: safe GPIO, clocks, buses, device IDs, sensor configuration, state initialization, scheduler start.

- [ ] **Step 2: Implement the scheduler.**

  Use a 1 kHz TIM6 event counter and a main-loop dispatcher. Schedule IMU/control at 1 kHz, attitude at 500 Hz, CRSF at 200 Hz, GPS at 100 Hz, health/ADC/ESC at 50 Hz, and telemetry/parameters at 10 Hz. Each task receives a deadline timestamp and returns a status; no task can block indefinitely.

- [ ] **Step 3: Implement boot self-test and application state snapshots.**

  Self-test checks MCU identity, bus availability, IMU WHO_AM_I, BMP280 ID, flash ID/status, ADC range, and DShot backend state. Publish immutable snapshots for RC, IMU, GPS, attitude, battery, faults, and RTH. A failed check records a named fault and keeps the system disarmed.

- [ ] **Step 4: Implement the control/navigation pipeline.**

  The 1 kHz task consumes the newest IMU/RC snapshots, updates the estimator and rate controller, runs the Quad-X mixer, applies battery/throttle limits, evaluates RTH intent only as an observation target, calls the output gate, and sends the resulting command to the selected DShot backend. The task must clear PID state when the IMU or control deadline is invalid.

- [ ] **Step 5: Add bounded USB console and telemetry status.**

  Implement line-buffered commands `GET STATUS`, `GET FAULTS`, `GET GPS`, `GET RTH`, and `CLEAR FAULTS` with a maximum line length of 96 bytes. Commands can inspect state but cannot enable `GOFLY_FLIGHT_ENABLE`, `GOFLY_REAL_OUTPUT`, or `GOFLY_RTH_REAL_OUTPUT` at runtime. Return machine-readable key/value lines with timestamps and fault reasons.

- [ ] **Step 6: Run scheduler tests and host integration.**

  ```bash
  make -C firmware host-test TEST_FILTER=app
  ```

  Expected: scheduler, boot-order, output-gate, and status-console tests pass with deterministic simulated time.

**Acceptance:** The complete software pipeline runs at fixed rates with bounded work, self-test failures are fail-safe, and inspection interfaces cannot weaken compile-time safety.

---

### Task 11: Add deterministic HIL scenarios for failsafe and observation-only RTH

**Files:**
- Create: `firmware/tests/hil/hil_runner.c`
- Create: `firmware/tests/hil/scenarios/rc_loss.json`
- Create: `firmware/tests/hil/scenarios/gps_home_rth.json`
- Create: `firmware/tests/hil/scenarios/imu_timeout.json`
- Create: `firmware/tests/hil/scenarios/low_battery.json`
- Create: `firmware/tests/hil/expected_results.json`
- Create: `firmware/tools/run_hil.py`
- Modify: `firmware/Makefile`
- Modify: `firmware/README.md`

**Interfaces:**
- Consumes: the integrated application API and all host-testable modules.
- Produces: repeatable event logs and pass/fail results for the safety-critical scenarios without requiring motors, GPS, or a flight battery.

- [ ] **Step 1: Define scenario schemas and failing runner tests.**

  Each JSON scenario contains `name`, `step_us`, `duration_ms`, initial sensor/RC/GPS state, and timed events. The runner must reject unknown event types, negative time, and a missing expected result before executing. Add a test for malformed JSON/schema input.

- [ ] **Step 2: Implement the fixed-step HIL harness.**

  Feed synthetic IMU, CRSF, GPS, ADC, and ESC telemetry samples into the integrated app at a fixed step. Capture output authorization, four motor values, fault masks, home state, RTH state, and intent validity at every event boundary. Use the bench DShot backend only.

- [ ] **Step 3: Add and assert the four required scenarios.**

  - `rc_loss`: valid bench state followed by RC timeout; expected output authorization false, all motor frames zero, and `GOFLY_FAULT_RC_LOSS` active.
  - `gps_home_rth`: stable GPS locks home, RTH request creates valid observation intent, but real-output authorization remains false because compile-time RTH output is disabled and yaw confidence is bounded.
  - `imu_timeout`: missing IMU samples clear PID/motor outputs and latch `GOFLY_FAULT_IMU_STALE`.
  - `low_battery`: voltage below critical threshold blocks arming and records `GOFLY_FAULT_BATTERY`.

- [ ] **Step 4: Run HIL and compare machine-readable results.**

  ```bash
  make -C firmware hil
  python firmware/tools/run_hil.py --scenarios firmware/tests/hil/scenarios \
      --expected firmware/tests/hil/expected_results.json
  ```

  Expected: all four scenarios pass; a failure prints the first timestamp, state, fault mask, and motor command that diverged.

**Acceptance:** The integrated code demonstrates deterministic safe behavior for link loss, sensor loss, low battery, and observation-only RTH without claiming real flight capability.

---

### Task 12: Complete target verification, documentation, and final review artifacts

**Files:**
- Modify: `firmware/README.md`
- Modify: `firmware/tools/probe_toolchain.py`
- Create: `firmware/tools/verify_contracts.py`
- Create: `firmware/build/verification-report.json`
- Create: `firmware/build/host-test-report.txt`
- Create: `firmware/build/hil-report.json`
- Create: `firmware/build/target-build-report.json` when target tools are available
- Create: `firmware/docs/bring-up-checklist.md`
- Create: `firmware/docs/pin-dma-review.md`

**Interfaces:**
- Consumes: all prior source, test, toolchain, Cube manifest, DMA contract, and HIL outputs.
- Produces: an evidence-backed handoff that distinguishes verified software behavior from hardware and toolchain gaps.

- [ ] **Step 1: Run the narrow host verification suite.**

  ```bash
  make -C firmware host-test
  make -C firmware hil
  ```

  Save test names, counts, compiler version, and exit status to the host/HIL reports. A report with a skipped test must name the missing prerequisite and cannot claim a full pass.

- [ ] **Step 2: Run target compilation if the probe permits it.**

  ```bash
  make -C firmware bench
  make -C firmware size
  ```

  Record ELF path, map path, text/data/bss sizes, compiler version, HAL manifest hash, board-map version, safety flags, and DShot backend status. If the probe still reports no ARM compiler or Cube source, record `target_build_status: unavailable` and preserve the exact probe reason.

- [ ] **Step 3: Perform static and contract checks.**

  Check that no `Flight/`, `Navigation/`, `Safety/`, or protocol parser file includes STM32 HAL headers; search for allocator symbols; verify all configured task periods divide 1000 Hz; verify every board-map signal appears once; verify `GOFLY_FLIGHT_ENABLE`, `GOFLY_REAL_OUTPUT`, and `GOFLY_RTH_REAL_OUTPUT` are zero in the default configuration.

  ```bash
  python firmware/tools/verify_contracts.py --firmware firmware
  ```

  `verify_contracts.py` must fail on PE pins, raw VBAT MCU connections in board-map metadata, duplicate motor pins, missing safety macros, unbounded protocol buffer declarations, or an enabled real-output default.

- [ ] **Step 4: Review the bring-up checklist against the actual hardware state.**

  Document the safe sequence: visual inspection, continuity/current-limited power, rail checks, SWD, clock/USB, IMU/BMP280/flash IDs, UART fixtures, DShot logic analyzer, no-prop ESC test, then controlled sensor/RTH observation. Include explicit checks for USB VBUS backfeed, TPS54360 transients, receiver/ESC/GPS/VTX levels, and MAX7456 5 V SDOUT protection.

- [ ] **Step 5: Produce the final verification report.**

  Include actual host/HIL/target counts, missing tools, unverified DMA mapping, absent magnetometer limitation, PCB DRC/placement status from the existing project ledger, and a prominent statement that the result is not flight-ready. Do not mark a hardware item verified from source code alone.

**Acceptance:** The handoff contains reproducible tests and honest evidence. A missing embedded toolchain or hardware test appears as an explicit limitation rather than a false success.

---

## Execution Order and Checkpoints

1. Complete Task 1 and Task 2 before any device or control implementation; these establish build modes and shared contracts.
2. Complete Tasks 3–7 as host-testable modules. Run the focused test group after each task, then run the complete host suite before HAL integration.
3. Complete Task 8 only after the HAL/Cube source manifest exists or the target-build blocker is recorded.
4. Complete Tasks 9–10 with the DShot hardware backend disabled unless the DMA contract is verified.
5. Complete Task 11 before claiming integrated safety behavior.
6. Complete Task 12 as the final evidence pass; do not modify the KiCad project as a substitute for a firmware test.

## Spec Coverage Review

- Spec purpose and explicit exclusions: Tasks 1, 10, and 12.
- Exact STM32 resource allocation and clock: Tasks 1, 2, and 8.
- No-RTOS/fixed-memory architecture: Tasks 2, 8, and 10.
- ICM-42688-P, BMP280, W25Q128JV, MAX7456, and ADC drivers: Task 4.
- DShot300 and separate ESC telemetry: Tasks 3 and 9.
- CRSF half-duplex behavior: Task 3 and Task 8.
- UBX/NMEA GPS and 57600 default: Tasks 3, 6, and 8.
- Quaternion/Mahony-style estimator, PID, Quad-X mixer: Task 5.
- Home, local NED, RTH intent and no-magnetometer limit: Task 6.
- Failsafe, fault priority, output authorization: Task 7 and Task 11.
- USB status/control boundary: Task 10.
- Toolchain, HAL/CMSIS provenance, target build evidence: Tasks 1 and 12.
- Bring-up and non-flight-ready limitations: Task 12.

## Plan Self-Review

- No step relies on an undefined function name: shared signatures are listed before Task 1 and each later API is introduced in its task.
- No task requires a real motor or battery to pass host/HIL tests.
- No task silently enables real output; the default gates remain zero in every build mode.
- No vendor HAL source is invented when the package is unavailable.
- No task changes the existing KiCad files or initializes a repository.
- The plan contains no `TODO`, `TBD`, or vague “handle edge cases” instruction; each failure condition has a named result, test, or explicit safe behavior.
