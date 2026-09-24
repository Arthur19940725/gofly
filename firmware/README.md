# gofly_fc firmware scaffold

This directory is the bench-safe firmware project for the STM32F405RGT6 research flight controller. It is not flight-certified, production-ready, or evidence that the current PCB can be powered with a battery, fitted with propellers, or used for autonomous return-to-home.

## Safety defaults

The default configuration must keep all of these values at zero:

```text
GOFLY_FLIGHT_ENABLE=0
GOFLY_REAL_OUTPUT=0
GOFLY_RTH_REAL_OUTPUT=0
GOFLY_DSHOT_HW_BACKEND=0
```

The firmware must zero all four motor commands before any output backend when authorization is false. Bench and host paths must not access motor GPIOs.

## Board baseline

- MCU: STM32F405RGT6, LQFP64
- HSE: 8 MHz; intended SYSCLK: 168 MHz; USB clock: 48 MHz
- IMU SPI1: PA5/PA6/PA7, CS PB0, INT1 PC4
- Flash SPI3: PB3/PB4/PB5, CS PB1
- MAX7456 SPI2: PB13/PB14/PB15, CS PB12, reset PA1; analog 5 V CVBS only
- BMP280 I2C1: PB8/PB9, address 0x76
- CRSF USART1: PA9/PA10, half-duplex, 420000 baud
- GPS USART3: PB10/PB11, 57600 baud
- VTX MSP UART4: PC10/PC11
- ESC telemetry USART2: PA2/PA3
- Motors TIM3: PC6/PC7/PC8/PC9
- ADC: PC0 battery sense and PC1 current sense; never raw VBAT
- USB FS: PA11/PA12; SWD: PA13/PA14

The `.ioc` file records the intended CubeMX configuration. It must be regenerated or validated with the exact STM32CubeF4 release used for the build; this workstation currently has no CubeMX executable.

## Toolchain provenance

Run the non-mutating probe:

```bash
python firmware/tools/probe_toolchain.py --output firmware/build/toolchain-report.json
```

Importing vendor sources is deliberately fail-closed and never creates substitute HAL/CMSIS code:

```bash
python firmware/tools/import_cube_sources.py --firmware-root firmware --manifest firmware/build/cube-source-manifest.json
```

The manifest must record the STM32CubeF4 package path/version and SHA-256 for every copied file. Do not commit machine-specific credentials or rely on undocumented global include paths.

## Build entry points

```bash
make -C firmware host-test
make -C firmware target
make -C firmware bench
make -C firmware size
make -C firmware clean
```

`host-test` is HAL-independent. `target` and `bench` require `arm-none-eabi-gcc`, GNU Make, a real Cube source manifest, and the target linker/startup inputs. If those prerequisites are absent, the command must report `target_build_status: unavailable`; no target result may be fabricated.

## Hardware bring-up boundary

The required order is current-limited visual/continuity inspection, rail checks, SWD, clock/USB, sensor IDs, flash status, UART fixtures, DShot logic analysis, and no-prop ESC testing. TIM3/DMA mapping, power transient behavior, external UART electrical levels, USB VBUS backfeed, and MAX7456 SDOUT level protection remain hardware verification gates. The absence of a magnetometer limits yaw confidence and keeps real autonomous RTH outside the default authorization path.
