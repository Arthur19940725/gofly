# 5-inch FPV Flight Controller Research Prototype

**Project:** `gofly_fc`  
**Date:** 2026-09-20  
**Status:** approved design specification; the KiCad project scaffold exists, while schematic and PCB capture remain incomplete  
**Scope:** a 40 mm four-layer research flight-controller board for a 5-inch quadcopter

## 1. Purpose and boundary

This design is a research prototype, not a flight-certified or production-ready flight controller. It provides the common electrical building blocks and external interfaces used by a 5-inch FPV quadcopter:

- STM32F405RGT6 flight-controller MCU;
- ICM-42688-P inertial measurement unit;
- BMP280 barometer;
- W25Q128JV 128-Mbit SPI NOR flash;
- MAX7456 analog CVBS OSD path;
- 4–6S battery input protection and 5 V conversion;
- separate main and IMU 3.3 V rails;
- 4-in-1 ESC motor/control, current, battery-sense, and telemetry signals;
- CRSF/ELRS receiver bus;
- GPS UART plus the shared I²C bus;
- USB-C USB Full-Speed device interface;
- SWD, buzzer, status LED, and LED-strip connections;
- UART4 MSP control interface for an external digital VTX.

The digital-VTX interface is deliberately limited to UART4 MSP control, power pins supplied by the selected external VTX/BEC, and ground. The board does not generate, serialize, or route MIPI, HDMI, SDI, or another high-speed digital-video stream. MAX7456 is retained only for the separate analog-CVBS OSD path, and its obsolete lifecycle is an explicit prototype risk.

The source record for component and interface constraints is [FPV flight-controller primary-source research](../../../research/fpv-flight-controller-sources.md). The claims below are tied to the official sources listed there, especially the ST, TI, TDK/InvenSense, Bosch, Winbond, Analog Devices, USB-IF, Betaflight, and iNav documents.

## 2. Approved architecture

### 2.1 Power tree

```text
VBAT_4S6S
  -> input fuse / current limiter
  -> reverse-polarity protection MOSFET stage
  -> battery-rated TVS and bulk/ceramic filtering
  -> TPS54360 buck regulator
  -> SYS_5V
       -> USB VBUS OR-ing path
       -> MAIN_3V3 low-noise LDO
       -> IMU_3V3 dedicated low-noise LDO
```

The protected input network is a system design, not a consequence of the TPS54360 absolute maximum rating. The fully charged pack voltage, hot-plug overshoot, wiring inductance, ESC noise, battery disconnect behavior, TVS clamping voltage, fuse/current limit, and measured converter-pin waveform must be specified and tested before flight use.

`SYS_5V` and USB-C `VBUS` are combined only after independent current limiting and reverse-current blocking. USB VBUS must not be back-fed from the battery converter. USB-C is a sink-only debug/data input in this revision; no USB-PD controller, host power output, or battery charger is included.

### 2.2 Rail ownership

| Rail | Loads | Required constraints |
|---|---|---|
| `SYS_5V` | MAX7456, external-interface power where appropriate, buzzer/VTX auxiliary power as explicitly documented | TPS54360 5 V feedback network, diode, inductor, compensation, input loop, and thermal design must follow TI guidance |
| `MAIN_3V3` | STM32F405, W25Q128JV, BMP280, digital interface protection/translation | 3.3 V regulator must have sufficient transient current and thermal margin from 5 V input |
| `IMU_3V3` | ICM-42688-P VDD and VDDIO | independent low-noise LDO; local 0.1 µF + 2.2 µF on VDD and 10 nF on VDDIO; short return to the local ground plane |
| `USB_VBUS_RAW` | USB-C VBUS detection and USB input path before OR-ing | current-limited and isolated from `SYS_5V`; never used as an unprotected board rail |

No external signal is assumed to be 5 V-safe merely because its connector is powered from 5 V. Each selected receiver, GPS, ESC, and VTX must be checked against the actual hardware electrical specification.

## 3. MCU resource allocation

The allocation below is for the STM32F405RGT6 in the LQFP64 package. It intentionally avoids PE pins, which are not available on this package. It uses the package-specific alternate-function table in the ST STM32F405xx/STM32F407xx datasheet, DS8626 Rev. 12.

### 3.1 Core and mandatory service pins

| Signal | STM32 pin | Function / AF | Notes |
|---|---|---|---|
| `HSE_IN` | PH0 | OSC_IN | external 8 MHz crystal |
| `HSE_OUT` | PH1 | OSC_OUT | external 8 MHz crystal |
| `USB_DM` | PA11 | USB_OTG_FS_DM | USB-C D− pair, ESD at connector |
| `USB_DP` | PA12 | USB_OTG_FS_DP | USB-C D+ pair, ESD at connector |
| `SWDIO` | PA13 | JTMS/SWDIO | SWD header/test pads |
| `SWCLK` | PA14 | JTCK/SWCLK | SWD header/test pads |
| `NRST` | NRST | reset | SWD, reset switch/test point, supervisor network |
| `BOOT0` | BOOT0 | boot configuration | 100 kΩ pulldown and momentary boot/test access |
| `VCAP1` | pin 31, VCAP_1 | internal regulator capacitor | 2.2 µF low-ESR ceramic to GND |
| `VCAP2` | pin 47, VCAP_2 | internal regulator capacitor | 2.2 µF low-ESR ceramic to GND |
| `VDDA/VSSA` | package analog supply pins | analog supply/return | filtered and decoupled separately from noisy switching return |

The LQFP64 package pinout assigns VCAP_1 to pin 31 and VCAP_2 to pin 47. Do not copy a VCAP assignment from a larger STM32F405 package; verify the remaining power pins against the selected ST datasheet revision during symbol/footprint review.

### 3.2 SPI and sensor allocation

| Bus | Signals | STM32 pins | Device |
|---|---|---|---|
| `SPI1` | `IMU_SCK/MISO/MOSI` | PA5 / PA6 / PA7, AF5 | ICM-42688-P |
| `SPI3` | `FLASH_SCK/MISO/MOSI` | PB3 / PB4 / PB5, AF6 | W25Q128JV |
| `SPI2` | `OSD_SCK/MISO/MOSI` | PB13 / PB14 / PB15, AF5 | MAX7456 |
| `I2C1` | `I2C1_SCL/SDA` | PB8 / PB9, AF4 | BMP280 and GPS expansion bus |

Chip-selects are GPIOs and are not treated as hardware NSS requirements:

| Signal | STM32 pin |
|---|---|
| `IMU_CS` | PB0 |
| `FLASH_CS` | PB1 |
| `OSD_CS` | PB12 |
| `OSD_RESET` | PA1 |
| `IMU_INT1` | PC4 |

The ICM-42688-P symbol uses the selected DS-000347 Rev. 1.9 pin table: INT1 is pin 4 and INT2/FSYNC/CLKIN is pin 9. INT1 is the primary interrupt routed to `PC4 / IMU_INT1`; INT2 remains an optional test pad rather than an invented MCU interrupt route. Firmware must still configure and verify interrupt polarity and drive mode.

The BMP280 is wired for I²C at address `0x76`: `CSB` is tied to `MAIN_3V3`, `SDO` is tied to GND, and SDA/SCL have pull-ups to `MAIN_3V3`. The interface pins must not be driven high while VDDIO is off; this revision therefore keeps the BMP280 rail permanently present whenever the MCU rail is present and does not implement independent barometer power gating.

The W25Q128JV is initially used in standard SPI mode. `/WP` and `/HOLD`/`/RESET` have defined pull states and test pads. Quad-SPI expansion is reserved in the symbol/net naming but is not required for the first board population.

### 3.3 Serial allocation

| Function | STM32 pins | Peripheral | Electrical implementation |
|---|---|---|---|
| CRSF/ELRS receiver | PA9 / `CRSF_BUS` | USART1_TX in single-wire half-duplex mode | one protected half-duplex bus; actual receiver voltage, inversion, and pull behavior must be confirmed |
| optional receiver sense/test | PA10 | USART1_RX | optional footprint/test access; not required for the one-wire production connection |
| GPS | PB10 / PB11 | USART3_TX/RX, AF7 | `GPS_TX`, `GPS_RX`, plus 5 V/GND and I²C expansion |
| digital VTX MSP | PC10 / PC11 | UART4_TX/RX, AF8 | `VTX_MSP_TX`, `VTX_MSP_RX`, external VTX/BEC 5 V and GND |
| ESC telemetry | PA2 / PA3 | USART2_TX/RX, AF7 | `ESC_TELEM_RX` is the required input; PA2 is exposed for optional bidirectional/diagnostic use |

The earlier proposal of using PC6/PC7 for CRSF is rejected because PC6–PC9 are reserved as a coherent four-channel motor timer group in this package-level allocation. The earlier use of `PE9/PE11/PE13/PE14` is also rejected because those pins are not present on the LQFP64 package.

### 3.4 Motor outputs and ADCs

| Signal | STM32 pin | Timer / AF | Notes |
|---|---|---|---|
| `MOTOR1` | PC6 | TIM3_CH1, AF2 | DShot-capable timer output candidate |
| `MOTOR2` | PC7 | TIM3_CH2, AF2 | DShot-capable timer output candidate |
| `MOTOR3` | PC8 | TIM3_CH3, AF2 | DShot-capable timer output candidate |
| `MOTOR4` | PC9 | TIM3_CH4, AF2 | DShot-capable timer output candidate |
| `VBAT_SENSE` | PC0 | ADC1/ADC2/ADC3 input | divider, RC filter, and input-current protection |
| `CURRENT_SENSE` | PC1 | ADC1/ADC2/ADC3 input | ESC sensor scaling and RC filter must be selected from the ESC specification |
| `USB_VBUS_SENSE` | PA8 | GPIO monitoring input | resistor divider or comparator network, never direct 5 V into the MCU |
| `BUZZER_CTRL` | PC5 | GPIO/timer-capable | low-side transistor/MOSFET driver; no buzzer current through MCU pin |
| `LED_STRIP` | PA0 | GPIO/timer-capable | series resistor and external 5 V strip power |
| `STATUS_LED` | PC13 | GPIO | current-limited indicator LED |

The PC6–PC9 group is a valid package-level timer grouping for the first schematic and PCB. The firmware implementation must still verify the exact timer DMA stream/request mapping and DShot mode for the selected STM32F405 firmware version before claiming bidirectional DShot support. The board exposes a separate ESC telemetry input so the prototype does not depend on bidirectional DShot for telemetry bring-up.

## 4. Device-level implementation requirements

### 4.1 STM32F405RGT6

- Use the LQFP64 symbol and matching 64-pin footprint.
- Use the 8 MHz HSE and configure the firmware clock tree for USB FS's 48 MHz requirement.
- Place all VDD/VSS pairs with local ceramic bypass capacitors according to the ST datasheet.
- Place 2.2 µF low-ESR ceramic capacitors at both VCAP pins.
- Route VDDA/VSSA and ADC inputs away from the buck switch node and MAX7456 CVBS return.
- Expose PA13, PA14, NRST, target 3.3 V, and GND on the SWD header.

### 4.2 ICM-42688-P

The project-local symbol follows the selected DS-000347 Rev. 1.9 pin table:

| Pin | Signal |
|---:|---|
| 1 | `AP_SDO / AP_AD0` |
| 2 | `RESV` |
| 3 | `RESV` |
| 4 | `INT1 / INT` |
| 5 | `VDDIO` |
| 6 | `GND` |
| 7 | `RESV` |
| 8 | `VDD` |
| 9 | `INT2 / FSYNC / CLKIN` |
| 10 | `RESV` |
| 11 | `RESV` |
| 12 | `AP_CS` |
| 13 | `AP_SCL / AP_SCLK` |
| 14 | `AP_SDA / AP_SDIO / AP_SDI` |

- Use `IMU_3V3` for both VDD and VDDIO.
- Place 0.1 µF + 2.2 µF on VDD and 10 nF on VDDIO close to the package.
- Use SPI1 at no more than the official 24 MHz four-wire SPI limit.
- Keep the IMU near the board center, isolated from the buck inductor, USB connector, buzzer driver, and high-current input path.
- Use INT1 on the validated sensor pin and retain a test pad for INT2/optional future use.
- Do not leave reserved pins floating; follow the exact current TDK pin table.

### 4.3 BMP280

- Use I²C1, address `0x76`, permanent `MAIN_3V3` supply, and two 100 nF local bypass capacitors as shown by Bosch.
- Tie CSB high and SDO low; do not leave SDO floating.
- Place the sensor in a mechanically protected but pressure-exposed area away from the buck inductor, hot regulator, and prop-wash obstruction.
- Keep the vent clear and observe the Bosch mechanical clearance/liquid/light restrictions.

### 4.4 W25Q128JV

- Use `MAIN_3V3` and the standard SPI command set for the first population.
- Apply defined power-up state to `/CS`; hold `/WP` and `/HOLD`/`/RESET` inactive with pull-ups.
- Do not issue commands until the vendor power-up/write-inhibit interval has elapsed and poll BUSY before dependent operations.
- Select a local decoupling capacitor from the final Winbond reference/design review; do not claim a value based on the sensor datasheets.

### 4.5 MAX7456 analog OSD

- Treat the part as a 5 V analog-CVBS OSD generator, not digital video.
- Provide AVDD, DVDD, and PVDD from the regulated 5 V rail with local bypassing.
- Provide the required 27 MHz crystal and follow the reset timing.
- Route `CAM_VIDEO` through the MAX7456 VIN path and `VTX_VIDEO` from VOUT through the required AC-coupling/75 Ω network.
- Use short, impedance-controlled-as-defined-by-the-selected-fabricator CVBS paths with the termination/coupling parts at the device and connectors.
- 3.3 V STM32 outputs may satisfy MAX7456 digital input thresholds, but MAX7456 SDOUT is a 5 V-domain output. Add a one-direction 5 V-to-3.3 V input buffer/translator or use a verified STM32 5 V-tolerant input only after both datasheets are checked. The first schematic shall include the translator footprint rather than rely on an unverified direct connection.
- Mark MAX7456 as an obsolete lifecycle component in the BOM and keep the analog OSD block replaceable.

## 5. Input power and protection detail

The KiCad schematic must show the protection functions explicitly, even if exact part numbers are finalized during component selection:

1. `VBAT_IN` connector and local ground;
2. replaceable fuse or resettable current limiter;
3. reverse-polarity MOSFET stage with gate protection;
4. battery-rated TVS selected from the fully charged pack voltage and measured clamping target;
5. input ceramic plus bulk capacitor with voltage/ripple/current derating;
6. TPS54360 VIN bypass loop, EN/UVLO divider, bootstrap capacitor, catch diode, inductor, feedback divider, compensation, output capacitors, and PowerPAD thermal vias;
7. `SYS_5V` test points and a load/voltage measurement point;
8. USB-C VBUS fuse/ESD path and reverse-current-blocking OR-ing element;
9. separate `MAIN_3V3` and `IMU_3V3` LDO blocks with enable/default behavior;
10. battery and current measurement dividers with clamp/RC filtering.

The first board must be powered through a current-limited bench supply or protected battery harness during bring-up. The design is not released for direct battery flight operation until hot-plug, reverse connection, disconnect, thermal, and load-transient tests pass.

## 6. External connectors

All prototype connectors use 2.54 mm headers unless a later mechanical review changes that decision.

### 6.1 4-in-1 ESC

```text
GND, 5V_ESC, MOTOR1, MOTOR2, MOTOR3, MOTOR4,
ESC_TELEM_RX, VBAT_SENSE, CURRENT_SENSE
```

The power connector carries signal/reference and sensor connections; the high-current motor/battery power path is not routed through the flight-controller PCB. The selected ESC documentation must define current-sense scale, telemetry level/direction, pull behavior, and whether the 5 V pin is a source or a load before the final connector pinout is frozen.

### 6.2 Receiver

```text
GND, 5V_RX, CRSF_BUS
```

The receiver bus is a single-wire half-duplex USART1 connection for CRSF/ELRS. The selected receiver's logic-level and inversion requirements determine whether a series resistor, open-drain stage, inverter, or level translator is populated.

### 6.3 GPS

```text
GND, 5V_GPS, GPS_TX, GPS_RX, I2C1_SCL, I2C1_SDA
```

GPS UART uses USART3. The GPS module's operating voltage and UART level must be checked; the 5 V pin is an auxiliary supply output only if the regulator/current budget permits it.

### 6.4 Digital VTX MSP

```text
GND, VTX_BEC_5V, VTX_MSP_TX, VTX_MSP_RX
```

`VTX_BEC_5V` is an external-VTX supply/reference label, not a promise that the board's `SYS_5V` can power an arbitrary digital VTX. The VTX model and control protocol must be selected before firmware integration.

### 6.5 Analog camera and analog VTX

```text
CAM_VIDEO, VTX_VIDEO, 5V, GND
```

These are separate CVBS paths for the MAX7456 block. They are not digital video lanes.

### 6.6 SWD, USB-C, buzzer, and LED

- SWD: `3V3`, `SWDIO`, `SWCLK`, `NRST`, `GND`.
- USB-C: USB2 D+/D−, VBUS, CC1, CC2, GND, shell/chassis treatment as defined by the PCB stackup and enclosure.
- Buzzer: `BUZZER_5V`, `BUZZER_SW`, `GND`; drive through an external transistor/MOSFET.
- LED: board status LED plus `LED_STRIP`, `5V`, `GND`.

USB-C CC1 and CC2 use 5.1 kΩ Rd pull-downs for sink mode. Connector-side low-capacitance ESD protection is required. USB D+/D− routing follows ST AN4879 and the applicable USB specification.

## 7. PCB and mechanical specification

- Board outline: 40 mm × 40 mm.
- Mounting pattern: 30.5 mm × 30.5 mm center-to-center.
- Four 3.2 mm NPTH mounting holes.
- Nominal 1.6 mm, four-layer board; final dielectric thickness and copper weights must come from the fabricator stackup.
- Layer intent:
  - `F.Cu`: components, short critical routes, analog video and sensor escape;
  - `In1.Cu`: continuous GND reference plane;
  - `In2.Cu`: low-current power distribution and selected slow signals;
  - `B.Cu`: connectors and low-speed routes.

### 7.1 Placement zones

1. **Power/input zone:** VBAT connector, fuse, reverse MOSFET, TVS, input bulk, TPS54360, diode, and inductor near one board edge.
2. **MCU/digital zone:** STM32, flash, USB, SWD, and digital connectors around the center/edge boundary.
3. **IMU zone:** ICM-42688-P close to the geometric center with a quiet local ground return and no buck switch-node copper underneath.
4. **Barometer zone:** BMP280 away from heat sources and mechanical obstruction, with an open pressure path.
5. **Analog-video zone:** MAX7456, 27 MHz crystal, CVBS input/output coupling and 75 Ω network in one compact region, isolated from the buck switch node and high-speed digital clocks.
6. **Connector edge:** ESC, receiver, GPS, VTX, camera, analog VTX, SWD, and USB connectors aligned to make harness routing unambiguous.

The final fabricator stackup must be used to calculate USB differential geometry and any controlled CVBS impedance. A four-layer label alone does not prove 90 Ω USB or 75 Ω video impedance.

## 8. Schematic and PCB acceptance criteria

### 8.1 Schematic

- Every external connector pin has a net name, power source/consumer meaning, and documented voltage expectation.
- No MCU pin is assigned to more than one required function.
- The LQFP64 symbol pin numbers match the selected STM32F405RGT6 footprint.
- VCAP1/VCAP2, all VDD/VSS, VDDA/VSSA, NRST, BOOT0, USB, and SWD are explicitly connected.
- BMP280 CSB/SDO straps, flash pull states, MAX7456 reset/clock, and USB-C CC resistors are present.
- ERC has no unresolved power-input, unconnected-pin, or illegal-driver errors after intentional no-connects are documented.
- A complete pin/AF/timer/DMA review table is attached to the project notes before firmware bring-up; the timer DMA row must be verified against the selected firmware implementation.

### 8.2 PCB

- Board is 40 × 40 mm with the four 30.5 mm-spaced 3.2 mm NPTH holes.
- No courtyard or copper violation around mounting holes or board edge.
- IMU is not in the buck inductor/switch-node keepout.
- USB D+/D− has continuous reference, short matched routing, and connector-side ESD.
- TPS54360 high-di/dt loop is compact with PowerPAD thermal vias and the SW node kept small.
- Analog CVBS routes are short and isolated from the switch node and motor outputs.
- Power rails have test points for `VBAT_PROTECTED`, `SYS_5V`, `MAIN_3V3`, and `IMU_3V3`.
- DRC has no errors; the board netlist matches the schematic; all four motor nets, sensor buses, UARTs, USB, and SWD are connected as intended.

### 8.3 Bring-up and safety

1. Inspect the bare board for shorts and correct component orientation.
2. Power from a current-limited bench supply with the battery protection stage populated.
3. Verify protected input, `SYS_5V`, `MAIN_3V3`, and `IMU_3V3` without an MCU installed or with reset held.
4. Verify USB VBUS does not back-feed the battery rail and battery input does not back-feed USB VBUS.
5. Program and halt through SWD; verify USB enumeration.
6. Read ICM-42688-P WHO_AM_I and BMP280 chip ID; read/write flash status safely.
7. Verify each UART with a known-good 3.3 V test fixture before connecting external FPV hardware.
8. Verify DShot output timing into a logic analyzer or current-limited ESC test setup before mounting propellers.
9. Test MAX7456 with a known composite source/load and confirm SDOUT level translation.
10. Perform controlled hot-plug, reverse-polarity, disconnect, thermal, and load-transient tests before any flight test.

## 9. Source-backed design constraints

- [ST STM32F405RG datasheet, DS8626](https://www.st.com/resource/en/datasheet/stm32f405rg.pdf): LQFP64 pinout and alternate-function tables, USB/SWD, power, VCAP, timers, ADC, SPI, and UART constraints.
- [ST AN4879 USB hardware guidance](https://www.st.com/resource/en/application_note/an4879-introduction-to-usb-hardware-and-pcb-guidelines-using-stm32-mcus-stmicroelectronics.pdf): USB FS routing, placement, and protection.
- [TI TPS54360 datasheet](https://www.ti.com/lit/ds/symlink/tps54360.pdf): 4.5–60 V recommended VIN, 65 V absolute maximum, 5 V design example, bootstrap, catch diode, input loop, EN/UVLO, and layout.
- [TDK/InvenSense ICM-42688-P product page](https://product.tdk.com/en/search/sensor/mortion-inertial/imu/info?part_no=ICM-42688-P) and [current official datasheet cited in the research record](https://d17t6iyxenbwp1.cloudfront.net/s3fs-public/2026-06/DS-000347%20ICM-42688-P%20v1.9.pdf): supply ranges, SPI rate, INT behavior, pinout, and decoupling.
- [Bosch BMP280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf): supply ranges, I²C address straps, bus limits, pinout, 100 nF bypass, pressure venting, and partial-power-down hazard.
- [Winbond W25Q128JV documentation portal](https://www.winbond.com/hq/support/documentation/?__locale=en&category=%2F.categories%2Fresources%2Fdatasheet%2F&family=%2Fproduct%2Fcode-storage-flash-memory%2Fserial-nor-flash%2Findex.html&line=%2Fproduct%2Fcode-storage-flash-memory%2Findex.html&pno=W25Q128JV) and [Winbond Rev. M datasheet](https://www.winbond.com/resource-files/W25Q128JV%20RevM%2012242024%20Plus.pdf): 2.7–3.6 V operation, SPI/QSPI behavior, power-up timing, status polling, and pin states.
- [Analog Devices MAX7456 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max7456.pdf) and [ADI product page](https://www.analog.com/en/products/max7456.html): 5 V analog CVBS OSD, SPI thresholds/rate, 27 MHz clock, reset, 75 Ω video environment, and obsolete lifecycle.
- [USB Type-C Specification R2.0](https://www.usb.org/sites/default/files/USB%20Type-C%20Spec%20R2.0%20-%20August%202019.pdf): sink CC termination, receptacle D+/D− mapping, VBUS/VCONN behavior.
- [Betaflight CRSF implementation](https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.c), [Betaflight DShot API](https://betaflight.com/docs/development/API/Dshot), [Betaflight ESC telemetry guide](https://betaflight.com/docs/wiki/guides/current/ESC-Telemetry), and [Betaflight connector standard](https://betaflight.com/docs/development/manufacturer/connector-standard): protocol framing, half-duplex behavior, DShot timer/DMA implications, ESC telemetry convention, and connector signals.

## 10. Explicit exclusions and remaining implementation gates

The following are intentionally not silently decided by this specification:

- exact fuse, MOSFET, TVS, inductor, catch diode, and LDO ordering codes;
- final battery chemistry and maximum fully charged pack voltage;
- exact current-sense scaling of the selected 4-in-1 ESC;
- exact CRSF/ELRS receiver model and its electrical level/inversion;
- exact GPS module, baud rate, and supply current;
- exact digital VTX model and MSP/DisplayPort control behavior;
- a MAX7456 replacement for future production work;
- final timer DMA stream/request verification in the selected firmware;
- final PCB fabricator stackup and controlled-impedance geometry.

These are implementation gates, not reasons to change the approved high-level architecture. The schematic capture may begin after this specification is reviewed, but the resulting board must retain explicit footprints/test points where these gates affect safety or interoperability.
