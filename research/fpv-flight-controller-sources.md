# FPV Flight-Controller Prototype: Primary-Source Architecture Research

**Project:** `C:\Users\admin\gofly`  
**Scope:** source-backed architecture constraints for an STM32F405-based FPV flight controller  
**Status:** research baseline; not a schematic or PCB design  
**Last consolidated:** 2026-09-20  

> This document records design constraints and unresolved decisions. It does **not** authorize a schematic or PCB implementation. No KiCad schematic, PCB, or project file is modified by this research.

## 1. Executive conclusions

1. **The STM32F405RGT6 LQFP64 pinout must be allocated globally before schematic capture.** SPI flash, IMU, barometer, USB FS, SWD, receiver UART, GPS, ESC telemetry, ADCs, and four DShot outputs compete for a constrained alternate-function matrix. A peripheral-by-peripheral assignment is unsafe.
2. **TPS54360 suitability for 4–6S is conditional, not proven by its 60 V rating.** Its recommended VIN range is 4.5–60 V and absolute maximum is 65 V, but battery hot-plug, wiring inductance, ESC noise, reverse polarity, disconnect/load-dump behavior, and TVS clamping still require a system-level design and verification.
3. **MAX7456 is an obsolete 5 V analog-CVBS OSD device.** It is not an HDMI, MIPI, SDI, or generic digital-video interface. Any digital-video requirement must name the VTX ecosystem and define the control/video architecture separately.
4. **The selected receiver, ESC, and VTX hardware determines electrical levels.** Betaflight/iNav protocol sources define framing and behavior, not a universal voltage standard for every CRSF/ELRS receiver, ESC telemetry output, or digital VTX control port.
5. **BMP280 power sequencing is safety-critical.** Driving SDI, SDO, SCK, or CSB high while VDDIO is off can permanently damage the sensor.
6. **This prototype selects the ICM-42688-P pin map from DS-000347 Rev. 1.9.** INT1 is pin 4 and INT2/FSYNC/CLKIN is pin 9; the custom symbol records those numbers, while interrupt polarity and drive configuration remain firmware-level checks.
7. **USB-C device-only versus dual-role/DRP, bidirectional DShot versus separate ESC telemetry, and 3.3 V/5 V rail architecture remain explicit design decisions.**

## 2. Source inventory

### 2.1 Semiconductor and USB primary sources

| Topic | Official source | Document / section used |
|---|---|---|
| STM32F405RGT6 | [ST STM32F405RG datasheet](https://www.st.com/resource/en/datasheet/stm32f405rg.pdf) and [ST alternate document endpoint](https://www.st.com/resource/en/datasheet/DM00037051.pdf) | *STM32F405xx/STM32F407xx datasheet*, DS8626 Rev. 12; pinout/AF tables, operating conditions, power supply, SPI/I2C/USB, package data |
| STM32 USB hardware | [ST AN4879](https://www.st.com/resource/en/application_note/an4879-introduction-to-usb-hardware-and-pcb-guidelines-using-stm32-mcus-stmicroelectronics.pdf) | *Introduction to USB hardware and PCB guidelines using STM32 MCUs*; USB routing, protection, placement, and hardware guidance |
| ICM-42688-P | [TDK/InvenSense ICM-42688-P datasheet](https://invensense.tdk.com/wp-content/uploads/2021/06/DS-000347-ICM-42688-P-v1.5.pdf), [TDK product page](https://product.tdk.com/en/search/sensor/mortion-inertial/imu/info?part_no=ICM-42688-P) | DS-000347 Rev. 1.9 pin-table extraction retained in the local research artifacts; the public PDF endpoint is Rev. 1.5; electrical characteristics, package, SPI, supply bypass |
| ICM-42688-P evaluation hardware | [TDK evaluation-board manual](https://invensense.tdk.com/wp-content/uploads/2024/10/AN-000493-QCIoT-ICM42688P-Evaluation-Board-Manual-v1.0.pdf) | Hardware implementation context, /CS and INT1 usage |
| BMP280 | [Bosch BMP280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf) | *BST-BMP280-DS001-26*, Rev. 1.26; electrical specifications, interfaces, power sequencing, pinout |
| W25Q128JV | [Winbond documentation portal](https://www.winbond.com/hq/support/documentation/?__locale=en&category=%2F.categories%2Fresources%2Fdatasheet%2F&family=%2Fproduct%2Fcode-storage-flash-memory%2Fserial-nor-flash%2Findex.html&line=%2Fproduct%2Fcode-storage-flash-memory%2Findex.html&pno=W25Q128JV), [Winbond product page](https://www.winbond.com/hq/product/code-storage-flash/qspi-nor/w25q-jv/?__locale=en&partNo=W25Q128JVBAQ) | W25Q128JV family documentation; pinout, SPI/QSPI, status bits, timing, power-up |
| TPS54360 | [TI TPS54360 datasheet](https://www.ti.com/lit/ds/symlink/tps54360.pdf) | *TPS54360 60-V Input, 3.5-A, Step-Down DC/DC Converter With Eco-Mode*, SLVSBB4G; operating limits, design example, transient/input/layout guidance |
| MAX7456 | [Analog Devices MAX7456 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max7456.pdf), [ADI product page](https://www.analog.com/en/products/max7456.html) | MAX7456 Rev. 1, September 2008; electrical characteristics, SPI, CVBS, reset, clock, layout; product page lifecycle |
| USB Type-C | [USB Type-C Cable and Connector Specification Rev. 2.0](https://www.usb.org/sites/default/files/USB%20Type-C%20Spec%20R2.0%20-%20August%202019.pdf) | Tables 3-4/3-5 receptacle assignment; Table 4-25 sink CC termination; VBUS/VCONN timing |
| USB 2.0 | [USB-IF USB 2.0 specification](https://www.usb.org/document-library/usb-20-specification) | USB attach, pull-up/pull-down, and bus behavior; use the applicable revision for final compliance |

### 2.2 First-party firmware and interface sources

| Topic | First-party source | Relevant material |
|---|---|---|
| Betaflight CRSF implementation | [crsf.c](https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.c), [crsf_protocol.h](https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf_protocol.h), [crsf.h](https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.h) | Frame parsing, CRC, serial behavior, protocol constants |
| Betaflight serial allocation | [Serial guide](https://betaflight.com/docs/wiki/guides/current/Serial) | UART resource and serial-function configuration |
| Betaflight GPS | [GPS guide](https://betaflight.com/docs/wiki/guides/current/Gps) | GPS UART and baud-rate configuration |
| Betaflight DShot | [DShot API documentation](https://betaflight.com/docs/development/API/Dshot) | 16-bit DShot frame, CRC, telemetry-request bit, bidirectional DShot behavior |
| Betaflight ESC telemetry | [ESC telemetry guide](https://betaflight.com/docs/wiki/guides/current/ESC-Telemetry) | Serial ESC telemetry and 115200-baud guidance |
| Betaflight connector convention | [Connector standard](https://betaflight.com/docs/development/manufacturer/connector-standard) | Typical flight-controller/ESC connector signals |
| iNav serial functions | [iNav Serial.md](https://github.com/iNavFlight/inav/blob/master/docs/Serial.md) | GPS, RX_SERIAL, and ESCSERIAL function allocation |
| iNav telemetry | [iNav Telemetry.md](https://github.com/iNavFlight/inav/blob/master/docs/Telemetry.md), [CRSF telemetry source](https://github.com/iNavFlight/inav/blob/master/src/main/telemetry/crsf.c) | CRSF telemetry fields and serial telemetry behavior |
| iNav OSD | [iNav OSD.md](https://github.com/iNavFlight/inav/blob/master/docs/OSD.md) | OSD and digital-VTX control context |

> **Winbond source note:** A temporary third-party copy hosted by SparkFun was used only for local text extraction/page verification during research: [temporary extraction copy](https://cdn.sparkfun.com/assets/9/d/7/a/d/w25q128jv_revf_03272018_plus.pdf). It is not treated as the primary authority; final production design review must use the matching Winbond revision from the Winbond portal.

## 3. STM32F405RGT6: supply, package, and actual AF/pin constraints

**Primary source:** ST, *STM32F405xx/STM32F407xx datasheet*, DS8626 Rev. 12, [official PDF](https://www.st.com/resource/en/datasheet/DM00037051.pdf).

### 3.1 Device-level constraints

- Maximum CPU frequency is **168 MHz** (datasheet device/features section, approximately PDF pp. 1–2).
- VDD operating range is **1.8–3.6 V** (Table 14, approximately PDF p. 81).
- VBAT operating range is approximately **1.65–3.6 V** (Table 14, approximately PDF p. 81).
- The LQFP64 package is approximately **10 mm × 10 mm** (LQFP64 mechanical data, approximately PDF pp. 169–170).
- USB OTG FS, USART/UART, SPI, I2C, timers/DMA, ADC, and SWD are integrated, but the available AF combinations depend on package pin and AF selection (pin/AF tables, approximately PDF pp. 49–70).

### 3.2 LQFP64 pin assignments that materially constrain the architecture

The following are package pin/AF facts to use as an initial resource map. They are not a substitute for checking the complete AF table, timer channel, DMA stream, and board-level signal list before capture.

| Pin | Function relevant to this design | Source location |
|---|---|---|
| PA2 | USART2_TX | ST pin/AF table, approximately PDF pp. 49–61 |
| PA3 | USART2_RX | ST pin/AF table, approximately PDF pp. 49–61 |
| PA5 | SPI1_SCK | ST pin/AF table, approximately PDF pp. 49–61 |
| PA6 | SPI1_MISO | ST pin/AF table, approximately PDF pp. 49–61 |
| PA7 | SPI1_MOSI | ST pin/AF table, approximately PDF pp. 49–61 |
| PA9 | USART1_TX; system bootloader USART1 TX | ST pin/AF table and system-memory bootloader interface, approximately PDF pp. 49–61 and bootloader section |
| PA10 | USART1_RX; system bootloader USART1 RX | Same as above |
| PA11 | USB OTG FS DM | ST pinout/AF and USB characteristics, approximately PDF pp. 43, 49–61, 130–131 |
| PA12 | USB OTG FS DP | Same as above |
| PA13 | JTMS/SWDIO | Debug/SWJ section, approximately PDF p. 42 and pinout table |
| PA14 | JTCK/SWCLK | Debug/SWJ section, approximately PDF p. 42 and pinout table |
| PB10/PB11 | USART3_TX/USART3_RX option | ST pin/AF table, approximately PDF pp. 49–61 |
| PC10/PC11 | USART3_TX/USART3_RX option | ST pin/AF table, approximately PDF pp. 49–61 |

**SPI1 implication:** PA5/PA6/PA7 are a natural high-speed SPI1 group for flash or a sensor, but SPI1 NSS and any second SPI device must be assigned deliberately. Chip-selects are ordinary GPIOs unless a timer/peripheral AF is specifically required. Do not consume PA5–PA7 for a device without checking the remaining motor, UART, USB, and debug requirements.

**USART implication:** USART3 can be placed on PB10/PB11 or PC10/PC11. This gives some flexibility for CRSF/GPS/ESC telemetry, but the selected pair must be checked against timer channels, ADC inputs, board connector placement, and bootloader/service access.

**USB/SWD implication:** PA11/PA12 are dedicated USB FS data pins in the normal design; PA13/PA14 are the SWD pair. Do not reuse them casually if field firmware update and debug access are requirements.

### 3.3 Power and clock implementation

- VCAP1 and VCAP2 require external ceramic capacitors when the internal regulator is used. The cited operating-condition recommendation is **2.2 µF per VCAP pin**; the datasheet specifies suitable low-ESR ceramic behavior and gives an ESR limit around 2 Ω (VCAP Table 16, approximately PDF p. 84).
- When the internal regulator is bypassed, the capacitor recommendation changes; use the exact regulator-bypass configuration from the selected datasheet revision rather than mixing the two arrangements.
- VDDA/VSSA supply the ADC, DAC, reset circuitry, RC oscillators, and PLL-related analog blocks. Route and decouple them according to the power-supply recommendation (power-supply section and Table 14, approximately PDF pp. 81–84).
- SPI timing allows approximately **42 Mbit/s** under the applicable conditions (SPI dynamic characteristics, Table 55, approximately PDF p. 124).
- I2C supports standard mode up to **100 kbit/s** and fast mode up to **400 kbit/s** in the relevant electrical/timing specification.
- USB FS requires the correct **48 MHz** clock configuration and the signal/power implementation from ST’s USB hardware guidance ([AN4879](https://www.st.com/resource/en/application_note/an4879-introduction-to-usb-hardware-and-pcb-guidelines-using-stm32-mcus-stmicroelectronics.pdf)).

### 3.4 Required allocation exercise before schematic capture

Create one reviewed table mapping every required signal to a concrete STM32 pin, AF, timer channel, DMA stream/request, pull-up/inversion requirement, and connector. At minimum include:

- SPI flash: SCK/MOSI/MISO plus CS and, if Quad SPI is used, IO2/IO3/QE handling.
- IMU SPI and interrupt(s).
- BMP280 SPI or I2C and address straps.
- USB FS DM/DP.
- SWDIO/SWCLK/NRST.
- CRSF/ELRS UART, including half-duplex behavior.
- GPS UART.
- ESC telemetry UART or bidirectional DShot return path.
- Four motor timer outputs with DShot-capable DMA resources.
- Battery/current ADC inputs and their scaling/protection.
- Any MAX7456 control/video signals or named digital-VTX control UART.

## 4. Power architecture and protection

### 4.1 TPS54360 input/output limits

**Primary source:** TI, *TPS54360 60-V Input, 3.5-A, Step-Down DC/DC Converter With Eco-Mode*, SLVSBB4G, [official PDF](https://www.ti.com/lit/ds/symlink/tps54360.pdf).

- Recommended VIN operating range: **4.5–60 V** (features, pin description, and electrical characteristics; PDF pp. 1, 4–5, and approximately 12–23).
- VIN absolute maximum: **65 V** (absolute maximum table, PDF p. 5).
- TI states that the device survives specified **65 V ISO 7637 load-dump pulses** (features/description, PDF p. 1). This statement does not qualify the whole flight-controller input network for arbitrary battery transients.
- Continuous output current rating: **3.5 A**; integrated high-side MOSFET (features/description, PDF p. 1).
- Internal feedback reference is approximately **0.8 V ±1%** (features and electrical characteristics, PDF pp. 1 and approximately 12–23).
- Typical internal UVLO rising threshold is approximately **4.3 V** (electrical characteristics, approximately PDF p. 12).
- EN threshold is approximately **1.2 V typical**; an EN divider can establish separate start/stop thresholds (pin description and design section, approximately PDF pp. 4 and 25–31).

### 4.2 What the 5 V design example does and does not prove

TI’s example is a **5 V output** design, with an example input range of approximately **8.5–60 V**, an example start threshold of **8 V**, and stop threshold of **6.25 V** (typical application/design example, approximately PDF pp. 25–31). Its load-step example is approximately **0.875 A to 2.625 A**, with a regulation target around **4% of 5 V**.

These are design-example values, not universal ratings. Output voltage must be set with the feedback network and validated with the selected inductor, catch diode, switching frequency, compensation, output capacitors, thermal design, and load transient.

### 4.3 TPS54360 implementation requirements

- **Bootstrap:** 0.1 µF ceramic between BOOT and SW; X5R or better is recommended and the capacitor voltage rating should be at least 10 V (Section 8.2.2.7, approximately PDF p. 30).
- **Input decoupling:** use X5R/X7R ceramic capacitance with at least approximately 3 µF effective capacitance in the example; account for DC-bias derating and select voltage rating above maximum VIN. Add bulk capacitance when the source is remote; TI gives approximately 100 µF electrolytic as a typical bulk option (Sections 8.2.2.6 and 9, approximately PDF pp. 29–30 and 37).
- **Catch diode:** external diode required. Its reverse-voltage rating should be at least VIN(max); the example recommends at least 60 V capability for transients up to the converter rating. Peak current must exceed maximum inductor current (design section, approximately PDF pp. 25–31).
- **Inductor:** saturation/current rating must include transient current; the detailed discussion gives a nominal switch-current limit around 5.5 A in the design context (electrical characteristics/design section).
- **EN/UVLO:** use the divider to define battery start/stop behavior. If EN could exceed its absolute maximum in a high-input/low-start configuration, implement the clamping recommended by TI (UVLO/EN design section, approximately PDF pp. 25–31).
- **Layout:** minimize the VIN bypass loop; keep VIN capacitor, VIN pin, catch diode, SW node, and inductor close; connect GND directly to the exposed PowerPAD and use multiple thermal vias; keep SW copper small and RT/CLK short and quiet (Section 10.1, PDF p. 38).

### 4.4 4S/6S battery input: unresolved safety design

A 4S or 6S battery’s normal voltage can fit inside the TPS54360’s recommended VIN range, but the following are unresolved and must be designed, calculated, and tested:

- maximum fully charged pack voltage for the chosen chemistry;
- hot-plug and connector-bounce overshoot;
- wiring inductance and ringing;
- ESC commutation noise;
- battery disconnect/load-dump events;
- reverse-polarity protection;
- fuse or current limiting;
- TVS standoff, breakdown, and clamping voltage;
- input ceramic and bulk capacitance, ESR, ripple-current rating, and placement;
- the actual voltage at the TPS54360 VIN pins after protection/filtering;
- downstream capacitor and semiconductor voltage ratings;
- thermal behavior at the expected output current and ambient temperature.

**Unsafe assumption to replace:** “The converter is safe because its absolute maximum is 65 V.” Absolute maximum is not a design target and is not a complete transient-protection design.

## 5. ICM-42688-P IMU

**Primary sources:** TDK/InvenSense, *ICM-42688-P Datasheet*, selected revision DS-000347 Rev. 1.9 (local official extraction), [public official PDF endpoint](https://invensense.tdk.com/wp-content/uploads/2021/06/DS-000347-ICM-42688-P-v1.5.pdf), and [TDK product page](https://product.tdk.com/en/search/sensor/mortion-inertial/imu/info?part_no=ICM-42688-P).

- Package: approximately **2.5 mm × 3.0 mm × 0.91 mm**, 14-pin LGA (package section/drawing).
- VDD: **1.71–3.6 V**.
- VDDIO: **1.71–3.6 V**.
- Maximum 4-wire SPI clock: approximately **24 MHz** (electrical/interface timing section).
- Typical local bypass: **0.1 µF + 2.2 µF on VDD** and approximately **10 nF on VDDIO** (application/reference connection information).
- Use /CS and a verified interrupt connection; the official evaluation-board manual documents the hardware context ([manual](https://invensense.tdk.com/wp-content/uploads/2024/10/AN-000493-QCIoT-ICM42688P-Evaluation-Board-Manual-v1.0.pdf)).
- The selected DS-000347 Rev. 1.9 pin table assigns **INT1 to pin 4** and **INT2/FSYNC/CLKIN to pin 9**. It also assigns VDDIO to pin 5, GND to pin 6, VDD to pin 8, /CS to pin 12, SCLK to pin 13, SDI/SDIO to pin 14, and SDO/AD0 to pin 1.
- Reserved pins 2, 3, 7, 10, and 11 remain subject to the current datasheet's reserved-pin instructions; the schematic must not invent connections for them.

**Implementation status:** the Rev. 1.9 pin map is now selected for this research prototype and will be recorded in the project-local symbol. Firmware interrupt polarity, drive mode, and any reserved-pin treatment remain verification items.

## 6. BMP280 barometer

**Primary source:** Bosch Sensortec, *BMP280 Data sheet*, BST-BMP280-DS001-26 Rev. 1.26, October 2021, [official PDF](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf).

### 6.1 Supply and package

- Package: 8-pin metal-lid LGA, approximately **2.0 mm × 2.5 mm × 0.95 mm** (package/drawing, approximately PDF pp. 35–37).
- VDD: **1.71–3.6 V**.
- VDDIO: **1.2–3.6 V**.
- Maximum VDD ripple: approximately **50 mVpp**.
- Low-power forced-mode 1 Hz current: typical approximately **2.8 µA**, maximum approximately **4.2 µA** in the cited electrical table (PDF p. 7).
- Peak pressure-measurement current: typical approximately **720 µA**, maximum approximately **1120 µA** (electrical table, PDF p. 7).

### 6.2 Interface and address

- I2C supports up to approximately **3.4 MHz**; 3-wire/4-wire SPI supports up to approximately **10 MHz** (key parameters, PDF p. 2 and interface/timing sections).
- CSB high during power-up selects I2C; CSB low selects SPI. Once CSB is pulled low, I2C remains disabled until power-on reset (interface selection section, approximately PDF pp. 10–11 and 29).
- SDO low selects I2C address **0x76**; SDO high selects **0x77**. SDO must not float (interface section, approximately PDF pp. 10–11).
- Pinout: pin 1 GND, pin 2 CSB, pin 3 SDI/SDA, pin 4 SCK/SCL, pin 5 SDO, pin 6 VDDIO, pin 7 GND, pin 8 VDD (pinout/connection diagrams, approximately PDF pp. 35–37).
- Bosch connection diagrams show approximately **100 nF** supply bypass capacitors for the relevant rails; use the diagram matching the selected bus and rail arrangement.

### 6.3 Power-sequencing hazard

Bosch explicitly warns that holding **SDI, SDO, SCK, or CSB high while VDDIO is switched off can permanently damage the device** through excessive current in ESD diodes. If VDDIO is present while VDD is absent, interface pins are high impedance (power-supply section, approximately PDF p. 11).

**Implementation consequence:** if the barometer rail can be disabled while the MCU or bus pull-ups remain powered, add a controlled power sequence, isolation/level strategy, or keep the interface pins in a safe state. Do not assume ordinary MCU pull-ups are harmless during partial power-down.

## 7. W25Q128JV serial NOR flash

**Primary sources:** [Winbond W25Q128JV documentation portal](https://www.winbond.com/hq/support/documentation/?__locale=en&category=%2F.categories%2Fresources%2Fdatasheet%2F&family=%2Fproduct%2Fcode-storage-flash-memory%2Fserial-nor-flash%2Findex.html&line=%2Fproduct%2Fcode-storage-flash-memory%2Findex.html&pno=W25Q128JV) and [Winbond product page](https://www.winbond.com/hq/product/code-storage-flash/qspi-nor/w25q-jv/?__locale=en&partNo=W25Q128JVBAQ). The page and section numbers below correspond to the W25Q128JV Rev. F extraction used during research; verify against the exact production ordering-code revision.

- Capacity: **128 Mbit** serial NOR flash.
- VCC operating range: **2.7–3.6 V**; absolute VCC range is approximately **−0.6 to +4.6 V** (operating range/absolute maximum sections, approximately extraction pp. 60–62).
- Supports standard SPI, Dual SPI, and Quad SPI; maximum clock capability is approximately **133 MHz** subject to mode/timing conditions (features/electrical characteristics).
- Array organization: **65,536 pages × 256 bytes**; 4 KiB sector erase, 32 KiB/64 KiB block erase, and chip erase (memory organization/instruction sections, approximately pp. 4–5 and 13–17).
- Standard 8-pin pinout: pin 1 /CS, pin 2 DO/IO1, pin 3 /WP/IO2, pin 4 GND, pin 5 DI/IO0, pin 6 CLK, pin 7 /HOLD or /RESET/IO3, pin 8 VCC (pin descriptions, approximately pp. 5–9).
- Quad SPI requires the **QE bit in Status Register-2**; IO0–IO3, /WP, /HOLD, and /RESET behavior must be designed together (status-register and Quad Enable sections, approximately pp. 13–17).
- BUSY is Status Register-1 bit S0; WEL is bit S1. WEL clears after program, erase, or status-register writes; poll BUSY before dependent operations (status-register/instruction sections, approximately pp. 13–17).
- Typical power-down current is below approximately **1 µA**; ordering options include industrial −40 to +85 °C and Industrial Plus −40 to +105 °C (features/electrical characteristics).
- Page Program maximum is approximately **3 ms**; 4 KiB Sector Erase maximum is approximately **400 ms** (AC characteristics, approximately pp. 64–65).
- During power-up/write-inhibit, program/erase/write instructions are ignored; VCC-to-/CS timing includes approximately **20 µs** from VCC(min) to /CS low in the cited revision, and /CS must track VCC during the write-inhibit interval (power-up timing, approximately PDF p. 61 / extracted timing section).

**Implementation consequence:** reserve all four Quad-SPI data lines if high-performance flash is a requirement, choose safe pull states for /WP and /HOLD or /RESET, and ensure the MCU does not issue commands before power-up and BUSY conditions are valid.

## 8. TPS54360-to-rail architecture

A practical architecture may use TPS54360 for a protected intermediate or 5 V rail and a separate 3.3 V regulator for the MCU/sensors/flash. The final choice is unresolved and must account for:

- whether MAX7456 is populated (requires a 5 V rail);
- 3.3 V regulator input range and dissipation;
- sensor/flash rail noise and sequencing;
- USB VBUS backfeed and self-powered behavior;
- ADC measurement scaling from battery and current sensors;
- ground partitioning between switching power, digital, IMU, barometer, and analog video.

No rail should be called “protected” until reverse-polarity, fuse/current limiting, TVS, hot-plug, and measured transient behavior are specified.

## 9. MAX7456 analog OSD

**Primary sources:** Analog Devices, *MAX7456*, Rev. 1, September 2008, [official datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max7456.pdf), and [ADI product page](https://www.analog.com/en/products/max7456.html).

### 9.1 Function and lifecycle

- MAX7456 is a single-channel monochrome **analog on-screen-display generator** for standard NTSC/PAL composite video (description and typical operating circuit, approximately PDF pp. 1 and 10).
- It is not HDMI, MIPI, SDI, a digital video serializer, or a generic digital-video processor.
- The ADI product page identifies the part as **obsolete**. This is a supply-chain and lifecycle risk; an approved replacement or alternate must be identified before committing the design.

### 9.2 Supply, logic, and SPI

- AVDD, DVDD, and PVDD each operate at **4.75–5.25 V**, with typical operation at 5 V (electrical characteristics, PDF pp. 2–4).
- Use local approximately **0.1 µF** bypass capacitors at the relevant supplies and follow the exposed-pad/thermal-via and analog/digital routing recommendations (typical operating circuit/layout, approximately PDF pp. 10 and 45).
- Digital input VIH minimum is approximately **2.0 V**; VIL maximum approximately **0.8 V** (electrical characteristics, PDF pp. 2–4). A 3.3 V STM32 output can generally satisfy the input threshold, but SDOUT is in the 5 V digital domain.
- SDOUT compatibility with a 3.3 V STM32 input must not be assumed. Confirm the chosen STM32 pin’s 5 V tolerance or add level translation/protection.
- SPI maximum SCLK is approximately **10 MHz**. Transactions are normally 16 bits: 8-bit address plus 8-bit data. Data is sampled on the rising edge and changes on the falling edge (SPI section, approximately PDF p. 20).

### 9.3 Composite video, clock, and reset

- VIN requires approximately **0.1 µF AC coupling**; the composite-video environment uses **75 Ω** termination. VIN and VOUT should be short and the termination/coupling parts close to the device (typical circuit/video input section, approximately PDF p. 10).
- The device uses a **27 MHz** parallel-resonant fundamental-mode crystal; the datasheet indicates external crystal load capacitors are not required (clock/pin description, approximately PDF pp. 10 and 15–20).
- RESET is pin 19. The datasheet specifies a minimum RESET pulse width of **50 ms**; SPI registers become accessible only after the stated post-reset interval (pin description, extracted PDF around the reset section).

**Unsafe assumption to replace:** “MAX7456 provides digital video.” It provides analog CVBS OSD. A digital-video board requirement must be separately specified around a named digital VTX and protocol.

## 10. USB-C, USB FS, UART, and SWD

### 10.1 USB-C sink implementation

**Primary source:** USB-IF, *USB Type-C Cable and Connector Specification Release 2.0, August 2019*, [official PDF](https://www.usb.org/sites/default/files/USB%20Type-C%20Spec%20R2.0%20-%20August%202019.pdf).

- CC1 and CC2 perform attach/orientation detection. A sink places **Rd pull-down resistors on both CC pins** (Type-C Table 4-25, approximately PDF p. 236).
- The specified Rd value is approximately **5.1 kΩ ±20%** (Table 4-25).
- USB 2.0 receptacle mapping is A6/B6 = D+ and A7/B7 = D− (receptacle pin assignment Tables 3-4 and 3-5, approximately PDF pp. 68–69).
- VBUS/VCONN timing and attach behavior must follow the relevant Type-C timing tables, including Table 4-29 (approximately PDF p. 237).
- A USB device must not enable its USB D+/D− pull-up before valid VBUS/attach conditions. Use the USB 2.0 and STM32 hardware guidance for the exact implementation.
- Connector-side ESD protection is required for a robust product. Select protection with capacitance and clamping behavior compatible with USB FS.
- Decide whether the connector is device-only, host-capable, or dual-role/DRP. The connector by itself does not define power-role behavior.
- For a self-powered flight controller, define VBUS sensing and prevent unwanted VBUS backfeed. USB-C PD is not implied unless a PD controller and policy are explicitly added.

### 10.2 STM32 USB FS and SWD

- STM32F405 USB OTG FS uses PA11 = DM and PA12 = DP (ST datasheet pin/AF and USB sections, approximately PDF pp. 43, 49–61, and 130–131).
- Follow [ST AN4879](https://www.st.com/resource/en/application_note/an4879-introduction-to-usb-hardware-and-pcb-guidelines-using-stm32-mcus-stmicroelectronics.pdf) for differential routing, placement, protection, and USB power considerations.
- SWD uses PA13 = SWDIO and PA14 = SWCLK (ST SWJ/SWD section, approximately PDF p. 42). Expose target VDD, GND, SWDIO, SWCLK, and NRST for connect-under-reset and production debug.

## 11. CRSF/ELRS, GPS, DShot, ESC telemetry, and connectors

### 11.1 CRSF/ELRS receiver

First-party references: [Betaflight CRSF implementation](https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.c), [protocol definitions](https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf_protocol.h), [header](https://github.com/betaflight/betaflight/blob/master/src/main/rx/crsf.h), and [Betaflight Serial guide](https://betaflight.com/docs/wiki/guides/current/Serial).

- CRSF is generally implemented as a **single-wire half-duplex UART**.
- Conventional Betaflight CRSF operation uses **420000 baud**, typically 8 data bits and one stop bit (implementation/protocol definitions and serial documentation).
- Frames include CRC protection; the relevant source definitions identify a maximum frame size of approximately **64 bytes**.
- The protocol source does **not** establish a universal electrical voltage level for every CRSF/ELRS receiver.
- Check the selected receiver’s hardware documentation for output voltage, inversion, protection, pull-up/open-drain behavior, and half-duplex wiring. Add level translation or protection if required by the actual receiver.

### 11.2 GPS

- GPS normally uses a dedicated hardware UART. Betaflight’s [GPS guide](https://betaflight.com/docs/wiki/guides/current/Gps) documents UBlox/NMEA configuration and common baud rates including **57600** and **115200**.
- Use 57600 if the selected GPS and wiring are unreliable at 115200; the final choice depends on update rate and message set.
- Reconcile the GPS UART with CRSF, USB bootloader access, ESC telemetry, and any VTX control UART in the STM32 AF allocation.

### 11.3 DShot motor outputs

Reference: [Betaflight DShot API](https://betaflight.com/docs/development/API/Dshot).

- A DShot frame is **16 bits**: 11-bit throttle value, one telemetry-request bit, and a 4-bit CRC.
- Bidirectional DShot returns telemetry/RPM over the same motor signal wire.
- A conventional quad/4-in-1 ESC arrangement requires four motor signal outputs with suitable STM32 timer and DMA support.
- The exact STM32F405 timer-channel/AF/DMA allocation must be checked for the selected pins. A GPIO that can toggle is not automatically a safe DShot resource.

### 11.4 4-in-1 ESC connector and telemetry

Reference: [Betaflight connector standard](https://betaflight.com/docs/development/manufacturer/connector-standard) and [ESC telemetry guide](https://betaflight.com/docs/wiki/guides/current/ESC-Telemetry).

Typical signals are:

- VBAT/V+;
- GND;
- current;
- telemetry;
- motor 1;
- motor 2;
- motor 3;
- motor 4.

The actual ESC documentation must define:

- battery/current sensor scaling;
- whether current is shared or per motor;
- telemetry electrical level and direction;
- inversion, pull-up, filtering, or level translation;
- whether telemetry is a dedicated UART, shared bus, or bidirectional DShot return;
- allowed input voltage and logic thresholds.

Betaflight documents **115200 baud** for serial ESC telemetry. Decide explicitly between:

1. bidirectional DShot telemetry;
2. a separate ESC telemetry UART;
3. both; or
4. neither.

Do not infer ESC electrical compatibility from the protocol name alone.

### 11.5 iNav serial function identifiers

Reference: [iNav Serial.md](https://github.com/iNavFlight/inav/blob/master/docs/Serial.md).

The reviewed iNav function values include:

- `GPS = 2`
- `RX_SERIAL = 64`
- `ESCSERIAL = 262144`

These values are firmware configuration identifiers, not pin electrical specifications.

## 12. Video architecture decision

### Analog path

If the requirement is analog FPV OSD, MAX7456 provides a 5 V CVBS solution, subject to its obsolete lifecycle and 3.3 V/5 V SDOUT compatibility. The design must include CVBS input/output coupling, 75 Ω termination, 27 MHz clocking, reset sequencing, and analog-video layout.

### Digital path

“Digital video” is incomplete as an electrical requirement. Name the intended VTX ecosystem and define:

- whether the board carries video pixels, only a control/OSD sideband, or both;
- UART direction and baud rate;
- protocol (for example, the selected vendor’s control protocol or MSP DisplayPort where applicable);
- connector and pinout;
- supply voltage and current;
- level shifting/inversion;
- OSD rendering ownership;
- whether the FC needs a camera/VTX video pass-through.

MSP DisplayPort is a firmware/control protocol; it is not itself the digital video stream. Do not use MAX7456 research as evidence of digital-video support.

## 13. Open decisions and unsafe assumptions checklist

Before schematic capture, obtain design-owner decisions for:

- [ ] 4S or 6S maximum pack chemistry and fully charged voltage.
- [ ] Reverse-polarity protection topology.
- [ ] Fuse/current limiting and connector hot-plug strategy.
- [ ] TVS selection and measured clamping target.
- [ ] Input bulk/ceramic capacitance and transient test plan.
- [ ] 5 V/3.3 V rail topology and USB VBUS isolation.
- [ ] STM32 complete AF/timer/DMA allocation.
- [ ] Exact ICM-42688-P official datasheet revision and INT pin.
- [ ] ICM-42688-P SPI versus I2C choice and interrupt polarity/pull configuration.
- [ ] BMP280 bus choice and address (`0x76` or `0x77`).
- [ ] W25Q128JV standard SPI versus Quad SPI and IO2/IO3 pull states.
- [ ] MAX7456 acceptance despite obsolete lifecycle, or approved replacement.
- [ ] Analog CVBS versus a named digital-VTX ecosystem.
- [ ] USB-C device-only versus host/DRP and VBUS sensing/backfeed behavior.
- [ ] CRSF/ELRS receiver model and actual I/O voltage/half-duplex behavior.
- [ ] GPS model, update rate, UART, and baud rate.
- [ ] Bidirectional DShot versus separate ESC telemetry UART.
- [ ] 4-in-1 ESC connector pinout, current scaling, telemetry level, and protection.
- [ ] SWD header/test pads including NRST.

## 14. Claims that must not be carried forward unchanged

- “The STM32 pinout can be assigned one peripheral at a time.” Replace with a complete package-level AF/timer/DMA resource map.
- “TPS54360 65 V absolute maximum means a 6S battery input needs no TVS or reverse protection.” Replace with a measured system-level transient/protection design.
- “MAX7456 is a digital-video interface.” Replace with “MAX7456 is an obsolete 5 V analog-CVBS OSD device,” or specify the actual digital VTX/control architecture.
- “CRSF/ELRS and ESC telemetry are 3.3 V because the firmware uses UART.” Replace with the selected hardware’s electrical specification.
- “Any ICM-42688-P interrupt pin is acceptable.” Replace with the exact selected datasheet revision’s verified pin mapping.
- “USB-C means USB device with no further CC/VBUS design.” Replace with explicit Type-C role, CC termination, VBUS sensing, ESD, and attach behavior.

## 15. Verification boundary

This file is a research record, not a released design. Before implementation:

1. Recheck every page/section against the exact ordering-code datasheet revision.
2. Replace the temporary Winbond extraction citations with the matching official Winbond PDF revision.
3. Obtain and archive a valid official ICM-42688-P PDF and verify the interrupt pin mapping.
4. Complete the STM32F405RGT6 AF/timer/DMA matrix and review it against the full schematic signal list.
5. Calculate and test the 4–6S input protection network with worst-case hot-plug and disconnect transients.
6. Confirm every selected receiver, ESC, GPS, VTX, connector, and regulator’s electrical levels and current limits.
7. Only then begin KiCad schematic capture.
