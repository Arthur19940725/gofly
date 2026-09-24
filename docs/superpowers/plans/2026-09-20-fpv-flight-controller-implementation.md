# 5-inch FPV Flight Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a source-backed KiCad research prototype for a 40 mm four-layer 5-inch FPV flight controller using STM32F405RGT6, ICM-42688-P, BMP280, W25Q128JV, MAX7456 analog OSD, protected 4–6S input, and the approved external interfaces.

**Architecture:** Build a root KiCad project with a modular schematic organized around power, MCU/core, sensors/storage, and external interfaces/video. Assign every signal to the approved STM32F405RGT6 LQFP64 resource map before wiring, then generate and place a four-layer 40 × 40 mm PCB, route critical power/sensor/USB/video nets, add a continuous inner GND plane, and verify the result with schematic validation/ERC, PCB DRC, courtyard checks, connectivity checks, and rendered views.

**Tech Stack:** KiCad MCP tools, KiCad schematic/PCB libraries, STM32F405RGT6 LQFP64, 4-layer PCB, official component datasheets and the project research record.

**Spec:** `docs/superpowers/specs/2026-09-20-fpv-flight-controller-design.md`

## Global Constraints

- This is a research prototype, not a flight-certified or production-ready flight controller.
- Board outline is 40 mm × 40 mm with a 30.5 mm × 30.5 mm mounting pattern and four 3.2 mm NPTH holes.
- Use a nominal 1.6 mm four-layer board: F.Cu, continuous In1.Cu GND, In2.Cu power/slow signals, and B.Cu connectors/low-speed signals.
- Digital VTX support is UART4 MSP control plus explicitly labeled external VTX/BEC power and GND; no MIPI, HDMI, SDI, or other high-speed digital-video lanes are implemented.
- MAX7456 is an obsolete-risk 5 V analog-CVBS OSD device and must not be represented as a digital-video processor.
- Use STM32F405RGT6 LQFP64 resources only; do not use PE9/PE11/PE13/PE14 or any other pin absent from the package.
- Preserve the approved resource map: SPI1 PA5/PA6/PA7 for IMU, SPI3 PB3/PB4/PB5 for flash, SPI2 PB13/PB14/PB15 for MAX7456, I2C1 PB8/PB9 for BMP280/GPS expansion, USART1 PA9/PA10 for CRSF/ELRS half-duplex, USART3 PB10/PB11 for GPS, UART4 PC10/PC11 for VTX MSP, USART2 PA2/PA3 for ESC telemetry, TIM3 PC6–PC9 for four motor outputs, ADC PC0/PC1 for battery/current sensing, USB PA11/PA12, and SWD PA13/PA14.
- Use `MAIN_3V3` for MCU/flash/BMP280 and independent `IMU_3V3` for ICM-42688-P VDD/VDDIO.
- Show fuse/current limiting, reverse-polarity MOSFET protection, battery-rated TVS, input filtering, TPS54360 buck details, USB VBUS isolation, and rail test points in the schematic.
- Do not infer CRSF/ELRS, GPS, ESC telemetry, current-sense, or VTX electrical levels from protocol names; preserve protection/translation footprints and record assumptions.
- USB-C is sink-only with CC1/CC2 Rd pull-downs, VBUS sensing, connector-side ESD, and no USB-PD or battery charging.
- All prototype external connectors use 2.54 mm headers unless the exact standard library search proves a matching equivalent footprint is required.
- Do not claim direct-battery safety, flight readiness, controlled impedance, or verified DShot DMA until the specified bring-up and review checks pass.
- The directory is not a Git repository; do not create commits or alter unrelated files.

---

### Task 1: Create and inventory the KiCad project

**Files:**
- Create: `gofly_fc.kicad_pro`
- Create: `gofly_fc.kicad_sch`
- Create: `gofly_fc.kicad_sym`
- Create if required: `gofly_fc.pretty/`
- Read: `docs/superpowers/specs/2026-09-20-fpv-flight-controller-design.md`
- Read: `research/fpv-flight-controller-sources.md`

**Interfaces:**
- Consumes: the approved specification and source record.
- Produces: an opened KiCad project, a known project-local library strategy, and a component inventory with exact symbol/footprint identifiers.

- [ ] **Step 1: Verify the KiCad backend and project directory.**

  Use `mcp__kicad__check_kicad_ui`, then `mcp__kicad__get_backend_state`. Confirm that the working directory is `C:\Users\admin\gofly` and that no existing KiCad files will be overwritten.

- [ ] **Step 2: Create and open the project.**

  Call `mcp__kicad__create_project` with `name="gofly_fc"` and `path="C:\Users\admin\gofly"`, then `mcp__kicad__open_project` on `C:\Users\admin\gofly\gofly_fc.kicad_pro`. Create the root schematic at `C:\Users\admin\gofly\gofly_fc.kicad_sch` if the project creator did not create it.

- [ ] **Step 3: Resolve standard symbols and footprints before placement.**

  Search with `mcp__kicad__search_symbols` and `mcp__kicad__search_footprints` for the exact parts and package families: STM32F405RGT6 LQFP64, ICM-42688-P 14-LGA, BMP280 8-LGA, W25Q128JV SOIC-8 or WSON-8, MAX7456 28-pin package, TPS54360 PowerPAD package, 3.3 V low-noise LDOs, USB-C receptacle, USB ESD array, MOSFETs, TVS, fuse, crystal, resistor/capacitor families, and 2.54 mm headers.

- [ ] **Step 4: Create only missing project-local symbols or footprints.**

  Create `ICM-42688-P` from the selected DS-000347 Rev. 1.9 table (INT1 pin 4, INT2/FSYNC/CLKIN pin 9, reserved pins 2/3/7/10/11 explicitly marked) and create `MAX7456` from the verified 28-pin TSSOP-EP table. Register both in `gofly_fc.kicad_sym`. Reuse the standard `Logic_LevelTranslator:SN74LVC1T45DBV` symbol. If a required footprint is absent, create it in `gofly_fc.pretty` with `mcp__kicad__create_footprint`, register it with `mcp__kicad__register_footprint_library`, and validate it with the applicable library/footprint checks.

- [ ] **Step 5: Verify pin and package identity.**

  Use `mcp__kicad__batch_list_symbol_pins` for the MCU and every custom symbol. Confirm the STM32 symbol exposes PA2/PA3, PA5–PA7, PA8, PA9–PA15, PB0/PB1/PB2, PB3–PB5, PB8/PB9, PB10/PB11, PB12–PB15, PC0/PC1, PC4–PC13, PH0/PH1, VCAP1/VCAP2, all VDD/VSS, VDDA/VSSA, BOOT0, and NRST with the correct pin numbers. Stop before schematic capture if any package pin identity conflicts with DS8626.

**Acceptance:** The project opens, every required part has a verified symbol and footprint identifier, missing-library artifacts are project-local and structurally valid, and no existing user file was overwritten.

---

### Task 2: Create the modular schematic structure and power subsystem

**Files:**
- Modify: `gofly_fc.kicad_sch`
- Create: `power.kicad_sch` if hierarchical sheets are used

**Interfaces:**
- Consumes: Task 1 project/library inventory.
- Produces: named rails and protected input network used by every later schematic block.

- [ ] **Step 1: Add a power sheet or power block.**

  Prefer `mcp__kicad__create_hierarchical_subsheet` to create `power.kicad_sch` linked from the root. If the installed KiCad backend cannot maintain hierarchical sheet metadata reliably, keep the same block in the root schematic with a clearly labeled `POWER INPUT / RAILS` section; do not create an unlinked orphan sheet.

- [ ] **Step 2: Place the battery and protection components.**

  Place a 2-pin `VBAT_IN` header, fuse/current limiter, reverse-polarity MOSFET stage, battery-rated TVS, input bulk capacitor, ceramic input capacitors, and test points. Use explicit nets `VBAT_IN`, `VBAT_PROTECTED`, and `PGND_IN`.

- [ ] **Step 3: Place and wire the TPS54360 5 V buck.**

  Place TPS54360, EN/UVLO divider, 0.1 µF BOOT capacitor, catch diode, inductor, feedback divider for nominal 5 V, compensation components, output capacitors, and a PowerPAD thermal-via note. Use `SYS_5V` as the output net and expose a test point. Keep the high-di/dt loop electrically explicit in the schematic.

- [ ] **Step 4: Add USB VBUS isolation and the two 3.3 V rails.**

  Add USB VBUS fuse/current limiting, reverse-current blocking/OR-ing, `USB_VBUS_RAW`, and the `SYS_5V` merge point. Add one main 3.3 V LDO producing `MAIN_3V3` and one low-noise LDO producing `IMU_3V3`; add rail enable/default pull components and test points.

- [ ] **Step 5: Add power flags and net labels.**

  Use `mcp__kicad__batch_connect` or exact-pin `mcp__kicad__connect_to_net` calls to label every regulator input/output, ground, enable, and test point. Add PWR_FLAG symbols only where required to make the intended source/consumer relationship unambiguous.

- [ ] **Step 6: Add the analog measurement front end.**

  Add `VBAT_SENSE` and `CURRENT_SENSE` dividers/RC filters from the selected ESC sensor assumptions, with clamp/protection footprints and explicit maximum expected voltage notes. Do not connect raw battery voltage directly to an MCU pin.

**Acceptance:** The power block has no unlabeled supply nodes, each rail has a test point, USB cannot back-feed the battery rail in the intended topology, and the TPS54360 network includes every required support component from the specification.

---

### Task 3: Add STM32 core, clock, reset, debug, and resource-labeled nets

**Files:**
- Modify: `gofly_fc.kicad_sch`
- Modify: `power.kicad_sch` only if rail labels need a hierarchical connection

**Interfaces:**
- Consumes: `MAIN_3V3`, `IMU_3V3`, `SYS_5V`, `VBAT_SENSE`, and `CURRENT_SENSE` from Task 2.
- Produces: a complete STM32F405RGT6 core with the approved pin/net allocation for all later blocks.

- [ ] **Step 1: Place the STM32F405RGT6 LQFP64 symbol.**

  Use `mcp__kicad__add_schematic_component` or `mcp__kicad__batch_add_components` with the verified STM32 symbol and `MAIN_3V3` supply. Place it centrally in the MCU/core block and assign the exact LQFP64 footprint.

- [ ] **Step 2: Wire all MCU power and regulator pins.**

  Connect every VDD/VSS pair, VDDA/VSSA, VCAP1 pin 31, and VCAP2 pin 47. Place one 2.2 µF low-ESR capacitor at each VCAP pin, local VDD bypass capacitors, and the VDDA filter/bypass network. Add BOOT0 100 kΩ pulldown and NRST pull/reset components.

- [ ] **Step 3: Add the 8 MHz HSE network.**

  Place the 8 MHz crystal and two load capacitors on PH0/PH1 using the selected crystal's load calculation. Label `HSE_IN` and `HSE_OUT` and keep the network local to the MCU.

- [ ] **Step 4: Add SWD and USB-C interface pins at the MCU boundary.**

  Label PA13 `SWDIO`, PA14 `SWCLK`, NRST `NRST`, PA11 `USB_DM`, and PA12 `USB_DP`. Add a 2.54 mm SWD header with `3V3`, `GND`, `SWDIO`, `SWCLK`, and `NRST`.

- [ ] **Step 5: Add every approved MCU signal label exactly once.**

  Use the following exact net names: `IMU_SCK`, `IMU_MISO`, `IMU_MOSI`, `IMU_CS`, `IMU_INT1`, `FLASH_SCK`, `FLASH_MISO`, `FLASH_MOSI`, `FLASH_CS`, `OSD_SCK`, `OSD_MISO`, `OSD_MOSI`, `OSD_CS`, `OSD_RESET`, `I2C1_SCL`, `I2C1_SDA`, `CRSF_BUS`, `GPS_TX`, `GPS_RX`, `VTX_MSP_TX`, `VTX_MSP_RX`, `ESC_TELEM_RX`, `ESC_TELEM_TX`, `MOTOR1`, `MOTOR2`, `MOTOR3`, `MOTOR4`, `VBAT_SENSE`, `CURRENT_SENSE`, `USB_VBUS_SENSE`, `BUZZER_CTRL`, `LED_STRIP`, and `STATUS_LED`.

- [ ] **Step 6: Run a pin-allocation collision check.**

  Use `mcp__kicad__get_schematic_component` and `mcp__kicad__get_schematic_pin_locations` for the MCU, then inspect the generated netlist with `mcp__kicad__generate_netlist`. Confirm no required pin is assigned to two nets and that no PE pin appears anywhere in the MCU signal map.

**Acceptance:** The MCU has all power/clock/debug connections, exact package VCAP pins, every required signal is labeled at the correct GPIO/peripheral pin, and the generated netlist contains no duplicate or impossible MCU assignments.

---

### Task 4: Add sensors, flash, OSD, level translation, and clocks

**Files:**
- Modify: `gofly_fc.kicad_sch`
- Create: `sensors_storage_video.kicad_sch` only if hierarchical sheets are used

**Interfaces:**
- Consumes: SPI/I2C/CS/interrupt nets and rails from Task 3.
- Produces: sensor, flash, and analog OSD circuits connected to named MCU nets.

- [ ] **Step 1: Place and wire ICM-42688-P.**

  Connect VDD/VDDIO to `IMU_3V3`, SPI1 nets to PA5/PA6/PA7, `IMU_CS` to PB0, and the validated INT1 device pin to `IMU_INT1` on PC4. Add 0.1 µF + 2.2 µF on VDD and 10 nF on VDDIO. Follow the current official sensor pin table for reserved/GND pins and add an INT2 test pad without inventing an MCU interrupt assignment.

- [ ] **Step 2: Place and wire BMP280.**

  Connect VDD/VDDIO to `MAIN_3V3`, CSB to `MAIN_3V3`, SDO to GND for address `0x76`, and SDA/SCL to PB8/PB9. Add the two 100 nF bypass capacitors and 4.7–10 kΩ I²C pull-ups. Do not add a switched barometer rail.

- [ ] **Step 3: Place and wire W25Q128JV.**

  Connect standard SPI3 PB3/PB4/PB5, `FLASH_CS` to PB1, VCC to `MAIN_3V3`, GND, `/WP` and `/HOLD`/`/RESET` to defined inactive pull states, and provide test pads for the reserved Quad-SPI lines. Add the selected local decoupling capacitor and a power-up note.

- [ ] **Step 4: Place and wire MAX7456 analog OSD.**

  Connect AVDD/DVDD/PVDD to `SYS_5V`, SPI2 PB13/PB14/PB15, `OSD_CS` to PB12, `OSD_RESET` to PA1 through the reset network, the 27 MHz crystal, `CAM_VIDEO` input coupling/75 Ω network, and `VTX_VIDEO` output coupling/termination. Label the block `ANALOG CVBS OSD — MAX7456` and add an obsolete-lifecycle property.

- [ ] **Step 5: Add the SDOUT voltage-protection/translation footprint.**

  Insert a one-direction 5 V-to-3.3 V buffer/translator between MAX7456 SDOUT and the STM32 input net. If the chosen STM32 input is not yet assigned to a required function, label the translated signal `OSD_SDOUT_3V3` and expose a test pad; never connect a 5 V MAX7456 output directly to an unverified STM32 input.

- [ ] **Step 6: Run component-level structural checks.**

  Use `mcp__kicad__validate_schematic` and `mcp__kicad__generate_netlist`. Check that all sensor supply pins, device chip selects, straps, bypass capacitors, and video coupling parts appear in the netlist.

**Acceptance:** IMU, BMP280, flash, and MAX7456 are fully connected with correct rails/straps, no sensor input is exposed to an invalid rail during normal power-up, and the MAX7456 SDOUT path contains an explicit safe interface.

---

### Task 5: Add external interfaces, connectors, protection, and documentation labels

**Files:**
- Modify: `gofly_fc.kicad_sch`

**Interfaces:**
- Consumes: all named MCU and power nets from Tasks 2–4.
- Produces: complete user-facing connector pinouts and protected interface boundaries.

- [ ] **Step 1: Add the 4-in-1 ESC header.**

  Use a 2.54 mm header with `GND`, `5V_ESC`, `MOTOR1`–`MOTOR4`, `ESC_TELEM_RX`, `VBAT_SENSE`, and `CURRENT_SENSE`. Add series resistors/RC filtering or translator footprints where selected ESC electrical levels require them. Do not route high-current battery/motor power through this board.

- [ ] **Step 2: Add CRSF/ELRS receiver header.**

  Add `GND`, `5V_RX`, and `CRSF_BUS` on a 2.54 mm header. Add a series protection resistor and optional inversion/level-translation footprint controlled by the selected receiver electrical assumptions. Label the net as single-wire half-duplex and do not duplicate it as two independent signal nets.

- [ ] **Step 3: Add GPS header and shared I²C expansion.**

  Add `GND`, `5V_GPS`, `GPS_TX`, `GPS_RX`, `I2C1_SCL`, and `I2C1_SDA`, with optional series resistors and explicit supply-current notes.

- [ ] **Step 4: Add UART4 MSP digital-VTX header.**

  Add `GND`, `VTX_BEC_5V`, `VTX_MSP_TX`, and `VTX_MSP_RX`. Place a note that the external VTX owns high-speed video processing and that `VTX_BEC_5V` is not an unconditional `SYS_5V` load output.

- [ ] **Step 5: Add analog camera/VTX, buzzer, LED, and USB-C.**

  Add CVBS camera/VTX connector pins, a low-side transistor/MOSFET buzzer driver controlled by `BUZZER_CTRL`, board status LED on `STATUS_LED`, LED strip header on `LED_STRIP`, and a USB-C receptacle with PA11/PA12, VBUS fuse/isolation, CC1/CC2 5.1 kΩ Rd, connector-side low-capacitance ESD, and VBUS divider to `USB_VBUS_SENSE`.

- [ ] **Step 6: Add functional notes and voltage labels.**

  Use schematic text to mark `4S–6S PROTECTED INPUT`, `USB-C SINK ONLY`, `MAX7456 ANALOG CVBS ONLY`, `UART4 MSP — EXTERNAL DIGITAL VTX`, `DO NOT CONNECT RAW VBAT TO MCU`, and `EXTERNAL VTX BEC CURRENT NOT PROVIDED`. Add net labels at every connector pin so the pinout remains readable after PCB annotation.

**Acceptance:** Every required external interface exists on the schematic, each connector pin has an exact net name and expected voltage role, USB-C and external serial boundaries have protection footprints, and the analog/digital video distinction is visible in the schematic.

---

### Task 6: Annotate, validate, and close schematic ERC

**Files:**
- Modify: `gofly_fc.kicad_sch`

**Interfaces:**
- Consumes: complete schematic from Tasks 2–5.
- Produces: an annotated, structurally valid schematic ready for PCB synchronization.

- [ ] **Step 1: Annotate and normalize fields.**

  Run `mcp__kicad__annotate_schematic`, then `mcp__kicad__autoplace_schematic_fields`. Use `mcp__kicad__batch_edit_schematic_components` to set clear values/footprints for every passive, regulator, connector, and translator.

- [ ] **Step 2: Validate file structure.**

  Run `mcp__kicad__validate_schematic` with `runKicadCli=true`. Fix only task-owned structural errors such as malformed symbols, broken hierarchy, orphan labels, and invalid pin references.

- [ ] **Step 3: Check connectivity hygiene.**

  Run `mcp__kicad__find_orphaned_wires`, `mcp__kicad__list_floating_labels`, `mcp__kicad__find_wires_crossing_symbols`, and `mcp__kicad__find_overlapping_elements`. Resolve all accidental floating labels/wires and document intentional no-connects with `mcp__kicad__batch_add_no_connects`.

- [ ] **Step 4: Run ERC and classify every remaining result.**

  Run `mcp__kicad__run_erc`. Correct power-driver, unconnected-pin, illegal-input, and label errors. Only leave a warning when it is an intentional prototype boundary, and add a schematic note explaining the external dependency rather than suppressing it blindly.

- [ ] **Step 5: Inspect the rendered schematic.**

  Use `mcp__kicad__get_schematic_view` and, for dense blocks, `mcp__kicad__get_schematic_view_region`. Confirm all block headings, connector pinouts, power flow, sensor orientation, and video labels are legible.

**Acceptance:** The schematic validates structurally, has no accidental orphaned wires/floating labels, has no unresolved ERC errors, and renders legibly enough for a human review before PCB creation.

---

### Task 7: Generate the PCB, stackup, outline, and mechanical features

**Files:**
- Create: `gofly_fc.kicad_pcb`
- Modify: `gofly_fc.kicad_pro` if board setup is persisted there

**Interfaces:**
- Consumes: annotated/ERC-clean `gofly_fc.kicad_sch`.
- Produces: a four-layer board with footprints, nets, outline, and mounting geometry.

- [ ] **Step 1: Create/synchronize the board from the schematic.**

  Use `mcp__kicad__create_board_from_schematic` or `mcp__kicad__sync_schematic_to_board` with the exact root schematic and board path. Confirm all schematic footprints and net names appear in `mcp__kicad__get_component_list` and `mcp__kicad__get_nets_list`.

- [ ] **Step 2: Set the board stackup and design rules.**

  Use `mcp__kicad__set_design_rules` for prototype clearance, minimum track width, via/drill limits, and courtyard requirement. Use `mcp__kicad__get_layer_list`; add/configure the four-layer stack so F.Cu, In1.Cu, In2.Cu, and B.Cu match the approved layer intent. Do not claim controlled impedance without a fabricator stackup.

- [ ] **Step 3: Create the 40 mm × 40 mm outline.**

  Use `mcp__kicad__replace_board_outline` with a 40 mm × 40 mm rectangle anchored at the project grid origin. Verify it with `mcp__kicad__get_board_extents` and `mcp__kicad__list_graphics` filtered to `Edge.Cuts`.

- [ ] **Step 4: Add the mounting holes.**

  Use `mcp__kicad__add_mounting_hole` for four 3.2 mm NPTH holes at the 30.5 mm × 30.5 mm pattern centered on the board. Run `mcp__kicad__check_courtyard_overlaps` with boundary checking enabled.

- [ ] **Step 5: Add board labels and test-point identity.**

  Add concise front-silkscreen labels for `VBAT`, `ESC`, `RX`, `GPS`, `VTX`, `CAM`, `SWD`, `USB`, and polarity/ground marks with `mcp__kicad__add_board_text`. Do not place text over pads, mounting holes, or antenna/connector keepouts.

**Acceptance:** The board contains every schematic footprint/net, has a verified 40 × 40 mm outline, correct mounting-hole pattern, four copper layers, and no component or courtyard is outside the outline.

---

### Task 8: Place components by noise, current, and connector zones

**Files:**
- Modify: `gofly_fc.kicad_pcb`

**Interfaces:**
- Consumes: generated board from Task 7 and the placement zones in the specification.
- Produces: a mechanically legal placement ready for routing.

- [ ] **Step 1: Identify all footprint geometry and anchors.**

  Use `mcp__kicad__get_component_geometry`, `mcp__kicad__get_component_properties`, and `mcp__kicad__get_component_pads` for the MCU, IMU, BMP280, MAX7456, TPS54360, inductors, USB-C, headers, crystals, and mounting holes.

- [ ] **Step 2: Place the power/input cluster.**

  Put VBAT connector, fuse, reverse-polarity MOSFET, TVS, input bulk, TPS54360, catch diode, inductor, and 5 V output capacitors along one board edge. Keep the SW node and high-di/dt loop compact and away from the IMU and analog video.

- [ ] **Step 3: Place the MCU/digital cluster.**

  Place the STM32 centrally, W25Q128JV adjacent to the SPI3 pins, USB-C and ESD at a board edge, and SWD/header access on a separate edge. Keep PA11/PA12 escape short with an uninterrupted In1.Cu reference.

- [ ] **Step 4: Place the IMU and barometer.**

  Place ICM-42688-P near the geometric center with its decoupling capacitors immediately adjacent and no buck switch-node copper below it. Place BMP280 in a vented edge/quiet area away from hot components and prop-wash obstruction.

- [ ] **Step 5: Place the analog-video cluster.**

  Place MAX7456, its 27 MHz crystal, SDOUT translator, CVBS coupling/termination components, camera connector, and analog VTX connector together, isolated from the switching node and motor outputs.

- [ ] **Step 6: Place external headers and test points.**

  Align ESC, CRSF/ELRS, GPS, UART4 MSP/VTX, camera, analog VTX, SWD, USB, buzzer, LED, and rail test points around the remaining edges with clear silkscreen orientation. Use `mcp__kicad__batch_move_components` for transactional moves.

- [ ] **Step 7: Check placement before routing.**

  Run `mcp__kicad__check_placement_clearance` and `mcp__kicad__check_courtyard_overlaps`. Resolve every body/courtyard/board-boundary violation before adding traces or zones.

**Acceptance:** Placement satisfies the functional zones, all footprints are inside the board, mounting holes are clear, and the IMU/power/video separation is visible in the 2D board render.

---

### Task 9: Route critical nets, add ground/power copper, and complete PCB connectivity

**Files:**
- Modify: `gofly_fc.kicad_pcb`

**Interfaces:**
- Consumes: legal placement from Task 8.
- Produces: routed prototype PCB with critical loops, buses, connectors, and copper zones implemented.

- [ ] **Step 1: Route the power converter first.**

  Route `VBAT_PROTECTED`, TPS54360 VIN bypass, switch/catch-diode/inductor loop, `SYS_5V`, `MAIN_3V3`, and `IMU_3V3` with short, wide traces and appropriate vias. Keep the SW node small and away from signal traces. Use `mcp__kicad__route_trace` or `mcp__kicad__route_pad_to_pad` after obtaining exact pad positions.

- [ ] **Step 2: Route IMU, flash, and MCU buses.**

  Route SPI1 to ICM-42688-P as a compact group with short chip-select/interrupt traces; route SPI3 to W25Q128JV; route SPI2 to MAX7456/translator. Keep bus returns over the continuous In1.Cu ground reference and avoid crossing the power switch region.

- [ ] **Step 3: Route USB and SWD.**

  Route PA11/PA12 to the USB-C ESD array/receptacle as a short matched pair with continuous reference. Route SWDIO/SWCLK/NRST to the header without crossing the buck switch node.

- [ ] **Step 4: Route motor, UART, ADC, and connector signals.**

  Route PC6–PC9 motor outputs to the ESC header, USART1 CRSF, USART3 GPS, UART4 VTX MSP, USART2 ESC telemetry, ADC sense lines, buzzer, LED, and I²C expansion. Add series resistors/filters where the schematic includes them.

- [ ] **Step 5: Route analog CVBS.**

  Route camera VIN and MAX7456 VOUT through the specified coupling/75 Ω networks with short paths and no parallel run beside the buck SW node or motor outputs. Keep the analog ground return intentional and tied to the main ground plane at the documented region.

- [ ] **Step 6: Add copper zones.**

  Add a continuous GND zone on In1.Cu using `mcp__kicad__add_zone`; add only deliberate low-current power pours on In2.Cu if they do not fragment the reference plane. Refill with `mcp__kicad__refill_zones` through the supported backend and inspect the fill state.

- [ ] **Step 7: Measure remaining airwires and route the remainder.**

  Use `mcp__kicad__get_ratsnest`/`mcp__kicad__estimate_airwire_lengths` and `mcp__kicad__query_traces`. Route every remaining required signal manually or with `mcp__kicad__autoroute` only after critical nets are protected; inspect and correct any autorouter path that violates the placement or noise rules.

- [ ] **Step 8: Save and reload the board.**

  Use `mcp__kicad__save_board`, then `mcp__kicad__reload_board` to ensure the persisted board matches the in-memory layout and no external-change conflict exists.

**Acceptance:** All required nets are routed or intentionally connected through zones, the GND plane is filled, critical power/USB/IMU/video routing meets the placement rules, and the ratsnest contains no accidental required connections.

---

### Task 10: Run final validation, generate review artifacts, and report limits

**Files:**
- Modify: `gofly_fc.kicad_sch` only for task-owned ERC/field corrections
- Modify: `gofly_fc.kicad_pcb` only for task-owned DRC/placement corrections
- Create: `outputs/` only if export tools require an output directory

**Interfaces:**
- Consumes: completed schematic and PCB from Tasks 6 and 9.
- Produces: verified project files, rendered review artifacts, and a concise limitation report for the user.

- [ ] **Step 1: Re-run schematic checks.**

  Run `mcp__kicad__validate_schematic`, `mcp__kicad__run_erc`, `mcp__kicad__find_orphaned_wires`, `mcp__kicad__list_floating_labels`, and `mcp__kicad__generate_netlist`. Confirm the PCB netlist and schematic netlist share the exact names for every motor, sensor, UART, USB, SWD, power, and video net.

- [ ] **Step 2: Run PCB geometry and DRC checks.**

  Run `mcp__kicad__run_drc`, `mcp__kicad__check_placement_clearance`, `mcp__kicad__check_courtyard_overlaps`, `mcp__kicad__get_drc_violations`, and `mcp__kicad__get_board_info`. Fix task-owned errors and report any warnings that arise from explicit prototype boundaries.

- [ ] **Step 3: Inspect visual artifacts.**

  Render the schematic with `mcp__kicad__get_schematic_view` and the board with `mcp__kicad__get_board_2d_view` using at least `F.Cu`, `B.Cu`, `In1.Cu`, `In2.Cu`, and `Edge.Cuts`. Inspect connector readability, mounting-hole clearance, IMU/power separation, USB routing, video routing, and silkscreen orientation.

- [ ] **Step 4: Export manufacturing/review data.**

  Export a BOM with `mcp__kicad__export_bom`, a position file with `mcp__kicad__export_position_file`, and a schematic PDF/SVG with the corresponding KiCad MCP export tool if the files are useful for review. Keep generated outputs under `outputs/` and do not add secrets or external credentials.

- [ ] **Step 5: Record unresolved engineering gates.**

  Report exact component ordering codes, receiver/ESC/GPS/VTX electrical assumptions, final timer DMA verification status, TPS54360 transient-test status, MAX7456 lifecycle risk, fabricator stackup/impedance status, and whether direct battery bring-up has been performed. Never describe the result as flight-ready unless those tests were actually observed.

- [ ] **Step 6: Save a project checkpoint.**

  After all checks pass, call `mcp__kicad__snapshot_project` with step `layout_ok` and label `initial_fc_prototype`. The checkpoint prompt must state the actual ERC/DRC counts and any remaining warnings rather than claiming a clean result without evidence.

**Acceptance:** The final response names the created KiCad files, lists actual ERC/DRC/geometry results, includes rendered review artifacts when available, distinguishes verified results from untested hardware assumptions, and reports all remaining safety/interoperability gates.

---

## Self-review checklist

- **Spec coverage:** Tasks 2–5 cover the power tree, MCU resources, all sensors/storage, MAX7456 analog video, UART4 MSP digital-VTX boundary, USB-C, SWD, ESC, CRSF/ELRS, GPS, buzzer, LED, and measurement interfaces. Tasks 7–9 cover the 40 mm outline, four-layer intent, 30.5 mm mounting pattern, placement zones, routing, and ground/power copper. Tasks 6 and 10 cover schematic/PCB validation and bring-up reporting.
- **No placeholders:** The plan uses explicit net names, pins, packages, tool calls, rail names, board dimensions, and verification commands. Conditional instructions are limited to library availability and backend support, with a concrete fallback path.
- **Type/identity consistency:** `CRSF_BUS` is one USART1 half-duplex net; `GPS_TX/GPS_RX` use USART3 PB10/PB11; `VTX_MSP_TX/VTX_MSP_RX` use UART4 PC10/PC11; `ESC_TELEM_RX/ESC_TELEM_TX` use USART2 PA2/PA3; `MOTOR1–4` use PC6–PC9/TIM3; `VBAT_SENSE/CURRENT_SENSE` use PC0/PC1; all later tasks consume the same names.
- **Safety boundary:** No task treats TPS54360's 65 V absolute maximum as sufficient protection, no task connects raw 5 V or VBAT directly to an unverified MCU input, and no task calls the board flight-ready before bring-up tests.
