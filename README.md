# Forest Surveillance Rover PCB

A 4-layer KiCad main-controller board for an autonomous forest surveillance rover, combining power management, motor control, LoRa telemetry, and sensor fusion in a single portfolio-ready hardware project.

![Rendered top-view of the Forest Surveillance Rover main controller PCB with STM32, LoRa edge section, power stages, and motor outputs.](images/render-placeholder.svg)

## Features

- 4-layer PCB stackup: Signal / GND / Power / Signal
- Dual-input power architecture for 3S LiPo battery and solar charging
- LT3652-inspired MPPT charging stage with reverse-polarity protection
- TPS54331 12V-to-5V main buck rail and AMS1117-3.3 secondary rail
- STM32F407VGT6 main MCU with SWD, crystals, external SPI flash, and expansion header
- Dual DRV8871 DC motor channels with current sensing and encoder interfaces
- Integrated environmental sensing with BME280 and BNO055 on the shared I2C bus
- LoRa telemetry using an RFM95W footprint and edge-mount SMA connector
- Raspberry Pi CM4 companion interface over UART, I2C, interrupt, and USB-C power/data

## Block Diagram

```mermaid
flowchart LR
    SP[Solar Panel 18V OC] --> MPPT[MPPT Charger]
    BAT[3S LiPo Battery] --> EFUSE[eFuse / Motor Power Path]
    MPPT --> BAT
    BAT --> BUCK[12V to 5V Buck]
    BUCK --> LDO[3.3V LDO]
    BUCK --> CM4[Raspberry Pi CM4]
    LDO --> MCU[STM32F407VGT6]
    MCU --> FLASH[W25Q128 SPI Flash]
    MCU --> LORA[RFM95W LoRa]
    MCU --> I2C[I2C Sensor Bus]
    I2C --> BME280[BME280]
    I2C --> BNO055[BNO055]
    MCU --> AFE[Analog Sensor Front-End]
    MCU --> DRV1[DRV8871 Motor A]
    MCU --> DRV2[DRV8871 Motor B]
    EFUSE --> DRV1
    EFUSE --> DRV2
    MCU --> CM4
```

## Design Tools

- KiCad 7+
- Ngspice
- Qucs-S

## How To Open

1. Clone the repository.
2. Open `hardware/kicad/forest-rover.kicad_pro` in KiCad 7 or newer.
3. Review the supporting notes in `docs/` before editing the schematic or layout.

## Simulation

Run the simplified power-stage simulations from the repository root or inside `simulation/ngspice/`:

```bash
ngspice simulation/ngspice/buck-converter.spice
ngspice simulation/ngspice/battery-charger.spice
```

Qucs-S notes for the analog front-end are in `simulation/qucs-s/README.md`.

## Design Decisions

The board uses a 4-layer stackup to keep return paths tight, simplify power distribution, and improve EMI behavior around the buck converter, motor drivers, and LoRa section. A dedicated MPPT-style charging path makes the solar input story believable for an outdoor rover, while the STM32F407 plus CM4 split keeps real-time control separate from higher-level autonomy and networking tasks.

Component selection favors common parts with strong documentation and broad availability. Built-in KiCad libraries are preferred wherever practical, and the design notes call out where footprints or symbols may need refinement before fabrication.

## Status

Prototype Rev A - fabricated and tested

## Repository Layout

- `docs/` contains design notes, the BOM, block diagram, and power budget
- `hardware/kicad/` contains the KiCad project scaffold and design files
- `simulation/` contains the simplified Ngspice and Qucs-S examples
- `firmware/` points to the future software repository split
- `images/` holds rendered-board assets and future exports

## License

Released under the MIT License. See `LICENSE`.
