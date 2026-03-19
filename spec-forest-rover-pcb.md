# Forest Surveillance Rover — Main Controller PCB

## Project Overview

A 4-layer KiCAD PCB project for an autonomous forest surveillance rover built for Team Onyx India. This board serves as the rover's central electronics hub, integrating motor control, solar power management, LoRa telemetry, and multi-sensor fusion on a single board. The repo should look like a real, portfolio-worthy hardware project with full documentation.

## Scope (Simplified for Portfolio)

This is a **simplified but realistic** version of the actual project. Focus on making the schematic and layout plausible and well-documented — not on producing a fabrication-ready board. KiCAD symbol/footprint libraries should use built-in KiCAD libraries wherever possible.

## Repo Structure

```
forest-rover-pcb/
├── README.md                          # Project overview with rendered images
├── LICENSE                            # MIT
├── docs/
│   ├── design-notes.md                # Design decisions, tradeoffs, constraints
│   ├── block-diagram.md               # System-level block diagram (mermaid or ASCII)
│   ├── bom.csv                        # Bill of materials
│   └── power-budget.md                # Power consumption analysis
├── hardware/
│   ├── kicad/
│   │   ├── forest-rover.kicad_pro     # KiCAD 7+ project file
│   │   ├── forest-rover.kicad_sch     # Top-level schematic
│   │   ├── forest-rover.kicad_pcb     # PCB layout (4-layer)
│   │   └── sym-lib-table              # Symbol library table
│   └── gerbers/                       # Exported Gerber files (placeholder README)
│       └── README.md
├── simulation/
│   ├── ngspice/
│   │   ├── buck-converter.spice       # 12V→5V buck converter transient sim
│   │   ├── battery-charger.spice      # Solar MPPT charge controller sim
│   │   └── README.md                  # How to run sims
│   └── qucs-s/
│       ├── sensor-frontend.sch        # Analog sensor conditioning circuit
│       └── README.md
├── firmware/
│   └── README.md                      # Placeholder: "firmware lives in separate repo"
└── images/
    └── README.md                      # "Rendered board images go here"
```

## Schematic Blocks (what the KiCAD schematic should contain)

Design the schematic with these hierarchical sheets / sections:

### 1. Power Management
- **Input:** 3S LiPo battery (11.1V nominal) OR solar panel (18V OC)
- **Solar MPPT charge controller:** Based on LT3652 (or simplified equivalent)
  - Input: solar panel via reverse-polarity protection MOSFET
  - Output: charges 3S LiPo pack
  - Sense resistors for charge current limiting (2A max)
- **Main 5V rail:** TPS54331 buck converter (12V → 5V, 3A)
  - Powers: Raspberry Pi CM4, sensors, LoRa module
  - LC output filter, bootstrap cap, feedback divider
- **3.3V rail:** AMS1117-3.3 LDO from 5V rail
  - Powers: MCU, analog sensor front-ends
- **Motor driver supply:** Direct from battery through e-fuse (TPS2596)
  - Overcurrent protection, soft-start

### 2. Microcontroller (STM32F407VGT6)
- 100-pin LQFP
- Decoupling: 100nF per VDD pin + 4.7µF bulk
- 8MHz HSE crystal, 32.768kHz LSE crystal
- SWD debug header (2x5 pin, 1.27mm)
- BOOT0 pin with pull-down + jumper
- GPIO breakout header for expansion
- UART1 → Raspberry Pi CM4 (compute module)
- UART2 → LoRa module
- SPI1 → External flash (W25Q128)
- I2C1 → Sensor bus
- ADC channels → battery voltage divider, current sense, temperature

### 3. Motor Control
- **Dual H-Bridge:** DRV8871 (x2) for two DC gear motors
  - PWM input from STM32 timer channels
  - Current sense resistor per channel
  - Flyback diodes (schottky)
- **Quadrature encoder inputs** (x2): differential line receivers

### 4. Sensor Interfaces
- **I2C sensor bus** (directly on the kicad schematic):
  - BME280 (temp/humidity/pressure) — I2C
  - BNO055 IMU — I2C
  - Pull-up resistors (4.7kΩ to 3.3V)
- **Analog front-end:**
  - PIR motion sensor amplifier (op-amp buffer + comparator with adjustable threshold)
  - Thermistor voltage divider for ambient temperature
- **Connector headers for off-board sensors:**
  - LIDAR (UART, 4-pin JST)
  - Thermal camera (I2C/SPI, 6-pin JST)

### 5. Communication
- **LoRa module:** RFM95W footprint
  - SPI interface to STM32
  - Antenna: SMA edge-mount connector
  - RF matching network (PI network placeholder)
- **Raspberry Pi CM4 interface:**
  - UART (TX/RX/GND)
  - I2C (shared bus)
  - GPIO interrupt line (STM32 → Pi)
  - USB-C connector for Pi power + data

### 6. Connectors & Misc
- Battery connector: XT60
- Solar panel input: MC4-to-screw-terminal
- Power switch: latching push button with soft-start
- Status LEDs: Power (green), Charging (amber), Fault (red), Comms (blue)
- Mounting holes: M3, 4 corners

## PCB Layout Notes

- **4-layer stackup:** Signal – GND – Power – Signal
- **Board size:** ~100mm × 80mm
- **Key layout constraints:**
  - Keep switching regulators away from analog sensor front-end
  - RF section (LoRa + antenna) on board edge, ground pour underneath
  - Motor driver section isolated with ground stitching vias
  - Thermal relief on power MOSFET pads
- **Copper pours:** Solid GND on layer 2, split power planes on layer 3

## Ngspice Simulations

### buck-converter.spice
Simulate the TPS54331-based 5V buck converter:
- Transient simulation: startup behavior, output voltage settling
- Load step response: 0.5A → 2.5A step
- Use simplified behavioral model (voltage-mode PWM controller)
- Plot: Vout, inductor current, switch node voltage
- **Keep it simple:** ~50-80 lines of SPICE, use `.tran`, `.control` / `.endc` blocks

### battery-charger.spice
Simulate the solar MPPT charging circuit:
- DC sweep: solar panel IV curve (use diode-based solar cell model)
- Show charge current regulation at 2A limit
- **Keep it simple:** ~40-60 lines of SPICE

## Qucs-S Simulation

### sensor-frontend.sch
- Op-amp buffer for PIR sensor signal
- 2nd order Sallen-Key low-pass filter (fc ≈ 10Hz) for PIR signal conditioning
- AC simulation showing filter response
- Use generic op-amp model (e.g., LM358)

## README.md Content

The main README should include:
- Project title and one-line description
- Photo/render placeholder (with alt text describing the board)
- **Features** list (bullet points of key specs)
- **Block diagram** (mermaid diagram embedded in markdown)
- **Design tools:** KiCAD 7, Ngspice, Qucs-S
- **How to open:** instructions for cloning and opening in KiCAD
- **Simulation:** how to run the Ngspice sims (`ngspice buck-converter.spice`)
- **Design decisions:** brief rationale for key choices (4-layer, component selection)
- **Status:** "Prototype Rev A — fabricated and tested"
- **License:** MIT

## BOM (docs/bom.csv)

Generate a realistic BOM with columns:
`Reference, Value, Footprint, Manufacturer, MPN, Quantity, Unit Cost (USD), Notes`

Include ~25-35 line items covering the major components listed above. Use real manufacturer part numbers where possible.

## Power Budget (docs/power-budget.md)

Simple markdown table:
| Subsystem | Voltage | Current (typ) | Current (max) | Notes |
With totals showing the system fits within the solar + battery power envelope.

## Design Notes (docs/design-notes.md)

~500 words covering:
- Why 4-layer stackup (EMC, power integrity)
- Solar MPPT vs direct charge tradeoff
- LoRa range considerations and antenna placement
- Motor noise isolation strategy
- Component availability choices (JLCPCB basic parts where possible)

## Style & Tone

- Professional but not dry — this is a portfolio piece
- Include "lessons learned" where appropriate
- Reference real datasheets and application notes where relevant
- Use proper EDA terminology (DRC, ERC, copper pour, thermal relief, etc.)
