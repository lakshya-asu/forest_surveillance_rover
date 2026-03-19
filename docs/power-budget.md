# Power Budget

| Subsystem | Voltage | Current (typ) | Current (max) | Notes |
| --- | --- | ---: | ---: | --- |
| STM32F407 + clocks + flash | 3.3V | 90 mA | 140 mA | Includes MCU core, crystals, and W25Q128 activity |
| BME280 + BNO055 | 3.3V | 18 mA | 28 mA | Shared I2C bus with pull-ups |
| Analog front-end | 3.3V | 12 mA | 20 mA | PIR buffer, comparator, thermistor divider |
| LoRa module RFM95W | 3.3V | 45 mA | 120 mA | Peak TX current depends on output power |
| Raspberry Pi CM4 interface budget | 5V | 450 mA | 900 mA | Budgetary allowance for CM4 carrier-side load |
| LIDAR connector budget | 5V | 180 mA | 350 mA | Off-board UART sensor allowance |
| Thermal camera connector budget | 5V | 160 mA | 300 mA | Off-board sensor allowance |
| Status LEDs and logic overhead | 3.3V | 16 mA | 25 mA | Four indicators plus pull-ups and glue logic |
| Motor driver logic | 5V | 20 mA | 35 mA | DRV8871 control-side consumption only |
| DC Gear Motor A | Battery | 650 mA | 1.8 A | Typical cruise vs stall-limited transient |
| DC Gear Motor B | Battery | 650 mA | 1.8 A | Typical cruise vs stall-limited transient |
| Buck converter losses | 12V in | 90 mA eq. | 180 mA eq. | Estimated conversion overhead at peak load |

## Totals

| Rail / Source | Typical Load | Maximum Load | Notes |
| --- | ---: | ---: | --- |
| 3.3V rail | 181 mA | 333 mA | Within AMS1117 thermal envelope only with moderate dissipation; validate in real hardware |
| 5V rail | 810 mA | 1.585 A | Safely inside a 3A TPS54331 design target |
| Battery motor path | 1.30 A | 3.60 A | eFuse and wiring sized for short-duration peak events |
| Whole system input power | ~19.5 W | ~53 W peak | Peak assumes both motors near stall and all loads active |

## Envelope Check

- The 5V electronics rail fits comfortably inside the 3A buck converter target, leaving design margin for startup transients.
- The solar charger is modeled around a 2A charge-current limit, which is reasonable for a 3S pack in a portable rover form factor.
- Continuous operation is expected to rely on the battery, with solar extending mission time and offsetting idle/low-load consumption rather than covering worst-case motor peaks by itself.
- A real Rev B should revisit the 3.3V LDO thermal dissipation if the 5V rail operates near its upper load range for long periods.
