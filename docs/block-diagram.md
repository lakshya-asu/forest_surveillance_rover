# System Block Diagram

```mermaid
flowchart TD
    Solar[Solar Panel Input<br/>18V OC] --> RPP[Reverse-Polarity MOSFET]
    RPP --> MPPT[LT3652-Style MPPT Charger]
    MPPT --> Batt[3S LiPo Battery Pack]

    Batt --> Buck[TPS54331 5V Buck]
    Buck --> LDO[AMS1117-3.3 LDO]
    Batt --> EFuse[TPS2596 eFuse]

    LDO --> STM[STM32F407VGT6]
    STM --> Flash[W25Q128 SPI Flash]
    STM --> LoRa[RFM95W LoRa Module]
    STM --> Enc[Quadrature Encoder Inputs]
    STM --> AFE[PIR + Thermistor Analog Front-End]
    STM --> I2C[I2C Sensor Bus]

    I2C --> BME[BME280]
    I2C --> BNO[BNO055]
    STM --> Lidar[LIDAR JST UART]
    STM --> Cam[Thermal Camera JST]

    EFuse --> M1[DRV8871 Motor A]
    EFuse --> M2[DRV8871 Motor B]

    Buck --> CM4[Raspberry Pi CM4]
    STM --> CM4
    LoRa --> SMA[SMA Antenna]
```

## Partitioning Notes

- Power conversion sits at one end of the board to keep high-current loops compact.
- The RF section stays on the board edge with a short feed to the SMA connector.
- Analog sensing is separated from motor and switching power stages.
- The STM32 acts as the real-time controller, while the CM4 is the companion compute node.
