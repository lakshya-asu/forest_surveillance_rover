# STM32F407 Firmware Project

STM32 real-time firmware for Forest Surveillance Rover.

## Project Structure

- `Core/Inc/` - Header files for application
- `Core/Src/` - Source files for application and FreeRTOS tasks
- `Drivers/` - STM32 HAL drivers
- `Middlewares/` - FreeRTOS kernel
- `Tools/` - OpenOCD debugging config, bootloader
- `Tests/` - Unit and integration tests

## Build Instructions

### Prerequisites

```bash
# ARM GCC toolchain
sudo apt install arm-none-eabi-gcc arm-none-eabi-gdb

# STM32CubeIDE (optional, for visual configuration)
# Download from https://www.st.com/stm32cubeide

# OpenOCD (for debugging)
sudo apt install openocd
```

### Build

```bash
cd STM32_FirmwareProject
mkdir -p build
cd build
cmake ..
make -j4
```

### Flash & Debug

```bash
# Via OpenOCD
openocd -f ../Tools/openocd/forest-rover-stm32f407.cfg

# In another terminal
arm-none-eabi-gdb build/forest-rover.elf
# (gdb) target remote localhost:3333
# (gdb) load
# (gdb) continue
```

## Phase 0 Status

- [x] Directory structure created
- [ ] CMakeLists.txt with ARM-GCC setup
- [ ] Main entry point (main.c)
- [ ] STM32 HAL initialization
- [ ] FreeRTOS kernel setup
- [ ] UART driver skeleton
- [ ] Motor control skeleton
- [ ] Sensor drivers skeleton

See IMPLEMENTATION_PLAN.md for full roadmap.
