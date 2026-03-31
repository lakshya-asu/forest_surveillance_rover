cmake_minimum_required(VERSION 3.15)

# Toolchain file for ARM-GCC cross-compilation
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Toolchain settings
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

# No operating system
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Project definition
project(forest-rover-firmware C CXX ASM)

# MCU and compiler settings
set(MCU STM32F407xx)
set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(COMMON_FLAGS "${CPU_FLAGS} -Wall -Wextra -O2 -g3 -ffunction-sections -fdata-sections")

set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=c99")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++14 -fno-exceptions -fno-rtti")
set(CMAKE_ASM_FLAGS "${COMMON_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS "${CPU_FLAGS} -Wl,--gc-sections -Wl,--print-memory-usage")

# Include directories
include_directories(
  Core/Inc
  Drivers/STM32F4xx_HAL_Driver/Inc
  Middlewares/Third_Party/FreeRTOS/include
)

# Source files
file(GLOB_RECURSE SOURCES
  Core/Src/*.c
  Drivers/STM32F4xx_HAL_Driver/Src/*.c
  Middlewares/Third_Party/FreeRTOS/*.c
)

# Add definitions
add_definitions(
  -D${MCU}
  -DUSE_HAL_DRIVER
  -DFREERTOS
)

# Main executable
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Generate HEX and BIN files
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
  COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Enable testing
enable_testing()

# Summary
message(STATUS "=== Forest Rover STM32F407 Firmware ===")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "C Flags: ${CMAKE_C_FLAGS}")
message(STATUS "Linker Flags: ${CMAKE_EXE_LINKER_FLAGS}")
message(STATUS "========================================")
