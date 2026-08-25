cmake_minimum_required(VERSION 3.20)

# ── Toolchain (must be set BEFORE project()) ────────────────────
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)
set(CMAKE_C_COMPILER    arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER  arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_OBJCOPY       arm-none-eabi-objcopy)
set(CMAKE_SIZE          arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(H_FOC_V3.0 C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# ── Global Compile Options ─────────────────────────────────────
add_compile_options(
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -Wall
    -fdata-sections
    -ffunction-sections
    -fno-common
)

add_link_options(
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -T${CMAKE_SOURCE_DIR}/Code/STM32G431CB_FLASH.ld
    -Wl,--gc-sections
    -Wl,-u,_printf_float
    -specs=nosys.specs
    -specs=nano.specs
    -Wl,--start-group
    -lc
    -lm
    -Wl,--end-group
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--no-warn-rwx-segments
    -Wl,--print-memory-usage
)

# ── Defines ────────────────────────────────────────────────────
add_compile_definitions(
    USE_FULL_LL_DRIVER
    USE_HAL_DRIVER
    STM32G431xx
    ARM_MATH_CM4
)

# ── Include Paths ──────────────────────────────────────────────
include_directories(
    Code/APP
    Code/MID
    Code/Core/Inc
    Code/Drivers/STM32G4xx_HAL_Driver/Inc
    Code/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    Code/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    Code/Drivers/CMSIS/Core/Include
)

# ── APP Sources ────────────────────────────────────────────────
file(GLOB_RECURSE APP_SOURCES Code/APP/*.c)

# ── MID Sources ────────────────────────────────────────────────
file(GLOB_RECURSE MID_SOURCES Code/MID/*.c)

# ── Core Sources ───────────────────────────────────────────────
set(CORE_SOURCES
    Code/Core/Src/adc.c
    Code/Core/Src/cordic.c
    Code/Core/Src/fdcan.c
    Code/Core/Src/gpio.c
    Code/Core/Src/main.c
    Code/Core/Src/spi.c
    Code/Core/Src/stm32g4xx_hal_msp.c
    Code/Core/Src/stm32g4xx_it.c
    Code/Core/Src/system_stm32g4xx.c
    Code/Core/Src/tim.c
    Code/Core/Src/usart.c
)

# ── HAL Driver Sources ─────────────────────────────────────────
set(HAL_SOURCES
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cordic.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_dma.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_exti.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_gpio.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rcc.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_spi.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_tim.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usart.c
    Code/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_utils.c
)

# ── Startup ────────────────────────────────────────────────────
set(STARTUP_SOURCE
    Code/Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g431xx.s
)

# ── Target ─────────────────────────────────────────────────────
add_executable(${CMAKE_PROJECT_NAME}
    ${APP_SOURCES}
    ${MID_SOURCES}
    ${CORE_SOURCES}
    ${HAL_SOURCES}
    ${STARTUP_SOURCE}
)

# ── Post-build: generate .hex and .bin ─────────────────────────
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating .hex and .bin files"
)
