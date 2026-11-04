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

project(H_MC_V3.0 C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# ── Generated Git Build Information ────────────────────────────
find_package(Git QUIET)
set(MC_GIT_BRANCH "unknown")
set(MC_GIT_COMMIT "unknown")
set(MC_GIT_TAG "unknown")
if(Git_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE MC_GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE MC_GIT_COMMIT_FULL
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(MC_GIT_COMMIT_FULL)
        string(SUBSTRING "${MC_GIT_COMMIT_FULL}" 0 6 MC_GIT_COMMIT)
    endif()
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" describe --tags --always
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE MC_GIT_TAG
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" diff --quiet
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE MC_GIT_DIFF_RESULT
        ERROR_QUIET
    )
    if(NOT MC_GIT_DIFF_RESULT EQUAL 0)
        string(APPEND MC_GIT_COMMIT "-dirty")
    endif()
endif()

file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/generated")
string(TIMESTAMP MC_BUILD_DATE "%b %d %Y")
string(TIMESTAMP MC_BUILD_TIME "%H:%M:%S")
configure_file(
    "${CMAKE_SOURCE_DIR}/cmake/git_info.h.in"
    "${CMAKE_BINARY_DIR}/generated/git_info.h"
    @ONLY
)

# ── Global Compile Options ─────────────────────────────────────
add_compile_options(
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -O2
    -fno-math-errno
    -Wall
    -fdata-sections
    -ffunction-sections
    -fno-common
)

add_compile_options(
    $<$<COMPILE_LANGUAGE:C>:-flto=auto>
    $<$<COMPILE_LANGUAGE:C>:-fno-fat-lto-objects>
)

add_link_options(
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -O2
    -flto=auto
    -fno-fat-lto-objects
    -T${CMAKE_SOURCE_DIR}/boards/stm32g431/common/board/STM32G431CB_FLASH.ld
    -Wl,--gc-sections
    -Wl,--wrap=printf
    -Wl,--wrap=sprintf
    -Wl,--wrap=snprintf
    -Wl,--wrap=vsnprintf
    -Wl,-u,__errno
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
    ARM_DSP_CONFIG_TABLES
    ARM_FFT_ALLOW_TABLES
    ARM_TABLE_TWIDDLECOEF_F32_512
    ARM_TABLE_BITREVIDX_FLT_512
    ARM_TABLE_TWIDDLECOEF_RFFT_F32_1024
    PRINTF_DISABLE_SUPPORT_EXPONENTIAL
    PRINTF_DISABLE_SUPPORT_LONG_LONG
)

# ── Application, module, driver and board libraries ───────────
include_directories(
    ${CMAKE_BINARY_DIR}/generated
    boards/stm32g431/common/board/Core/Inc
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Inc
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    boards/stm32g431/common/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    boards/stm32g431/common/Drivers/CMSIS/Core/Include
    boards/stm32g431/common/Drivers/CMSIS/DSP/Include
)

add_subdirectory(src)
add_subdirectory(boards)

# ── Core Sources ───────────────────────────────────────────────
set(CORE_SOURCES
    boards/stm32g431/common/board/Core/Src/adc.c
    boards/stm32g431/common/board/Core/Src/cordic.c
    boards/stm32g431/common/board/Core/Src/gpio.c
    boards/stm32g431/common/board/Core/Src/main.c
    boards/stm32g431/common/board/Core/Src/spi.c
    boards/stm32g431/common/board/Core/Src/stm32g4xx_hal_msp.c
    boards/stm32g431/common/board/Core/Src/stm32g4xx_it.c
    boards/stm32g431/common/board/Core/Src/system_stm32g4xx.c
    boards/stm32g431/common/board/Core/Src/tim.c
    boards/stm32g431/common/board/Core/Src/usart.c
)

# ── HAL Driver Sources ─────────────────────────────────────────
set(HAL_SOURCES
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cordic.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_dma.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_exti.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_gpio.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rcc.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_spi.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_tim.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usart.c
    boards/stm32g431/common/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_utils.c
)

set(CMSIS_DSP_SOURCES
    boards/stm32g431/common/Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
    boards/stm32g431/common/Drivers/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal2.c
    boards/stm32g431/common/Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_f32.c
    boards/stm32g431/common/Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix8_f32.c
    boards/stm32g431/common/Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_f32.c
    boards/stm32g431/common/Drivers/CMSIS/DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.c
)

# ── Startup ────────────────────────────────────────────────────
set(STARTUP_SOURCE
    boards/stm32g431/common/Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g431xx.s
)

# ── Target ─────────────────────────────────────────────────────
add_executable(${CMAKE_PROJECT_NAME}
    ${CORE_SOURCES}
    ${HAL_SOURCES}
    ${CMSIS_DSP_SOURCES}
    ${STARTUP_SOURCE}
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE mc_board mc_app)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES SUFFIX ".elf")

# ── Post-build: generate .hex and .bin ─────────────────────────
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating .hex and .bin files"
)
