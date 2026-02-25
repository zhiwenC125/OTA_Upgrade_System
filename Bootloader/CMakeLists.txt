cmake_minimum_required(VERSION 3.16)

# 强制跳过编译器检查
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

# 交叉编译设置
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 工具链路径
if(NOT DEFINED ARM_TOOLCHAIN_PATH)
    if(DEFINED ENV{ARM_TOOLCHAIN_PATH})
        set(ARM_TOOLCHAIN_PATH "$ENV{ARM_TOOLCHAIN_PATH}")
    else()
        set(ARM_TOOLCHAIN_PATH "D:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin")
    endif()
endif()
set(TOOLCHAIN_PATH "${ARM_TOOLCHAIN_PATH}")

set(CMAKE_C_COMPILER "${TOOLCHAIN_PATH}/arm-none-eabi-gcc.exe")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PATH}/arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PATH}/arm-none-eabi-g++.exe")
set(CMAKE_OBJCOPY "${TOOLCHAIN_PATH}/arm-none-eabi-objcopy.exe")
set(CMAKE_SIZE_UTIL "${TOOLCHAIN_PATH}/arm-none-eabi-size.exe")

project(STM32_Bootloader C ASM)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

# 宏定义
add_definitions(
    -DSTM32F103xB
    -DUSE_HAL_DRIVER
)

# 编译选项
set(MCU_FLAGS "-mcpu=cortex-m3 -mthumb -mfloat-abi=soft")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS} -Wall -fdata-sections -ffunction-sections -Os")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${MCU_FLAGS} -x assembler-with-cpp")

# 链接脚本
set(LDSCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8Tx_BOOT.ld")

# 链接参数
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MCU_FLAGS} -T${LDSCRIPT} -Wl,--gc-sections -specs=nosys.specs -specs=nano.specs")

# 头文件路径（只需要 HAL 和 CMSIS）
include_directories(
    ../Core/Inc
    ../Drivers/STM32F1xx_HAL_Driver/Inc
    ../Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ../Drivers/CMSIS/Include
)

# 源文件：Bootloader 自身 + HAL 驱动 + 启动文件
set(SOURCES
    main.c
    ../Core/Startup/startup_stm32f103xb.s
)

# HAL 源文件（Bootloader 需要：Flash + SPI + GPIO + RCC）
list(APPEND SOURCES
    ../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    ../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    ../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    ../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    ../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    ../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    ../Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c
)

# Bootloader 专用 SystemInit（向量表在 0x08000000，无需偏移）
# 不使用 App 的 system_stm32f1xx.c（其中 VTOR 偏移了 0x2000）
list(APPEND SOURCES
    system_boot.c
)

# 生成目标
add_executable(${PROJECT_NAME} ${SOURCES})

# 生成 bin/hex
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE_UTIL} ${PROJECT_NAME}.elf
)
