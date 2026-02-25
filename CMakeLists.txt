cmake_minimum_required(VERSION 3.16)

# 1. 强制跳过编译器检查 (解决 _exit 报错)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

# 2. 交叉编译设置
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# --------------------------------------------------------------------------------
# 工具链路径：优先使用环境变量 ARM_TOOLCHAIN_PATH，未设置则用默认值
# 用法：cmake -DARM_TOOLCHAIN_PATH="your/path/bin" ..
# --------------------------------------------------------------------------------
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

project(STM32_AIoT_Project C CXX ASM)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

# 3. 宏定义
add_definitions(
    -DSTM32F103xB        
    -DUSE_HAL_DRIVER     
)

# 4. 编译选项
set(MCU_FLAGS "-mcpu=cortex-m3 -mthumb -mfloat-abi=soft")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS} -Wall -fdata-sections -ffunction-sections -Os")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${MCU_FLAGS} -x assembler-with-cpp")

# --------------------------------------------------------------------------------
# 【路径修复】使用相对路径，彻底解决 Windows 盘符报错
# --------------------------------------------------------------------------------
set(LDSCRIPT "../Core/Startup/STM32F103C8Tx_FLASH.ld")

# 链接参数
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MCU_FLAGS} -T${LDSCRIPT} -Wl,--gc-sections -specs=nosys.specs -specs=nano.specs")

# 5. 头文件搜索路径
include_directories(
    Core/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc
    Drivers/CMSIS/Device/ST/STM32F1xx/Include
    Drivers/CMSIS/Include
    Middlewares/FreeRTOS/Inc
    Middlewares/FreeRTOS/CMSIS_RTOS_V2
    Middlewares/FreeRTOS/Port/ARM_CM3
    Hardware/W25Qxx
    Hardware/ESP32
    Hardware/DHT11
    Hardware/Crypto
    App/OTA/Inc
    App/MQTT/Inc
    App/Test/Inc
    App/Sensor/Inc
)

# 6. 源文件搜索
file(GLOB_RECURSE SOURCES 
    "Core/Src/*.c"
    "Core/Startup/*.s"
    "Drivers/STM32F1xx_HAL_Driver/Src/*.c"
    "Middlewares/FreeRTOS/Src/*.c"
    "Middlewares/FreeRTOS/Port/ARM_CM3/*.c"
    "Middlewares/FreeRTOS/Port/MemMang/heap_4.c"
    "Middlewares/FreeRTOS/CMSIS_RTOS_V2/*.c"
    "Hardware/**/*.c"
    "App/**/*.c"
)

# 7. 生成目标
add_executable(${PROJECT_NAME} ${SOURCES})

# 8. 生成 bin/hex
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
)