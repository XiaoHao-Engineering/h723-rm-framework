cmake_minimum_required(VERSION 3.22)

#
# 达妙 DM-MC-Board02 (STM32H723VGT6) 工程
#
# 结构说明:
#   Core/       CubeMX 生成的 H7 HAL 初始化代码(时钟/GPIO/FDCAN/SPI/UART/ADC/USB)
#   bsp/        板级支持包: 对 HAL 的实例化封装(dwt/log/can/usart/spi/pwm/adc/gpio/flash/usb/ws2812)
#   modules/    模块层: 电机/IMU/算法/遥控/通信/守护线程等, 与具体硬件解耦
#   application/应用层: robot.c 作为总入口, demo/ 是达妙板示例, rm_example/ 是 RM 步兵示例(参考)
#   Middlewares/Third_Party/FreeRTOS  FreeRTOS 内核 + CMSIS-RTOS v1 封装
#   Middlewares/Third_Party/SEGGER    SEGGER RTT(日志通过 SWD 输出, 不占串口)
#
# 默认编译达妙板示例 app; 想编译 RM 步兵示例请在 CMake 里把 USE_RM_EXAMPLE 设为 ON.
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RM)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# ------------------------------------------------------------------------------
# 应用选择
#   OFF(默认): 达妙板示例 app —— IMU/蜂鸣器/WS2812/按键/电压/VOFA + 可选电机闭环
#   ON       : RoboMaster 步兵示例 app —— 需要裁判系统、底盘、云台等硬件
# ------------------------------------------------------------------------------
option(USE_RM_EXAMPLE "编译并使用 RoboMaster 步兵示例 app(需要裁判系统/底盘/云台硬件)" OFF)
if(USE_RM_EXAMPLE)
    message(STATUS "Application: RoboMaster infantry example")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USE_RM_EXAMPLE=1)
else()
    message(STATUS "Application: DAMIAO MC02 board demo")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USE_RM_EXAMPLE=0)
endif()

# ------------------------------------------------------------------------------
# 最小自检固件: 只测 "5V + 蜂鸣器 + 彩灯", 跳过 USB/ADC/FDCAN/串口/RTOS/IMU.
# 用于快速区分 "硬件问题" 和 "框架初始化问题":
#   cmake --preset Debug -DUSE_MINIMAL_TEST=ON
#   cmake --build --preset Debug && .\.flash\flash.ps1 -NoBuild
# 测完记得改回 OFF 重新编译。
# ------------------------------------------------------------------------------
option(USE_MINIMAL_TEST "编译最小自检固件(只测 5V/蜂鸣器/彩灯, 跳过框架初始化)" OFF)
if(USE_MINIMAL_TEST)
    message(STATUS "*** MINIMAL SELF-TEST firmware: only 5V / buzzer / ws2812 ***")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USE_MINIMAL_TEST=1)
else()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USE_MINIMAL_TEST=0)
endif()

# ------------------------------------------------------------------------------
# 递归收集 bsp/modules/application 下所有子目录作为 include 路径
# (三层框架的代码都用 "头文件名" 直接 include, 例如 #include "bsp_can.h")
# ------------------------------------------------------------------------------
set(FRAMEWORK_INCLUDE_DIRS "")
foreach(layer bsp modules application)
    list(APPEND FRAMEWORK_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/${layer}")
    file(GLOB_RECURSE _layer_entries LIST_DIRECTORIES true "${CMAKE_SOURCE_DIR}/${layer}/*")
    foreach(entry ${_layer_entries})
        if(IS_DIRECTORY "${entry}")
            list(APPEND FRAMEWORK_INCLUDE_DIRS "${entry}")
        endif()
    endforeach()
endforeach()

# ------------------------------------------------------------------------------
# 源码
# ------------------------------------------------------------------------------
file(GLOB_RECURSE FRAMEWORK_SOURCES
    "${CMAKE_SOURCE_DIR}/bsp/*.c"
    "${CMAKE_SOURCE_DIR}/modules/*.c"
    "${CMAKE_SOURCE_DIR}/application/*.c"
)

set(FREERTOS_SOURCES
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/list.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
)

set(SEGGER_SOURCES
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.c
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_printf.c
)

# RTT 的汇编实现(可选, 用于更快的 no-lock 写入)
set(SEGGER_ASM_SOURCES
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s
)
set_source_files_properties(${SEGGER_ASM_SOURCES} PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${FRAMEWORK_SOURCES}
    ${FREERTOS_SOURCES}
    ${SEGGER_SOURCES}
    ${SEGGER_ASM_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${FRAMEWORK_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/SEGGER/RTT
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/SEGGER/Config
)

# ------------------------------------------------------------------------------
# 编译选项
# ------------------------------------------------------------------------------
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wall
    -fdata-sections
    -ffunction-sections
    -fstack-usage
)
# 框架和 HAL 中有不少"未使用参数"的回调, 不当作错误
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wno-unused-parameter)

# 不要在不使用 FPU 时保存 FPU 上下文, 由 FreeRTOS 的 port 决定
# H7 的 FPU/CPU 参数来自 cmake/gcc-arm-none-eabi.cmake

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# ------------------------------------------------------------------------------
# 生成 hex/bin 文件, 方便用 OpenOCD/JLink 烧录
# 注意: 这里用 $<TARGET_FILE_NAME:...>(相对文件名) 而不是绝对路径,
#       避免工程路径里含有中文/空格时 cmd.exe 传参编码出错.
# ------------------------------------------------------------------------------
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE_NAME:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE_NAME:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE_NAME:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating hex & bin file..."
)
