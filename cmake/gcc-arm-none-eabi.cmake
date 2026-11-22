set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# =============================================================================
# 查找 arm-none-eabi 工具链
#
# 查找优先级:
#   1. -DARM_GCC_PATH=<bin目录>             (命令行/CLion 传入)
#   2. 环境变量 ARM_GCC_PATH
#   3. PATH 中的 arm-none-eabi-gcc
#   4. 常见安装位置(xpack / ARM 官方 / STM32CubeCLT / STM32CubeIDE)
#
# 找不到时会给出明确的报错和处理方法, 而不是让 CMake 报 "is not a full path"。
# =============================================================================
set(ARM_GCC_PATH "" CACHE PATH "arm-none-eabi 工具链的 bin 目录(留空则自动查找)")

set(_arm_gcc_dir "")

if(ARM_GCC_PATH)
    set(_arm_gcc_dir "${ARM_GCC_PATH}")
elseif(DEFINED ENV{ARM_GCC_PATH} AND NOT "$ENV{ARM_GCC_PATH}" STREQUAL "")
    set(_arm_gcc_dir "$ENV{ARM_GCC_PATH}")
else()
    # 3. PATH
    find_program(_arm_gcc_program arm-none-eabi-gcc)
    if(_arm_gcc_program)
        get_filename_component(_arm_gcc_dir "${_arm_gcc_program}" DIRECTORY)
    else()
        # 4. 常见安装位置(Windows 为主, 每项都是 ".../bin" 目录)
        set(_arm_gcc_globs
            "$ENV{LOCALAPPDATA}/arm-gcc/*/bin"
            "$ENV{USERPROFILE}/AppData/Local/arm-gcc/*/bin"
            "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/*/bin"
            "C:/Program Files/Arm GNU Toolchain arm-none-eabi/*/bin"
            "C:/Program Files/ARM GNU Toolchain arm-none-eabi/*/bin"
            "C:/ST/STM32CubeCLT*/GNU-tools-for-stm32/bin"
            "C:/ST/STM32CubeIDE*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin"
            "/opt/arm-gnu-toolchain*/bin"
            "/usr/local/arm-gnu-toolchain*/bin"
        )

        foreach(_glob ${_arm_gcc_globs})
            file(GLOB _arm_gcc_candidates "${_glob}/arm-none-eabi-gcc.exe"
                                          "${_glob}/arm-none-eabi-gcc")
            if(_arm_gcc_candidates)
                list(SORT _arm_gcc_candidates)
                list(GET _arm_gcc_candidates -1 _arm_gcc_pick) # 取版本号最大的一个
                get_filename_component(_arm_gcc_dir "${_arm_gcc_pick}" DIRECTORY)
                break()
            endif()
        endforeach()
    endif()
endif()

if(NOT _arm_gcc_dir)
    message(FATAL_ERROR
        "未找到 arm-none-eabi-gcc!\n"
        "请任选一种方式解决:\n"
        "  1) 安装工具链: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads\n"
        "     (xPack 版: https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases)\n"
        "  2) 把工具链的 bin 目录加入系统环境变量 PATH, 然后重启 VS Code;\n"
        "  3) 配置 CMake 时显式指定: -DARM_GCC_PATH=\"C:/xxx/bin\",\n"
        "     或在 CMakePresets.json 的 cacheVariables 里加 \"ARM_GCC_PATH\": \"C:/xxx/bin\".\n"
        "查找过的位置包括: PATH, %LOCALAPPDATA%/arm-gcc/*/bin, Program Files 下的 Arm GNU Toolchain,\n"
        "STM32CubeCLT / STM32CubeIDE 自带工具链.")
endif()

message(STATUS "ARM toolchain: ${_arm_gcc_dir}")

# 使用绝对路径, 这样即使 PATH 里没有工具链也能编译.
# 注意: Windows 下必须让 find_program 解析出带 .exe 的完整路径, 直接拼 "xxx/arm-none-eabi-g++"
#       会被 CMake 判定为 "不是有效的编译器路径"。
find_program(ARM_GCC_EXECUTABLE NAMES arm-none-eabi-gcc HINTS "${_arm_gcc_dir}" NO_DEFAULT_PATH)
find_program(ARM_GXX_EXECUTABLE NAMES arm-none-eabi-g++ HINTS "${_arm_gcc_dir}" NO_DEFAULT_PATH)
find_program(ARM_OBJCOPY_EXECUTABLE NAMES arm-none-eabi-objcopy HINTS "${_arm_gcc_dir}" NO_DEFAULT_PATH)
find_program(ARM_SIZE_EXECUTABLE NAMES arm-none-eabi-size HINTS "${_arm_gcc_dir}" NO_DEFAULT_PATH)

# 兜底: 如果上面没找到(例如只装了 gcc), 再在 PATH 里找一次
if(NOT ARM_GCC_EXECUTABLE)
    find_program(ARM_GCC_EXECUTABLE NAMES arm-none-eabi-gcc)
endif()
if(NOT ARM_GXX_EXECUTABLE)
    find_program(ARM_GXX_EXECUTABLE NAMES arm-none-eabi-g++)
endif()
if(NOT ARM_OBJCOPY_EXECUTABLE)
    find_program(ARM_OBJCOPY_EXECUTABLE NAMES arm-none-eabi-objcopy)
endif()
if(NOT ARM_SIZE_EXECUTABLE)
    find_program(ARM_SIZE_EXECUTABLE NAMES arm-none-eabi-size)
endif()

if(NOT ARM_GCC_EXECUTABLE OR NOT ARM_GXX_EXECUTABLE)
    message(FATAL_ERROR
        "找到目录 ${_arm_gcc_dir}, 但其中没有 arm-none-eabi-gcc / arm-none-eabi-g++.\n"
        "请安装完整的 ARM GNU Toolchain(xPack 或官方版), 或设置 -DARM_GCC_PATH 指向正确的 bin 目录.")
endif()

set(TOOLCHAIN_PREFIX "${_arm_gcc_dir}/arm-none-eabi-") # 仅供参考/兼容旧配置, 实际使用下面的绝对路径

set(CMAKE_C_COMPILER                "${ARM_GCC_EXECUTABLE}")
set(CMAKE_ASM_COMPILER              "${ARM_GCC_EXECUTABLE}")
set(CMAKE_CXX_COMPILER              "${ARM_GXX_EXECUTABLE}")
set(CMAKE_LINKER                    "${ARM_GXX_EXECUTABLE}")
set(CMAKE_OBJCOPY                   "${ARM_OBJCOPY_EXECUTABLE}")
set(CMAKE_SIZE                      "${ARM_SIZE_EXECUTABLE}")

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32H723xG_flash.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
