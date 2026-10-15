set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(TOOLCHAIN_PREFIX arm-none-eabi-)
get_filename_component(DEVENV_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../DevEnv" ABSOLUTE)
set(ARM_TOOLCHAIN_ROOT "" CACHE PATH
    "Arm GNU Toolchain root (contains bin/); empty selects bundled native tools or PATH")
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES ARM_TOOLCHAIN_ROOT)

# Use the HOST platform: the target is Generic/ARM on every operating system.
set(ARM_TOOL_SUFFIX "")
if(CMAKE_HOST_WIN32)
    set(ARM_TOOL_SUFFIX ".exe")
    set(BUNDLED_TOOLCHAIN_ROOT "${DEVENV_ROOT}/GNU-tools-for-STM32")
elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
    set(BUNDLED_TOOLCHAIN_ROOT "${DEVENV_ROOT}/linux/gcc")
else()
    set(BUNDLED_TOOLCHAIN_ROOT "")
endif()

unset(ARM_GCC)
unset(ARM_GCC CACHE)
if(ARM_TOOLCHAIN_ROOT)
    # An explicit override is strict: never silently mix installations.
    find_program(ARM_GCC NAMES "${TOOLCHAIN_PREFIX}gcc${ARM_TOOL_SUFFIX}"
        PATHS "${ARM_TOOLCHAIN_ROOT}/bin" NO_DEFAULT_PATH NO_CACHE)
else()
    if(BUNDLED_TOOLCHAIN_ROOT)
        find_program(ARM_GCC NAMES "${TOOLCHAIN_PREFIX}gcc${ARM_TOOL_SUFFIX}"
            PATHS "${BUNDLED_TOOLCHAIN_ROOT}/bin" NO_DEFAULT_PATH NO_CACHE)
    endif()
    if(NOT ARM_GCC)
        find_program(ARM_GCC NAMES "${TOOLCHAIN_PREFIX}gcc${ARM_TOOL_SUFFIX}"
            NO_CACHE)
    endif()
endif()
if(NOT ARM_GCC)
    message(FATAL_ERROR "Native arm-none-eabi-gcc not found. On Linux run "
        "bash ${DEVENV_ROOT}/install-linux.sh, install a native toolchain on PATH, "
        "or set -DARM_TOOLCHAIN_ROOT=/path/to/toolchain.")
endif()
get_filename_component(ARM_TOOLCHAIN_BIN "${ARM_GCC}" DIRECTORY)

function(find_arm_tool TOOL_NAME OUTPUT_VARIABLE)
    unset(FOUND_TOOL)
    unset(FOUND_TOOL CACHE)
    find_program(FOUND_TOOL NAMES "${TOOL_NAME}${ARM_TOOL_SUFFIX}"
        PATHS "${ARM_TOOLCHAIN_BIN}" NO_DEFAULT_PATH NO_CACHE)
    if(NOT FOUND_TOOL)
        message(FATAL_ERROR "Missing ${TOOL_NAME} in ${ARM_TOOLCHAIN_BIN}; install a complete toolchain.")
    endif()
    if(NOT CMAKE_HOST_WIN32)
        file(READ "${FOUND_TOOL}" TOOL_MAGIC LIMIT 2 HEX)
        if(TOOL_MAGIC STREQUAL "4d5a")
            message(FATAL_ERROR "${FOUND_TOOL} is a Windows executable. Install native host tools.")
        endif()
    endif()
    set(${OUTPUT_VARIABLE} "${FOUND_TOOL}" PARENT_SCOPE)
endfunction()

find_arm_tool(${TOOLCHAIN_PREFIX}gcc ARM_GCC)
find_arm_tool(${TOOLCHAIN_PREFIX}g++ ARM_GXX)
find_arm_tool(${TOOLCHAIN_PREFIX}objcopy ARM_OBJCOPY)
find_arm_tool(${TOOLCHAIN_PREFIX}size ARM_SIZE)
find_arm_tool(${TOOLCHAIN_PREFIX}ar ARM_AR)
find_arm_tool(${TOOLCHAIN_PREFIX}ranlib ARM_RANLIB)
find_arm_tool(${TOOLCHAIN_PREFIX}ld ARM_LD)

set(CMAKE_C_COMPILER                "${ARM_GCC}")
set(CMAKE_ASM_COMPILER              "${ARM_GCC}")
set(CMAKE_CXX_COMPILER              "${ARM_GXX}")
set(CMAKE_LINKER                    "${ARM_LD}")
set(CMAKE_AR                        "${ARM_AR}")
set(CMAKE_RANLIB                    "${ARM_RANLIB}")
set(CMAKE_OBJCOPY                   "${ARM_OBJCOPY}")
set(CMAKE_SIZE                      "${ARM_SIZE}")

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

# Initializers avoid appending duplicate flags when CMake reads this file twice.
set(CMAKE_C_FLAGS_INIT "${TARGET_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")
set(CMAKE_ASM_FLAGS_INIT "${TARGET_FLAGS} -x assembler-with-cpp -MMD -MP")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

get_filename_component(FIRMWARE_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(CMAKE_EXE_LINKER_FLAGS_INIT "${TARGET_FLAGS} -T \"${FIRMWARE_ROOT}/STM32F407XX_FLASH.ld\" --specs=nano.specs -Wl,-Map=smart_car.map -Wl,-gc-sections -Wl,--print-memory-usage -u _printf_float -u _scanf_float")
set(TOOLCHAIN_LINK_LIBRARIES "m" "c" "nosys")
