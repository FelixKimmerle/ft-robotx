cmake_minimum_required(VERSION 3.28)

# Programs run on the controller's ARM CPU, so this project always uses the ARM cross toolchain.
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake)
endif()

project(FtRoboTxOffline
    VERSION 1.0.0
    DESCRIPTION "ft-robotx-online, offline part: C programs that run on the fischertechnik ROBO TX Controller"
    LANGUAGES C ASM)

set(FTROBOTX_PORT "" CACHE STRING "Port for the deploy targets: /dev/ttyACM0, bt:XX:XX:XX:XX:XX:XX, ... (empty: find the USB controller)")
set(FTROBOTX_LOAD ${CMAKE_CURRENT_SOURCE_DIR}/tools/ftrobotx-load)
find_package(Python3 COMPONENTS Interpreter) # for the deploy targets

# Startup code, program header and layout checks, plus the graphics library (robotx_gfx.h); linked into every
# program (unused parts are dropped by the linker).
add_library(ftrobotx_runtime OBJECT
    sdk/runtime/dispatcher.c
    sdk/runtime/entry.S
    sdk/runtime/string.c
    sdk/gfx/gfx.c
    sdk/gfx/font5x7.c
)
target_include_directories(ftrobotx_runtime PUBLIC sdk/include)
target_compile_options(ftrobotx_runtime PUBLIC
    -mcpu=arm926ej-s            # AT91SAM9260
    -marm                       # the firmware calls the entry point in ARM state
    -mfloat-abi=soft
    -fno-short-enums            # enums are 4 bytes, as for the firmware
    -ffreestanding
    -fno-common
    -ffunction-sections
    -fdata-sections
    -fno-unwind-tables
    -fno-asynchronous-unwind-tables
    $<$<COMPILE_LANGUAGE:C>:-std=gnu17 -Wall -Wextra>
)
set_source_files_properties(sdk/runtime/string.c PROPERTIES COMPILE_OPTIONS -fno-tree-loop-distribute-patterns)
target_link_options(ftrobotx_runtime PUBLIC
    -mcpu=arm926ej-s
    -marm
    -mfloat-abi=soft
    -nostdlib
    -T${CMAKE_CURRENT_SOURCE_DIR}/sdk/runtime/robotx.ld
    -Wl,--gc-sections
    -Wl,--no-enum-size-warning
    -Wl,--no-warn-rwx-segments
)
target_link_libraries(ftrobotx_runtime PUBLIC gcc) # division and other helpers

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE MinSizeRel CACHE STRING "Build type" FORCE)
endif()

# ftrobotx_add_program(<name> <sources>...)
# Builds <name>.bin, the file that goes onto the controller, and adds the target <name>-deploy, which uploads it to
# the RAM disk, loads and starts it (port: FTROBOTX_PORT).
function(ftrobotx_add_program name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE ftrobotx_runtime)
    set_target_properties(${name} PROPERTIES SUFFIX .elf)
    target_link_options(${name} PRIVATE -Wl,-Map=$<TARGET_FILE_DIR:${name}>/${name}.map)
    add_custom_command(TARGET ${name} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} --output-target=binary $<TARGET_FILE:${name}> $<TARGET_FILE_DIR:${name}>/${name}.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${name}>
        COMMENT "Creating ${name}.bin"
        VERBATIM)

    set(port_args)
    if(FTROBOTX_PORT)
        set(port_args --port ${FTROBOTX_PORT})
    endif()
    add_custom_target(${name}-deploy
        COMMAND ${Python3_EXECUTABLE} ${FTROBOTX_LOAD} ${port_args} deploy $<TARGET_FILE_DIR:${name}>/${name}.bin
        DEPENDS ${name}
        USES_TERMINAL
        COMMENT "Uploading ${name}.bin to the controller and starting it")
endfunction()

ftrobotx_add_program(ButtonLamp examples/button_lamp.c)
ftrobotx_add_program(ExtensionTest examples/extension_test.c)
ftrobotx_add_program(DisplayDemo examples/display_demo.c)
ftrobotx_add_program(IoMonitor examples/io_monitor.c)
