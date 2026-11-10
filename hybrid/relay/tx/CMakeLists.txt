cmake_minimum_required(VERSION 3.28)

# Programs for the model TX (the controller that runs on its own and talks to the relay over Bluetooth). They run on
# the controller's ARM CPU and use the offline SDK.
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../../offline/cmake/arm-none-eabi.cmake)
endif()

project(FtRoboTxRelayTx
    DESCRIPTION "ft-robotx hybrid mode: programs for the model TX behind a relay TX"
    LANGUAGES C ASM)

add_subdirectory(../../../offline ftrobotx_offline)

# Bluetooth address of the relay TX; the model TX only accepts a connection from it.
set(FT_RELAY_ADDRESS "00:13:7B:53:DB:5E" CACHE STRING "Bluetooth address of the relay TX")
string(REPLACE ":" ",0x" relay_bytes "0x${FT_RELAY_ADDRESS}")

ftrobotx_add_program(BtEcho bt_echo.c)
target_compile_definitions(BtEcho PRIVATE RELAY_ADDRESS=${relay_bytes})

# The model TX for the relay protocol (../protocol/relay_protocol.h).
ftrobotx_add_program(RelayModel relay_model.c)
target_compile_definitions(RelayModel PRIVATE RELAY_ADDRESS=${relay_bytes})
target_include_directories(RelayModel PRIVATE ../protocol)
