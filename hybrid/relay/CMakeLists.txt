cmake_minimum_required(VERSION 3.28)

project(FtRoboTxRelay
    DESCRIPTION "ft-robotx hybrid mode: Bluetooth messages between ROBO TX Controllers, relayed by a TX in online mode"
    LANGUAGES CXX)

option(FT_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# The online library; as a subproject it builds without its examples, tests and GUI.
add_subdirectory(../../online/cpp ftrobotx)

# PC side of the relay protocol: RelayConnection talks to a model TX running tx/relay_model.c.
add_library(ftrobotx_relay src/RelayConnection.cpp)
add_library(ftrobotx::relay ALIAS ftrobotx_relay)
target_include_directories(ftrobotx_relay PUBLIC include protocol)
target_link_libraries(ftrobotx_relay PUBLIC ftrobotx::ftrobotx)
ftrobotx_set_warnings(ftrobotx_relay)

add_executable(ftrobotx_bt_ping examples/bt_ping.cpp)
target_include_directories(ftrobotx_bt_ping PRIVATE ../../online/cpp/examples) # Cli.hpp
target_link_libraries(ftrobotx_bt_ping PRIVATE ftrobotx::ftrobotx)
ftrobotx_set_warnings(ftrobotx_bt_ping)

add_executable(ftrobotx_relay_button_lamp examples/relay_button_lamp.cpp)
target_include_directories(ftrobotx_relay_button_lamp PRIVATE ../../online/cpp/examples) # Cli.hpp
target_link_libraries(ftrobotx_relay_button_lamp PRIVATE ftrobotx::relay)
ftrobotx_set_warnings(ftrobotx_relay_button_lamp)
