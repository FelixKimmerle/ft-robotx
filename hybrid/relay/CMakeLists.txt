cmake_minimum_required(VERSION 3.28)

project(FtRoboTxRelay
    DESCRIPTION "ft-robotx hybrid mode: Bluetooth messages between ROBO TX Controllers, relayed by a TX in online mode"
    LANGUAGES CXX)

option(FT_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# The online library; as a subproject it builds without its examples, tests and GUI.
add_subdirectory(../../online/cpp ftrobotx)

add_executable(ftrobotx_bt_ping examples/bt_ping.cpp)
target_include_directories(ftrobotx_bt_ping PRIVATE ../../online/cpp/examples) # Cli.hpp
target_link_libraries(ftrobotx_bt_ping PRIVATE ftrobotx::ftrobotx)
ftrobotx_set_warnings(ftrobotx_bt_ping)
