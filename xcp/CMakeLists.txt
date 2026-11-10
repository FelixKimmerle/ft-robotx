cmake_minimum_required(VERSION 3.28)

project(FtRoboTxXcp
    DESCRIPTION "ft-robotx XCP bridge: ROBO TX Controllers in CANape via XCPlite, online mode or through a relay TX"
    LANGUAGES C CXX)

option(FT_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# The online library and the relay (RelayConnection); their examples are off as subprojects.
add_subdirectory(../online/cpp ftrobotx)
add_subdirectory(../hybrid/relay ftrobotx_relay)

# XCPlite (Vector, MIT): fetched from GitHub at a fixed version, or from a local checkout with
# -DFT_XCPLITE_DIR=<path>.
set(FT_XCPLITE_DIR "" CACHE PATH "Local checkout of https://github.com/vectorgrp/XCPlite (empty: fetch it)")
set(FT_XCPLITE_TAG "V2.3.2" CACHE STRING "XCPlite version to fetch")
set(XCPLITE_CONFIGURATION "default")
set(XCPLITE_BUILD_EXAMPLES OFF)
set(XCPLITE_BUILD_TESTS OFF)
set(XCPLITE_BUILD_TOOLS OFF)
if(FT_XCPLITE_DIR)
    add_subdirectory(${FT_XCPLITE_DIR} xcplite EXCLUDE_FROM_ALL SYSTEM) # no warnings from its headers
else()
    include(FetchContent)
    FetchContent_Declare(xcplite
        GIT_REPOSITORY https://github.com/vectorgrp/XCPlite
        GIT_TAG ${FT_XCPLITE_TAG}
        GIT_SHALLOW TRUE
        EXCLUDE_FROM_ALL
        SYSTEM) # no warnings from its headers
    FetchContent_MakeAvailable(xcplite)
endif()

add_executable(ftrobotx_xcp_bridge src/xcp_bridge.cpp)
target_include_directories(ftrobotx_xcp_bridge PRIVATE src ../online/cpp/examples) # Cli.hpp
target_link_libraries(ftrobotx_xcp_bridge PRIVATE ftrobotx::relay xcplite::xcplite)
target_compile_features(ftrobotx_xcp_bridge PRIVATE cxx_std_20)
ftrobotx_set_warnings(ftrobotx_xcp_bridge)
