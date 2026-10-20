cmake_minimum_required(VERSION 3.28)

project(FtRoboTxOnline
    VERSION 1.0.0
    DESCRIPTION "ft-robotx-online: control fischertechnik ROBO TX Controllers in online mode"
    LANGUAGES CXX)

option(FT_BUILD_EXAMPLES "Build the demo program" ${PROJECT_IS_TOP_LEVEL})
option(FT_BUILD_TESTS "Build the protocol tests" ${PROJECT_IS_TOP_LEVEL})
option(FT_BUILD_GUI "Build the ROBO TX test tool (downloads SDL3 and Dear ImGui)" ${PROJECT_IS_TOP_LEVEL})
option(FT_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

function(ftrobotx_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- $<$<BOOL:${FT_WARNINGS_AS_ERRORS}>:/WX>)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow
            $<$<BOOL:${FT_WARNINGS_AS_ERRORS}>:-Werror>)
    endif()
endfunction()

add_library(ftrobotx
    src/FtConnection.cpp
    src/Packets.cpp
)

if(WIN32)
    target_sources(ftrobotx PRIVATE src/SerialPortWin32.cpp)
else()
    target_sources(ftrobotx PRIVATE src/SerialPortPosix.cpp)
endif()

add_library(ftrobotx::ftrobotx ALIAS ftrobotx)

target_include_directories(ftrobotx PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(ftrobotx PUBLIC cxx_std_20)
set_target_properties(ftrobotx PROPERTIES CXX_EXTENSIONS OFF)

ftrobotx_set_warnings(ftrobotx)

if(FT_BUILD_EXAMPLES)
    add_executable(ftrobotx_demo examples/main.cpp)
    target_link_libraries(ftrobotx_demo PRIVATE ftrobotx::ftrobotx)

    add_executable(ftrobotx_button_lamp examples/button_lamp.cpp)
    target_link_libraries(ftrobotx_button_lamp PRIVATE ftrobotx::ftrobotx)

    add_executable(ftrobotx_extension_test examples/extension_test.cpp)
    target_link_libraries(ftrobotx_extension_test PRIVATE ftrobotx::ftrobotx)
endif()

if(FT_BUILD_TESTS)
    enable_testing()
    add_executable(ftrobotx_protocol_test tests/protocol_test.cpp)
    target_link_libraries(ftrobotx_protocol_test PRIVATE ftrobotx::ftrobotx)
    add_test(NAME protocol COMMAND ftrobotx_protocol_test)
endif()

if(FT_BUILD_GUI)
    add_subdirectory(gui)
endif()
