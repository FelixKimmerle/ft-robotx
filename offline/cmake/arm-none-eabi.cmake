# Cross toolchain for ROBO TX Controller programs: GCC for bare-metal ARM (Arch: arm-none-eabi-gcc,
# Debian/Ubuntu: gcc-arm-none-eabi).
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy CACHE FILEPATH "objcopy for the ARM toolchain")
set(CMAKE_SIZE arm-none-eabi-size CACHE FILEPATH "size for the ARM toolchain")

# There's no C library to link test programs against.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
