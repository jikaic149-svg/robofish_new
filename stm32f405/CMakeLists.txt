cmake_minimum_required(VERSION 3.24)
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/arm-gcc.cmake"
        CACHE FILEPATH "ARM GCC toolchain")
endif()
project(RoboFishF405 LANGUAGES C CXX ASM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(CTest)
add_subdirectory(stm32f405)
