cmake_minimum_required(VERSION 3.20)

set(PICO_BOARD pico_w)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

include(pico_sdk_import.cmake)

project(pico_ws_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise the Pico SDK
pico_sdk_init()

# Add library cpp files
add_library(ina219 INTERFACE)
target_sources(ina219 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ina219.cc
)

# Add include directory
target_include_directories(ina219 INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Add the standard library to the build
target_link_libraries(ina219 INTERFACE pico_stdlib hardware_i2c)