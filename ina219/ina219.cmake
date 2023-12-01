# Add library cpp files
add_library(ina219 INTERFACE)
target_sources(ina219 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ina219.cc
)

# Add include directory
target_include_directories(ina219 INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Add the standard library to the build
target_link_libraries(ina219 INTERFACE pico_stdlib hardware_i2c)