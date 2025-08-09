# ~/zephyrproject/modules/bno055_driver/CMakeLists.txt
cmake_minimum_required(VERSION 3.20.0)

# Build a named Zephyr library from this module
zephyr_library_named(bno055_driver)

# Export this module's include/ to any application that uses it
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add the driver sources
zephyr_library_sources(src/bno055_driver.cpp)

