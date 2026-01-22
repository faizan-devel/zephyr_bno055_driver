cmake_minimum_required(VERSION 3.20.0)
zephyr_library_named(bno055_driver)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
zephyr_library_sources(src/bno055_driver.cpp)









