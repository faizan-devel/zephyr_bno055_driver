cmake_minimum_required(VERSION 3.20.0)

set(BOARD esp_wrover_kit/esp32/procpu)
set(DTC_OVERLAY_FILE "esp32.overlay")


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bno055_basic)

target_sources(app PRIVATE src/main.cpp)

# Link against module's library
#target_link_libraries(app PRIVATE bno055_driver)
