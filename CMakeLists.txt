zephyr_library_named(bno055_driver)

zephyr_library_sources(
    src/bno055_driver.cpp
)

zephyr_library_include_directories(
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
)

