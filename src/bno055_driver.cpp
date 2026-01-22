#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <math.h>
#include "bno055_driver.hpp"

namespace {

constexpr uint8_t BNO055_ADDRESS_A                         = 0x28;
constexpr uint8_t OPERATION_MODE_CONFIG                    = 0x00;
constexpr uint8_t OPERATION_MODE_NDOF                      = 0x0C;

constexpr uint8_t BNO055_CHIP_ID_ADDR                      = 0x00;
constexpr uint8_t BNO055_PAGE_ID_ADDR                      = 0x07;
constexpr uint8_t BNO055_SYS_TRIGGER_ADDR                  = 0x3F;
constexpr uint8_t BNO055_OPR_MODE_ADDR                     = 0x3D;

constexpr uint8_t BNO055_EULER_H_LSB_ADDR                  = 0x1A;
constexpr uint8_t BNO055_QUATERNION_H_LSB_ADDR             = 0x20;
constexpr uint8_t BNO055_LINEAR_ACCELERATION_H_LSB_ADDR    = 0x28;
constexpr uint8_t BNO055_GRAVITY_H_LSB_ADDR                = 0x2E;

constexpr uint8_t BNO055_TEMP_ADDR                         = 0x34;
constexpr uint8_t BNO055_CALIB_STAT_ADDR                   = 0x35;

constexpr int READ_SENSOR_INTERVAL                         = 50;

} // namespace

// ---------------------------------------------------------
// Constructor
// ---------------------------------------------------------
BNO055Driver::BNO055Driver(const struct device *i2c_dev)
    : i2c(i2c_dev),

      sleep_ms(0),
      is_calibrated(false),
      timestamp_ms(0),

      temp(0),

      yaw(0), pitch(0), roll(0),
      quat_w(0), quat_x(0), quat_y(0), quat_z(0),
      lin_accel_x(0), lin_accel_y(0), lin_accel_z(0),
      gravity_x(0), gravity_y(0), gravity_z(0),

      state(BNOState::READ_DEVICE_ID)
{}


// ---------------------------------------------------------
// Start IMU
// ---------------------------------------------------------
void BNO055Driver::start() {
    if (!device_is_ready(i2c)) {
        printk("I2C device not ready!\n");
        return;
    }

    k_msleep(700); // Datasheet startup delay

    uint8_t reg = BNO055_CHIP_ID_ADDR;
    uint8_t chip_id = 0;

    if (i2c_write_read(i2c, BNO055_ADDRESS_A, &reg, 1, &chip_id, 1) < 0) {
        printk("Failed to read chip ID at startup\n");
        return;
    }

    if (chip_id != 0xA0) {
        printk("Unexpected chip ID: 0x%02X\n", chip_id);
        return;
    }

    printk("BNO055 startup OK | Chip ID: 0x%02X\n", chip_id);
    state = BNOState::READ_DEVICE_ID;
}


// ---------------------------------------------------------
// FSM Loop
// ---------------------------------------------------------
void BNO055Driver::loop() {
    bool ok = false;

    switch (state) {
        case BNOState::READ_DEVICE_ID:          ok = read_device_id(); break;
        case BNOState::SET_CONFIG_MODE:         ok = set_configmode(); break;
        case BNOState::SET_PAGE_ID:             ok = set_pageid0(); break;
        case BNOState::SET_EXTERNAL_CRYSTAL:    ok = set_externalcrystal(); break;
        case BNOState::SET_OPMODE:              ok = set_opmode(); break;

        case BNOState::READ_EULER:              ok = read_eulerreg(); break;
        case BNOState::READ_QUATERNION:         ok = read_quaternionreg(); break;
        case BNOState::READ_LINEAR_ACCELERATION:ok = read_linear_accelerationreg(); break;
        case BNOState::READ_GRAVITY:            ok = read_gravityreg(); break;
        case BNOState::READ_TEMPERATURE:        ok = read_tempreg(); break;
        case BNOState::READ_CALIB:              ok = read_calibrationreg(); break;
    }

    // Only update timestamp if successful
    if (ok) {
        timestamp_ms = k_uptime_get_32();
    }

    k_msleep(sleep_ms);
}


// ---------------------------------------------------------
// FSM Helper Functions
// ---------------------------------------------------------
bool BNO055Driver::read_device_id() {
    uint8_t reg = BNO055_CHIP_ID_ADDR;
    uint8_t chip_id = 0;

    if (i2c_write_read(i2c, BNO055_ADDRESS_A, &reg, 1, &chip_id, 1) < 0) {
        printk("Failed: read_device_id\n");
        return false;
    }

    printk("Chip ID: 0x%02X\n", chip_id);
    sleep_ms = 1000;
    state = BNOState::SET_CONFIG_MODE;
    return true;
}

bool BNO055Driver::set_configmode() {
    uint8_t cmd[2] = { BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG };

    if (i2c_write(i2c, cmd, 2, BNO055_ADDRESS_A) < 0) return false;
    
    sleep_ms = 25;
    state = BNOState::SET_PAGE_ID;
    return true;
}

bool BNO055Driver::set_pageid0() {
    uint8_t cmd[2] = { BNO055_PAGE_ID_ADDR, 0x00 };

    if (i2c_write(i2c, cmd, 2, BNO055_ADDRESS_A) < 0) return false;

    sleep_ms = 10;
    state = BNOState::SET_EXTERNAL_CRYSTAL;
    return true;
}

bool BNO055Driver::set_externalcrystal() {
    uint8_t cmd[2] = { BNO055_SYS_TRIGGER_ADDR, 0x80 };

    if (i2c_write(i2c, cmd, 2, BNO055_ADDRESS_A) < 0) return false;

    sleep_ms = 10;
    state = BNOState::SET_OPMODE;
    return true;
}

bool BNO055Driver::set_opmode() {
    uint8_t cmd[2] = { BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF };

    if (i2c_write(i2c, cmd, 2, BNO055_ADDRESS_A) < 0) return false;

    sleep_ms = 600;
    state = BNOState::READ_EULER;
    return true;
}


// ---------------------------------------------------------
// Sensor Reading Functions
// ---------------------------------------------------------
bool BNO055Driver::read_eulerreg() {
    uint8_t reg = BNO055_EULER_H_LSB_ADDR;
    uint8_t data[6];

    if (i2c_write_read(i2c, BNO055_ADDRESS_A, &reg, 1, data, 6) < 0) return false;

    yaw   = ((int16_t)((data[1] << 8) | data[0])) / 16.0f;
    roll  = ((int16_t)((data[3] << 8) | data[2])) / 16.0f;
    pitch = ((int16_t)((data[5] << 8) | data[4])) / 16.0f;

    sleep_ms = READ_SENSOR_INTERVAL;
    state = BNOState::READ_QUATERNION;
    return true;
}

bool BNO055Driver::read_quaternionreg() {
    uint8_t reg = BNO055_QUATERNION_H_LSB_ADDR;
    uint8_t data[8];

    if (i2c_write_read(i2c, BNO055_ADDRESS_A, &reg, 1, data, 8) < 0) return false;

    constexpr float scale = 1.0f / 16384.0f;

    quat_w = ((int16_t)((data[1] << 8) | data[0])) * scale;
    quat_x = ((int16_t)((data[3] << 8) | data[2])) * scale;
    quat_y = ((int16_t)((data[5] << 8) | data[4])) * scale;
    quat_z = ((int16_t)((data[7] << 8) | data[6])) * scale;

    sleep_ms = READ_SENSOR_INTERVAL;
    state = BNOState::READ_LINEAR_ACCELERATION;
    return true;
}

bool BNO055Driver::read_linear_accelerationreg() {
    uint8_t reg = BNO055_LINEAR_ACCELERATION_H_LSB_ADDR;
    uint8_t data[6];

    if (i2c_write_read(i2c, BNO055_ADDRESS_A, &reg, 1, data, 6) < 0) return false;

    lin_accel_x = ((int16_t)((data[1] << 8) | data[0])) / 100.0f;
    lin_accel_y = ((int16_t)((data[3] << 8) | data[2])) / 100.0f;
    lin_accel_z = ((int16_t)((data[5] << 8) | data[4])) / 100.0f;

    sleep_ms = READ_SENSOR_INTERVAL;
    state = BNOState::READ_GRAVITY;
    return true;
}

bool BNO055Driver::read_gravityreg() {
    uint8_t reg = BNO055_GRAVITY_H_LSB_ADDR;
    uint8_t data[6];

    if (i2c_write_read(i2c, BNO055_ADDRESS_A, &reg, 1, data, 6) < 0) return false;

    gravity_x = ((int16_t)((data[1] << 8) | data[0])) / 100.0f;
    gravity_y = ((int16_t)((data[3] << 8) | data[2])) / 100.0f;
    gravity_z = ((int16_t)((data[5] << 8) | data[4])) / 100.0f;

    sleep_ms = READ_SENSOR_INTERVAL;
    state = BNOState::READ_TEMPERATURE;
    return true;
}

bool BNO055Driver::read_tempreg() {
    uint8_t reg = BNO055_TEMP_ADDR;
    int8_t temp_raw = 0;

    if (i2c_write_read(i2c, BNO055_ADDRESS_A, &reg, 1, (uint8_t*)&temp_raw, 1) < 0) return false;

    temp = static_cast<float>(temp_raw);

    state = BNOState::READ_CALIB;
    return true;
}

bool BNO055Driver::read_calibrationreg() {
    uint8_t reg = BNO055_CALIB_STAT_ADDR;
    uint8_t calib = 0;

    if (i2c_write_read(i2c, BNO055_ADDRESS_A, &reg, 1, &calib, 1) < 0) return false;

    is_calibrated = (calib == 0xFF);

    state = BNOState::READ_EULER;
    return true;
}


// ---------------------------------------------------------
// Getters
// ---------------------------------------------------------
float BNO055Driver::getYaw() const { return yaw; }
float BNO055Driver::getPitch() const { return pitch; }
float BNO055Driver::getRoll() const { return roll; }

float BNO055Driver::getQuatX() const { return quat_x; }
float BNO055Driver::getQuatY() const { return quat_y; }
float BNO055Driver::getQuatZ() const { return quat_z; }
float BNO055Driver::getQuatW() const { return quat_w; }

float BNO055Driver::getLinAccelX() const { return lin_accel_x; }
float BNO055Driver::getLinAccelY() const { return lin_accel_y; }
float BNO055Driver::getLinAccelZ() const { return lin_accel_z; }

float BNO055Driver::getGravityX() const { return gravity_x; }
float BNO055Driver::getGravityY() const { return gravity_y; }
float BNO055Driver::getGravityZ() const { return gravity_z; }

float BNO055Driver::getTemp() const { return temp; }

bool BNO055Driver::isCalibrated() const { return is_calibrated; }
