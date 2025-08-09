#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <math.h>
#include "bno055_driver.hpp"

#define BNO055_ADDRESS_A             0x28
#define OPERATION_MODE_CONFIG       0x00
#define OPERATION_MODE_NDOF         0x0C
#define BNO055_CHIP_ID_ADDR         0x00
#define BNO055_PAGE_ID_ADDR         0x07
#define BNO055_SYS_TRIGGER_ADDR     0x3F
#define BNO055_OPR_MODE_ADDR        0x3D
#define BNO055_EULER_H_LSB_ADDR     0x1A
#define BNO055_QUATERNION_H_LSB_ADDR     0x20
#define BNO055_LINEAR_ACCELERATION_H_LSB_ADDR     0x28
#define BNO055_GRAVITY_H_LSB_ADDR     0x2E
#define BNO055_TEMP_ADDR 0x34
#define BNO055_CALIB_STAT_ADDR      0x35
#define READ_SENSOR_INTERVAL        50

BNO055Driver::BNO055Driver(const struct device *i2c_dev)
    : i2c(i2c_dev), sleep_ms(0), is_calibrated(false), yaw(0), pitch(0), roll(0), state(BNOState::READ_DEVICE_ID) {}

void BNO055Driver::start() {
    if (!device_is_ready(i2c)) {
    printk("I2C device not ready!\n");
    return;
    }

    // Wait for sensor startup (required ~650ms per datasheet)
    k_msleep(700);

    // Try reading chip ID
    uint8_t reg = BNO055_CHIP_ID_ADDR;
    uint8_t chip_id;
    int err = i2c_write_read(i2c, BNO055_ADDRESS_A, &reg, 1, &chip_id, 1);
    if (err < 0) {
        printk("Failed to read chip ID during startup check (err=%d)\n", err);
        return;
    }

    if (chip_id != 0xA0) {
        printk("Unexpected chip ID: 0x%02X (expected 0xA0)\n", chip_id);
        return;
    }

    printk("BNO055 startup check passed. Chip ID: 0x%02X\n", chip_id);

    // Reset internal state machine
    state = BNOState::READ_DEVICE_ID;
}

void BNO055Driver::loop() {
    switch (state) {
        case BNOState::READ_DEVICE_ID:
            read_device_id();
            break;
        case BNOState::SET_CONFIG_MODE:
            set_configmode();
            break;
        case BNOState::SET_PAGE_ID:
            set_pageid0();
            break;
        case BNOState::SET_EXTERNAL_CRYSTAL:
            set_externalcrystal();
            break;
        case BNOState::SET_OPMODE:
            set_opmode();
            break;
        case BNOState::READ_EULER:
            read_eulerreg();
            break;
        case BNOState::READ_QUATERNION:
            read_quaternionreg();
            break;
        case BNOState::READ_LINEAR_ACEELERATION:
            read_linear_accelerationreg();
            break;
        case BNOState::READ_GRAVITY:
            read_gravityreg();
            break;
        case BNOState::READ_TEMPERATURE:
            read_tempreg();
            break;
        case BNOState::READ_CALIB:
            read_calibrationreg();
            break;
    }
    k_msleep(sleep_ms);
}

void BNO055Driver::read_device_id() {
    uint8_t reg = BNO055_CHIP_ID_ADDR;
    uint8_t chip_id;
    int err = i2c_write_read(i2c, BNO055_ADDRESS_A, &reg, 1, &chip_id, 1);
    if (err < 0) {
        printk("Failed to read chip ID\n");
        return;
    }
    printk("Chip ID: 0x%02X\n", chip_id);
    sleep_ms = 1000;
    state = BNOState::SET_CONFIG_MODE;
}

void BNO055Driver::set_configmode() {
    uint8_t cmd[2] = { BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG };
    i2c_write(i2c, cmd, 2, BNO055_ADDRESS_A);
    sleep_ms = 25;
    state = BNOState::SET_PAGE_ID;
}

void BNO055Driver::set_pageid0() {
    uint8_t cmd[2] = { BNO055_PAGE_ID_ADDR, 0x00 };
    i2c_write(i2c, cmd, 2, BNO055_ADDRESS_A);
    sleep_ms = 10;
    state = BNOState::SET_EXTERNAL_CRYSTAL;
}

void BNO055Driver::set_externalcrystal() {
    uint8_t cmd[2] = { BNO055_SYS_TRIGGER_ADDR, 0x80 };
    i2c_write(i2c, cmd, 2, BNO055_ADDRESS_A);
    sleep_ms = 10;
    state = BNOState::SET_OPMODE;
}

void BNO055Driver::set_opmode() {
    uint8_t cmd[2] = { BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF };
    i2c_write(i2c, cmd, 2, BNO055_ADDRESS_A);
    sleep_ms = 600;
    state = BNOState::READ_EULER;
}

void BNO055Driver::read_eulerreg() {
    uint8_t reg = BNO055_EULER_H_LSB_ADDR;
    uint8_t data[6];
    int err = i2c_write_read(i2c, BNO055_ADDRESS_A, &reg, 1, data, 6);
    if (err < 0) {
        printk("read_eulerreg: I2C error\n");
        return;
    }

    int16_t raw_heading = (data[1] << 8) | data[0]; // Yaw
    int16_t raw_roll    = (data[3] << 8) | data[2];
    int16_t raw_pitch   = (data[5] << 8) | data[4];

    yaw   = raw_heading / 16.0f;
    pitch = raw_pitch / 16.0f;
    roll  = raw_roll / 16.0f;

    sleep_ms = READ_SENSOR_INTERVAL;
    state = BNOState::READ_QUATERNION;
}

void BNO055Driver::read_quaternionreg(){
    uint8_t reg = BNO055_QUATERNION_H_LSB_ADDR;
    uint8_t data[8];
    int err = i2c_write_read(i2c,BNO055_ADDRESS_A,&reg,1,data,8);
    if(err<0){
        printk("read_quaternion: I2C error\n");
        return;
    }

    int16_t qw = (data[1] << 8) | data[0];
    int16_t qx = (data[3] << 8) | data[2];
    int16_t qy = (data[5] << 8) | data[4];
    int16_t qz = (data[7] << 8) | data[6];

    constexpr float scale = 1.0f / 16384.0f;
    quat_w = qw * scale;
    quat_x = qx * scale;
    quat_y = qy * scale;
    quat_z = qz * scale;

    sleep_ms = READ_SENSOR_INTERVAL;
    state = BNOState::READ_LINEAR_ACEELERATION;
}


void BNO055Driver::read_linear_accelerationreg(){
    uint8_t reg = BNO055_LINEAR_ACCELERATION_H_LSB_ADDR;
    uint8_t data[6];
    int err = i2c_write_read(i2c,BNO055_ADDRESS_A,&reg,1,data,6);
    if(err<0){
        printk("read_linear_acceleration: I2C error\n");
        return;
    }

    int16_t x = (data[1] << 8) | data[0];
    int16_t y = (data[3] << 8) | data[2];
    int16_t z = (data[5] << 8) | data[4];

    lin_accel_x = x / 100.0f;  // scale = 1 LSB = 0.01 m/s²
    lin_accel_y = y / 100.0f;
    lin_accel_z = z / 100.0f;

    sleep_ms = READ_SENSOR_INTERVAL;
    state = BNOState::READ_GRAVITY;
}


void BNO055Driver::read_gravityreg(){
    uint8_t reg = BNO055_GRAVITY_H_LSB_ADDR;
    uint8_t data[6];
    int err = i2c_write_read(i2c,BNO055_ADDRESS_A,&reg,1,data,6);
    if(err<0){
        printk("read_gravity: I2C error\n");
        return;
    }

    int16_t x = (data[1] << 8) | data[0];
    int16_t y = (data[3] << 8) | data[2];
    int16_t z = (data[5] << 8) | data[4];

    gravity_x = x / 100.0f;
    gravity_y = y / 100.0f;
    gravity_z = z / 100.0f;


    sleep_ms = READ_SENSOR_INTERVAL;
    state = BNOState::READ_TEMPERATURE;
}


void BNO055Driver::read_tempreg() {
    uint8_t reg = BNO055_TEMP_ADDR;
    int8_t temp_raw = 0;

    int err = i2c_write_read(i2c, BNO055_ADDRESS_A, &reg, 1, (uint8_t*)&temp_raw, 1);
    if (err < 0) {
        printk("read_temperature: I2C error\n");
        return;
    }

    temp = static_cast<float>(temp_raw);
    state = BNOState::READ_CALIB;
}

void BNO055Driver::read_calibrationreg() {
    uint8_t reg = BNO055_CALIB_STAT_ADDR;
    uint8_t calib;
    int err = i2c_write_read(i2c, BNO055_ADDRESS_A, &reg, 1, &calib, 1);
    if (err < 0) {
        printk("read_calibrationreg: I2C error\n");
        return;
    }
    is_calibrated = ((calib & 0xFF) == 0xFF);
    state = BNOState::READ_EULER;
}

float BNO055Driver::getYaw() const { return yaw; }
float BNO055Driver::getPitch() const { return pitch; }
float BNO055Driver::getRoll() const { return roll; }

float BNO055Driver::getQuatX() const {return quat_x;}
float BNO055Driver::getQuatY() const {return quat_y;}
float BNO055Driver::getQuatZ() const {return quat_z;}
float BNO055Driver::getQuatW() const {return quat_w;}

float BNO055Driver::getLinAccelX() const { return lin_accel_x; }
float BNO055Driver::getLinAccelY() const { return lin_accel_y; }
float BNO055Driver::getLinAccelZ() const { return lin_accel_z; }

float BNO055Driver::getGravityX() const { return gravity_x; }
float BNO055Driver::getGravityY() const { return gravity_y; }
float BNO055Driver::getGravityZ() const { return gravity_z; }

float BNO055Driver::getTemp() const { return temp; }

bool BNO055Driver::isCalibrated() const { return is_calibrated; }
