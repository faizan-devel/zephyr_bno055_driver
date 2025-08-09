#ifndef BNO055_DRIVER_HPP
#define BNO055_DRIVER_HPP

#include <zephyr/device.h>

enum class BNOState {
    READ_DEVICE_ID,
    SET_CONFIG_MODE,
    SET_PAGE_ID,
    SET_EXTERNAL_CRYSTAL,
    SET_OPMODE,
    READ_EULER,
    READ_QUATERNION,
    READ_LINEAR_ACEELERATION,
    READ_GRAVITY,
    READ_TEMPERATURE,
    READ_CALIB
};

class BNO055Driver {
public:
    BNO055Driver(const struct device *i2c_dev);

    void start();
    void loop();

    float getYaw() const;
    float getPitch() const;
    float getRoll() const;

    float getQuatW() const;
    float getQuatX() const;
    float getQuatY() const;
    float getQuatZ() const;

    float getGravityX() const;
    float getGravityY() const;
    float getGravityZ() const;

    float getLinAccelX() const;
    float getLinAccelY() const;
    float getLinAccelZ() const;
    
    float getTemp() const;

    bool isCalibrated() const;

private:
    const struct device *i2c;
    int sleep_ms;
    bool is_calibrated;
    float temp;
    float yaw, pitch, roll;
    float quat_w, quat_x, quat_y, quat_z;
    float lin_accel_x,lin_accel_y,lin_accel_z;
    float gravity_x,gravity_y,gravity_z;

    BNOState state;

    void read_device_id();
    void set_configmode();
    void set_pageid0();
    void set_externalcrystal();
    void set_opmode();
    void read_eulerreg();
    void read_quaternionreg();
    void read_linear_accelerationreg();
    void read_gravityreg();
    void read_tempreg();
    void read_calibrationreg();
};

#endif // BNO055_DRIVER_HPP
