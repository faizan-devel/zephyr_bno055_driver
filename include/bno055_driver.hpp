#ifndef BNO055_DRIVER_HPP
#define BNO055_DRIVER_HPP

#include <zephyr/device.h>
#include <stdint.h>

/**
 * @brief Driver state machine for BNO055 setup and sensor readout.
 */
enum class BNOState {
    READ_DEVICE_ID,
    SET_CONFIG_MODE,
    SET_PAGE_ID,
    SET_EXTERNAL_CRYSTAL,
    SET_OPMODE,
    READ_EULER,
    READ_QUATERNION,
    READ_LINEAR_ACCELERATION,
    READ_GRAVITY,
    READ_TEMPERATURE,
    READ_CALIB
};

/**
 * @brief Minimal BNO055 IMU driver for Zephyr (I2C).
 *
 * Provides:
 *  - Euler angles (yaw, pitch, roll)
 *  - Quaternion (w,x,y,z)
 *  - Linear acceleration
 *  - Gravity vector
 *  - Temperature
 *
 * Uses a non-blocking FSM-driven update loop via loop().
 */
class BNO055Driver {
public:
    /**
     * @brief Construct a new BNO055 driver.
     *
     * @param i2c_dev Pointer to a Zephyr I2C device.
     */
    BNO055Driver(const struct device *i2c_dev);

    /**
     * @brief Initialize IMU, verify chip ID, and prepare FSM.
     */
    void start();

    /**
     * @brief Run one step of the internal FSM. Call periodically.
     */
    void loop();

    /// Orientation getters
    float getYaw() const;
    float getPitch() const;
    float getRoll() const;

    /// Quaternion getters
    float getQuatW() const;
    float getQuatX() const;
    float getQuatY() const;
    float getQuatZ() const;

    /// Gravity getters
    float getGravityX() const;
    float getGravityY() const;
    float getGravityZ() const;

    /// Linear acceleration getters
    float getLinAccelX() const;
    float getLinAccelY() const;
    float getLinAccelZ() const;

    /// Temperature getter
    float getTemp() const;

    /// Calibration status
    bool isCalibrated() const;

    /// Timestamp of last successful sensor update (ms)
    uint32_t getTimestamp() const { return timestamp_ms; }

private:
    const struct device *i2c;

    int sleep_ms;
    bool is_calibrated;
    uint32_t timestamp_ms;

    float temp;

    float yaw, pitch, roll;
    float quat_w, quat_x, quat_y, quat_z;
    float lin_accel_x, lin_accel_y, lin_accel_z;
    float gravity_x, gravity_y, gravity_z;

    BNOState state;

    // -------- FSM Internal Methods (all return bool) ---------

    /** @brief Read chip ID and validate device presence */
    bool read_device_id();

    /** @brief Switch IMU into CONFIG mode */
    bool set_configmode();

    /** @brief Select register page 0 */
    bool set_pageid0();

    /** @brief Enable external crystal oscillator */
    bool set_externalcrystal();

    /** @brief Switch IMU to the desired operation mode */
    bool set_opmode();

    /** @brief Read Euler angles */
    bool read_eulerreg();

    /** @brief Read quaternion values */
    bool read_quaternionreg();

    /** @brief Read linear acceleration vector */
    bool read_linear_accelerationreg();

    /** @brief Read gravity vector */
    bool read_gravityreg();

    /** @brief Read temperature register */
    bool read_tempreg();

    /** @brief Read calibration status */
    bool read_calibrationreg();
};

#endif // BNO055_DRIVER_HPP
