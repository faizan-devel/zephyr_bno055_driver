#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include "bno055_driver.hpp"

#define I2C_NODE DT_NODELABEL(i2c0)
const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

BNO055Driver bno(i2c_dev);

void main(void)
{
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready!\n");
        return;
    }

    printk("Starting BNO055 driver...\n");
    bno.start();

    while (true) {
        bno.loop();

        float yaw   = bno.getYaw();
        float pitch = bno.getPitch();
        float roll  = bno.getRoll();

        float lin_x = bno.getLinAccelX();
        float lin_y = bno.getLinAccelY();
        float lin_z = bno.getLinAccelZ();

        float gx = bno.getGravityX();
        float gy = bno.getGravityY();
        float gz = bno.getGravityZ();

        float qx = bno.getQuatX();
        float qy = bno.getQuatY();
        float qz = bno.getQuatZ();
        float qw = bno.getQuatW();

        float temp = bno.getTemp();

        printk("Euler   => Y: %.2f°, P: %.2f°, R: %.2f°\n", yaw, pitch, roll);
        printk("LinAcc  => X: %.2f, Y: %.2f, Z: %.2f (m/s²)\n", lin_x, lin_y, lin_z);
        printk("Gravity => X: %.2f, Y: %.2f, Z: %.2f (m/s²)\n", gx, gy, gz);
        printk("Quat    => W: %.4f, X: %.4f, Y: %.4f, Z: %.4f\n", qw, qx, qy, qz);
        printk("Temp    => %.2f °C\n\n", temp);

        k_msleep(50);
    }
}

