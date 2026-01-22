# BNO055 IMU Driver for Zephyr RTOS

A **minimal, non-blocking BNO055 IMU driver** for **Zephyr RTOS**, designed for real-time and robotics applications.

This driver uses a **finite state machine (FSM)** over I²C to initialize and read sensor data without blocking the system, making it suitable for control loops, estimation pipelines, and embedded autonomy stacks.

Tested with:
- Zephyr RTOS v4.3+
- ESP32 (`esp_wrover_kit`)

---

## ✨ Features

- Non-blocking FSM-based driver design  
- Quaternion-first orientation handling  
- Euler angles (Yaw, Pitch, Roll)  
- Linear acceleration and gravity vectors  
- Temperature reading  
- Clean C++ API  
- RTOS-friendly periodic polling model  

---

## 📁 Repository Structure

```
bno055_driver/
├── include/
│   └── bno055_driver.hpp
├── src/
│   └── bno055_driver.cpp
├── samples/
│   └── basic/
│       ├── src/main.cpp
│       ├── prj.conf
│       ├── CMakeLists.txt
│       └── esp32.overlay
├── zephyr/
│   └── module.yml
├── CMakeLists.txt
└── README.md
```

---

## 🚀 Getting Started

### 1️⃣ Prerequisites

- Zephyr SDK installed
- Zephyr workspace initialized using `west`
- Target board with I²C support

---

### 2️⃣ Add Driver to Zephyr Workspace

Clone or copy this repository into your Zephyr workspace:

```
zephyrproject/
├── zephyr/
├── modules/
│   └── bno055_driver/
```

The included `zephyr/module.yml` enables **automatic module discovery** by Zephyr — no manual `west.yaml` changes are required.

---

### 3️⃣ Build the Sample Application

```bash
cd zephyrproject/modules/bno055_driver/samples/basic
west build -b esp_wrover_kit/esp32/procpu --pristine
```

Flash to the board:

```bash
west flash
```

Monitor output:

```bash
west espressif monitor 
```

---

## 🧩 Using the Driver in Your Application

### Include the Driver

```cpp
#include "bno055_driver.hpp"
```

### Initialize

```cpp
const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
BNO055Driver bno(i2c_dev);

bno.start();
```

### Periodic Update Loop

```cpp
while (true) {
    bno.loop();

    float yaw   = bno.getYaw();
    float pitch = bno.getPitch();
    float roll  = bno.getRoll();

    k_msleep(50);
}
```

> `loop()` advances the internal FSM and performs **one non-blocking operation per call**.

---

## 📊 Available API

### Orientation
- `getYaw()`
- `getPitch()`
- `getRoll()`

### Quaternion
- `getQuatW()`
- `getQuatX()`
- `getQuatY()`
- `getQuatZ()`

### Motion
- `getLinAccelX()`, `getLinAccelY()`, `getLinAccelZ()`
- `getGravityX()`, `getGravityY()`, `getGravityZ()`

### Misc
- `getTemp()`
- `isCalibrated()`
- `getTimestamp()`

---

