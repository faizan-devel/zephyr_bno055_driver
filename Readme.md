📄 BNO055 Orientation Conversion Documentation
🎯 Objective
Understand how to convert raw I²C data from the BNO055 sensor into human-readable orientation values (yaw, pitch, roll) using quaternion math.

1️⃣ Sensor Output: Quaternion Data
🔧 Data Source
Register: 0x20 to 0x27

Length: 8 bytes

Format: 4 × 16-bit signed integers

W: LSB @ 0x20, MSB @ 0x21

X: LSB @ 0x22, MSB @ 0x23

Y: LSB @ 0x24, MSB @ 0x25

Z: LSB @ 0x26, MSB @ 0x27

2️⃣ Step-by-Step Conversion Pipeline
➤ Step 1: Read 8 Bytes over I²C
c
Copy
Edit
i2c_write_read(i2c_dev, BNO055_ADDRESS_A, write_buf, 1, read_buf, 8);
➤ Step 2: Convert Bytes to Signed Integers
c
Copy
Edit
int16_t quat_w = (read_buf[1] << 8) | read_buf[0];
int16_t quat_x = (read_buf[3] << 8) | read_buf[2];
int16_t quat_y = (read_buf[5] << 8) | read_buf[4];
int16_t quat_z = (read_buf[7] << 8) | read_buf[6];
📝 Endian Format: Little-endian (LSB first)

📦 Data Type: int16_t

3️⃣ Step 3: Fixed-Point to Float
📌 Format: 1Q14
1 sign bit + 1 integer bit + 14 fractional bits

Scale Factor: 1.0 / 16384.0

🔢 Conversion:
c
Copy
Edit
float qw = quat_w * (1.0f / 16384.0f);
float qx = quat_x * (1.0f / 16384.0f);
float qy = quat_y * (1.0f / 16384.0f);
float qz = quat_z * (1.0f / 16384.0f);
4️⃣ Step 4: Convert Quaternion → Euler Angles
📐 Definitions
Term	Axis	Description
Roll	X	Left/Right tilt
Pitch	Y	Forward/Backward tilt
Yaw	Z	Heading/Compass direction

🧮 Math Formula:
c
Copy
Edit
float t0 = 2.0f * (qw * qx + qy * qz);
float t1 = 1.0f - 2.0f * (qx * qx + qy * qy);
float roll = atan2f(t0, t1);

float t2 = 2.0f * (qw * qy - qz * qx);
t2 = clamp(t2, -1.0f, 1.0f);
float pitch = asinf(t2);

float t3 = 2.0f * (qw * qz + qx * qy);
float t4 = 1.0f - 2.0f * (qy * qy + qz * qz);
float yaw = atan2f(t3, t4);
5️⃣ Step 5: Radians → Degrees
c
Copy
Edit
constexpr float RAD_TO_DEG = 180.0f / M_PI;
roll  *= RAD_TO_DEG;
pitch *= RAD_TO_DEG;
yaw   *= RAD_TO_DEG;
🧪 Example Debug Print
c
Copy
Edit
printk("Raw Quaternion: W=%d, X=%d, Y=%d, Z=%d\n", quat_w, quat_x, quat_y, quat_z);
printk("Quaternion (float): W=%.4f, X=%.4f, Y=%.4f, Z=%.4f\n", qw, qx, qy, qz);
printk("Euler Angles: Yaw=%.2f°, Pitch=%.2f°, Roll=%.2f°\n", yaw, pitch, roll);
📊 Summary Table
Step	Description	Input	Output
1	Read 8 bytes from IMU	I²C	Raw bytes
2	Convert to 16-bit signed ints	LSB, MSB pairs	int16_t values
3	Fixed-point → float	1Q14 format	float [-1, +1]
4	Quaternion → Euler conversion	float quaternions	yaw, pitch, roll
5	Convert radians to degrees	float (radians)	float (degrees)
