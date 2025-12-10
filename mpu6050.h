#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

// I2C address for the MPU6050 (7-bit)
#define MPU6050_ADDR 0x68

// Key registers used by the driver
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_WHO_AM_I     0x75

// Scale factors for default ±2g and ±250°/s ranges
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f

// Fixed-point Q16.15 helpers
typedef int32_t fix15;
#define float2fix15(a) ((fix15)((a) * 32768.0f))
#define fix2float15(a) ((float)(a) / 32768.0f)
#define int2fix15(a)   ((fix15)((a) << 15))
#define fix2int15(a)   ((int)((a) >> 15))
#define multfix15(a,b) ((fix15)(((int64_t)(a) * (int64_t)(b)) >> 15))

void mpu6050_reset(void);
void mpu6050_read_raw(fix15 accel[3], fix15 gyro[3]);
void mpu6050_read_accel(fix15 accel[3]);
void mpu6050_read_gyro(fix15 gyro[3]);
bool mpu6050_is_connected(void);

#endif // MPU6050_H
