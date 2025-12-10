/**
 * MPU6050 IMU Driver
 * 6-axis accelerometer and gyroscope
 * I2C Interface
 * Compatible with any I2C port configuration
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "hardware/i2c.h"
#include "pico/stdlib.h"

// MPU6050 I2C address (AD0 pin = LOW)
#define MPU6050_ADDR 0x68

// MPU6050 Register addresses
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_WHO_AM_I     0x75

// Scale factors for converting raw values
// Accelerometer: ±2g range, 16384 LSB/g
#define ACCEL_SCALE 16384.0f
// Gyroscope: ±250°/s range, 131 LSB/(°/s)
#define GYRO_SCALE 131.0f

// Fixed-point format (15.16) - compatible with Hunter Adams' format
typedef signed int fix15;

// Fixed point macros (matching Hunter Adams' format from helicopter project)
#define float2fix15(a) ((fix15)((a)*65536.0f))
#define fix2float15(a) ((float)(a)/65536.0f)
#define int2fix15(a) ((fix15)((a)<<16))
#define fix2int15(a) ((int)((a)>>16))
#define multfix15(a,b) ((fix15)(((( signed long long)(a))*(( signed long long)(b)))>>16))
#define divfix(a,b) ((fix15)(((( signed long long)(a) << 16 / (b)))))

/**
 * Initialize the MPU6050
 * Wakes up the device from sleep mode
 */
void mpu6050_reset(void);

/**
 * Read raw accelerometer and gyroscope data
 * @param accel Array to store acceleration values [x, y, z] in g's (15.16 fixed point)
 * @param gyro Array to store gyroscope values [x, y, z] in °/s (15.16 fixed point)
 */
void mpu6050_read_raw(fix15 accel[3], fix15 gyro[3]);

/**
 * Read raw accelerometer data only
 * @param accel Array to store acceleration values [x, y, z] in g's (15.16 fixed point)
 */
void mpu6050_read_accel(fix15 accel[3]);

/**
 * Read raw gyroscope data only
 * @param gyro Array to store gyroscope values [x, y, z] in °/s (15.16 fixed point)
 */
void mpu6050_read_gyro(fix15 gyro[3]);

/**
 * Check if MPU6050 is connected and responding
 * @return true if device responds correctly, false otherwise
 */
bool mpu6050_is_connected(void);

#endif // MPU6050_H
