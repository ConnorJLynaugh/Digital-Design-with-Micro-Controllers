/**
 * MPU6050 IMU Driver Implementation
 * 6-axis accelerometer and gyroscope

 */


#include "mpu6050.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

// I2C instance - will use i2c1
// Make sure this matches your main.c I2C_PORT definition
#define MPU6050_I2C i2c1












/**
 * Write a single byte to an MPU6050 register
 */
static void mpu6050_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(MPU6050_I2C, MPU6050_ADDR, data, 2, false);
}

/**
 * Read multiple bytes from MPU6050 starting at a register
 */
static void mpu6050_read_registers(uint8_t reg, uint8_t *buffer, uint8_t len) {
    // Write register address
    i2c_write_blocking(MPU6050_I2C, MPU6050_ADDR, &reg, 1, true);
    // Read data
    i2c_read_blocking(MPU6050_I2C, MPU6050_ADDR, buffer, len, false);
}

/**
 * Initialize the MPU6050
 * Wakes up the device from sleep mode
 */
void mpu6050_reset(void) {
    // Wake up the MPU6050 (clear sleep bit)
    mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 0x00);
    sleep_ms(100); // Wait for device to stabilize
}

/**
 * Read raw accelerometer and gyroscope data
 */
void mpu6050_read_raw(fix15 accel[3], fix15 gyro[3]) {
    uint8_t buffer[14];
    
    // Read 14 bytes starting from ACCEL_XOUT_H
    // This reads all accelerometer (6 bytes) and gyroscope (6 bytes) data
    // Plus temperature (2 bytes) in between (which we'll ignore)
    mpu6050_read_registers(MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
    
    // Combine high and low bytes for each axis (Big Endian)
    int16_t accel_x_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t accel_y_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t accel_z_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
    // buffer[6-7] is temperature (skipped)
    int16_t gyro_x_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t gyro_y_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t gyro_z_raw = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    // Convert to g's and °/s, then to fixed-point
    accel[0] = float2fix15((float)accel_x_raw / ACCEL_SCALE);
    accel[1] = float2fix15((float)accel_y_raw / ACCEL_SCALE);
    accel[2] = float2fix15((float)accel_z_raw / ACCEL_SCALE);
    
    gyro[0] = float2fix15((float)gyro_x_raw / GYRO_SCALE);
    gyro[1] = float2fix15((float)gyro_y_raw / GYRO_SCALE);
    gyro[2] = float2fix15((float)gyro_z_raw / GYRO_SCALE);
}

/**
 * Read raw accelerometer data only
 */
void mpu6050_read_accel(fix15 accel[3]) {
    uint8_t buffer[6];
    
    // Read 6 bytes starting from ACCEL_XOUT_H
    mpu6050_read_registers(MPU6050_REG_ACCEL_XOUT_H, buffer, 6);
    
    // Combine high and low bytes for each axis
    int16_t accel_x_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t accel_y_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t accel_z_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // Convert to g's, then to fixed-point
    accel[0] = float2fix15((float)accel_x_raw / ACCEL_SCALE);
    accel[1] = float2fix15((float)accel_y_raw / ACCEL_SCALE);
    accel[2] = float2fix15((float)accel_z_raw / ACCEL_SCALE);
}

/**
 * Read raw gyroscope data only
 */
void mpu6050_read_gyro(fix15 gyro[3]) {
    uint8_t buffer[6];
    
    // Read 6 bytes starting from GYRO_XOUT_H
    mpu6050_read_registers(MPU6050_REG_GYRO_XOUT_H, buffer, 6);
    
    // Combine high and low bytes for each axis
    int16_t gyro_x_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t gyro_y_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t gyro_z_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // Convert to °/s, then to fixed-point
    gyro[0] = float2fix15((float)gyro_x_raw / GYRO_SCALE);
    gyro[1] = float2fix15((float)gyro_y_raw / GYRO_SCALE);
    gyro[2] = float2fix15((float)gyro_z_raw / GYRO_SCALE);
}

/**
 * Check if MPU6050 is connected and responding
 */
bool mpu6050_is_connected(void) {
    uint8_t who_am_i;
    mpu6050_read_registers(MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    
    // WHO_AM_I register should return 0x68
    return (who_am_i == 0x68);
}