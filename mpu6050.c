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
    int16_t temp_accel, temp_gyro ;

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_CHAN, ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_CHAN, ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        temp_accel = (buffer[i<<1] << 8 | buffer[(i<<1) + 1]);
        accel[i] = temp_accel ;
        accel[i] <<= 2 ; // convert to g's (fixed point)
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(I2C_CHAN, ADDRESS, &val, 1, true);
    i2c_read_blocking(I2C_CHAN, ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        temp_gyro = (buffer[i<<1] << 8 | buffer[(i<<1) + 1]);
        gyro[i] = temp_gyro ;
        gyro[i] = multfix15(gyro[i], 500<<16) ; // deg/sec
    }
}
/////////////////////////////////////////////////////////////////
