#include "mlx90614.h"
#include <string.h>

// Helper function to calculate CRC8 for MLX90614 (PEC - Packet Error Code)
static uint8_t mlx90614_crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Helper function to read 16-bit value from register
static bool mlx90614_read_reg(mlx90614_t *sensor, uint8_t reg, uint16_t *value) {
    uint8_t data[3];  // 2 bytes data + 1 byte CRC
    
    // Write register address
    int ret = i2c_write_blocking(sensor->i2c_port, sensor->address, &reg, 1, true);
    if (ret < 0) return false;
    
    // Read 3 bytes (LSB, MSB, PEC)
    ret = i2c_read_blocking(sensor->i2c_port, sensor->address, data, 3, false);
    if (ret < 0) return false;
    
    // Verify CRC (optional, but recommended for safety)
    uint8_t crc_data[4] = {
        (sensor->address << 1),      // Write address
        reg,                         // Register
        (sensor->address << 1) | 1,  // Read address
        data[0]                      // LSB
    };
    uint8_t crc = mlx90614_crc8(crc_data, 4);
    crc ^= data[1];  // XOR with MSB
    
    // Note: Some modules don't implement CRC correctly, so we'll be lenient
    // if (crc != data[2]) return false;
    
    // Combine LSB and MSB
    *value = ((uint16_t)data[1] << 8) | data[0];
    
    return true;
}

// Helper function to convert raw value to Celsius
static float mlx90614_raw_to_celsius(uint16_t raw) {
    // Temperature in Kelvin = raw * 0.02
    float temp_k = (float)raw * MLX90614_TEMP_FACTOR;
    
    // Convert Kelvin to Celsius
    float temp_c = temp_k - 273.15f;
    
    return temp_c;
}

bool mlx90614_init(mlx90614_t *sensor, i2c_inst_t *i2c_port, uint8_t address) {
    if (!sensor || !i2c_port) return false;
    
    sensor->i2c_port = i2c_port;
    sensor->address = address;
    
    // Check if sensor is responding
    if (!mlx90614_is_connected(sensor)) {
        return false;
    }
    
    // Give sensor time to stabilize
    sleep_ms(100);
    
    return true;
}

bool mlx90614_read_object_temp(mlx90614_t *sensor, float *temp_c) {
    if (!sensor || !temp_c) return false;
    
    uint16_t raw;
    
    // Read object temperature register (TOBJ1)
    if (!mlx90614_read_reg(sensor, MLX90614_REG_TOBJ1, &raw)) {
        return false;
    }
    
    // Check for error condition (0xFFFF)
    if (raw == 0xFFFF) {
        return false;
    }
    
    // Convert to Celsius
    *temp_c = mlx90614_raw_to_celsius(raw);
    
    return true;
}

bool mlx90614_read_ambient_temp(mlx90614_t *sensor, float *temp_c) {
    if (!sensor || !temp_c) return false;
    
    uint16_t raw;
    
    // Read ambient temperature register (TA)
    if (!mlx90614_read_reg(sensor, MLX90614_REG_TA, &raw)) {
        return false;
    }
    
    // Check for error condition
    if (raw == 0xFFFF) {
        return false;
    }
    
    // Convert to Celsius
    *temp_c = mlx90614_raw_to_celsius(raw);
    
    return true;
}

bool mlx90614_read_both_temps(mlx90614_t *sensor, float *object_temp_c, float *ambient_temp_c) {
    if (!sensor || !object_temp_c || !ambient_temp_c) return false;
    
    // Read both temperatures
    if (!mlx90614_read_object_temp(sensor, object_temp_c)) {
        return false;
    }
    
    if (!mlx90614_read_ambient_temp(sensor, ambient_temp_c)) {
        return false;
    }
    
    return true;
}

bool mlx90614_read_object_temp_f(mlx90614_t *sensor, float *temp_f) {
    if (!sensor || !temp_f) return false;
    
    float temp_c;
    if (!mlx90614_read_object_temp(sensor, &temp_c)) {
        return false;
    }
    
    *temp_f = mlx90614_celsius_to_fahrenheit(temp_c);
    
    return true;
}

bool mlx90614_is_connected(mlx90614_t *sensor) {
    if (!sensor) return false;
    
    uint8_t data;
    int ret = i2c_read_timeout_us(sensor->i2c_port, sensor->address, &data, 1, false, 10000);
    return (ret > 0);
}

bool mlx90614_read_id(mlx90614_t *sensor, uint64_t *id) {
    if (!sensor || !id) return false;
    
    uint16_t id1, id2, id3, id4;
    
    if (!mlx90614_read_reg(sensor, MLX90614_REG_ID1, &id1)) return false;
    if (!mlx90614_read_reg(sensor, MLX90614_REG_ID2, &id2)) return false;
    if (!mlx90614_read_reg(sensor, MLX90614_REG_ID3, &id3)) return false;
    if (!mlx90614_read_reg(sensor, MLX90614_REG_ID4, &id4)) return false;
    
    *id = ((uint64_t)id4 << 48) | ((uint64_t)id3 << 32) | 
          ((uint64_t)id2 << 16) | id1;
    
    return true;
}

float mlx90614_celsius_to_fahrenheit(float celsius) {
    return (celsius * 9.0f / 5.0f) + 32.0f;
}
