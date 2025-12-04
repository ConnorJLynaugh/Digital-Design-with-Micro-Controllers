/***************************************************
 * MLX90614 Temperature Sensor Driver for Raspberry Pi Pico
 * 
 * Based on Adafruit's MLX90614 Library
 * Original: https://github.com/adafruit/Adafruit-MLX90614-Library
 * 
 * Adapted for Raspberry Pi Pico by Sarah Grace
 * Cornell University - MAE/CS Robotics Projects
 * 
 * Sensor Info:
 * - Non-contact infrared thermometer
 * - I2C interface (default address 0x5A)
 * - Measures object and ambient temperature
 * - Temperature resolution: 0.02K per LSB
 ****************************************************/

#include "mlx90614.h"
#include <string.h>

/**
 * @brief Calculate CRC-8 for MLX90614 (PEC - Packet Error Code)
 * 
 * Based on Adafruit's implementation. The PEC calculation includes all bits 
 * except START, REPEATED START, STOP, ACK, and NACK bits.
 * Polynomial: X8+X2+X1+1 (0x07)
 * 
 * @param data Pointer to data buffer
 * @param len Number of bytes to calculate CRC over
 * @return Calculated CRC-8 value
 */
static uint8_t mlx90614_crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    
    for (uint8_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (uint8_t j = 8; j > 0; j--) {
            uint8_t carry = (crc ^ inbyte) & 0x80;
            crc <<= 1;
            if (carry) {
                crc ^= 0x07;
            }
            inbyte <<= 1;
        }
    }
    
    return crc;
}

/**
 * @brief Read 16-bit value from MLX90614 register
 * 
 * This function performs a write-then-read I2C transaction and verifies
 * the PEC (Packet Error Code) returned by the sensor.
 * 
 * NOTE: Adafruit's library reads the PEC but doesn't validate it because
 * some modules don't implement CRC correctly. We'll be lenient too.
 * 
 * @param sensor Pointer to sensor structure
 * @param reg Register address to read
 * @param value Pointer to store the 16-bit result
 * @return true if successful, false on error
 */
static bool mlx90614_read_reg(mlx90614_t *sensor, uint8_t reg, uint16_t *value) {
    uint8_t data[3];  // LSB, MSB, PEC
    
    // Write register address (with repeated start)
    int ret = i2c_write_blocking(sensor->i2c_port, sensor->address, &reg, 1, true);
    if (ret < 0) {
        return false;
    }
    
    // Read 3 bytes: data LSB, data MSB, PEC
    ret = i2c_read_blocking(sensor->i2c_port, sensor->address, data, 3, false);
    if (ret < 0) {
        return false;
    }
    
    // Optional: Verify PEC (based on Adafruit's approach - lenient)
    // The PEC includes: write_addr, command, read_addr, data_LSB, data_MSB
    // Uncomment below if you want strict CRC checking
    /*
    uint8_t crc_data[5] = {
        (sensor->address << 1),       // Write address
        reg,                          // Command byte
        (sensor->address << 1) | 1,   // Read address
        data[0],                      // Data LSB
        data[1]                       // Data MSB
    };
    uint8_t expected_crc = mlx90614_crc8(crc_data, 5);
    if (expected_crc != data[2]) {
        return false;  // CRC mismatch
    }
    */
    
    // Combine LSB and MSB (little-endian)
    *value = ((uint16_t)data[1] << 8) | data[0];
    
    return true;
}

/**
 * @brief Write 16-bit value to MLX90614 register (EEPROM)
 * 
 * This function writes to EEPROM registers with proper PEC calculation.
 * Note: EEPROM cells must be erased (write 0x0000) before writing new values.
 * 
 * @param sensor Pointer to sensor structure
 * @param reg Register address to write
 * @param value 16-bit value to write
 * @return true if successful, false on error
 */
static bool mlx90614_write_reg(mlx90614_t *sensor, uint8_t reg, uint16_t value) {
    uint8_t data[4];
    
    // Build command: register, data LSB, data MSB, PEC
    data[0] = reg;
    data[1] = value & 0xFF;        // LSB
    data[2] = (value >> 8) & 0xFF; // MSB
    
    // Calculate PEC over: write_addr, command, data_LSB, data_MSB
    uint8_t crc_data[4] = {
        (sensor->address << 1),  // Write address
        data[0],                 // Command (register)
        data[1],                 // Data LSB
        data[2]                  // Data MSB
    };
    data[3] = mlx90614_crc8(crc_data, 4);
    
    // Write all 4 bytes
    int ret = i2c_write_blocking(sensor->i2c_port, sensor->address, data, 4, false);
    
    return (ret == 4);
}

/**
 * @brief Convert raw temperature value to Celsius
 * 
 * Based on Adafruit's implementation:
 * - temp_kelvin = raw * 0.02
 * - temp_celsius = temp_kelvin - 273.15
 * 
 * @param raw Raw 16-bit temperature value from sensor
 * @return Temperature in degrees Celsius
 */
static float mlx90614_raw_to_celsius(uint16_t raw) {
    // Temperature in Kelvin = raw * 0.02
    float temp_k = (float)raw * MLX90614_TEMP_FACTOR;
    
    // Convert Kelvin to Celsius
    float temp_c = temp_k - 273.15f;
    
    return temp_c;
}

/**
 * @brief Read temperature from specified register
 * 
 * Internal helper function used by public temperature reading functions.
 * 
 * @param sensor Pointer to sensor structure
 * @param reg Register to read (TA, TOBJ1, or TOBJ2)
 * @return Temperature in Celsius, or NAN on error
 */
static float mlx90614_read_temp(mlx90614_t *sensor, uint8_t reg) {
    uint16_t raw;
    
    if (!mlx90614_read_reg(sensor, reg, &raw)) {
        return NAN;
    }
    
    // Check for error condition (all bits set)
    if (raw == 0xFFFF) {
        return NAN;
    }
    
    // Optionally check bit 15 for error flag (uncomment if needed)
    // if (raw & 0x8000) {
    //     return NAN;
    // }
    
    return mlx90614_raw_to_celsius(raw);
}

/*******************************************************************************
 * PUBLIC API FUNCTIONS
 ******************************************************************************/

bool mlx90614_init(mlx90614_t *sensor, i2c_inst_t *i2c_port, uint8_t address) {
    if (!sensor || !i2c_port) {
        return false;
    }
    
    sensor->i2c_port = i2c_port;
    sensor->address = address;
    
    // Check if sensor is responding
    if (!mlx90614_is_connected(sensor)) {
        return false;
    }
    
    // Optional: Read and verify ID
    uint64_t id;
    if (mlx90614_read_id(sensor, &id)) {
        if (id == 0) {
            return false;  // Invalid ID
        }
    }
    
    // Give sensor time to stabilize after power-on
    sleep_ms(250);  // Datasheet recommends 250ms after POR
    
    return true;
}

bool mlx90614_read_object_temp(mlx90614_t *sensor, float *temp_c) {
    if (!sensor || !temp_c) {
        return false;
    }
    
    float temp = mlx90614_read_temp(sensor, MLX90614_REG_TOBJ1);
    
    if (isnan(temp)) {
        return false;
    }
    
    *temp_c = temp;
    return true;
}

bool mlx90614_read_ambient_temp(mlx90614_t *sensor, float *temp_c) {
    if (!sensor || !temp_c) {
        return false;
    }
    
    float temp = mlx90614_read_temp(sensor, MLX90614_REG_TA);
    
    if (isnan(temp)) {
        return false;
    }
    
    *temp_c = temp;
    return true;
}

bool mlx90614_read_both_temps(mlx90614_t *sensor, float *object_temp_c, float *ambient_temp_c) {
    if (!sensor || !object_temp_c || !ambient_temp_c) {
        return false;
    }
    
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
    if (!sensor || !temp_f) {
        return false;
    }
    
    float temp_c;
    if (!mlx90614_read_object_temp(sensor, &temp_c)) {
        return false;
    }
    
    *temp_f = mlx90614_celsius_to_fahrenheit(temp_c);
    return true;
}

bool mlx90614_read_ambient_temp_f(mlx90614_t *sensor, float *temp_f) {
    if (!sensor || !temp_f) {
        return false;
    }
    
    float temp_c;
    if (!mlx90614_read_ambient_temp(sensor, &temp_c)) {
        return false;
    }
    
    *temp_f = mlx90614_celsius_to_fahrenheit(temp_c);
    return true;
}

bool mlx90614_is_connected(mlx90614_t *sensor) {
    if (!sensor) {
        return false;
    }
    
    // Try to read a register to check if device responds
    uint16_t dummy;
    return mlx90614_read_reg(sensor, MLX90614_REG_TA, &dummy);
}

bool mlx90614_read_id(mlx90614_t *sensor, uint64_t *id) {
    if (!sensor || !id) {
        return false;
    }
    
    uint16_t id1, id2, id3, id4;
    
    if (!mlx90614_read_reg(sensor, MLX90614_REG_ID1, &id1)) return false;
    if (!mlx90614_read_reg(sensor, MLX90614_REG_ID2, &id2)) return false;
    if (!mlx90614_read_reg(sensor, MLX90614_REG_ID3, &id3)) return false;
    if (!mlx90614_read_reg(sensor, MLX90614_REG_ID4, &id4)) return false;
    
    // Combine into 64-bit ID
    *id = ((uint64_t)id4 << 48) | ((uint64_t)id3 << 32) | 
          ((uint64_t)id2 << 16) | id1;
    
    return true;
}

float mlx90614_celsius_to_fahrenheit(float celsius) {
    return (celsius * 9.0f / 5.0f) + 32.0f;
}

/*******************************************************************************
 * EMISSIVITY FUNCTIONS (NEW - from Adafruit library)
 ******************************************************************************/

bool mlx90614_read_emissivity(mlx90614_t *sensor, float *emissivity) {
    if (!sensor || !emissivity) {
        return false;
    }
    
    uint16_t raw;
    if (!mlx90614_read_reg(sensor, MLX90614_REG_EMISSIVITY, &raw)) {
        return false;
    }
    
    // Emissivity stored as 16-bit value: 0xFFFF = 1.0
    // Factory default is typically 1.0 (0xFFFF)
    *emissivity = (float)raw / 65535.0f;
    
    return true;
}

bool mlx90614_write_emissivity(mlx90614_t *sensor, float emissivity) {
    if (!sensor || emissivity < 0.1f || emissivity > 1.0f) {
        return false;
    }
    
    // Convert float to 16-bit register value
    uint16_t ereg = (uint16_t)(emissivity * 65535.0f + 0.5f);
    
    return mlx90614_write_emissivity_reg(sensor, ereg);
}

bool mlx90614_read_emissivity_reg(mlx90614_t *sensor, uint16_t *ereg) {
    if (!sensor || !ereg) {
        return false;
    }
    
    return mlx90614_read_reg(sensor, MLX90614_REG_EMISSIVITY, ereg);
}

bool mlx90614_write_emissivity_reg(mlx90614_t *sensor, uint16_t ereg) {
    if (!sensor) {
        return false;
    }
    
    // CRITICAL: EEPROM cells must be erased before writing!
    // Step 1: Erase the cell (write 0x0000)
    if (!mlx90614_write_reg(sensor, MLX90614_REG_EMISSIVITY, 0x0000)) {
        return false;
    }
    
    sleep_ms(10);  // Wait for erase to complete
    
    // Step 2: Write the new value
    if (!mlx90614_write_reg(sensor, MLX90614_REG_EMISSIVITY, ereg)) {
        return false;
    }
    
    sleep_ms(10);  // Wait for write to complete
    
    return true;
}

/*******************************************************************************
 * ADVANCED FUNCTIONS (Optional)
 ******************************************************************************/

bool mlx90614_read_config(mlx90614_t *sensor, uint16_t *config) {
    if (!sensor || !config) {
        return false;
    }
    
    return mlx90614_read_reg(sensor, MLX90614_REG_CONFIG, config);
}

bool mlx90614_read_raw_ir1(mlx90614_t *sensor, uint16_t *raw_ir) {
    if (!sensor || !raw_ir) {
        return false;
    }
    
    return mlx90614_read_reg(sensor, MLX90614_REG_RAWIR1, raw_ir);
}

bool mlx90614_read_raw_ir2(mlx90614_t *sensor, uint16_t *raw_ir) {
    if (!sensor || !raw_ir) {
        return false;
    }
    
    return mlx90614_read_reg(sensor, MLX90614_REG_RAWIR2, raw_ir);
}