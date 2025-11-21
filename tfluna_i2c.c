#include "tfluna_i2c.h"
#include <string.h>

// Helper function to read register
static bool read_register(tfluna_t *sensor, uint8_t reg, uint8_t *data, size_t len) {
    int ret = i2c_write_blocking(sensor->i2c_port, sensor->address, &reg, 1, true);
    if (ret < 0) return false;
    
    ret = i2c_read_blocking(sensor->i2c_port, sensor->address, data, len, false);
    return (ret >= 0);
}

// Helper function to write register
static bool write_register(tfluna_t *sensor, uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    int ret = i2c_write_blocking(sensor->i2c_port, sensor->address, buffer, 2, false);
    return (ret >= 0);
}

// Helper function to write 16-bit value to register
static bool write_register_16(tfluna_t *sensor, uint8_t reg, uint16_t data) {
    uint8_t buffer[3] = {reg, (uint8_t)(data & 0xFF), (uint8_t)(data >> 8)};
    int ret = i2c_write_blocking(sensor->i2c_port, sensor->address, buffer, 3, false);
    return (ret >= 0);
}

bool tfluna_init(tfluna_t *sensor, i2c_inst_t *i2c_port, uint8_t address) {
    if (!sensor || !i2c_port) return false;
    
    sensor->i2c_port = i2c_port;
    sensor->address = address;
    
    // Check if sensor is responding
    if (!tfluna_is_connected(sensor)) {
        return false;
    }
    
    // Set to continuous mode by default
    sleep_ms(100);
    if (!tfluna_set_mode(sensor, TFLUNA_MODE_CONTINUOUS)) {
        return false;
    }
    
    sleep_ms(100);
    return true;
}

bool tfluna_read_data(tfluna_t *sensor, tfluna_data_t *data) {
    if (!sensor || !data) return false;
    
    uint8_t buffer[8];
    
    // Read all 8 bytes starting from distance low register
    if (!read_register(sensor, TFLUNA_REG_DIST_LOW, buffer, 8)) {
        data->valid = false;
        return false;
    }
    
    // Parse the data
    data->distance = (uint16_t)(buffer[1] << 8) | buffer[0];
    data->amplitude = (uint16_t)(buffer[3] << 8) | buffer[2];
    data->temperature = (uint16_t)(buffer[5] << 8) | buffer[4];
    data->tick = (uint16_t)(buffer[7] << 8) | buffer[6];
    
    // Check if distance is valid (non-zero and within range)
    data->valid = (data->distance > 0 && data->distance <= 800);
    
    return true;
}

bool tfluna_read_distance(tfluna_t *sensor, uint16_t *distance) {
    if (!sensor || !distance) return false;
    
    uint8_t buffer[2];
    
    // Read only distance registers (2 bytes)
    if (!read_register(sensor, TFLUNA_REG_DIST_LOW, buffer, 2)) {
        return false;
    }
    
    *distance = (uint16_t)(buffer[1] << 8) | buffer[0];
    return true;
}

bool tfluna_get_version(tfluna_t *sensor, uint8_t *version) {
    if (!sensor || !version) return false;
    
    return read_register(sensor, TFLUNA_REG_VERSION, version, 1);
}

bool tfluna_set_mode(tfluna_t *sensor, uint8_t mode) {
    if (!sensor) return false;
    
    return write_register(sensor, TFLUNA_REG_MODE, mode);
}

bool tfluna_trigger(tfluna_t *sensor) {
    if (!sensor) return false;
    
    return write_register(sensor, TFLUNA_REG_TRIGGER, 0x01);
}

bool tfluna_set_frequency(tfluna_t *sensor, uint16_t freq) {
    if (!sensor) return false;
    
    // Frequency range: 0-1000 Hz (typical usage: 1-250 Hz)
    if (freq > 1000) freq = 1000;
    
    return write_register_16(sensor, TFLUNA_REG_FREQ_LOW, freq);
}

bool tfluna_set_address(tfluna_t *sensor, uint8_t new_address) {
    if (!sensor) return false;
    
    // Valid I2C addresses: 0x08 to 0x77
    if (new_address < 0x08 || new_address > 0x77) return false;
    
    if (write_register(sensor, TFLUNA_REG_SLAVE_ADDR, new_address)) {
        sensor->address = new_address;
        return true;
    }
    return false;
}

bool tfluna_save_settings(tfluna_t *sensor) {
    if (!sensor) return false;
    
    // Write 0x01 to save register
    if (!write_register(sensor, TFLUNA_REG_SAVE, 0x01)) {
        return false;
    }
    
    // Wait for save to complete
    sleep_ms(100);
    return true;
}

bool tfluna_restore_factory(tfluna_t *sensor) {
    if (!sensor) return false;
    
    // Write 0x01 to restore register
    if (!write_register(sensor, TFLUNA_REG_RESTORE, 0x01)) {
        return false;
    }
    
    // Wait for restore to complete
    sleep_ms(500);
    return true;
}

bool tfluna_reboot(tfluna_t *sensor) {
    if (!sensor) return false;
    
    // Write 0x02 to reboot register
    if (!write_register(sensor, TFLUNA_REG_REBOOT, 0x02)) {
        return false;
    }
    
    // Wait for reboot to complete
    sleep_ms(500);
    return true;
}

bool tfluna_set_low_power(tfluna_t *sensor, bool enable) {
    if (!sensor) return false;
    
    return write_register(sensor, TFLUNA_REG_LOW_POWER, enable ? 0x01 : 0x00);
}

bool tfluna_is_connected(tfluna_t *sensor) {
    if (!sensor) return false;
    
    uint8_t data;
    int ret = i2c_read_timeout_us(sensor->i2c_port, sensor->address, &data, 1, false, 10000);
    return (ret > 0);
}
