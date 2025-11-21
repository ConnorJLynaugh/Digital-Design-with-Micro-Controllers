#ifndef TFLUNA_I2C_H
#define TFLUNA_I2C_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// TF-Luna I2C default address
#define TFLUNA_DEFAULT_ADDR 0x10

// TF-Luna Register Addresses
#define TFLUNA_REG_DIST_LOW     0x00  // Distance low byte
#define TFLUNA_REG_DIST_HIGH    0x01  // Distance high byte
#define TFLUNA_REG_AMP_LOW      0x02  // Amplitude low byte
#define TFLUNA_REG_AMP_HIGH     0x03  // Amplitude high byte
#define TFLUNA_REG_TEMP_LOW     0x04  // Temperature low byte
#define TFLUNA_REG_TEMP_HIGH    0x05  // Temperature high byte
#define TFLUNA_REG_TICK_LOW     0x06  // Tick counter low byte
#define TFLUNA_REG_TICK_HIGH    0x07  // Tick counter high byte

// Command registers
#define TFLUNA_REG_VERSION      0x0A  // Firmware version
#define TFLUNA_REG_SLAVE_ADDR   0x11  // I2C slave address
#define TFLUNA_REG_MODE         0x12  // Mode register
#define TFLUNA_REG_TRIGGER      0x13  // Trigger register
#define TFLUNA_REG_FREQ_LOW     0x14  // Frequency low byte
#define TFLUNA_REG_FREQ_HIGH    0x15  // Frequency high byte
#define TFLUNA_REG_LOW_POWER    0x16  // Low power mode
#define TFLUNA_REG_SAVE         0x20  // Save settings to flash
#define TFLUNA_REG_REBOOT       0x21  // Reboot sensor
#define TFLUNA_REG_RESTORE      0x25  // Restore factory settings

// Measurement modes
#define TFLUNA_MODE_CONTINUOUS  0x00  // Continuous measurement mode
#define TFLUNA_MODE_TRIGGER     0x01  // Trigger mode (single shot)

// Data structure to hold sensor readings
typedef struct {
    uint16_t distance;      // Distance in cm
    uint16_t amplitude;     // Signal strength
    uint16_t temperature;   // Temperature (°C * 100)
    uint16_t tick;          // Internal tick counter
    bool valid;             // Data validity flag
} tfluna_data_t;

// TF-Luna sensor structure
typedef struct {
    i2c_inst_t *i2c_port;
    uint8_t address;
} tfluna_t;

// Function prototypes

/**
 * @brief Initialize the TF-Luna sensor
 * 
 * @param sensor Pointer to tfluna_t structure
 * @param i2c_port I2C port (i2c0 or i2c1)
 * @param address I2C address (default: 0x10)
 * @return true if initialization successful, false otherwise
 */
bool tfluna_init(tfluna_t *sensor, i2c_inst_t *i2c_port, uint8_t address);

/**
 * @brief Read distance measurement from sensor
 * 
 * @param sensor Pointer to tfluna_t structure
 * @param data Pointer to tfluna_data_t structure to store readings
 * @return true if read successful, false otherwise
 */
bool tfluna_read_data(tfluna_t *sensor, tfluna_data_t *data);

/**
 * @brief Read only distance from sensor (faster than full read)
 * 
 * @param sensor Pointer to tfluna_t structure
 * @param distance Pointer to store distance value (in cm)
 * @return true if read successful, false otherwise
 */
bool tfluna_read_distance(tfluna_t *sensor, uint16_t *distance);

/**
 * @brief Get firmware version
 * 
 * @param sensor Pointer to tfluna_t structure
 * @param version Pointer to store version number
 * @return true if read successful, false otherwise
 */
bool tfluna_get_version(tfluna_t *sensor, uint8_t *version);

/**
 * @brief Set measurement mode
 * 
 * @param sensor Pointer to tfluna_t structure
 * @param mode TFLUNA_MODE_CONTINUOUS or TFLUNA_MODE_TRIGGER
 * @return true if successful, false otherwise
 */
bool tfluna_set_mode(tfluna_t *sensor, uint8_t mode);

/**
 * @brief Trigger a single measurement (when in trigger mode)
 * 
 * @param sensor Pointer to tfluna_t structure
 * @return true if successful, false otherwise
 */
bool tfluna_trigger(tfluna_t *sensor);

/**
 * @brief Set measurement frequency (in Hz)
 * 
 * @param sensor Pointer to tfluna_t structure
 * @param freq Frequency in Hz (0-1000, typical: 1-250)
 * @return true if successful, false otherwise
 */
bool tfluna_set_frequency(tfluna_t *sensor, uint16_t freq);

/**
 * @brief Change I2C address of sensor
 * 
 * @param sensor Pointer to tfluna_t structure
 * @param new_address New I2C address (0x08-0x77)
 * @return true if successful, false otherwise
 */
bool tfluna_set_address(tfluna_t *sensor, uint8_t new_address);

/**
 * @brief Save current settings to flash memory
 * 
 * @param sensor Pointer to tfluna_t structure
 * @return true if successful, false otherwise
 */
bool tfluna_save_settings(tfluna_t *sensor);

/**
 * @brief Restore factory default settings
 * 
 * @param sensor Pointer to tfluna_t structure
 * @return true if successful, false otherwise
 */
bool tfluna_restore_factory(tfluna_t *sensor);

/**
 * @brief Reboot the sensor
 * 
 * @param sensor Pointer to tfluna_t structure
 * @return true if successful, false otherwise
 */
bool tfluna_reboot(tfluna_t *sensor);

/**
 * @brief Enable/disable low power mode
 * 
 * @param sensor Pointer to tfluna_t structure
 * @param enable true to enable low power mode, false to disable
 * @return true if successful, false otherwise
 */
bool tfluna_set_low_power(tfluna_t *sensor, bool enable);

/**
 * @brief Check if sensor is responding
 * 
 * @param sensor Pointer to tfluna_t structure
 * @return true if sensor responds, false otherwise
 */
bool tfluna_is_connected(tfluna_t *sensor);

#endif // TFLUNA_I2C_H
