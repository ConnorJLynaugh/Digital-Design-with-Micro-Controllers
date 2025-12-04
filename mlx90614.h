/***************************************************
 * MLX90614 Temperature Sensor Driver for Raspberry Pi Pico
 * 
 * Based on Adafruit's MLX90614 Library
 * Original: https://github.com/adafruit/Adafruit-MLX90614-Library
 * 
 * Adapted for Raspberry Pi Pico by Sarah Grace
 * Cornell University - MAE/CS Robotics Projects
 * 
 * Sensor Specifications:
 * - Non-contact infrared thermometer
 * - I2C interface (SMBus compatible)
 * - Default I2C address: 0x5A
 * - Temperature resolution: 0.02K per LSB
 * - Object temp range: -70°C to +380°C
 * - Ambient temp range: -40°C to +125°C
 * - FOV options: 90°, 50°, 35°, 10°, 5°
 ****************************************************/

#ifndef MLX90614_H
#define MLX90614_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>  // For NAN

/*******************************************************************************
 * I2C ADDRESS
 ******************************************************************************/
#define MLX90614_DEFAULT_ADDR 0x5A  ///< Default I2C address

/*******************************************************************************
 * RAM REGISTERS (Volatile - cleared on power cycle)
 ******************************************************************************/
#define MLX90614_REG_RAWIR1     0x04  ///< Raw IR data channel 1
#define MLX90614_REG_RAWIR2     0x05  ///< Raw IR data channel 2
#define MLX90614_REG_TA         0x06  ///< Ambient temperature
#define MLX90614_REG_TOBJ1      0x07  ///< Object 1 temperature
#define MLX90614_REG_TOBJ2      0x08  ///< Object 2 temperature (dual zone only)

/*******************************************************************************
 * EEPROM REGISTERS (Non-volatile - persists across power cycles)
 ******************************************************************************/
#define MLX90614_REG_TOMAX      0x20  ///< Object temperature max
#define MLX90614_REG_TOMIN      0x21  ///< Object temperature min
#define MLX90614_REG_PWMCTRL    0x22  ///< PWM control
#define MLX90614_REG_TARANGE    0x23  ///< Ambient temperature range
#define MLX90614_REG_EMISSIVITY 0x24  ///< Emissivity correction coefficient
#define MLX90614_REG_CONFIG     0x25  ///< Configuration register
#define MLX90614_REG_ADDR       0x2E  ///< I2C address (for changing address)
#define MLX90614_REG_ID1        0x3C  ///< ID number byte 1
#define MLX90614_REG_ID2        0x3D  ///< ID number byte 2
#define MLX90614_REG_ID3        0x3E  ///< ID number byte 3
#define MLX90614_REG_ID4        0x3F  ///< ID number byte 4

/*******************************************************************************
 * CONVERSION CONSTANTS
 ******************************************************************************/
#define MLX90614_TEMP_FACTOR 0.02f  ///< Temperature resolution: 0.02K per LSB

/*******************************************************************************
 * DATA STRUCTURES
 ******************************************************************************/

/**
 * @brief MLX90614 sensor instance structure
 */
typedef struct {
    i2c_inst_t *i2c_port;  ///< Pointer to I2C instance (i2c0 or i2c1)
    uint8_t address;       ///< I2C device address (typically 0x5A)
} mlx90614_t;

/*******************************************************************************
 * CORE FUNCTIONS
 ******************************************************************************/

/**
 * @brief Initialize MLX90614 sensor
 * 
 * Initializes the sensor structure and verifies communication.
 * Waits 250ms for sensor stabilization per datasheet recommendation.
 * 
 * @param sensor Pointer to sensor structure
 * @param i2c_port I2C instance (i2c0 or i2c1)
 * @param address I2C address (typically MLX90614_DEFAULT_ADDR)
 * @return true if initialization successful, false otherwise
 * 
 * @note I2C bus must be initialized before calling this function
 * 
 * Example:
 * @code
 * mlx90614_t sensor;
 * i2c_init(i2c0, 100000);
 * gpio_set_function(4, GPIO_FUNC_I2C);
 * gpio_set_function(5, GPIO_FUNC_I2C);
 * gpio_pull_up(4);
 * gpio_pull_up(5);
 * 
 * if (mlx90614_init(&sensor, i2c0, MLX90614_DEFAULT_ADDR)) {
 *     printf("Sensor initialized!\n");
 * }
 * @endcode
 */
bool mlx90614_init(mlx90614_t *sensor, i2c_inst_t *i2c_port, uint8_t address);

/**
 * @brief Read object temperature in Celsius
 * 
 * Reads the non-contact object temperature from TOBJ1 register.
 * 
 * @param sensor Pointer to sensor structure
 * @param temp_c Pointer to store temperature in degrees Celsius
 * @return true if read successful, false on error
 */
bool mlx90614_read_object_temp(mlx90614_t *sensor, float *temp_c);

/**
 * @brief Read ambient temperature in Celsius
 * 
 * Reads the ambient (sensor die) temperature from TA register.
 * 
 * @param sensor Pointer to sensor structure
 * @param temp_c Pointer to store temperature in degrees Celsius
 * @return true if read successful, false on error
 */
bool mlx90614_read_ambient_temp(mlx90614_t *sensor, float *temp_c);

/**
 * @brief Read both object and ambient temperatures
 * 
 * Convenience function to read both temperatures in one call.
 * 
 * @param sensor Pointer to sensor structure
 * @param object_temp_c Pointer to store object temperature in Celsius
 * @param ambient_temp_c Pointer to store ambient temperature in Celsius
 * @return true if both reads successful, false on error
 */
bool mlx90614_read_both_temps(mlx90614_t *sensor, float *object_temp_c, float *ambient_temp_c);

/**
 * @brief Read object temperature in Fahrenheit
 * 
 * @param sensor Pointer to sensor structure
 * @param temp_f Pointer to store temperature in degrees Fahrenheit
 * @return true if read successful, false on error
 */
bool mlx90614_read_object_temp_f(mlx90614_t *sensor, float *temp_f);

/**
 * @brief Read ambient temperature in Fahrenheit
 * 
 * @param sensor Pointer to sensor structure
 * @param temp_f Pointer to store temperature in degrees Fahrenheit
 * @return true if read successful, false on error
 */
bool mlx90614_read_ambient_temp_f(mlx90614_t *sensor, float *temp_f);

/*******************************************************************************
 * UTILITY FUNCTIONS
 ******************************************************************************/

/**
 * @brief Check if sensor is connected and responding
 * 
 * @param sensor Pointer to sensor structure
 * @return true if sensor responds, false otherwise
 */
bool mlx90614_is_connected(mlx90614_t *sensor);

/**
 * @brief Read 64-bit unique device ID
 * 
 * Each MLX90614 has a unique ID programmed at factory.
 * 
 * @param sensor Pointer to sensor structure
 * @param id Pointer to store 64-bit ID
 * @return true if read successful, false on error
 */
bool mlx90614_read_id(mlx90614_t *sensor, uint64_t *id);

/**
 * @brief Convert Celsius to Fahrenheit
 * 
 * @param celsius Temperature in Celsius
 * @return Temperature in Fahrenheit
 */
float mlx90614_celsius_to_fahrenheit(float celsius);

/*******************************************************************************
 * EMISSIVITY FUNCTIONS (from Adafruit library)
 ******************************************************************************/

/**
 * @brief Read emissivity coefficient as float (0.1 to 1.0)
 * 
 * Emissivity correction factor for different materials.
 * Factory default is typically 1.0 (perfect black body).
 * 
 * Common emissivity values:
 * - Human skin: 0.98
 * - Matte surfaces: 0.95
 * - Shiny metal: 0.1-0.3
 * 
 * @param sensor Pointer to sensor structure
 * @param emissivity Pointer to store emissivity value
 * @return true if read successful, false on error
 */
bool mlx90614_read_emissivity(mlx90614_t *sensor, float *emissivity);

/**
 * @brief Write emissivity coefficient as float (0.1 to 1.0)
 * 
 * WARNING: This writes to EEPROM and has limited write cycles (~100k).
 * The function automatically erases the cell before writing.
 * 
 * @param sensor Pointer to sensor structure
 * @param emissivity Emissivity value (0.1 to 1.0)
 * @return true if write successful, false on error
 */
bool mlx90614_write_emissivity(mlx90614_t *sensor, float emissivity);

/**
 * @brief Read raw emissivity register value (0x0000 to 0xFFFF)
 * 
 * @param sensor Pointer to sensor structure
 * @param ereg Pointer to store raw 16-bit value
 * @return true if read successful, false on error
 */
bool mlx90614_read_emissivity_reg(mlx90614_t *sensor, uint16_t *ereg);

/**
 * @brief Write raw emissivity register value (0x0000 to 0xFFFF)
 * 
 * WARNING: This writes to EEPROM. The function automatically erases
 * the cell before writing.
 * 
 * @param sensor Pointer to sensor structure
 * @param ereg Raw 16-bit value (0xFFFF = 1.0)
 * @return true if write successful, false on error
 */
bool mlx90614_write_emissivity_reg(mlx90614_t *sensor, uint16_t ereg);

/*******************************************************************************
 * ADVANCED FUNCTIONS
 ******************************************************************************/

/**
 * @brief Read configuration register
 * 
 * @param sensor Pointer to sensor structure
 * @param config Pointer to store 16-bit config value
 * @return true if read successful, false on error
 */
bool mlx90614_read_config(mlx90614_t *sensor, uint16_t *config);

/**
 * @brief Read raw IR sensor data from channel 1
 * 
 * @param sensor Pointer to sensor structure
 * @param raw_ir Pointer to store raw IR value
 * @return true if read successful, false on error
 */
bool mlx90614_read_raw_ir1(mlx90614_t *sensor, uint16_t *raw_ir);

/**
 * @brief Read raw IR sensor data from channel 2 (dual zone only)
 * 
 * @param sensor Pointer to sensor structure
 * @param raw_ir Pointer to store raw IR value
 * @return true if read successful, false on error
 */
bool mlx90614_read_raw_ir2(mlx90614_t *sensor, uint16_t *raw_ir);

#endif // MLX90614_H