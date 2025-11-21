#ifndef MLX90614_H
#define MLX90614_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// MLX90614 I2C default address
#define MLX90614_DEFAULT_ADDR 0x5A

// MLX90614 Register addresses
#define MLX90614_REG_RAWIR1     0x04  // Raw IR data 1
#define MLX90614_REG_RAWIR2     0x05  // Raw IR data 2
#define MLX90614_REG_TA         0x06  // Ambient temperature
#define MLX90614_REG_TOBJ1      0x07  // Object temperature 1
#define MLX90614_REG_TOBJ2      0x08  // Object temperature 2
#define MLX90614_REG_TOMAX      0x20  // Object temp max
#define MLX90614_REG_TOMIN      0x21  // Object temp min
#define MLX90614_REG_PWMCTRL    0x22  // PWM control
#define MLX90614_REG_TARANGE    0x23  // Ambient temp range
#define MLX90614_REG_EMISSIVITY 0x24  // Emissivity correction
#define MLX90614_REG_CONFIG     0x25  // Configuration register
#define MLX90614_REG_ADDR       0x2E  // I2C address
#define MLX90614_REG_ID1        0x3C  // ID number 1
#define MLX90614_REG_ID2        0x3D  // ID number 2
#define MLX90614_REG_ID3        0x3E  // ID number 3
#define MLX90614_REG_ID4        0x3F  // ID number 4

// Temperature conversion
#define MLX90614_TEMP_FACTOR 0.02  // Temperature resolution (Kelvin per LSB)

// MLX90614 sensor structure
typedef struct {
    i2c_inst_t *i2c_port;
    uint8_t address;
} mlx90614_t;

// Function prototypes

/**
 * @brief Initialize the MLX90614 sensor
 * 
 * @param sensor Pointer to mlx90614_t structure
 * @param i2c_port I2C port (i2c0 or i2c1)
 * @param address I2C address (default: 0x5A)
 * @return true if initialization successful, false otherwise
 */
bool mlx90614_init(mlx90614_t *sensor, i2c_inst_t *i2c_port, uint8_t address);

/**
 * @brief Read object temperature (non-contact)
 * 
 * @param sensor Pointer to mlx90614_t structure
 * @param temp_c Pointer to store temperature in Celsius
 * @return true if read successful, false otherwise
 */
bool mlx90614_read_object_temp(mlx90614_t *sensor, float *temp_c);

/**
 * @brief Read ambient temperature (sensor's own temperature)
 * 
 * @param sensor Pointer to mlx90614_t structure
 * @param temp_c Pointer to store temperature in Celsius
 * @return true if read successful, false otherwise
 */
bool mlx90614_read_ambient_temp(mlx90614_t *sensor, float *temp_c);

/**
 * @brief Read both object and ambient temperatures
 * 
 * @param sensor Pointer to mlx90614_t structure
 * @param object_temp_c Pointer to store object temperature in Celsius
 * @param ambient_temp_c Pointer to store ambient temperature in Celsius
 * @return true if read successful, false otherwise
 */
bool mlx90614_read_both_temps(mlx90614_t *sensor, float *object_temp_c, float *ambient_temp_c);

/**
 * @brief Read object temperature in Fahrenheit
 * 
 * @param sensor Pointer to mlx90614_t structure
 * @param temp_f Pointer to store temperature in Fahrenheit
 * @return true if read successful, false otherwise
 */
bool mlx90614_read_object_temp_f(mlx90614_t *sensor, float *temp_f);

/**
 * @brief Check if sensor is responding
 * 
 * @param sensor Pointer to mlx90614_t structure
 * @return true if sensor responds, false otherwise
 */
bool mlx90614_is_connected(mlx90614_t *sensor);

/**
 * @brief Read sensor ID (for verification)
 * 
 * @param sensor Pointer to mlx90614_t structure
 * @param id Pointer to store 64-bit ID
 * @return true if read successful, false otherwise
 */
bool mlx90614_read_id(mlx90614_t *sensor, uint64_t *id);

/**
 * @brief Convert Celsius to Fahrenheit
 * 
 * @param celsius Temperature in Celsius
 * @return Temperature in Fahrenheit
 */
float mlx90614_celsius_to_fahrenheit(float celsius);

#endif // MLX90614_H
