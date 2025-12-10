#ifndef _MLX90614CONFIG_H
#define _MLX90614CONFIG_H

#include "hardware/i2c.h"

// MLX90614 is wired to I2C1 on the Pico
#define _MLX90614_I2C         i2c1
// 8-bit address (0x5A << 1) used by the driver for PEC calculations
#define _MLX90614_I2C_ADDRESS 0xB4
// FreeRTOS is not used on the Pico build
#define _MLX90614_FREERTOS    0

#endif
