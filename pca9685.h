#ifndef PCA9685_H
#define PCA9685_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>

// PCA9685 Register Addresses
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_SUBADR1 0x02
#define PCA9685_SUBADR2 0x03
#define PCA9685_SUBADR3 0x04
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06
#define PCA9685_LED0_ON_H 0x07
#define PCA9685_LED0_OFF_L 0x08
#define PCA9685_LED0_OFF_H 0x09
#define PCA9685_ALL_LED_ON_L 0xFA
#define PCA9685_ALL_LED_ON_H 0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD

// Mode1 register bits
#define MODE1_RESTART 0x80
#define MODE1_SLEEP 0x10
#define MODE1_ALLCALL 0x01
#define MODE1_AI 0x20

// Default I2C address
#define PCA9685_DEFAULT_ADDRESS 0x40

typedef struct {
    i2c_inst_t *i2c;
    uint8_t address;
} PCA9685;

// Function prototypes
void pca9685_init(PCA9685 *pca, i2c_inst_t *i2c, uint8_t address);
void pca9685_set_pwm_freq(PCA9685 *pca, float freq);
void pca9685_set_pwm(PCA9685 *pca, uint8_t channel, uint16_t on, uint16_t off);
void pca9685_reset(PCA9685 *pca);

#endif // PCA9685_H
