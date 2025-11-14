#include "pca9685.h"
#include <math.h>
#include <stdint.h>  // ← ADD THIS LINE


static void pca9685_write_byte(PCA9685 *pca, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(pca->i2c, pca->address, buf, 2, false);
}

static uint8_t pca9685_read_byte(PCA9685 *pca, uint8_t reg) {
    uint8_t value;
    i2c_write_blocking(pca->i2c, pca->address, &reg, 1, true);
    i2c_read_blocking(pca->i2c, pca->address, &value, 1, false);
    return value;
}

void pca9685_init(PCA9685 *pca, i2c_inst_t *i2c, uint8_t address) {
    pca->i2c = i2c;
    pca->address = address;
    pca9685_reset(pca);
}

void pca9685_reset(PCA9685 *pca) {
    pca9685_write_byte(pca, PCA9685_MODE1, MODE1_RESTART);
    sleep_ms(10);
}

void pca9685_set_pwm_freq(PCA9685 *pca, float freq) {
    // Calculate prescale value
    // prescale = round(25MHz / (4096 * freq)) - 1
    float prescaleval = 25000000.0f;
    prescaleval /= 4096.0f;
    prescaleval /= freq;
    prescaleval -= 1.0f;
    
    uint8_t prescale = (uint8_t)(prescaleval + 0.5f);
    
    uint8_t oldmode = pca9685_read_byte(pca, PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | MODE1_SLEEP; // Sleep mode
    
    pca9685_write_byte(pca, PCA9685_MODE1, newmode); // Go to sleep
    pca9685_write_byte(pca, PCA9685_PRESCALE, prescale); // Set prescale
    pca9685_write_byte(pca, PCA9685_MODE1, oldmode);
    sleep_ms(5);
    pca9685_write_byte(pca, PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

void pca9685_set_pwm(PCA9685 *pca, uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t buf[5];
    buf[0] = PCA9685_LED0_ON_L + 4 * channel;
    buf[1] = on & 0xFF;
    buf[2] = on >> 8;
    buf[3] = off & 0xFF;
    buf[4] = off >> 8;
    
    i2c_write_blocking(pca->i2c, pca->address, buf, 5, false);
}
