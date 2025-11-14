#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/error.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pca9685.h"
#include "movement_library.h"

// I2C configuration
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C_FREQ 100000

void print_menu(void) {
    printf("\n=== Quadruped Robot Control Menu ===\n");
    printf("Movement Commands:\n");
    printf("  w - Forward\n");
    printf("  s - Backward\n");
    printf("  a - Counterclockwise\n");
    printf("  d - Clockwise\n");
    printf("  q - Left\n");
    printf("  e - Right\n");
    printf("  t - Creep Forward\n");
    printf("  g - Creep Backward\n");
    printf("\nSpecial Actions:\n");
    printf("  h - Say Hi\n");
    printf("  c - Shuffle\n");
    printf("  v - Humping\n");
    printf("  b - Squads\n");
    printf("\nPositions:\n");
    printf("  x - X Position (neutral)\n");
    printf("  1-4 - Shift to position 1-4\n");
    printf("  i - Sit\n");
    printf("  u - Stand Up\n");
    printf("  l - Legs Up\n");
    printf("\nOther:\n");
    printf("  m - Show this menu\n");
    printf("  p - Exit program\n");
    printf("====================================\n\n");
}

int main(void) {
    // Initialize stdio for USB serial
    stdio_init_all();
    
    // Wait for USB serial connection
    sleep_ms(2000);
    
    printf("\n\n=== Quadruped Robot Starting ===\n");
    printf("Initializing I2C...\n");
    // Initialize I2C
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Initialize PCA9685
    printf("Initializing PCA9685...\n");
    pca9685_init(&pwm, I2C_PORT, PCA9685_DEFAULT_ADDRESS);
    pca9685_set_pwm_freq(&pwm, 60.0f);
    
    printf("Please wait for 10 seconds...\n");
    stand_up();
    printf("I am Ready!\n");
    
    print_menu();
    
    bool running = true;
    
    while (running) {
        // Get character from serial input
        int c = getchar_timeout_us(0);
        
        if (c != PICO_ERROR_TIMEOUT) {
            switch(c) {
                case 'w':
                case 'W':
                    printf("Going Forward\n");
                    forward();
                    break;
                    
                case 's':
                case 'S':
                    printf("Going Backward\n");
                    backward();
                    break;
                    
                case 'e':
                case 'E':
                    printf("Going Right\n");
                    right();
                    break;
                    
                case 'q':
                case 'Q':
                    printf("Going Left\n");
                    left();
                    break;
                    
                case 'a':
                case 'A':
                    printf("Going Counterclockwise\n");
                    ccw();
                    break;
                    
                case 'd':
                case 'D':
                    printf("Going Clockwise\n");
                    cw();
                    break;
                    
                case 't':
                case 'T':
                    printf("Creep Forward\n");
                    leg_position_fb(1.0f, 4, 0);
                    sleep_ms(100);
                    c_f();
                    break;
                    
                case 'g':
                case 'G':
                    printf("Creep Backward\n");
                    leg_position_fb(1.0f, 2, 0);
                    sleep_ms(100);
                    c_b();
                    break;
                    
                case 'h':
                case 'H':
                    printf("Saying Hi\n");
                    hi();
                    break;
                    
                case 'c':
                case 'C':
                    printf("Shuffling\n");
                    shuffle();
                    break;
                    
                case 'v':
                case 'V':
                    printf("Humping\n");
                    humping();
                    break;
                    
                case 'b':
                case 'B':
                    printf("Squading from all sides\n");
                    squads();
                    break;
                    
                case 'x':
                case 'X':
                    printf("X Position\n");
                    xposition();
                    break;
                    
                case '1':
                    printf("Shift to position 1\n");
                    shift_to(1);
                    break;
                    
                case '2':
                    printf("Shift to position 2\n");
                    shift_to(2);
                    break;
                    
                case '3':
                    printf("Shift to position 3\n");
                    shift_to(3);
                    break;
                    
                case '4':
                    printf("Shift to position 4\n");
                    shift_to(4);
                    break;
                    
                case 'i':
                case 'I':
                    printf("Sitting\n");
                    sit();
                    break;
                    
                case 'u':
                case 'U':
                    printf("Standing up\n");
                    stand_up();
                    break;
                    
                case 'l':
                case 'L':
                    printf("Legs up\n");
                    legs_up();
                    break;
                    
                case 'm':
                case 'M':
                    print_menu();
                    break;
                    
                case 'p':
                case 'P':
                    printf("Ending Program\n");
                    sit();
                    sleep_ms(1000);
                    legs_up();
                    printf("Please! I don't want to go\n");
                    running = false;
                    break;
                    
                default:
                    // Ignore other characters
                    break;
            }
        }
        
        // Small delay to prevent CPU hogging
        sleep_ms(10);
    }
    
    printf("Goodbye\n");
    return 0;
}
