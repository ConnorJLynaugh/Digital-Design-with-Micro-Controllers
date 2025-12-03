#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/error.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "pca9685.h"
#include "movement_library.h"
#include "tfluna_i2c.h"
#include "mlx90614.h"

#include "wifi.h"
#include "pico/cyw43_arch.h"

// I2C configuration
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C_FREQ 100000

// Sensors
tfluna_t lidar_sensor;
mlx90614_t temp_sensor;

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
    printf("\nLiDAR / Temp Commands:\n");
    printf("  r - Read Distance & Temperature\n");
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

    ////////////////// WIFI //////////////////
    if (!wifi_ap_start("quadruped_ap", "password")) {
        printf("Failed to start WiFi AP\n");
    } else {
        printf("WiFi AP started successfully\n");
        // Keep onboard LED on while the Pico W is running
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }
    ////////////////// WIFI END //////////////////
    
    printf("\n\n=== Quadruped Robot Starting ===\n");
    printf("I2C Configuration:\n");
    printf("  SDA Pin: GPIO %d\n", I2C_SDA_PIN);
    printf("  SCL Pin: GPIO %d\n", I2C_SCL_PIN);
    printf("  I2C Frequency: %d Hz\n", I2C_FREQ);
    printf("  PCA9685 Address: 0x%02X\n\n", PCA9685_DEFAULT_ADDRESS);
    printf("  TF-Luna Address: 0x%02X\n\n", TFLUNA_DEFAULT_ADDR);
    printf("  MLX90614 Address: 0x%02X\n\n", MLX90614_DEFAULT_ADDR);
    
    printf("Initializing I2C...\n");
    // Initialize I2C
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Scan I2C bus
    printf("Scanning I2C bus (this may take a moment)...\n");
    bool found = false;
    bool found_lidar = false;
    bool found_temp = false;
    for (int addr = 0; addr < 128; addr++) {
        uint8_t data;
        int ret = i2c_read_timeout_us(I2C_PORT, addr, &data, 1, false, 10000);
        if (ret > 0) {
            printf("  Found device at address 0x%02X", addr);
            if (addr == TFLUNA_DEFAULT_ADDR) {
                printf(" (TF-Luna LiDAR)");
                found_lidar = true;
            } else if (addr == MLX90614_DEFAULT_ADDR) {
                printf(" (MLX90614 IR Thermometer)");
                found_temp = true;
            }
            printf("\n");
            found = true;
        }
    }
    if (!found) {
        printf("  No I2C devices found! Check wiring and power.\n");
    }
    
    // Initialize PCA9685
    printf("Initializing PCA9685...\n");
    pca9685_init(&pwm, I2C_PORT, PCA9685_DEFAULT_ADDRESS);
    pca9685_set_pwm_freq(&pwm, 60.0f);

    // Initialize TF-Luna LiDAR
    if (found_lidar) {
        printf("Initializing TF-Luna LiDAR...\n");
        if (tfluna_init(&lidar_sensor, I2C_PORT, TFLUNA_DEFAULT_ADDR)) {
            printf("✓ TF-Luna initialized successfully!\n");
            tfluna_set_frequency(&lidar_sensor, 100);
            printf("  Measurement rate: 100 Hz\n");

            tfluna_data_t data;
            if (tfluna_read_data(&lidar_sensor, &data) && data.valid) {
                printf("  Initial distance: %d cm\n", data.distance);
                printf("  Sensor temp: %.2f°C\n\n", data.temperature / 100.0f);
            }
        } else {
            printf("✗ Failed to initialize TF-Luna\n\n");
            found_lidar = false;
        }
    } else {
        printf("⚠️  TF-Luna not found - LiDAR features disabled\n");
        printf("   Make sure Pin 5 (Mode) is connected to GND!\n\n");
    }

    // Initialize MLX90614 IR Thermometer
    if (found_temp) {
        printf("Initializing MLX90614 IR Thermometer...\n");
        if (mlx90614_init(&temp_sensor, I2C_PORT, MLX90614_DEFAULT_ADDR)) {
            printf("✓ MLX90614 initialized successfully!\n");

            float obj_temp, amb_temp;
            if (mlx90614_read_both_temps(&temp_sensor, &obj_temp, &amb_temp)) {
                printf("  Object temp: %.2f°C | Ambient: %.2f°C\n", obj_temp, amb_temp);
            }
        } else {
            printf("✗ Failed to initialize MLX90614\n");
            found_temp = false;
        }
    } else {
        printf("⚠️  MLX90614 not found - temperature measurement disabled\n");
    }
    printf("\n");
    
    printf("Please wait for 10 seconds...\n");
    stand_up();
    //setup_servos();
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

                case 'r':
                case 'R':
                    printf("\n========== SENSOR READING ==========\n");
                    if (found_lidar) {
                        tfluna_data_t lidar_data;
                        if (tfluna_read_data(&lidar_sensor, &lidar_data)) {
                            if (lidar_data.valid) {
                                printf("Distance:     %d cm\n", lidar_data.distance);
                                printf("   Signal:       %d\n", lidar_data.amplitude);
                                printf("   LiDAR Temp:   %.2f°C\n", lidar_data.temperature / 100.0f);
                            } else {
                                printf("Distance:     No target detected\n");
                            }
                        } else {
                            printf("Distance:     Read error\n");
                        }
                    } else {
                        printf("Distance:     LiDAR not available\n");
                    }

                    printf("\n");

                    if (found_temp) {
                        float obj_temp, amb_temp;
                        if (mlx90614_read_both_temps(&temp_sensor, &obj_temp, &amb_temp)) {
                            printf("Object Temp:  %.2f°C (%.1f°F)\n",
                                   obj_temp, mlx90614_celsius_to_fahrenheit(obj_temp));
                            printf("Ambient Temp: %.2f°C (%.1f°F)\n",
                                   amb_temp, mlx90614_celsius_to_fahrenheit(amb_temp));

                            if (found_lidar) {
                                tfluna_data_t lidar_data;
                                if (tfluna_read_data(&lidar_sensor, &lidar_data) && lidar_data.valid) {
                                    printf("\nAnalysis:\n");
                                    printf("   Object at %d cm\n", lidar_data.distance);
                                }
                            }
                        } else {
                            printf("Object Temp:  Read error\n");
                        }
                    } else {
                        printf("Object Temp:  Thermometer not available\n");
                    }

                    printf("====================================\n\n");
                    break;
                    
                default:
                    // Ignore other characters
                    break;
            }
        }

        // Wifi
        wifi_ap_background();
        
        // Small delay to prevent CPU hogging
        sleep_ms(10);
    }
    
    printf("Goodbye\n");
    return 0;
}
