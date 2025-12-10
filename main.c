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
#include "mpu6050.h"

#include "wifi.h"
#include "sensor_data.h"
#include "pico/cyw43_arch.h"

// I2C configuration for main devices
#define I2C_PORT i2c1
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
#define I2C_FREQ 100000

// MLX90614 shares the main bus on I2C1
#define I2C_TEMP_PORT I2C_PORT
#define I2C_TEMP_SDA_PIN I2C_SDA_PIN
#define I2C_TEMP_SCL_PIN I2C_SCL_PIN

// Sensors
tfluna_t lidar_sensor;
fix15 acceleration[3], gyro[3];

// Define the global sensor data
sensor_data_t g_sensor_data = {0};
volatile robot_mode_t g_robot_mode = ROBOT_MODE_WIFI_CONTROL;

static const char* mode_to_string(robot_mode_t mode) {
    return (mode == ROBOT_MODE_SCAN_APPROACH) ? "Scan/Approach" : "WiFi Control";
}

static void handle_scan_approach_mode(void) {

}

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
    printf("\nSensor Commands:\n");
    printf("  r - Read Distance & Temperature\n");
    printf("  y - Read IMU (Accelerometer & Gyroscope)\n");
    printf("\nOther:\n");
    printf("  m - Show this menu\n");
    printf("  p - Exit program\n");
    printf("====================================\n\n");
}

void update_sensor_data(bool found_imu, bool found_lidar, bool found_temp) {
    // Update distance
    if (found_lidar) {
        tfluna_data_t lidar_data;
        if (tfluna_read_data(&lidar_sensor, &lidar_data) && lidar_data.valid) {
            g_sensor_data.distance = lidar_data.distance;
            g_sensor_data.distance_valid = true;
        } else {
            g_sensor_data.distance_valid = false;
        }
    } else {
        g_sensor_data.distance_valid = false;
    }

    // Update temperature
    if (found_temp) {
        float obj_temp_c, amb_temp_c;
        if (mlx90614_read_both_temps(&obj_temp_c, &amb_temp_c)) {
            g_sensor_data.temp_c = obj_temp_c;
            g_sensor_data.temp_f = mlx90614_celsius_to_fahrenheit(obj_temp_c);
            g_sensor_data.temp_valid = true;
        } else {
            g_sensor_data.temp_valid = false;
        }
    } else {
        g_sensor_data.temp_valid = false;
    }

    // Update pitch
    if (found_imu) {
        // Local arrays to ensure fresh data
        fix15 local_accel[3], local_gyro[3];
        mpu6050_read_raw(local_accel, local_gyro);
        
        float ax = fix2float15(local_accel[0]);
        float ay = fix2float15(local_accel[1]);
        float az = fix2float15(local_accel[2]);
        
        // Calculate pitch (rotation around Y-axis)
        g_sensor_data.pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / 3.14159f;
        g_sensor_data.pitch_valid = true;
    } else {
        g_sensor_data.pitch_valid = false;
    }
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
    printf("  Main I2C (GPIO 2/3):\n");
    printf("    SDA Pin: GPIO %d, SCL Pin: GPIO %d\n", I2C_SDA_PIN, I2C_SCL_PIN);
    printf("    PCA9685: 0x%02X, TF-Luna: 0x%02X, MPU6050: 0x68\n", 
           PCA9685_DEFAULT_ADDRESS, TFLUNA_DEFAULT_ADDR);
    printf("  Temp Sensor I2C:\n");
    printf("    SDA Pin: GPIO %d, SCL Pin: GPIO %d\n", I2C_TEMP_SDA_PIN, I2C_TEMP_SCL_PIN);
    printf("    MLX90614: 0x%02X\n\n", MLX90614_DEFAULT_ADDR);

    bool temp_shares_main_bus = (I2C_TEMP_PORT == I2C_PORT) &&
                                (I2C_TEMP_SDA_PIN == I2C_SDA_PIN) &&
                                (I2C_TEMP_SCL_PIN == I2C_SCL_PIN);

    printf("Initializing main I2C (GPIO %d/%d)...\n", I2C_SDA_PIN, I2C_SCL_PIN);
    // Initialize main I2C
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    if (!temp_shares_main_bus) {
        printf("Initializing temp sensor I2C (GPIO %d/%d)...\n", I2C_TEMP_SDA_PIN, I2C_TEMP_SCL_PIN);
        // Initialize temp sensor I2C
        i2c_init(I2C_TEMP_PORT, I2C_FREQ);
        gpio_set_function(I2C_TEMP_SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(I2C_TEMP_SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_TEMP_SDA_PIN);
        gpio_pull_up(I2C_TEMP_SCL_PIN);
    } else {
        printf("Temp sensor shares the main I2C bus.\n");
    }
    
    // Scan main I2C bus
    printf("\nScanning main I2C bus (GPIO 2/3)...\n");
    bool found = false;
    bool found_lidar = false;
    bool found_imu = false;
    bool found_pca = false;
    bool found_temp = false;
    bool mlx_ping = false;

    for (int addr = 0; addr < 128; addr++) {
        uint8_t data;
        int ret = i2c_read_timeout_us(I2C_PORT, addr, &data, 1, false, 10000);
        if (ret > 0) {
            printf("  Found device at address 0x%02X", addr);
            if (addr == TFLUNA_DEFAULT_ADDR) {
                printf(" (TF-Luna LiDAR)");
                found_lidar = true;
            } else if (addr == 0x68) {
                printf(" (MPU6050 IMU)");
                found_imu = true;
            } else if (addr == PCA9685_DEFAULT_ADDRESS) {
                printf(" (PCA9685 Servo Driver)");
                found_pca = true;
            } else if (addr == MLX90614_DEFAULT_ADDR) {
                printf(" (MLX90614 IR Thermometer)");
                found_temp = true;
            }
            printf("\n");
            found = true;
        }
    }

    mlx_ping = mlx90614_ping();
    if (mlx_ping && !found_temp) {
        printf("  MLX90614 responded to ping at 0x%02X\n", MLX90614_DEFAULT_ADDR);
        found_temp = true;
    }

    if (!found) {
        printf("  No devices found on main I2C bus\n");
    }

    if (!found_pca) {
        printf("⚠️  PCA9685 NOT detected during I2C scan at 0x%02X\n", PCA9685_DEFAULT_ADDRESS);
    } else {
        printf("✓ PCA9685 detected on the I2C bus at 0x%02X\n", PCA9685_DEFAULT_ADDRESS);
    }

    if (!temp_shares_main_bus) {
        // Scan temp sensor I2C bus
        printf("\nScanning temp sensor I2C bus (GPIO %d/%d)...\n", I2C_TEMP_SDA_PIN, I2C_TEMP_SCL_PIN);
        for (int addr = 0; addr < 128; addr++) {
            uint8_t data;
            int ret = i2c_read_timeout_us(I2C_TEMP_PORT, addr, &data, 1, false, 10000);
            if (ret > 0) {
                printf("  Found device at address 0x%02X", addr);
                if (addr == MLX90614_DEFAULT_ADDR) {
                    printf(" (MLX90614 IR Thermometer)");
                    found_temp = true;
                }
                printf("\n");
            }
        }
        if (!found_temp) {
            printf("  No devices found on temp sensor I2C bus\n");
        }
        printf("\n");
    } else if (!found_temp) {
        printf("⚠️  MLX90614 not found on shared I2C bus (ping: %s)\n\n", mlx_ping ? "responded" : "no response");
    } else {
        printf("✓ MLX90614 detected on shared I2C bus at 0x%02X (ping: %s)\n\n", MLX90614_DEFAULT_ADDR, mlx_ping ? "responded" : "no response");
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
        printf("⚠️  TF-Luna not found - LiDAR features disabled\n\n");
    }

    // Initialize MLX90614 IR Thermometer (try even if scan missed it)
    printf("Initializing MLX90614 IR Thermometer%s...\n", found_temp ? "" : " (not detected in scan, attempting anyway)");
    if (mlx90614_init()) {
        printf("✓ MLX90614 initialized successfully!\n");
        found_temp = true;

        float obj_temp, amb_temp;
        if (mlx90614_read_both_temps(&obj_temp, &amb_temp)) {
            printf("  Object temp: %.2f°C | Ambient: %.2f°C\n\n", obj_temp, amb_temp);
        }
    } else {
        printf("✗ Failed to initialize MLX90614\n\n");
        found_temp = false;
    }

    // Initialize MPU6050 IMU
    if (found_imu) {
        printf("Initializing MPU6050 IMU...\n");
        mpu6050_reset();
        sleep_ms(100);
        
        // Test read
        mpu6050_read_raw(acceleration, gyro);
        printf("✓ MPU6050 initialized successfully!\n");
        printf("  Accel X: %.2f g, Y: %.2f g, Z: %.2f g\n", 
               fix2float15(acceleration[0]), 
               fix2float15(acceleration[1]), 
               fix2float15(acceleration[2]));
        printf("  Gyro X: %.2f °/s, Y: %.2f °/s, Z: %.2f °/s\n\n",
               fix2float15(gyro[0]), 
               fix2float15(gyro[1]), 
               fix2float15(gyro[2]));
    } else {
        printf("⚠️  MPU6050 not found - IMU features disabled\n\n");
    }
    
    printf("Please wait for 10 seconds...\n");
    stand_up();
    //setup_servos();
    printf("I am Ready!\n");
    
    print_menu();
    
    bool running = true;
    robot_mode_t last_mode = g_robot_mode;
    
    // Timer for sensor updates
    absolute_time_t last_sensor_update = get_absolute_time();
    
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
                        if (mlx90614_read_both_temps(&obj_temp, &amb_temp)) {
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

                case 'y':
                case 'Y':
                    if (found_imu) {
                        printf("\n========== IMU READING ==========\n");
                        
                        // Read raw accelerometer and gyroscope data
                        mpu6050_read_raw(acceleration, gyro);
                        
                        // Display accelerometer data (in g's)
                        printf("Accelerometer (g):\n");
                        printf("   X: %7.3f\n", fix2float15(acceleration[0]));
                        printf("   Y: %7.3f\n", fix2float15(acceleration[1]));
                        printf("   Z: %7.3f\n", fix2float15(acceleration[2]));
                        
                        printf("\n");
                        
                        // Display gyroscope data (in degrees/second)
                        printf("Gyroscope (°/s):\n");
                        printf("   X: %7.2f\n", fix2float15(gyro[0]));
                        printf("   Y: %7.2f\n", fix2float15(gyro[1]));
                        printf("   Z: %7.2f\n", fix2float15(gyro[2]));
                        
                        printf("\n");
                        
                        // Calculate and display roll/pitch
                        float ax = fix2float15(acceleration[0]);
                        float ay = fix2float15(acceleration[1]);
                        float az = fix2float15(acceleration[2]);
                        
                        // Roll (rotation around X-axis)
                        float roll = atan2f(ay, az) * 180.0f / 3.14159f;
                        // Pitch (rotation around Y-axis)
                        float pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / 3.14159f;
                        
                        printf("Orientation:\n");
                        printf("   Roll:  %7.2f°\n", roll);
                        printf("   Pitch: %7.2f°\n", pitch);
                        
                        printf("=================================\n\n");
                    } else {
                        printf("⚠️  IMU not available\n\n");
                    }
                    break;
                    
                default:
                    // Ignore other characters
                    break;
            }
        }

        // Update sensor data periodically (every 500ms) for WiFi display
        if (absolute_time_diff_us(last_sensor_update, get_absolute_time()) > 500000) {
            update_sensor_data(found_imu, found_lidar, found_temp);
            last_sensor_update = get_absolute_time();
        }

        // Mode handling
        if (g_robot_mode != last_mode) {
            printf("Mode changed to %s\n", mode_to_string(g_robot_mode));
            last_mode = g_robot_mode;
        }

        if (g_robot_mode == ROBOT_MODE_SCAN_APPROACH) {
            handle_scan_approach_mode();
        }

        // Wifi background tasks
        wifi_ap_background();
        
        // Small delay to prevent CPU hogging
        sleep_ms(10);
    }
    
    printf("Goodbye\n");
    return 0;
}
