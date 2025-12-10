#include <stdio.h>
#include <math.h>
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
#include "mapping.h"
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

// Control modes (local only; WiFi page no longer toggles)
typedef enum {
    ROBOT_MODE_WIFI_CONTROL = 0,
    ROBOT_MODE_SCAN_APPROACH = 1,
} robot_mode_t;

// Sensors
tfluna_t lidar_sensor;
fix15 acceleration[3], gyro[3];

// Define the global sensor data
sensor_data_t g_sensor_data = {0};
static bool g_found_lidar = false;
static bool g_found_imu = false;
static bool g_found_temp = false;
static absolute_time_t g_last_sensor_update;
static inline void refresh_sensor_data(void);
volatile robot_mode_t g_robot_mode = ROBOT_MODE_WIFI_CONTROL;
static float g_gyro_heading = 0.0f;
static uint64_t g_last_gyro_time = 0;
#define GYRO_Z_BIAS_DEFAULT 2.5f
static float g_gyro_bias = GYRO_Z_BIAS_DEFAULT;
env_map_t g_map;
static uint64_t g_last_collect = 0;
static uint64_t g_last_rotate = 0;
static float g_start_heading = 0.0f;
static bool g_sweep_started = false;
static float g_prev_heading = 0.0f;
static float g_rotated_sum = 0.0f;

// Simple helpers to gather cleaner measurements during a sweep
static uint16_t median_u16(uint16_t *vals, int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (vals[j] < vals[i]) {
                uint16_t tmp = vals[i];
                vals[i] = vals[j];
                vals[j] = tmp;
            }
        }
    }
    return vals[n / 2];
}

static uint16_t read_lidar_filtered(int samples) {
    if (!g_found_lidar) return 0;
    uint16_t buf[8];
    int count = 0;
    for (int i = 0; i < samples && count < 8; i++) {
        tfluna_data_t data;
        if (tfluna_read_data(&lidar_sensor, &data) && data.valid && data.distance > 0) {
            buf[count++] = data.distance;
        }
        sleep_ms(2);
    }
    if (count == 0) return 0;
    return median_u16(buf, count);
}

static float read_temp_filtered(int samples) {
    if (!g_found_temp) return 0.0f;
    float sum = 0.0f;
    int count = 0;
    for (int i = 0; i < samples; i++) {
        float obj, amb;
        if (mlx90614_read_both_temps(&obj, &amb)) {
            sum += obj;
            count++;
        }
        sleep_ms(2);
    }
    return count ? (sum / count) : 0.0f;
}

static void sweep_collect_sample_if_due(void) {
    if (!g_map.active) return;

    uint64_t now_us = time_us_64();
    if ((now_us - g_last_collect) <= 20000) return; // ~50 Hz

    uint16_t dist = read_lidar_filtered(3);
    float temp = read_temp_filtered(2);

    map_add(&g_map, g_gyro_heading, dist, temp);
    printf("Angle: %6.1f° | Distance: %4d cm | Temp: %5.1f°C | Samples: %u\n",
           g_gyro_heading, dist, temp, g_map.count);
    g_last_collect = now_us;
}

static void dump_map_csv(void) {
    printf("angle_deg,distance_cm,temp_c\n");
    for (uint16_t i = 0; i < g_map.count; i++) {
        printf("%.1f,%u,%.1f\n", g_map.angles[i], g_map.distances[i], g_map.temps[i]);
    }
}

static void dump_map_csv_block(void) {
    if (g_map.count == 0) {
        printf("# sweep_csv_begin (no samples)\n");
        printf("# sweep_csv_end\n");
        return;
    }

    printf("\n# sweep_csv_begin\n");
    dump_map_csv();
    printf("# sweep_csv_end\n\n");
}

static const char* mode_to_string(robot_mode_t mode) {
    return (mode == ROBOT_MODE_SCAN_APPROACH) ? "Scan/Approach" : "WiFi Control";
}

void handle_scan_approach_mode(void) {

    float temps[9] = {0.0f};
    float highest_temp;
    int hottest_index;
    int turn_counter = 0;

    printf("Going CounterClockwise\n");
    for (int i = 0; i < 4; i++) {
        ccw();
    }

    // Take first measurement
    sleep_ms(200);
    refresh_sensor_data();
    temps[turn_counter] = g_sensor_data.temp_f;
    highest_temp = temps[turn_counter];
    hottest_index = turn_counter;

    printf("Going Clockwise\n");
    for (int i = 0; i < 8; i++) {
        cw();
        sleep_ms(200);
        refresh_sensor_data();
        turn_counter++;
        temps[turn_counter] = g_sensor_data.temp_f;
        if (temps[turn_counter] > highest_temp) {
            highest_temp = temps[turn_counter];
            hottest_index = turn_counter;
        }
    }

    // Return to hottest pose
    while (turn_counter != hottest_index) {
        ccw();
        turn_counter--;
    }

    refresh_sensor_data();
    float dist = g_sensor_data.distance;

    // Do Something
    while (dist >= 10.0) {
        backward();
        refresh_sensor_data();
        dist = g_sensor_data.distance;
    }
    hi();

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

    // Update heading from integrated gyro
    if (found_imu) {
        g_sensor_data.heading = g_gyro_heading;
        g_sensor_data.heading_valid = true;
    } else {
        g_sensor_data.heading_valid = false;
    }
}

// Integrate gyro Z to maintain heading with bias and deadband compensation
static inline void update_heading_from_gyro(void) {
    if (!g_found_imu) return;

    uint64_t now = time_us_64();
    if (g_last_gyro_time == 0) {
        g_last_gyro_time = now;
        return;
    }

    float dt = (now - g_last_gyro_time) / 1000000.0f;
    if (dt < 0.002f) return; // integrate even on short intervals

    mpu6050_read_raw(acceleration, gyro);
    float gz = fix2float15(gyro[2]);
    gz += g_gyro_bias;
    if (fabsf(gz) < 0.2f) gz = 0.0f; // tighter deadband

    g_gyro_heading += gz * dt;
    while (g_gyro_heading < 0) g_gyro_heading += 360.0f;
    while (g_gyro_heading >= 360.0f) g_gyro_heading -= 360.0f;

    g_last_gyro_time = now;
}

// Simple bias calibration: average gyro Z while stationary
static void calibrate_gyro_bias(void) {
    if (!g_found_imu) return;
    const int samples = 200;
    float sum = 0.0f;
    for (int i = 0; i < samples; i++) {
        mpu6050_read_raw(acceleration, gyro);
        sum += fix2float15(gyro[2]);
        sleep_ms(2);
    }
    g_gyro_bias = -(sum / samples);
    printf("Gyro Z bias calibrated to %.3f °/s\n", g_gyro_bias);
}

// Helper
static inline void refresh_sensor_data(void) {
    // Keep heading fresh even when the main loop is blocked
    update_heading_from_gyro();

    // Faster sensor refresh for snappier web updates (~100 ms)
    if (absolute_time_diff_us(g_last_sensor_update, get_absolute_time()) > 100000) {
        update_sensor_data(g_found_imu, g_found_lidar, g_found_temp);
        g_last_sensor_update = get_absolute_time();
    } else if (g_found_imu) {
        // Still publish the latest heading even if we skip a full sensor refresh
        g_sensor_data.heading = g_gyro_heading;
        g_sensor_data.heading_valid = true;
    }
}

// Background hook used by movement_library cooperative sleeps
void robot_background_tick(void) {
    update_heading_from_gyro();
    refresh_sensor_data();
    // Keep collecting sweep samples even while movement functions are running
    sweep_collect_sample_if_due();
}

void start_mapping_sweep(void) {
    if (g_found_imu && g_found_lidar && g_found_temp) {
        printf("Starting 360° sweep...\n");
        printf("CSV will print at completion between '# sweep_csv_begin' and '# sweep_csv_end'.\n");
        calibrate_gyro_bias();
        g_gyro_heading = 0.0f;
        g_last_gyro_time = time_us_64();
        g_map.active = true;
        g_sweep_started = false;
        g_last_collect = g_last_rotate = 0;
        g_prev_heading = g_gyro_heading;
        g_rotated_sum = 0.0f;
    } else {
        printf("⚠️  Cannot start sweep - missing sensors:\n");
        if (!g_found_imu) printf("  - IMU not available\n");
        if (!g_found_lidar) printf("  - LiDAR not available\n");
        if (!g_found_temp) printf("  - Temperature sensor not available\n");
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
    bool found_pca = false;
    bool mlx_ping = false;

    for (int addr = 0; addr < 128; addr++) {
        uint8_t data;
        int ret = i2c_read_timeout_us(I2C_PORT, addr, &data, 1, false, 10000);
        if (ret > 0) {
            printf("  Found device at address 0x%02X", addr);
            if (addr == TFLUNA_DEFAULT_ADDR) {
                printf(" (TF-Luna LiDAR)");
                g_found_lidar = true;
            } else if (addr == 0x68) {
                printf(" (MPU6050 IMU)");
                g_found_imu = true;
            } else if (addr == PCA9685_DEFAULT_ADDRESS) {
                printf(" (PCA9685 Servo Driver)");
                found_pca = true;
            } else if (addr == MLX90614_DEFAULT_ADDR) {
                printf(" (MLX90614 IR Thermometer)");
                g_found_temp = true;
            }
            printf("\n");
            found = true;
        }
    }

    mlx_ping = mlx90614_ping();
    if (mlx_ping && !g_found_temp) {
        printf("  MLX90614 responded to ping at 0x%02X\n", MLX90614_DEFAULT_ADDR);
        g_found_temp = true;
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
                    g_found_temp = true;
                }
                printf("\n");
            }
        }
        if (!g_found_temp) {
            printf("  No devices found on temp sensor I2C bus\n");
        }
        printf("\n");
    } else if (!g_found_temp) {
        printf("⚠️  MLX90614 not found on shared I2C bus (ping: %s)\n\n", mlx_ping ? "responded" : "no response");
    } else {
        printf("✓ MLX90614 detected on shared I2C bus at 0x%02X (ping: %s)\n\n", MLX90614_DEFAULT_ADDR, mlx_ping ? "responded" : "no response");
    }
    
    // Initialize PCA9685
    printf("Initializing PCA9685...\n");
    pca9685_init(&pwm, I2C_PORT, PCA9685_DEFAULT_ADDRESS);
    pca9685_set_pwm_freq(&pwm, 60.0f);

    // Initialize TF-Luna LiDAR
    if (g_found_lidar) {
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
            g_found_lidar = false;
        }
    } else {
        printf("⚠️  TF-Luna not found - LiDAR features disabled\n\n");
    }

    // Initialize MLX90614 IR Thermometer (try even if scan missed it)
    printf("Initializing MLX90614 IR Thermometer%s...\n", g_found_temp ? "" : " (not detected in scan, attempting anyway)");
    if (mlx90614_init()) {
        printf("✓ MLX90614 initialized successfully!\n");
        g_found_temp = true;

        float obj_temp, amb_temp;
        if (mlx90614_read_both_temps(&obj_temp, &amb_temp)) {
            printf("  Object temp: %.2f°C | Ambient: %.2f°C\n\n", obj_temp, amb_temp);
        }
    } else {
        printf("✗ Failed to initialize MLX90614\n\n");
        g_found_temp = false;
    }

    // Initialize MPU6050 IMU
    if (g_found_imu) {
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
        g_gyro_heading = 0.0f;
        g_last_gyro_time = time_us_64();
    } else {
        printf("⚠️  MPU6050 not found - IMU features disabled\n\n");
    }

    // Initialize mapping storage
    map_init(&g_map);
    
    printf("Please wait for 10 seconds...\n");
    stand_up();
    //setup_servos();
    printf("I am Ready!\n");
    
    //print_menu();
    
    bool running = true;
    robot_mode_t last_mode = g_robot_mode;
    g_last_sensor_update = get_absolute_time();
    
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
                case 'n':
                case 'N':
                    if (g_found_imu && g_found_lidar && g_found_temp) {
                        start_mapping_sweep();
                    } else {
                        printf("⚠️  Cannot start sweep - missing sensors:\n");
                        if (!g_found_imu) printf("  - IMU not available\n");
                        if (!g_found_lidar) printf("  - LiDAR not available\n");
                        if (!g_found_temp) printf("  - Temperature sensor not available\n");
                    }
                    break;

                case 'r':
                case 'R':
                    printf("\n========== SENSOR READING ==========\n");
                    if (g_found_lidar) {
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

                    if (g_found_temp) {
                        float obj_temp, amb_temp;
                        if (mlx90614_read_both_temps(&obj_temp, &amb_temp)) {
                            printf("Object Temp:  %.2f°C (%.1f°F)\n",
                                   obj_temp, mlx90614_celsius_to_fahrenheit(obj_temp));
                            printf("Ambient Temp: %.2f°C (%.1f°F)\n",
                                   amb_temp, mlx90614_celsius_to_fahrenheit(amb_temp));

                            if (g_found_lidar) {
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
                    if (g_found_imu) {
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
                        printf("   Heading (Z): %7.2f°\n", g_gyro_heading);
                        
                        printf("=================================\n\n");
                    } else {
                        printf("⚠️  IMU not available\n\n");
                    }
                    break;

                case 'z':
                case 'Z':
                    if (g_found_imu) {
                        g_gyro_heading = 0.0f;
                        g_last_gyro_time = time_us_64();
                        printf("Heading reset to 0°\n");
                    } else {
                        printf("⚠️  IMU not available\n");
                    }
                    break;
                    
                default:
                    // Ignore other characters
                    break;
            }
        }

        // Maintain heading integration
        update_heading_from_gyro();

        // Update sensor data periodically (every 500ms) for WiFi display
        refresh_sensor_data();

        // Mapping sweep handling (non-blocking)
        if (g_map.active) {
            uint64_t now_us = time_us_64();

            if (!g_sweep_started) {
                g_start_heading = g_gyro_heading;
                g_sweep_started = true;
                g_last_collect = now_us;
                g_last_rotate = now_us;
                printf("Starting 360° sweep from %.1f°\n", g_start_heading);
                g_map.count = 0;
                g_prev_heading = g_gyro_heading;
                g_rotated_sum = 0.0f;
            } else {
                // Accumulate rotation based on heading change (favor CCW; ignore backward drift)
                float delta = g_gyro_heading - g_prev_heading;
                if (delta < -180.0f) delta += 360.0f;
                else if (delta > 180.0f) delta -= 360.0f;
                if (delta > 0.0f) {
                    g_rotated_sum += delta;
                    if (g_rotated_sum > 360.0f) g_rotated_sum = 360.0f;
                }
                g_prev_heading = g_gyro_heading;
            }

            // Collect data (also runs inside robot_background_tick during motion)
            sweep_collect_sample_if_due();

            // Rotate every 2s until 360 covered
            float angle_rotated = g_rotated_sum;

            if (angle_rotated >= 360.0f) {
                g_map.active = false;
                g_sweep_started = false;
                dump_map_csv_block();
                printf("360° sweep complete! Samples: %u\n", g_map.count);
            } else if ((now_us - g_last_rotate) > 2000000) {
                ccw();
                g_last_rotate = now_us;
                printf("Heading: %.1f° (rotated %.1f° of 360°)\n", g_gyro_heading, angle_rotated);
            }
        } else {
            g_sweep_started = false;
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
