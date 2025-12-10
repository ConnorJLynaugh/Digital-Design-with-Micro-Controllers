#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdbool.h>

// Shared sensor data structure
typedef struct {
    float pitch;
    float temp_c;
    float temp_f;
    int distance;
    bool pitch_valid;
    bool temp_valid;
    bool distance_valid;
} sensor_data_t;

// Global sensor data (defined in main.c)
extern sensor_data_t g_sensor_data;

#endif // SENSOR_DATA_H
