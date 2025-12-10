#ifndef MAPPING_H
#define MAPPING_H

#include <stdbool.h>
#include <stdint.h>

#define MAP_MAX_POINTS 4096

typedef struct {
    bool active;
    uint16_t count;
    float angles[MAP_MAX_POINTS];
    uint16_t distances[MAP_MAX_POINTS];
    float temps[MAP_MAX_POINTS];
} env_map_t;

void map_init(env_map_t *map);
void map_add(env_map_t *map, float angle_deg, uint16_t distance_cm, float temp_c);
void map_clear(env_map_t *map);

#endif // MAPPING_H
