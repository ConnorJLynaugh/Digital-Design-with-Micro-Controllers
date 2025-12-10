#include "mapping.h"

void map_init(env_map_t *map) {
    if (!map) return;
    map->active = false;
    map->count = 0;
}

void map_add(env_map_t *map, float angle_deg, uint16_t distance_cm, float temp_c) {
    if (!map || map->count >= MAP_MAX_POINTS) return;
    map->angles[map->count] = angle_deg;
    map->distances[map->count] = distance_cm;
    map->temps[map->count] = temp_c;
    map->count++;
}

void map_clear(env_map_t *map) {
    if (!map) return;
    map->active = false;
    map->count = 0;
}
