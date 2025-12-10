#include <stdbool.h>

// Control modes available for the robot
typedef enum {
    ROBOT_MODE_WIFI_CONTROL = 0,
    ROBOT_MODE_SCAN_APPROACH = 1,
} robot_mode_t;

// Global mode flag (defined in main.c)
extern volatile robot_mode_t g_robot_mode;

bool wifi_ap_start(const char *ap_name, const char *password);
void wifi_ap_background(void);
