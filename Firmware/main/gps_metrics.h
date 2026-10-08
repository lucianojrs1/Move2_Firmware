#ifndef GPS_METRICS_H
#define GPS_METRICS_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    double trip_distance_m;
    uint64_t moving_ms, stopped_ms;
    float speed_kmh, course_deg, altitude_m, hdop;
    float average_speed_kmh, max_speed_kmh;
    uint32_t first_fix_ms;
    bool speed_valid, course_valid, altitude_valid, hdop_valid;
    bool first_fix_valid, moving, tracking;
} gps_metrics_t;
#endif
