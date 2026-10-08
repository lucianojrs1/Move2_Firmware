#ifndef GPS_NAVIGATION_H
#define GPS_NAVIGATION_H
#include "gps_app.h"
typedef struct {
    uint32_t started_ms, previous_ms;
    double previous_lat, previous_lon, anchor_lat, anchor_lon;
    float previous_speed;
    uint32_t candidate_ms;
    double candidate_lat, candidate_lon;
    bool candidate;
    bool have_previous, have_utc;
    double previous_utc;
    int previous_date;
} gps_navigation_t;
void gps_navigation_init(gps_navigation_t *nav, uint32_t now_ms);
// Only checksum-valid RMC/GGA sentences change navigation. Raw SD capture is independent.
// Returns true for a newly parsed RMC (including no-fix); duplicates return false.
bool gps_navigation_process(gps_navigation_t *nav, gps_status_t *g, const char *line, uint32_t now_ms);
#endif
