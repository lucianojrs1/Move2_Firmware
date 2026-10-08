#include "gps_format.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

gps_position_state_t gps_position_state(const gps_status_t *g, uint32_t now_ms)
{
    if (!g || !g->uart_seen) return GPS_NO_DATA;
    if (!g->fix || !g->location_valid || !isfinite(g->lat) || !isfinite(g->lon) ||
        g->lat < -90 || g->lat > 90 || g->lon < -180 || g->lon > 180) return GPS_NO_FIX;
    if ((uint32_t)(now_ms - g->last_fix_ms) >= GPS_FIX_MAX_AGE_MS) return GPS_STALE;
    return GPS_VALID;
}

const char *gps_position_state_name(gps_position_state_t state)
{
    switch (state) {
    case GPS_NO_DATA: return "sem_dados";
    case GPS_NO_FIX: return "sem_fix";
    case GPS_STALE: return "desatualizado";
    case GPS_VALID: return "valido";
    default: return "desconhecido";
    }
}

void gps_metrics_json(json_writer_t *w, const gps_metrics_t *m, bool fresh, bool gga) {
    json_add(w,",\"speedKmh\":"); json_number(w,m->speed_kmh,fresh && m->speed_valid);
    json_add(w,",\"courseDeg\":"); json_number(w,m->course_deg,fresh && m->course_valid);
    json_add(w,",\"altitudeM\":"); json_number(w,m->altitude_m,fresh && gga && m->altitude_valid);
    json_add(w,",\"hdop\":"); json_number(w,m->hdop,gga && m->hdop_valid);
    json_add(w,",\"tripDistanceM\":"); json_number(w,m->trip_distance_m,true);
    json_add(w,",\"movingTimeS\":"); json_number(w,(double)m->moving_ms/1000,true);
    json_add(w,",\"stoppedTimeS\":"); json_number(w,(double)m->stopped_ms/1000,true);
    json_add(w,",\"averageSpeedKmh\":"); json_number(w,m->average_speed_kmh,true);
    json_add(w,",\"maxSpeedKmh\":"); json_number(w,m->max_speed_kmh,true);
    json_add(w,",\"timeToFirstFixS\":"); json_number(w,m->first_fix_ms/1000.0,m->first_fix_valid);
    json_add(w,",\"tripTracking\":%s,\"moving\":",fresh && gga && m->tracking ? "true":"false");
    if (fresh && gga && m->tracking) json_add(w,m->moving ? "true":"false");
    else json_add(w,"null");
}

size_t gps_format_payload(char *out, size_t size, const gps_status_t *g,
                          uint32_t now_ms, uint64_t epoch_ms)
{
    if (!out || !size || !g) return 0;
    gps_position_state_t state = gps_position_state(g, now_ms);
    char lat[32] = "null", lon[32] = "null", age[24] = "null";
    if (state == GPS_VALID) {
        snprintf(lat, sizeof(lat), "%.6f", g->lat);
        snprintf(lon, sizeof(lon), "%.6f", g->lon);
    }
    if (g->last_fix_ms || g->location_valid)
        snprintf(age, sizeof(age), "%" PRIu32, (uint32_t)(now_ms - g->last_fix_ms));
    json_writer_t w = {.out=out,.cap=size};
    json_add(&w,
                     "{\"sensorId\":\"gps_modulo\",\"sensorType\":\"gps\",\"value\":{"
                     "\"latitude\":%s,\"longitude\":%s,\"satellites\":%d,"
                     "\"valid\":%s,\"fixQuality\":%d,\"ageMs\":%s,\"positionStatus\":\"%s\"",
                     lat, lon, g->sats_used, state == GPS_VALID ? "true" : "false",
                     g->fix_quality, age, gps_position_state_name(state));
    gps_metrics_json(&w,&g->metrics,state==GPS_VALID,g->gga_count && (uint32_t)(now_ms-g->last_gga_ms)<GPS_FIX_MAX_AGE_MS);
    json_add(&w,"},\"unit\":\"\\u00b0\",\"timestamp\":%" PRIu64 "}",epoch_ms);
    return w.failed ? 0 : w.used;
}
