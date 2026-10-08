#include "gps_navigation.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define PI 3.14159265358979323846
static bool number(const char *s, double low, double high, double *out) {
    if (!s || !*s) return false;
    const char *p=s; if (*p=='-') ++p;
    bool digit=false, dot=false;
    for (;*p;++p) {
        if (*p=='.' && !dot) dot=true;
        else if (*p>='0' && *p<='9') digit=true;
        else return false;
    }
    if (!digit) return false;
    char *end; double x = strtod(s, &end);
    if (*end || !isfinite(x) || x < low || x > high) return false;
    *out = x; return true;
}
static int hex_digit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)toupper((unsigned char)c);
    return c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
}
static bool sentence(char *out, const char *line) {
    size_t n = strlen(line);
    if (n < 10 || n >= 192 || line[0] != '$') return false;
    const char *star = strchr(line, '*');
    if (!star || strlen(star) != 3) return false;
    int a = hex_digit(star[1]), b = hex_digit(star[2]);
    if (a < 0 || b < 0) return false;
    unsigned sum = 0;
    for (const char *p = line + 1; p < star; ++p) sum ^= (unsigned char)*p;
    if (sum != (unsigned)(a * 16 + b)) return false;
    memcpy(out, line, (size_t)(star - line)); out[star - line] = 0;
    return true;
}
static bool coordinate(const char *value, const char *hemisphere, bool latitude, double *out) {
    double raw;
    if (strlen(hemisphere) != 1 || !number(value, 0, latitude ? 9000 : 18000, &raw)) return false;
    char h = *hemisphere;
    if (latitude ? (h != 'N' && h != 'S') : (h != 'E' && h != 'W')) return false;
    double degrees = floor(raw / 100), minutes = raw - degrees * 100;
    if (minutes >= 60) return false;
    *out = degrees + minutes / 60;
    if (*out > (latitude ? 90 : 180)) return false;
    if (h == 'S' || h == 'W') *out = -*out;
    return true;
}
static double distance(double lat1, double lon1, double lat2, double lon2) {
    double p = (lat2 - lat1) * PI / 180, l = (lon2 - lon1) * PI / 180;
    double a = sin(p/2)*sin(p/2) + cos(lat1*PI/180)*cos(lat2*PI/180)*sin(l/2)*sin(l/2);
    if (a > 1) a = 1;
    return 6371000.0 * 2 * asin(sqrt(a));
}
void gps_navigation_init(gps_navigation_t *nav, uint32_t now_ms) {
    *nav = (gps_navigation_t){.started_ms = now_ms};
}
static void update_trip(gps_navigation_t *nav, gps_status_t *g, uint32_t now) {
    gps_metrics_t *m = &g->metrics;
    m->tracking = g->fix && m->speed_valid && g->gga_count &&
        (uint32_t)(now - g->last_gga_ms) < 3000 && g->fix_quality >= 1 && g->fix_quality <= 5 &&
        g->sats_used >= 4 && m->hdop_valid && m->hdop <= 4;
    if (!m->tracking) {
        nav->have_previous = nav->candidate = m->moving = false;
        return;
    }
    uint32_t dt = now - nav->previous_ms;
    bool continuous = nav->have_previous && dt > 0 && dt <= 2500;
    double step = continuous ? distance(nav->previous_lat, nav->previous_lon, g->lat, g->lon) : 0;
    // Reject jumps inconsistent with reported speed, and impossible road speed.
    double expected = fmax(m->speed_kmh, nav->previous_speed) / 3.6 * dt / 1000.0;
    if (continuous && step > fmax(20, expected * 2 + 10)) {
        nav->have_previous = nav->candidate = m->moving = false;
        m->tracking = false; return;
    }
    if (!continuous) { nav->candidate = false; m->moving = false; }
    if (m->speed_kmh <= 2.5f) m->moving = false;
    bool just_started = false;
    if (!m->moving) {
        if (m->speed_kmh < 5) nav->candidate = false;
        else if (!nav->candidate) {
            nav->candidate = true; nav->candidate_ms = now;
            nav->candidate_lat = g->lat; nav->candidate_lon = g->lon;
        } else if ((uint32_t)(now - nav->candidate_ms) >= 4000 &&
                   distance(nav->candidate_lat, nav->candidate_lon, g->lat, g->lon) >= 8) {
            m->moving = just_started = true; nav->candidate = false;
            // Start at the confirmed position; never backfill an uncertain launch.
            nav->anchor_lat = g->lat; nav->anchor_lon = g->lon;
        }
    }
    if (m->moving && m->speed_kmh > m->max_speed_kmh) m->max_speed_kmh = m->speed_kmh;
    if (continuous) {
        if (m->moving && !just_started) {
            m->moving_ms += dt;
            double displacement = distance(nav->anchor_lat, nav->anchor_lon, g->lat, g->lon);
            if (displacement >= 2) {
                m->trip_distance_m += displacement;
                nav->anchor_lat = g->lat; nav->anchor_lon = g->lon;
            }
        } else {
            m->stopped_ms += dt;
            nav->anchor_lat = g->lat; nav->anchor_lon = g->lon;
        }
    } else { nav->anchor_lat = g->lat; nav->anchor_lon = g->lon; }
    m->average_speed_kmh = m->moving_ms ? (float)(m->trip_distance_m * 3600 / m->moving_ms) : 0;
    nav->previous_lat = g->lat; nav->previous_lon = g->lon;
    nav->previous_speed = m->speed_kmh; nav->previous_ms = now; nav->have_previous = true;
}
bool gps_navigation_process(gps_navigation_t *nav, gps_status_t *g, const char *line, uint32_t now) {
    if (!nav || !g || !line) return false;
    char copy[192];
    if (!sentence(copy, line)) return false;
    char *f[24]; unsigned count = 1; f[0] = copy;
    for (char *p = copy; *p; ++p) if (*p == ',') {
        *p = 0; if (count < 24) f[count++] = p + 1;
    }
    if (strlen(f[0]) != 6) return false;
    gps_metrics_t *m = &g->metrics;
    double x=0;
    if (!strcmp(f[0] + 3, "GGA") && count >= 11) {
        g->gga_count++; g->last_gga_ms = now;
        g->fix_quality = number(f[6], 0, 8, &x) && floor(x)==x ? (int)x : 0;
        g->sats_used = number(f[7], 0, 99, &x) && floor(x)==x ? (int)x : 0;
        m->hdop_valid = number(f[8], 0.1, 99, &x); if (m->hdop_valid) m->hdop = (float)x;
        m->altitude_valid = g->fix_quality >= 1 && g->fix_quality <= 5 &&
            !strcmp(f[10], "M") && number(f[9], -1000, 20000, &x);
        if (m->altitude_valid) m->altitude_m = (float)x;
        if (g->fix_quality < 1 || g->fix_quality > 5) {
            g->fix = g->location_valid = false; m->tracking = false; nav->have_previous = false;
        }
        if (!m->hdop_valid || m->hdop>4 || g->sats_used<4) { m->tracking=false; nav->have_previous=false; }
        return false;
    }
    if (strcmp(f[0] + 3, "RMC") || count < 10) return false;
    double utc = 0, date = 0;
    size_t time_len=strlen(f[1]);
    bool time_ok = number(f[1], 0, 235959.999, &utc) && time_len>=6 &&
        (time_len==6 || (f[1][6]=='.' && time_len>7));
    int hh = (int)(utc / 10000), mm = ((int)utc / 100) % 100, ss = (int)utc % 100;
    time_ok = time_ok && hh < 24 && mm < 60 && ss < 60;
    bool date_ok = strlen(f[9]) == 6 && number(f[9], 10100, 311299, &date);
    int day = (int)date / 10000, month = ((int)date / 100) % 100, yy = (int)date % 100;
    date_ok = date_ok && day >= 1 && day <= 31 && month >= 1 && month <= 12;
    int year=yy>=80 ? 1900+yy:2000+yy;
    static const int month_days[12]={31,28,31,30,31,30,31,31,30,31,30,31};
    if (date_ok) {
        int days=month_days[month-1]+(month==2 && year%4==0 && (year%100!=0 || year%400==0));
        date_ok=day<=days;
    }
    int date_key=year*10000+month*100+day;
    // Repeated/out-of-order receiver epochs must not refresh position or integrate a trip.
    if (time_ok && date_ok && nav->have_utc &&
        (date_key<nav->previous_date || (date_key==nav->previous_date && utc<=nav->previous_utc)))
        return false;
    if (time_ok && date_ok) { nav->previous_utc = utc; nav->previous_date = date_key; nav->have_utc = true; }
    g->rmc_count++; g->rmc_status = f[2][0];
    g->time_valid = time_ok; g->date_valid = date_ok;
    if (time_ok) { g->hour = hh; g->minute = mm; g->second = ss; g->centisecond = (int)((utc-floor(utc))*100); }
    if (date_ok) { g->day = day; g->month = month; g->year = yy >= 80 ? 1900+yy : 2000+yy; }
    bool mode_ok = count <= 12 || !*f[12] || strchr("ADFR", *f[12]) != NULL;
    g->fix = g->location_valid = time_ok && date_ok && !strcmp(f[2], "A") && mode_ok &&
        coordinate(f[3], f[4], true, &g->lat) && coordinate(f[5], f[6], false, &g->lon);
    if (g->gga_count && (uint32_t)(now-g->last_gga_ms) < 3000 &&
        (g->fix_quality < 1 || g->fix_quality > 5)) g->fix = g->location_valid = false;
    m->speed_valid = g->fix && number(f[7], 0, 300.0/1.852, &x);
    if (m->speed_valid) m->speed_kmh = (float)(x * 1.852);
    m->course_valid = g->fix && m->speed_valid && m->speed_kmh >= 3 && number(f[8], 0, 360, &x);
    if (m->course_valid) m->course_deg = (float)(x == 360 ? 0 : x);
    if (g->fix) {
        g->last_fix_ms = now;
        if (!m->first_fix_valid) { m->first_fix_valid = true; m->first_fix_ms = now-nav->started_ms; }
    }
    update_trip(nav, g, now);
    return true;
}
