#ifndef GPS_FORMAT_H
#define GPS_FORMAT_H
#include <stddef.h>
#include "gps_app.h"
#include "json_writer.h"
#define GPS_PAYLOAD_MAX 1024
void gps_metrics_json(json_writer_t *w, const gps_metrics_t *m, bool fix_fresh, bool gga_fresh);
#define GPS_FIX_MAX_AGE_MS 3000U
typedef enum { GPS_NO_DATA, GPS_NO_FIX, GPS_STALE, GPS_VALID } gps_position_state_t;
gps_position_state_t gps_position_state(const gps_status_t *gps, uint32_t now_ms);
const char *gps_position_state_name(gps_position_state_t state);
// Invalid/stale coordinates are null, never a fabricated position at (0, 0).
size_t gps_format_payload(char *out, size_t size, const gps_status_t *gps,
                          uint32_t now_ms, uint64_t epoch_ms);
#endif
