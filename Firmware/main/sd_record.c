#include "sd_record.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "gps_format.h"
_Static_assert(sizeof(sd_record_t) <= 216, "SD queue RAM budget changed");

size_t sd_record_format(char *out, size_t size, const sd_record_t *r,
                        uint64_t boot_id, uint64_t epoch_ms)
{
    if (!out || !size || !r) return 0;
    char utc[32] = "null";
    if (epoch_ms) snprintf(utc, sizeof(utc), "%" PRIu64, epoch_ms);
    int n = snprintf(out, size,
                     "{\"v\":1,\"boot\":\"%016" PRIx64 "\",\"seq\":%" PRIu64
                     ",\"mono_us\":%" PRIu64 ",\"timestamp_ms\":%s,",
                     boot_id, r->seq, r->mono_us, utc);
    if (n < 0 || (size_t)n >= size) return 0;
    size_t used = (size_t)n;
    switch (r->type) {
    case SD_RECORD_MOTION: {
        json_writer_t w={.out=out,.cap=size,.used=used};
        json_add(&w,"\"type\":\"motion\",\"value\":{");
        motion_json_fields(&w,&r->value.motion,true);
        json_add(&w,"}}\n"); return w.failed ? 0:w.used;
    }
    case SD_RECORD_TRIP: {
        json_writer_t w={.out=out,.cap=size,.used=used};
        json_add(&w,"\"type\":\"trip\",\"value\":{\"latitude\":");
        if (r->value.trip.valid) json_add(&w,"%.6f",r->value.trip.lat); else json_add(&w,"null");
        json_add(&w,",\"longitude\":");
        if (r->value.trip.valid) json_add(&w,"%.6f",r->value.trip.lon); else json_add(&w,"null");
        gps_metrics_json(&w,&r->value.trip.metrics,r->value.trip.valid,r->value.trip.gga_fresh);
        json_add(&w,"}}\n"); return w.failed ? 0:w.used;
    }
    case SD_RECORD_CAN: {
        if (r->value.can.dlc > 15) return 0;
        char hex[17] = {0};
        if (!r->value.can.rtr) {
            for (unsigned i = 0; i < r->value.can.dlc && i < 8; ++i)
                snprintf(hex + i * 2, sizeof(hex) - i * 2, "%02X", r->value.can.data[i]);
        }
        n = snprintf(out + used, size - used,
                     "\"type\":\"can\",\"id\":%" PRIu32 ",\"extended\":%s,\"rtr\":%s,"
                     "\"dlc\":%u,\"data\":\"%s\"}\n", r->value.can.id,
                     r->value.can.extended ? "true" : "false",
                     r->value.can.rtr ? "true" : "false", r->value.can.dlc, hex);
        break;
    }
    case SD_RECORD_IMU:
        n = snprintf(out + used, size - used,
                     "\"type\":\"imu\",\"acc_raw\":[%d,%d,%d],\"gyr_raw\":[%d,%d,%d],"
                     "\"temp_raw\":%d,\"sensor_time\":%" PRIu32 ",\"status\":%u,"
                     "\"acc_conf\":%u,\"gyr_conf\":%u,\"calibrated\":%s}\n",
                     r->value.imu.axes[0], r->value.imu.axes[1], r->value.imu.axes[2],
                     r->value.imu.axes[3], r->value.imu.axes[4], r->value.imu.axes[5],
                     r->value.imu.temperature, r->value.imu.sensor_time, r->value.imu.status,
                     r->value.imu.acc_conf, r->value.imu.gyr_conf,
                     r->value.imu.calibrated ? "true" : "false");
        break;
    case SD_RECORD_NMEA: {
        char escaped[SD_NMEA_MAX * 6 + 1];
        size_t p = 0, i;
        for (i = 0; i < SD_NMEA_MAX && r->value.nmea[i]; ++i) {
            unsigned char c = (unsigned char)r->value.nmea[i];
            if (c == '"' || c == '\\') {
                escaped[p++] = '\\'; escaped[p++] = (char)c;
            } else if (c < 0x20 || c >= 0x7f) {
                snprintf(escaped + p, sizeof(escaped) - p, "\\u%04x", c);
                p += 6;
            } else escaped[p++] = (char)c;
        }
        if (i == SD_NMEA_MAX) return 0;
        escaped[p] = '\0';
        n = snprintf(out + used, size - used, "\"type\":\"gps_nmea\",\"sentence\":\"%s\"}\n", escaped);
        break;
    }
    case SD_RECORD_HEALTH:
        n = snprintf(out + used, size - used,
                     "\"type\":\"health\",\"accepted\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 "],"
                     "\"dropped\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 "],"
                     "\"queued\":%" PRIu32 ",\"queue_peak\":%" PRIu32 ",\"committed\":%" PRIu32
                     ",\"io_errors\":%" PRIu32 ",\"max_write_us\":%" PRIu32 ",\"max_queue_age_ms\":%" PRIu32
                     ",\"imu_samples\":%" PRIu32 ",\"imu_errors\":%" PRIu32 ",\"imu_missed_estimate\":%" PRIu32
                     ",\"can_received\":%" PRIu32 ",\"can_cloud_dropped\":%" PRIu32 ",\"can_bus_errors\":%" PRIu32
                     ",\"gps_overflows\":%" PRIu32
                     ",\"last_data_us\":%" PRIu32 ",\"max_data_us\":%" PRIu32
                     ",\"last_sync_us\":%" PRIu32 ",\"max_sync_us\":%" PRIu32
                     ",\"last_batch_bytes\":%" PRIu32 ",\"last_batch_records\":%" PRIu32
                     ",\"committed_bytes\":%" PRIu64 ",\"derived_dropped\":[%" PRIu32 ",%" PRIu32 "]"
                     ",\"motion_queue_dropped\":%" PRIu32 "}\n",
                     r->value.health.accepted[0], r->value.health.accepted[1], r->value.health.accepted[2],
                     r->value.health.dropped[0], r->value.health.dropped[1], r->value.health.dropped[2],
                     r->value.health.queued, r->value.health.queue_peak, r->value.health.committed,
                     r->value.health.io_errors, r->value.health.max_write_us, r->value.health.max_queue_age_ms,
                     r->value.health.imu_samples, r->value.health.imu_errors, r->value.health.imu_missed,
                     r->value.health.can_received, r->value.health.can_cloud_dropped,
                     r->value.health.can_bus_errors, r->value.health.gps_overflows,
                     r->value.health.last_data_us, r->value.health.max_data_us,
                     r->value.health.last_sync_us, r->value.health.max_sync_us,
                     r->value.health.last_batch_bytes, r->value.health.last_batch_records,
                     r->value.health.committed_bytes,r->value.health.derived_dropped[0],r->value.health.derived_dropped[1],
                     r->value.health.motion_queue_dropped);
        break;
    default: return 0;
    }
    if (n < 0 || (size_t)n >= size - used) return 0;
    return used + (size_t)n;
}
