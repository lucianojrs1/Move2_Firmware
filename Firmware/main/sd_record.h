#ifndef SD_RECORD_H
#define SD_RECORD_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "motion.h"
#include "gps_metrics.h"
#define SD_NMEA_MAX 192
#define SD_RECORD_LINE_MAX 1536
typedef enum { SD_RECORD_CAN, SD_RECORD_IMU, SD_RECORD_NMEA, SD_RECORD_HEALTH,
               SD_RECORD_MOTION, SD_RECORD_TRIP, SD_RECORD_COUNT } sd_record_type_t;
// Compact records: producers never allocate memory or format JSON.
typedef struct {
    uint64_t mono_us, seq;
    sd_record_type_t type;
    union {
        struct { uint32_t id; uint8_t dlc; bool extended, rtr; uint8_t data[8]; } can;
        struct {
            int16_t axes[6], temperature; // Unmodified sensor counts.
            uint16_t status, acc_conf, gyr_conf;
            uint32_t sensor_time;
            bool calibrated;
        } imu;
        char nmea[SD_NMEA_MAX];
        motion_sample_t motion;
        struct { gps_metrics_t metrics; double lat, lon; bool valid, gga_fresh; } trip;
        struct {
            uint32_t accepted[3], dropped[3], queued, queue_peak, committed, io_errors;
            uint32_t max_write_us, max_queue_age_ms, imu_samples, imu_errors, imu_missed;
            uint32_t can_received, can_cloud_dropped, can_bus_errors, gps_overflows;
            uint32_t last_data_us, max_data_us, last_sync_us, max_sync_us;
            uint32_t last_batch_bytes, last_batch_records;
            uint64_t committed_bytes;
            uint32_t derived_dropped[2];
            uint32_t motion_queue_dropped;
        } health;
    } value;
} sd_record_t;
// Bytes including newline, excluding NUL; zero means invalid or too small.
size_t sd_record_format(char *out, size_t size, const sd_record_t *record,
                        uint64_t boot_id, uint64_t epoch_ms);
#endif
