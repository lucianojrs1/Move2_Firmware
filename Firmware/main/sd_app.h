#ifndef SD_APP_H
#define SD_APP_H
#include "freertos/FreeRTOS.h"
#include "sd_record.h"
typedef struct {
    uint32_t accepted[SD_RECORD_COUNT], dropped[SD_RECORD_COUNT];
    uint32_t committed, io_errors, queue_peak, max_write_us, max_queue_age_ms;
    uint32_t last_data_us, max_data_us, last_sync_us, max_sync_us;
    uint32_t last_batch_bytes, last_batch_records;
    uint64_t committed_bytes;
    uint32_t queued;
    bool mounted;
} sdcard_status_t;
// Creates queue immediately; mounting and retries run in the writer task.
void sdcard_app_init(void);
// Copy, assign sequence, enqueue without waiting. No card access in producers.
bool sdcard_app_record(sd_record_t *record);
bool sdcard_app_record_from_isr(sd_record_t *record, BaseType_t *task_woken);
sdcard_status_t sdcard_app_get_status(void);
#endif
