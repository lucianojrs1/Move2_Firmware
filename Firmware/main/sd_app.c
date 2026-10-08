#include "sd_app.h"
#include "sd_batch.h"
#include "sntp_app.h"
#include "bmi_app.h"
#include "can_app.h"
#include "gps_app.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sdkconfig.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define MOUNT_POINT "/sdcard"
#define RETRY_MS 5000
#define REPORT_MS 5000
static const char *TAG = "SD_CARD";
static QueueHandle_t queue;
static portMUX_TYPE stats_lock = portMUX_INITIALIZER_UNLOCKED;
static sdcard_status_t stats;
static uint64_t sequence, boot_id;
static sdmmc_card_t *card;
static bool spi_ready;
static _Alignas(16) sd_batch_t batch;
static char line[SD_RECORD_LINE_MAX]; // Writer-owned buffers.

bool sdcard_app_record(sd_record_t *r)
{
    if (!r || (unsigned)r->type >= SD_RECORD_COUNT) return false;
    portENTER_CRITICAL(&stats_lock);
    r->seq = ++sequence;
    portEXIT_CRITICAL(&stats_lock);
    bool ok = queue && xQueueSend(queue, r, 0) == pdTRUE;
    uint32_t depth = queue ? uxQueueMessagesWaiting(queue) : 0;
    portENTER_CRITICAL(&stats_lock);
    if (ok) stats.accepted[r->type]++; else stats.dropped[r->type]++;
    if (depth > stats.queue_peak) stats.queue_peak = depth;
    portEXIT_CRITICAL(&stats_lock);
    return ok;
}

bool sdcard_app_record_from_isr(sd_record_t *r, BaseType_t *woken)
{
    if (!r || (unsigned)r->type >= SD_RECORD_COUNT) return false;
    portENTER_CRITICAL_ISR(&stats_lock);
    r->seq = ++sequence;
    portEXIT_CRITICAL_ISR(&stats_lock);
    bool ok = queue && xQueueSendFromISR(queue, r, woken) == pdTRUE;
    uint32_t depth = queue ? uxQueueMessagesWaitingFromISR(queue) : 0;
    portENTER_CRITICAL_ISR(&stats_lock);
    if (ok) stats.accepted[r->type]++; else stats.dropped[r->type]++;
    if (depth > stats.queue_peak) stats.queue_peak = depth;
    portEXIT_CRITICAL_ISR(&stats_lock);
    return ok;
}

sdcard_status_t sdcard_app_get_status(void)
{
    portENTER_CRITICAL(&stats_lock);
    sdcard_status_t copy = stats;
    portEXIT_CRITICAL(&stats_lock);
    copy.queued = queue ? uxQueueMessagesWaiting(queue) : 0;
    return copy;
}

static void set_mounted(bool mounted)
{
    portENTER_CRITICAL(&stats_lock);
    stats.mounted = mounted;
    portEXIT_CRITICAL(&stats_lock);
}

static esp_err_t mount_card(void)
{
    if (!spi_ready) {
        spi_bus_config_t bus = {
            .mosi_io_num = 15, .miso_io_num = 2, .sclk_io_num = 14,
            .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 4096,
        };
        esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK) return ret;
        // DAT0/MISO must not float while the card is deselected. This supplements
        // the board pull-up; a physical pull-down can still keep the idle line low.
        gpio_pullup_en(GPIO_NUM_2);
        spi_ready = true;
    }
    esp_vfs_fat_sdmmc_mount_config_t cfg = {
        .format_if_mount_failed = false, .max_files = 2, .allocation_unit_size = 16 * 1024,
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = 13;
    slot.host_id = host.slot;
    slot.wait_for_miso = CONFIG_MOVE2_SD_MISO_WAIT_MS;
    esp_err_t ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot, &cfg, &card);
    if (ret == ESP_OK) {
        set_mounted(true);
        int real_khz = 0;
        if (card->host.get_real_freq)
            card->host.get_real_freq(card->host.slot, &real_khz);
        int miso_idle = gpio_get_level(GPIO_NUM_2);
        ESP_LOGI(TAG, "SPI=%d kHz sector=%u bytes MISO_idle=%d pre_cmd_wait=%d ms",
                 real_khz, (unsigned)card->csd.sector_size, miso_idle, CONFIG_MOVE2_SD_MISO_WAIT_MS);
        if (!miso_idle)
            ESP_LOGW(TAG, "MISO baixo com CS inativo; confira pull-up do GPIO2/slot. Espera pre-comando limitada.");
        ESP_LOGI(TAG, "SD montado; fila=%d registros (%u bytes), sync=%d ms",
                 CONFIG_MOVE2_SD_QUEUE_LENGTH, (unsigned)(CONFIG_MOVE2_SD_QUEUE_LENGTH * sizeof(sd_record_t)),
                 CONFIG_MOVE2_SD_SYNC_MS);
    }
    return ret;
}

static void disconnect_card(int *fd)
{
    if (*fd >= 0) { close(*fd); *fd = -1; }
    if (card) { esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card); card = NULL; }
    set_mounted(false);
}

static int open_segment(uint32_t *segment)
{
    char path[80];
    // Never overwrite an earlier session, even in case of a random ID collision.
    for (unsigned attempt = 0; attempt < 100; ++attempt) {
        snprintf(path, sizeof(path), MOUNT_POINT "/%016" PRIx64 "-%06" PRIu32 ".jsonl",
                 boot_id, (*segment)++);
        int fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0666);
        if (fd >= 0) { ESP_LOGI(TAG, "Gravando %s", path); return fd; }
        if (errno != EEXIST) break;
        if (*segment == 1) {
            boot_id = ((uint64_t)esp_random() << 32) | esp_random();
            *segment = 0;
        }
    }
    return -1;
}

typedef struct { int fd; uint32_t data_us, sync_us; } sd_io_t;

static ptrdiff_t write_batch_data(void *ctx, const char *data, size_t size)
{
    sd_io_t *io = ctx;
    int64_t started = esp_timer_get_time();
    ssize_t n;
    do { n = write(io->fd, data, size); } while (n < 0 && errno == EINTR);
    io->data_us += (uint32_t)(esp_timer_get_time() - started);
    return n;
}
static bool sync_batch_data(void *ctx)
{
    sd_io_t *io = ctx;
    int64_t started = esp_timer_get_time();
    bool ok = fsync(io->fd) == 0;
    io->sync_us += (uint32_t)(esp_timer_get_time() - started);
    return ok;
}

static void report_status(void)
{
    sdcard_status_t s = sdcard_app_get_status();
    bmi323_status_t imu = bmi323_app_get_status();
    can_app_stats_t can = can_app_get_stats();
    gps_status_t gps = gps_app_get_status();
    sd_record_t record = {.type = SD_RECORD_HEALTH, .mono_us = (uint64_t)esp_timer_get_time()};
    for (unsigned i = 0; i < 3; ++i) {
        record.value.health.accepted[i] = s.accepted[i];
        record.value.health.dropped[i] = s.dropped[i];
    }
    record.value.health.queued = s.queued;
    record.value.health.queue_peak = s.queue_peak;
    record.value.health.committed = s.committed;
    record.value.health.io_errors = s.io_errors;
    record.value.health.max_write_us = s.max_write_us;
    record.value.health.max_queue_age_ms = s.max_queue_age_ms;
    record.value.health.imu_samples = imu.samples;
    record.value.health.imu_errors = imu.read_errors;
    record.value.health.imu_missed = imu.estimated_missed;
    record.value.health.motion_queue_dropped = imu.motion_queue_dropped;
    record.value.health.can_received = can.received;
    record.value.health.can_cloud_dropped = can.cloud_dropped;
    record.value.health.can_bus_errors = can.bus_errors;
    record.value.health.gps_overflows = gps.line_overflows;
    record.value.health.last_data_us = s.last_data_us;
    record.value.health.max_data_us = s.max_data_us;
    record.value.health.last_sync_us = s.last_sync_us;
    record.value.health.max_sync_us = s.max_sync_us;
    record.value.health.last_batch_bytes = s.last_batch_bytes;
    record.value.health.last_batch_records = s.last_batch_records;
    record.value.health.committed_bytes = s.committed_bytes;
    record.value.health.derived_dropped[0] = s.dropped[SD_RECORD_MOTION];
    record.value.health.derived_dropped[1] = s.dropped[SD_RECORD_TRIP];
    sdcard_app_record(&record);
    ESP_LOGI(TAG, "mounted=%d queue=%" PRIu32 "/%d peak=%" PRIu32
             " saved=%" PRIu32 " accepted(C/I/G)=%" PRIu32 "/%" PRIu32 "/%" PRIu32
             " dropped(C/I/G)=%" PRIu32 "/%" PRIu32 "/%" PRIu32
             " io_errors=%" PRIu32 " write_max_us=%" PRIu32 " age_max_ms=%" PRIu32,
             s.mounted, s.queued, CONFIG_MOVE2_SD_QUEUE_LENGTH, s.queue_peak, s.committed,
             s.accepted[0], s.accepted[1], s.accepted[2], s.dropped[0], s.dropped[1], s.dropped[2],
             s.io_errors, s.max_write_us, s.max_queue_age_ms);
    ESP_LOGI(TAG, "batch=%" PRIu32 " bytes/%" PRIu32 " records data_us=%" PRIu32
             " sync_us=%" PRIu32 " data_max_us=%" PRIu32 " sync_max_us=%" PRIu32
             " saved_bytes=%" PRIu64,
             s.last_batch_bytes, s.last_batch_records, s.last_data_us, s.last_sync_us,
             s.max_data_us, s.max_sync_us, s.committed_bytes);
    ESP_LOGI(TAG, "IMU samples=%" PRIu32 " read_errors=%" PRIu32 " estimated_missed=%" PRIu32
             " motion_queue_dropped=%" PRIu32
             " CAN rx=%" PRIu32 " cloud_dropped=%" PRIu32 " bus_errors=%" PRIu32
             " GPS overflow=%" PRIu32,
             imu.samples, imu.read_errors, imu.estimated_missed, imu.motion_queue_dropped,
             can.received, can.cloud_dropped, can.bus_errors, gps.line_overflows);
}

static void sdcard_task(void *arg)
{
    int fd = -1;
    uint32_t segment = 0, file_bytes = 0;
    int64_t batch_started = 0, last_report = esp_timer_get_time();
    bool retry_batch = false;
    const uint32_t rotate_bytes = CONFIG_MOVE2_SD_FILE_MB * 1024U * 1024U;
    for (;;) {
        int64_t now = esp_timer_get_time();
        if (now - last_report >= REPORT_MS * 1000LL) {
            report_status();
            last_report = now;
        }
        if (!card) {
            esp_err_t ret = mount_card();
            if (ret != ESP_OK) {
                portENTER_CRITICAL(&stats_lock);
                stats.io_errors++;
                portEXIT_CRITICAL(&stats_lock);
                ESP_LOGW(TAG, "SD indisponivel (%s); nova tentativa em %d ms. Buffer RAM limitado.",
                         esp_err_to_name(ret), RETRY_MS);
                vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
                continue;
            }
        }
        if (fd < 0) {
            fd = open_segment(&segment);
            if (fd < 0) goto io_failure;
            file_bytes = 0;
        }
        if (!retry_batch) {
            sd_record_t record;
            // Reserve space for the largest escaped line before dequeuing.
            while (sizeof(batch.data) - batch.used >= SD_RECORD_LINE_MAX) {
                TickType_t wait = batch.used ? 0 : pdMS_TO_TICKS(50);
                if (xQueueReceive(queue, &record, wait) != pdTRUE) break;
                now = esp_timer_get_time();
                uint32_t age_ms = (uint32_t)((now - (int64_t)record.mono_us) / 1000);
                portENTER_CRITICAL(&stats_lock);
                if (age_ms > stats.max_queue_age_ms) stats.max_queue_age_ms = age_ms;
                portEXIT_CRITICAL(&stats_lock);
                size_t len = sd_record_format(line, sizeof(line), &record, boot_id,
                                              sntp_app_epoch_ms_at(record.mono_us));
                if (!len) {
                    portENTER_CRITICAL(&stats_lock);
                    stats.dropped[record.type]++;
                    portEXIT_CRITICAL(&stats_lock);
                    continue;
                }
                if (!batch.used) batch_started = now;
                sd_batch_append(&batch, line, len);
                if (now - batch_started >= CONFIG_MOVE2_SD_SYNC_MS * 1000LL) break;
            }
        }
        now = esp_timer_get_time();
        bool due = batch.used && (retry_batch || sizeof(batch.data) - batch.used < SD_RECORD_LINE_MAX ||
                            now - batch_started >= CONFIG_MOVE2_SD_SYNC_MS * 1000LL);
        if (due) {
            int64_t start = esp_timer_get_time();
            size_t bytes_to_commit = batch.used;
            uint32_t records_to_commit = batch.count;
            sd_io_t io = {.fd = fd};
            bool ok = sd_batch_commit(&batch, &io, write_batch_data, sync_batch_data);
            uint32_t elapsed = (uint32_t)(esp_timer_get_time() - start);
            portENTER_CRITICAL(&stats_lock);
            if (elapsed > stats.max_write_us) stats.max_write_us = elapsed;
            stats.last_data_us = io.data_us;
            stats.last_sync_us = io.sync_us;
            if (io.data_us > stats.max_data_us) stats.max_data_us = io.data_us;
            if (io.sync_us > stats.max_sync_us) stats.max_sync_us = io.sync_us;
            stats.last_batch_bytes = bytes_to_commit;
            stats.last_batch_records = records_to_commit;
            if (ok) {
                stats.committed += records_to_commit;
                stats.committed_bytes += bytes_to_commit;
            }
            portEXIT_CRITICAL(&stats_lock);
            if (!ok) goto io_failure;
            file_bytes += bytes_to_commit;
            retry_batch = false;
            if (file_bytes >= rotate_bytes) {
                if (close(fd) != 0) { fd = -1; goto io_failure; }
                fd = -1;
            }
        } else vTaskDelay(1);
        continue;
io_failure:
        portENTER_CRITICAL(&stats_lock);
        stats.io_errors++;
        portEXIT_CRITICAL(&stats_lock);
        ESP_LOGE(TAG, "Erro SD errno=%d; lote RAM mantido (%u bytes). Retry em %d ms.",
                 errno, (unsigned)batch.used, RETRY_MS);
        // Failed fsync has unknown durability. Retry SAME sequence numbers in a new
        // segment. Readers deduplicate (boot, seq) and ignore a partial final line.
        retry_batch = batch.used != 0;
        disconnect_card(&fd);
        vTaskDelay(pdMS_TO_TICKS(RETRY_MS));
    }
}

void sdcard_app_init(void)
{
    if (queue) return;
    boot_id = ((uint64_t)esp_random() << 32) | esp_random();
    queue = xQueueCreate(CONFIG_MOVE2_SD_QUEUE_LENGTH, sizeof(sd_record_t));
    if (!queue) { ESP_LOGE(TAG, "Sem RAM para fila SD"); return; }
    if (xTaskCreate(sdcard_task, "sdcard_task", 6144, NULL, 3, NULL) != pdPASS) {
        vQueueDelete(queue); queue = NULL;
        ESP_LOGE(TAG, "Sem RAM para task SD");
    }
}
