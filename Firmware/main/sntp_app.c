#include "esp_sntp.h"
#include "esp_log.h"
#include "sntp_app.h"
#include <time.h>
#include <sys/time.h>
#include <stdbool.h>
#include <stdatomic.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "SNTP_APP";
static bool s_time_synchronized;
static int64_t s_epoch_offset_us;
static portMUX_TYPE s_time_lock = portMUX_INITIALIZER_UNLOCKED;
static atomic_bool s_started;

// Callback disparado automaticamente quando a hora for ajustada
static void time_sync_notification_cb(struct timeval *tv) {
    int64_t offset = (int64_t)tv->tv_sec * 1000000 + tv->tv_usec - esp_timer_get_time();
    portENTER_CRITICAL(&s_time_lock);
    s_epoch_offset_us = offset;
    s_time_synchronized = true;
    portEXIT_CRITICAL(&s_time_lock);
    ESP_LOGI(TAG, "RTC interno sincronizado com sucesso via rede!");
}

void sntp_app_init(void) {
    if (atomic_exchange(&s_started, true)) return;
    ESP_LOGI(TAG, "Inicializando SNTP...");

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    
    // Servidores NTP primário e secundário
    esp_sntp_setservername(0, "a.st1.ntp.br"); // NTP Brasil
    esp_sntp_setservername(1, "pool.ntp.org"); // Pool Global

    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

// Verifica se o relógio já foi sincronizado
bool sntp_app_is_synced(void) {
    portENTER_CRITICAL(&s_time_lock);
    bool synced = s_time_synchronized;
    portEXIT_CRITICAL(&s_time_lock);
    return synced;
}

uint64_t sntp_app_epoch_ms_at(uint64_t mono_us) {
    portENTER_CRITICAL(&s_time_lock);
    bool synced = s_time_synchronized;
    int64_t offset = s_epoch_offset_us;
    portEXIT_CRITICAL(&s_time_lock);
    return synced ? (uint64_t)((int64_t)mono_us + offset) / 1000ULL : 0;
}

// Retorna o timestamp UTC em milissegundos (64 bits)
uint64_t get_rtc_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return ((uint64_t)tv.tv_sec * 1000ULL) + (tv.tv_usec / 1000ULL);
}
