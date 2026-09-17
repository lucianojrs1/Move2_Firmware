#include "esp_sntp.h"
#include "esp_log.h"
#include "sntp_app.h"
#include <time.h>
#include <sys/time.h>
#include <stdbool.h>

static const char *TAG = "SNTP_APP";
static bool s_time_synchronized = false;

// Callback disparado automaticamente quando a hora for ajustada
static void time_sync_notification_cb(struct timeval *tv) {
    s_time_synchronized = true;
    ESP_LOGI(TAG, "RTC interno sincronizado com sucesso via rede!");
}

void sntp_app_init(void) {
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
    return sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED;
}

// Retorna o timestamp UTC em milissegundos (64 bits)
uint64_t get_rtc_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return ((uint64_t)tv.tv_sec * 1000ULL) + (tv.tv_usec / 1000ULL);
}
