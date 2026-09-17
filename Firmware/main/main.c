#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "bmi_app.h"
#include "can_app.h"
#include "gps_app.h"
#include "mqtt_app.h"
#include "sntp_app.h"
#include "telemetry_app.h"

#if CONFIG_MOVE2_USE_LTE
#include "lte_app.h"
#else
#include "wifi_app.h"
#endif

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando sistema da moto eletrica...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

#if CONFIG_MOVE2_USE_LTE
    ESP_LOGI(TAG, "Inicializando conectividade LTE...");
    ESP_ERROR_CHECK(lte_app_init());

    ESP_LOGI(TAG, "Aguardando IP via LTE/PPP...");
    if (!lte_app_wait_connected(CONFIG_MOVE2_LTE_CONNECT_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "LTE nao recebeu IP dentro do timeout configurado.");
        return;
    }

    sntp_app_init();
#else
    wifi_app_init();
#endif

    mqtt_app_init();

    gps_app_init();
    bmi323_app_init();
    can_app_init();

    telemetry_app_init();

    ESP_LOGI(TAG, "Boot concluido. Dados fluindo para a nuvem!");
}
