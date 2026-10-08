#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#include "bmi_app.h"
#include "can_app.h"
#include "gps_app.h"
#include "mqtt_app.h"
#include "sd_app.h"
#include "sntp_app.h"
#include "telemetry_app.h"

#if CONFIG_MOVE2_USE_LTE
#include "lte_app.h"
#else
#include "wifi_app.h"
#endif

static const char *TAG = "MAIN";

static void connectivity_task(void *arg)
{
#if CONFIG_MOVE2_USE_LTE
    for (;;) {
        if (!lte_app_is_connected()) {
            esp_err_t err = lte_app_init();
            if (err != ESP_OK || !lte_app_wait_connected(CONFIG_MOVE2_LTE_CONNECT_TIMEOUT_MS)) {
                ESP_LOGW(TAG, "LTE indisponivel (%s); coleta local continua.", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
        }
        sntp_app_init();
        mqtt_app_init();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
#else
    wifi_app_init(); // May wait indefinitely; acquisition already runs independently.
    mqtt_app_init();
    vTaskDelete(NULL);
#endif
}

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando sistema da moto eletrica...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Keep board peripheral power enabled independently of the modem sequence.
    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_12, 1);
    sdcard_app_init();
    gps_app_init();
    bmi323_app_init();
    can_app_init();

    telemetry_app_init();
    if (xTaskCreate(connectivity_task, "connectivity", 6144, NULL, 3, NULL) != pdPASS)
        ESP_LOGE(TAG, "Sem RAM para conectividade; coleta local continua.");
    ESP_LOGI(TAG, "Coleta local iniciada. Conectividade em background.");
}
