#include "gps_app.h"
#include "gps_navigation.h"
#include "gps_format.h"
#include "sd_app.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "GPS_APP";

// GPS usa apenas UART e WAKEUP. PPS fica desabilitado para liberar GPIO23 ao CAN.
#define PIN_GPS_TX GPIO_NUM_21
#define PIN_GPS_RX GPIO_NUM_22
#define PIN_GPS_PPS GPIO_NUM_NC
#define PIN_GPS_WAKEUP GPIO_NUM_19

#define GPS_UART_PORT UART_NUM_2
#define GPS_BAUDRATE 9600
#define GPS_RX_BUF_SIZE 2048

static gps_status_t g_gps = {0};
static gps_navigation_t navigation;
static SemaphoreHandle_t g_mutex = NULL;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void config_input_pin_if_present(gpio_num_t pin) {
    if (pin == GPIO_NUM_NC) {
        return;
    }

    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&conf);
}

static void safe_copy(char *dst, size_t dst_size, const char *src) {
    if (!dst || dst_size == 0) return;
    if (!src) { dst[0] = '\0'; return; }
    strncpy(dst, src, dst_size - 1); dst[dst_size - 1] = '\0';
}

static void extract_after_key(const char *line, const char *key, char *out, size_t out_sz) {
    const char *p = strstr(line, key);
    if (!p) { out[0] = '\0'; return; }
    p += strlen(key); size_t i = 0;
    while (*p && *p != ',' && *p != '*' && i < out_sz - 1) out[i++] = *p++;
    out[i] = '\0';
}

static void parse_gptxt(const char *line) {
    bool new_antenna_ok = strstr(line, "ANTENNA OK") != NULL;
    char module_tmp[48] = "", fw_tmp[64] = "";
    extract_after_key(line, "IC=", module_tmp, sizeof(module_tmp));
    extract_after_key(line, "SW=", fw_tmp, sizeof(fw_tmp));
    
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    if (new_antenna_ok && !g_gps.antenna_ok) { 
        g_gps.antenna_ok = true; 
        ESP_LOGI(TAG, "GPS antenna OK"); 
    }
    if (module_tmp[0]) safe_copy(g_gps.module_info, sizeof(g_gps.module_info), module_tmp);
    if (fw_tmp[0]) safe_copy(g_gps.fw_info, sizeof(g_gps.fw_info), fw_tmp);
    xSemaphoreGive(g_mutex);
}

static void gps_task(void *arg) {
    char line[SD_NMEA_MAX]; size_t used = 0; bool announced_uart = false;
    bool discard_line = false;
    ESP_LOGI(TAG, "Task de recepcao GPS iniciada");
    
    while (1) {
        uint8_t ch;
        if (uart_read_bytes(GPS_UART_PORT, &ch, 1, pdMS_TO_TICKS(200)) <= 0) continue;
        
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_gps.bytes++;
        if (!g_gps.uart_seen) { 
            g_gps.uart_seen = true; 
            if (!announced_uart) { ESP_LOGI(TAG, "GPS UART data detected"); announced_uart = true; } 
        }
        xSemaphoreGive(g_mutex);
        
        if (ch == '\r') continue;
        if (ch == '\n') {
            if (used > 0 && !discard_line) {
                line[used] = '\0';
                sd_record_t record = {.type = SD_RECORD_NMEA, .mono_us = (uint64_t)esp_timer_get_time()};
                memcpy(record.value.nmea, line, used + 1);
                sdcard_app_record(&record);
                xSemaphoreTake(g_mutex, portMAX_DELAY);
                g_gps.lines++; g_gps.last_sentence_ms = now_ms();
                xSemaphoreGive(g_mutex);
                
                if (strncmp(line, "$GPTXT", 6) == 0 || strncmp(line, "$GNTXT", 6) == 0) parse_gptxt(line);
                else {
                    sd_record_t trip = {.type=SD_RECORD_TRIP, .mono_us=(uint64_t)esp_timer_get_time()};
                    uint32_t stamp = (uint32_t)(trip.mono_us/1000);
                    xSemaphoreTake(g_mutex, portMAX_DELAY);
                    bool updated = gps_navigation_process(&navigation, &g_gps, line, stamp);
                    trip.value.trip.metrics = g_gps.metrics;
                    trip.value.trip.lat = g_gps.lat; trip.value.trip.lon = g_gps.lon;
                    trip.value.trip.valid = gps_position_state(&g_gps, stamp)==GPS_VALID;
                    trip.value.trip.gga_fresh = g_gps.gga_count && (uint32_t)(stamp-g_gps.last_gga_ms)<3000;
                    xSemaphoreGive(g_mutex);
                    if (updated) sdcard_app_record(&trip);
                }
            }
            used = 0;
            discard_line = false;
            continue;
        }
        if (discard_line) continue;
        if (used < sizeof(line) - 1) line[used++] = (char)ch;
        else {
            discard_line = true;
            xSemaphoreTake(g_mutex, portMAX_DELAY);
            g_gps.line_overflows++;
            xSemaphoreGive(g_mutex);
        }
    }
}

// ======================================================
// FUNÇÕES PÚBLICAS
// ======================================================
void gps_app_init(void) {
    g_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(g_mutex ? ESP_OK : ESP_ERR_NO_MEM);
    gps_navigation_init(&navigation, now_ms());
    
    gpio_config_t out_conf = { .pin_bit_mask = (1ULL << PIN_GPS_WAKEUP), .mode = GPIO_MODE_OUTPUT, .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE };
    gpio_config(&out_conf);
    
    config_input_pin_if_present(PIN_GPS_PPS);
    
    gpio_set_level(PIN_GPS_WAKEUP, 1);
    
    const uart_config_t cfg = { .baud_rate = GPS_BAUDRATE, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };
    uart_driver_install(GPS_UART_PORT, GPS_RX_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_PORT, &cfg);
    uart_set_pin(GPS_UART_PORT, PIN_GPS_TX, PIN_GPS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Dispara a Task do GPS internamente
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
}

gps_status_t gps_app_get_status(void) {
    if (!g_mutex) { gps_status_t empty = {0}; return empty; }
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    gps_status_t copy = g_gps;
    xSemaphoreGive(g_mutex);
    return copy;
}
