#include "gps_app.h"
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
static SemaphoreHandle_t g_mutex = NULL;

static uint32_t now_ms(void) { return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS); }

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

static int split_csv_preserve_empty(char *s, char **fields, int max_fields) {
    int count = 0; char *p = s;
    if (!s || !fields || max_fields <= 0) return 0;
    fields[count++] = p;
    while (*p && count < max_fields) {
        if (*p == ',') { *p = '\0'; fields[count++] = p + 1; }
        else if (*p == '*') { *p = '\0'; break; }
        p++;
    }
    return count;
}

static void extract_after_key(const char *line, const char *key, char *out, size_t out_sz) {
    const char *p = strstr(line, key);
    if (!p) { out[0] = '\0'; return; }
    p += strlen(key); size_t i = 0;
    while (*p && *p != ',' && *p != '*' && i < out_sz - 1) out[i++] = *p++;
    out[i] = '\0';
}

static double nmea_degmin_to_decimal(const char *value) {
    double raw = atof(value);
    int deg = (int)(raw / 100.0);
    double minutes = raw - (deg * 100.0);
    return deg + (minutes / 60.0);
}

static bool parse_nmea_time(const char *s, int *hh, int *mm, int *ss, int *cc) {
    if (!s || strlen(s) < 6) return false;
    *hh = (s[0]-'0')*10 + (s[1]-'0'); *mm = (s[2]-'0')*10 + (s[3]-'0'); *ss = (s[4]-'0')*10 + (s[5]-'0'); *cc = 0;
    char *dot = strchr((char*)s, '.');
    if (dot && strlen(dot + 1) >= 2) *cc = (dot[1]-'0')*10 + (dot[2]-'0');
    else if (dot && strlen(dot + 1) == 1) *cc = (dot[1]-'0')*10;
    return true;
}

static bool parse_nmea_date(const char *s, int *day, int *month, int *year) {
    if (!s || strlen(s) < 6) return false;
    *day = (s[0]-'0')*10 + (s[1]-'0'); *month = (s[2]-'0')*10 + (s[3]-'0');
    int yy = (s[4]-'0')*10 + (s[5]-'0'); *year = (yy >= 80) ? (1900 + yy) : (2000 + yy);
    return true;
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

static void parse_gga(const char *line) {
    if (strncmp(line, "$GNGGA", 6) != 0 && strncmp(line, "$GPGGA", 6) != 0) return;
    char copy[192] = {0}; safe_copy(copy, sizeof(copy), line);
    char *fields[20] = {0};
    if (split_csv_preserve_empty(copy, fields, 20) < 8) return;
    int fixq = fields[6] ? atoi(fields[6]) : 0;
    int sats = fields[7] ? atoi(fields[7]) : 0;
    
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_gps.fix_quality = fixq; g_gps.sats_used = sats;
    if (fixq == 0) g_gps.fix = false;
    xSemaphoreGive(g_mutex);
}

static void parse_rmc(const char *line) {
    if (strncmp(line, "$GNRMC", 6) != 0 && strncmp(line, "$GPRMC", 6) != 0) return;
    char copy[192] = {0}; safe_copy(copy, sizeof(copy), line);
    char *fields[16] = {0};
    if (split_csv_preserve_empty(copy, fields, 16) < 10) return;
    
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_gps.rmc_status = (fields[2] && fields[2][0]) ? fields[2][0] : '?';
    if (fields[1] && strlen(fields[1]) >= 6) g_gps.time_valid = parse_nmea_time(fields[1], &g_gps.hour, &g_gps.minute, &g_gps.second, &g_gps.centisecond);
    if (fields[9] && strlen(fields[9]) >= 6) g_gps.date_valid = parse_nmea_date(fields[9], &g_gps.day, &g_gps.month, &g_gps.year);
    
    if (fields[2] && fields[2][0] == 'A' && fields[3] && fields[4] && fields[5] && fields[6] && strlen(fields[3]) > 0 && strlen(fields[5]) > 0) {
        double lat = nmea_degmin_to_decimal(fields[3]);
        double lon = nmea_degmin_to_decimal(fields[5]);
        if (fields[4][0] == 'S') lat = -lat;
        if (fields[6][0] == 'W') lon = -lon;
        g_gps.location_valid = true; g_gps.lat = lat; g_gps.lon = lon; g_gps.fix = true; g_gps.last_fix_ms = now_ms();
    } else { 
        g_gps.location_valid = false; g_gps.fix = false; 
    }
    xSemaphoreGive(g_mutex);
}

static void gps_task(void *arg) {
    char line[192]; size_t used = 0; bool announced_uart = false;
    ESP_LOGI(TAG, "Task de recepcao GPS iniciada");
    
    while (1) {
        uint8_t ch;
        if (uart_read_bytes(GPS_UART_PORT, &ch, 1, pdMS_TO_TICKS(200)) <= 0) continue;
        
        xSemaphoreTake(g_mutex, portMAX_DELAY);
        g_gps.bytes++; g_gps.last_sentence_ms = now_ms();
        if (!g_gps.uart_seen) { 
            g_gps.uart_seen = true; 
            if (!announced_uart) { ESP_LOGI(TAG, "GPS UART data detected"); announced_uart = true; } 
        }
        xSemaphoreGive(g_mutex);
        
        if (ch == '\r') continue;
        if (ch == '\n') {
            if (used > 0) {
                line[used] = '\0';
                xSemaphoreTake(g_mutex, portMAX_DELAY); g_gps.lines++; xSemaphoreGive(g_mutex);
                
                if (strncmp(line, "$GPTXT", 6) == 0 || strncmp(line, "$GNTXT", 6) == 0) parse_gptxt(line);
                else if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) parse_gga(line);
                else if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) parse_rmc(line);
                used = 0;
            }
            continue;
        }
        if (used < sizeof(line) - 1) line[used++] = (char)ch;
        else used = 0;
    }
}

// ======================================================
// FUNÇÕES PÚBLICAS
// ======================================================
void gps_app_init(void) {
    g_mutex = xSemaphoreCreateMutex();
    
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
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    gps_status_t copy = g_gps;
    xSemaphoreGive(g_mutex);
    return copy;
}
