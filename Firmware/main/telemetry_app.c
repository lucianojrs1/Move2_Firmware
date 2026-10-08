#include "telemetry_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include "gps_app.h"
#include "gps_format.h"
#include "sd_app.h"
#include <inttypes.h>
#include "bmi_app.h"
#include "imu_format.h"
#include "can_app.h"
#include "mqtt_app.h"
#include "sntp_app.h"
#include "esp_timer.h"

static const char *TAG = "TELEMETRIA";

// Tópicos MQTT
#define TOPIC_MQTT_CAN       "moto/can"
#define TOPIC_MQTT_SENSORES  "moto/sensores"

// Configurações de tempo
#define SENSORES_PERIOD_MS       500  // Envia GPS/IMU a cada 500ms
#define CAN_TIMEOUT_FORCAR_MS   500  // Timeout para enviar CAN incompleta

// IDs da CAN
#define ID_CAN_BAT     0x00000014U
#define ID_CAN_SOC     0x1803F3F4U
#define ID_CAN_MOTOR   0x000006A0U
#define ID_CAN_MODE    0x000006A1U

// Flags em bitmask
#define MASK_BAT    (1 << 0)
#define MASK_SOC    (1 << 1)
#define MASK_MOTOR  (1 << 2)
#define MASK_MODE   (1 << 3)
#define MASK_TODOS  (MASK_BAT | MASK_SOC | MASK_MOTOR | MASK_MODE)

typedef struct {
    can_frame_t bat;
    can_frame_t soc;
    can_frame_t motor;
    can_frame_t mode;
    uint8_t flags_recebidos;
} can_snapshot_t;

static void bytes_para_hex(char *dest, size_t dest_len, const uint8_t *data, uint8_t dlc) {
    int len = 0;
    for (int i = 0; i < dlc && len < (int)dest_len - 1; i++) {
        len += snprintf(dest + len, dest_len - len, "%02X", data[i]);
    }
}

static void telemetry_task(void *arg) {
    ESP_LOGI(TAG, "Task de Telemetria Iniciada");

    can_frame_t frame;
    can_snapshot_t snapshot = {0};

    char payload_can[768];
    char hex_bat[17], hex_soc[17], hex_motor[17], hex_mode[17];

    TickType_t ultimo_envio_sensores = xTaskGetTickCount();
    TickType_t ultimo_envio_can = xTaskGetTickCount();

    while (1) {
        uint64_t agora_ms = get_rtc_time_ms();

        // ------------ 1. CAPTURA DA CAN (Timeout curto de 20ms) ------------
        if (can_app_receive(&frame, pdMS_TO_TICKS(20))) {
            switch (frame.id) {
                case ID_CAN_BAT:
                    snapshot.bat = frame;
                    snapshot.flags_recebidos |= MASK_BAT;
                    break;

                case ID_CAN_SOC:
                    snapshot.soc = frame;
                    snapshot.flags_recebidos |= MASK_SOC;
                    break;

                case ID_CAN_MOTOR:
                    snapshot.motor = frame;
                    snapshot.flags_recebidos |= MASK_MOTOR;
                    break;

                case ID_CAN_MODE:
                    snapshot.mode = frame;
                    snapshot.flags_recebidos |= MASK_MODE;
                    break;

                default:
                    break;
            }
        }

        // ------------ 2. DISPARO DA CAN ------------
        bool can_completa = (snapshot.flags_recebidos == MASK_TODOS);
        bool can_timeout  = ((xTaskGetTickCount() - ultimo_envio_can) >= pdMS_TO_TICKS(CAN_TIMEOUT_FORCAR_MS));

        if ((can_completa || can_timeout) && snapshot.flags_recebidos != 0) {
            int len = snprintf(payload_can, sizeof(payload_can), "[");
            bool emitiu_item = false;

            // 1. FRAME BAT
            if (snapshot.flags_recebidos & MASK_BAT) {
                bytes_para_hex(hex_bat, sizeof(hex_bat), snapshot.bat.data, snapshot.bat.dlc);
                len += snprintf(payload_can + len, sizeof(payload_can) - len,
                    "%s{"
                    "\"canId\":\"0x00000014\","
                    "\"dlc\":%d,"
                    "\"data\":\"%s\","
                    "\"timestamp\":%llu"
                    "}",
                    emitiu_item ? "," : "",
                    snapshot.bat.dlc, hex_bat, agora_ms
                );
                emitiu_item = true;
            }

            // 2. FRAME SOC
            if (snapshot.flags_recebidos & MASK_SOC) {
                bytes_para_hex(hex_soc, sizeof(hex_soc), snapshot.soc.data, snapshot.soc.dlc);
                len += snprintf(payload_can + len, sizeof(payload_can) - len,
                    "%s{"
                    "\"canId\":\"0x1803F3F4\","
                    "\"dlc\":%d,"
                    "\"data\":\"%s\","
                    "\"timestamp\":%llu"
                    "}",
                    emitiu_item ? "," : "",
                    snapshot.soc.dlc, hex_soc, agora_ms
                );
                emitiu_item = true;
            }

            // 3. FRAME MOTOR
            if (snapshot.flags_recebidos & MASK_MOTOR) {
                bytes_para_hex(hex_motor, sizeof(hex_motor), snapshot.motor.data, snapshot.motor.dlc);
                len += snprintf(payload_can + len, sizeof(payload_can) - len,
                    "%s{"
                    "\"canId\":\"0x000006A0\","
                    "\"dlc\":%d,"
                    "\"data\":\"%s\","
                    "\"timestamp\":%llu"
                    "}",
                    emitiu_item ? "," : "",
                    snapshot.motor.dlc, hex_motor, agora_ms
                );
                emitiu_item = true;
            }

            // 4. FRAME MODE
            if (snapshot.flags_recebidos & MASK_MODE) {
                bytes_para_hex(hex_mode, sizeof(hex_mode), snapshot.mode.data, snapshot.mode.dlc);
                len += snprintf(payload_can + len, sizeof(payload_can) - len,
                    "%s{"
                    "\"canId\":\"0x000006A1\","
                    "\"dlc\":%d,"
                    "\"data\":\"%s\","
                    "\"timestamp\":%llu"
                    "}",
                    emitiu_item ? "," : "",
                    snapshot.mode.dlc, hex_mode, agora_ms
                );
                emitiu_item = true;
            }

            // Fecha o Array JSON
            snprintf(payload_can + len, sizeof(payload_can) - len, "]");

            if (mqtt_app_is_connected() && mqtt_app_publish(TOPIC_MQTT_CAN, payload_can)) {
                ESP_LOGD(TAG, "CAN enfileirada para %s", TOPIC_MQTT_CAN);
                ESP_LOGD("CAN_PAYLOAD", "%s", payload_can);
            }

            snapshot.flags_recebidos = 0;
            memset(&snapshot, 0, sizeof(snapshot));
            ultimo_envio_can = xTaskGetTickCount();
        }

//----------- ENVIO DOS SENSORES (INDIVIDUAIS) --------------------
if ((xTaskGetTickCount() - ultimo_envio_sensores) >= pdMS_TO_TICKS(SENSORES_PERIOD_MS)) {
    uint64_t agora_ms = get_rtc_time_ms();
    char payload_sensor[IMU_PAYLOAD_MAX];

    // ==========================================
    // 1. PAYLOAD IMU (BMI323)
    // ==========================================
    bmi323_status_t imu = bmi323_app_get_status();

    size_t imu_len=imu_format_payload(payload_sensor,sizeof(payload_sensor),&imu,
                                     (uint32_t)(esp_timer_get_time()/1000),agora_ms);
    if (imu_len && mqtt_app_is_connected() && mqtt_app_publish(TOPIC_MQTT_SENSORES,payload_sensor)) {
        ESP_LOGD(TAG,"IMU enfileirada para %s",TOPIC_MQTT_SENSORES);
    }

    // ==========================================
    // 2. PAYLOAD GPS
    // ==========================================
    gps_status_t gps = gps_app_get_status();

    size_t gps_len = gps_format_payload(payload_sensor, sizeof(payload_sensor), &gps,
                                        (uint32_t)(esp_timer_get_time() / 1000), agora_ms);
    if (gps_len && mqtt_app_is_connected()) {
        if (mqtt_app_publish(TOPIC_MQTT_SENSORES, payload_sensor)) {
            ESP_LOGD(TAG, "GPS enfileirado para %s", TOPIC_MQTT_SENSORES);
            ESP_LOGD(TAG, "Dados: %s", payload_sensor);
        }
    }

    ultimo_envio_sensores = xTaskGetTickCount();
}
    }
}

// Dedicated monitor: card writes and network timeouts cannot delay the display.
static void live_monitor_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t previous_us = (uint64_t)esp_timer_get_time();
    uint32_t previous_saved = 0, previous_dropped = 0;
    uint32_t previous_mqtt_failed = 0, previous_mqtt_expired = 0;
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        uint32_t now_ms = (uint32_t)(now_us / 1000);
        gps_status_t gps = gps_app_get_status();
        bmi323_status_t imu = bmi323_app_get_status();
        mqtt_app_stats_t mqtt = mqtt_app_get_stats();
        sdcard_status_t sd = sdcard_app_get_status();
        gps_position_state_t state = gps_position_state(&gps, now_ms);
        char lat[24] = "n/a", lon[24] = "n/a", age[24] = "n/a";
        if (state == GPS_VALID) {
            snprintf(lat, sizeof(lat), "%.6f", gps.lat);
            snprintf(lon, sizeof(lon), "%.6f", gps.lon);
        }
        if (gps.last_fix_ms || gps.location_valid)
            snprintf(age, sizeof(age), "%" PRIu32, (uint32_t)(now_ms - gps.last_fix_ms));
        ESP_LOGI("LIVE", "GPS=%s sat=%d qualidade=%d RMC=%c lat=%s lon=%s age_ms=%s sentencas=%" PRIu32
                 " RMC/GGA=%" PRIu32 "/%" PRIu32,
                 gps_position_state_name(state), gps.sats_used, gps.fix_quality,
                 gps.rmc_status ? gps.rmc_status : '?', lat, lon, age, gps.lines,
                 gps.rmc_count, gps.gga_count);
        bool imu_fresh = imu.i2c_ok && (uint32_t)(now_ms - imu.last_update_ms) < 100;
        ESP_LOGI("LIVE", "IMU=%s acc[m/s2]=%.2f,%.2f,%.2f gyro[deg/s]=%.2f,%.2f,%.2f age_ms=%" PRIu32,
                 imu_fresh ? "valida" : "indisponivel/calibrando",
                 imu.acc_x * (9.80665f / 4096.0f), imu.acc_y * (9.80665f / 4096.0f),
                 imu.acc_z * (9.80665f / 4096.0f),
                 imu.gyr_x / 65.536f, imu.gyr_y / 65.536f, imu.gyr_z / 65.536f,
                 (uint32_t)(now_ms - imu.last_update_ms));
        char speed[24]="n/a";
        if (state==GPS_VALID && gps.metrics.speed_valid) snprintf(speed,sizeof(speed),"%.1f",gps.metrics.speed_kmh);
        ESP_LOGI("LIVE", "Viagem tracking=%d velocidade=%s km/h distancia=%.1f m; movimento valido=%d roll=%.1f pitch=%.1f; eventos freio/impacto/queda=%" PRIu32 "/%" PRIu32 "/%" PRIu32,
                 state==GPS_VALID && gps.metrics.tracking && gps.gga_count && (uint32_t)(now_ms-gps.last_gga_ms)<3000,
                 speed,gps.metrics.trip_distance_m,
                 imu_fresh && imu.motion.valid,imu.motion.roll_deg,imu.motion.pitch_deg,
                 imu.motion.braking_count,imu.motion.impact_count,imu.motion.fall_count);
        uint32_t drops = 0;
        for (unsigned i=0;i<SD_RECORD_COUNT;++i) drops+=sd.dropped[i];
        double saved_hz = (double)(uint32_t)(sd.committed - previous_saved) * 1000000.0 /
                          (double)(now_us - previous_us);
        ESP_LOGI("LIVE", "MQTT=%s enviados_fila=%" PRIu32 " falhas=%" PRIu32
                 " fila_bytes=%" PRIu32 "/%d cheias=%" PRIu32 " expiradas=%" PRIu32
                 " SD=%s fila=%" PRIu32 " gravados/s=%.1f perdas_novas=%" PRIu32,
                 mqtt.connected ? "conectado" : "offline", mqtt.submitted, mqtt.failed,
                 mqtt.outbox_bytes, MQTT_APP_OUTBOX_LIMIT, mqtt.full, mqtt.expired,
                 sd.mounted ? "montado" : "offline", sd.queued, saved_hz,
                 (uint32_t)(drops - previous_dropped));
        if (drops != previous_dropped)
            ESP_LOGW("LIVE", "SD nao acompanhou a coleta: perdas C/I/G=%" PRIu32 "/%" PRIu32 "/%" PRIu32
                     " M/T=%" PRIu32 "/%" PRIu32,
                     sd.dropped[0], sd.dropped[1], sd.dropped[2],sd.dropped[SD_RECORD_MOTION],sd.dropped[SD_RECORD_TRIP]);
        if (mqtt.failed != previous_mqtt_failed || mqtt.expired != previous_mqtt_expired)
            ESP_LOGW("LIVE", "MQTT com perdas: falhas_novas=%" PRIu32 " expiradas_novas=%" PRIu32
                     "; confira fila/rede. Gravacao SD tem contadores independentes.",
                     (uint32_t)(mqtt.failed - previous_mqtt_failed),
                     (uint32_t)(mqtt.expired - previous_mqtt_expired));
        previous_mqtt_failed = mqtt.failed; previous_mqtt_expired = mqtt.expired;
        previous_dropped = drops; previous_saved = sd.committed; previous_us = now_us;
    }
}

void telemetry_app_init(void) {
    if (xTaskCreate(telemetry_task, "telemetry_task", 6144, NULL, 4, NULL) != pdPASS)
        ESP_LOGE(TAG, "Falha ao criar task de telemetria");
    if (xTaskCreate(live_monitor_task, "live_monitor", 4096, NULL, 2, NULL) != pdPASS)
        ESP_LOGE(TAG, "Falha ao criar monitor ao vivo");
}
