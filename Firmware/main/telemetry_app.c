#include "telemetry_app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include "gps_app.h"
#include "bmi_app.h"
#include "can_app.h"
#include "mqtt_app.h"
#include "sntp_app.h"

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

    ESP_LOGI(TAG, "Aguardando sincronizacao do relogio via rede...");

    // Aguarda até o NTP sincronizar o relógio ou sai após 15 segundos
    int tentativas = 0;
    while (!sntp_app_is_synced() && tentativas < 30) {
        vTaskDelay(pdMS_TO_TICKS(500));
        tentativas++;
    }

    if (sntp_app_is_synced()) {
        ESP_LOGI(TAG, "Relógio pronto. Iniciando telemetria...");
    } else {
        ESP_LOGW(TAG, "Timeout do NTP. Prosseguindo com tempo relativo local.");
    }
    ////////////////////////////////////

    while (1) {
        uint32_t agora_ms = get_rtc_time_ms();

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
                    "\"timestamp\":%lu"
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
                    "\"timestamp\":%lu"
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
                    "\"timestamp\":%lu"
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
                    "\"timestamp\":%lu"
                    "}",
                    emitiu_item ? "," : "",
                    snapshot.mode.dlc, hex_mode, agora_ms
                );
                emitiu_item = true;
            }

            // Fecha o Array JSON
            snprintf(payload_can + len, sizeof(payload_can) - len, "]");

            if (mqtt_app_is_connected()) {
                mqtt_app_publish(TOPIC_MQTT_CAN, payload_can);
                ESP_LOGI(TAG, "CAN enviada para %s", TOPIC_MQTT_CAN);
                ESP_LOGI("CAN_PAYLOAD", "%s", payload_can);
            }

            snapshot.flags_recebidos = 0;
            memset(&snapshot, 0, sizeof(snapshot));
            ultimo_envio_can = xTaskGetTickCount();
        }

//----------- ENVIO DOS SENSORES (INDIVIDUAIS) --------------------
if ((xTaskGetTickCount() - ultimo_envio_sensores) >= pdMS_TO_TICKS(SENSORES_PERIOD_MS)) {
    uint64_t agora_ms = get_rtc_time_ms();
    char payload_sensor[384];

// ==========================================
    // 1. PAYLOAD IMU (BMI323)
    // ==========================================
    bmi323_status_t imu = bmi323_app_get_status();

    // Conversão do Acelerômetro (Escala 8G) para Força G (1G = 4096)
    float acc_x = imu.acc_x / 4096.0f; 
    float acc_y = imu.acc_y / 4096.0f;
    float acc_z = imu.acc_z / 4096.0f;

    // Conversão do Giroscópio (Escala 2000 DPS). 
    // No BMI323 a 2000DPS, o divisor natural para chegar em graus/s é 16.4.
    float gyr_x = imu.gyr_x / 16.4f;
    float gyr_y = imu.gyr_y / 16.4f;
    float gyr_z = imu.gyr_z / 16.4f;
    //float temp_c = imu.temp_mdeg_c / 1000.0f; // mili-graus para Celsius

    snprintf(payload_sensor, sizeof(payload_sensor),
        "{"
        "\"sensorId\":\"imu\","
        "\"sensorType\":\"Imu\","
        "\"value\":{"
          "\"accelerometer\":{\"acc_x\":%.2f,\"acc_y\":%.2f,\"acc_z\":%.2f},"
          "\"gyroscope\":{\"gyr_x\":%.2f,\"gyr_y\":%.2f,\"gyr_z\":%.2f}"
         // "\"temperature\":%.1f"
        "},"
        //"\"unit\":\"m/s², deg/s\","
        "\"timestamp\":%llu"
        "}",
        acc_x, acc_y, acc_z,
        gyr_x, gyr_y, gyr_z,
        //temp_c,
        agora_ms
    );


    if (mqtt_app_is_connected()) {
        mqtt_app_publish(TOPIC_MQTT_SENSORES, payload_sensor);
        ESP_LOGI(TAG, "IMU enviada para %s", TOPIC_MQTT_SENSORES);
        ESP_LOGI(TAG, "Dados: %s", payload_sensor);
    }

    // ==========================================
    // 2. PAYLOAD GPS
    // ==========================================
    gps_status_t gps = gps_app_get_status();

    snprintf(payload_sensor, sizeof(payload_sensor),
        "{"
        "\"sensorId\":\"gps_modulo\","
        "\"sensorType\":\"gps\","
        "\"value\":{"
          "\"latitude\":%.6f,"
          "\"longitude\":%.6f,"
         // "\"altitude\":%.1f,"
          //"\"speed\":%.1f,"
          "\"satellites\":%d"
        "},"
        "\"unit\":\"°\","
        "\"timestamp\":%llu"
        "}",
        gps.location_valid ? gps.lat : 0.0,
        gps.location_valid ? gps.lon : 0.0,
        //gps.altitude,  // Adicione gps.altitude no seu gps_app se não houver
        //gps.speed,     // Adicione gps.speed no seu gps_app se não houver
        gps.sats_used,
        agora_ms
    );

    if (mqtt_app_is_connected()) {
        mqtt_app_publish(TOPIC_MQTT_SENSORES, payload_sensor);
        ESP_LOGI(TAG, "GPS enviado para %s", TOPIC_MQTT_SENSORES);
        ESP_LOGI(TAG, "Dados: %s", payload_sensor);
    }

    ultimo_envio_sensores = xTaskGetTickCount();
}
    }
}

void telemetry_app_init(void) {
    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 4, NULL);
}
