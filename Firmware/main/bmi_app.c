#include "bmi_app.h"
#include "sd_app.h"
#include "gps_app.h"
#include "gps_format.h"
#include "imu_calibration.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"

#define BMI_I2C_SDA GPIO_NUM_32
#define BMI_I2C_SCL GPIO_NUM_18
#define BMI_I2C_ADDR 0x68 // SDO ligado ao GND; CSB ligado a 3.3 V
#define BMI_I2C_FREQ_HZ 400000
#define BMI_I2C_TIMEOUT_MS 100
#define BMI_I2C_DUMMY_BYTES 2
#define BMI_MAX_READ_BYTES 18
#define BMI_ODR_HZ CONFIG_MOVE2_IMU_ODR_HZ
#define BMI_CONF (0x4020 | (BMI_ODR_HZ == 400 ? 0x0A : BMI_ODR_HZ == 200 ? 0x09 : 0x08))

static const char *TAG = "BMI323";
static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t i2c_dev;
static bmi323_status_t g_bmi = {0};
static SemaphoreHandle_t g_bmi_mutex = NULL;
static motion_state_t motion;
typedef struct {
    uint64_t mono_us;
    uint32_t sequence;
    float acc[3], gyro[3];
    int16_t temperature;
} motion_input_t;
static QueueHandle_t motion_queue;
static uint32_t motion_update_ms;



// --- Funções Privadas de Comunicação I2C ---
static esp_err_t bmi_i2c_read_regs(uint8_t reg_addr, uint8_t *data, size_t data_len) {
    if (data == NULL || data_len == 0 || data_len > BMI_MAX_READ_BYTES || data_len % 2 != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t rx_data[BMI_I2C_DUMMY_BYTES + BMI_MAX_READ_BYTES];

    // Endereco do registrador sem bit SPI; repeated START e dois dummy bytes.
    esp_err_t ret = i2c_master_transmit_receive(i2c_dev, &reg_addr, 1,
                                               rx_data, data_len + BMI_I2C_DUMMY_BYTES,
                                               BMI_I2C_TIMEOUT_MS);
    if (ret == ESP_OK) {
        memcpy(data, rx_data + BMI_I2C_DUMMY_BYTES, data_len);
    }
    return ret;
}

static esp_err_t bmi_i2c_write_reg(uint8_t reg_addr, uint16_t data) {
    uint8_t tx_data[3];
    tx_data[0] = reg_addr;
    tx_data[1] = data & 0xFF;           
    tx_data[2] = (data >> 8) & 0xFF;    

    return i2c_master_transmit(i2c_dev, tx_data, sizeof(tx_data), BMI_I2C_TIMEOUT_MS);
}

static int16_t signed_word(const uint8_t *data) {
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}
static int16_t corrected(int16_t raw, int32_t offset) {
    int32_t value = (int32_t)raw - offset;
    if (value > INT16_MAX) return INT16_MAX;
    if (value < INT16_MIN) return INT16_MIN;
    return (int16_t)value;
}
static void poll_timer(void *arg) { xTaskNotifyGive((TaskHandle_t)arg); }
static void publish_status(const bmi323_status_t *status) {
    xSemaphoreTake(g_bmi_mutex, portMAX_DELAY);
    motion_sample_t latest_motion = g_bmi.motion;
    g_bmi = *status;
    g_bmi.motion = latest_motion;
    xSemaphoreGive(g_bmi_mutex);
}
static void motion_task(void *arg) {
    const motion_config_t motion_config = {
        .axes={CONFIG_MOVE2_BODY_X_AXIS,CONFIG_MOVE2_BODY_Y_AXIS,CONFIG_MOVE2_BODY_Z_AXIS},
        .braking_mps2=CONFIG_MOVE2_BRAKING_THRESHOLD_TENTHS/10.0f,
        .impact_g=CONFIG_MOVE2_IMPACT_THRESHOLD_TENTHS_G/10.0f,
        .fall_deg=CONFIG_MOVE2_FALL_TILT_DEG,
    };
    bool motion_enabled=motion_init(&motion,&motion_config);
    if (!motion_enabled) ESP_LOGE(TAG,"Mapeamento dos eixos invalido; calculos de movimento desabilitados.");
    else ESP_LOGI(TAG,"Movimento experimental: eixos frente/esquerda/cima=%d/%d/%d",motion_config.axes[0],motion_config.axes[1],motion_config.axes[2]);
    gps_status_t gps = {0};
    uint32_t last_gps_ms = 0, previous_sequence = 0;
    bool have_sequence = false;
    for (;;) {
        motion_input_t input;
        if (xQueueReceive(motion_queue, &input, pdMS_TO_TICKS(100)) != pdTRUE) {
            xSemaphoreTake(g_bmi_mutex, portMAX_DELAY);
            g_bmi.motion.valid = false;
            xSemaphoreGive(g_bmi_mutex);
            motion.initialized = false;
            continue;
        }
        if (!motion_enabled) continue;
        if (have_sequence && input.sequence-previous_sequence != 1) motion.initialized = false;
        previous_sequence = input.sequence; have_sequence = true;
        uint32_t now_ms = (uint32_t)(input.mono_us/1000);
        if ((uint32_t)(now_ms-last_gps_ms)>=100) {
            gps=gps_app_get_status(); last_gps_ms=now_ms;
        }
        bool speed_valid=gps_position_state(&gps,now_ms)==GPS_VALID && gps.metrics.speed_valid;
        bool completed=motion_update(&motion,input.acc,input.gyro,input.temperature,input.mono_us,
                                     speed_valid,gps.metrics.speed_kmh);
        if (completed || !motion.sample.valid) {
            xSemaphoreTake(g_bmi_mutex, portMAX_DELAY);
            g_bmi.motion=motion.sample; motion_update_ms=now_ms;
            xSemaphoreGive(g_bmi_mutex);
        }
        if (completed) {
            sd_record_t summary={.type=SD_RECORD_MOTION,.mono_us=input.mono_us};
            summary.value.motion=motion.sample;
            sdcard_app_record(&summary);
        }
    }
}
static void bmi323_task(void *pvParameters) {
    esp_timer_handle_t timer;
    esp_timer_create_args_t timer_cfg = {
        .callback = poll_timer, .arg = xTaskGetCurrentTaskHandle(), .name = "imu_poll",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_cfg, &timer));
    bmi323_status_t status = {0};
    for (;;) {
        uint8_t id[2], config[4];
        esp_err_t err = bmi_i2c_read_regs(0x00, id, sizeof(id));
        if (err == ESP_OK && id[0] != 0x43) err = ESP_ERR_INVALID_RESPONSE;
        if (err == ESP_OK) err = bmi_i2c_write_reg(0x20, BMI_CONF);
        if (err == ESP_OK) err = bmi_i2c_write_reg(0x21, BMI_CONF);
        vTaskDelay(pdMS_TO_TICKS(100));
        if (err == ESP_OK) err = bmi_i2c_read_regs(0x20, config, sizeof(config));
        if (err == ESP_OK && ((uint16_t)signed_word(config) != BMI_CONF ||
                             (uint16_t)signed_word(config + 2) != BMI_CONF))
            err = ESP_ERR_INVALID_RESPONSE;
        if (err != ESP_OK) {
            status.i2c_ok = false; status.read_errors++;
            publish_status(&status);
            ESP_LOGW(TAG, "BMI323 em 0x68: %s; nova tentativa", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }
        status.chip_id = (uint16_t)signed_word(id);
        status.i2c_ok = false;
        int32_t offsets[6] = {0};
        imu_calibration_t calibration = {0};
        float acc_scale = 1;
        unsigned errors = 0;
        bool calibrated = false, have_sensor_time = false;
        uint32_t last_sensor_time = 0;
        uint16_t pending = 0;
        int64_t started = esp_timer_get_time(), last_sample = started;
        const unsigned calibration_samples = 2 * BMI_ODR_HZ;
        ESP_LOGI(TAG, "BMI323 detectado via I2C: chip ID 0x%04X; ODR=%d Hz", status.chip_id, BMI_ODR_HZ);
        ESP_LOGI(TAG, "Calibrando: mantenha a moto parada por 3 s. SD ja coleta dados brutos.");
        ulTaskNotifyTake(pdTRUE, 0);
        ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000000 / (2 * BMI_ODR_HZ)));
        for (;;) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            uint8_t flags[2], data[18];
            err = bmi_i2c_read_regs(0x02, flags, sizeof(flags));
            if (err == ESP_OK) {
                status.status_reg = (uint16_t)signed_word(flags);
                pending |= status.status_reg & 0xC0; // Clear-on-read DRDY flags.
                if (pending != 0xC0) {
                    if (esp_timer_get_time() - last_sample > 1000000) break;
                    continue;
                }
                err = bmi_i2c_read_regs(0x03, data, sizeof(data));
            }
            if (err != ESP_OK) {
                status.i2c_ok = false; status.read_errors++;
                publish_status(&status);
                if (++errors >= 10) break;
                continue;
            }
            errors = 0; pending = 0;
            int64_t now = esp_timer_get_time();
            uint32_t sensor_time = (uint32_t)data[14] | ((uint32_t)data[15] << 8) |
                                   ((uint32_t)data[16] << 16) | ((uint32_t)data[17] << 24);
            if (have_sensor_time) {
                uint32_t delta = sensor_time - last_sensor_time;
                uint32_t expected = 25600 / BMI_ODR_HZ;
                if (delta > expected + expected / 2 && delta < 25600)
                    status.estimated_missed += (delta + expected / 2) / expected - 1;
            }
            have_sensor_time = true; last_sensor_time = sensor_time; last_sample = now;
            sd_record_t record = {.type = SD_RECORD_IMU, .mono_us = (uint64_t)now};
            for (unsigned i = 0; i < 6; ++i) record.value.imu.axes[i] = signed_word(data + i * 2);
            record.value.imu.temperature = signed_word(data + 12);
            record.value.imu.sensor_time = sensor_time;
            record.value.imu.status = status.status_reg;
            record.value.imu.acc_conf = BMI_CONF; record.value.imu.gyr_conf = BMI_CONF;
            record.value.imu.calibrated = calibrated;
            sdcard_app_record(&record);
            status.samples++;
            if (!calibrated && now - started >= 1000000) {
                imu_calibration_add(&calibration, record.value.imu.axes);
                if (calibration.count == calibration_samples) {
                    calibrated = imu_calibration_finish(&calibration, offsets, &acc_scale);
                    if (!calibrated) {
                        calibration = (imu_calibration_t){0};
                        ESP_LOGW(TAG, "Movimento na calibracao; aguardando janela estavel de 2 s.");
                    } else {
                        ESP_LOGI(TAG, "Calibracao concluida. offsets=%ld,%ld,%ld,%ld,%ld,%ld",
                                 (long)offsets[0], (long)offsets[1], (long)offsets[2],
                                 (long)offsets[3], (long)offsets[4], (long)offsets[5]);
                        ESP_LOGI(TAG, "Escala do acelerometro para movimento=%.6f; inclinacao preservada", acc_scale);
                    }
                }
            }
            status.acc_x = corrected(record.value.imu.axes[0], offsets[0]);
            status.acc_y = corrected(record.value.imu.axes[1], offsets[1]);
            status.acc_z = corrected(record.value.imu.axes[2], offsets[2]);
            status.gyr_x = corrected(record.value.imu.axes[3], offsets[3]);
            status.gyr_y = corrected(record.value.imu.axes[4], offsets[4]);
            status.gyr_z = corrected(record.value.imu.axes[5], offsets[5]);
            status.last_update_ms = (uint32_t)(now / 1000); status.i2c_ok = calibrated;
            if (calibrated) {
                motion_input_t input={.mono_us=(uint64_t)now,.sequence=status.samples,
                                      .temperature=record.value.imu.temperature};
                for (unsigned i=0;i<3;++i) {
                    input.acc[i]=record.value.imu.axes[i]*(9.80665f/4096.0f)*acc_scale;
                    input.gyro[i]=corrected(record.value.imu.axes[i+3],offsets[i+3])/65.536f;
                }
                if (xQueueSend(motion_queue,&input,0)!=pdTRUE) status.motion_queue_dropped++;
            }
            publish_status(&status);
        }
        esp_timer_stop(timer);
        status.i2c_ok = false; publish_status(&status);
        ESP_LOGW(TAG, "IMU sem dados novos ou com falhas; reinicializando.");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void bmi323_app_init(void) {
    g_bmi_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(g_bmi_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    motion_queue = xQueueCreate(32, sizeof(motion_input_t));
    ESP_ERROR_CHECK(motion_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(motion_task, "motion_task", 4096, NULL, 3, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    i2c_master_bus_config_t buscfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BMI_I2C_SDA,
        .scl_io_num = BMI_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMI_I2C_ADDR,
        .scl_speed_hz = BMI_I2C_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&buscfg, &i2c_bus));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &devcfg, &i2c_dev));
    ESP_LOGI(TAG, "I2C: SDA=%d SCL=%d @ %d Hz, endereco=0x%02X",
             BMI_I2C_SDA, BMI_I2C_SCL, BMI_I2C_FREQ_HZ, BMI_I2C_ADDR);

    vTaskDelay(pdMS_TO_TICKS(50));

    // Cria a Task que ficará rodando no background eternamente
    BaseType_t created = xTaskCreate(bmi323_task, "bmi_task", 4096, NULL, 5, NULL);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

bmi323_status_t bmi323_app_get_status(void) {
    bmi323_status_t copy = {0};
    if (g_bmi_mutex != NULL) {
        xSemaphoreTake(g_bmi_mutex, portMAX_DELAY);
        copy = g_bmi;
        if (!copy.i2c_ok || (uint32_t)(esp_timer_get_time()/1000-motion_update_ms)>1000)
            copy.motion.valid=false;
        xSemaphoreGive(g_bmi_mutex);
    }
    return copy;
}
