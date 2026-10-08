#include "can_app.h"
#include "sd_app.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sdkconfig.h"

#define CAN_TX ((gpio_num_t)CONFIG_MOVE2_CAN_TX_GPIO)
#define CAN_RX ((gpio_num_t)CONFIG_MOVE2_CAN_RX_GPIO)
#define CAN_BITRATE CONFIG_MOVE2_CAN_BITRATE

static const char *TAG = "CAN_MOD";
static QueueHandle_t can_queue;
static twai_node_handle_t s_node_hdl = NULL;
static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static can_app_stats_t s_stats;

static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t recv_buff[8] = {0};
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };

    if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK) {
        can_frame_t frame = {
            .id = rx_frame.header.id,
            .dlc = rx_frame.header.dlc,
            .mono_us = (uint64_t)esp_timer_get_time(),
            .extended = rx_frame.header.ide,
            .rtr = rx_frame.header.rtr,
        };

        if (frame.dlc > sizeof(frame.data)) {
            frame.dlc = sizeof(frame.data);
        }

        for (int i = 0; i < frame.dlc; i++) {
            frame.data[i] = recv_buff[i];
        }

        BaseType_t higher_priority_task_woken = pdFALSE;
        sd_record_t record = {.type = SD_RECORD_CAN, .mono_us = frame.mono_us};
        record.value.can.id = frame.id;
        record.value.can.dlc = rx_frame.header.dlc;
        record.value.can.extended = frame.extended;
        record.value.can.rtr = frame.rtr;
        for (unsigned i = 0; i < frame.dlc; ++i) record.value.can.data[i] = frame.data[i];
        sdcard_app_record_from_isr(&record, &higher_priority_task_woken);
        // Cloud queue is independent: MQTT delays never suppress the local record.
        bool cloud_ok = xQueueSendFromISR(can_queue, &frame, &higher_priority_task_woken) == pdTRUE;
        portENTER_CRITICAL_ISR(&s_stats_lock);
        s_stats.received++;
        if (!cloud_ok) s_stats.cloud_dropped++;
        portEXIT_CRITICAL_ISR(&s_stats_lock);
        return higher_priority_task_woken == pdTRUE;
    }

    return false;
}

can_app_stats_t can_app_get_stats(void)
{
    portENTER_CRITICAL(&s_stats_lock);
    can_app_stats_t copy = s_stats;
    portEXIT_CRITICAL(&s_stats_lock);
    if (s_node_hdl) {
        twai_node_record_t hw = {0};
        if (twai_node_get_info(s_node_hdl, NULL, &hw) == ESP_OK) copy.bus_errors = hw.bus_err_num;
    }
    return copy;
}

void can_app_init(void)
{
    if (s_node_hdl != NULL) {
        ESP_LOGW(TAG, "TWAI ja inicializado");
        return;
    }

    can_queue = xQueueCreate(20, sizeof(can_frame_t));
    if (can_queue == NULL) {
        ESP_LOGE(TAG, "Nao foi possivel criar a fila CAN");
        return;
    }

    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = CAN_TX,
        .io_cfg.rx = CAN_RX,
        .bit_timing.bitrate = CAN_BITRATE,
        .tx_queue_depth = 5,
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &s_node_hdl));

    twai_event_callbacks_t user_cbs = {
        .on_rx_done = twai_rx_cb,
    };

    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_node_hdl, &user_cbs, NULL));
    ESP_ERROR_CHECK(twai_node_enable(s_node_hdl));

    ESP_LOGI(TAG, "TWAI iniciado. TX:%d RX:%d @ %d bps", CAN_TX, CAN_RX, CAN_BITRATE);
}

bool can_app_receive(can_frame_t *frame, TickType_t ticks_to_wait)
{
    if (can_queue == NULL || frame == NULL) {
        return false;
    }

    return xQueueReceive(can_queue, frame, ticks_to_wait) == pdTRUE;
}
