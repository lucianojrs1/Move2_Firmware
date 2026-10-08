#include "lte_app.h"

#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif_ppp.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_modem_api.h"

#define MODEM_UART_PORT     UART_NUM_1
#define MODEM_TX_PIN        GPIO_NUM_26
#define MODEM_RX_PIN        GPIO_NUM_27
#define MODEM_PWRKEY_PIN    GPIO_NUM_4
#define MODEM_POWERON_PIN   GPIO_NUM_12
#define MODEM_RESET_PIN     GPIO_NUM_5
#define MODEM_DTR_PIN       GPIO_NUM_25
#define MODEM_RESET_LEVEL   1

#define LTE_CONNECTED_BIT       BIT0
#define LTE_DISCONNECTED_BIT    BIT1

#define LTE_REGISTRATION_TIMEOUT_MS 120000
#define LTE_REGISTRATION_POLL_MS    5000
#define PPP_START_RETRY_COUNT       5

static const char *TAG = "LTE_APP";

static EventGroupHandle_t s_lte_events;
static esp_netif_t *s_ppp_netif;
static esp_modem_dce_t *s_dce;
static bool s_handlers_registered;
static int s_last_rssi = 99;
static int s_last_ber = 99;

static esp_err_t init_netif_and_events(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    return ESP_OK;
}

static const char *registration_state_name(int state)
{
    switch (state) {
    case 0:
        return "not registered";
    case 1:
        return "registered home";
    case 2:
        return "searching";
    case 3:
        return "registration denied";
    case 4:
        return "unknown";
    case 5:
        return "registered roaming";
    case 8:
        return "emergency only";
    default:
        return "other";
    }
}

static bool is_registered_for_data(int state)
{
    return state == 1 || state == 5;
}

static void log_optional_modem_info(esp_modem_dce_t *dce)
{
    char name[ESP_MODEM_C_API_STR_BUF_SIZE] = {0};
    char imei[ESP_MODEM_C_API_STR_BUF_SIZE] = {0};
    int radio_state = -1;

    if (esp_modem_get_module_name(dce, name) == ESP_OK) {
        ESP_LOGI(TAG, "Modem module: %s", name);
    }
    if (esp_modem_get_imei(dce, imei) == ESP_OK) {
        ESP_LOGI(TAG, "Modem IMEI: %s", imei);
    }
    if (esp_modem_get_radio_state(dce, &radio_state) == ESP_OK) {
        ESP_LOGI(TAG, "Initial radio state: %d", radio_state);
    }
}

static bool wait_for_lte_registration(esp_modem_dce_t *dce)
{
    int64_t start_ms = esp_timer_get_time() / 1000;

    esp_err_t err = esp_modem_set_echo(dce, false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not disable modem echo: %s", esp_err_to_name(err));
    }

    err = esp_modem_config_mobile_termination_error(dce, 2);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not enable verbose CME errors: %s", esp_err_to_name(err));
    }

    esp_modem_PdpContext_t pdp = {
        .context_id = 1,
        .protocol_type = "IP",
        .apn = CONFIG_MOVE2_LTE_APN,
    };
    err = esp_modem_set_pdp_context(dce, &pdp);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not set PDP context before registration: %s", esp_err_to_name(err));
    }

    err = esp_modem_set_radio_state(dce, 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not force radio full functionality: %s", esp_err_to_name(err));
    }

    err = esp_modem_config_network_registration_urc(dce, 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not enable CEREG URC: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Waiting up to %d seconds for LTE registration",
             LTE_REGISTRATION_TIMEOUT_MS / 1000);

    while ((esp_timer_get_time() / 1000) - start_ms < LTE_REGISTRATION_TIMEOUT_MS) {
        int registration_state = -1;
        int attachment_state = -1;
        int rssi = 99;
        int ber = 99;
        int act = -1;
        char operator_name[ESP_MODEM_C_API_STR_BUF_SIZE] = {0};
        bool got_registration = false;
        bool got_signal = false;

        if (esp_modem_get_network_registration_state(dce, &registration_state) == ESP_OK) {
            got_registration = true;
        }

        esp_modem_get_network_attachment_state(dce, &attachment_state);

        if (esp_modem_get_signal_quality(dce, &rssi, &ber) == ESP_OK) {
            got_signal = true;
            s_last_rssi = rssi;
            s_last_ber = ber;
        }

        if (is_registered_for_data(registration_state)) {
            esp_modem_get_operator_name(dce, operator_name, &act);
        }

        ESP_LOGI(TAG, "LTE status: reg=%d(%s) attach=%d rssi=%d ber=%d operator=%s act=%d",
                 got_registration ? registration_state : -1,
                 got_registration ? registration_state_name(registration_state) : "read failed",
                 attachment_state, got_signal ? rssi : -1, got_signal ? ber : -1,
                 operator_name[0] ? operator_name : "-", act);

        if (got_registration && is_registered_for_data(registration_state)) {
            err = esp_modem_set_network_attachment_state(dce, 1);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "CGATT attach request failed: %s", esp_err_to_name(err));
            }
            esp_modem_config_network_registration_urc(dce, 0);
            return true;
        }

        if (got_registration && registration_state == 3) {
            ESP_LOGE(TAG, "LTE registration denied by network/operator");
            esp_modem_config_network_registration_urc(dce, 0);
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(LTE_REGISTRATION_POLL_MS));
    }

    esp_modem_config_network_registration_urc(dce, 0);
    ESP_LOGE(TAG, "LTE did not register before timeout");
    return false;
}

static bool start_ppp_data_mode_with_retry(esp_modem_dce_t *dce)
{
    for (int attempt = 1; attempt <= PPP_START_RETRY_COUNT; attempt++) {
        ESP_LOGI(TAG, "Switching modem to PPP data mode, attempt %d/%d",
                 attempt, PPP_START_RETRY_COUNT);
        esp_err_t err = esp_modem_set_mode(dce, ESP_MODEM_MODE_DATA);
        if (err == ESP_OK) {
            return true;
        }

        ESP_LOGW(TAG, "PPP data mode attempt failed: %s", esp_err_to_name(err));
        esp_modem_hang_up(dce);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    return false;
}

static void pulse_modem_power_key(void)
{
    // GPIO12 also powers peripherals: never pulse/reset it during local recording.
    gpio_set_direction(MODEM_POWERON_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MODEM_POWERON_PIN, 1);

    gpio_reset_pin(MODEM_RESET_PIN);
    gpio_set_direction(MODEM_RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(2600));
    gpio_set_level(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);

    gpio_reset_pin(MODEM_DTR_PIN);
    gpio_set_direction(MODEM_DTR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MODEM_DTR_PIN, 0);

    gpio_reset_pin(MODEM_PWRKEY_PIN);
    gpio_set_direction(MODEM_PWRKEY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MODEM_PWRKEY_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(MODEM_PWRKEY_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(MODEM_PWRKEY_PIN, 0);

    ESP_LOGI(TAG, "Modem power sequence done, waiting for boot");
    vTaskDelay(pdMS_TO_TICKS(10000));
}

static void on_ppp_changed(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "PPP status event: %" PRId32, event_id);
    if (event_id == NETIF_PPP_ERRORUSER || event_id == NETIF_PPP_ERRORCONNECT ||
        event_id == NETIF_PPP_ERRORAUTHFAIL || event_id == NETIF_PPP_ERRORPEERDEAD) {
        xEventGroupClearBits(s_lte_events, LTE_CONNECTED_BIT);
        xEventGroupSetBits(s_lte_events, LTE_DISCONNECTED_BIT);
    }
}

static void on_ip_event(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_PPP_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        esp_netif_dns_info_t dns_info;

        ESP_LOGI(TAG, "LTE PPP got IP");
        ESP_LOGI(TAG, "IP      : " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Netmask : " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "Gateway : " IPSTR, IP2STR(&event->ip_info.gw));

        if (esp_netif_get_dns_info(event->esp_netif, ESP_NETIF_DNS_MAIN, &dns_info) == ESP_OK) {
            ESP_LOGI(TAG, "DNS     : " IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        }

        xEventGroupClearBits(s_lte_events, LTE_DISCONNECTED_BIT);
        xEventGroupSetBits(s_lte_events, LTE_CONNECTED_BIT);
    } else if (event_id == IP_EVENT_PPP_LOST_IP) {
        ESP_LOGW(TAG, "LTE PPP lost IP");
        xEventGroupClearBits(s_lte_events, LTE_CONNECTED_BIT);
        xEventGroupSetBits(s_lte_events, LTE_DISCONNECTED_BIT);
    }
}

esp_err_t lte_app_init(void)
{
    if (s_dce != NULL) {
        if (lte_app_is_connected()) return ESP_OK;
        ESP_LOGI(TAG, "Restabelecendo registro LTE/PPP...");
        if (esp_modem_get_mode(s_dce) == ESP_MODEM_MODE_UNDEF) {
            ESP_RETURN_ON_ERROR(esp_modem_set_mode(s_dce, ESP_MODEM_MODE_DETECT), TAG,
                                "Could not detect modem mode");
        }
        // esp_modem rejects COMMAND -> COMMAND; registration retries already in
        // command mode must proceed directly to AT commands.
        if (esp_modem_get_mode(s_dce) != ESP_MODEM_MODE_COMMAND) {
            ESP_RETURN_ON_ERROR(esp_modem_set_mode(s_dce, ESP_MODEM_MODE_COMMAND), TAG,
                                "Could not return to command mode");
        }
        xEventGroupClearBits(s_lte_events, LTE_CONNECTED_BIT | LTE_DISCONNECTED_BIT);
        if (!wait_for_lte_registration(s_dce)) return ESP_ERR_TIMEOUT;
        return start_ppp_data_mode_with_retry(s_dce) ? ESP_OK : ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(init_netif_and_events(), TAG, "network stack init failed");

    if (s_lte_events == NULL) {
        s_lte_events = xEventGroupCreate();
        if (s_lte_events == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (!s_handlers_registered) {
        ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &on_ip_event, NULL),
                            TAG, "IP event handler register failed");
        ESP_RETURN_ON_ERROR(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID,
                                                       &on_ppp_changed, NULL),
                            TAG, "PPP event handler register failed");
        s_handlers_registered = true;
    }

    ESP_LOGI(TAG, "Starting LTE stack. APN=%s UART%d TX=%d RX=%d",
             CONFIG_MOVE2_LTE_APN, MODEM_UART_PORT, MODEM_TX_PIN, MODEM_RX_PIN);

    pulse_modem_power_key();

    esp_netif_config_t ppp_config = ESP_NETIF_DEFAULT_PPP();
    s_ppp_netif = esp_netif_new(&ppp_config);
    if (s_ppp_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (strlen(CONFIG_MOVE2_LTE_APN_USER) > 0) {
        esp_err_t err = esp_netif_ppp_set_auth(s_ppp_netif, NETIF_PPP_AUTHTYPE_PAP,
                                              CONFIG_MOVE2_LTE_APN_USER, CONFIG_MOVE2_LTE_APN_PASS);
        if (err != ESP_OK) {
            esp_netif_destroy(s_ppp_netif); s_ppp_netif = NULL;
            return err;
        }
    }

    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(CONFIG_MOVE2_LTE_APN);
    dce_config.protocol_type = "IP";
    dce_config.context_id = 1;

    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_config.uart_config.port_num = MODEM_UART_PORT;
    dte_config.uart_config.tx_io_num = MODEM_TX_PIN;
    dte_config.uart_config.rx_io_num = MODEM_RX_PIN;
    dte_config.uart_config.rts_io_num = UART_PIN_NO_CHANGE;
    dte_config.uart_config.cts_io_num = UART_PIN_NO_CHANGE;
    dte_config.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_NONE;
    dte_config.uart_config.baud_rate = 115200;
    dte_config.uart_config.rx_buffer_size = 2048;
    dte_config.uart_config.tx_buffer_size = 1024;
    dte_config.uart_config.event_queue_size = 30;
    dte_config.task_stack_size = 4096;
    dte_config.task_priority = 5;
    dte_config.dte_buffer_size = 1024;

    s_dce = esp_modem_new(&dte_config, &dce_config, s_ppp_netif);
    if (s_dce == NULL) {
        ESP_LOGE(TAG, "Could not create esp_modem DCE");
        esp_netif_destroy(s_ppp_netif); s_ppp_netif = NULL;
        return ESP_FAIL;
    }

    log_optional_modem_info(s_dce);

    bool pin_ready = false;
    if (esp_modem_read_pin(s_dce, &pin_ready) == ESP_OK) {
        ESP_LOGI(TAG, "SIM status: %s", pin_ready ? "ready" : "PIN required");
    } else {
        ESP_LOGW(TAG, "Could not read SIM PIN status");
    }

    if (!wait_for_lte_registration(s_dce)) {
        ESP_LOGE(TAG, "LTE is not ready for PPP. Check antenna, SIM plan, APN and coverage.");
        return ESP_ERR_TIMEOUT;
    }

    if (!start_ppp_data_mode_with_retry(s_dce)) {
        ESP_LOGE(TAG, "Could not switch modem to PPP data mode after retries");
        return ESP_FAIL;
    }

    return ESP_OK;
}

bool lte_app_wait_connected(uint32_t timeout_ms)
{
    if (s_lte_events == NULL) {
        return false;
    }

    EventBits_t bits = xEventGroupWaitBits(s_lte_events,
                                           LTE_CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(timeout_ms));
    return (bits & LTE_CONNECTED_BIT) != 0;
}

bool lte_app_is_connected(void)
{
    if (s_lte_events == NULL) {
        return false;
    }
    return (xEventGroupGetBits(s_lte_events) & LTE_CONNECTED_BIT) != 0;
}

esp_netif_t *lte_app_get_netif(void)
{
    return s_ppp_netif;
}

void lte_app_get_signal_quality(int *rssi, int *ber)
{
    if (rssi != NULL) {
        *rssi = s_last_rssi;
    }
    if (ber != NULL) {
        *ber = s_last_ber;
    }
}
