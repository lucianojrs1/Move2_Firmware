#include "mqtt_app.h"

#include <string.h>
#include <stdatomic.h>

#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "sdkconfig.h"

static const char *TAG = "MQTT_APP";
static const char *s_wss_alpn_protos[] = {"http/1.1", NULL};
static esp_mqtt_client_handle_t mqtt_client = NULL;
static atomic_bool mqtt_conectado;
static atomic_uint s_submitted, s_failed;
static atomic_uint s_full, s_expired, s_outbox_bytes;

static void sample_outbox(esp_mqtt_client_handle_t client)
{
    int bytes = esp_mqtt_client_get_outbox_size(client);
    atomic_store(&s_outbox_bytes, bytes > 0 ? (unsigned)bytes : 0);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT conectado com sucesso!");
        mqtt_conectado = true;
        sample_outbox(event->client);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT desconectado.");
        mqtt_conectado = false;
        break;
    case MQTT_EVENT_DELETED:
        // Accepted into RAM earlier, but expired before it could leave the outbox.
        atomic_fetch_add(&s_expired, 1);
        sample_outbox(event->client);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "Erro na conexao MQTT: type=%d esp_tls=%d tls_stack=0x%x cert_flags=0x%x sock_errno=%d",
                 event->error_handle ? event->error_handle->error_type : 0,
                 event->error_handle ? event->error_handle->esp_tls_last_esp_err : 0,
                 event->error_handle ? event->error_handle->esp_tls_stack_err : 0,
                 event->error_handle ? event->error_handle->esp_tls_cert_verify_flags : 0,
                 event->error_handle ? event->error_handle->esp_transport_sock_errno : 0);
        break;
    default:
        break;
    }
}

void mqtt_app_init(void)
{
    if (mqtt_client != NULL) return;
    esp_mqtt_client_config_t mqtt_cfg = {0};

    mqtt_cfg.broker.address.uri = CONFIG_MOVE2_MQTT_URI;
    mqtt_cfg.outbox.limit = MQTT_APP_OUTBOX_LIMIT;

    if (strncmp(CONFIG_MOVE2_MQTT_URI, "mqtts://", 8) == 0 ||
        strncmp(CONFIG_MOVE2_MQTT_URI, "wss://", 6) == 0) {
        mqtt_cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    }

    if (strncmp(CONFIG_MOVE2_MQTT_URI, "wss://", 6) == 0) {
        mqtt_cfg.broker.verification.alpn_protos = s_wss_alpn_protos;
    }

    if (strlen(CONFIG_MOVE2_MQTT_USERNAME) > 0) {
        mqtt_cfg.credentials.username = CONFIG_MOVE2_MQTT_USERNAME;
        mqtt_cfg.credentials.authentication.password = CONFIG_MOVE2_MQTT_PASSWORD;
    }

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    if (!mqtt_client) { ESP_LOGE(TAG, "Sem memoria para MQTT; coleta local continua."); return; }
    esp_err_t err = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err == ESP_OK) err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha MQTT: %s; coleta local continua.", esp_err_to_name(err));
        esp_mqtt_client_destroy(mqtt_client); mqtt_client = NULL;
        mqtt_conectado = false;
        return;
    }

    ESP_LOGI(TAG, "Cliente MQTT iniciado: %s", CONFIG_MOVE2_MQTT_URI);
#ifdef CONFIG_MQTT_POLL_READ_TIMEOUT_MS
    ESP_LOGI(TAG, "Fila assincrona: limite=%d bytes poll=%d ms expira=%d ms",
             MQTT_APP_OUTBOX_LIMIT, CONFIG_MQTT_POLL_READ_TIMEOUT_MS,
             CONFIG_MQTT_OUTBOX_EXPIRED_TIMEOUT_MS);
    if (CONFIG_MQTT_POLL_READ_TIMEOUT_MS > 100)
        ESP_LOGW(TAG, "Poll MQTT alto para telemetria: configure MQTT_POLL_READ_TIMEOUT_MS=50.");
#else
    ESP_LOGW(TAG, "Poll MQTT padrao de 1000 ms pode acumular dados; habilite a configuracao customizada (50 ms).");
#endif
}

bool mqtt_app_is_connected(void)
{
    return mqtt_conectado;
}

mqtt_app_stats_t mqtt_app_get_stats(void)
{
    mqtt_app_stats_t stats = {
        .connected = atomic_load(&mqtt_conectado),
        .submitted = atomic_load(&s_submitted), .failed = atomic_load(&s_failed),
        .full = atomic_load(&s_full), .expired = atomic_load(&s_expired),
        .outbox_bytes = atomic_load(&s_outbox_bytes),
    };
    return stats;
}

bool mqtt_app_publish(const char *topic, const char *payload)
{
    if (mqtt_conectado && mqtt_client != NULL && topic && payload) {
        // Network IO runs in the MQTT task. Bounded RAM outbox, live data only.
        int result = esp_mqtt_client_enqueue(mqtt_client, topic, payload, 0, 0, 0, true);
        sample_outbox(mqtt_client);
        if (result >= 0) { atomic_fetch_add(&s_submitted, 1); return true; }
        if (result == -2) atomic_fetch_add(&s_full, 1);
    }
    atomic_fetch_add(&s_failed, 1);
    return false;
}
