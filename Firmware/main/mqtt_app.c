#include "mqtt_app.h"
#include <string.h>
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "mqtt_client.h"

// Credenciais EMQX
#define EMQX_MQTT_URI      "mqtts://eaa7d5aa.ala.eu-central-1.emqxsl.com:8883"
#define EMQX_MQTT_USERNAME "modcs"
#define EMQX_MQTT_PASSWORD "12345678"

static const char *TAG = "MQTT_APP";
static const char *s_wss_alpn_protos[] = {"http/1.1", NULL};
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_conectado = false;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT conectado com sucesso no EMQX!");
        mqtt_conectado = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT desconectado.");
        mqtt_conectado = false;
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
    esp_mqtt_client_config_t mqtt_cfg = {0};

    // Aplica a URI diretamente
    mqtt_cfg.broker.address.uri = EMQX_MQTT_URI;

    // Configura os certificados TLS automaticamente por causa do prefixo "mqtts://"
    if (strncmp(EMQX_MQTT_URI, "mqtts://", 8) == 0 ||
        strncmp(EMQX_MQTT_URI, "wss://", 6) == 0) {
        mqtt_cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    }

    if (strncmp(EMQX_MQTT_URI, "wss://", 6) == 0) {
        mqtt_cfg.broker.verification.alpn_protos = s_wss_alpn_protos;
    }

    // Aplica Usuário e Senha
    mqtt_cfg.credentials.username = EMQX_MQTT_USERNAME;
    mqtt_cfg.credentials.authentication.password = EMQX_MQTT_PASSWORD;

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));

    ESP_LOGI(TAG, "Cliente MQTT iniciado, tentando conectar em: %s", EMQX_MQTT_URI);
}

bool mqtt_app_is_connected(void)
{
    return mqtt_conectado;
}

void mqtt_app_publish(const char *topic, const char *payload)
{
    if (mqtt_conectado && mqtt_client != NULL) {
        // QoS 0 utilizado por padrão. Se precisar de garantia de entrega, mude para 1.
        esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 0, 0);
    } else {
        ESP_LOGW(TAG, "Tentativa de publicacao falhou: MQTT desconectado.");
    }
}