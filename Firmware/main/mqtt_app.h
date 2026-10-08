#ifndef MQTT_APP_H
#define MQTT_APP_H

#include <stdbool.h>
#include <stdint.h>

#define MQTT_APP_OUTBOX_LIMIT (16 * 1024)

typedef struct {
    uint32_t submitted, failed, full, expired;
    // Cached on enqueue/events: reading stats never takes the MQTT API lock.
    uint32_t outbox_bytes;
    bool connected;
} mqtt_app_stats_t;
mqtt_app_stats_t mqtt_app_get_stats(void);

// Conecta ao broker MQTT e inicia a task em background
void mqtt_app_init(void);

// Retorna o status da conexão
bool mqtt_app_is_connected(void);

// Publica no broker
bool mqtt_app_publish(const char *topic, const char *payload);

#endif // MQTT_APP_H
