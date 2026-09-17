#ifndef MQTT_APP_H
#define MQTT_APP_H

#include <stdbool.h>

// Conecta ao broker MQTT e inicia a task em background
void mqtt_app_init(void);

// Retorna o status da conexão
bool mqtt_app_is_connected(void);

// Publica no broker
void mqtt_app_publish(const char *topic, const char *payload);

#endif // MQTT_APP_H