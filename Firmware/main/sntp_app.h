#ifndef SNTP_APP_H
#define SNTP_APP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa o serviço SNTP e configura os servidores NTP para ajuste do RTC interno.
 */
void sntp_app_init(void);

/**
 * @brief Verifica se o RTC interno já foi sincronizado com sucesso via NTP.
 * 
 * @return true se o relógio já foi ajustado, false se ainda está pendente.
 */
bool sntp_app_is_synced(void);

/**
 * @brief Obtém o timestamp UTC atual em milissegundos (Unix Epoch de 64 bits).
 * 
 * @return uint64_t Milissegundos desde 01/01/1970 (equivalente a Date.now() do JS).
 */
uint64_t get_rtc_time_ms(void);

#ifdef __cplusplus
}
#endif

#endif // SNTP_APP_H