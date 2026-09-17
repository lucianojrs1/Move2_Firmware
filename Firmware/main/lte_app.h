#ifndef LTE_APP_H
#define LTE_APP_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lte_app_init(void);
bool lte_app_wait_connected(uint32_t timeout_ms);
bool lte_app_is_connected(void);
esp_netif_t *lte_app_get_netif(void);
void lte_app_get_signal_quality(int *rssi, int *ber);

#ifdef __cplusplus
}
#endif

#endif
