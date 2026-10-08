#ifndef GPS_APP_H
#define GPS_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "gps_metrics.h"

// Estrutura com todos os dados extraídos do GPS
typedef struct {
    bool antenna_ok;
    char module_info[48];
    char fw_info[64];
    
    int fix_quality;
    int sats_used;
    bool fix;
    char rmc_status;
    
    bool time_valid;
    int hour;
    int minute;
    int second;
    int centisecond;
    
    bool date_valid;
    int day;
    int month;
    int year;
    
    bool location_valid;
    double lat;
    double lon;
    
    uint32_t last_fix_ms;
    uint32_t bytes;
    uint32_t last_sentence_ms;
    bool uart_seen;
    uint32_t lines;
    uint32_t line_overflows;
    uint32_t rmc_count, gga_count;
    uint32_t last_gga_ms;
    gps_metrics_t metrics;
} gps_status_t;

// Inicializa a UART do GPS e dispara a Task de leitura internamente
void gps_app_init(void);

// Retorna uma cópia segura da estrutura do GPS atualizada
gps_status_t gps_app_get_status(void);

#endif // GPS_APP_H
