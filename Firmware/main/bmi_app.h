#ifndef BMI323_APP_H
#define BMI323_APP_H

#include <stdint.h>
#include <stdbool.h>

// Estrutura com todos os dados extraídos do IMU
typedef struct {
    bool spi_ok;            // Indica se a comunicação SPI está ativa e sem erros
    uint16_t chip_id;       // Deve ser 0x0043 ou 0x1043
    uint16_t status_reg;    // Registrador de status bruto (Data Ready, etc)
    
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;
    
    uint32_t last_update_ms; // Timestamp da última leitura bem-sucedida
} bmi323_status_t;

// Inicializa o SPI do IMU e dispara a Task de leitura internamente
void bmi323_app_init(void);

// Retorna uma cópia segura da estrutura do IMU atualizada
bmi323_status_t bmi323_app_get_status(void);

#endif // BMI323_APP_H