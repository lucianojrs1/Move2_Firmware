#include "bmi_app.h"
#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#define PIN_NUM_MISO 39
#define PIN_NUM_MOSI 32
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   13

#define BMI323_ACC_CONF_NORMAL_100HZ_8G      0x4428
#define BMI323_GYR_CONF_NORMAL_100HZ_2000DPS 0x4448

static spi_device_handle_t spi;
static bmi323_status_t g_bmi = {0};
static SemaphoreHandle_t g_bmi_mutex = NULL;

static int16_t acc_off_x = 0, acc_off_y = 0, acc_off_z = 0;
static int16_t gyr_off_x = 0, gyr_off_y = 0, gyr_off_z = 0;

// --- Funções Privadas de Comunicação SPI ---
static esp_err_t bmi_spi_read_regs(uint8_t reg_addr, uint8_t *data, size_t data_len) {
    uint8_t tx_data[16] = {0}; 
    uint8_t rx_data[16] = {0}; 
    tx_data[0] = reg_addr | 0x80;

    spi_transaction_t t = {
        .length = (2 + data_len) * 8, 
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };

    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret == ESP_OK) {
        for(size_t i = 0; i < data_len; i++) {
            data[i] = rx_data[i + 2];
        }
    }
    return ret;
}

//Calibrar imu
static void bmi323_calibrate(void){
    long sum_ax = 0, sum_ay = 0, sum_az = 0;
    long sum_gx = 0, sum_gy = 0, sum_gz = 0;
    const int amostras = 200;

    printf("\n==========================================\n");
    printf("CALIBRANDO IMU: MANTENHA A MOTO PARADA...\n");
    printf("==========================================\n");

    vTaskDelay(pdMS_TO_TICKS(1000));

    for(int i = 0; i < amostras; i++){
        uint8_t acc_buf[6], gyr_buf[6];
        if (bmi_spi_read_regs(0x03, acc_buf, 6) == ESP_OK && 
            bmi_spi_read_regs(0x06, gyr_buf, 6) == ESP_OK) {
            
            sum_ax += (int16_t)((acc_buf[1] << 8) | acc_buf[0]);
            sum_ay += (int16_t)((acc_buf[3] << 8) | acc_buf[2]);
            sum_az += (int16_t)((acc_buf[5] << 8) | acc_buf[4]);
            
            sum_gx += (int16_t)((gyr_buf[1] << 8) | gyr_buf[0]);
            sum_gy += (int16_t)((gyr_buf[3] << 8) | gyr_buf[2]);
            sum_gz += (int16_t)((gyr_buf[5] << 8) | gyr_buf[4]);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Espera 10ms (coleta a 100Hz)
    }
// A média ideal para ACC X e Y é zero
    acc_off_x = sum_ax / amostras;
    acc_off_y = sum_ay / amostras;
    
    // O eixo Z sofre 1G da gravidade, então a média ideal é 4096.
    // O "erro" é o quanto ele passa ou falta de 4096.
    acc_off_z = (sum_az / amostras) - 4096; 

    // A média ideal para todo o giroscópio é zero
    gyr_off_x = sum_gx / amostras;
    gyr_off_y = sum_gy / amostras;
    gyr_off_z = sum_gz / amostras;

    printf("Calibracao Concluida!\n");
    printf("Compensacao ACC -> X:%d | Y:%d | Z:%d\n", acc_off_x, acc_off_y, acc_off_z);
    printf("Compensacao GYR -> X:%d | Y:%d | Z:%d\n", gyr_off_x, gyr_off_y, gyr_off_z);
    printf("==========================================\n\n");
}




static esp_err_t bmi_spi_write_reg(uint8_t reg_addr, uint16_t data) {
    uint8_t tx_data[3];
    tx_data[0] = reg_addr & 0x7F;       
    tx_data[1] = data & 0xFF;           
    tx_data[2] = (data >> 8) & 0xFF;    

    spi_transaction_t t = {
        .length = 3 * 8, 
        .tx_buffer = tx_data
    };
    return spi_device_transmit(spi, &t);
}

// --- Task de Background do FreeRTOS ---
static void bmi323_task(void *pvParameters) {
    // Liga Acelerômetro e Giroscópio
    bmi_spi_write_reg(0x20, BMI323_ACC_CONF_NORMAL_100HZ_8G); 
    bmi_spi_write_reg(0x21, BMI323_GYR_CONF_NORMAL_100HZ_2000DPS);
    vTaskDelay(pdMS_TO_TICKS(100));

    bmi323_calibrate();

    while (1) {
        bmi323_status_t local_copy = {0};
        uint8_t id_buf[2], stat_buf[2], acc_buf[6], gyr_buf[6];
        bool all_ok = true;

        // Leitura do Chip ID e Status
        if (bmi_spi_read_regs(0x00, id_buf, 2) == ESP_OK && 
            bmi_spi_read_regs(0x02, stat_buf, 2) == ESP_OK) {
            local_copy.chip_id = (id_buf[1] << 8) | id_buf[0];
            local_copy.status_reg = (stat_buf[1] << 8) | stat_buf[0];
        } else {
            all_ok = false;
        }

        // Leitura do Acelerômetro (0x03) e Giroscópio (0x06)
        if (all_ok && 
            bmi_spi_read_regs(0x03, acc_buf, 6) == ESP_OK && 
            bmi_spi_read_regs(0x06, gyr_buf, 6) == ESP_OK) {
            
            local_copy.acc_x = (int16_t)((acc_buf[1] << 8) | acc_buf[0]) - acc_off_x;
            local_copy.acc_y = (int16_t)((acc_buf[3] << 8) | acc_buf[2]) - acc_off_y;
            local_copy.acc_z = (int16_t)((acc_buf[5] << 8) | acc_buf[4]) - acc_off_z;
            
            local_copy.gyr_x = (int16_t)((gyr_buf[1] << 8) | gyr_buf[0]) - gyr_off_x;
            local_copy.gyr_y = (int16_t)((gyr_buf[3] << 8) | gyr_buf[2]) - gyr_off_y;
            local_copy.gyr_z = (int16_t)((gyr_buf[5] << 8) | gyr_buf[4]) - gyr_off_z;
            
            local_copy.last_update_ms = (uint32_t)(esp_timer_get_time() / 1000);
            local_copy.spi_ok = true;
        } else {
            local_copy.spi_ok = false;
        }

        // Atualiza a variável global protegida pelo Mutex
        xSemaphoreTake(g_bmi_mutex, portMAX_DELAY);
        g_bmi = local_copy;
        xSemaphoreGive(g_bmi_mutex);

        // Define a frequência de amostragem da task (ex: 100Hz = 10ms)
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// --- Funções Públicas ---
void bmi323_app_init(void) {
    g_bmi_mutex = xSemaphoreCreateMutex();

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, 
        .mode = 0,                 
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t dummy_rx[2];
    bmi_spi_read_regs(0x00, dummy_rx, 2); // Desperta SPI

    // Cria a Task que ficará rodando no background eternamente
    xTaskCreate(bmi323_task, "bmi_task", 4096, NULL, 5, NULL);
}

bmi323_status_t bmi323_app_get_status(void) {
    bmi323_status_t copy = {0};
    if (g_bmi_mutex != NULL) {
        xSemaphoreTake(g_bmi_mutex, portMAX_DELAY);
        copy = g_bmi;
        xSemaphoreGive(g_bmi_mutex);
    }
    return copy;
}