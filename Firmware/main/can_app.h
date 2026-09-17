#ifndef CAN_APP_H
#define CAN_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

void can_app_init(void);
bool can_app_receive(can_frame_t *frame, TickType_t ticks_to_wait);

#endif