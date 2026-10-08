#ifndef CAN_APP_H
#define CAN_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint64_t mono_us;
    bool extended, rtr;
} can_frame_t;

typedef struct { uint32_t received, cloud_dropped, bus_errors; } can_app_stats_t;
can_app_stats_t can_app_get_stats(void);

void can_app_init(void);
bool can_app_receive(can_frame_t *frame, TickType_t ticks_to_wait);

#endif
