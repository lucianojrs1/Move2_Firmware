#ifndef IMU_FORMAT_H
#define IMU_FORMAT_H
#include "bmi_app.h"
#include <stddef.h>
#define IMU_PAYLOAD_MAX 1536
size_t imu_format_payload(char *out, size_t size, const bmi323_status_t *imu, uint32_t now_ms, uint64_t epoch_ms);
#endif
