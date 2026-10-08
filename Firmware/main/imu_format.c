#include "imu_format.h"
#include <inttypes.h>
size_t imu_format_payload(char *out, size_t size, const bmi323_status_t *imu, uint32_t now_ms, uint64_t epoch_ms) {
    if (!out || !size || !imu) return 0;
    bool fresh=imu->i2c_ok && (uint32_t)(now_ms-imu->last_update_ms)<100;
    if (!fresh) return 0;
    json_writer_t w={.out=out,.cap=size};
    json_add(&w,"{\"sensorId\":\"imu\",\"sensorType\":\"Imu\",\"value\":{"
        "\"accelerometer\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
        "\"gyroscope\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},",
        imu->acc_x*(9.80665f/4096),imu->acc_y*(9.80665f/4096),imu->acc_z*(9.80665f/4096),
        imu->gyr_x/65.536f,imu->gyr_y/65.536f,imu->gyr_z/65.536f);
    motion_json_fields(&w,&imu->motion,fresh);
    json_add(&w,"},\"timestamp\":%" PRIu64 "}",epoch_ms);
    return w.failed ? 0:w.used;
}
