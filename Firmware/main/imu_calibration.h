#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int64_t sum[6], square[6];
    uint32_t count;
} imu_calibration_t;

static inline void imu_calibration_add(imu_calibration_t *c, const int16_t axes[6]) {
    for (unsigned i=0; i<6; ++i) {
        int64_t x=axes[i]; c->sum[i]+=x; c->square[i]+=x*x;
    }
    c->count++;
}

// A single still pose estimates gyro bias and a common accelerometer scale.
// It cannot identify individual accelerometer offsets or mounting angles.
static inline bool imu_calibration_finish(const imu_calibration_t *c,
                                          int32_t offsets[6], float *acc_scale) {
    if (c->count<2) return false;
    double mean[6], length2=0;
    for (unsigned i=0; i<6; ++i) {
        mean[i]=(double)c->sum[i]/c->count;
        double variance=(double)c->square[i]/c->count-mean[i]*mean[i];
        double limit=i<3 ? 80.0:65.536;
        if (variance>limit*limit || (i>=3 && fabs(mean[i])>10*65.536)) return false;
        if (i<3) length2+=mean[i]*mean[i];
    }
    double length=sqrt(length2);
    if (length<0.85*4096 || length>1.15*4096) return false;
    for (unsigned i=0; i<6; ++i) offsets[i]=(int32_t)mean[i];
    offsets[2]-=4096; // Preserve legacy MQTT sensor-axis calibration.
    *acc_scale=(float)(4096/length);
    return true;
}
#endif
