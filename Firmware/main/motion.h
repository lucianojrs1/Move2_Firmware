#ifndef MOTION_H
#define MOTION_H
#include <stdbool.h>
#include <stdint.h>
#include "json_writer.h"
#define MOTION_BRAKING 1U
#define MOTION_IMPACT 2U
#define MOTION_POSSIBLE_FALL 4U
typedef struct {
    float temperature_c, magnitude, linear[3], longitudinal, jerk;
    float vibration_rms, peak_acceleration, roll_deg, pitch_deg;
    uint32_t samples, window_ms, events, braking_count, impact_count, fall_count;
    bool valid, temperature_valid, attitude_valid;
} motion_sample_t;
typedef struct {
    // Signed sensor axes for body X forward, Y left, Z up. Right-handed permutation.
    int axes[3];
    float braking_mps2, impact_g, fall_deg;
} motion_config_t;
typedef struct {
    motion_config_t config;
    motion_sample_t sample;
    float gravity[3], filtered_linear[3], slow_acc[3];
    double vibration_energy, window_time;
    float peak, fall_cos;
    uint64_t last_us, window_start_us, impact_us, rotation_us;
    float braking_s, tilted_s, impact_cooldown_s;
    uint32_t samples, events;
    bool initialized, braking_active, fall_active, jerk_ready, impact_seen, rotation_seen;
} motion_state_t;
bool motion_init(motion_state_t *state, const motion_config_t *config);
// Acceleration and corrected gyro in SENSOR axes. All calculations use body axes.
// Returns true when a new 500 ms summary is complete. No heap allocation or IO.
bool motion_update(motion_state_t *state, const float acc[3], const float gyro[3],
                   int16_t temperature_raw, uint64_t now_us, bool speed_valid, float speed_kmh);
void motion_json_fields(json_writer_t *w, const motion_sample_t *sample, bool fresh);
#endif
