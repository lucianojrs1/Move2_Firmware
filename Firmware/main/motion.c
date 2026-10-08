#include "motion.h"
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#define G 9.80665f
#define RAD 0.017453292519943295f
static float norm(const float a[3]) { return sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]); }
bool motion_init(motion_state_t *s, const motion_config_t *c) {
    if (!s || !c) return false;
    int a=abs(c->axes[0]), b=abs(c->axes[1]), d=abs(c->axes[2]);
    if (a<1 || a>3 || b<1 || b>3 || d<1 || d>3 || a==b || b==d || a==d) return false;
    int permutation = ((a-b)*(b-d)*(a-d) < 0) ? 1 : -1;
    int signs = (c->axes[0]>0 ? 1:-1)*(c->axes[1]>0 ? 1:-1)*(c->axes[2]>0 ? 1:-1);
    if (permutation*signs != 1 || !isfinite(c->braking_mps2) || c->braking_mps2<=0 ||
        !isfinite(c->impact_g) || c->impact_g<=1 || !isfinite(c->fall_deg) || c->fall_deg<=0 || c->fall_deg>=90) return false;
    *s = (motion_state_t){.config=*c,.fall_cos=cosf(c->fall_deg*RAD)}; return true;
}
bool motion_update(motion_state_t *s, const float acc[3], const float gyro[3],
                   int16_t temperature, uint64_t now, bool speed_valid, float speed) {
    float a[3], omega[3];
    for (int i=0;i<3;++i) {
        if (!isfinite(acc[i]) || !isfinite(gyro[i])) { s->initialized=false; s->sample.valid=false; return false; }
        int axis=s->config.axes[i]; float sign=axis>0 ? 1.0f:-1.0f;
        a[i]=acc[abs(axis)-1]*sign; omega[i]=gyro[abs(axis)-1]*sign*RAD;
    }
    float magnitude=norm(a);
    float rotation=norm(omega);
    if (!s->initialized || now<=s->last_us || now-s->last_us>100000) {
        s->sample.valid=false; s->sample.attitude_valid=false;
        // Never initialize gravity from free fall or an impact.
        if (fabsf(magnitude-G)>0.15f*G) { s->initialized=false; return false; }
        for (int i=0;i<3;++i) {
            s->gravity[i]=a[i]/magnitude; s->filtered_linear[i]=0; s->slow_acc[i]=a[i];
        }
        s->window_time=s->vibration_energy=0; s->samples=s->events=0; s->peak=0;
        s->braking_s=s->tilted_s=0; s->braking_active=s->fall_active=false;
        s->impact_seen=s->rotation_seen=s->jerk_ready=false;
        s->last_us=s->window_start_us=now; s->initialized=true; return false;
    }
    float dt=(float)(now-s->last_us)*1e-6f; s->last_us=now;
    float *g=s->gravity;
    float derivative[3]={g[1]*omega[2]-g[2]*omega[1], g[2]*omega[0]-g[0]*omega[2], g[0]*omega[1]-g[1]*omega[0]};
    for (int i=0;i<3;++i) g[i]+=derivative[i]*dt;
    float gn=norm(g); for (int i=0;i<3;++i) g[i]/=gn;
    // Slow accelerometer correction, gated during forces/rotation. No yaw claim.
    float residual[3]={a[0]-G*g[0],a[1]-G*g[1],a[2]-G*g[2]};
    bool stationary=speed_valid && speed<1 && rotation<0.1f;
    if (fabsf(magnitude-G)<0.10f*G && rotation<1.5f && (stationary || norm(residual)<1)) {
        float weight=dt/((stationary ? 1.0f:5.0f)+dt);
        for (int i=0;i<3;++i) g[i]=(1-weight)*g[i]+weight*a[i]/magnitude;
        gn=norm(g); for (int i=0;i<3;++i) g[i]/=gn;
    }
    motion_sample_t *o=&s->sample;
    o->temperature_valid=temperature!=INT16_MIN;
    o->temperature_c=23.0f+temperature/512.0f;
    o->magnitude=magnitude;
    float jerk2=0, vibration2=0;
    for (int i=0;i<3;++i) {
        float previous=s->filtered_linear[i];
        s->filtered_linear[i]+=dt/(0.05f+dt)*(a[i]-G*g[i]-s->filtered_linear[i]);
        o->linear[i]=s->filtered_linear[i];
        float derivative_linear=(o->linear[i]-previous)/dt;
        jerk2+=derivative_linear*derivative_linear;
        // High-pass acceleration, cutoff approximately 2 Hz; RMS over each window.
        s->slow_acc[i]+=dt/(0.08f+dt)*(a[i]-s->slow_acc[i]);
        float high=a[i]-s->slow_acc[i]; vibration2+=high*high;
    }
    o->jerk=s->jerk_ready ? sqrtf(jerk2):0; s->jerk_ready=true;
    o->longitudinal=o->linear[0];
    o->attitude_valid=true;
    if (s->impact_cooldown_s>0) s->impact_cooldown_s-=dt;
    if (magnitude>=s->config.impact_g*G && s->impact_cooldown_s<=0) {
        s->events|=MOTION_IMPACT; o->impact_count++; s->impact_us=now;
        s->impact_seen=true; s->impact_cooldown_s=1;
    }
    if (rotation>100*RAD) { s->rotation_us=now; s->rotation_seen=true; }
    bool braking=speed_valid && speed>=5 && o->longitudinal < -s->config.braking_mps2;
    if (braking) s->braking_s+=dt; else { s->braking_s=0; s->braking_active=false; }
    if (s->braking_s>=0.25f && !s->braking_active) {
        s->braking_active=true; s->events|=MOTION_BRAKING; o->braking_count++;
    }
    // Possible fall: sustained tilt AND recent impact/fast rotation; slow/unknown GPS.
    bool tilted=g[2]<s->fall_cos;
    if (tilted) s->tilted_s+=dt; else { s->tilted_s=0; s->fall_active=false; }
    bool precursor=(s->impact_seen && now-s->impact_us<=5000000) ||
                   (s->rotation_seen && now-s->rotation_us<=5000000);
    if (s->tilted_s>=1 && precursor && (!speed_valid || speed<3) && !s->fall_active) {
        s->fall_active=true; s->events|=MOTION_POSSIBLE_FALL; o->fall_count++;
    }
    s->vibration_energy+=vibration2*dt; s->window_time+=dt; s->samples++;
    if (magnitude>s->peak) s->peak=magnitude;
    if (now-s->window_start_us<500000) return false;
    o->roll_deg=atan2f(g[1],g[2])/RAD;
    o->pitch_deg=atan2f(-g[0],sqrtf(g[1]*g[1]+g[2]*g[2]))/RAD;
    o->vibration_rms=(float)sqrt(s->vibration_energy/s->window_time);
    o->peak_acceleration=s->peak; o->events=s->events; o->samples=s->samples;
    o->window_ms=(uint32_t)((now-s->window_start_us)/1000); o->valid=true;
    s->window_start_us=now; s->window_time=s->vibration_energy=0;
    s->samples=s->events=0; s->peak=0;
    return true;
}
void motion_json_fields(json_writer_t *w, const motion_sample_t *s, bool fresh) {
    bool valid=fresh && s->valid;
    json_add(w,"\"motionValid\":%s,\"imuTemperatureC\":",valid?"true":"false");
    json_number(w,s->temperature_c,valid && s->temperature_valid);
    json_add(w,",\"accelerationMagnitude\":"); json_number(w,s->magnitude,valid);
    json_add(w,",\"linearAcceleration\":{\"x\":"); json_number(w,s->linear[0],valid);
    json_add(w,",\"y\":"); json_number(w,s->linear[1],valid);
    json_add(w,",\"z\":"); json_number(w,s->linear[2],valid);
    json_add(w,"},\"longitudinalAcceleration\":"); json_number(w,s->longitudinal,valid);
    json_add(w,",\"jerk\":"); json_number(w,s->jerk,valid);
    json_add(w,",\"vibrationRms\":"); json_number(w,s->vibration_rms,valid);
    json_add(w,",\"peakAcceleration\":"); json_number(w,s->peak_acceleration,valid);
    json_add(w,",\"rollDeg\":"); json_number(w,s->roll_deg,valid && s->attitude_valid);
    json_add(w,",\"pitchDeg\":"); json_number(w,s->pitch_deg,valid && s->attitude_valid);
    json_add(w,",\"windowMs\":%" PRIu32 ",\"windowSamples\":%" PRIu32
        ",\"eventsExperimental\":true,\"events\":{\"braking\":%s,\"impact\":%s,\"possibleFall\":%s,"
        "\"brakingCount\":%" PRIu32 ",\"impactCount\":%" PRIu32 ",\"possibleFallCount\":%" PRIu32 "}",
        s->window_ms,s->samples,valid && (s->events&MOTION_BRAKING)?"true":"false",
        valid && (s->events&MOTION_IMPACT)?"true":"false",valid && (s->events&MOTION_POSSIBLE_FALL)?"true":"false",
        s->braking_count,s->impact_count,s->fall_count);
}
