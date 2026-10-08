#include "gps_navigation.h"
#include "gps_format.h"
#include "imu_format.h"
#include "sd_record.h"
#include "imu_calibration.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static bool feed(gps_navigation_t *nav, gps_status_t *g, const char *body, uint32_t now) {
    unsigned sum=0; for (const char *p=body;*p;++p) sum^=(unsigned char)*p;
    char line[192]; snprintf(line,sizeof(line),"$%s*%02X",body,sum);
    return gps_navigation_process(nav,g,line,now);
}
static void gga(gps_navigation_t *n, gps_status_t *g, uint32_t t) {
    feed(n,g,"GNGGA,120000,0000.0000,N,00000.0000,E,1,08,0.9,42.5,M,0,M,,",t);
}
static void navigation_test(void) {
    gps_navigation_t n; gps_navigation_init(&n,0);
    gps_status_t g={.uart_seen=true};
    gga(&n,&g,900);
    assert(feed(&n,&g,"GNRMC,120000,A,0000.0000,N,00000.0000,E,19.438,90,220926,,,A",1000));
    assert(g.fix && g.metrics.first_fix_ms==1000 && g.metrics.tracking);
    assert(fabs(g.metrics.speed_kmh-36)<0.01 && g.metrics.course_deg==90);
    assert(g.metrics.altitude_valid && g.metrics.altitude_m==42.5f);
    gga(&n,&g,1900);
    const char *second="GNRMC,120001,A,0000.0000,N,00000.0054,E,19.438,90,220926,,,A";
    assert(feed(&n,&g,second,2000));
    assert(g.metrics.trip_distance_m==0 && !g.metrics.moving && g.metrics.max_speed_kmh==0);
    assert(g.metrics.moving_ms==0 && g.metrics.stopped_ms==1000);
    assert(!feed(&n,&g,second,2500)); assert(g.last_fix_ms==2000 && g.metrics.moving_ms==0);
    double saved=g.metrics.trip_distance_m;
    assert(feed(&n,&g,"GNRMC,120002,V,,,,,,,220926,,,N",3000));
    assert(!g.fix && !g.metrics.tracking && !g.metrics.speed_valid);
    gga(&n,&g,6900);
    feed(&n,&g,"GNRMC,120006,A,0000.0000,N,00001.0000,E,19.438,90,220926,,,A",7000);
    assert(g.metrics.trip_distance_m==saved && g.metrics.moving_ms==0); // No bridge across a GPS outage.
    gga(&n,&g,7900);
    feed(&n,&g,"GNRMC,120007,A,0000.0000,N,00001.0001,E,0,90,220926,,,A",8000);
    assert(!g.metrics.moving && !g.metrics.course_valid && g.metrics.stopped_ms==2000);
    assert(g.metrics.trip_distance_m==saved); // Stationary jitter is not distance.
    gga(&n,&g,8900);
    feed(&n,&g,"GNRMC,120008,A,0100.0000,N,00001.0001,E,19.438,90,220926,,,A",9000);
    assert(!g.metrics.tracking && g.metrics.trip_distance_m==saved); // Teleport rejected.
    uint32_t last=g.last_fix_ms;
    assert(!gps_navigation_process(&n,&g,"$GNRMC,120009,A*00",10000));
    assert(g.last_fix_ms==last); // Checksum failure cannot refresh the fix.
    feed(&n,&g,"GNRMC,120009,A,0060.0000,N,00000.0000,E,1,0,220926,,,A",10000);
    assert(!g.fix); // Invalid degrees/minutes representation.
    gga(&n,&g,10900);
    feed(&n,&g,"GNRMC,120010,A,0000.0000,Q,00000.0000,E,1,0,220926,,,A",11000);
    assert(!g.fix);
    gga(&n,&g,11900);
    feed(&n,&g,"GNRMC,120011,A,0000.0000,N,00000.0000,E,nan,inf,220926,,,A",12000);
    assert(g.fix && !g.metrics.speed_valid && !g.metrics.course_valid && !g.metrics.tracking);
    feed(&n,&g,"GNGGA,120012,0000.0000,N,00000.0000,E,6,08,0.9,42.5,M,0,M,,",12900);
    feed(&n,&g,"GNRMC,120012,A,0000.0000,N,00000.0000,E,1,0,220926,,,A",13000);
    assert(!g.fix && !g.metrics.altitude_valid); // Estimated GGA is not a satellite fix.
    gga(&n,&g,13900);
    feed(&n,&g,"GNRMC,120013,A,0000.0000,N,00000.0000,E,0,0,220926,,,A",14000);
    char output[GPS_PAYLOAD_MAX];
    assert(gps_format_payload(output,sizeof(output),&g,14100,1790045000000ULL)); puts(output);
    assert(gps_format_payload(output,sizeof(output),&g,18000,1790045004000ULL));
    assert(strstr(output,"\"speedKmh\":null") && strstr(output,"\"altitudeM\":null") && strstr(output,"\"moving\":null"));
    sd_record_t r={.type=SD_RECORD_TRIP}; r.value.trip.metrics=g.metrics;
    r.value.trip.valid=r.value.trip.gga_fresh=true;
    char line[SD_RECORD_LINE_MAX]; assert(sd_record_format(line,sizeof(line),&r,1,0)); fputs(line,stdout);

    // No elapsed-time wrap, midnight, or duplicate integration surprises.
    gps_navigation_init(&n,UINT32_MAX-500); g=(gps_status_t){.uart_seen=true};
    gga(&n,&g,UINT32_MAX-200);
    feed(&n,&g,"GPRMC,235959,A,0000.0000,N,00000.0000,E,0,0,220926,,,A",UINT32_MAX-100);
    gga(&n,&g,850);
    feed(&n,&g,"GPRMC,000000,A,0000.0000,N,00000.0000,E,0,0,230926,,,A",899);
    assert(g.metrics.stopped_ms==1000 && g.metrics.first_fix_ms==400);
    last=g.last_fix_ms;
    feed(&n,&g,"GPRMC,235959,A,0000.0000,N,00000.0000,E,0,0,220926,,,A",1900);
    assert(g.last_fix_ms==last); // Old date after midnight.
    feed(&n,&g,"GPRMC,000001,A,0000.0000,N,00000.0000,E,0,0,310226,,,A",2900);
    assert(!g.fix && !g.date_valid);
    gga(&n,&g,3900);
    feed(&n,&g,"GPRMC,000003,A,0000.0000,N,00000.0000,E,10,0,230926,,,A",4000);
    saved=g.metrics.trip_distance_m;
    feed(&n,&g,"GPGGA,000004,0000.0000,N,00000.0000,E,1,08,9.0,,M,0,M,,",4900);
    feed(&n,&g,"GPRMC,000004,A,0000.0000,N,00000.0010,E,10,0,230926,,,A",5000);
    assert(!g.metrics.tracking && !g.metrics.altitude_valid && g.metrics.trip_distance_m==saved);
    assert(gps_format_payload(output,sizeof(output),&g,5100,0));
    assert(strstr(output,"\"altitudeM\":null"));
}
static void position(gps_navigation_t *n, gps_status_t *g, unsigned seconds,
                     uint32_t now, double east_m, double speed_kmh) {
    char body[160];
    gga(n,g,now);
    snprintf(body,sizeof(body),"GNRMC,12%02u%02u,A,0000.0000,N,%010.4f,%s,%.4f,90,220926,,,A",
             seconds/60,seconds%60,fabs(east_m)/1853.2488,east_m<0 ? "W":"E",speed_kmh/1.852);
    assert(feed(n,g,body,now));
}
static void stationary_navigation_test(void) {
    gps_navigation_t n; gps_navigation_init(&n,0);
    gps_status_t g={.uart_seen=true};
    // Bench-like drifting positions and sub-5 km/h speed, plus a brief higher spike.
    for (unsigned i=0;i<180;++i) {
        double speed=(i>=30 && i<33) ? 6.5 : (i%5==0 ? 3.6:0.5);
        position(&n,&g,i,(i+1)*1000,6*sin(i*0.1),speed);
        assert(!g.metrics.moving && g.metrics.trip_distance_m==0 && g.metrics.max_speed_kmh==0);
    }
    assert(g.metrics.moving_ms==0 && g.metrics.stopped_ms==179000);
    // Speed alone, even sustained, cannot confirm movement without displacement.
    for (unsigned i=180;i<190;++i) {
        position(&n,&g,i,(i+1)*1000,0,6.5);
        assert(!g.metrics.moving && g.metrics.trip_distance_m==0);
    }
    // Continuous 36 km/h travel: confirm after four seconds, then integrate 10 m.
    for (unsigned i=200;i<=205;++i) {
        position(&n,&g,i,(i+1)*1000,(i-200)*10,36);
        if (i<204) assert(!g.metrics.moving && g.metrics.max_speed_kmh==0);
        if (i==204) assert(g.metrics.moving && g.metrics.trip_distance_m==0);
    }
    assert(g.metrics.trip_distance_m>9.8 && g.metrics.trip_distance_m<10.2);
    assert(g.metrics.moving_ms==1000 && fabs(g.metrics.average_speed_kmh-36)<1);
    assert(fabs(g.metrics.max_speed_kmh-36)<0.01);
    double saved=g.metrics.trip_distance_m;
    position(&n,&g,206,207000,51,2.2);
    assert(!g.metrics.moving && g.metrics.trip_distance_m==saved);
    // A quality interruption cancels the candidate, without linking across the gap.
    position(&n,&g,207,208000,60,36);
    position(&n,&g,208,209000,70,36);
    feed(&n,&g,"GNGGA,120329,0000.0000,N,00000.0000,E,1,08,9.0,42.5,M,0,M,,",210000);
    for (unsigned i=210;i<=213;++i) {
        position(&n,&g,i,(i+1)*1000,90+(i-210)*10,36);
        assert(!g.metrics.moving && g.metrics.trip_distance_m==saved);
    }
    // Duration logic must survive the millisecond counter wrapping around.
    gps_navigation_init(&n,UINT32_MAX-2000); g=(gps_status_t){.uart_seen=true};
    for (unsigned i=0;i<=5;++i) position(&n,&g,i,UINT32_MAX-1500+i*1000,i*10,36);
    assert(g.metrics.moving && g.metrics.moving_ms==1000 && g.metrics.trip_distance_m>9.8);
}
static motion_config_t config(void) {
    return (motion_config_t){.axes={1,2,3},.braking_mps2=3,.impact_g=2.5f,.fall_deg=60};
}
static void motion_test(void) {
    motion_state_t s; motion_config_t c=config(); assert(motion_init(&s,&c));
    float a[3]={0,0,9.80665f}, gyro[3]={0}; uint64_t t=0;
    for (unsigned i=0;i<=100;++i) { motion_update(&s,a,gyro,512,t,true,0); t+=10000; }
    assert(s.sample.valid && s.sample.temperature_c==24 && s.sample.samples==50);
    assert(fabs(s.sample.roll_deg)<0.01 && fabs(s.sample.pitch_deg)<0.01);
    assert(s.sample.vibration_rms<0.001 && s.sample.jerk<0.001);
    assert(fabs(s.sample.peak_acceleration-9.80665)<0.001);
    assert(!s.sample.impact_count && !s.sample.braking_count && !s.sample.fall_count);
    // Sustained longitudinal deceleration with GPS motion: one debounced event.
    a[0]=-4;
    for (unsigned i=0;i<100;++i) { motion_update(&s,a,gyro,512,t,true,40); t+=10000; }
    assert(s.sample.braking_count==1 && s.sample.longitudinal<-3);
    a[0]=0;
    for (unsigned i=0;i<50;++i) { motion_update(&s,a,gyro,512,t,true,40); t+=10000; }
    // Impact shorter than the MQTT period is latched in the high-rate window.
    a[2]=30; motion_update(&s,a,gyro,512,t,true,0); t+=10000; a[2]=9.80665f;
    bool saw_impact=false;
    for (unsigned i=0;i<50;++i) {
        if (motion_update(&s,a,gyro,512,t,true,0) && (s.sample.events&MOTION_IMPACT)) saw_impact=true;
        t+=10000;
    }
    assert(saw_impact && s.sample.impact_count==1 && s.sample.fall_count==0);
    // Rotate onto the side, then stay there: candidate fall, not repeated every window.
    for (unsigned i=1;i<=50;++i) {
        float angle=(float)i*1.8f*0.01745329252f;
        a[1]=9.80665f*sinf(angle); a[2]=9.80665f*cosf(angle); gyro[0]=180;
        motion_update(&s,a,gyro,512,t,true,0); t+=10000;
    }
    gyro[0]=0;
    for (unsigned i=0;i<200;++i) { motion_update(&s,a,gyro,INT16_MIN,t,true,0); t+=10000; }
    assert(s.sample.fall_count==1 && fabs(s.sample.roll_deg-90)<3);
    assert(!s.sample.temperature_valid);
    bmi323_status_t imu={.i2c_ok=true,.last_update_ms=500,.acc_z=4096,.motion=s.sample};
    char payload[IMU_PAYLOAD_MAX];
    assert(imu_format_payload(payload,sizeof(payload),&imu,550,1790045000000ULL)); puts(payload);
    assert(!imu_format_payload(payload,sizeof(payload),&imu,600,0));
    char tiny[4]={'a','b','c','d'};
    assert(!imu_format_payload(tiny,3,&imu,550,0) && tiny[3]=='d');
    sd_record_t r={.type=SD_RECORD_MOTION}; r.value.motion=s.sample;
    char line[SD_RECORD_LINE_MAX]; assert(sd_record_format(line,sizeof(line),&r,1,0)); fputs(line,stdout);
    t+=200000; motion_update(&s,a,gyro,512,t,true,0); assert(!s.sample.valid); // Gap resets derivatives/window.
    a[0]=NAN; motion_update(&s,a,gyro,512,t+10000,true,0); assert(!s.sample.valid);
    c.axes[2]=-3; assert(!motion_init(&s,&c)); // Reflections invert gyro interpretation.
    c=config(); c.axes[0]=0; assert(!motion_init(&s,&c));
    c=config(); c.axes[1]=1; assert(!motion_init(&s,&c));
    c=config(); c.axes[0]=-2; c.axes[1]=1; assert(motion_init(&s,&c));
    a[0]=4.903325f; a[1]=0; a[2]=8.492808f;
    for (unsigned i=0;i<=100;++i) motion_update(&s,a,gyro,0,(uint64_t)i*10000,false,0);
    assert(fabs(s.sample.roll_deg-30)<0.1 && fabs(s.sample.pitch_deg)<0.1);
    // A board powered up tilted must retain its gravity direction, without a fall event.
    a[0]=0; a[1]=4.903325f; a[2]=8.492808f;
    c=config(); assert(motion_init(&s,&c));
    for (unsigned i=0;i<=200;++i) motion_update(&s,a,gyro,0,(uint64_t)i*10000,false,0);
    assert(fabs(s.sample.roll_deg-30)<0.1 && fabs(s.sample.longitudinal)<0.01 && !s.sample.fall_count);
    c=config(); assert(motion_init(&s,&c));
    a[0]=a[1]=0;
    for (unsigned i=0;i<=200;++i) {
        a[2]=9.80665f+sinf((float)i*0.62831853f); // 10 Hz vibration, 1 m/s2 amplitude.
        motion_update(&s,a,gyro,0,(uint64_t)i*10000,false,0);
    }
    assert(s.sample.vibration_rms>0.5 && s.sample.vibration_rms<0.8);
    assert(s.sample.peak_acceleration>10.7 && !s.sample.impact_count);
}
static void calibration_test(void) {
    imu_calibration_t calibration={0}; int32_t offsets[6]={0}; float scale=1;
    int16_t raw[6]={0,2025,3507,12,-8,5}; // Approx. 9.7 m/s2, tilted 30 degrees.
    for (unsigned i=0;i<200;++i) imu_calibration_add(&calibration,raw);
    assert(imu_calibration_finish(&calibration,offsets,&scale));
    assert(scale>1.01f && scale<1.02f && offsets[3]==12 && offsets[4]==-8);
    motion_state_t s; motion_config_t c=config(); assert(motion_init(&s,&c));
    float a[3],gyro[3]={0};
    for (unsigned i=0;i<3;++i) a[i]=raw[i]*(9.80665f/4096)*scale;
    for (unsigned i=0;i<=200;++i) motion_update(&s,a,gyro,0,(uint64_t)i*10000,true,0);
    assert(fabs(s.sample.magnitude-9.80665)<0.001 && fabs(s.sample.roll_deg-30)<0.1);
    assert(fabs(s.sample.linear[0])+fabs(s.sample.linear[1])+fabs(s.sample.linear[2])<0.002);
    assert(!s.sample.fall_count && !s.sample.impact_count);
    calibration=(imu_calibration_t){0};
    for (unsigned i=0;i<200;++i) {
        raw[0]=i%2 ? 300:-300; imu_calibration_add(&calibration,raw);
    }
    assert(!imu_calibration_finish(&calibration,offsets,&scale));
    calibration=(imu_calibration_t){0}; raw[0]=0; raw[3]=1311;
    for (unsigned i=0;i<200;++i) imu_calibration_add(&calibration,raw);
    assert(!imu_calibration_finish(&calibration,offsets,&scale)); // Steady fast rotation.
    calibration=(imu_calibration_t){0}; memset(raw,0,sizeof(raw));
    for (unsigned i=0;i<200;++i) imu_calibration_add(&calibration,raw);
    assert(!imu_calibration_finish(&calibration,offsets,&scale)); // No gravity reference.
}
void test_metrics(void) {
    navigation_test(); stationary_navigation_test(); motion_test(); calibration_test();
    fprintf(stderr,"Navigation/motion tests passed: checksums, outages, drift/jumps, wrap, gravity, braking, impact, fall, stale data and bounds.\n");
}
