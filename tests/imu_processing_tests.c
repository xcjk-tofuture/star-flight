#include "imu_pipeline.h"
#include "fusion.h"
#include "gyro_calibration.h"
#include "imu_processing_config.h"
#include "imu_calibration_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const float gravity[3]={0,0,9.80665f},zero[3]={0},north[3]={25,0,40};
static void filter_tests(void) {
    uav_biquad_t filter;
    float output[3];
    assert(uav_biquad_configure(&filter,500,30)==0);
    assert(uav_biquad_apply(&filter,gravity,output)==0);
    assert(fabsf(output[2]-gravity[2])<1e-5f); /* No false zero-g first frame. */
    float energy=0;
    for (unsigned n=0;n<2000;n++) {
        float input[3]={sinf(n*2*3.14159265358979323846f*150/500),0,9.80665f};
        assert(uav_biquad_apply(&filter,input,output)==0);
        if (n>=500) energy+=output[0]*output[0];
    }
    assert(sqrtf(energy/1500)<.05f);
    float invalid[3]={NAN,0,1};
    assert(uav_biquad_apply(&filter,invalid,output)==-1);
    assert(uav_biquad_apply(&filter,gravity,output)==0 && fabsf(output[2]-gravity[2])<1e-5f);
    assert(uav_biquad_configure(&filter,100,50)==-1);
}
static void pipeline_tests(void) {
    uav_imu_pipeline_t pipeline;
    uav_imu_frame_t frame;
    const float spin[3]={0,0,1};
    assert(uav_imu_pipeline_init(&pipeline,&uav_board_imu_processing)==0);
    uint32_t now=UINT32_MAX-3000u;
    assert(uav_imu_pipeline_push(&pipeline,gravity,spin,now)==0);
    assert(uav_imu_pipeline_consume(&pipeline,&frame)==-1);
    for (unsigned n=0;n<3;n++) { now+=2000; assert(uav_imu_pipeline_push(&pipeline,gravity,spin,now)==0); }
    assert(uav_imu_pipeline_consume(&pipeline,&frame)==0);
    assert(fabsf(frame.dt_s-.006f)<1e-6f && fabsf(frame.gyro_average[2]-1)<1e-6f);
    assert(uav_imu_pipeline_consume(&pipeline,&frame)==-1); /* An interval is consumed once. */
    assert(uav_imu_pipeline_push(&pipeline,gravity,spin,now)==-1); /* Duplicate timestamp. */
    assert(uav_imu_pipeline_consume(&pipeline,&frame)==-1);
    now+=2000; assert(uav_imu_pipeline_push(&pipeline,gravity,spin,now)==0);
    now+=20000; assert(uav_imu_pipeline_push(&pipeline,gravity,spin,now)==-1);
    assert(pipeline.stats.timing_resets==2);
    now+=2000; assert(uav_imu_pipeline_push(&pipeline,gravity,spin,now)==0);
    now+=2000; assert(uav_imu_pipeline_push(&pipeline,gravity,spin,now)==0);
    const float bias[3]={0,0,.25f};
    uav_imu_pipeline_set_bias(&pipeline,bias);
    now+=2000; assert(uav_imu_pipeline_push(&pipeline,gravity,spin,now)==0);
    assert(uav_imu_pipeline_consume(&pipeline,&frame)==0);
    assert(fabsf(frame.gyro_average[2]-.75f)<1e-6f && fabsf(frame.gyro_control[2]-.75f)<1e-5f);
    float bad[3]={NAN,0,0};
    assert(uav_imu_pipeline_push(&pipeline,gravity,bad,now+2000)==-1);
    assert(uav_imu_pipeline_consume(&pipeline,&frame)==-1);
    assert(pipeline.stats.invalid_samples==1);
}
static void calibration_tests(void) {
    uav_imu_pipeline_t p;
    uav_gyro_calibration_t cal;
    uav_imu_frame_t frame;
    float bias[3]={99,99,99};
    const float actual[3]={-.025f,.015f,.008f};
    assert(uav_imu_pipeline_init(&p,&uav_board_imu_processing)==0);
    assert(uav_gyro_calibration_init(&cal,&uav_board_gyro_calibration)==0);
    uint32_t random=7, next_frame=5000;
    for (unsigned n=0;n<2500 && !cal.ready;n++) {
        float gyro[3];
        for (unsigned i=0;i<3;i++) {
            random=random*1664525u+1013904223u;
            /* Reproduce 0.05..0.08rad/s raw noise with deterministic zero-mean noise. */
            gyro[i]=actual[i]+(((random>>8)&65535)/65535.0f-.5f)*.20f;
        }
        assert(uav_imu_pipeline_push(&p,gravity,gyro,n*2000u)==0);
        if (n*2000u>=next_frame) {
            next_frame+=5000;
            if (uav_imu_pipeline_consume(&p,&frame)==0 && n*2>=UAV_GYRO_CAL_WARMUP_MS)
                uav_gyro_calibration_feed(&cal,frame.gyro_calibration,frame.acc,bias);
        }
    }
    assert(cal.ready && !cal.failed);
    for (unsigned i=0;i<3;i++) assert(fabsf(bias[i]-actual[i])<.008f);
    assert(uav_gyro_calibration_expire(&cal,40000,UAV_GYRO_CAL_TIMEOUT_MS)==0);
    assert(uav_gyro_calibration_init(&cal,&uav_board_gyro_calibration)==0);
    const float motion[3]={-.4f,0,0};
    for (unsigned n=0;n<100;n++) assert(uav_gyro_calibration_feed(&cal,motion,gravity,bias)==UAV_GYRO_REJECTED);
    assert(cal.reject_reason==UAV_GYRO_REASON_RATE);
    assert(uav_gyro_calibration_expire(&cal,29999,30000)==0);
    assert(uav_gyro_calibration_expire(&cal,30000,30000)==1 && !cal.ready);
    float saved[3]; memcpy(saved,bias,sizeof(saved));
    assert(uav_gyro_calibration_feed(&cal,zero,gravity,bias)==UAV_GYRO_FAILED);
    assert(memcmp(saved,bias,sizeof(saved))==0);
    assert(uav_gyro_calibration_init(&cal,&uav_board_gyro_calibration)==0 && !cal.failed);
}
static void fusion_tests(void) {
    uav_fusion_t fusion;
    assert(uav_fusion_init(&fusion,&uav_board_fusion)==0);
    assert(uav_fusion_step(&fusion,gravity,zero,north,1000,1000,1,.005f)==0);
    assert(fusion.mag_used && fabsf(fusion.roll_deg)<1e-5f);
    float corrupt_mag[3]={NAN,0,0};
    assert(uav_fusion_step(&fusion,gravity,zero,corrupt_mag,6000,6000,1,.005f)==0);
    assert(!fusion.mag_used);
    float disturbance[3]={250,0,400};
    assert(uav_fusion_step(&fusion,gravity,zero,disturbance,11000,11000,1,.005f)==0 && !fusion.mag_used);
    uint32_t now=11000;
    for (unsigned n=0;n<50;n++) {
        now+=5000;
        assert(uav_fusion_step(&fusion,gravity,zero,north,now,11000,1,.005f)==0);
    }
    assert(!fusion.mag_used); /* Held data does not count as ten fresh recovery samples. */
    for (unsigned n=0;n<10;n++) {
        now+=20000; assert(uav_fusion_step(&fusion,gravity,zero,north,now,now,1,.005f)==0);
    }
    assert(fusion.mag_used && fusion.mag_weight<.1f);
    assert(uav_fusion_init(&fusion,&uav_board_fusion)==0);
    assert(uav_fusion_step(&fusion,gravity,zero,zero,0,0,0,.005f)==0);
    float spin[3]={0,0,1},acceleration[3]={0,0,19.6133f};
    now=0;
    for (unsigned n=0;n<200;n++) {
        uint32_t dt=n%2 ? 6000:4000; now+=dt;
        assert(uav_fusion_step(&fusion,acceleration,spin,zero,now,0,0,dt*1e-6f)==0);
    }
    assert(fusion.accel_weight==0 && fabsf(fusion.yaw_deg+57.2957795f)<.01f);
    float q[4]; memcpy(q,fusion.q,sizeof(q));
    assert(uav_fusion_step(&fusion,gravity,spin,zero,now,0,0,.021f)==-1);
    assert(memcmp(q,fusion.q,sizeof(q))==0);
    float bad_gyro[3]={INFINITY,0,0};
    assert(uav_fusion_step(&fusion,gravity,bad_gyro,zero,now,0,0,.005f)==-1);
    assert(memcmp(q,fusion.q,sizeof(q))==0);
    float tilt[3]={0,4.903325f,8.492808f};
    assert(uav_fusion_init(&fusion,&uav_board_fusion)==0);
    assert(uav_fusion_step(&fusion,tilt,zero,zero,0,0,0,.005f)==0);
    assert(fabsf(fusion.roll_deg-30)<.01f); /* Startup uses measured tilt, not identity. */
}
int main(void) {
    filter_tests(); pipeline_tests(); calibration_tests(); fusion_tests();
    puts("PASS IMU: seeded filters, anti-noise calibration, bounded failure, real dt, gaps/wrap, magnetic recovery and quaternion rollback");
    return 0;
}
