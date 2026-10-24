#ifndef NAVCAR_FEEDBACK_V2_H
#define NAVCAR_FEEDBACK_V2_H
/* Optional 0x82 feedback extension. See 底盘反馈V2.md.
 * Packing only: caller supplies genuine, same-cycle measured values and flags.
 * No motor output, HAL, DMA ownership or control-mode inference in this helper.
 * Requires IEEE754 binary32 float (STM32). Output has exactly 46 bytes.
 */
#include <stdint.h>
#include <string.h>

#define NAVCAR_UART1_ALLOWED 1u
#define NAVCAR_WHEELS_VALID 2u
#define NAVCAR_GIMBAL_VALID 4u
#define NAVCAR_IMU_YAW_VALID 8u

static inline void navcar_fb_u32(uint8_t *p, uint32_t v) {
    p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24);
}
static inline void navcar_fb_f32(uint8_t *p, float v) {
    uint32_t bits; memcpy(&bits, &v, 4); navcar_fb_u32(p, bits);
}
static inline uint16_t navcar_fb_crc(const uint8_t *p, unsigned n) {
    uint16_t crc=0xffff;
    for(unsigned i=0;i<n;i++) {
        crc^=p[i];
        for(unsigned b=0;b<8;b++) crc=(uint16_t)((crc>>1)^((crc&1)?0xa001:0));
    }
    return crc;
}
static inline unsigned navcar_pack_feedback_v2(
        uint8_t out[46], uint16_t last_control_sequence, uint8_t flags,
        uint32_t sample_sequence, uint32_t measurement_ms, uint32_t authority_epoch,
        float vy_body_mps, float vx_body_mps, float chassis_ccw_radps,
        float gimbal_relative_ccw_rad, float imu_yaw_deg) {
    out[0]=0xa5;out[1]=0x5a;out[2]=1;out[3]=0x82;
    out[4]=(uint8_t)last_control_sequence;out[5]=(uint8_t)(last_control_sequence>>8);
    out[6]=36;out[7]=0;
    out[8]=1;out[9]=flags;out[10]=0;out[11]=0;
    navcar_fb_u32(out+12,sample_sequence);
    navcar_fb_u32(out+16,measurement_ms);
    navcar_fb_u32(out+20,authority_epoch);
    navcar_fb_f32(out+24,vy_body_mps);navcar_fb_f32(out+28,vx_body_mps);
    navcar_fb_f32(out+32,chassis_ccw_radps);navcar_fb_f32(out+36,gimbal_relative_ccw_rad);
    navcar_fb_f32(out+40,imu_yaw_deg);
    uint16_t crc=navcar_fb_crc(out+2,42);
    out[44]=(uint8_t)crc;out[45]=(uint8_t)(crc>>8);
    return 46;
}

/* Preferred revision 2: velocity is ALREADY transformed into gimbal/lidar axes.
 * No encoder-angle argument or validity bit is transmitted.
 * Do not pass body-axis velocities to this function.
 * flags: UART1_ALLOWED | WHEELS_VALID | IMU_YAW_VALID as actually applicable.
 */
static inline unsigned navcar_pack_feedback_yaw_v2(
        uint8_t out[46], uint16_t last_control_sequence, uint8_t flags,
        uint32_t sample_sequence, uint32_t measurement_ms, uint32_t authority_epoch,
        float forward_gimbal_mps, float right_gimbal_mps, float chassis_ccw_radps,
        float imu_yaw_deg) {
    navcar_pack_feedback_v2(out,last_control_sequence,flags,sample_sequence,measurement_ms,
        authority_epoch,forward_gimbal_mps,right_gimbal_mps,chassis_ccw_radps,0.f,imu_yaw_deg);
    out[8]=2;
    uint16_t crc=navcar_fb_crc(out+2,42);
    out[44]=(uint8_t)crc;out[45]=(uint8_t)(crc>>8);
    return 46;
}
#endif
