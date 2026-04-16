#ifndef __ADVANCED_PID_H
#define __ADVANCED_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float err;
    float err_prev_1;
    float err_prev_2;
    float err_prev_d;
    float last_measured;
    float last_pwm_out;
    float last_d_out;
    float integral;
    float speed[3];  // P、I、D输出，统一命名

    // 静态误差检测用的温度历史（100个样本=1s）
    float temp_history[100];
    uint8_t temp_history_index;
    uint8_t temp_history_count;
    uint32_t heating_time_samples;  // 加热累计时间(10ms单位)，用于静态检测延迟
} AdvPID_struct;

void AdvPID_Init(AdvPID_struct* adv_pid, float kp, float ki, float kd);
float AdvPID_Calculate(AdvPID_struct* adv_pid, float target, float measured);

#ifdef __cplusplus
}
#endif

#endif
