#ifndef __ADVANCED_PID_H
#define __ADVANCED_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"

typedef struct {
    float err;
    float err_prev_1;
    float err_prev_2;
    float err_prev_d;
    float last_measured;
    float last_pwm_out;
    float last_d_out;
    float integral;
    float speed[3];  // P、I、D输出，统一命名

    // 高级PID可调参数
    float deadband_threshold;        // 死区阈值
    float mode_switch_threshold;     // 模式切换温差阈值
    float static_err_threshold;      // 静态误差阈值
    float temp_change_threshold;     // 温度变化阈值
    float output_max_pd_mode;        // P+D模式输出上限
    float integral_limit;            // 积分限幅值
    float output_max_pid_mode;       // P+I+D模式输出上限
    uint32_t static_delay_samples;   // 静态检测延迟

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
