#include "advanced_pid.h"

void AdvPID_Init(AdvPID_struct* adv_pid, float kp, float ki, float kd) {
    adv_pid->err = 0.0f;
    adv_pid->err_prev_1 = 0.0f;
    adv_pid->err_prev_2 = 0.0f;
    adv_pid->err_prev_d = 0.0f;
    adv_pid->last_measured = 0.0f;
    adv_pid->last_pwm_out = 0.0f;
    adv_pid->last_d_out = 0.0f;
    adv_pid->integral = 0.0f;
    adv_pid->speed[0] = adv_pid->speed[1] = adv_pid->speed[2] = 0.0f;

    // 初始化高级PID可调参数
    adv_pid->deadband_threshold = 0.5f;
    adv_pid->mode_switch_threshold = 10.0f;
    adv_pid->static_err_threshold = 5.0f;
    adv_pid->temp_change_threshold = 5.0f;
    adv_pid->output_max_pd_mode = 750.0f;
    adv_pid->integral_limit = 1000.0f;
    adv_pid->output_max_pid_mode = 1000.0f;
    adv_pid->static_delay_samples = 300;

    // 初始化温度历史
    for (int i = 0; i < 100; i++) {
        adv_pid->temp_history[i] = 0.0f;
    }
    adv_pid->temp_history_index = 0;
    adv_pid->temp_history_count = 0;
    adv_pid->heating_time_samples = 0;
}

float AdvPID_Calculate(AdvPID_struct* adv_pid, float target, float measured) {
    adv_pid->err = target - measured;
    float abs_err = (adv_pid->err >= 0.0f) ? adv_pid->err : -adv_pid->err;

    // ================================================
    // 高级PID可调参数
    // ================================================
    // 微分滤波器系数：D值 = 0.6*上次D输出 + 0.4*原始D值 (滤波平滑)
    const float D_FILTER_COEF_LAST = 0.6f;
    const float D_FILTER_COEF_RAW  = 0.4f;
    // ================================================

    // 更新温度历史（每10ms调用一次，100个样本=1s）
    adv_pid->temp_history[adv_pid->temp_history_index] = measured;
    adv_pid->temp_history_index = (adv_pid->temp_history_index + 1) % 100;
    if (adv_pid->temp_history_count < 100) {
        adv_pid->temp_history_count++;
    }

    // 计算1s内温度变化量
    float temp_change = 0.0f;
    if (adv_pid->temp_history_count >= 2) {
        float oldest_temp = adv_pid->temp_history[(adv_pid->temp_history_index + 100 - adv_pid->temp_history_count) % 100];
        float newest_temp = adv_pid->temp_history[(adv_pid->temp_history_index + 99) % 100];
        temp_change = (newest_temp >= oldest_temp) ? (newest_temp - oldest_temp) : (oldest_temp - newest_temp);
    }

    // 静态误差检测：误差>静态阈值 且 1s内温度变化<阈值 且 加热>延迟 → 强制开启积分
    uint8_t force_integral = 0;
    if (abs_err > adv_pid->static_err_threshold && temp_change < adv_pid->temp_change_threshold && adv_pid->temp_history_count >= 50 && adv_pid->heating_time_samples >= adv_pid->static_delay_samples) {
        force_integral = 1;
    }
    adv_pid->heating_time_samples++;

    if (abs_err < adv_pid->deadband_threshold) {
        return adv_pid->last_pwm_out;
    }

    float P_out = pid_base.Kp * adv_pid->err;

    float D_raw = -(measured - adv_pid->last_measured);
    float filtered_D = D_FILTER_COEF_LAST * adv_pid->last_d_out + D_FILTER_COEF_RAW * D_raw;
    adv_pid->last_d_out = filtered_D;
    float D_out = pid_base.Kd * filtered_D;

    float I_out = 0.0f;
    float Output = 0.0f;

    if (abs_err > adv_pid->mode_switch_threshold || force_integral) {
        Output = P_out + D_out;
        if (Output > adv_pid->output_max_pd_mode) Output = adv_pid->output_max_pd_mode;
        if (Output < 0.0f) Output = 0.0f;
        if (force_integral) {
            adv_pid->integral = adv_pid->integral + adv_pid->err * 0.5f;  // 加速积分累积
            if (adv_pid->integral > adv_pid->integral_limit) adv_pid->integral = adv_pid->integral_limit;
            I_out = pid_base.Ki * adv_pid->integral;
            Output = P_out + I_out + D_out;
            if (Output > adv_pid->output_max_pid_mode) Output = adv_pid->output_max_pid_mode;
            if (Output < 0.0f) Output = 0.0f;
        }
    } else {
        adv_pid->integral = adv_pid->integral + adv_pid->err;
        if (adv_pid->integral > adv_pid->integral_limit) {
            adv_pid->integral = adv_pid->integral_limit;
        } else if (adv_pid->integral < -adv_pid->integral_limit) {
            adv_pid->integral = -adv_pid->integral_limit;
        }
        I_out = pid_base.Ki * adv_pid->integral;
        Output = P_out + I_out + D_out;
        if (Output > adv_pid->output_max_pid_mode) Output = adv_pid->output_max_pid_mode;
        if (Output < 0.0f) Output = 0.0f;
    }

    adv_pid->speed[0] = P_out;
    adv_pid->speed[1] = I_out;
    adv_pid->speed[2] = D_out;

    adv_pid->err_prev_1 = adv_pid->err;
    adv_pid->err_prev_2 = adv_pid->err_prev_1;
    adv_pid->last_measured = measured;
    adv_pid->last_pwm_out = Output;

    return Output;
}
