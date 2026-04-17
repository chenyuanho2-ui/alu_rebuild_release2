#include "advanced_pid.h"

void AdvPID_Init(AdvPID_struct* adv_pid, float kp, float ki, float kd) {
    adv_pid->Kp = kp;
    adv_pid->Ki = ki;
    adv_pid->Kd = kd;
    adv_pid->err = 0.0f;
    adv_pid->err_prev_1 = 0.0f;
    adv_pid->err_prev_2 = 0.0f;
    adv_pid->err_prev_d = 0.0f;
    adv_pid->last_measured = 0.0f;
    adv_pid->last_pwm_out = 0.0f;
    adv_pid->last_d_out = 0.0f;
    adv_pid->integral = 0.0f;
    adv_pid->speed[0] = adv_pid->speed[1] = adv_pid->speed[2] = 0.0f;

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
    // 死区阈值：误差小于此值时不调节，保持上次输出；注意对应的温度抖动
    const float DEADBAND_THRESHOLD = 0.5f;

    // 微分滤波器系数：D值 = 0.6*上次D输出 + 0.4*原始D值 (滤波平滑)
    const float D_FILTER_COEF_LAST = 0.6f;
    const float D_FILTER_COEF_RAW  = 0.4f;

    // 模式切换温差阈值：大误差用P+D，小误差用P+I+D
    const float MODE_SWITCH_THRESHOLD = 10.0f;

    // 静态误差强制积分参数
    const float STATIC_ERR_THRESHOLD = 5.0f;   // 静态误差大于此值时检测
    const float TEMP_CHANGE_THRESHOLD = 5.0f;   // 1s内温度变化小于此值认为静态

    // P+D模式输出上限
    const float OUTPUT_MAX_PD_MODE = 750.0f;

    // 积分限幅值（防止积分饱和）
    const float INTEGRAL_LIMIT = 1000.0f;

    // P+I+D模式输出上限
    const float OUTPUT_MAX_PID_MODE = 1000.0f;

    const uint32_t STATIC_DELAY_SAMPLES = 300;  // 加热3秒后才启用静态检测(300*10ms=3s)
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

    // 静态误差检测：误差>5度 且 1s内温度变化<5度 且 加热>3秒 → 强制开启积分
    uint8_t force_integral = 0;
    if (abs_err > STATIC_ERR_THRESHOLD && temp_change < TEMP_CHANGE_THRESHOLD && adv_pid->temp_history_count >= 50 && adv_pid->heating_time_samples >= STATIC_DELAY_SAMPLES) {
        force_integral = 1;
    }
    adv_pid->heating_time_samples++;

    if (abs_err < DEADBAND_THRESHOLD) {
        return adv_pid->last_pwm_out;
    }

    float P_out = adv_pid->Kp * adv_pid->err;

    float D_raw = -(measured - adv_pid->last_measured);
    float filtered_D = D_FILTER_COEF_LAST * adv_pid->last_d_out + D_FILTER_COEF_RAW * D_raw;
    adv_pid->last_d_out = filtered_D;
    float D_out = adv_pid->Kd * filtered_D;

    float I_out = 0.0f;
    float Output = 0.0f;

    if (abs_err > MODE_SWITCH_THRESHOLD || force_integral) {
        Output = P_out + D_out;
        if (Output > OUTPUT_MAX_PD_MODE) Output = OUTPUT_MAX_PD_MODE;
        if (Output < 0.0f) Output = 0.0f;
        if (force_integral) {
            adv_pid->integral = adv_pid->integral + adv_pid->err * 0.5f;  // 加速积分累积
            if (adv_pid->integral > INTEGRAL_LIMIT) adv_pid->integral = INTEGRAL_LIMIT;
            I_out = adv_pid->Ki * adv_pid->integral;
            Output = P_out + I_out + D_out;
            if (Output > OUTPUT_MAX_PID_MODE) Output = OUTPUT_MAX_PID_MODE;
            if (Output < 0.0f) Output = 0.0f;
        }
    } else {
        adv_pid->integral = adv_pid->integral + adv_pid->err;
        if (adv_pid->integral > INTEGRAL_LIMIT) {
            adv_pid->integral = INTEGRAL_LIMIT;
        } else if (adv_pid->integral < -INTEGRAL_LIMIT) {
            adv_pid->integral = -INTEGRAL_LIMIT;
        }
        I_out = adv_pid->Ki * adv_pid->integral;
        Output = P_out + I_out + D_out;
        if (Output > OUTPUT_MAX_PID_MODE) Output = OUTPUT_MAX_PID_MODE;
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
