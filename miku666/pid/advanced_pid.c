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
}

float AdvPID_Calculate(AdvPID_struct* adv_pid, float target, float measured) {
    adv_pid->err = target - measured;
    float abs_err = (adv_pid->err >= 0.0f) ? adv_pid->err : -adv_pid->err;

    // ================================================
    // 高级PID可调参数
    // ================================================
    // 死区阈值：误差小于此值时不调节，保持上次输出
    const float DEADBAND_THRESHOLD = 0.5f;

    // 微分滤波器系数：D值 = 0.6*上次D输出 + 0.4*原始D值 (滤波平滑)
    const float D_FILTER_COEF_LAST = 0.6f;
    const float D_FILTER_COEF_RAW  = 0.4f;

    // 模式切换阈值：大误差用P+D，小误差用P+I+D
    const float MODE_SWITCH_THRESHOLD = 10.0f;

    // P+D模式输出上限
    const float OUTPUT_MAX_PD_MODE = 800.0f;

    // 积分限幅值（防止积分饱和）
    const float INTEGRAL_LIMIT = 1500.0f;

    // P+I+D模式输出上限
    const float OUTPUT_MAX_PID_MODE = 1000.0f;
    // ================================================

    if (abs_err < DEADBAND_THRESHOLD) {
        return adv_pid->last_pwm_out;
    }

    float P_out = adv_pid->Kp * adv_pid->err;

    float D_raw = -(measured - adv_pid->last_measured);
    float filtered_D = D_FILTER_COEF_LAST * adv_pid->last_d_out + D_FILTER_COEF_RAW * D_raw;
    adv_pid->last_d_out = filtered_D;
    float D_out = adv_pid->Kd * filtered_D;

    float Output = 0.0f;

    if (abs_err > MODE_SWITCH_THRESHOLD) {
        Output = P_out + D_out;
        if (Output > OUTPUT_MAX_PD_MODE) Output = OUTPUT_MAX_PD_MODE;
        if (Output < 0.0f) Output = 0.0f;
    } else {
        adv_pid->integral = adv_pid->integral + adv_pid->err;
        if (adv_pid->integral > INTEGRAL_LIMIT) {
            adv_pid->integral = INTEGRAL_LIMIT;
        } else if (adv_pid->integral < -INTEGRAL_LIMIT) {
            adv_pid->integral = -INTEGRAL_LIMIT;
        }
        float I_out = adv_pid->Ki * adv_pid->integral;
        Output = P_out + I_out + D_out;
        if (Output > OUTPUT_MAX_PID_MODE) Output = OUTPUT_MAX_PID_MODE;
        if (Output < 0.0f) Output = 0.0f;
    }

    adv_pid->err_prev_1 = adv_pid->err;
    adv_pid->err_prev_2 = adv_pid->err_prev_1;
    adv_pid->last_measured = measured;
    adv_pid->last_pwm_out = Output;

    return Output;
}
