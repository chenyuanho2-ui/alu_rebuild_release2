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

    if (abs_err < 0.5f) {
        return adv_pid->last_pwm_out;
    }

    float P_out = adv_pid->Kp * adv_pid->err;

    float D_raw = -(measured - adv_pid->last_measured);
    float D_out = 0.6f * adv_pid->last_d_out + 0.4f * D_raw;
    adv_pid->last_d_out = D_out;

    float Output = 0.0f;

    if (abs_err > 10.0f) {
        Output = P_out + D_out;
        if (Output > 500.0f) Output = 500.0f;
        if (Output < 0.0f) Output = 0.0f;
    } else {
        adv_pid->integral = adv_pid->integral + adv_pid->err;
        float I_out = adv_pid->Ki * adv_pid->integral;
        Output = P_out + I_out + D_out;
        if (Output > 1000.0f) Output = 1000.0f;
        if (Output < 0.0f) Output = 0.0f;
    }

    adv_pid->err_prev_1 = adv_pid->err;
    adv_pid->err_prev_2 = adv_pid->err_prev_1;
    adv_pid->last_measured = measured;
    adv_pid->last_pwm_out = Output;

    return Output;
}
