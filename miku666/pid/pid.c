#include "pid.h"

// 全局基础PID参数实例
PID_Base_TypeDef pid_base = {
    .Kp = 40.0f,
    .Ki = 0.8f,
    .Kd = 125.0f
};

void PID_init(PID_struct* pid_info) {
    pid_info->v_target = 0.0f;
    pid_info->v_current = 0.0f;
    pid_info->err = 0.0f;
    pid_info->err_prev_1 = 0.0f;
    pid_info->err_prev_2 = 0.0f;
    pid_info->speed[0] = pid_info->speed[1] = pid_info->speed[2] = 0.0f;
}

float PID_PWM_iteration(PID_struct* pid_info, float value_thres, float value_current) {
    pid_info->v_target = value_thres;
    pid_info->v_current = value_current;
    pid_info->err = pid_info->v_target - pid_info->v_current;

    float speed_p = pid_base.Kp * pid_info->err;

    pid_info->err_prev_2 = pid_info->err_prev_2 + pid_info->err;
    float speed_i = pid_base.Ki * pid_info->err_prev_2;

    float speed_d = pid_base.Kd * (pid_info->err - pid_info->err_prev_1);

    pid_info->speed[0] = speed_p;
    pid_info->speed[1] = speed_i;
    pid_info->speed[2] = speed_d;

    pid_info->err_prev_1 = pid_info->err;

    float speed = speed_p + speed_i + speed_d;

    if (speed > 1000.0f) {
        return 1000.0f;
    } else if (speed > 0.0f) {
        return speed;
    } else {
        return 1.0f;
    }
}

float Fuzzy_PID_PWM_iteration(PID_struct* pid_info, float value_thres, float value_current) {
    return PID_PWM_iteration(pid_info, value_thres, value_current);
}
