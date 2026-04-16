#include "fuzzy_pid.h"

void FuzzyPID_init(FuzzyPID_struct* fuzzy_pid) {
    fuzzy_pid->v_target = 0.0f;
    fuzzy_pid->v_current = 0.0f;
    fuzzy_pid->err = 0.0f;
    fuzzy_pid->err_prev_1 = 0.0f;
    fuzzy_pid->err_prev_2 = 0.0f;
    fuzzy_pid->speed[0] = fuzzy_pid->speed[1] = fuzzy_pid->speed[2] = 0.0f;
    fuzzy_pid->kp_scale = 1.0f;
    fuzzy_pid->ki_scale = 1.0f;
    fuzzy_pid->kd_scale = 1.0f;
    fuzzy_pid->Kp = 40.0f;
    fuzzy_pid->Ki = 0.8f;
    fuzzy_pid->Kd = 125.0f;
}

static void Fuzzy_Bound_Fix(float* value, float min_val, float max_val) {
    if (*value < min_val) *value = min_val;
    if (*value > max_val) *value = max_val;
}

static float Fuzzy_Compute_Scale(float error, float de) {
    float abs_e = (error >= 0.0f) ? error : -error;
    float abs_de = (de >= 0.0f) ? de : -de;

    if (abs_e < 5.0f) {
        return 1.2f;
    } else if (abs_e < 15.0f) {
        if (abs_de < 2.0f) return 1.1f;
        else return 0.9f;
    } else {
        if (abs_de > 5.0f) return 0.7f;
        else return 0.85f;
    }
}

float FuzzyPID_Calculate(FuzzyPID_struct* fuzzy_pid, float value_thres, float value_current) {
    fuzzy_pid->v_target = value_thres;
    fuzzy_pid->v_current = value_current;
    fuzzy_pid->err = fuzzy_pid->v_target - fuzzy_pid->v_current;

    float de = fuzzy_pid->err - fuzzy_pid->err_prev_1;
    float scale = Fuzzy_Compute_Scale(fuzzy_pid->err, de);

    if (scale > 1.0f) {
        fuzzy_pid->kp_scale = scale;
        fuzzy_pid->kd_scale = 2.0f - scale;
        fuzzy_pid->ki_scale = 0.5f;
    } else {
        fuzzy_pid->kp_scale = scale;
        fuzzy_pid->kd_scale = scale;
        fuzzy_pid->ki_scale = 1.5f - 0.5f * scale;
    }

    float speed_p = fuzzy_pid->Kp * fuzzy_pid->kp_scale * fuzzy_pid->err;

    fuzzy_pid->err_prev_2 = fuzzy_pid->err_prev_2 + fuzzy_pid->err;
    float speed_i = fuzzy_pid->Ki * fuzzy_pid->ki_scale * fuzzy_pid->err_prev_2;

    float speed_d = fuzzy_pid->Kd * fuzzy_pid->kd_scale * de;

    fuzzy_pid->speed[0] = speed_p;
    fuzzy_pid->speed[1] = speed_i;
    fuzzy_pid->speed[2] = speed_d;

    fuzzy_pid->err_prev_1 = fuzzy_pid->err;

    float speed = speed_p + speed_i + speed_d;

    Fuzzy_Bound_Fix(&speed, 1.0f, 1000.0f);

    return speed;
}
