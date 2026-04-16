#ifndef __ALU_PID_H
#define __ALU_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
    float v_target;
    float v_current;
    float err;
    float err_prev_1;
    float err_prev_2;
    float Kp;
    float Ki;
    float Kd;
    float speed[3];
} PID_struct;

extern PID_struct pid_TEMP;

void PID_init(PID_struct* pid_info);
float PID_PWM_iteration(PID_struct* pid_info, float value_thres, float value_current);
float Fuzzy_PID_PWM_iteration(PID_struct* pid_info, float value_thres, float value_current);

#ifdef __cplusplus
}
#endif

#endif
