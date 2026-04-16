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
} AdvPID_struct;

void AdvPID_Init(AdvPID_struct* adv_pid, float kp, float ki, float kd);
float AdvPID_Calculate(AdvPID_struct* adv_pid, float target, float measured);

#ifdef __cplusplus
}
#endif

#endif
