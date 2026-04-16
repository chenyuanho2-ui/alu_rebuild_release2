#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"

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
    float kp_scale;
    float ki_scale;
    float kd_scale;
} FuzzyPID_struct;

void FuzzyPID_init(FuzzyPID_struct* fuzzy_pid);
float FuzzyPID_Calculate(FuzzyPID_struct* fuzzy_pid, float value_thres, float value_current);

#ifdef __cplusplus
}
#endif

#endif
