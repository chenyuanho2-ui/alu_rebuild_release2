#ifndef __ALU_PID_H
#define __ALU_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// 基础PID参数结构体
typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_Base_TypeDef;

// 全局共享的基础PID参数
extern PID_Base_TypeDef pid_base;

typedef struct {
    float v_target;
    float v_current;
    float err;
    float err_prev_1;
    float err_prev_2;
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
