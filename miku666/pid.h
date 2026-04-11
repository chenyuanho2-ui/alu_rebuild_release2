#ifndef __ALU_PID_H
#define __ALU_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// 从 alu_temp.h 迁移过来的 PID 结构体
typedef struct {
    float v_target;
    float v_current;
    float err;
    float err_prev_1;
    float err_prev_2;
    float Kp;
    float Ki;
    float Kd;
    float speed[3]; // 分别保存 P, I, D 的输出分量
} PID_struct;

// 全局 PID 实例声明 (在 alu_control.c 中定义)
extern PID_struct pid_TEMP;

// 暴露给外部的任务接口
void PID_init(PID_struct* pid_info);
float PID_PWM_iteration(PID_struct* pid_info, float value_thres, float value_current);

#ifdef __cplusplus
}
#endif

#endif /* __ALU_PID_H */
