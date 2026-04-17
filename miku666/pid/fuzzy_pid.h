#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// 模糊PID结构体
typedef struct {
    float v_target;       // 目标值 (设定的目标温度)
    float v_current;      // 当前值 (当前反馈温度)
    float err;            // 当前误差 e
    float err_prev_1;     // 上次误差
    float err_prev_2;     // 上上次误差 (用于积分)
    
    // 实际输出的PID参数
    float Kp;
    float Ki;
    float Kd;
    float speed[3];       // 输出的P, I, D分量

    // ================= 以下为你需要填写的“魔法参数” =================
    float Kp_base;        // 基础Kp (你原本调好的普通Kp)
    float Ki_base;        // 基础Ki (你原本调好的普通Ki)
    float Kd_base;        // 基础Kd (你原本调好的普通Kd)

    float e_max;          // 误差的最大范围 (例如：最大温差设定为 50度)
    float ec_max;         // 误差变化率的最大范围 (例如：10ms内温度最大变化 2度)

    float Kp_weight;      // 模糊规则对Kp的调节力度 (倍率)
    float Ki_weight;      // 模糊规则对Ki的调节力度
    float Kd_weight;      // 模糊规则对Kd的调节力度
    // ================================================================

} FuzzyPID_struct;

void FuzzyPID_init(FuzzyPID_struct* fuzzy_pid);
float FuzzyPID_Calculate(FuzzyPID_struct* fuzzy_pid, float value_thres, float value_current);

#ifdef __cplusplus
}
#endif

#endif
