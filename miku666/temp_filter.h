#ifndef __TEMP_FILTER_H
#define __TEMP_FILTER_H

#include "main.h"

// 函数声明
void TempFilter_Init(void);
void TempFilter_Process(void);

// 新增：双通道数据获取接口
float TempFilter_GetLatestAvgTemp(void); // 通道A：获取 20ms 实时滤波温度（专供 PID 控温调用）
float TempFilter_GetUIAvgTemp(void);     // 通道B：获取 250ms 平均温度（专供 TouchGFX 屏幕显示调用）

#endif /* __TEMP_FILTER_H */
