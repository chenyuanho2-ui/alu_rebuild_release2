#ifndef __TEMP_FILTER_H
#define __TEMP_FILTER_H

#include "main.h"

// 函数声明
void TempFilter_Init(void);
void TempFilter_Process(void);

// 数据获取接口
float TempFilter_GetUIAvgTemp(void); // 获取实时滤波温度（供 PID 控温和 UI 显示调用）

#endif /* __TEMP_FILTER_H */
