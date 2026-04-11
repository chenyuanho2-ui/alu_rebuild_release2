#include "temp_filter.h"
#include "ads1118.h"
#include "thermocouple.h"
#include <stdio.h>

// 静态私有变量
static float latest_avg_temp = 0.0f;     // 给 PID 用的滤波值

// 滤波变量
static float filtered_val = -999.0f;

void TempFilter_Init(void) {
    filtered_val = -999.0f;
    latest_avg_temp = 0.0f;
    ADS1118_Init();
}

void TempFilter_Process(void) {
    // 1. 获取原始热电偶计算值
    float raw_temp = Get_Actual_Temperature(); 
    
    // 2. 立即进行 11 点校准
    float cal_temp = ADS1118_CalibrateTemp(raw_temp);
    
    // 3. 一阶低通滤波 (EMA) - 彻底消除 ADC 噪声且保持曲线连续
    const float alpha = 0.15f; 
    if (filtered_val == -999.0f) filtered_val = cal_temp;
    filtered_val = alpha * cal_temp + (1.0f - alpha) * filtered_val;
    
    // 4. 更新供 PID 使用的全局变量
    latest_avg_temp = filtered_val;

    // 每 20ms 输出一次最鲜活的 PID 供电温度，正好可以配合串口示波器看加热曲线！
    printf("[Monitor] 20ms PID Temp: %.2f C\r\n", latest_avg_temp);
}

// ---------------------------------------------------------
// 对外提供的安全获取接口 (Getter)
// ---------------------------------------------------------

// 供 PID 控温和 UI 显示调用
float TempFilter_GetUIAvgTemp(void) {
    return latest_avg_temp;
}
