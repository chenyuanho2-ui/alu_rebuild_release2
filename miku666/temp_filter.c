#include "temp_filter.h"
#include "ads1118.h"
#include "thermocouple.h"
#include <stdio.h>

#define SAMPLE_INTERVAL_MS   20   // 控制采样间隔：20ms采样一次 (50Hz，保证PID闭环质量)
#define UI_REFRESH_INTERVAL_MS 250 // UI刷新间隔：250ms计算一次均值 (4Hz，人眼看着不闪烁)

// 静态私有变量 (保存双通道的最新结果)
static float latest_avg_temp = 0.0f;     // 给 PID 用的 20ms 滤波值
static float ui_display_temp = 0.0f;     // 给 UI 用的 250ms 均值

// 状态机时间戳与计算变量
static uint32_t last_sample_tick = 0;  
static uint32_t last_print_tick = 0;   
static float temp_accumulator = 0;      
static uint32_t sample_count = 0;       

void TempFilter_Init(void) {
    last_sample_tick = HAL_GetTick();
    last_print_tick = HAL_GetTick();
    temp_accumulator = 0;
    sample_count = 0;
    latest_avg_temp = 0.0f;
    ui_display_temp = 0.0f;
    ADS1118_Init();
}

void TempFilter_Process(void) {
    uint32_t current_tick = HAL_GetTick();

    // =================================================================
    // --- 任务 A: 高频采样、校准、滤波 (约每 20ms 执行一次) ---
    // 注意：这里用 if 判断，即使被频繁调用也不会卡死 CPU，是纯纯的非阻塞机制
    // =================================================================
    if (current_tick - last_sample_tick >= SAMPLE_INTERVAL_MS) {
        // 1. 获取原始热电偶计算值
        float raw_temp = Get_Actual_Temperature(); 
        
        // 2. 立即进行 11 点校准
        float cal_temp = ADS1118_CalibrateTemp(raw_temp);
        
        // 3. 一阶低通滤波 (EMA) - 彻底消除 ADC 噪声且保持曲线连续
        static float filtered_val = -999.0f;
        const float alpha = 0.15f; 
        if (filtered_val == -999.0f) filtered_val = cal_temp;
        filtered_val = alpha * cal_temp + (1.0f - alpha) * filtered_val;
        
        // 4. 更新【通道A】：供 PID 使用的全局变量 (20ms 更新一次，非常丝滑)
        latest_avg_temp = filtered_val;

		// 每 20ms 输出一次最鲜活的 PID 供电温度，正好可以配合串口示波器看加热曲线！
        printf("[Monitor] 20ms PID Temp: %.2f C\r\n", latest_avg_temp);
        
        // 5. 累加用于 250ms 任务的平均值
        temp_accumulator += filtered_val;
        sample_count++;
        
        last_sample_tick = current_tick;
    }

    // =================================================================
    // --- 任务 B: 专供 UI 和串口的平均值任务 (约每 250ms 执行一次) ---
    // =================================================================
    if (current_tick - last_print_tick >= UI_REFRESH_INTERVAL_MS ) {
        if (sample_count > 0) {
            // 计算这 250ms 内收集到的数据的平均值
            ui_display_temp = temp_accumulator / sample_count;
            

            
            // 清零，准备下一个 250ms 的周期
            temp_accumulator = 0;
            sample_count = 0;
        }
        last_print_tick = current_tick;
    }
}

// ---------------------------------------------------------
// 对外提供的安全获取接口 (Getter)
// ---------------------------------------------------------

// 【通道A】供 alu_temp.c 里的 PID_PWM_iteration 调用
float TempFilter_GetLatestAvgTemp(void) {
    return latest_avg_temp;
}

// 【通道B】供 TouchGFX Model::tick() 调用
float TempFilter_GetUIAvgTemp(void) {
    return ui_display_temp;
}
