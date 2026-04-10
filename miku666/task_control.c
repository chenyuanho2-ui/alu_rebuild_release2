#include "task_control.h"
#include "temp_filter.h"
#include "alu_temp.h"
#include "alu_control.h"
#include "alu_file.h"
#include <stdio.h>

// 声明外部的 20ms 信号量
extern osSemaphoreId Sem_20msHandle;

// ==========================================================
// 核心控制任务 (覆盖 freertos.c 中的 __weak 空函数)
// ==========================================================
void StartTask_Control(void const * argument)
{
  // 1. 任务开始前，初始化测温模块
  TempFilter_Init();
  
  // 2. 这是一个专门用于 250ms 降频处理的时间戳
  uint32_t last_pid_tick = HAL_GetTick();
  
  /* Infinite loop */
  for(;;)
  {
    // ==========================================================
    // 1. 【休眠死等】：平时彻底让出 CPU，直到 TIM7 发来 20ms 信号量
    // ==========================================================
    osSemaphoreWait(Sem_20msHandle, osWaitForever);
    
    // ==========================================================
    // 2. 【高频通道】：信号量一到，立刻醒来执行 20ms 测温和数字滤波
    // ==========================================================
    TempFilter_Process();
    
    // ==========================================================
    // 3. 【低频通道】：如果处于加热状态，且距离上次计算满了 250ms
    // ==========================================================
    if (is_heating_active == 1 && (HAL_GetTick() - last_pid_tick >= 250))
    {
        extern double K_Temperature;
        extern float temp_modify;
        extern float temp_thres;
        extern float pwm_percent;
        
        // --- A. 获取并处理温度 ---
        K_Temperature = alu_SPI_gettemp(); 
        if (K_Temperature >= 150) K_Temperature = 150;
        else if (K_Temperature <= 0) K_Temperature = 0;
        K_Temperature = K_Temperature + temp_modify;  
        
        // --- B. 计算 PID 并且给 TouchGFX 发信号量 ---
        pwm_percent = PID_PWM_iteration(&pid_TEMP, temp_thres, K_Temperature) / 1000; 
        osSemaphoreRelease(alu_temperatureHandle);  
        
        // --- C. 格式化数据并写入 SD 卡 ---
        char BufferWrite[50] = " "; 
        sprintf(BufferWrite, "\n%d,%.2f,%.3f,%.3f,%.3f", heating_num_count, K_Temperature, pid_TEMP.speed[0], pid_TEMP.speed[1], pid_TEMP.speed[2]);
        Alu_SD_write((uint8_t*)BufferWrite, sizeof(BufferWrite), (const char *)file_name_cache);
        
        // --- D. 检查脚踏是否松开 (每隔 5 次即 1.25秒 检查一次) ---
        if (heating_num_count % 5 == 0 && heating_num_count > 10) 
        {     
            if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 1) {  
                osDelay(20); // 任务内安全消抖           
                if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 1) {
                    // 脚踏确实松开了，调用清理函数
                    Heating_Stop_Routine();
                }
            }
        }
        
        // --- E. 计数器累加，更新时间戳 ---
        heating_num_count++;
        last_pid_tick = HAL_GetTick();
    }
  }
}
