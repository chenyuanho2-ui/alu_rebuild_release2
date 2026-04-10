#include "task_control.h"
#include "temp_filter.h"
#include "alu_temp.h"
#include "alu_control.h"
#include "alu_file.h"
#include <stdio.h>

extern osSemaphoreId Sem_20msHandle;

// ==========================================================
// 【引入任务句柄】根据你 CubeMX 里的名字，严格对应！
// ==========================================================
extern osThreadId defaultTaskHandle;
extern osThreadId aluMainHandle;
extern osThreadId aluSubProgressHandle;
extern osThreadId Task_ControlHandle;

void StartTask_Control(void const * argument)
{
  TempFilter_Init();
  
  uint32_t last_pid_tick = HAL_GetTick();
  uint32_t last_stack_tick = HAL_GetTick(); // 新增：2秒监控一次的时间戳
  
  /* Infinite loop */
  for(;;)
  {
    // 1. 死等定时器 20ms 信号量
    osSemaphoreWait(Sem_20msHandle, osWaitForever);
    
    // 2. 20ms 高频测温与滤波
    TempFilter_Process();
    
    // ==========================================================
    // 3. 【系统健康状态监控通道】(每 2000ms 执行一次)
    // ==========================================================
    if (HAL_GetTick() - last_stack_tick >= 2000)
    {
        // 强转为 TaskHandle_t 获取栈历史最低剩余容量
        uint32_t stack_ctrl = uxTaskGetStackHighWaterMark((TaskHandle_t)Task_ControlHandle);
        uint32_t stack_main = uxTaskGetStackHighWaterMark((TaskHandle_t)aluMainHandle);
        uint32_t stack_gui  = uxTaskGetStackHighWaterMark((TaskHandle_t)defaultTaskHandle);
		uint32_t stack_sub  = uxTaskGetStackHighWaterMark((TaskHandle_t)aluSubProgressHandle);
        
        printf("\r\n=== [SYS RAM Monitor] ===\r\n");
        printf("Task_Control Free : %u Words\r\n", stack_ctrl);
        printf("AluMain Free      : %u Words\r\n", stack_main);
        printf("TouchGFX Free     : %u Words\r\n", stack_gui);
		printf("aluSubProgress Free     : %u Words\r\n", stack_sub);
        printf("=========================\r\n");
        
        last_stack_tick = HAL_GetTick();
    }
    
    // ==========================================================
    // 4. 【低频控制通道】：250ms PID 与 SD 卡记录
    // ==========================================================
    if (is_heating_active == 1 && (HAL_GetTick() - last_pid_tick >= 250))
    {
        extern double K_Temperature;
        extern float temp_modify;
        extern float temp_thres;
        extern float pwm_percent;
        
        K_Temperature = alu_SPI_gettemp(); 
        if (K_Temperature >= 150) K_Temperature = 150;
        else if (K_Temperature <= 0) K_Temperature = 0;
        K_Temperature = K_Temperature + temp_modify;  
        
        pwm_percent = PID_PWM_iteration(&pid_TEMP, temp_thres, K_Temperature) / 1000; 
        
        // 屏幕降频适配 (500ms 刷新一次 UI)
        if (heating_num_count % 2 == 0) {
            osSemaphoreRelease(alu_temperatureHandle);  
        }
        
        char BufferWrite[50] = " "; 
        sprintf(BufferWrite, "\n%d,%.2f,%.3f,%.3f,%.3f", heating_num_count, K_Temperature, pid_TEMP.speed[0], pid_TEMP.speed[1], pid_TEMP.speed[2]);
        Alu_SD_write((uint8_t*)BufferWrite, sizeof(BufferWrite), (const char *)file_name_cache);
        
        if (heating_num_count % 5 == 0 && heating_num_count > 10) 
        {     
            if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 1) {  
                // 此时已经是 250ms 后，无需挂起 20ms，直接判定松开
                Heating_Stop_Routine();
            }
        }
        
        heating_num_count++;
        last_pid_tick = HAL_GetTick();
    }
  }
}
