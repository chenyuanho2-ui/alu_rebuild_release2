#include "task_control.h"
#include "temp_filter.h"
#include "pid.h"
#include "alu_control.h"
#include "alu_file.h"
#include "alu_temp.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t SDWriteQueueHandle;

extern osSemaphoreId Sem_20msHandle;

// ==========================================================
// 【引入任务句柄】根据你 CubeMX 里的名字，严格对应！
// ==========================================================
extern osThreadId defaultTaskHandle;
extern osThreadId aluMainHandle;
extern osThreadId aluSubProgressHandle;
extern osThreadId Task_ControlHandle;

extern double K_Temperature;
extern float temp_modify;
extern float temp_thres;
extern float pwm_percent;

void StartTask_Control(void const * argument)
{
  TempFilter_Init();
  
  // 定义软计数器
  uint16_t tick_sd_card = 0;  // 用于 200ms 任务
  uint16_t tick_health = 0;   // 用于 2000ms 任务
  
  /* Infinite loop */
  for(;;)
  {
    // 1. 死等定时器 20ms 信号量
    osSemaphoreWait(Sem_20msHandle, osWaitForever);
    
    // 2. 20ms 高频测温与滤波
    TempFilter_Process();
    
    // ==========================================================
    // 【20ms 高频核心区】
    // ==========================================================
    if (is_heating_active == 1) {
        // 严禁使用 alu_SPI_gettemp()，必须使用滤波后的数据
        K_Temperature = TempFilter_GetUIAvgTemp();
		K_Temperature = K_Temperature + temp_modify; 
        if (K_Temperature >= 150) K_Temperature = 150;
        else if (K_Temperature <= 0) K_Temperature = 0;
         
        
        pwm_percent = PID_PWM_iteration(&pid_TEMP, temp_thres, K_Temperature) / 1000;
    }
    
    // ==========================================================
    // 【200ms 低频控制区】(10个节拍)
    // ==========================================================
    tick_sd_card++;
    if (tick_sd_card >= 10) {
        tick_sd_card = 0;
        
        // UI 刷新：200ms 刷新一次屏幕，达到 5Hz 丝滑刷新率
        osSemaphoreRelease(alu_temperatureHandle);  
        
        // SD 卡记录
            if (is_heating_active == 1) {
                char BufferWrite[64] = {0}; // 扩大至 64 字节防止溢出
                sprintf(BufferWrite, "\n%d,%.2f,%.3f,%.3f,%.3f", 
                    heating_num_count, 
                    K_Temperature, 
                    pid_TEMP.speed[0], 
                    pid_TEMP.speed[1], 
                    pid_TEMP.speed[2]);
                
                // 【修改点】：不再调用 Alu_SD_write！改为发送到 FreeRTOS 队列
                // 等待时间设为 0（非阻塞），即使队列满了也立刻返回，绝不卡顿 PID 任务
                if (SDWriteQueueHandle != NULL) {
                    xQueueSend(SDWriteQueueHandle, BufferWrite, 0);
                }
            
            // 按脚踏开关停止加热的逻辑判定（5次即为 1秒）
            if (heating_num_count % 5 == 0 && heating_num_count > 10) 
            {
                if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 1) {  
                    // 此时已经是 200ms 后，无需挂起 20ms，直接判定松开
                    Heating_Stop_Routine();
                }
            }
            
            heating_num_count++;
        }
    }
    
    // ==========================================================
    // 【2000ms 系统健康监控区】(100个节拍)
    // ==========================================================
    tick_health++;
    if (tick_health >= 100) {
        tick_health = 0;
        
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
    }
  }
}
