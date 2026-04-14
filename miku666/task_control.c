#include "task_control.h"
#include "temp_filter.h"
#include "pid.h"
#include "thermocouple.h"
#include "alu_control.h"
#include "alu_file.h"
#include "alu_temp.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t SDWriteQueueHandle;

extern osSemaphoreId Sem_10msHandle;

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

  // 10ms 周期计数器
  uint32_t tick_10ms = 0;

  // ========== [耗时监控变量] ==========
  uint32_t start_tick = 0;
  uint32_t current_cost = 0;
  uint32_t max_cost_in_200ms = 0;

  for(;;)
  {
    // 等待 10ms 定时器信号量 (TIM7)
    osSemaphoreWait(Sem_10msHandle, osWaitForever);

    // 记录本轮起始时间
    start_tick = xTaskGetTickCount();

    tick_10ms++;

    // 10ms 高频区: 读取热电偶温度并进行滤波
    TempFilter_Process();

    // ========== [10ms 高频区 - 核心 PID 控制] ==========
    if (is_heating_active == 1) {
        // 获取滤波后的温度
        K_Temperature = TempFilter_GetUIAvgTemp();
        K_Temperature = K_Temperature + temp_modify;
        if (K_Temperature >= 150) K_Temperature = 150;
        else if (K_Temperature <= 0) K_Temperature = 0;

        // 执行 PID 迭代计算 (dt = 0.01s)
        pwm_percent = PID_PWM_iteration(&pid_TEMP, temp_thres, K_Temperature) / 1000;
    }

    // 计算本轮耗时
    current_cost = xTaskGetTickCount() - start_tick;

    // 擂台法记录 200ms 内的最大耗时
    if (current_cost > max_cost_in_200ms) {
        max_cost_in_200ms = current_cost;
    }

    // 极简高频打印（仅打印温度数字，减少串口阻塞时长）
    printf("%.2f\r\n", K_Temperature);

    // ========== [200ms 低频区] (每 20 个 10ms 节拍) ==========
    if (tick_10ms % 20 == 0) {
        // 更新冷端补偿电压 (环境温度变化慢，200ms 刷新一次足够)
        Thermocouple_UpdateColdJunction();

        if (is_heating_active == 1) {
            // 构造 SD 卡记录数据
            char BufferWrite[64] = {0};
            sprintf(BufferWrite, "\n%d,%.2f,%.3f,%.3f,%.3f",
                heating_num_count,
                K_Temperature,
                pid_TEMP.speed[0],
                pid_TEMP.speed[1],
                pid_TEMP.speed[2]);

            // 发送到 SD 卡写队列 (非阻塞)
            if (SDWriteQueueHandle != NULL) {
                xQueueSend(SDWriteQueueHandle, BufferWrite, 0);
            }

            // 脚踏开关停止加热判定 (每 5 次即 1 秒检测一次)
            if (heating_num_count % 5 == 0 && heating_num_count > 10)
            {
                if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 1) {
                    Heating_Stop_Routine();
                }
            }

            heating_num_count++;
        }

        // 打印系统运行状态诊断信息
//        printf("[SYS %u] MaxCost:%ums\r\n", xTaskGetTickCount(), max_cost_in_200ms);

//        // 打印当前 PWM 输出比例
//        printf("[PWM] Output: %.1f %%\r\n", pwm_percent * 100.0f);

//        // 打印当前设定温度阈值
//        printf("[SET] Target: %.1f C\r\n", temp_thres);

        // 重置最大耗时，为下一个 200ms 周期做准备
        max_cost_in_200ms = 0;
    }

    // ========== [1000ms UI 刷新专区] (每 100 个 10ms 节拍) ==========
    // 降低释放频率，让图表横轴的时间与物理时间完美对齐
    if (tick_10ms % 100 == 0) {
        // 释放 UI 刷新信号量 (每秒 1 次)
        osSemaphoreRelease(alu_temperatureHandle);//alu_temperatureHandle每200ms释放一次
    }

    // ========== [2000ms 超低频区] (每 200 个 10ms 节拍) ==========
    if (tick_10ms % 200 == 0) {
        // 获取各任务栈历史最低剩余容量
        uint32_t stack_ctrl = uxTaskGetStackHighWaterMark((TaskHandle_t)Task_ControlHandle);
        uint32_t stack_main = uxTaskGetStackHighWaterMark((TaskHandle_t)aluMainHandle);
        uint32_t stack_gui  = uxTaskGetStackHighWaterMark((TaskHandle_t)defaultTaskHandle);
        uint32_t stack_sub  = uxTaskGetStackHighWaterMark((TaskHandle_t)aluSubProgressHandle);

//        printf("\r\n=== [SYS RAM Monitor] ===\r\n");
//        printf("Task_Control Free : %u Words\r\n", stack_ctrl);
//        printf("AluMain Free      : %u Words\r\n", stack_main);
//        printf("TouchGFX Free     : %u Words\r\n", stack_gui);
//        printf("aluSubProgress Free     : %u Words\r\n", stack_sub);
//        printf("=========================\r\n");
    }
  }
}
