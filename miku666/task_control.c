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

extern osThreadId defaultTaskHandle;
extern osThreadId aluMainHandle;
extern osThreadId aluSubProgressHandle;
extern osThreadId Task_ControlHandle;

extern double K_Temperature;
extern float temp_modify;
extern float temp_thres;
extern float pwm_percent;

// 引入状态
extern volatile uint8_t is_heating_active;
extern volatile uint32_t heating_num_count;
extern uint8_t sd_record_enable;  // SD 记录开关
extern PID_struct pid_TEMP;
extern uint8_t uart_pid_state;
extern uint8_t pid_algorithm_type;

void StartTask_Control(void const * argument)
{
  TempFilter_Init();
  uint32_t tick_10ms = 0;

  for(;;)
  {
    osSemaphoreWait(Sem_10msHandle, osWaitForever);
    tick_10ms++;

    // 高频读取并滤波
    TempFilter_Process();
    K_Temperature = TempFilter_GetUIAvgTemp();

    // ========== [如果处于加热状态，执行控制] ==========
    if (is_heating_active == 1) {
        K_Temperature = K_Temperature + temp_modify;
        if (K_Temperature >= 150) K_Temperature = 150;
        else if (K_Temperature <= 0) K_Temperature = 0;

        if (pid_algorithm_type == 0) {
            pwm_percent = PID_PWM_iteration(&pid_TEMP, temp_thres, K_Temperature) / 1000;
        } else {
            pwm_percent = Fuzzy_PID_PWM_iteration(&pid_TEMP, temp_thres, K_Temperature) / 1000;
        }
    }

    // ========== [200ms 低频区处理队列和按键] ==========
    if (tick_10ms % 20 == 0) {
        Thermocouple_UpdateColdJunction();

        if (is_heating_active == 1) {
            // 发送到写入队列
            char BufferWrite[64] = {0};
            sprintf(BufferWrite, "\n%d,%.2f,%.3f,%.3f,%.3f",
                heating_num_count, K_Temperature,
                pid_TEMP.speed[0], pid_TEMP.speed[1], pid_TEMP.speed[2]);

            if (sd_record_enable && SDWriteQueueHandle != NULL) {
                xQueueSend(SDWriteQueueHandle, BufferWrite, 0);
            }

            // 【关键】：检测到松开脚踏，瞬间切断！
            if (heating_num_count > 2) 
            {
                if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 0) {
                    is_heating_active = 0; // 停止PID

                    // 瞬间关停 485 硬件
                    HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_SET);
                    uint8_t alu_485_off[] = {0x55, 0x33, 0x01, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x0D};		
					
					extern UART_HandleTypeDef huart4;
                    HAL_UART_Transmit(&huart4, alu_485_off, 10, 100);
                    HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_RESET);

                    // 呼叫主任务进行收尾清理
                    extern uint8_t need_stop_cleanup;
                    need_stop_cleanup = 1;
                }
            }
            heating_num_count++;
        } else if (uart_pid_state == 0) {
           printf("%.2f(off)\r\n", K_Temperature);  // 200ms 打印温度（待机）
        }
    }

    // 1000ms UI刷新
    if (tick_10ms % 100 == 0) {
        osSemaphoreRelease(alu_temperatureHandle);
    }
  }
}
