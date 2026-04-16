#include "task_control.h"
#include "temp_filter.h"
#include "pid.h"
#include "fuzzy_pid.h"
#include "advanced_pid.h"
#include "thermocouple.h"
#include "alu_control.h"
#include "alu_file.h"
#include "alu_temp.h"
#include "alu_main.h"
#include "dac.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"

extern UART_HandleTypeDef huart4;
extern TIM_HandleTypeDef htim1;

extern QueueHandle_t SDWriteQueueHandle;
extern osSemaphoreId Sem_10msHandle;
extern osSemaphoreId alu_temperatureHandle;
extern osSemaphoreId alu_screenHandle;

extern osThreadId defaultTaskHandle;
extern osThreadId aluMainHandle;
extern osThreadId aluSubProgressHandle;
extern osThreadId Task_ControlHandle;

extern double K_Temperature;
extern float temp_modify;
extern float temp_thres;
extern float pwm_percent;

extern volatile uint8_t is_heating_active;
extern volatile uint32_t heating_num_count;
extern uint8_t sd_record_enable;
extern PID_struct pid_TEMP;
extern uint8_t uart_pid_state;

extern uint8_t need_stop_cleanup;

extern FuzzyPID_struct fuzzy_pid_TEMP;
extern AdvPID_struct adv_pid_TEMP;

static uint8_t last_heating_active = 0;

void StartTask_Control(void const * argument)
{
    TempFilter_Init();
    FuzzyPID_init(&fuzzy_pid_TEMP);
    AdvPID_Init(&adv_pid_TEMP, 40.0f, 0.8f, 125.0f);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

    uint32_t tick_10ms = 0;
    uint32_t heating_startup_counter = 0;
    uint8_t alu_485_off[] = {0x55, 0x33, 0x01, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x0D};

    for(;;)
    {
        osSemaphoreWait(Sem_10msHandle, osWaitForever);
        tick_10ms++;

        TempFilter_Process();
        K_Temperature = TempFilter_GetUIAvgTemp();

        if (last_heating_active == 0 && is_heating_active == 1) {
            heating_startup_counter = 0;
        }
        last_heating_active = is_heating_active;

        if (is_heating_active == 1) {
            heating_startup_counter++;
        }

        if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 0 && is_heating_active == 1) {
            if (heating_startup_counter > 3) {
                is_heating_active = 0;

                HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_SET);
                HAL_UART_Transmit(&huart4, alu_485_off, 10, 100);
                HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_RESET);

                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1);
                DAC_SetLaserCurrent(0.0f);
                need_stop_cleanup = 1;
            }
        }

        if (is_heating_active == 1) {
            K_Temperature = K_Temperature + temp_modify;
            if (K_Temperature >= 150) K_Temperature = 150;
            else if (K_Temperature <= 0) K_Temperature = 0;

            float pwm_value = 0.0f;

            if (laser_test_state == 3) {
                extern float target_laser_current;
                extern float target_laser_pwm;
                HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_SET);
                uint8_t alu_485_laser_on[] = {0x55, 0x33, 0x01, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x0D};
                HAL_UART_Transmit(&huart4, alu_485_laser_on, 10, 100);
                HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_RESET);
                DAC_SetLaserCurrent(target_laser_current);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)target_laser_pwm);
                pwm_value = target_laser_pwm;
            } else {
                if (pid_algorithm_type == 0) {
                    pwm_value = PID_PWM_iteration(&pid_TEMP, temp_thres, K_Temperature);
                } else if (pid_algorithm_type == 1) {
                    pwm_value = FuzzyPID_Calculate(&fuzzy_pid_TEMP, temp_thres, K_Temperature);
                } else {
                    pwm_value = AdvPID_Calculate(&adv_pid_TEMP, temp_thres, (float)K_Temperature);
                }
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)pwm_value);
            }

            pwm_percent = pwm_value / 1000.0f;
        } else {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1);
        }

        if (tick_10ms % 20 == 0) {
            Thermocouple_UpdateColdJunction();

            if (is_heating_active == 1) {
                char BufferWrite[64] = {0};
                sprintf(BufferWrite, "\n%d,%.2f,%.3f,%.3f,%.3f",
                    heating_num_count, K_Temperature,
                    pid_TEMP.speed[0], pid_TEMP.speed[1], pid_TEMP.speed[2]);

                if (sd_record_enable && SDWriteQueueHandle != NULL) {
                    xQueueSend(SDWriteQueueHandle, BufferWrite, 0);
                }
                heating_num_count++;
            } else {
                if (is_serial_interacting == 0 && uart_pid_state == 0) {
                    printf("%.2f(off)\r\n", K_Temperature);
                }
            }
        }

        if (tick_10ms % 100 == 0) {
            osSemaphoreRelease(alu_temperatureHandle);
            printf("[STACK] aluMain:%u, Control:%u, SubProg:%u\r\n",
                   (unsigned int)uxTaskGetStackHighWaterMark(aluMainHandle),
                   (unsigned int)uxTaskGetStackHighWaterMark(Task_ControlHandle),
                   (unsigned int)uxTaskGetStackHighWaterMark(aluSubProgressHandle));
        }
    }
}
