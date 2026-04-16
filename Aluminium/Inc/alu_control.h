#ifndef _ALU_CONTROL_H_
#define _ALU_CONTROL_H_

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "pid.h"
#include "fuzzy_pid.h"
#include "advanced_pid.h"

extern osSemaphoreId alu_chooseHandle;
extern osSemaphoreId alu_temperatureHandle;
extern osSemaphoreId alu_thresholdHandle;
extern osSemaphoreId alu_screenHandle;

extern volatile uint8_t is_heating_active;
extern volatile uint32_t heating_num_count;
extern char current_file_name[32];
extern PID_struct pid_TEMP;

extern uint8_t pid_algorithm_type;
extern uint8_t uart_pid_state;
extern float temp_Kp, temp_Ki, temp_Kd;
extern QueueHandle_t UartRxQueue;

extern FuzzyPID_struct fuzzy_pid_TEMP;
extern AdvPID_struct adv_pid_TEMP;

extern uint8_t enable_pid_tune;
extern uint8_t enable_laser_test;
extern uint8_t is_serial_interacting;
extern uint8_t laser_test_state;
extern float target_laser_current;
extern float target_laser_pwm;

long btn_sniff_pressed(void);
int  active_key_1(int index_scr);
int  active_key_2(int index_choose,float *temp_thres,float *power_thres);
int  active_key_3(int index_choose,float *temp_thres,float *power_thres);
int  active_key_4(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t *data485, float power_thres);
int  active_key_1vs4(int* temp_modify);
int  active_key_foot_start(uint8_t *data_485, float temp_thres, float power_thres);
void Heating_Stop_Routine(void);

#endif
