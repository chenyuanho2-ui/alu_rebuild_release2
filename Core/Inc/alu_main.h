#ifndef __ALU_MAIN_H
#define __ALU_MAIN_H

#include "cmsis_os.h"
#include "main.h"

// 声明主 UI/按键任务的强函数
void AluMain(void const * argument);

// ==========================================
// 模块1: 全局状态变量 (在 alu_main.c 中定义)
// ==========================================
extern uint8_t enable_pid_tune;
extern uint8_t enable_laser_test;
extern uint8_t pid_algorithm_type;
extern uint8_t is_serial_interacting;
extern uint8_t enable_stack_print;
extern uint8_t laser_test_state;
extern float target_laser_current;
extern float target_laser_pwm;
extern float g_display_temp_avg;
extern uint8_t enable_print_timing;

#endif /* __ALU_MAIN_H */
