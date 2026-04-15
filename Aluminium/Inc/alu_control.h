#ifndef _ALU_CONTROL_H_ 
#define _ALU_CONTROL_H_ 

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "pid.h" // 包含PID结构定义

extern osSemaphoreId alu_chooseHandle;         // 选择是温度还是功率
extern osSemaphoreId alu_temperatureHandle;    // 温度阈值设置信号量
extern osSemaphoreId alu_thresholdHandle;      // 功率阈值设置信号量
extern osSemaphoreId alu_screenHandle;         // 屏幕切换信号量

// ==========================================
// 【同步更新】：全局状态变量声明 (供其他任务调用)
// 类型必须与 alu_main.c 中的定义完全一致！
// ==========================================
extern volatile uint8_t is_heating_active;     // 加热状态标志位：1 表示正在加热，0 表示停止
extern volatile uint32_t heating_num_count;    // PID 与 SD 卡记录的计数器
extern char current_file_name[32];             // 动态生成的 CSV 文件名缓存
extern PID_struct pid_TEMP;                    // 全局 PID 结构体

// ==========================================
// 串口PID交互新增全局变量
// ==========================================
extern uint8_t pid_algorithm_type;              // PID算法选择: 0=普通位置型PID, 1=模糊PID
extern uint8_t uart_pid_state;                 // 串口PID交互状态机: 0=正常模式, 1=等待输入, 2=等待确认
extern float temp_Kp, temp_Ki, temp_Kd;        // 临时缓存解析出的PID参数
extern QueueHandle_t UartRxQueue;              // 串口接收队列句柄

long btn_sniff_pressed(void);
int  active_key_1(int index_scr);
int  active_key_2(int index_choose,float *temp_thres,float *power_thres);
int  active_key_3(int index_choose,float *temp_thres,float *power_thres);
int  active_key_4(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t *data485, float power_thres);
int  active_key_1vs4(int* temp_modify);

// 【同步更新】：将旧的脚踏阻塞函数改为新的瞬间启动函数
int  active_key_foot_start(uint8_t *data_485, float temp_thres, float power_thres);

// 停止加热清理函数的声明
void Heating_Stop_Routine(void);

#endif // _ALU_CONTROL_H_
