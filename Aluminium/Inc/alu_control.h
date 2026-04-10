#ifndef _ALU_CONTROL_H_ 
#define _ALU_CONTROL_H_ 

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "alu_temp.h" // 确保能认识 PID_struct

extern osSemaphoreId alu_chooseHandle;         // 选择是温度还是功率
extern osSemaphoreId alu_temperatureHandle;    // 温度阈值设置信号量
extern osSemaphoreId alu_thresholdHandle;      // 功率阈值设置信号量
extern osSemaphoreId alu_screenHandle;         // 屏幕切换信号量

// ==========================================
// 【新增】：全局状态变量声明 (供其他任务调用)
// ==========================================
extern uint8_t is_heating_active;    // 加热状态标志位：1 表示正在加热，0 表示停止
extern int heating_num_count;        // PID 与 SD 卡记录的计数器
extern char file_name_cache[14];     // 动态生成的 CSV 文件名缓存
extern PID_struct pid_TEMP;          // 全局 PID 结构体

long btn_sniff_pressed(void);
int  active_key_1(int index_scr);
int  active_key_2(int index_choose,float *temp_thres,float *power_thres);
int  active_key_3(int index_choose,float *temp_thres,float *power_thres);
int  active_key_4(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t *data485, float power_thres);
int  active_key_1vs4(int* temp_modify);

int  active_key_foot(uint8_t *data_485, float temp_thres,float power_thres);

// 【新增】：停止加热清理函数的声明
void Heating_Stop_Routine(void);

#endif // _ALU_CONTROL_H_
