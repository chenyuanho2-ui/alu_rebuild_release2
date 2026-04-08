#ifndef _ALU_CONTROL_H_ 
#define _ALU_CONTROL_H_ 

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"


extern osSemaphoreId alu_chooseHandle;         // 选择是温度还是功率
extern osSemaphoreId alu_temperatureHandle;    // 温度阈值设置信号量
extern osSemaphoreId alu_thresholdHandle;      // 功率阈值设置信号量
extern osSemaphoreId alu_screenHandle;         // 屏幕切换信号量


long btn_sniff_pressed(void);
int  active_key_1(int index_scr);
int  active_key_2(int index_choose,float *temp_thres,float *power_thres);
int  active_key_3(int index_choose,float *temp_thres,float *power_thres);
int  active_key_4(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t *data485, float power_thres);
int  active_key_1vs4(int* temp_modify);

int  active_key_foot(uint8_t *data_485, float temp_thres,float power_thres);
#endif // _ALU_CONTROL_H_ 
