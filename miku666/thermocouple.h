#ifndef __THERMOCOUPLE_H
#define __THERMOCOUPLE_H

#include "main.h"

// 将冷端温度（℃）转换为对应的热电偶电压（mV）
float Temp_To_Voltage_T_Type(float temp);

// 将总电压（mV）转换为实际温度（℃）
float Voltage_To_Temp_T_Type(float voltage);

// 结合ADS1118获取最终实际温度的综合函数
float Get_Actual_Temperature(void);

#endif /* __THERMOCOUPLE_H */
