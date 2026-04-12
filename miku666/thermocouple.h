#ifndef __THERMOCOUPLE_H
#define __THERMOCOUPLE_H

#include "main.h"

float Temp_To_Voltage_T_Type(float temp);

float Voltage_To_Temp_T_Type(float voltage);

void Thermocouple_UpdateColdJunction(void);

float Get_Actual_Temperature(void);

#endif /* __THERMOCOUPLE_H */
