#ifndef _ALU_TEMP_H_ 
#define _ALU_TEMP_H_ 

#include "main.h"

extern SPI_HandleTypeDef hspi2;    // 热电偶SPI通信

double alu_SPI_gettemp(void);

#endif // _ALU_TEMP_H_
