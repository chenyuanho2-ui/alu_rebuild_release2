#include "alu_485.h"

/*
LDD电流设置指令（16进制）：
55 33 01 02 00 00 YY XX 00 0D  （YY是电流值，对应16进制；XX是校验位，是从01开始一直到YY的这5个bytes按位异或的结果）

*/
//HAL_UART_Transmit(&huart1,(uint8_t*)"aluminium\r\n",11,0xFFFF);  // 上下左右ABAB不要使用printf




