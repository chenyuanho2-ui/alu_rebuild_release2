#ifndef _ALU_TEMP_H_ 
#define _ALU_TEMP_H_ 

#include "main.h"

extern SPI_HandleTypeDef hspi2;    // 热电偶SPI通信

double alu_SPI_gettemp(void);

extern TIM_HandleTypeDef htim1;    // PWM调制

typedef struct {   // 自定义动态文件列表
	float v_target;	   //定义设定值
	float v_current;   //定义实际值
    float speed[3];    //分别对应输出pid的速度
	float err;		   //定义当前偏差值  k
	float err_prev_1;	   //定义上次偏差    k-1  用于求微分值吧
	float err_prev_2;	   //定义上上次偏差  k-2  用于求积分值吧
	float Kp, Ki, Kd;  //定义比例、积分、微分
} PID_struct;    


void PID_init(PID_struct* pid_info);

float PID_PWM_iteration(PID_struct* pid_info, float value_thres, float value_current);





#endif // _ALU_TEMP_H_ 
