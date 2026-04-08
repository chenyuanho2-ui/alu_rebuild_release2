#include "alu_temp.h"


double alu_SPI_gettemp(void)
{
		double temp_couple;        // 温度值
		uint16_t tmp;              // 临时值
// 开启片选
	  HAL_GPIO_WritePin(SPI2_CS_T_GPIO_Port, SPI2_CS_T_Pin, GPIO_PIN_RESET);
//  第1次读取数据(高8位)
		unsigned char txdata,rxdata;
		txdata = 0XFF;
		HAL_SPI_TransmitReceive(&hspi2,&txdata,&rxdata,1,1000);
		tmp = rxdata;
		tmp <<= 8;
// 第2次读取数据(低8位)
		HAL_SPI_TransmitReceive(&hspi2,&txdata,&rxdata,1,1000);		
		tmp |= rxdata;
// 关闭片选
		HAL_GPIO_WritePin(SPI2_CS_T_GPIO_Port, SPI2_CS_T_Pin, GPIO_PIN_SET);
		if (tmp & 4) {
			tmp = 4095; //未检测到热电偶
		} else {
			tmp = tmp >> 3;
		}
		temp_couple = tmp * 1024.0 / 4096 - 23.75;
		
		if (temp_couple >= 100)
			temp_couple = 100;
		else if (temp_couple <= 0)
			temp_couple = 0;
		
		return temp_couple;
}

void PID_init(PID_struct* pid_info) {
	pid_info->v_target = 0.0;
	pid_info->v_current = 0.0;
	pid_info->err = 0.0;
	pid_info->err_prev_1 = 0.0;
	pid_info->err_prev_2 = 0.0;
	pid_info->speed[0] = pid_info->speed[1] = pid_info->speed[2] = 0.0;
	// 0.5 0.04 0.25
	pid_info->Kp = 40; // 0.32 30                 T=500 Ti=5000000 Td=1000
	pid_info->Ki = 20; // 0.16 30 * 250 / 600000  kp*计算周期/总积分时间
	pid_info->Kd = 5;  // 0.04 30 * 1000 / 250    kp*微分时间/计算周期
}


float PID_PWM_iteration(PID_struct* pid_info, float value_thres, float value_current) {
	pid_info->v_target = value_thres+5;	 // 设定目标值  // 考虑到散热,额外加5度
	pid_info->v_current = value_current; // 设定当前值  // pid_T.v_current = (float)MAX6675_ReadTemperature();  // 设定当前值
	// 误差值
	pid_info->err = pid_info->v_target - pid_info->v_current;
	// 比例输出
	float speed_p = pid_info->Kp * pid_info->err;
	// 误差积分值
	pid_info->err_prev_2 = pid_info->err_prev_2 + pid_info->err;
	// 积分输出
	float speed_i = pid_info->Ki * pid_info->err;
	// 微分输出
	float speed_d = pid_info->Kd * (pid_info->err - pid_info->err_prev_1);
	float speed = speed_p + speed_i + speed_d;
	// 更新当前值(理论上pid求和后的大小和PWM的值建立映射,但是参数得调)
	pid_info->speed[0] = speed_p;
	pid_info->speed[1] = speed_i;
	pid_info->speed[2] = speed_d;
	// 更新偏差
	pid_info->err_prev_1 = pid_info->err;
	// PWM调节,假定调整后kp,ki,kd几个的参数能直接映射到PWM0~1000
	if (speed > 1000) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
		return 1000;
	}
	else if (speed > 0) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
		return speed;
	}
	else {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1);
		return 1;
	}
}
