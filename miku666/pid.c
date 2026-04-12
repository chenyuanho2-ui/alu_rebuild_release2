#include "pid.h"

extern TIM_HandleTypeDef htim1;

void PID_init(PID_struct* pid_info) {
	pid_info->v_target = 0.0;
	pid_info->v_current = 0.0;
	pid_info->err = 0.0;
	pid_info->err_prev_1 = 0.0;
	pid_info->err_prev_2 = 0.0;
	pid_info->speed[0] = pid_info->speed[1] = pid_info->speed[2] = 0.0;
	// PID 参数 (适用于 10ms 控制周期，dt = 0.01s)
	// 旧 20ms 参数参考: Kp=40, Ki=1.6(=0.08*20), Kd=62.5(=5/0.08)
	pid_info->Kp = 40;                                                    // 比例系数保持不变
	pid_info->Ki = 0.8;  // 积分系数: 0.08 * 10ms = 0.8 (旧 1.6 减半，防止积分过冲)
	pid_info->Kd = 125.0; // 微分系数: 5 / 0.01 = 125.0 (旧 62.5 加倍，补偿更快采样)
}


float PID_PWM_iteration(PID_struct* pid_info, float value_thres, float value_current) {
	pid_info->v_target = value_thres+5; // 设定目标值  // 考虑到散热,额外加5度
	pid_info->v_current = value_current; // 设定当前值  // pid_T.v_current = (float)MAX6675_ReadTemperature();  // 设定当前值
	// 误差值
	pid_info->err = pid_info->v_target - pid_info->v_current;
	// 比例输出
	float speed_p = pid_info->Kp * pid_info->err;
	// 误差积分值
	pid_info->err_prev_2 = pid_info->err_prev_2 + pid_info->err;
	// 积分输出
//	float speed_i = pid_info->Ki * pid_info->err;这是算成了Ki*e？
	float speed_i = pid_info->Ki * pid_info->err_prev_2;//ki*积分e
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
