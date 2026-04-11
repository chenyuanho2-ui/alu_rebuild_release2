#include "alu_temp.h"
#include "temp_filter.h"  // 新增：引入我们的双通道测温接口


// 劫持原有的测温接口
double alu_SPI_gettemp(void)
{
    // 直接获取【通道A】的 20ms 实时平滑数据，转为 double 给 PID 用
    // 这样原来的控制任务只要调用这个函数，拿到的就是 ADS1118 的最新数据
    return (double)TempFilter_GetLatestAvgTemp();
}

void PID_init(PID_struct* pid_info) {
	pid_info->v_target = 0.0;
	pid_info->v_current = 0.0;
	pid_info->err = 0.0;
	pid_info->err_prev_1 = 0.0;
	pid_info->err_prev_2 = 0.0;
	pid_info->speed[0] = pid_info->speed[1] = pid_info->speed[2] = 0.0;
	// 0.5 0.04 0.25
	pid_info->Kp = 40; // 比例参数保持不变
	pid_info->Ki = 1.6; // 积分参数等效换算：20 * 0.08 = 1.6
	pid_info->Kd = 62.5;  // 微分参数等效换算：5 / 0.08 = 62.5
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
