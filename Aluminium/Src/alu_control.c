#include "alu_control.h"
#include "alu_file.h"
#include "pid.h"
#include "temp_filter.h"

// ==========================================
// 按键状态变量
// ==========================================
uint8_t key1_pressed;
uint8_t key2_pressed;
uint8_t key3_pressed;
uint8_t key4_pressed;
uint8_t key_foot_pressed;

// ==========================================
// 1至4号按键扫描与参数修改函数 (保持原样)
// ==========================================
long btn_sniff_pressed() {
	key1_pressed = 0;key2_pressed = 0;key3_pressed = 0;key4_pressed = 0;key_foot_pressed = 0;  	     // 先重置
	
	if (HAL_GPIO_ReadPin(btn_1_GPIO_Port, btn_1_Pin) == 0) { // 判断按键
		vTaskDelay(pdMS_TO_TICKS(50));										     // 软件消抖
		if (HAL_GPIO_ReadPin(btn_1_GPIO_Port, btn_1_Pin) == 0) // 给一个值
			key1_pressed = 1;
	}
	if (HAL_GPIO_ReadPin(btn_2_GPIO_Port, btn_2_Pin) == 0) {
		vTaskDelay(pdMS_TO_TICKS(50));;
		if (HAL_GPIO_ReadPin(btn_2_GPIO_Port, btn_2_Pin) == 0)
			key2_pressed = 1;
	}
	if (HAL_GPIO_ReadPin(btn_3_GPIO_Port, btn_3_Pin) == 0) {
		vTaskDelay(pdMS_TO_TICKS(50));;
		if (HAL_GPIO_ReadPin(btn_3_GPIO_Port, btn_3_Pin) == 0)
			key3_pressed = 1;
	}
	if (HAL_GPIO_ReadPin(btn_4_GPIO_Port, btn_4_Pin) == 0) {
		vTaskDelay(pdMS_TO_TICKS(50));;
		if (HAL_GPIO_ReadPin(btn_4_GPIO_Port, btn_4_Pin) == 0)
			key4_pressed = 1;
	}
	if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 1) { // 脚踏踩下
		vTaskDelay(pdMS_TO_TICKS(50));;
		if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 1)
			key_foot_pressed = 1;
	}
	
	long btns_statu = key_foot_pressed << 4 | key4_pressed << 3 | key3_pressed << 2 | key2_pressed << 1 | key1_pressed;
	return btns_statu;
}

int active_key_1(int choose_type) {
	choose_type ^= 1;    // 异或1,进行0和1的切换
	osSemaphoreRelease(alu_chooseHandle);  // 发送信号量更新
	return choose_type;  // 返回选择类型更新
}

int active_key_2(int index_choose,float *temp_thres,float *power_thres) {
	if (index_choose==0)  //表示当前设置温度阈值
	{  
		if (*temp_thres < 150) {
			*temp_thres = *temp_thres+5;
		} else {
			*temp_thres = 150;
		}
	}
	else if(index_choose==1)  //表示当前设置功率阈值
	{
		if (*power_thres < 9.0f) {
			*power_thres = *power_thres+0.1f;
		}else{
			*power_thres = 9.0f;
		}
	}
	osSemaphoreRelease(alu_thresholdHandle);  // 发送信号量更新
	return 1;
}

int active_key_3(int index_choose,float *temp_thres,float *power_thres) {
	if (index_choose==0)  //表示当前设置温度阈值
	{  
		if (*temp_thres>=5){
			*temp_thres = *temp_thres-5;
		} else {
			*temp_thres = 0;
		}
	}
	else if(index_choose==1)  //表示当前设置功率阈值
	{
		if (*power_thres>=0.1f){
			*power_thres = *power_thres-0.1f;
		} else {
			*power_thres = 0.0f;
		}
	}
	osSemaphoreRelease(alu_thresholdHandle);  // 发送信号量更新
	return 1;
}

int active_key_4(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, uint8_t *data_485, float power_thres){
	data_485[6] = (uint8_t)(power_thres * 10);
	uint8_t xorResult = data_485[2] ^ data_485[3] ^ data_485[4] ^ data_485[5] ^ data_485[6];
	data_485[7] = xorResult;
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_SET);
	HAL_UART_Transmit(huart,(uint8_t*)data_485,10,0xFFFF);
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_RESET);
	return 1;
}

int active_key_1vs4(int* temp_modify) {
	if (*temp_modify == 0){
		*temp_modify = 10;
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
	} else {
		*temp_modify = 0;
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	}
	vTaskDelay(pdMS_TO_TICKS(1000));
	return 1;
}

// ====================================================================
// 引入外部变量 (这些变量统一在 alu_main.c 等文件中定义)
// ====================================================================
extern double K_Temperature;      
extern int    index_screen;       
extern int    index_choose;       
extern AluDynList sd_file_list;   
extern int    num_file;           
extern float  pwm_percent;        
extern int    temp_modify;        

// 引入系统加热控制的核心全局变量（在 alu_main.c 中定义）
extern volatile uint8_t is_heating_active;
extern volatile uint32_t heating_num_count;
extern char current_file_name[32];
extern PID_struct pid_TEMP;
extern uint8_t laser_test_state;
extern uint8_t pid_algorithm_type;
extern PID_Base_TypeDef pid_base;
extern FuzzyPID_struct fuzzy_pid_TEMP;
extern AdvPID_struct adv_pid_TEMP;

// ==========================================
// 串口PID交互新增全局变量定义
// ==========================================
QueueHandle_t UartRxQueue = NULL;
// uart_pid_state is defined in alu_main.c
float temp_Kp = 0.0f, temp_Ki = 0.0f, temp_Kd = 0.0f;

// ====================================================================
// 【重构】：脚踏瞬间启动函数 (纯非阻塞启动模式)
// 只要踩下脚踏，完成开火和建文件后立刻退出！循环计算由 10ms 任务接管。
// ====================================================================
int active_key_foot_start(uint8_t *data_485, float temp_thres, float power_thres)
{
	if (laser_test_state == 3) {
		uint8_t alu_485_laser_on[] = {0x55, 0x33, 0x01, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x0D};
		HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit(&huart4, alu_485_laser_on, 10, 0xFFFF);
		HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_RESET);
		is_heating_active = 1;
		return 1;
	}

	// 1. 设置并发送 RS485 开启激光指令
	data_485[6] = (uint8_t)(power_thres * 10);
	uint8_t xorResult = data_485[2] ^ data_485[3] ^ data_485[4] ^ data_485[5] ^ data_485[6]; 
	data_485[7] = xorResult;
	
	HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart4, (uint8_t*)data_485, 10, 0xFFFF);  
	HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_RESET);
	
	// 2. 通知系统切换到屏幕1 (加热图表界面)
	index_screen = 1;
	osSemaphoreRelease(alu_screenHandle);  

	// 3. 创建本次加热的全新 CSV 数据文件(sd_record_enable=1时)
	if (sd_record_enable) {
		num_file = Alu_SD_GetNextFileNum();
		sprintf(current_file_name, "data_%d.csv", num_file);

		// 通知 UI 刷新当前阈值
		osSemaphoreRelease(alu_thresholdHandle);

		// 4. 写入 PID 配置表头（第一行）
		char pidCfgBuf[256] = {0};
		if (pid_algorithm_type == 0) {
			snprintf(pidCfgBuf, sizeof(pidCfgBuf),
				"Mode: Standard PID, Kp: %.2f, Ki: %.2f, Kd: %.2f\n",
				pid_base.Kp, pid_base.Ki, pid_base.Kd);
		} else if (pid_algorithm_type == 1) {
			snprintf(pidCfgBuf, sizeof(pidCfgBuf),
				"Mode: Fuzzy PID, Kp: %.2f, Ki: %.2f, Kd: %.2f, e_max: %.1f, ec_max: %.1f, Kp_w: %.1f, Ki_w: %.2f, Kd_w: %.1f\n",
				pid_base.Kp, pid_base.Ki, pid_base.Kd,
				fuzzy_pid_TEMP.e_max, fuzzy_pid_TEMP.ec_max,
				fuzzy_pid_TEMP.Kp_weight, fuzzy_pid_TEMP.Ki_weight, fuzzy_pid_TEMP.Kd_weight);
		} else {
			snprintf(pidCfgBuf, sizeof(pidCfgBuf),
				"Mode: Advanced PID, Kp: %.2f, Ki: %.2f, Kd: %.2f, deadband: %.2f, mode_sw: %.2f, static_err: %.2f, temp_chg: %.2f, out_max_pd: %.2f, int_limit: %.2f, out_max_pid: %.2f\n",
				pid_base.Kp, pid_base.Ki, pid_base.Kd,
				adv_pid_TEMP.deadband_threshold, adv_pid_TEMP.mode_switch_threshold,
				adv_pid_TEMP.static_err_threshold, adv_pid_TEMP.temp_change_threshold,
				adv_pid_TEMP.output_max_pd_mode, adv_pid_TEMP.integral_limit,
				adv_pid_TEMP.output_max_pid_mode);
		}
		Alu_SD_write((uint8_t*)pidCfgBuf, strlen(pidCfgBuf), current_file_name);

		// 5. 写入 CSV 列标题（第二行）
		const char* BufferTitle = "index,temperature,speed_p,speed_i,speed_d,pwm_out";
		Alu_SD_write((uint8_t*)BufferTitle, strlen(BufferTitle), current_file_name);
	}
    
	// 5. 初始化并清空 PID 历史数据
	PID_init(&pid_TEMP);
	
	// 6. 重置加热循环计数器
	heating_num_count = 1;
	
	// 7. 【核心指令】：点火！将标志位置 1，瞬间唤醒 Task_Control 里的 10ms PID 闭环计算
	is_heating_active = 1;

	// 8. 立刻返回，彻底释放当前任务资源！
	return 1;
}
