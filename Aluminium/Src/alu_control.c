#include "alu_control.h"
#include "alu_file.h"
#include "pid.h"
#include "temp_filter.h"

// 原有的按键状态变量
uint8_t key1_pressed;
uint8_t key2_pressed;
uint8_t key3_pressed;
uint8_t key4_pressed;
uint8_t key_foot_pressed;

// ==========================================
// 【新增】：全局状态变量的实际定义
// 这些变量被提取出来，供 freertos.c 里的控制任务跨文件调用
// ==========================================
uint8_t is_heating_active = 0;   // 默认不加热
int heating_num_count = 1;       // 默认计数为 1
char file_name_cache[14];        // 文件名缓存
PID_struct pid_TEMP;             // 全局 PID 实例

// ==========================================
// 原有的按键扫描与参数修改函数 (保持原样)
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
	if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 1) {
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

extern double K_Temperature;      
extern int    index_screen;       
extern int    index_choose;       
extern AluDynList sd_file_list;   
extern int    num_file;           
extern float  pwm_percent;        
extern int    temp_modify;        


// ====================================================================
// 【新增函数】：加热停止与系统清理逻辑
// 负责关闭硬件(RS485)并恢复 UI(切回主屏幕、重新扫描文件)
// ====================================================================
void Heating_Stop_Routine(void)
{
	// 1. 关闭 RS485 输出
	HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_SET);
	uint8_t alu_485_off[] = {0x55, 0x33, 0x01, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x0D};
	HAL_UART_Transmit(&huart4, (uint8_t*)alu_485_off, 10, 0xFFFF);
	HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_RESET);
	
	// 2. 清理标志位，彻底停止控制任务里的 PID 运算
	is_heating_active = 0;
    
	// 3. 延时等待 SD 卡缓冲区完全刷入硬件
	vTaskDelay(pdMS_TO_TICKS(1000));
    
	// 4. 重新扫描根目录下的文件
	Alu_list_init(&sd_file_list);
	Alu_sniff_files(&sd_file_list, "/");
    
	// 5. 切换回主屏幕 (显示文件列表)
	index_screen = 0;
	osSemaphoreRelease(alu_screenHandle);  
	vTaskDelay(pdMS_TO_TICKS(1000));
}

// ====================================================================
// 【重构】：脚踏触发函数 (纯非阻塞启动模式)
// 只要踩下脚踏，设置好初始参数后就立刻退出，唤醒后台控制任务，绝不死等！
// ====================================================================
int active_key_foot(uint8_t *data_485, float temp_thres, float power_thres)
{
	// 1. 设置并发送 RS485 开启指令
	data_485[6] = (uint8_t)(power_thres * 10);
	uint8_t xorResult = data_485[2] ^ data_485[3] ^ data_485[4] ^ data_485[5] ^ data_485[6]; 
	data_485[7] = xorResult;
	HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart4, (uint8_t*)data_485, 10, 0xFFFF);  
	HAL_GPIO_WritePin(flag_485_GPIO_Port, flag_485_Pin, GPIO_PIN_RESET);
	
	// 2. 切换到加热数据显示界面
	index_screen = 1;
	osSemaphoreRelease(alu_screenHandle);  
	
	// 3. 准备生成新的 CSV 数据文件
	num_file = Alu_SD_csv_num("/") + 1; 
	sprintf(file_name_cache, "data_%d.csv", num_file);  
	
	// 通知 UI 阈值可能已改变
	osSemaphoreRelease(alu_thresholdHandle);
	
	// 4. 写入 CSV 表头
	uint8_t BufferTitle[] = "index,temperature,speed_p,speed_i,speed_d";
	Alu_SD_write(BufferTitle, sizeof(BufferTitle), (const char *)file_name_cache);
    
	// 5. 初始化 PID 模块
	PID_init(&pid_TEMP);
	
	// 6. 重置计数器
	heating_num_count = 1;
	
	// 7. 【核心指令】：点火！将标志位置 1，唤醒 Task_Control 里的 PID 循环
	is_heating_active = 1;

	// 8. 立刻返回，绝不死锁当前按键任务！
	return 1;
}
