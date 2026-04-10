#include "alu_control.h"
#include "alu_file.h"
#include "alu_temp.h"
#include "temp_filter.h"  // 【新增】：引入测温接口

uint8_t key1_pressed;
uint8_t key2_pressed;
uint8_t key3_pressed;
uint8_t key4_pressed;
uint8_t key_foot_pressed;

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

int active_key_foot(uint8_t *data_485, float temp_thres,float power_thres)
{
	uint8_t key_foot_pressed = 0;  
	
	data_485[6] = (uint8_t)(power_thres * 10);
	uint8_t xorResult = data_485[2] ^ data_485[3] ^ data_485[4] ^ data_485[5] ^ data_485[6]; 
	data_485[7] = xorResult;
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_SET);
	HAL_UART_Transmit(&huart4,(uint8_t*)data_485,10,0xFFFF);  
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_RESET);
	
	index_screen = 1;
	osSemaphoreRelease(alu_screenHandle);  
	
	num_file = Alu_SD_csv_num("/") + 1; 
	char file_name_cache[14];				
	sprintf(file_name_cache, "data_%d.csv", num_file);  
	
	osSemaphoreRelease(alu_thresholdHandle);
	
	uint8_t BufferTitle[] = "index,temperature,speed_p,speed_i,speed_d";
	uint8_t BufferWrite[50] = " "; 
	Alu_SD_write(BufferTitle, sizeof(BufferTitle), (const char *)file_name_cache);
    
	PID_struct pid_TEMP;      
	PID_init(&pid_TEMP);
	
	int num_count = 1;
	// 【新增】：用于 PID 逻辑的分频时间戳
	uint32_t last_pid_tick = HAL_GetTick();

	do{
		// -------------------------------------------------------------------
		// 【关键修改1】：即使在这个 do-while 循环里，也要不停调用测温引擎
		// 这样串口打印才不会停，屏幕温度才会更新
		TempFilter_Process(); 
		// -------------------------------------------------------------------

		// 【关键修改2】：使用“分频锁”，让 PID 和写 SD 卡逻辑每 250ms 才跑一次
		if (HAL_GetTick() - last_pid_tick >= 250)
		{
			K_Temperature = alu_SPI_gettemp(); 
			if (K_Temperature >= 150) K_Temperature = 150;
			else if (K_Temperature <= 0) K_Temperature = 0;
			K_Temperature = K_Temperature + temp_modify;  
			
			pwm_percent = PID_PWM_iteration(&pid_TEMP, temp_thres, K_Temperature) / 1000; 
			osSemaphoreRelease(alu_temperatureHandle);  
			
			sprintf((char *)BufferWrite, "\n%d,%.2f,%.3f,%.3f,%.3f", num_count, K_Temperature, pid_TEMP.speed[0], pid_TEMP.speed[1], pid_TEMP.speed[2]);
			Alu_SD_write(BufferWrite, sizeof(BufferWrite), (const char *)file_name_cache);
			
			// 退出判断逻辑移动到这里
			if (num_count % 5 == 0 && num_count > 10) {     
				if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 1) {  
					HAL_Delay(20);                 
					if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 1)
						key_foot_pressed = 1;      
				} else {
					key_foot_pressed = 0;      
				}
			}
			
			num_count = num_count + 1;
			last_pid_tick = HAL_GetTick(); // 重置时间戳
		}
		
		// 【关键修改3】：把原本死等 250ms 改为 10ms
		// 这样循环每 10ms 转一圈，测温引擎才能发挥作用
		vTaskDelay(pdMS_TO_TICKS(10)); 

	} while (key_foot_pressed==0); 
	
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_SET);
	uint8_t alu_485_off[] = {0x55, 0x33, 0x01, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x0D};
	HAL_UART_Transmit(&huart4,(uint8_t*)alu_485_off,10,0xFFFF);
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_RESET);
	
	vTaskDelay(pdMS_TO_TICKS(1000));
	Alu_list_init(&sd_file_list);
	Alu_sniff_files(&sd_file_list,"/");
	index_screen = 0;
	osSemaphoreRelease(alu_screenHandle);  
	vTaskDelay(pdMS_TO_TICKS(1000));
	return 1;
}
