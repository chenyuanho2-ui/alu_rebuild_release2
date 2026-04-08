#include "alu_control.h"
#include "alu_file.h"
#include "alu_temp.h"



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

	data_485[6] = (uint8_t)(power_thres * 10);// 将乘以10后的值,强转直接就是整数部分
	// 异或操作放到校验位
	uint8_t xorResult = data_485[2] ^ data_485[3] ^ data_485[4] ^ data_485[5] ^ data_485[6];
	data_485[7] = xorResult;
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_SET);
	HAL_UART_Transmit(huart,(uint8_t*)data_485,10,0xFFFF);  // 已经传指针了
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_RESET);
	
	
	printf("send_data:{ ");
    for (int i = 0; i < 10; i++) {
        printf("0x%02X, ", data_485[i]);
    }printf(" }\n");
	
	return 1;
}


int active_key_1vs4(int* temp_modify) {
	if (*temp_modify == 0){
		*temp_modify = 10;
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);  // 拉低LED点亮+10
	} else {
		*temp_modify = 0;
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);    // 拉高LED熄灭+ 0
	}
	vTaskDelay(pdMS_TO_TICKS(1000));
	return 1;
}




extern double K_Temperature;      // 温度值
extern int    index_screen;       // 屏幕索引
extern int    index_choose;       // 设置索引 功率还是温度
extern AluDynList sd_file_list;   // 文件列表
extern int    num_file;           // 待创建文件名
extern float  pwm_percent;        // pwn比例

extern int    temp_modify;        // 温度偏移
/*
	跳转窗口
	485发送功率阈值
	按照温度阈值进行PID调试
	存储文件名
*/
int active_key_foot(uint8_t *data_485, float temp_thres,float power_thres)
{
	uint8_t key_foot_pressed = 0;  // 重置脚踏状态
	
	data_485[6] = (uint8_t)(power_thres * 10);// 将乘以10后的值,强转直接就是整数部分
	uint8_t xorResult = data_485[2] ^ data_485[3] ^ data_485[4] ^ data_485[5] ^ data_485[6]; // 异或操作放到校验位
	data_485[7] = xorResult;
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_SET);
	HAL_UART_Transmit(&huart4,(uint8_t*)data_485,10,0xFFFF);  // 已经传指针了
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_RESET);
	
	
	
	
	// 发送屏幕切换信号量更新,切换到Srceen1
	index_screen = 1;
	osSemaphoreRelease(alu_screenHandle);  
;  
	
	// 确定保存文件名
	num_file = Alu_SD_csv_num("/") + 1; // 获得当前对应存储的文件名
	printf("num_file %d\n",num_file);
	char file_name_cache[14];				// 这次采集的文件名  = "DAT_5.CSV"
	sprintf(file_name_cache, "data_%d.csv", num_file);  //必须是<xxx>_%d.<xxx>的格式，Alu_SD_csv_num检测新文件名需要
	
	// 发送阈值信号量更新
	osSemaphoreRelease(alu_thresholdHandle);
	
	// 文件读取数组
	uint8_t BufferTitle[] = "index,temperature,speed_p,speed_i,speed_d";
	uint8_t BufferWrite[50] = " "; // 每行写入的数据
	Alu_SD_write(BufferTitle, sizeof(BufferTitle), (const char *)file_name_cache);
    
	// 初始化PID对象
	PID_struct pid_TEMP;      
	PID_init(&pid_TEMP);
	
	// 循环采集温度并按阈值调节PWM占比
	int num_count = 1;
	do{
		K_Temperature = alu_SPI_gettemp(); // 获取当前温度值
		if (K_Temperature >= 150)
			K_Temperature = 150;
		else if (K_Temperature <= 0)
			K_Temperature = 0;
		K_Temperature = K_Temperature + temp_modify;  // 添加温度偏移
		
		pwm_percent = PID_PWM_iteration(&pid_TEMP, temp_thres, K_Temperature) / 1000; // 调整PID，输出PWM比例
		
		osSemaphoreRelease(alu_temperatureHandle);  // 发送信号量更新
		
		
		sprintf((char *)BufferWrite, "\n%d,%.2f,%.3f,%.3f,%.3f", num_count, K_Temperature, pid_TEMP.speed[0], pid_TEMP.speed[1], pid_TEMP.speed[2]);
//		sprintf((char *)BufferWrite, "%d,%.2f\n", num_count, K_Temperature);
		Alu_SD_write(BufferWrite, sizeof(BufferWrite), (const char *)file_name_cache);  // 很奇怪，为什么在临界区之外才能正常读写
		num_count = num_count + 1;
		
		// 延时以及退出的判断
		vTaskDelay(pdMS_TO_TICKS(250));
		if (num_count % 5 == 0 && num_count > 10) {     //  加上10此之后才进行结束判断
			if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 1) {  //判断按键
				HAL_Delay(20);                 // 软件消抖
				if (HAL_GPIO_ReadPin(btn_foot_GPIO_Port, btn_foot_Pin) == 1)
					key_foot_pressed = 1;      // 踩下 1
			} else {
					key_foot_pressed = 0;      // 没踩 0
			}
		}
	} while (key_foot_pressed==0); // 检测脚踏状态查看是否退出 从检测1变成检测0
	
	
	// 关闭输出,发送屏幕切换信号量更新,切换到main,并更新文件列表
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_SET);
	uint8_t alu_485_send[] = {0x55, 0x33, 0x01, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x0D};
	HAL_UART_Transmit(&huart4,(uint8_t*)alu_485_send,10,0xFFFF);
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_RESET);
	
	vTaskDelay(pdMS_TO_TICKS(1000));
	Alu_list_init(&sd_file_list);
	Alu_sniff_files(&sd_file_list,"/");
	index_screen = 0;
	osSemaphoreRelease(alu_screenHandle);  
	vTaskDelay(pdMS_TO_TICKS(1000));
	return 1;
}

