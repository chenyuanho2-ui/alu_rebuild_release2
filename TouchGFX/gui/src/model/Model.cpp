#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#ifndef SIMULATOR

#include "cmsis_os.h"
#include "main.h"



extern osSemaphoreId alu_chooseHandle;         // 选择是温度还是功率
extern osSemaphoreId alu_thresholdHandle;      // 功率阈值设置信号量

extern osSemaphoreId alu_temperatureHandle;    // 温度信息更新信号量
extern osSemaphoreId alu_screenHandle;         // 屏幕切换信号量

extern osSemaphoreId alu_savenameHandle;       // 保存文件名信号量

extern double K_Temperature;      // 温度值
extern float  pwm_percent;        // pwn比例
extern int    index_screen;       // 屏幕索引
extern int    index_choose;       // 设置索引 功率还是温度

extern float  temp_thres;	      // 温度阈值
extern float  power_thres;        // 功率阈值

extern AluDynList sd_file_list;   // 文件列表
extern int    num_file;           // 文件待创建文件名
#endif

Model::Model() : modelListener(0)
{
}

void Model::tick()
{
#ifndef SIMULATOR
	
// 从硬件层向前面板传递的代码
	if(osOK == osSemaphoreWait(alu_temperatureHandle,0)) // 更新温度
	{
		modelListener->alu_to_temperature(K_Temperature, pwm_percent);  
	}
	else if(osOK == osSemaphoreWait(alu_screenHandle,0)) // 更新页面
	{  
		modelListener->alu_to_screen(index_screen,&sd_file_list);
	}
	else if(osOK == osSemaphoreWait(alu_chooseHandle,0)) // 更新设置项
	{
		modelListener->alu_to_choose(index_choose);
	}
	else if(osOK == osSemaphoreWait(alu_thresholdHandle,0))  // 设置阈值参数更新
	{
		modelListener->alu_to_thres(index_choose,temp_thres,power_thres,num_file);  // 在screen1中直接确认文件
	}

#endif
}


void Model::alu_do_back_test(bool state)
{
#ifndef SIMULATOR
	if (state)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
	}
#endif
}

void Model::alu_do_back_delFile(int file_index,const char * file_name)
{
#ifndef SIMULATOR
	printf("remove: %d ",file_index);
	printf("File: %s\n", file_name);
	Alu_SD_del_file(file_name);
#endif
}



