#include "alu_main.h"
#include "alu_control.h"
#include "alu_file.h"
#include "usart.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t SDWriteQueueHandle;


double K_Temperature   = 0;        // 温度值
float  pwm_percent     = 0;        // pwm占空比
int    temp_modify     = 0;        // 温度偏移
float  temp_thres      = 30;	   // 温度阈值

float  power_thres     = 0;        // 功率阈值

int    index_screen    = 2;        // 屏幕切换 0为SrcMain,1为Screen1, 2为launch
int    index_choose    = 1;        // 0为设置温度阈值,1为设置功率阈值

AluDynList sd_file_list;           // 动态文件列表
int    num_file        = 0;        // 当前扫描到的文件数量

// ==========================================================
// 按键与 UI 逻辑主任务 (覆盖 freertos.c 中的 __weak 空函数)
// ==========================================================
void AluMain(void const * argument)
{
  // 1. 485芯片的使能脚,默认关闭接收和发送
  HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_SET);
  uint8_t alu_485_send[] = {0x55, 0x33, 0x01, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x0D};
  HAL_UART_Transmit(&huart4,(uint8_t*)alu_485_send,10,0xFFFF);
  HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_RESET);
  
  // 2. 开启 PWM 输出
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1); // 0~999
  
  // 3. 初始化文件列表
  Alu_list_init(&sd_file_list);  
  Alu_sniff_files(&sd_file_list,"/"); // 扫描根目录
  for (int i = 0; i < sd_file_list.size; i++) {
      printf("File %d: %s\n", i+1, sd_file_list.items[i]);
  }

  int num_file = Alu_SD_csv_num("/") + 1; // 扫描根目录下的文件数
  printf("num_file %d\n",num_file);
  
  // 4. 创建 SD 卡写入队列
  if (SDWriteQueueHandle == NULL) {
      SDWriteQueueHandle = xQueueCreate(10, 64);
  }
  
  vTaskDelay(pdMS_TO_TICKS(1000));
  index_screen = 0;
  osSemaphoreRelease(alu_screenHandle);  // 释放屏幕切换信号量资源,切换到SrcMain
  
  long btns_statu = 0;  // 按键状态  
  char recv_buf[64]; // 接收缓冲区
  
  /* Infinite loop */
  for(;;)
  {
      // 后台慢慢处理 SD 卡写入请求
      // timeout 设为 0 表示快速轮询，不挂起当前 UI 任务
		if (SDWriteQueueHandle != NULL) {
			// 把 if 改成 while，一口气把队列里积压的数据全写进SD卡
			while (xQueueReceive(SDWriteQueueHandle, recv_buf, 0) == pdTRUE) {
				Alu_SD_write((uint8_t*)recv_buf, strlen(recv_buf), (const char *)file_name_cache);
			}
		}
      
      btns_statu = btn_sniff_pressed();
      
      if (btns_statu>0){
          printf("btn pressed %ld\n", btns_statu);
      }
      
      switch (btns_statu) {
          case 1: // 按键1
              index_choose = active_key_1(index_choose); 
              break;
          case 2: // 按键2 增加阈值  
              active_key_2(index_choose, &temp_thres, &power_thres);   
              break;
          case 4: // 按键3 减少阈值
              active_key_3(index_choose, &temp_thres, &power_thres);   
              break;
          case 8: // 按键4 发送实际参数
              active_key_4(&huart4, &htim1, alu_485_send, power_thres);
              break;
          case (1+8): // 按键1+4 温度偏移
              active_key_1vs4(&temp_modify);
              break;
          case 16: // 脚踏
              // 踩下脚踏只会瞬间发信号点火，绝不死锁当前任务！
              active_key_foot(alu_485_send, temp_thres, power_thres);
              break;  
      }
      
      // 规规矩矩的 50ms 按键消抖延时
      vTaskDelay(pdMS_TO_TICKS(50));
  }
}
