#include "alu_main.h"
#include "alu_control.h"
#include "alu_file.h"
#include "usart.h"
#include "tim.h"
#include <stdio.h>

// 引入外部需要用到的全局变量
extern float temp_thres;
extern float power_thres;
extern int   temp_modify;
extern int   index_screen;
extern int   index_choose;
extern AluDynList sd_file_list;

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
  
  vTaskDelay(pdMS_TO_TICKS(1000));
  index_screen = 0;
  osSemaphoreRelease(alu_screenHandle);  // 释放屏幕切换信号量资源,切换到SrcMain
  
  long btns_statu = 0;  // 按键状态  
  
  /* Infinite loop */
  for(;;)
  {
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
