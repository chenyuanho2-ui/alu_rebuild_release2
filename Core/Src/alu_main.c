#include "alu_main.h"
#include "alu_control.h"
#include "alu_file.h"
#include "usart.h"
#include "tim.h"
#include <stdio.h>
#include <stdlib.h> // 【新增】用于 free() 函数释放内存
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t SDWriteQueueHandle;
extern uint8_t uart_pid_state;
extern uint8_t pid_algorithm_type;
extern float temp_Kp, temp_Ki, temp_Kd;
extern QueueHandle_t UartRxQueue;

double K_Temperature   = 0;        
float  pwm_percent     = 0;        
int    temp_modify     = 0;        
float  temp_thres      = 60;	   
float  power_thres     = 1.0;      

int    index_screen    = 2;        
int    index_choose    = 1;        

AluDynList sd_file_list;           
int    num_file        = 0;        

// ==========================================
// 全局状态控制变量
// ==========================================
uint8_t need_stop_cleanup = 0;           // 清理信号
volatile uint8_t is_heating_active = 0;  // 1:正在加热, 0:停止加热
volatile uint32_t heating_num_count = 0; // 加热时间计数
uint8_t sd_record_enable = 0;            // SD 记录开关：1=启用，0=禁用
char current_file_name[32] = {0};        // 当前正在写入的CSV文件名
PID_struct pid_TEMP;                     // 全局PID对象

// ==========================================
// 收尾清理函数 (加入防卡死内存释放)
// ==========================================
void Heating_Stop_Routine(void) {
    vTaskDelay(pdMS_TO_TICKS(500)); // 等待队列最后一点数据安全写完
    
    // 【核心修复：彻底消灭内存泄露！】
    // 释放上一次 SD 卡扫描分配的动态内存，防止第二次运行榨干单片机 RAM
    if (sd_file_list.items != NULL) {
        for (int i = 0; i < sd_file_list.size; i++) {
            if (sd_file_list.items[i] != NULL) {
                free(sd_file_list.items[i]); // 释放每个文件名占用的内存
            }
        }
        free(sd_file_list.items);            // 释放列表数组本身的内存
    }
    
    // 释放干净后，再重新初始化并分配新内存
    Alu_list_init(&sd_file_list);
    Alu_sniff_files(&sd_file_list, "/"); // 重新扫描 SD 卡更新列表
    
    index_screen = 0;
    osSemaphoreRelease(alu_screenHandle); // 切回主屏幕
    vTaskDelay(pdMS_TO_TICKS(500));
}

void AluMain(void const * argument)
{
  HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_SET);
  uint8_t alu_485_send[] = {0x55, 0x33, 0x01, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x0D};
  HAL_UART_Transmit(&huart4,(uint8_t*)alu_485_send,10,0xFFFF);
  HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_RESET);
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1);
  
  Alu_list_init(&sd_file_list);  
  Alu_sniff_files(&sd_file_list,"/"); 

  num_file = Alu_SD_csv_num("/") + 1; 
  
  if (SDWriteQueueHandle == NULL) {
      SDWriteQueueHandle = xQueueCreate(20, 64); // 创建SD卡异步写入队列
  }
  
  vTaskDelay(pdMS_TO_TICKS(1000));
  index_screen = 0;
  osSemaphoreRelease(alu_screenHandle);
  
  long btns_statu = 0;  
  char recv_buf[64]; 
  
  for(;;)
  {
      // 1. 后台极速处理 SD 卡写入请求 (绝对不卡主循环)
      if (SDWriteQueueHandle != NULL) {
          while (xQueueReceive(SDWriteQueueHandle, recv_buf, 0) == pdTRUE) {
              Alu_SD_write((uint8_t*)recv_buf, strlen(recv_buf), current_file_name);
          }
      }

      // 2. 检测到松开脚踏的清理信号，安全收尾
      if (need_stop_cleanup == 1) {
          need_stop_cleanup = 0;
          Heating_Stop_Routine();
      }

      // 3. 处理串口接收（从环形FIFO读取数据发送到队列）
      Uart_Process_Rx();

      // 4. 串口PID交互状态机处理
      if (UartRxQueue != NULL) {
          char uart_buf[64] = {0};
          if (xQueueReceive(UartRxQueue, uart_buf, 0) == pdTRUE) {
              switch (uart_pid_state) {
                  case 0: {
                      if (strcmp(uart_buf, "SET_PID") == 0 || strcmp(uart_buf, "set_pid") == 0) {
                          if (is_heating_active == 0 && pid_algorithm_type == 0) {
                              uart_pid_state = 1;
                              printf("当前PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", pid_TEMP.Kp, pid_TEMP.Ki, pid_TEMP.Kd);
                              printf("请输入新参数(格式: x,y,z):\r\n");
                          } else if (is_heating_active != 0) {
                              printf("错误: 正在加热中，无法修改PID参数\r\n");
                          } else {
                              printf("错误: 当前不是普通PID模式\r\n");
                          }
                      }
                      break;
                  }
                  case 1: {
                      if (sscanf(uart_buf, "%f,%f,%f", &temp_Kp, &temp_Ki, &temp_Kd) == 3) {
                          uart_pid_state = 2;
                          printf("解析成功: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", temp_Kp, temp_Ki, temp_Kd);
                          printf("确认修改请回复 Y，重新输入请回复 N\r\n");
                      } else {
                          printf("格式错误，请重新输入(格式: x,y,z):\r\n");
                      }
                      break;
                  }
                  case 2: {
                      if (uart_buf[0] == 'Y' || uart_buf[0] == 'y') {
                          pid_TEMP.Kp = temp_Kp;
                          pid_TEMP.Ki = temp_Ki;
                          pid_TEMP.Kd = temp_Kd;
                          printf("修改成功，已退出修改模式\r\n");
                          uart_pid_state = 0;
                      } else if (uart_buf[0] == 'N' || uart_buf[0] == 'n') {
                          printf("已取消，请输入新参数(格式: x,y,z):\r\n");
                          uart_pid_state = 1;
                      } else {
                          printf("输入无效，请回复 Y 或 N\r\n");
                      }
                      break;
                  }
                  default:
                      break;
              }
          }
      }

      // 4. 扫描按键
      btns_statu = btn_sniff_pressed();

      switch (btns_statu) {
          case 1: 
              index_choose = active_key_1(index_choose); 
              break;
          case 2: 
              active_key_2(index_choose, &temp_thres, &power_thres);   
              break;
          case 4: 
              active_key_3(index_choose, &temp_thres, &power_thres);   
              break;
          case 8: 
              active_key_4(&huart4, &htim1, alu_485_send, power_thres);
              break;
          case (1+8): 
              active_key_1vs4(&temp_modify);
              break;
          case 16: // 脚踏踩下
              // 必须在未加热状态下踩下才有效，防止重复触发
              if (is_heating_active == 0) {
                  active_key_foot_start(alu_485_send, temp_thres, power_thres);
              }
              break;  
      }
      
      vTaskDelay(pdMS_TO_TICKS(50));
  }
}
