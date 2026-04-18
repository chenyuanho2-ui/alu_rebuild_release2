#include "alu_main.h"
#include "alu_control.h"
#include "alu_file.h"
#include "usart.h"
#include "tim.h"
#include "pid_storage.h"
#include "fuzzy_pid.h"
#include "advanced_pid.h"
#include "pid.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t SDWriteQueueHandle;
extern uint8_t uart_pid_state;
extern float temp_Kp, temp_Ki, temp_Kd;
extern QueueHandle_t UartRxQueue;

// ================================================
// 变量分类说明:
// A区-用户可配置变量: 初始值设定处，可按需修改
// B区-系统内部运行变量: 由系统自动更新，仅供读取
// C区-UI界面状态变量: 界面显示和交互相关
// D区-数据结构体实例: PID控制器实例
// E区-文件与激光测试: SD卡管理与激光测试参数
// ================================================

// ================================================
// A区: 用户可配置变量 (初始值设定处，可按需修改)
// ================================================
float  temp_thres      = 65;   // 目标温度阈值(°C)，范围0-150，按Key2/Key3调节
float  power_thres     = 2.6;  // 激光功率阈值(0-9.0)，按Key2/Key3调节
int    temp_modify     = 0;    // 温度修正值，Key1+Key4组合键切换(0或10)，补偿传感器误差
uint8_t pid_algorithm_type = 2;    // PID算法类型：0=标准PID，1=模糊PID，2=高级PID
uint8_t enable_pid_tune = 1;       // PID串口调试使能：0=禁用，1=启用(通过set_pid指令调节)
uint8_t enable_laser_test = 0;     // 激光测试模式使能：0=禁用，1=启用(通过laseron指令)
uint8_t sd_record_enable = 1;             // SD卡温度数据记录使能：1=记录，0=禁用
uint8_t sd_pid_save_enable = 1;           // SD卡PID参数保存使能：1=保存，0=禁用
uint8_t enable_stack_print = 0;          // 堆栈打印使能：0=禁用，1=启用(每1s打印任务堆栈)
uint8_t enable_print_timing = 0;         // 打印耗时统计使能：0=禁用，1=启用(每1s打印最长打印耗时和内容)
uint8_t hardware_break_enabled = 0;     // 硬件刹车功能开关：0=关闭，1=开启

// ================================================
// B区: 系统内部运行变量 (只读，勿手动修改)
// ================================================
double K_Temperature   = 0;    // 实时温度值，双通道滤波后供PID和UI使用(°C)
float  pwm_percent     = 0;    // PWM输出百分比(0-1)，只读，用于UI显示
float  g_display_temp_avg = 0;  // 屏幕显示用1s平均温度
volatile uint8_t is_heating_active = 0;   // 加热激活状态：1=加热中，0=停止
volatile uint32_t heating_num_count = 0;  // 加热计数：CSV数据行号，每10ms递增
uint8_t need_stop_cleanup = 0;           // 停止加热清理标志：脚踏松开时置1，触发数据保存和界面切换
uint8_t is_serial_interacting = 0;      // 串口交互状态标志：防止加热时串口指令冲突
uint8_t laser_test_state = 0;           // 激光测试状态机：0=空闲，1=输入电流PWM，2=确认，3=就绪

// PID调试状态机
uint8_t uart_pid_state = 0;           // 基础PID状态：0=空闲，1=输入参数，2=确认
uint8_t uart_apid_state = 0;          // 高级PID状态：0=空闲，1=菜单，2=输入参数
uint8_t uart_fpid_state = 0;          // 模糊PID状态：0=空闲，1=菜单，2=输入参数
uint8_t uart_pidmode_state = 0;       // PID模式切换状态：0=空闲，1=输入模式
uint8_t has_valid_input = 0;           // 有效输入标志
uint8_t current_apid_param = 0;        // 当前高级PID参数编号
uint8_t current_fpid_param = 0;        // 当前模糊PID参数编号
char current_file_name[32] = {0};      // 当前数据文件名，格式: "data_N.csv"

// ================================================
// C区: UI界面状态变量
// ================================================
int    index_screen    = 2;    // 当前屏幕索引：0=主界面，1=加热监控界面
int    index_choose    = 1;    // 当前设置项选择：0=温度阈值，1=功率阈值

// ================================================
// D区: 数据结构体实例
// ================================================
PID_struct pid_TEMP;              // 标准PID结构体实例
FuzzyPID_struct fuzzy_pid_TEMP;   // 模糊PID结构体实例
AdvPID_struct adv_pid_TEMP;       // 高级PID结构体实例

// ================================================
// E区: 文件与激光测试
// ================================================
AluDynList sd_file_list;       // SD卡文件动态列表，管理已存储的CSV文件
int    num_file        = 0;    // 当前CSV文件编号(递增)，格式: data_N.csv
float target_laser_current = 0.0f; // 激光测试目标电流值(mA)
float target_laser_pwm = 0.0f;     // 激光测试目标PWM值(0-1000)

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
  
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1);
  
  Alu_list_init(&sd_file_list);  
  Alu_sniff_files(&sd_file_list,"/"); 

  num_file = Alu_SD_csv_num("/") + 1; 
  
  if (SDWriteQueueHandle == NULL) {
      SDWriteQueueHandle = xQueueCreate(20, 64); // 创建SD卡异步写入队列
  }
  if (UartRxQueue == NULL) {
      UartRxQueue = xQueueCreate(10, 64);  // 创建串口接收队列
  }

  SD_Load_PID_Config();

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

      // 4. 串口指令解析
      if (UartRxQueue != NULL) {
          char uart_buf[64] = {0};
          if (xQueueReceive(UartRxQueue, uart_buf, 0) == pdTRUE) {
              if (laser_test_state == 0) {
                  if (strcmp(uart_buf, "laseron") == 0 || strcmp(uart_buf, "LASERON") == 0) {
                      if (enable_laser_test == 1 && is_heating_active == 0) {
                          is_serial_interacting = 1;
                          laser_test_state = 1;
                          printf("Laser Test Mode ON\r\n");
                          printf("Enter target current and PWM (format: X,Y):\r\n");
                      } else if (enable_laser_test == 0) {
                          printf("Error: Laser test not enabled (set enable_laser_test=1)\r\n");
                      } else {
                          printf("Error: Cannot enter laser mode while heating active\r\n");
                      }
                  } else if (strcmp(uart_buf, "pid") == 0 || strcmp(uart_buf, "PID") == 0) {
                      if (enable_pid_tune == 1 && is_heating_active == 0) {
                          is_serial_interacting = 1;
                          uart_pid_state = 1;
                          has_valid_input = 0;
                          printf("PID Tuning Mode ON\r\n");
                          printf("Current: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", pid_base.Kp, pid_base.Ki, pid_base.Kd);
                          printf("Enter new params (format: Kp,Ki,Kd):\r\n");
                          printf("Enter N to exit\r\n");
                      } else if (enable_pid_tune == 0) {
                          printf("Error: PID tuning not enabled (set enable_pid_tune=1)\r\n");
                      } else {
                          printf("Error: Cannot tune PID while heating active\r\n");
                      }
                  } else if (strcmp(uart_buf, "apid") == 0 || strcmp(uart_buf, "APID") == 0) {
                      if (enable_pid_tune == 1 && is_heating_active == 0) {
                          is_serial_interacting = 1;
                          uart_apid_state = 1;
                          has_valid_input = 0;
                          printf("Advanced PID Tuning Mode ON\r\n");
                          printf("1. Deadband Threshold (0.5)\r\n");
                          printf("2. Mode Switch Threshold (10.0)\r\n");
                          printf("3. Static Error Threshold (5.0)\r\n");
                          printf("4. Temp Change Threshold (5.0)\r\n");
                          printf("5. Output Max PD Mode (750.0)\r\n");
                          printf("6. Integral Limit (1000.0)\r\n");
                          printf("7. Output Max PID Mode (1000.0)\r\n");
                          printf("Enter parameter number to modify, Y to save exit, N to exit\r\n");
                      } else if (enable_pid_tune == 0) {
                          printf("Error: PID tuning not enabled (set enable_pid_tune=1)\r\n");
                      } else {
                          printf("Error: Cannot tune PID while heating active\r\n");
                      }
                  } else if (strcmp(uart_buf, "fpid") == 0 || strcmp(uart_buf, "FPID") == 0) {
                      if (enable_pid_tune == 1 && is_heating_active == 0) {
                          is_serial_interacting = 1;
                          uart_fpid_state = 1;
                          has_valid_input = 0;
                          printf("Fuzzy PID Tuning Mode ON\r\n");
                          printf("1. E Max (50.0)\r\n");
                          printf("2. EC Max (5.0)\r\n");
                          printf("3. Kp Weight (5.0)\r\n");
                          printf("4. Ki Weight (0.1)\r\n");
                          printf("5. Kd Weight (10.0)\r\n");
                          printf("Enter parameter number to modify, Y to save exit, N to exit\r\n");
                      } else if (enable_pid_tune == 0) {
                          printf("Error: PID tuning not enabled (set enable_pid_tune=1)\r\n");
                      } else {
                          printf("Error: Cannot tune PID while heating active\r\n");
                      }
                  } else if (strcmp(uart_buf, "pidmode") == 0 || strcmp(uart_buf, "PIDMODE") == 0) {
                      if (is_heating_active == 0) {
                          is_serial_interacting = 1;
                          uart_pidmode_state = 1;
                          printf("PID Mode Switch\r\n");
                          printf("Current mode: %d\r\n", pid_algorithm_type);
                          printf("Enter 0 (Basic), 1 (Fuzzy), 2 (Advanced):\r\n");
                      } else {
                          printf("Error: Cannot switch mode while heating active\r\n");
                      }
                  } else if (strcmp(uart_buf, "help") == 0 || strcmp(uart_buf, "HELP") == 0) {
                      printf("=== Commands ===\r\n");
                      printf("pid - Enter basic PID tuning mode\r\n");
                      printf("apid - Enter advanced PID tuning mode\r\n");
                      printf("fpid - Enter fuzzy PID tuning mode\r\n");
                      printf("pidmode - Switch PID algorithm mode\r\n");
                      printf("laseron - Enter laser test mode\r\n");
                      printf("help - Show this message\r\n");
                  }
              } else if (laser_test_state == 1) {
                  float tmp_current = 0.0f;
                  float tmp_pwm = 0.0f;
                  if (sscanf(uart_buf, "%f,%f", &tmp_current, &tmp_pwm) == 2) {
                      target_laser_current = tmp_current;
                      target_laser_pwm = tmp_pwm;
                      laser_test_state = 2;
                      printf("Target: Current=%.2f, PWM=%.2f\r\n", target_laser_current, target_laser_pwm);
                      printf("Confirm apply? [Y/N]:\r\n");
                  } else {
                      printf("Format error, enter: X,Y (e.g., 2.5,500)\r\n");
                  }
              } else if (laser_test_state == 2) {
                  if (uart_buf[0] == 'Y' || uart_buf[0] == 'y') {
                      laser_test_state = 3;
                      is_serial_interacting = 0;
                      printf("Laser ready! Step on foot pedal to test.\r\n");
                  } else if (uart_buf[0] == 'N' || uart_buf[0] == 'n') {
                      target_laser_current = 0.0f;
                      target_laser_pwm = 0.0f;
                      laser_test_state = 1;
                      printf("Cancelled. Enter new values (format: X,Y):\r\n");
                  } else {
                      printf("Reply Y or N\r\n");
                  }
              }

              if (laser_test_state > 0) {
                  if (strcmp(uart_buf, "laseroff") == 0 || strcmp(uart_buf, "LASEROFF") == 0) {
                      laser_test_state = 0;
                      target_laser_current = 0.0f;
                      target_laser_pwm = 0.0f;
                      is_serial_interacting = 0;
                      printf("Laser test mode OFF\r\n");
                  }
              }

              // 基础PID调试状态机
              if (uart_pid_state == 1 && laser_test_state == 0) {
                  if (uart_buf[0] == 'N' || uart_buf[0] == 'n') {
                      uart_pid_state = 0;
                      is_serial_interacting = 0;
                      printf("PID tuning cancelled\r\n");
                  } else if (sscanf(uart_buf, "%f,%f,%f", &temp_Kp, &temp_Ki, &temp_Kd) == 3) {
                      uart_pid_state = 2;
                      has_valid_input = 1;
                      printf("Parsed: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", temp_Kp, temp_Ki, temp_Kd);
                      printf("Confirm save [Y], Re-enter [N]:\r\n");
                  } else {
                      printf("Format error, enter: Kp,Ki,Kd (e.g., 40,0.8,125)\r\n");
                  }
              } else if (uart_pid_state == 2 && laser_test_state == 0) {
                  if (uart_buf[0] == 'N' || uart_buf[0] == 'n') {
                      uart_pid_state = 0;
                      is_serial_interacting = 0;
                      printf("PID tuning cancelled\r\n");
                  } else if (uart_buf[0] == 'Y' || uart_buf[0] == 'y') {
                      if (has_valid_input) {
                          pid_base.Kp = temp_Kp;
                          pid_base.Ki = temp_Ki;
                          pid_base.Kd = temp_Kd;
                          SD_Save_PID_Config(temp_Kp, temp_Ki, temp_Kd);
                          printf("Success, PID updated\r\n");
                          uart_pid_state = 0;
                          is_serial_interacting = 0;
                      } else {
                          printf("No valid input detected. Press N to exit or enter parameters first\r\n");
                      }
                  } else {
                      printf("Reply Y or N\r\n");
                  }
              }

              // 高级PID调试状态机
              if (uart_apid_state == 1 && laser_test_state == 0) {
                  if (uart_buf[0] == 'N' || uart_buf[0] == 'n') {
                      uart_apid_state = 0;
                      is_serial_interacting = 0;
                      printf("Advanced PID tuning cancelled\r\n");
                  } else if (uart_buf[0] == 'Y' || uart_buf[0] == 'y') {
                      if (has_valid_input) {
                          // Save advanced PID parameters
                          SD_Save_PID_Advanced();
                          printf("Advanced PID parameters saved\r\n");
                          uart_apid_state = 0;
                          is_serial_interacting = 0;
                      } else {
                          printf("No valid input detected. Press N to exit or modify parameters first\r\n");
                      }
                  } else if (sscanf(uart_buf, "%hhu", &current_apid_param) == 1 && current_apid_param >= 1 && current_apid_param <= 7) {
                      uart_apid_state = 2;
                      printf("Enter new value for parameter %d:\r\n", current_apid_param);
                  } else {
                      printf("Invalid input. Enter parameter number (1-7), Y to save exit, N to exit\r\n");
                  }
              } else if (uart_apid_state == 2 && laser_test_state == 0) {
                  if (uart_buf[0] == 'N' || uart_buf[0] == 'n') {
                      uart_apid_state = 1;
                      printf("Advanced PID Tuning Mode\r\n");
                      printf("1. Deadband Threshold (0.5)\r\n");
                      printf("2. Mode Switch Threshold (10.0)\r\n");
                      printf("3. Static Error Threshold (5.0)\r\n");
                      printf("4. Temp Change Threshold (5.0)\r\n");
                      printf("5. Output Max PD Mode (750.0)\r\n");
                      printf("6. Integral Limit (1000.0)\r\n");
                      printf("7. Output Max PID Mode (1000.0)\r\n");
                      printf("Enter parameter number to modify, Y to save exit, N to exit\r\n");
                  } else {
                      float new_value = 0.0f;
                      if (sscanf(uart_buf, "%f", &new_value) == 1) {
                          has_valid_input = 1;
                          // Update the corresponding parameter
                          switch (current_apid_param) {
                              case 1:
                                  adv_pid_TEMP.deadband_threshold = new_value;
                                  break;
                              case 2:
                                  adv_pid_TEMP.mode_switch_threshold = new_value;
                                  break;
                              case 3:
                                  adv_pid_TEMP.static_err_threshold = new_value;
                                  break;
                              case 4:
                                  adv_pid_TEMP.temp_change_threshold = new_value;
                                  break;
                              case 5:
                                  adv_pid_TEMP.output_max_pd_mode = new_value;
                                  break;
                              case 6:
                                  adv_pid_TEMP.integral_limit = new_value;
                                  break;
                              case 7:
                                  adv_pid_TEMP.output_max_pid_mode = new_value;
                                  break;
                          }
                          printf("Parameter %d updated to %.2f\r\n", current_apid_param, new_value);
                          uart_apid_state = 1;
                          printf("Advanced PID Tuning Mode\r\n");
                          printf("1. Deadband Threshold (%.2f)\r\n", adv_pid_TEMP.deadband_threshold);
                          printf("2. Mode Switch Threshold (%.2f)\r\n", adv_pid_TEMP.mode_switch_threshold);
                          printf("3. Static Error Threshold (%.2f)\r\n", adv_pid_TEMP.static_err_threshold);
                          printf("4. Temp Change Threshold (%.2f)\r\n", adv_pid_TEMP.temp_change_threshold);
                          printf("5. Output Max PD Mode (%.2f)\r\n", adv_pid_TEMP.output_max_pd_mode);
                          printf("6. Integral Limit (%.2f)\r\n", adv_pid_TEMP.integral_limit);
                          printf("7. Output Max PID Mode (%.2f)\r\n", adv_pid_TEMP.output_max_pid_mode);
                          printf("Enter parameter number to modify, Y to save exit, N to exit\r\n");
                      } else {
                          printf("Invalid value. Enter a number:\r\n");
                      }
                  }
              }

              // 模糊PID调试状态机
              if (uart_fpid_state == 1 && laser_test_state == 0) {
                  if (uart_buf[0] == 'N' || uart_buf[0] == 'n') {
                      uart_fpid_state = 0;
                      is_serial_interacting = 0;
                      printf("Fuzzy PID tuning cancelled\r\n");
                  } else if (uart_buf[0] == 'Y' || uart_buf[0] == 'y') {
                      if (has_valid_input) {
                          // Save fuzzy PID parameters
                          SD_Save_PID_Fuzzy();
                          printf("Fuzzy PID parameters saved\r\n");
                          uart_fpid_state = 0;
                          is_serial_interacting = 0;
                      } else {
                          printf("No valid input detected. Press N to exit or modify parameters first\r\n");
                      }
                  } else if (sscanf(uart_buf, "%hhu", &current_fpid_param) == 1 && current_fpid_param >= 1 && current_fpid_param <= 5) {
                      uart_fpid_state = 2;
                      printf("Enter new value for parameter %d:\r\n", current_fpid_param);
                  } else {
                      printf("Invalid input. Enter parameter number (1-5), Y to save exit, N to exit\r\n");
                  }
              } else if (uart_fpid_state == 2 && laser_test_state == 0) {
                  if (uart_buf[0] == 'N' || uart_buf[0] == 'n') {
                      uart_fpid_state = 1;
                      printf("Fuzzy PID Tuning Mode\r\n");
                      printf("1. E Max (50.0)\r\n");
                      printf("2. EC Max (5.0)\r\n");
                      printf("3. Kp Weight (5.0)\r\n");
                      printf("4. Ki Weight (0.1)\r\n");
                      printf("5. Kd Weight (10.0)\r\n");
                      printf("Enter parameter number to modify, Y to save exit, N to exit\r\n");
                  } else {
                      float new_value = 0.0f;
                      if (sscanf(uart_buf, "%f", &new_value) == 1) {
                          has_valid_input = 1;
                          // Update the corresponding parameter
                          switch (current_fpid_param) {
                              case 1:
                                  fuzzy_pid_TEMP.e_max = new_value;
                                  break;
                              case 2:
                                  fuzzy_pid_TEMP.ec_max = new_value;
                                  break;
                              case 3:
                                  fuzzy_pid_TEMP.Kp_weight = new_value;
                                  break;
                              case 4:
                                  fuzzy_pid_TEMP.Ki_weight = new_value;
                                  break;
                              case 5:
                                  fuzzy_pid_TEMP.Kd_weight = new_value;
                                  break;
                          }
                          printf("Parameter %d updated to %.2f\r\n", current_fpid_param, new_value);
                          uart_fpid_state = 1;
                          printf("Fuzzy PID Tuning Mode\r\n");
                          printf("1. E Max (%.1f)\r\n", fuzzy_pid_TEMP.e_max);
                          printf("2. EC Max (%.1f)\r\n", fuzzy_pid_TEMP.ec_max);
                          printf("3. Kp Weight (%.1f)\r\n", fuzzy_pid_TEMP.Kp_weight);
                          printf("4. Ki Weight (%.2f)\r\n", fuzzy_pid_TEMP.Ki_weight);
                          printf("5. Kd Weight (%.1f)\r\n", fuzzy_pid_TEMP.Kd_weight);
                          printf("Enter parameter number to modify, Y to save exit, N to exit\r\n");
                      } else {
                          printf("Invalid value. Enter a number:\r\n");
                      }
                  }
              }

              // PID模式切换状态机
              if (uart_pidmode_state == 1 && laser_test_state == 0) {
                  if (uart_buf[0] == 'N' || uart_buf[0] == 'n') {
                      uart_pidmode_state = 0;
                      is_serial_interacting = 0;
                      printf("Mode switch cancelled\r\n");
                  } else {
                      int new_mode = -1;
                      if (sscanf(uart_buf, "%d", &new_mode) == 1 && new_mode >= 0 && new_mode <= 2) {
                          pid_algorithm_type = new_mode;
                          // Save mode to SD card
                          SD_Save_PID_Mode();
                          printf("PID mode changed to %d\r\n", new_mode);
                          uart_pidmode_state = 0;
                          is_serial_interacting = 0;
                      } else {
                          printf("Invalid mode. Enter 0 (Basic), 1 (Fuzzy), 2 (Advanced):\r\n");
                      }
                  }
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
              active_key_4(&huart4, &htim8, alu_485_send, power_thres);
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
