/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  *	@author          : Lv YiPin 
	* @email           : AluminiumOxide@163.com
	* @SMA             : https://space.bilibili.com/87077691
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "dma2d.h"
#include "fatfs.h"
#include "ltdc.h"
#include "mdma.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"
#include "app_touchgfx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "touch_800x480.h"

#include "alu_file.h"
#include "alu_temp.h"
//#include "alu_485.h"

#include "alu_control.h"
//#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Initialize(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern osSemaphoreId alu_chooseHandle;         // 选择是温度还是功率
extern osSemaphoreId alu_temperatureHandle;    // 温度设置信号量
extern osSemaphoreId alu_thresholdHandle;      // 阈值设置信号量
extern osSemaphoreId alu_screenHandle;         // 屏幕切换信号量
extern osSemaphoreId alu_savenameHandle;       // 保存文件名信号量
extern SPI_HandleTypeDef hspi2;
extern DAC_HandleTypeDef hdac1;


double K_Temperature   = 0;        // 温度值
float  pwm_percent     = 0;        // pwn比例
int    temp_modify     = 0;        // 温度偏移
float  temp_thres      = 30;	   // 温度阈值

float  power_thres     = 0;        // 功率阈值

int    index_screen    = 2;        // 屏幕切换 0为SrcMain,1为Screen1, 2为launch
int    index_choose    = 1;        // 0为控制温度阈值,1为调整功率阈值

AluDynList sd_file_list;           // 动态文件列表
int    num_file        = 0;        // 当前待创建文件索引


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	SCB->VTOR = 0X90000000;       /* Vector Table Relocation in off-chip */
	
	// Icache开启会一定程度上加速页面刷新,Dcache不能一起开启,否则会导致FatFs加载失败
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_MDMA_Init();
  MX_LTDC_Init();
  MX_CRC_Init();
  MX_SPI2_Init();
  MX_DAC1_Init();
  MX_FMC_Init();
  MX_DMA2D_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TouchGFX_Init();
  /* Call PreOsInit function */
  MX_TouchGFX_PreOSInit();
  /* USER CODE BEGIN 2 */
//	TIM1->CR2&=0X0000;
//	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)Sine12bit, 32, DAC_ALIGN_12B_R);


	Touch_Init();				// 触摸屏初始化	
	Alu_SD_mount();             // 判断FatFs是否挂载成功
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 添加个人的函数
void StartTouchGFX(void *argument)
{
	MX_TouchGFX_Process();	// 交给TouchGFX处理
  for(;;){
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


void AluMain(void *argument)
{
	
  /* Infinite loop */
	// 控制激光器,首先停止激光输出
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_SET);
	uint8_t alu_485_send[] = {0x55, 0x33, 0x01, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x0D};
	HAL_UART_Transmit(&huart4,(uint8_t*)alu_485_send,10,0xFFFF);
	HAL_GPIO_WritePin(flag_485_GPIO_Port,flag_485_Pin,GPIO_PIN_RESET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1); // 0~999
	
	
	Alu_list_init(&sd_file_list);  // 初始化文件列表
	Alu_sniff_files(&sd_file_list,"/");// 遍历根目录
	for (int i = 0; i < sd_file_list.size; i++) {
        printf("File %d: %s\n", i+1, sd_file_list.items[i]);
    }
/*
	Alu_list_add(&sd_file_list, "example.txt");
	Alu_list_add(&sd_file_list, "sd.txt");
	Alu_list_add(&sd_file_list, "alu.txt");
	Alu_list_add(&sd_file_list, "425.txt");
	Alu_list_add(&sd_file_list, "430.txt");
	
	for (int i = 0; i < sd_file_list.size; i++) {
        printf("File %d: %s\n", i+1, sd_file_list.items[i]);
    }
	Alu_list_del(&sd_file_list,0); // 删除第1个
	Alu_list_del(&sd_file_list,(int)(sd_file_list.size-1));  // 删除最后1个
	Alu_list_del(&sd_file_list,(int)(sd_file_list.size-1));
	printf("\n");
	for (int i = 0; i < sd_file_list.size; i++) {
        printf("File %d: %s\n", i+1, sd_file_list.items[i]);
    }
*/

	int num_file = Alu_SD_csv_num("/") + 1; // 遍历根目录下的索引
	printf("num_file %d\n",num_file);
	
	vTaskDelay(pdMS_TO_TICKS(1000));
	index_screen = 0;
	osSemaphoreRelease(alu_screenHandle);  // 发送屏幕切换信号量更新,切换到SrcMain,并更新文件列表
	

	
	long btns_statu = 0;  // 输入状态  
  for(;;)
  {
	  
		vTaskDelay(pdMS_TO_TICKS(50));
	  
		btns_statu = btn_sniff_pressed();
	  
		if (btns_statu>0){
			printf("btn pressed %ld\n", btns_statu);
		}
		
		switch (btns_statu) {
		case 1: // 按键1
			index_choose = active_key_1(index_choose); // 选择设置功率还是温度
			break;
		case 2: // 按键2 提高阈值	
			active_key_2(index_choose, &temp_thres, &power_thres);   // 提高阈值
			break;
		case 4: // 按键3 降低阈值
			active_key_3(index_choose, &temp_thres, &power_thres);   // 降低阈值
			break;
		case 8: // 按键4 输出功率调节
			active_key_4(&huart4, &htim1, alu_485_send, power_thres);
			break;
		case (1+8): // 按键1+4 调温度偏移（以板载LED做指示）
			active_key_1vs4(&temp_modify);
			break;
		case 16: //脚踏
			active_key_foot(alu_485_send,temp_thres, power_thres);
			break;	
		}

//		taskENTER_CRITICAL();		// 进临界区
//		taskEXIT_CRITICAL();	// 出临界区

  }
}

/*
void AluSubProgress(void *argument)
{
	// 设置DAC输出，直接输出电压,按照FM147调节,还需要跳帽,
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1);
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
  for(;;)
  {
	vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
*/




/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0xC0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */




