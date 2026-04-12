#include "ads1118.h"
#include "spi.h"
#include "cmsis_os.h" // 必须引入以支持 osDelay
#include <stdio.h>

/* 使用 H750 工程中定义的 SPI2 片选宏 */
#define ADS1118_CS_LOW()   HAL_GPIO_WritePin(SPI2_CS_T_GPIO_Port, SPI2_CS_T_Pin, GPIO_PIN_RESET)
#define ADS1118_CS_HIGH()  HAL_GPIO_WritePin(SPI2_CS_T_GPIO_Port, SPI2_CS_T_Pin, GPIO_PIN_SET)

/* 11点校准表定义 */
#define CAL_POINTS 11
static const float cal_measured[CAL_POINTS] = {0.0f, 17.8f, 21.5f, 29.0f, 43.0f, 49.0f, 56.0f, 63.0f, 70.0f, 79.0f, 100.0f};
static const float cal_actual[CAL_POINTS]   = {0.0f, 16.4f, 20.7f, 31.0f, 43.5f, 50.8f, 60.5f, 65.0f, 71.2f, 81.5f, 100.0f};

/**
 * @brief 芯片初始化
 */
void ADS1118_Init(void) {
    ADS1118_CS_HIGH(); 
    osDelay(10); // 使用 RTOS 延时替代 HAL_Delay
}

/**
 * @brief 底层 SPI 读写函数 (适配 hspi2)
 */
uint16_t ADS1118_WriteRead(uint16_t config_cmd) {
    uint8_t tx_data[2];
    uint8_t rx_data[2];
    uint16_t result = 0;

    // 大端转换
    tx_data[0] = (uint8_t)(config_cmd >> 8);
    tx_data[1] = (uint8_t)(config_cmd & 0xFF);

    ADS1118_CS_LOW();
    
    // 使用当前工程的 hspi2
    HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, 2, 1000);
    
    ADS1118_CS_HIGH();

    result = (rx_data[0] << 8) | rx_data[1];
    return result;
}

/**
 * @brief 获取芯片内部温度 (用于冷端补偿)
 */
float ADS1118_GetInternalTemp(void) {
    ADS1118_WriteRead(CONFIG_TEMP_SENSOR);
    
    osDelay(2);
    
    uint16_t raw_data = ADS1118_WriteRead(0x0000); 
    int16_t temp_reg = (int16_t)raw_data >> 2;
    return (float)temp_reg * 0.03125f;
}

/**
 * @brief 获取热电偶两端的差分电压 (单位：mV)
 */
float ADS1118_GetVoltage_mV(void) {
    ADS1118_WriteRead(CONFIG_TC_ADC);

    osDelay(2); // ADC 转换等待 2ms (10ms 周期内留出余量)

    int16_t raw_data = (int16_t)ADS1118_WriteRead(0x0000);
    return (float)raw_data * 0.0078125f;
}

/**
 * @brief 11点分段线性插值校准
 */
float ADS1118_CalibrateTemp(float raw_temp) {
    if (raw_temp <= cal_measured[0]) return cal_actual[0];
    if (raw_temp >= cal_measured[CAL_POINTS-1]) return cal_actual[CAL_POINTS-1];
    
    for (int i = 0; i < CAL_POINTS - 1; i++) {
        if (raw_temp >= cal_measured[i] && raw_temp <= cal_measured[i+1]) {
            return cal_actual[i] + (cal_actual[i+1] - cal_actual[i]) / 
                   (cal_measured[i+1] - cal_measured[i]) * (raw_temp - cal_measured[i]);
        }
    }
    return raw_temp;
}
