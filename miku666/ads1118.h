#ifndef __ADS1118_H
#define __ADS1118_H

#include "main.h"

/* --- 寄存器配置宏定义 --- */
#define ADS1118_SS_ONCE        (1 << 15) // 单次转换开始
#define ADS1118_MUX_01         (0 << 12) // AINP=AIN0, AINN=AIN1 (热电偶差分输入)
#define ADS1118_MUX_0G         (4 << 12) // AINP=AIN0, AINN=GND
#define ADS1118_MUX_1G         (5 << 12) // AINP=AIN1, AINN=GND

#define ADS1118_PGA_256mV      (5 << 9)  // FSR = ±0.256V (适合热电偶微弱信号)
#define ADS1118_PGA_2048mV     (2 << 9)  // FSR = ±2.048V (默认)

#define ADS1118_MODE_CONTINUE  (0 << 8)  // 连续转换模式
#define ADS1118_MODE_SINGLE    (1 << 8)  // 单次转换模式 (推荐)

#define ADS1118_DR_475SPS      (6 << 5)  // 采样率 475 SPS

#define ADS1118_TS_ADC         (0 << 4)  // ADC 模式 (测外部电压)
#define ADS1118_TS_TEMP        (1 << 4)  // 内部温度传感器模式 (冷端补偿)

#define ADS1118_PULLUP_EN      (1 << 3)  // DOUT上拉使能
#define ADS1118_NOP_VALID      (1 << 1)  // 数据有效，更新配置

/* 常用配置组合 */
#define CONFIG_TEMP_SENSOR  (ADS1118_SS_ONCE | ADS1118_MUX_01 | ADS1118_PGA_256mV | \
                             ADS1118_MODE_SINGLE | ADS1118_DR_475SPS | ADS1118_TS_TEMP | \
                             ADS1118_PULLUP_EN | ADS1118_NOP_VALID)

#define CONFIG_TC_ADC       (ADS1118_SS_ONCE | ADS1118_MUX_01 | ADS1118_PGA_256mV | \
                             ADS1118_MODE_SINGLE | ADS1118_DR_475SPS | ADS1118_TS_ADC | \
                             ADS1118_PULLUP_EN | ADS1118_NOP_VALID)

/* 函数声明 */
void ADS1118_Init(void);
uint16_t ADS1118_WriteRead(uint16_t config_cmd);
float ADS1118_GetInternalTemp(void);
float ADS1118_GetVoltage_mV(void);
float ADS1118_CalibrateTemp(float raw_temp);

#endif /* __ADS1118_H */
