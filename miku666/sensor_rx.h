#ifndef __SENSOR_RX_H__
#define __SENSOR_RX_H__

#include "main.h"

// 结构体总长度：1(头) + 4(时间戳) + 4(温度float) + 2(CRC) + 1(尾) = 12 字节
#pragma pack(push, 1)
typedef struct {
    uint8_t  header;      // 0x55
    uint32_t timestamp;   // 时间戳
    float    temperature; // 【修改点】温度数据，匹配发送端
    uint16_t crc16;       // CRC校验
    uint8_t  tail;        // 0xAA
} SensorData_t;
#pragma pack(pop)

// 声明新的函数
void StartRxTask(void *argument);
void USART1_IDLE_Callback(void);
void Sensor_Rx_Init(void);

#endif
