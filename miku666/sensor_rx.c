#include "sensor_rx.h"
#include "cmsis_os.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;

// 1. 解决 rxSemHandle 报错：直接在这里定义并分配信号量
osSemaphoreId_t rxSemHandle = NULL;
const osSemaphoreAttr_t rxSem_attributes = {
  .name = "rxSem"
};

// DMA 接收缓冲区 (H7 Cache 必须按 32 字节对齐)
#define RX_BUF_SIZE 64
uint8_t dma_rx_buf[RX_BUF_SIZE] __attribute__((aligned(32))); 

// 内部 CRC16 函数
static uint16_t Calculate_CRC16(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (uint16_t j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

void Parse_DMA_Buffer(uint8_t* buffer, uint16_t length) {
    // 如果收到的数据长度小于一帧的最短长度(12字节)，直接返回
    if (length < sizeof(SensorData_t)) {
        return; 
    }

    // 遍历缓冲区，寻找合法的帧
    for (uint16_t i = 0; i <= length - sizeof(SensorData_t); i++) {
        
        // 校验帧头和帧尾 (使用 sizeof(SensorData_t) - 1 动态定位帧尾)
        if (buffer[i] == 0x55 && buffer[i + sizeof(SensorData_t) - 1] == 0xAA) {
            
            SensorData_t *p = (SensorData_t *)&buffer[i];
            
            // 计算CRC (除去最后的 CRC16 和 tail，一共需要计算前 1+4+4 = 9 字节)
            uint16_t calculated_crc = Calculate_CRC16(&buffer[i], sizeof(SensorData_t) - 3);
            
            if (calculated_crc == p->crc16) {
                // 【修改点】直接打印浮点数温度
                printf("[RX] Time: %u ms | Temp: %.2f C\r\n", p->timestamp, p->temperature);
                
                // 解析成功后，索引跳过这一整帧数据(12字节)
                // 减 1 是因为 for 循环自身会执行 i++
                i += (sizeof(SensorData_t) - 1); 
            } else {
                printf("CRC mismatch: calculated %04X, received %04X\r\n", calculated_crc, p->crc16);
            }
        }
    }
}

// 初始化函数
void Sensor_Rx_Init(void) {
    // 创建二进制信号量
    if (rxSemHandle == NULL) {
        rxSemHandle = osSemaphoreNew(1, 0, &rxSem_attributes);
    }
    
    // 开启 DMA 接收
    HAL_UART_Receive_DMA(&huart1, dma_rx_buf, RX_BUF_SIZE);
    // 开启串口空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

// 3. 必须调用的中断回调函数 (见后文说明)
void USART1_IDLE_Callback(void) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1); // 清除空闲标志位
        
        // 释放信号量，唤醒任务去处理数据
        if (rxSemHandle != NULL) {
            osSemaphoreRelease(rxSemHandle);
        }
    }
}

// 4. 这是你的 FreeRTOS 接收任务主函数
void StartRxTask(void *argument) {
    Sensor_Rx_Init();

    for(;;) {
        // 阻塞式等待信号量，不占用任何 CPU
        if (osSemaphoreAcquire(rxSemHandle, osWaitForever) == osOK) {
            
            // 【重点】H7 必须：清除 Cache 保证读到的是 DMA 刚搬运进来的新数据
            SCB_InvalidateDCache_by_Addr((uint32_t*)dma_rx_buf, RX_BUF_SIZE);
            
            // 解析数据
            Parse_DMA_Buffer(dma_rx_buf, RX_BUF_SIZE);
            
            // 数据处理完后，重新启动下一轮 DMA 接收
            HAL_UART_Receive_DMA(&huart1, dma_rx_buf, RX_BUF_SIZE);
        }
    }
}
