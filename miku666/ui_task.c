#include "ui_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_touchgfx.h"

/**
 * @brief TouchGFX 界面线程强定义
 * @details 
 * 1. 本函数通过链接器特性直接覆盖 freertos.c 中的 __weak 定义。
 * 2. 核心逻辑为调用 MX_TouchGFX_Process() 进入 TouchGFX 渲染引擎主循环。
 * 3. 随后的 for 循环作为安全兜底，防止任务意外退出导致内核崩溃。
 * @param argument: 由 osThreadCreate 传入的参数
 */
void StartTouchGFX(void *argument)
{
    /* 进入 TouchGFX 引擎主循环（正常情况下此函数不返回） */
    MX_TouchGFX_Process();

    /* --- 安全兜底区域 --- */
    /* 如果 TouchGFX 引擎意外退出，任务进入低功耗挂起状态 */
    for(;;)
    {
        /* 1000ms 的长延时，确保 UI 挂掉后不消耗 CPU 资源，保证激光控制等核心任务继续运行 */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
