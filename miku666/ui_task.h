#ifndef __ALU_UI_TASK_H
#define __ALU_UI_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief TouchGFX 任务入口函数
 * @note 该函数将覆盖 freertos.c 中的同名 __weak 函数
 * @param argument 任务参数
 */
void StartTouchGFX(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __ALU_UI_TASK_H */
