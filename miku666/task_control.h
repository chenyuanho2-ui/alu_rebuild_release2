#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include "cmsis_os.h"
#include "main.h"

// 声明咱们的强控制任务入口函数
void StartTask_Control(void const * argument);

#endif /* __TASK_CONTROL_H */
