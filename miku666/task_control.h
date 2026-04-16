#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include "cmsis_os.h"
#include "main.h"
#include "pid.h"
#include "fuzzy_pid.h"
#include "advanced_pid.h"

extern FuzzyPID_struct fuzzy_pid_TEMP;
extern AdvPID_struct adv_pid_TEMP;

void StartTask_Control(void const * argument);

#endif
