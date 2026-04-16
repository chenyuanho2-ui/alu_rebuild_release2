#ifndef __PID_STORAGE_H
#define __PID_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

void SD_Load_PID_Config(void);
void SD_Save_PID_Config(float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif
