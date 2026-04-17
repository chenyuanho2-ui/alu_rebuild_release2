#ifndef __PID_STORAGE_H
#define __PID_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

void SD_Load_PID_Config(void);
void SD_Save_PID_Config(float kp, float ki, float kd);
void SD_Save_PID_Base(void);
void SD_Save_PID_Advanced(void);
void SD_Save_PID_Fuzzy(void);
void SD_Save_PID_Mode(void);

#ifdef __cplusplus
}
#endif

#endif
