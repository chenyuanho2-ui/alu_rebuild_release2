#include "pid_storage.h"
#include "pid.h"
#include "fuzzy_pid.h"
#include "advanced_pid.h"
#include "fatfs.h"
#include <stdio.h>

extern PID_struct pid_TEMP;
extern FuzzyPID_struct fuzzy_pid_TEMP;
extern AdvPID_struct adv_pid_TEMP;

void SD_Save_PID_Config(float kp, float ki, float kd) {
    extern uint8_t sd_pid_save_enable;
    if (sd_pid_save_enable == 0) return;

    FRESULT res;
    FIL file;
    UINT bw;

    res = f_open(&file, "0:/pid_cfg.txt", FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("SD: Failed to open pid_cfg.txt for write (res=%d)\r\n", res);
        return;
    }

    bw = f_printf(&file, "%.2f,%.2f,%.2f", kp, ki, kd);
    if ((INT)bw <= 0) {
        printf("SD: Failed to write PID config (bw=%u)\r\n", bw);
    }

    f_close(&file);
}

void SD_Load_PID_Config(void) {
    FRESULT res;
    FIL file;
    char buf[64];
    UINT br;
    float temp_kp, temp_ki, temp_kd;

    res = f_open(&file, "0:/pid_cfg.txt", FA_READ);
    if (res != FR_OK) {
        printf("SD: No pid_cfg.txt, using default PID (Kp=40, Ki=0.8, Kd=125)\r\n");
        pid_TEMP.Kp = fuzzy_pid_TEMP.Kp = adv_pid_TEMP.Kp = 40.0f;
        pid_TEMP.Ki = fuzzy_pid_TEMP.Ki = adv_pid_TEMP.Ki = 0.8f;
        pid_TEMP.Kd = fuzzy_pid_TEMP.Kd = adv_pid_TEMP.Kd = 125.0f;
        return;
    }

    res = f_read(&file, buf, sizeof(buf) - 1, &br);
    f_close(&file);

    if (res != FR_OK || br == 0) {
        printf("SD: Failed to read pid_cfg.txt, using default PID parameters\r\n");
        return;
    }

    buf[br] = '\0';

    if (sscanf(buf, "%f,%f,%f", &temp_kp, &temp_ki, &temp_kd) == 3) {
        if (temp_kp == 0 && temp_ki == 0 && temp_kd == 0) {
            printf("SD: PID all zeros, using default values\r\n");
        } else {
            pid_TEMP.Kp = fuzzy_pid_TEMP.Kp = adv_pid_TEMP.Kp = temp_kp;
            pid_TEMP.Ki = fuzzy_pid_TEMP.Ki = adv_pid_TEMP.Ki = temp_ki;
            pid_TEMP.Kd = fuzzy_pid_TEMP.Kd = adv_pid_TEMP.Kd = temp_kd;
            printf("SD: Loaded PID - Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", temp_kp, temp_ki, temp_kd);
        }
    } else {
        printf("SD: Parse failed, using default PID (Kp=40, Ki=0.8, Kd=125)\r\n");
        pid_TEMP.Kp = fuzzy_pid_TEMP.Kp = adv_pid_TEMP.Kp = 40.0f;
        pid_TEMP.Ki = fuzzy_pid_TEMP.Ki = adv_pid_TEMP.Ki = 0.8f;
        pid_TEMP.Kd = fuzzy_pid_TEMP.Kd = adv_pid_TEMP.Kd = 125.0f;
    }
}
