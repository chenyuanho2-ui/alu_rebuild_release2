#include "pid_storage.h"
#include "pid.h"
#include "fatfs.h"
#include <stdio.h>

void SD_Save_PID_Config(float kp, float ki, float kd) {
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
        printf("SD: No pid_cfg.txt found, using default PID parameters\r\n");
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
        pid_TEMP.Kp = temp_kp;
        pid_TEMP.Ki = temp_ki;
        pid_TEMP.Kd = temp_kd;
        printf("SD: Loaded PID from SD card - Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", temp_kp, temp_ki, temp_kd);
    } else {
        printf("SD: Failed to parse pid_cfg.txt, using default PID parameters\r\n");
    }
}
