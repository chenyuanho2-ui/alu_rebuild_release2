#include "pid_storage.h"
#include "pid.h"
#include "fuzzy_pid.h"
#include "advanced_pid.h"
#include "fatfs.h"
#include <stdio.h>

extern PID_struct pid_TEMP;
extern FuzzyPID_struct fuzzy_pid_TEMP;
extern AdvPID_struct adv_pid_TEMP;
extern uint8_t pid_algorithm_type;

void SD_Save_PID_Base(void) {
    extern uint8_t sd_pid_save_enable;
    if (sd_pid_save_enable == 0) return;

    FRESULT res;
    FIL file;
    UINT bw;
    char buf[32];

    sprintf(buf, "%.2f,%.2f,%.2f", pid_base.Kp, pid_base.Ki, pid_base.Kd);

    res = f_open(&file, "0:/pid_base.txt", FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("SD: Failed to open pid_base.txt for write (res=%d)\r\n", res);
        return;
    }

    bw = f_printf(&file, "%s", buf);
    if ((INT)bw <= 0) {
        printf("SD: Failed to write PID base config (bw=%u)\r\n", bw);
    } else {
        printf("SD: PID base saved: %s\r\n", buf);
    }

    f_close(&file);
}

void SD_Save_PID_Advanced(void) {
    extern uint8_t sd_pid_save_enable;
    if (sd_pid_save_enable == 0) return;

    FRESULT res;
    FIL file;
    UINT bw;
    char buf[128];

    // Save advanced PID parameters
    sprintf(buf, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", 
            adv_pid_TEMP.deadband_threshold, 
            adv_pid_TEMP.mode_switch_threshold, 
            adv_pid_TEMP.static_err_threshold, 
            adv_pid_TEMP.temp_change_threshold, 
            adv_pid_TEMP.output_max_pd_mode, 
            adv_pid_TEMP.integral_limit, 
            adv_pid_TEMP.output_max_pid_mode);

    res = f_open(&file, "0:/pid_adv.txt", FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("SD: Failed to open pid_adv.txt for write (res=%d)\r\n", res);
        return;
    }

    bw = f_printf(&file, "%s", buf);
    if ((INT)bw <= 0) {
        printf("SD: Failed to write advanced PID config (bw=%u)\r\n", bw);
    } else {
        printf("SD: Advanced PID saved\r\n");
    }

    f_close(&file);
}

void SD_Save_PID_Fuzzy(void) {
    extern uint8_t sd_pid_save_enable;
    if (sd_pid_save_enable == 0) return;

    FRESULT res;
    FIL file;
    UINT bw;
    char buf[128];

    // Save fuzzy PID parameters
    sprintf(buf, "%.2f,%.2f,%.2f,%.2f,%.2f", 
            fuzzy_pid_TEMP.e_max, fuzzy_pid_TEMP.ec_max, 
            fuzzy_pid_TEMP.Kp_weight, fuzzy_pid_TEMP.Ki_weight, fuzzy_pid_TEMP.Kd_weight);

    res = f_open(&file, "0:/pid_fuzzy.txt", FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("SD: Failed to open pid_fuzzy.txt for write (res=%d)\r\n", res);
        return;
    }

    bw = f_printf(&file, "%s", buf);
    if ((INT)bw <= 0) {
        printf("SD: Failed to write fuzzy PID config (bw=%u)\r\n", bw);
    } else {
        printf("SD: Fuzzy PID saved: %s\r\n", buf);
    }

    f_close(&file);
}

void SD_Save_PID_Mode(void) {
    extern uint8_t sd_pid_save_enable;
    if (sd_pid_save_enable == 0) return;

    FRESULT res;
    FIL file;
    UINT bw;
    char buf[16];

    sprintf(buf, "%d", pid_algorithm_type);

    res = f_open(&file, "0:/pid_mode.txt", FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("SD: Failed to open pid_mode.txt for write (res=%d)\r\n", res);
        return;
    }

    bw = f_printf(&file, "%s", buf);
    if ((INT)bw <= 0) {
        printf("SD: Failed to write PID mode (bw=%u)\r\n", bw);
    } else {
        printf("SD: PID mode saved: %s\r\n", buf);
    }

    f_close(&file);
}

void SD_Save_PID_Config(float kp, float ki, float kd) {
    // Update global base PID parameters
    pid_base.Kp = kp;
    pid_base.Ki = ki;
    pid_base.Kd = kd;
    
    // Save to SD card
    SD_Save_PID_Base();
}

void SD_Load_PID_Base(void) {
    FRESULT res;
    FIL file;
    char buf[64];
    UINT br;
    float temp_kp, temp_ki, temp_kd;

    res = f_open(&file, "0:/pid_base.txt", FA_READ);
    if (res != FR_OK) {
        printf("SD: No pid_base.txt found (res=%d), using default PID (Kp=40, Ki=0.8, Kd=125)\r\n", res);
        pid_base.Kp = 40.0f;
        pid_base.Ki = 0.8f;
        pid_base.Kd = 125.0f;
        // Create default file
        SD_Save_PID_Base();
        return;
    }

    res = f_read(&file, buf, sizeof(buf) - 1, &br);
    f_close(&file);

    if (res != FR_OK || br == 0) {
        printf("SD: Failed to read pid_base.txt, using default PID parameters\r\n");
        return;
    }

    buf[br] = '\0';
    printf("SD: Read pid_base.txt content: '%s'\r\n", buf);

    if (sscanf(buf, "%f,%f,%f", &temp_kp, &temp_ki, &temp_kd) == 3) {
        if (temp_kp == 0 && temp_ki == 0 && temp_kp == 0) {
            printf("SD: PID all zeros, using default values\r\n");
        } else {
            pid_base.Kp = temp_kp;
            pid_base.Ki = temp_ki;
            pid_base.Kd = temp_kd;
            printf("SD: Loaded PID base - Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", temp_kp, temp_ki, temp_kd);
        }
    } else {
        printf("SD: Parse failed, using default PID (Kp=40, Ki=0.8, Kd=125)\r\n");
        pid_base.Kp = 40.0f;
        pid_base.Ki = 0.8f;
        pid_base.Kd = 125.0f;
    }
}

void SD_Load_PID_Fuzzy(void) {
    FRESULT res;
    FIL file;
    char buf[64];
    UINT br;
    float e_max, ec_max, kp_weight, ki_weight, kd_weight;

    res = f_open(&file, "0:/pid_fuzzy.txt", FA_READ);
    if (res != FR_OK) {
        printf("SD: No pid_fuzzy.txt found (res=%d), using default fuzzy PID parameters\r\n", res);
        fuzzy_pid_TEMP.e_max = 50.0f;
        fuzzy_pid_TEMP.ec_max = 5.0f;
        fuzzy_pid_TEMP.Kp_weight = 5.0f;
        fuzzy_pid_TEMP.Ki_weight = 0.1f;
        fuzzy_pid_TEMP.Kd_weight = 10.0f;
        // Create default file
        SD_Save_PID_Fuzzy();
        return;
    }

    res = f_read(&file, buf, sizeof(buf) - 1, &br);
    f_close(&file);

    if (res != FR_OK || br == 0) {
        printf("SD: Failed to read pid_fuzzy.txt, using default fuzzy PID parameters\r\n");
        return;
    }

    buf[br] = '\0';
    printf("SD: Read pid_fuzzy.txt content: '%s'\r\n", buf);

    if (sscanf(buf, "%f,%f,%f,%f,%f", &e_max, &ec_max, &kp_weight, &ki_weight, &kd_weight) == 5) {
        fuzzy_pid_TEMP.e_max = e_max;
        fuzzy_pid_TEMP.ec_max = ec_max;
        fuzzy_pid_TEMP.Kp_weight = kp_weight;
        fuzzy_pid_TEMP.Ki_weight = ki_weight;
        fuzzy_pid_TEMP.Kd_weight = kd_weight;
        printf("SD: Loaded fuzzy PID parameters\r\n");
    } else {
        printf("SD: Parse failed, using default fuzzy PID parameters\r\n");
        fuzzy_pid_TEMP.e_max = 50.0f;
        fuzzy_pid_TEMP.ec_max = 5.0f;
        fuzzy_pid_TEMP.Kp_weight = 5.0f;
        fuzzy_pid_TEMP.Ki_weight = 0.1f;
        fuzzy_pid_TEMP.Kd_weight = 10.0f;
    }
}

void SD_Load_PID_Mode(void) {
    FRESULT res;
    FIL file;
    char buf[16];
    UINT br;
    int mode;

    res = f_open(&file, "0:/pid_mode.txt", FA_READ);
    if (res != FR_OK) {
        printf("SD: No pid_mode.txt found (res=%d), using default mode (2 - Advanced)\r\n", res);
        pid_algorithm_type = 2;
        // Create default file
        SD_Save_PID_Mode();
        return;
    }

    res = f_read(&file, buf, sizeof(buf) - 1, &br);
    f_close(&file);

    if (res != FR_OK || br == 0) {
        printf("SD: Failed to read pid_mode.txt, using default mode (2 - Advanced)\r\n");
        return;
    }

    buf[br] = '\0';
    printf("SD: Read pid_mode.txt content: '%s'\r\n", buf);

    if (sscanf(buf, "%d", &mode) == 1 && mode >= 0 && mode <= 2) {
        pid_algorithm_type = mode;
        printf("SD: Loaded PID mode: %d\r\n", mode);
    } else {
        printf("SD: Parse failed, using default mode (2 - Advanced)\r\n");
        pid_algorithm_type = 2;
    }
}

void SD_Load_PID_Advanced(void) {
    FRESULT res;
    FIL file;
    char buf[64];
    UINT br;
    float deadband_threshold, mode_switch_threshold, static_err_threshold;
    float temp_change_threshold, output_max_pd_mode, integral_limit, output_max_pid_mode;

    res = f_open(&file, "0:/pid_adv.txt", FA_READ);
    if (res != FR_OK) {
        printf("SD: No pid_adv.txt found (res=%d), using default advanced PID parameters\r\n", res);
        adv_pid_TEMP.deadband_threshold = 0.5f;
        adv_pid_TEMP.mode_switch_threshold = 10.0f;
        adv_pid_TEMP.static_err_threshold = 5.0f;
        adv_pid_TEMP.temp_change_threshold = 5.0f;
        adv_pid_TEMP.output_max_pd_mode = 750.0f;
        adv_pid_TEMP.integral_limit = 1000.0f;
        adv_pid_TEMP.output_max_pid_mode = 1000.0f;
        // Create default file
        SD_Save_PID_Advanced();
        return;
    }

    res = f_read(&file, buf, sizeof(buf) - 1, &br);
    f_close(&file);

    if (res != FR_OK || br == 0) {
        printf("SD: Failed to read pid_adv.txt, using default advanced PID parameters\r\n");
        return;
    }

    buf[br] = '\0';
    printf("SD: Read pid_adv.txt content: '%s'\r\n", buf);

    if (sscanf(buf, "%f,%f,%f,%f,%f,%f,%f", 
               &deadband_threshold, &mode_switch_threshold, &static_err_threshold, 
               &temp_change_threshold, &output_max_pd_mode, &integral_limit, &output_max_pid_mode) == 7) {
        adv_pid_TEMP.deadband_threshold = deadband_threshold;
        adv_pid_TEMP.mode_switch_threshold = mode_switch_threshold;
        adv_pid_TEMP.static_err_threshold = static_err_threshold;
        adv_pid_TEMP.temp_change_threshold = temp_change_threshold;
        adv_pid_TEMP.output_max_pd_mode = output_max_pd_mode;
        adv_pid_TEMP.integral_limit = integral_limit;
        adv_pid_TEMP.output_max_pid_mode = output_max_pid_mode;
        printf("SD: Loaded advanced PID parameters\r\n");
    } else {
        printf("SD: Parse failed, using default advanced PID parameters\r\n");
        adv_pid_TEMP.deadband_threshold = 0.5f;
        adv_pid_TEMP.mode_switch_threshold = 10.0f;
        adv_pid_TEMP.static_err_threshold = 5.0f;
        adv_pid_TEMP.temp_change_threshold = 5.0f;
        adv_pid_TEMP.output_max_pd_mode = 750.0f;
        adv_pid_TEMP.integral_limit = 1000.0f;
        adv_pid_TEMP.output_max_pid_mode = 1000.0f;
    }
}

void SD_Load_PID_Config(void) {
    // Load all PID configurations
    SD_Load_PID_Base();
    SD_Load_PID_Advanced();
    SD_Load_PID_Fuzzy();
    SD_Load_PID_Mode();
}
